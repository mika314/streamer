#include "streamer.hpp"
#include "rgb2yuv.hpp"
#include <array>
#include <log/log.hpp>

static const int ChN = 2;
static const int SampleRate = 48'000;
static const int AudioBufSz = 1024;

Streamer::Streamer()
{
  avformat_network_init();
}

Streamer::~Streamer()
{
  stopStreaming();
}

auto Streamer::initVideoStream() -> bool
{
  auto codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    LOG("H.264 encoder not found");
    return false;
  }
  videoSt = avformat_new_stream(fmtCtx, codec);
  if (!videoSt)
  {
    LOG("Failed to create video stream");
    return false;
  }
  videoEncCtx = avcodec_alloc_context3(codec);
  if (!videoEncCtx)
  {
    LOG("Could not allocate video coedec context");
    return false;
  }
  videoEncCtx->bit_rate = 0;
  videoEncCtx->width = width;
  videoEncCtx->height = height;
  videoEncCtx->time_base = {1, 60};
  videoEncCtx->framerate = {60, 1};
  videoEncCtx->gop_size = 120;
  videoEncCtx->max_b_frames = 0;
  videoEncCtx->pix_fmt = AV_PIX_FMT_YUV420P;

  videoEncCtx->flags |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_GLOBAL_HEADER;
  videoEncCtx->thread_count = 0;

  av_opt_set(videoEncCtx->priv_data, "preset", "ultrafast", 0);
  av_opt_set(videoEncCtx->priv_data, "profile", "baseline", 0);
  av_opt_set(videoEncCtx->priv_data, "tune", "zerolatency", 0);
  av_opt_set(videoEncCtx->priv_data, "crf", "27", 0);
  if (avcodec_open2(videoEncCtx, codec, nullptr) < 0)
  {
    LOG("Failed to create video encoder");
    return false;
  }
  avcodec_parameters_from_context(videoSt->codecpar, videoEncCtx);
  return true;
}

auto Streamer::initAudioStream() -> bool
{
  auto codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
  if (!codec)
  {
    LOG("AAC encoder not found");
    return false;
  }
  audioSt = avformat_new_stream(fmtCtx, codec);
  audioEncCtx = avcodec_alloc_context3(codec);
  audioEncCtx->sample_rate = SampleRate;
  audioEncCtx->channel_layout = ChN == 2 ? AV_CH_LAYOUT_STEREO : AV_CH_LAYOUT_MONO;
  audioEncCtx->channels = ChN;
  audioEncCtx->sample_fmt = codec->sample_fmts[0]; // pick first supported
  audioEncCtx->bit_rate = 128000;
  audioEncCtx->time_base = {1, audioEncCtx->sample_rate};
  audioEncCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  if (avcodec_open2(audioEncCtx, codec, nullptr) < 0)
  {
    LOG("Failed to open audio encoder");
    return false;
  }
  avcodec_parameters_from_context(audioSt->codecpar, audioEncCtx);

  // After audioEncCtx is open:
  swrCtx = swr_alloc_set_opts(nullptr,
                              audioEncCtx->channel_layout,
                              audioEncCtx->sample_fmt,
                              audioEncCtx->sample_rate,
                              audioEncCtx->channel_layout,
                              AV_SAMPLE_FMT_S16,
                              audioEncCtx->sample_rate,
                              0,
                              nullptr);

  if (!swrCtx || swr_init(swrCtx) < 0)
  {
    LOG("Failed to initialize SwrContext");
    avcodec_free_context(&audioEncCtx);
    return false;
  }

  return true;
}

int Streamer::encodeAndWrite(AVCodecContext *encCtx, AVFrame *frame, AVStream *st)
{
  auto lock = std::unique_lock{mutex};

  if (const auto ret = avcodec_send_frame(encCtx, frame);
      ret < 0 && ret != AVERROR_EOF && ret != AVERROR(EAGAIN))
    return ret;

  auto pkt = av_packet_alloc();
  if (!pkt)
  {
    LOG("Error allocation packet");
    return AVERROR(ENOMEM);
  }

  auto ret = 0;
  while (true)
  {
    {
      ret = avcodec_receive_packet(encCtx, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      {
        ret = 0;
        break;
      }
      else if (ret < 0)
        break;
    }
    pkt->stream_index = st->index;
    // LOG("1 stream_index:",
    //     pkt->stream_index,
    //     "write frame pts:",
    //     pkt->pts,
    //     "enc time_base",
    //     encCtx->time_base.num,
    //     encCtx->time_base.den,
    //     "stream time_base",
    //     st->time_base.num,
    //     st->time_base.den);
    av_packet_rescale_ts(pkt, encCtx->time_base, st->time_base);
    // LOG("2 stream_index:", pkt->stream_index, "write frame pts:", pkt->pts);
    {
      ret = av_interleaved_write_frame(fmtCtx, pkt);
      if (ret < 0)
      {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        LOG("Error writing frame:", errbuf);
        break;
      }
    }
    av_packet_unref(pkt);
  }
  av_packet_free(&pkt);
  return ret;
}

int Streamer::sendVideoFrame(AVFrame *frame)
{
  return encodeAndWrite(videoEncCtx, frame, videoSt);
}

// For audio, you need to convert your captured PCM (interleaved int16_t) to AVFrame samples.
// This code assumes your PCM is int16_t stereo, and we need to convert it to the encoder’s format
// (likely FLTP). In a real scenario, use SwrContext to resample from S16 to FLTP if needed.
int Streamer::sendAudioFrame(const uint8_t *pcmData, int nbSamples)
{
  if (!swrCtx)
  {
    LOG("No SwrContext initialized");
    return -1;
  }

  // Allocate a frame for the converted samples
  AVFrame *af = av_frame_alloc();
  af->nb_samples = nbSamples;
  af->channel_layout = audioEncCtx->channel_layout;
  af->format = audioEncCtx->sample_fmt;
  af->sample_rate = audioEncCtx->sample_rate;

  if (av_frame_get_buffer(af, 0) < 0)
  {
    LOG("Could not allocate audio frame samples");
    av_frame_free(&af);
    return -1;
  }

  if (av_frame_make_writable(af) < 0)
  {
    LOG("Could not make audio frame writable");
    av_frame_free(&af);
    return -1;
  }

  // Input from pa_simple_read is interleaved S16
  const uint8_t *inData[1] = {pcmData};

  // Convert S16 interleaved to FLTP planar
  int convertedSamples = swr_convert(swrCtx,
                                     af->data, // output buffers
                                     nbSamples,
                                     inData, // input buffers
                                     nbSamples);

  if (convertedSamples < 0)
  {
    LOG("Error converting audio samples");
    av_frame_free(&af);
    return -1;
  }

  // Set pts (in samples)
  af->pts = audioPts;
  audioPts += convertedSamples; // increment pts by the number of samples

  // Now encode and send
  int ret = encodeAndWrite(audioEncCtx, af, audioSt);
  av_frame_free(&af);
  return ret;
}

auto Streamer::startStreaming(std::string url, std::string key) -> void
{
  if (streamingRunning.load())
    return;

  std::string fullUrl = url + "/" + key;

  // Initialize capture resources (GLX context, display, streamer, audio, etc.)
  // This is separate from the main thread’s SDL context.
  if (!initVideoCapture())
  {
    LOG("Failed to init video capture");
    return;
  }

  if (!initAudioCapture())
  {
    LOG("Failed to init audio capture");
    return;
  }

#ifndef TEST
  if (const auto ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", fullUrl.c_str()); ret < 0)
#else
  if (const auto ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", "test.flv"); ret < 0)
#endif
  {
    LOG("Could not create output context");
    cleanupCapture();
    av_frame_free(&videoFrame);
    return;
  }
  fmtCtx->oformat->flags |= AVFMT_GLOBALHEADER;

  if (!initVideoStream())
  {
    LOG("Failed to start streamer");
    cleanupCapture();
    av_frame_free(&videoFrame);
    return;
  }
  if (!initAudioStream())
  {
    LOG("Failed to start streamer");
    cleanupCapture();
    av_frame_free(&videoFrame);
    return;
  }

  if ((fmtCtx->oformat->flags & AVFMT_NOFILE) == 0) // no no file == file
  {
#ifndef TEST
    if (const auto ret = avio_open2(&fmtCtx->pb, fullUrl.c_str(), AVIO_FLAG_WRITE, nullptr, nullptr);
        ret < 0)
#else
    if (const auto ret = avio_open2(&fmtCtx->pb, "test.flv", AVIO_FLAG_WRITE, nullptr, nullptr); ret < 0)
#endif
    {
      LOG("Could not open output URL");
      cleanupCapture();
      av_frame_free(&videoFrame);
      return;
    }
  }

#ifndef TEST
  av_dump_format(fmtCtx, 0, "rtmp://[censored]", 1);
#else
  av_dump_format(fmtCtx, 0, "test.flv", 1);
#endif

  if (const auto ret = avformat_write_header(fmtCtx, nullptr); ret < 0)
  {
    LOG("Failed to start streamer");
    cleanupCapture();
    av_frame_free(&videoFrame);
    return;
  }

  streamingRunning.store(true);
  streamingShouldStop.store(false);

  // Launch the streaming thread
  streamingVideoThread = std::thread{[this]() { streamingVideoWorker(); }};
  streamingAudioThread = std::thread{[this]() { streamingAudioWorker(); }};
}

auto Streamer::stopStreaming() -> void
{
  if (!streamingRunning.load())
    return;
  streamingShouldStop.store(true);

  // Wait for the thread to finish
  if (streamingVideoThread.joinable())
    streamingVideoThread.join();
  if (streamingAudioThread.joinable())
    streamingAudioThread.join();

  // Cleanup streaming
  av_write_trailer(fmtCtx);
  if ((fmtCtx->oformat->flags & AVFMT_NOFILE) == 0) // no no file == file
    avio_closep(&fmtCtx->pb);
  if (videoEncCtx)
    avcodec_free_context(&videoEncCtx);
  if (audioEncCtx)
    avcodec_free_context(&audioEncCtx);

  if (swrCtx)
  {
    swr_free(&swrCtx);
    swrCtx = nullptr;
  }

  avformat_free_context(fmtCtx);
  fmtCtx = nullptr;
  cleanupCapture();

  streamingRunning.store(false);
}

auto Streamer::streamingVideoWorker() -> void
{
  if (!glXMakeCurrent(captureDisplay, captureRootWindow, captureGLContext))
  {
    LOG("Failed to make capture GL context current");
    return;
  }

  auto rgb2yuv = Rgb2Yuv{8 /* number of threads */, width, height};
  auto videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_YUV420P;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    LOG("Could not allocate the video frame data");
    exit(1);
  }

  auto rgbData = static_cast<uint8_t *>(std::aligned_alloc(32, width * height * 3));
  if (!rgbData)
  {
    LOG("Allocation failed");
    return;
  }

  auto target = std::chrono::steady_clock::now() + std::chrono::microseconds(1'000'000 / 60);
  // Loop until stop requested
  while (!streamingShouldStop.load())
  {
    if (captureFrame(rgbData))
    {
      // Convert and send video
      uint8_t *dst[3] = {videoFrame->data[0], videoFrame->data[1], videoFrame->data[2]};
      int dstStride[3] = {videoFrame->linesize[0], videoFrame->linesize[1], videoFrame->linesize[2]};
      rgb2yuv.convert(rgbData, width * 3, dst, dstStride);
      videoFrame->pts = videoPts++;
      sendVideoFrame(videoFrame);
    }

    const auto t = std::chrono::steady_clock::now();
    if (t < target)
      std::this_thread::sleep_for(target - t);
    target += std::chrono::microseconds(1'000'000 / 60);
  }
  av_frame_free(&videoFrame);
}

auto Streamer::streamingAudioWorker() -> void
{
  auto audioSamples = std::array<int16_t, AudioBufSz * ChN>{};
  while (!streamingShouldStop.load())
  {
    if (!captureAudio(audioSamples.data(), AudioBufSz))
      continue;
    sendAudioFrame(reinterpret_cast<uint8_t *>(audioSamples.data()), AudioBufSz);
  }
}

auto Streamer::initVideoFrame() -> void
{
  videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_YUV420P;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    LOG("Could not allocate the video frame data");
    exit(1);
  }
}

// Capture a frame from the desktop using glReadPixels
auto Streamer::captureFrame(uint8_t *rgbData) -> bool
{
  if (!hideDesktop)
  {
    glReadBuffer(GL_FRONT);
    glReadPixels(x, displayHeight - height + y, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgbData);
    const auto cursorImage = XFixesGetCursorImage(captureDisplay);
    if (cursorImage)
    {
      const auto cursorX = cursorImage->x - cursorImage->xhot - x;
      const auto cursorY = cursorImage->y - cursorImage->yhot - y;

      for (auto j = 0; j < cursorImage->height; ++j)
      {
        const int imgY = cursorY + j;
        if (imgY < 0 || imgY >= height)
          continue;

        for (auto i = 0; i < cursorImage->width; ++i)
        {
          const auto imgX = cursorX + i;
          if (imgX < 0 || imgX >= width)
            continue;

          const auto cursorPixel = cursorImage->pixels[j * cursorImage->width + i];
          const auto alpha = (cursorPixel >> 24) & 0xff;
          if (alpha == 0)
            continue;

          const auto cr = static_cast<uint8_t>((cursorPixel >> 16) & 0xff);
          const auto cg = static_cast<uint8_t>((cursorPixel >> 8) & 0xff);
          const auto cb = static_cast<uint8_t>(cursorPixel & 0xff);

          // rgbData is (width*height*3)
          const auto imageIndex = ((height - imgY) * width + imgX) * 3;
          if (imageIndex < 0 || imageIndex >= width * height * 3)
            continue;

          const auto ir = rgbData[imageIndex];
          const auto ig = rgbData[imageIndex + 1];
          const auto ib = rgbData[imageIndex + 2];

          const auto nr = static_cast<uint8_t>((cr * alpha + ir * (255 - alpha)) / 255);
          const auto ng = static_cast<uint8_t>((cg * alpha + ig * (255 - alpha)) / 255);
          const auto nb = static_cast<uint8_t>((cb * alpha + ib * (255 - alpha)) / 255);

          rgbData[imageIndex] = nr;
          rgbData[imageIndex + 1] = ng;
          rgbData[imageIndex + 2] = nb;
        }
      }
      XFree(cursorImage);
    }
  }
  else
  {
    static int t = 0;
    ++t;
    for (auto y = 0; y < height; ++y)
      for (auto x = 0; x < width; ++x)
      {
        rgbData[(x + y * width) * 3 + 0] = (t + x + y) % 127;
        rgbData[(x + y * width) * 3 + 1] = (5 * t + x + 2 * y) % 127;
        rgbData[(x + y * width) * 3 + 2] = (7 * t + x + 3 * y) % 127;
      }
  }

  return true;
}

// Initialize X11, GLX, and set up OpenGL context for capturing
auto Streamer::initVideoCapture() -> bool
{
  // Open a separate display connection for capturing
  captureDisplay = XOpenDisplay(nullptr);
  if (!captureDisplay)
  {
    LOG("Cannot open capture display");
    return false;
  }

  auto captureRoot = DefaultRootWindow(captureDisplay);
  displayHeight = DisplayHeight(captureDisplay, 0);

  // Choose a visual
  GLint att[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  auto vi = glXChooseVisual(captureDisplay, DefaultScreen(captureDisplay), att);
  if (!vi)
  {
    LOG("No suitable visual found for capture");
    return false;
  }

  // Create a GLX context for capturing
  captureGLContext = glXCreateContext(captureDisplay, vi, NULL, GL_TRUE);
  if (!captureGLContext)
  {
    LOG("Cannot create GL context for capture");
    return false;
  }

  // Store other global variables as needed:
  captureRootWindow = captureRoot;
  return true;
}

// Initialize PulseAudio for capturing microphone audio
// Code adapted from original web-socket-session.cpp::initAudio()
auto Streamer::initAudioCapture() -> bool
{
  const auto ss = pa_sample_spec{.format = PA_SAMPLE_S16LE, .rate = SampleRate, .channels = ChN};
  const auto bufferAttr = pa_buffer_attr{.maxlength = (uint32_t)-1, // Default maximum buffer size
                                         .tlength = (uint32_t)-1,   // Not used for recording
                                         .prebuf = (uint32_t)-1,    // Not used for recording
                                         .minreq = (uint32_t)-1,    // Default minimum request size
                                         .fragsize = AudioBufSz * 4};

  auto error = 0;
  paStreamMic = pa_simple_new(nullptr,          // Default server
                              "Streamer",       // Application name
                              PA_STREAM_RECORD, // Record stream
                              nullptr,          // Default device (microphone)
                              "record",         // Stream description
                              &ss,              // Sample format spec
                              nullptr,          // Default channel map
                              &bufferAttr,      // Buffer attributes
                              &error);

  if (!paStreamMic)
  {
    LOG("pa_simple_new() failed:", pa_strerror(error));
    return false;
  }

  paStreamDesktop = pa_simple_new(nullptr,          // Default server
                                  "Streamer",       // Application name
                                  PA_STREAM_RECORD, // Record stream
                                  "@DEFAULT_SINK@.monitor",
                                  "record",    // Stream description
                                  &ss,         // Sample format spec
                                  nullptr,     // Default channel map
                                  &bufferAttr, // Buffer attributes
                                  &error);

  if (!paStreamDesktop)
  {
    LOG("pa_simple_new() failed:", pa_strerror(error));
    return false;
  }

  return true;
}

static auto calcLogVol(auto v)
{
  return exp(5 * (v - 1));
}

// Capture audio samples from the microphone
auto Streamer::captureAudio(int16_t *samples, int nbSamples) -> bool
{
  if (!paStreamMic)
    return false;
  if (!paStreamDesktop)
    return false;

  auto error = 0;
  const auto bytesToRead = nbSamples * ChN * sizeof(int16_t);
  auto micAudio = std::array<int16_t, AudioBufSz * ChN>{};
  if (pa_simple_read(paStreamMic, micAudio.data(), bytesToRead, &error) < 0)
  {
    LOG("pa_simple_read() failed:", pa_strerror(error));
    return false;
  }

  auto desktopAudio = std::array<int16_t, AudioBufSz * ChN>{};
  if (pa_simple_read(paStreamDesktop, desktopAudio.data(), bytesToRead, &error) < 0)
  {
    LOG("pa_simple_read() failed:", pa_strerror(error));
    return false;
  }

  const auto k = muteDesktopAudio ? 0.f : calcLogVol(desktopAudioVolume);
  for (auto i = 0; i < nbSamples * ChN; ++i)
    samples[i] = micAudio[i] + k * desktopAudio[i];

  return true;
}

auto Streamer::cleanupCapture() -> void
{
  // Destroy the GLX context
  if (captureGLContext)
  {
    glXDestroyContext(captureDisplay, captureGLContext);
    captureGLContext = nullptr;
  }

  // Close the X11 display
  if (captureDisplay)
  {
    XCloseDisplay(captureDisplay);
    captureDisplay = nullptr;
  }

  // Free PulseAudio stream
  if (paStreamMic)
  {
    pa_simple_free(paStreamMic);
    paStreamMic = nullptr;
  }
  if (paStreamDesktop)
  {
    pa_simple_free(paStreamDesktop);
    paStreamDesktop = nullptr;
  }
}

auto Streamer::setMuteDesktopAudio(bool v) -> void
{
  muteDesktopAudio = v;
}

auto Streamer::setDesktopAudioVolume(float v) -> void
{
  desktopAudioVolume = v;
}

auto Streamer::setHideDesktop(bool v) -> void
{
  hideDesktop = v;
}
