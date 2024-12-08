#include "streamer.hpp"
#include "rgb2yuv.hpp"
#include <iostream>
#include <log/log.hpp>

static const int ChannelsNum = 2;
static const int SampleRate = 48'000;
static const int AudioBufSz = 1024;

Streamer::Streamer()
{
  avformat_network_init();
}

Streamer::~Streamer()
{
  stop();
}

bool Streamer::start(const std::string &url, int width, int height)
{
  if (running)
    return true;

#ifndef TEST
  int ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", url.c_str());
#else
  int ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", "test.flv");
#endif
  if (ret < 0)
  {
    std::cerr << "Could not create output context\n";
    return false;
  }
  fmtCtx->oformat->flags |= AVFMT_GLOBALHEADER;

  if (!initVideoStream(width, height))
    return false;
  if (!initAudioStream())
    return false;

  if (!(fmtCtx->oformat->flags & AVFMT_NOFILE))
  {
    LOG("format is file format, open file");
#ifndef TEST
    ret = avio_open2(&fmtCtx->pb, url.c_str(), AVIO_FLAG_WRITE, nullptr, nullptr);
#else
    ret = avio_open2(&fmtCtx->pb, "test.flv", AVIO_FLAG_WRITE, nullptr, nullptr);
#endif
    if (ret < 0)
    {
      std::cerr << "Could not open output URL\n";
      return false;
    }
  }

#ifndef TEST
  av_dump_format(fmtCtx, 0, "rtmp://[censored]", 1);
#else
  av_dump_format(fmtCtx, 0, "test.flv", 1);
#endif
  ret = avformat_write_header(fmtCtx, nullptr);
  if (ret < 0)
  {
    std::cerr << "Error occurred when opening output URL\n";
    return false;
  }

  running = true;
  return true;
}

void Streamer::stop()
{
  if (!running)
    return;
  av_write_trailer(fmtCtx);
  if (!(fmtCtx->oformat->flags & AVFMT_NOFILE))
    avio_closep(&fmtCtx->pb);
  if (videoEncCtx)
    avcodec_free_context(&videoEncCtx);
  if (audioEncCtx)
    avcodec_free_context(&audioEncCtx);

  // Free SwrContext
  if (swrCtx)
  {
    swr_free(&swrCtx);
    swrCtx = nullptr;
  }

  avformat_free_context(fmtCtx);
  fmtCtx = nullptr;
  running = false;
}

bool Streamer::initVideoStream(int width, int height)
{
  AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    std::cerr << "H.264 encoder not found\n";
    return false;
  }
  videoSt = avformat_new_stream(fmtCtx, codec);
  if (!videoSt)
    return false;
  videoEncCtx = avcodec_alloc_context3(codec);
  if (!videoEncCtx)
  {
    LOG("Could not allocate video codec context");
    exit(1);
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
    return false;
  avcodec_parameters_from_context(videoSt->codecpar, videoEncCtx);
  return true;
}

bool Streamer::initAudioStream()
{
  AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
  if (!codec)
  {
    std::cerr << "AAC encoder not found\n";
    return false;
  }
  audioSt = avformat_new_stream(fmtCtx, codec);
  audioEncCtx = avcodec_alloc_context3(codec);
  audioEncCtx->sample_rate = SampleRate;
  audioEncCtx->channel_layout = ChannelsNum == 2 ? AV_CH_LAYOUT_STEREO : AV_CH_LAYOUT_MONO;
  audioEncCtx->channels = ChannelsNum;
  audioEncCtx->sample_fmt = codec->sample_fmts[0]; // pick first supported
  audioEncCtx->bit_rate = 128000;
  audioEncCtx->time_base = {1, audioEncCtx->sample_rate};
  audioEncCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  if (avcodec_open2(audioEncCtx, codec, nullptr) < 0)
    return false;
  avcodec_parameters_from_context(audioSt->codecpar, audioEncCtx);

  // After audioEncCtx is open:
  SwrContext *swrCtx =
    swr_alloc_set_opts(nullptr,
                       audioEncCtx->channel_layout, // output layout
                       audioEncCtx->sample_fmt,     // output format (FLTP)
                       audioEncCtx->sample_rate,    // output sample rate
                       audioEncCtx->channel_layout, // input layout (S16LE from pa_simple)
                       AV_SAMPLE_FMT_S16,           // input format from PulseAudio
                       audioEncCtx->sample_rate,    // input sample rate (assuming same)
                       0,
                       nullptr);

  if (!swrCtx || swr_init(swrCtx) < 0)
  {
    std::cerr << "Failed to initialize SwrContext\n";
    avcodec_free_context(&audioEncCtx);
    return false;
  }

  // Store swrCtx as a class member so we can use it in sendAudioFrame()
  this->swrCtx = swrCtx;

  return true;
}

int Streamer::encodeAndWrite(AVCodecContext *encCtx, AVFrame *frame, AVStream *st)
{
  auto lock = std::unique_lock{mutex};
  int ret = avcodec_send_frame(encCtx, frame);
  if (ret < 0 && ret != AVERROR_EOF && ret != AVERROR(EAGAIN))
    return ret;

  AVPacket *pkt = av_packet_alloc();
  if (!pkt)
    return AVERROR(ENOMEM);

  while (true)
  {
    ret = avcodec_receive_packet(encCtx, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    {
      av_packet_free(&pkt);
      return 0;
    }
    else if (ret < 0)
    {
      av_packet_free(&pkt);
      return ret;
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
    ret = av_interleaved_write_frame(fmtCtx, pkt);
    if (ret < 0)
    {
      char errbuf[128];
      av_strerror(ret, errbuf, sizeof(errbuf));
      LOG("Error writing frame:", errbuf);
      return false;
    }
    av_packet_unref(pkt);
    if (ret < 0)
    {
      av_packet_free(&pkt);
      return ret;
    }
  }

  // Unreachable, but for clarity:
  av_packet_free(&pkt);
  return 0;
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
    std::cerr << "No SwrContext initialized\n";
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
    std::cerr << "Could not allocate audio frame samples\n";
    av_frame_free(&af);
    return -1;
  }

  if (av_frame_make_writable(af) < 0)
  {
    std::cerr << "Could not make audio frame writable\n";
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
    std::cerr << "Error converting audio samples\n";
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
  // Combine URL and key
  std::string fullUrl = url + "/" + key;

  // Initialize capture resources (GLX context, display, streamer, audio, etc.)
  // This is separate from the main thread’s SDL context.
  if (!initVideoCapture())
  {
    std::cerr << "Failed to init video capture\n";
    streamingRunning.store(false);
    return;
  }

  if (!initAudioCapture())
  {
    std::cerr << "Failed to init audio capture\n";
    streamingRunning.store(false);
    return;
  }

  if (!start(fullUrl, width, height))
  {
    std::cerr << "Failed to start streamer\n";
    cleanupCapture();
    av_frame_free(&videoFrame);
    streamingRunning.store(false);
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
  stop();
  cleanupCapture();

  streamingRunning.store(false);
}

void Streamer::streamingVideoWorker()
{
  auto rgb2yuv = Rgb2Yuv{8, width, height};
  AVFrame *videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_YUV420P;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    std::cerr << "Could not allocate the video frame data" << std::endl;
    exit(1);
  }

  static uint8_t *rgbData = (uint8_t *)std::aligned_alloc(32, width * height * 3);
  if (!rgbData)
  {
    std::cerr << "Allocation failed\n";
    return;
  }

  auto target = std::chrono::steady_clock::now() + std::chrono::microseconds(1'000'000 / 60);
  // Loop until stop requested
  while (!streamingShouldStop.load())
  {
    // Capture frame
    if (captureFrame(rgbData))
    {
      // Convert and send video
      uint8_t *dst[3] = {videoFrame->data[0], videoFrame->data[1], videoFrame->data[2]};
      int dstStride[3] = {videoFrame->linesize[0], videoFrame->linesize[1], videoFrame->linesize[2]};
      rgb2yuv.convert(rgbData, width * 3, dst, dstStride);
      videoFrame->pts = videoPts++;
      sendVideoFrame(videoFrame);
    }

    // Add sleep or timing control if necessary to maintain frame rate
    const auto t = std::chrono::steady_clock::now();
    if (t < target)
      std::this_thread::sleep_for(target - t);
    target += std::chrono::microseconds(1'000'000 / 60);
  }
  av_frame_free(&videoFrame);
}

void Streamer::streamingAudioWorker()
{
  while (!streamingShouldStop.load())
  {
    // Capture audio
    static int16_t audioSamples[AudioBufSz * ChannelsNum];
    if (captureAudio(audioSamples, AudioBufSz))
    {
      sendAudioFrame((uint8_t *)audioSamples, AudioBufSz);
    }
  }
}

void Streamer::initVideoFrame()
{
  videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_YUV420P;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    std::cerr << "Could not allocate the video frame data" << std::endl;
    exit(1);
  }
}

// Capture a frame from the desktop using glReadPixels
// This code is adapted from the original screen cast app's video capture logic.
bool Streamer::captureFrame(uint8_t *rgbData)
{
  GLXContext oldContext = glXGetCurrentContext();
  GLXDrawable oldDrawable = glXGetCurrentDrawable();
  Display *oldDisplay = glXGetCurrentDisplay();

  // Before reading pixels, make the capture context current
  if (!glXMakeCurrent(captureDisplay, captureRootWindow, captureGLContext))
  {
    std::cerr << "Failed to make capture GL context current" << std::endl;
    return false;
  }

  // At this point, glReadPixels will read from the root window's front buffer,
  // assuming a compositor or other mechanism is in place. If not, you may see black frames.
  glReadBuffer(GL_FRONT);
  glReadPixels(x, displayHeight - height + y, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgbData);

  const auto cursorImage = XFixesGetCursorImage(captureDisplay);
  if (cursorImage)
  {
    const auto cursorX = cursorImage->x - cursorImage->xhot - x;
    const auto cursorY = cursorImage->y - cursorImage->yhot - y;

    for (int j = 0; j < (int)cursorImage->height; ++j)
    {
      const int imgY = cursorY + j;
      if (imgY < 0 || imgY >= height)
        continue;

      for (int i = 0; i < (int)cursorImage->width; ++i)
      {
        const int imgX = cursorX + i;
        if (imgX < 0 || imgX >= width)
          continue;

        const unsigned long cursorPixel = cursorImage->pixels[j * cursorImage->width + i];
        const auto alpha = (cursorPixel >> 24) & 0xff;
        if (alpha == 0)
          continue;

        const auto cr = static_cast<uint8_t>((cursorPixel >> 16) & 0xff);
        const auto cg = static_cast<uint8_t>((cursorPixel >> 8) & 0xff);
        const auto cb = static_cast<uint8_t>(cursorPixel & 0xff);

        // rgbData is (width*height*3)
        const auto imageIndex = ((height - imgY) * width + imgX) * 3;

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

  glXMakeCurrent(oldDisplay, oldDrawable, oldContext);

  return true;
}

// -------------------- IMPLEMENTATIONS --------------------

// Initialize X11, GLX, and set up OpenGL context for capturing
bool Streamer::initVideoCapture()
{
  // Open a separate display connection for capturing
  captureDisplay = XOpenDisplay(nullptr);
  if (!captureDisplay)
  {
    std::cerr << "Cannot open capture display" << std::endl;
    return false;
  }

  Window captureRoot = DefaultRootWindow(captureDisplay);
  displayHeight = DisplayHeight(captureDisplay, 0);
  int screen = DefaultScreen(captureDisplay);

  // Choose a visual
  GLint att[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  XVisualInfo *vi = glXChooseVisual(captureDisplay, screen, att);
  if (!vi)
  {
    std::cerr << "No suitable visual found for capture" << std::endl;
    return false;
  }

  // Create a GLX context for capturing
  captureGLContext = glXCreateContext(captureDisplay, vi, NULL, GL_TRUE);
  if (!captureGLContext)
  {
    std::cerr << "Cannot create GL context for capture" << std::endl;
    return false;
  }

  // Note: We do NOT call glXMakeCurrent here permanently.
  // We'll do that in captureFrame() when we actually need to read pixels.

  // Store other global variables as needed:
  captureRootWindow = captureRoot;
  return true;
}

// Initialize PulseAudio for capturing microphone audio
// Code adapted from original web-socket-session.cpp::initAudio()
bool Streamer::initAudioCapture()
{
  pa_sample_spec ss;
  ss.format = PA_SAMPLE_S16LE; // 16-bit PCM
  ss.rate = SampleRate;
  ss.channels = ChannelsNum;

  pa_buffer_attr buffer_attr;
  buffer_attr.maxlength = (uint32_t)-1; // Default maximum buffer size
  buffer_attr.tlength = (uint32_t)-1;   // Not used for recording
  buffer_attr.prebuf = (uint32_t)-1;    // Not used for recording
  buffer_attr.minreq = (uint32_t)-1;    // Default minimum request size
  buffer_attr.fragsize = AudioBufSz;

  int error;
  paStream = pa_simple_new(nullptr,          // Default server
                           "Streamer",       // Application name
                           PA_STREAM_RECORD, // Record stream
                           nullptr,          // Default device (microphone)
                           "record",         // Stream description
                           &ss,              // Sample format spec
                           nullptr,          // Default channel map
                           &buffer_attr,     // Buffer attributes
                           &error);

  if (!paStream)
  {
    std::cerr << "pa_simple_new() failed: " << pa_strerror(error) << std::endl;
    return false;
  }

  return true;
}

// Capture audio samples from the microphone
// using pa_simple_read (adapted from original code).
bool Streamer::captureAudio(int16_t *samples, int nbSamples)
{
  if (!paStream)
    return false;

  int error;
  size_t bytesToRead = nbSamples * ChannelsNum * sizeof(int16_t);
  if (pa_simple_read(paStream, samples, bytesToRead, &error) < 0)
  {
    std::cerr << "pa_simple_read() failed: " << pa_strerror(error) << std::endl;
    // On error, zero out samples to avoid NaN.
    memset(samples, 0, bytesToRead);
    return false;
  }
  return true;
}

void Streamer::cleanupCapture()
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
  if (paStream)
  {
    pa_simple_free(paStream);
    paStream = nullptr;
  }
}
