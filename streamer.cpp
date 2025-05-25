#include "streamer.hpp"
#include "rgb2yuv.hpp"
#include <algorithm>
#include <array>
#include <log/log.hpp>

static const int Fps = 60;

Streamer::Streamer() : startTime(std::chrono::steady_clock::now())
{
  avformat_network_init();
}

Streamer::~Streamer()
{
  stopStreaming();
}

auto Streamer::initVideoStream() -> bool
{
  // auto codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  auto codec = avcodec_find_encoder_by_name("h264_nvenc");
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
  videoEncCtx->bit_rate = 5'000'000;
  videoEncCtx->width = width;
  videoEncCtx->height = height;
  videoEncCtx->time_base = {1, Fps};
  videoEncCtx->framerate = {Fps, 1};
  videoEncCtx->gop_size = 2 * Fps;
  videoEncCtx->max_b_frames = 0;
  videoEncCtx->pix_fmt = AV_PIX_FMT_NV12;

  videoEncCtx->flags |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_GLOBAL_HEADER;
  videoEncCtx->thread_count = 0;

  av_opt_set(videoEncCtx->priv_data, "preset", "ultrafast", 0);
  av_opt_set(videoEncCtx->priv_data, "profile", "baseline", 0);
  av_opt_set(videoEncCtx->priv_data, "tune", "zerolatency", 0);
  av_opt_set(videoEncCtx->priv_data, "crf", "30", 0);
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
  audioEncCtx->sample_rate = Audio::SampleRate;
  audioEncCtx->channel_layout = Audio::ChN == 2 ? AV_CH_LAYOUT_STEREO : AV_CH_LAYOUT_MONO;
  audioEncCtx->channels = Audio::ChN;
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

  // LOG("0 send frame", st->index, "pts", frame->pts);

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
  if (audioPts < 0)
    audioPts = std::chrono::duration_cast<std::chrono::seconds>(
                 (std::chrono::steady_clock::now() - startTime) * Audio::SampleRate)
                 .count();
  af->pts = audioPts;
  audioPts += Audio::BufSz;

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
  videoReady.store(false);

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
  videoFrame->format = AV_PIX_FMT_NV12;
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

  using namespace std::chrono_literals;

  videoReady.store(true);
  const auto now = std::chrono::steady_clock::now();
  videoPts = std::chrono::duration_cast<std::chrono::seconds>((now - startTime) * Fps).count();

  auto target = now + std::chrono::microseconds(1'000'000 / Fps);
  auto tryingCatchUp = false;
  while (!streamingShouldStop.load())
  {
    if (!tryingCatchUp)
    {
      captureFrame(rgbData);
      // Convert and send video
      uint8_t *dst[3] = {videoFrame->data[0], videoFrame->data[1], videoFrame->data[2]};
      int dstStride[3] = {videoFrame->linesize[0], videoFrame->linesize[1], videoFrame->linesize[2]};
      rgb2yuv.convert(rgbData, width * 3, dst, dstStride);
    }

    videoFrame->pts = videoPts++;
    encodeAndWrite(videoEncCtx, videoFrame, videoSt);

    const auto t = std::chrono::steady_clock::now();
    if (t < target)
    {
      std::this_thread::sleep_for(target - t);
      tryingCatchUp = false;
    }
    else if (t - target > 1s)
    {
      tryingCatchUp = true;
      LOG("trying to catch up",
          std::chrono::duration_cast<std::chrono::milliseconds>(t - target).count());
    }

    target += std::chrono::microseconds(1'000'000 / Fps);
  }
  av_frame_free(&videoFrame);
}

auto Streamer::streamingAudioWorker() -> void
{
  audioPts = -1;
  auto skipCnt = 0;

  audio.run([&](const std::array<int16_t, Audio::BufSz * Audio::ChN> &buf) {
    if (streamingShouldStop.load())
      audio.stop();
    if (!videoReady.load())
      return;
    if (skipCnt > 0)
    {
      LOG("skipping");
      --skipCnt;
      return;
    }
    sendAudioFrame(reinterpret_cast<const uint8_t *>(buf.data()), Audio::BufSz);
    const auto expectedPts = std::chrono::duration_cast<std::chrono::seconds>(
                               (std::chrono::steady_clock::now() - startTime) * Audio::SampleRate)
                               .count();
    if (std::abs(audioPts - expectedPts) > Audio::SampleRate * Audio::MaxAvDesync / 1000)
    {
      LOG("desync more than ",
          Audio::MaxAvDesync,
          "ms",
          (audioPts - expectedPts) * 1000 / Audio::SampleRate);
      if (expectedPts > audioPts)
      {
        while (expectedPts > audioPts)
        {
          LOG("Sending extra");
          sendAudioFrame(reinterpret_cast<const uint8_t *>(buf.data()), Audio::BufSz);
        }
      }
      else
        skipCnt = (audioPts - expectedPts) / Audio::BufSz;
    }
  });
}

auto Streamer::initVideoFrame() -> void
{
  videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_NV12;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    LOG("Could not allocate the video frame data");
    exit(1);
  }
}

// Capture a frame from the desktop using glReadPixels
auto Streamer::captureFrame(uint8_t *rgbData) -> void
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
        rgbData[(x + y * width) * 3 + 0] = (t / 5 + x + y) % 127;
        rgbData[(x + y * width) * 3 + 1] = (t + x + 2 * y) % 127;
        rgbData[(x + y * width) * 3 + 2] = (7 * t / 5 + x + 3 * y) % 127;
      }
  }
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
}

auto Streamer::setMuteDesktopAudio(bool v) -> void
{
  audio.setMuteDesktopAudio(v);
}

auto Streamer::setDesktopAudioVolume(double v) -> void
{
  audio.setDesktopAudioVolume(v);
}

auto Streamer::setHideDesktop(bool v) -> void
{
  hideDesktop = v;
}

auto Streamer::setMuteMic(bool v) -> void
{
  audio.setMuteMic(v);
}

auto Streamer::setBoostMic(bool v) -> void
{
  audio.setBoostMic(v);
}

auto Streamer::getDesktopAudioLevel() -> float
{
  return audio.getDesktopAudioLevel();
}

auto Streamer::getMicLevel() -> float
{
  return audio.getMicLevel();
}

auto Streamer::setNoiseLevel(double v) -> void
{
  audio.setNoiseLevel(v);
}
