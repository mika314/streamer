#pragma once
#include "audio.hpp"
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xfixes.h>
#include <atomic>
#include <string>
#include <thread>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

class Streamer
{
public:
  Streamer();
  ~Streamer();
  Streamer(Streamer &) = delete;
  auto operator=(Streamer) = delete;

  auto startStreaming(std::string url, std::string key) -> void;
  auto stopStreaming() -> void;
  auto setMuteDesktopAudio(bool) -> void;
  auto setDesktopAudioVolume(double) -> void;
  auto setHideDesktop(bool) -> void;
  auto setMuteMic(bool) -> void;
  auto setBoostMic(bool) -> void;
  auto getDesktopAudioLevel() -> float;
  auto getMicLevel() -> float;
  auto setNoiseLevel(double) -> void;

private:
  AVFormatContext *fmtCtx = nullptr;
  AVCodecContext *videoEncCtx = nullptr;
  AVCodecContext *audioEncCtx = nullptr;
  AVStream *videoSt = nullptr;
  AVStream *audioSt = nullptr;
  int64_t videoPts = 0;
  int64_t audioPts = 0;
  SwrContext *swrCtx = nullptr; // for audio format conversion
  Display *captureDisplay = nullptr;
  std::thread streamingVideoThread;
  std::thread streamingAudioThread;
  std::atomic<bool> streamingRunning{false};
  std::atomic<bool> streamingShouldStop{false};
  AVFrame *videoFrame = nullptr;
  const int width = 1920;
  const int height = 1080;
  GLXContext captureGLContext;
  Window captureRootWindow;
  int displayHeight;
  int x = 0;
  int y = 0;
  Audio audio;
  std::mutex mutex;
  bool hideDesktop = false;
  decltype(std::chrono::steady_clock::now()) startTime;
  std::atomic<bool> videoReady{false};

  auto captureAudio(int16_t *samples, int nbSamples) -> bool;
  auto captureFrame(uint8_t *rgbData) -> void;
  auto cleanupCapture() -> void;
  auto encodeAndWrite(AVCodecContext *, AVFrame *, AVStream *) -> int;
  auto initAudioStream() -> bool;
  auto initVideoCapture() -> bool;
  auto initVideoFrame() -> void;
  auto initVideoStream() -> bool;
  auto sendAudioFrame(const uint8_t *pcmData, int nbSamples) -> int;
  auto streamingAudioWorker() -> void;
  auto streamingVideoWorker() -> void;
};
