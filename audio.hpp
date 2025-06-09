#pragma once
#include <cstdint>
#include <deque>
#include <functional>

extern "C" {
#include <pulse/context.h>
#include <pulse/mainloop.h>
#include <pulse/stream.h>
}

class Audio
{
public:
  enum {
    BufSz = 1024,
    ChN = 2,
    SampleRate = 48'000,
    MaxAvDesync = 50 // ms
  };

  Audio();
  ~Audio();
  auto getDesktopAudioLevel() -> float;
  auto getMicLevel() -> float;
  auto run(std::function<void(const std::array<int16_t, BufSz * ChN> &)>) -> void;
  auto setBoostMic(bool v) -> void;
  auto setDesktopAudioVolume(double v) -> void;
  auto setMuteDesktopAudio(bool v) -> void;
  auto setMuteMic(bool v) -> void;
  auto setNoiseLevel(double v) -> void;
  auto stop() -> void;

private:
  pa_mainloop *loop;
  pa_context *ctx;
  pa_stream *mic;
  pa_stream *desktopAudio;
  std::array<int16_t, BufSz * ChN> buf;
  std::deque<int16_t> micOverflowBuf;
  std::deque<int16_t> desktopAudioOverflowBuf;
  int micPos = 0;
  int desktopAudioPos = 0;
  std::function<void(const std::array<int16_t, BufSz * ChN> &)> cb;
  bool muteDesktopAudio = false;
  double desktopAudioVolume = 1.;
  double micK = 0.0;
  bool muteMic = false;
  bool boostMic = true;
  double desktopAudioLevel = 0.0;
  double micLevel = 0.0;
  double a = 1.1106;
  double b = 0.32807;
  double c = 0.00020504;

  auto micReadCb(size_t nbytes) -> void;
  auto desktopAudioReadCb(size_t nbytes) -> void;
  auto callbackIfNeeded() -> void;
  auto ctxStateCb() -> void;
};
