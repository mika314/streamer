#include "audio.hpp"
#include <cmath>
#include <log/log.hpp>

Audio::Audio() : loop(pa_mainloop_new()), ctx(pa_context_new(pa_mainloop_get_api(loop), "Streamer"))
{

  pa_context_set_state_callback(
    ctx, [](pa_context *, void *userdata) { static_cast<Audio *>(userdata)->ctxStateCb(); }, this);

  pa_context_connect(ctx, nullptr, PA_CONTEXT_NOFLAGS, nullptr);
}

void Audio::ctxStateCb()
{
  pa_context_state_t state = pa_context_get_state(ctx);

  switch (state)
  {
  case PA_CONTEXT_READY: {
    const auto bufferAttr = pa_buffer_attr{.maxlength = (uint32_t)-1, // Default maximum buffer size
                                           .tlength = (uint32_t)-1,   // Not used for recording
                                           .prebuf = (uint32_t)-1,    // Not used for recording
                                           .minreq = (uint32_t)-1,    // Default minimum request size
                                           .fragsize = BufSz};
    const auto ss = pa_sample_spec{.format = PA_SAMPLE_S16LE, .rate = SampleRate, .channels = ChN};
    mic = pa_stream_new(ctx, "mic", &ss, nullptr);
    pa_stream_connect_record(mic, nullptr, &bufferAttr, PA_STREAM_NOFLAGS);
    pa_stream_set_read_callback(
      mic,
      [](pa_stream *, size_t nbytes, void *userdata) {
        static_cast<Audio *>(userdata)->micReadCb(nbytes);
      },
      this);
    desktopAudio = pa_stream_new(ctx, "desktop audio", &ss, nullptr);
    pa_stream_connect_record(desktopAudio, "@DEFAULT_SINK@.monitor", &bufferAttr, PA_STREAM_NOFLAGS);
    pa_stream_set_read_callback(
      desktopAudio,
      [](pa_stream *, size_t nbytes, void *userdata) {
        static_cast<Audio *>(userdata)->desktopAudioReadCb(nbytes);
      },
      this);
    break;
  }

  case PA_CONTEXT_FAILED:
  case PA_CONTEXT_TERMINATED:
    /* Something went wrong or server shut down */
    pa_mainloop_quit(loop, 1);
    break;

  default:
    /* Ignored states: UNCONNECTED, CONNECTING, AUTHORIZING, SETUP */
    break;
  }
}

static auto calcLogVol(auto v)
{
  return exp(5 * (v - 1));
}

auto Audio::micReadCb(size_t nbytes) -> void
{
  const void *data;
  pa_stream_peek(mic, &data, &nbytes);
  const auto desieredMicK = [&]() {
    if (muteMic)
      return 0.0;
    if (!boostMic)
      return 1.0;
    const auto k = *std::max_element(static_cast<const int16_t *>(data),
                                     static_cast<const int16_t *>(data) + nbytes / sizeof(int16_t));
    if (k <= 0)
      return 1.;
    return .5 * (1. + a * pow(k, b) * exp(-c * k));
  }();

  micLevel = 0.0;
  for (auto it = static_cast<const int16_t *>(data);
       it < static_cast<const int16_t *>(data) + nbytes / sizeof(int16_t);
       ++it)
  {
    const auto y = std::clamp(micK * *it, -32767., 32767.);
    micK = micK + 0.00005 * (desieredMicK - micK);
    micLevel = std::max(y, micLevel);
    if (micPos < static_cast<int>(buf.size()))
    {
      buf[micPos] = std::clamp(buf[micPos] + y, -32767., 32767.);
      ++micPos;
    }
    else
      micOverflowBuf.push_back(y);
  }
  pa_stream_drop(mic);
  callbackIfNeeded();
}

auto Audio::desktopAudioReadCb(size_t nbytes) -> void
{
  const void *data;
  pa_stream_peek(desktopAudio, &data, &nbytes);
  const auto desktopK = muteDesktopAudio ? 0.0 : calcLogVol(desktopAudioVolume);
  desktopAudioLevel = 0.0;
  for (auto it = static_cast<const int16_t *>(data);
       it < static_cast<const int16_t *>(data) + nbytes / sizeof(int16_t);
       ++it)
  {
    const auto y = std::clamp(desktopK * *it, -32767., 32767.);
    desktopAudioLevel = std::max(y, desktopAudioLevel);
    if (desktopAudioPos < static_cast<int>(buf.size()))
    {
      buf[desktopAudioPos] = std::clamp(y + buf[desktopAudioPos], -32767., 32767.);
      ++desktopAudioPos;
    }
    else
      desktopAudioOverflowBuf.push_back(y);
  }
  pa_stream_drop(desktopAudio);
  callbackIfNeeded();
}

auto Audio::callbackIfNeeded() -> void
{
  for (;;)
  {
    if (micPos < static_cast<int>(buf.size()))
      break;
    if (desktopAudioPos < static_cast<int>(buf.size()))
      break;
    cb(buf);
    for (auto &v : buf)
      v = 0;
    micPos = 0;
    desktopAudioPos = 0;
    while (!micOverflowBuf.empty() && micPos < static_cast<int>(buf.size()))
    {
      buf[micPos] = std::clamp(static_cast<int>(buf[micPos]) + micOverflowBuf.front(), -0x7fff, 0x7fff);
      ++micPos;
      micOverflowBuf.pop_front();
    }
    while (!desktopAudioOverflowBuf.empty() && desktopAudioPos < static_cast<int>(buf.size()))
    {
      buf[desktopAudioPos] = std::clamp(
        static_cast<int>(buf[desktopAudioPos]) + desktopAudioOverflowBuf.front(), -0x7fff, 0x7fff);
      ++desktopAudioPos;
      desktopAudioOverflowBuf.pop_front();
    }
  }

  if (std::abs(static_cast<int>(desktopAudioOverflowBuf.size()) -
               static_cast<int>(micOverflowBuf.size())) > SampleRate * ChN * MaxAvDesync / 1000)
  {
    LOG("desync between desktop audio and mic",
        static_cast<int>(desktopAudioOverflowBuf.size()) - static_cast<int>(micOverflowBuf.size()));
    while (desktopAudioOverflowBuf.size() != micOverflowBuf.size())
      if (desktopAudioOverflowBuf.size() > micOverflowBuf.size())
        micOverflowBuf.push_back(0);
      else
        desktopAudioOverflowBuf.push_back(0);
  }
}

auto Audio::run(std::function<void(const std::array<int16_t, BufSz * ChN> &)> lCb) -> void
{
  cb = std::move(lCb);
  LOG("Starting the infinite loop");
  pa_mainloop_run(loop, nullptr);
  LOG("The infinite loop is ended");
}

auto Audio::stop() -> void
{
  pa_mainloop_quit(loop, 0);
}

Audio::~Audio()
{
  pa_stream_disconnect(desktopAudio);
  pa_stream_unref(desktopAudio);
  pa_stream_disconnect(mic);
  pa_stream_unref(mic);
  pa_context_disconnect(ctx);
  pa_context_unref(ctx);
  pa_mainloop_free(loop);
}

auto Audio::setMuteDesktopAudio(bool v) -> void
{
  muteDesktopAudio = v;
}

auto Audio::setDesktopAudioVolume(double v) -> void
{
  desktopAudioVolume = v;
}

auto Audio::setMuteMic(bool v) -> void
{
  muteMic = v;
}

auto Audio::setBoostMic(bool v) -> void
{
  boostMic = v;
}

auto Audio::getDesktopAudioLevel() -> float
{
  return std::clamp(10. * log10(desktopAudioLevel / 0x7fff) / 30. + 1., 0.0, 1.);
}

auto Audio::getMicLevel() -> float
{
  return std::clamp(10. * log10(micLevel / 0x7fff) / 30. + 1., 0.0, 1.);
}

auto Audio::setNoiseLevel(double v) -> void
{
  const auto j = calcLogVol(v) * 0x7fff;
  const auto k = 16000 / j;
  b = log(k - 1) / ((k - 1) - log(k));
  c = b / j;
  a = (k - 1) * exp(b) / (pow(j, b));
  LOG("j", j, "a", a, "b", b, "c", c);
}
