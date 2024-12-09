#pragma once
#include "streamer.hpp"
#include <ser/macro.hpp>
#include <string>

class Gui
{
public:
  std::string rtmpUrl = "rtmp://lax.contribute.live-video.net/app";
  std::string streamKey;
  std::string twitchUser = "mika314";
  std::string twitchKey;
  std::string twitchChannel = "mika314";
  bool muteDesktopAudio = false;
  float desktopAudioVolume = 1.f;
  SER_PROPS(rtmpUrl,
            streamKey,
            twitchUser,
            twitchKey,
            twitchChannel,
            muteDesktopAudio,
            desktopAudioVolume);

  auto run() -> void;

private:
  std::unique_ptr<Streamer> streamer;
};
