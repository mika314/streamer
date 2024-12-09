#pragma once
#include "streamer.hpp"
#include <ser/macro.hpp>
#include <string>

class Gui
{
public:
  std::string rtmpUrl = "rtmp://lax.contribute.live-video.net/app";
  std::string streamKey;
  SER_PROPS(rtmpUrl, streamKey);

  auto run() -> void;

private:
  std::unique_ptr<Streamer> streamer;
};
