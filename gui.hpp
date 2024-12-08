#pragma once
#include "streamer.hpp"
#include <ser/macro.hpp>
#include <string>

struct GuiState
{
  std::string rtmpUrl = "rtmp://lax.contribute.live-video.net/app";
  std::string streamKey; // user’s streaming key
  bool startRequested = false;
  bool stopRequested = false;
  std::unique_ptr<Streamer> streamer;
  SER_PROPS(rtmpUrl, streamKey);

  void RunGUI();
};
