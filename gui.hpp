#pragma once
#include <functional>
#include <ser/macro.hpp>
#include <string>

struct GuiState
{
  std::string rtmpUrl = "rtmp://lax.contribute.live-video.net/app";
  std::string streamKey; // user’s streaming key
  bool startRequested = false;
  bool stopRequested = false;
  bool streaming = false;
  SER_PROPS(rtmpUrl, streamKey);
};

// Initialize and run the GUI loop. The callback is called each frame.
// The callback can draw ImGui elements and read/write GuiState.
void RunGUI(GuiState &state, std::function<void()> mainLoopCallback);
