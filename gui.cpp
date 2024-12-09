#include "gui.hpp"
#include "imgui-impl-opengl3.h"
#include "imgui-impl-sdl2.h"
#include "pref.hpp"
#include "rgb2yuv.hpp"
#include "twitch-notify.hpp"
#include "twitch.hpp"
#include "uv.hpp"
#include <GL/gl.h>
#include <chrono>
#include <cstring>
#include <functional>
#include <imgui/imgui.h>
#include <iostream>
#include <log/log.hpp>
#include <sdlpp/sdlpp.hpp>
#include <string>

static auto inputText(const char *label, std::string &str, size_t bufSz = 256, unsigned flags = 0)
  -> bool
{
  auto buf = std::vector<char>{};
  buf.resize(bufSz);
  std::strncpy(buf.data(), str.c_str(), bufSz);
  if (!ImGui::InputText(label, buf.data(), bufSz, flags))
    return false;
  str = std::string(buf.data());
  return true;
}

auto Gui::run() -> void
{
  loadPref(*this);
  auto sdl = sdl::Init{SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_AUDIO};

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

  auto window = sdl::Window{"Streamer",
                            SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED,
                            800,
                            600,
                            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE};
  auto glCtx = SDL_GL_CreateContext(window.get());
  SDL_GL_MakeCurrent(window.get(), glCtx);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOpenGL(window.get(), glCtx);
  ImGui_ImplOpenGL3_Init("#version 150");

  auto uv = uv::Uv{};
  auto twitch = std::unique_ptr<Twitch>{};
  auto twitchNotify = std::unique_ptr<TwitchNotify>{};
  auto have = SDL_AudioSpec{};
  const auto want = SDL_AudioSpec{.freq = 24000, .format = AUDIO_S16, .channels = 1, .samples = 4096};
  auto audio = sdl::Audio{nullptr, false, &want, &have, 0};
  audio.pause(false);
  auto done = false;
  auto timer = uv.createTimer();
  timer.start(
    [&] {
      SDL_Event event;
      while (SDL_PollEvent(&event))
      {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT)
          done = true;
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();

      ImGui::Begin("Stream Controls");
      inputText("RTMP URL", rtmpUrl);
      inputText("Stream Key", streamKey, 256, ImGuiInputTextFlags_Password);
      inputText("User", twitchUser);
      inputText("Key", twitchKey, 256, ImGuiInputTextFlags_Password);
      inputText("Channel", twitchChannel);
      if (ImGui::Checkbox("Mute Desktop Audio", &muteDesktopAudio) && streamer)
        streamer->setMuteDesktopAudio(muteDesktopAudio);
      if (ImGui::SliderFloat("Desktop Audio Volume", &desktopAudioVolume, 0.0f, 1.0f) && streamer)
        streamer->setDesktopAudioVolume(desktopAudioVolume);

      if (!streamer)
      {
        if (ImGui::Button("Start Streaming"))
        {
          savePref(*this);
          streamer = std::make_unique<Streamer>();
          streamer->startStreaming(rtmpUrl, streamKey);
          streamer->setMuteDesktopAudio(muteDesktopAudio);
          streamer->setDesktopAudioVolume(desktopAudioVolume);

          twitch = std::make_unique<Twitch>(uv, twitchUser, twitchKey, twitchChannel);
          twitchNotify = std::make_unique<TwitchNotify>(audio);
          twitch->reg(*twitchNotify);
        }
      }
      else
      {
        if (ImGui::Button("Stop Streaming"))
        {
          twitch = nullptr;
          twitchNotify = nullptr;
          streamer->stopStreaming();
          streamer = nullptr;
        }
      }
      ImGui::End();

      ImGui::Render();
      auto &io = ImGui::GetIO();
      glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window.get());
    },
    0,
    1000 / 60);
  while (!done)
    uv.tick();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glCtx);
  savePref(*this);
}
