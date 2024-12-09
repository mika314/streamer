#include "gui.hpp"
#include "imgui-impl-opengl3.h"
#include "imgui-impl-sdl2.h"
#include "pref.hpp"
#include "rgb2yuv.hpp"
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
  auto sdl = sdl::Init{SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER};

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

  for (auto done = false; !done;)
  {
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
    if (!streamer)
    {
      if (ImGui::Button("Start Streaming"))
      {
        savePref(*this);
        streamer = std::make_unique<Streamer>();
        streamer->startStreaming(rtmpUrl, streamKey);
      }
    }
    else
    {
      if (ImGui::Button("Stop Streaming"))
      {
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
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glCtx);
  savePref(*this);
}
