#include "gui.hpp"
#include "imgui-impl-opengl3.h"
#include "imgui-impl-sdl2.h"
#include "pref.hpp"
#include "rgb2yuv.hpp"
#include <GL/gl.h>
#include <SDL.h>
#include <chrono>
#include <cstring>
#include <functional>
#include <imgui/imgui.h>
#include <iostream>
#include <libavutil/frame.h>
#include <log/log.hpp>
#include <string>

bool InputText(const char *label, std::string &str, size_t buf_size = 256, unsigned flags = 0)
{
  char buffer[buf_size];
  std::strncpy(buffer, str.c_str(), buf_size);
  if (ImGui::InputText(label, buffer, buf_size, flags))
  {
    str = std::string(buffer);
    return true;
  }
  return false;
}

void GuiState::RunGUI()
{
  // SDL/GL init...
  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

  SDL_Window *window = SDL_CreateWindow("Streamer",
                                        SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED,
                                        800,
                                        600,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  SDL_GLContext gl_context = SDL_GL_CreateContext(window);
  SDL_GL_MakeCurrent(window, gl_context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
  ImGui_ImplOpenGL3_Init("#version 150");

  bool done = false;
  while (!done)
  {
    // Handle events
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
    InputText("RTMP URL", rtmpUrl);
    InputText("Stream Key", streamKey, 256, ImGuiInputTextFlags_Password);
    if (!streamer)
    {
      if (ImGui::Button("Start Streaming"))
      {
        streamer = std::make_unique<Streamer>();
        streamer->startStreaming(rtmpUrl, streamKey);
        SavePreferences(*this);
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
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
