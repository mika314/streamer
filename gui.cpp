#include "gui.hpp"
#include "imgui-impl-opengl3.h"
#include "imgui-impl-sdl2.h"
#include <cstring>
#include <imgui/imgui.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif
#include <SDL.h>

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

void RunGUI(GuiState &state, std::function<void()> mainLoopCallback)
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

    // Main loop callback (for capturing frames, etc.)
    if (mainLoopCallback)
      mainLoopCallback();

    SDL_GL_MakeCurrent(window, gl_context);

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Stream Controls");
    InputText("RTMP URL", state.rtmpUrl);
    InputText("Stream Key", state.streamKey, 256, ImGuiInputTextFlags_Password);
    if (!state.streaming)
    {
      if (ImGui::Button("Start Streaming"))
      {
        state.startRequested = true;
      }
    }
    else
    {
      if (ImGui::Button("Stop Streaming"))
      {
        state.stopRequested = true;
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
