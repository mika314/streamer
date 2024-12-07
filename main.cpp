#include "gui.hpp"
#include "pref.hpp"
#include "rgb2yuv.hpp"
#include "streamer.hpp"
#include <GL/gl.h>
#include <GL/glx.h>
#include <SDL.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xfixes.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <libavutil/frame.h>
#include <log/log.hpp>
#include <string>

extern "C" {
#include <pulse/error.h>
#include <pulse/simple.h>
}

// Global variables for capturing
static Display *captureDisplay = nullptr;
static Window captureRootWindow;
static GLXContext captureGLContext;
static int displayHeight;
static int width = 1920;
static int height = 1080;
static int x = 0;
static int y = 0;

// Audio capture
static pa_simple *paStream = nullptr;

// GUI/streaming state
GuiState guiState;
Streamer streamer;
Rgb2Yuv *rgb2yuv = nullptr;
AVFrame *videoFrame = nullptr;

// Forward declarations
bool initVideoCapture();
bool captureFrame(uint8_t *rgbData);
bool initAudioCapture();
bool captureAudio(int16_t *samples, int nbSamples);

// Initialize the video frame buffer
void initVideoFrame()
{
  videoFrame = av_frame_alloc();
  videoFrame->format = AV_PIX_FMT_YUV420P;
  videoFrame->width = width;
  videoFrame->height = height;
  if (av_frame_get_buffer(videoFrame, 32) < 0)
  {
    std::cerr << "Could not allocate the video frame data" << std::endl;
    exit(1);
  }
}

// Main loop callback for GUI
void mainLoopCallback()
{
  // Start or stop streaming if requested
  if (guiState.startRequested)
  {
    guiState.startRequested = false;
    // Combine URL + key
    std::string fullUrl = guiState.rtmpUrl + "/" + guiState.streamKey;
    if (streamer.start(fullUrl, width, height))
    {
      guiState.streaming = true;
      SavePreferences(guiState);
    }
  }

  if (guiState.stopRequested)
  {
    guiState.stopRequested = false;
    streamer.stop();
    guiState.streaming = false;
  }

  // If streaming, capture and send frames/audio
  if (guiState.streaming)
  {
    // Use aligned_alloc for proper alignment:
    static uint8_t *rgbData = (uint8_t *)std::aligned_alloc(32, width * height * 3);
    // Handle if aligned_alloc fails:
    if (!rgbData)
    {
      std::cerr << "Failed to allocate aligned memory for rgbData\n";
      exit(1);
    }

    // Capture a frame from the desktop
    if (captureFrame(rgbData))
    {
      // Convert RGB to YUV420P
      uint8_t *dst[3] = {videoFrame->data[0], videoFrame->data[1], videoFrame->data[2]};
      int dstStride[3] = {videoFrame->linesize[0], videoFrame->linesize[1], videoFrame->linesize[2]};
      rgb2yuv->convert(rgbData, width * 3, dst, dstStride);

      // Send video frame
      streamer.sendVideoFrame(videoFrame);
    }

    // Capture audio
    static int16_t audioSamples[960 * 2]; // 5ms at 48kHz stereo (960 samples)
    if (captureAudio(audioSamples, 960))
    {
      streamer.sendAudioFrame((uint8_t *)audioSamples, 960);
    }
  }
}

int main(int argc, char **argv)
{
  LoadPreferences(guiState);

  // Initialize video capture (OpenGL + X11)
  if (!initVideoCapture())
  {
    std::cerr << "Failed to initialize video capture" << std::endl;
    return 1;
  }

  // Initialize audio capture (PulseAudio)
  if (!initAudioCapture())
  {
    std::cerr << "Failed to initialize audio capture" << std::endl;
    return 1;
  }

  // Initialize ImGui, etc.
  // This UI context is separate and will remain active for UI tasks.

  rgb2yuv = new Rgb2Yuv(8, width, height);
  initVideoFrame();

  RunGUI(guiState, mainLoopCallback);

  // Cleanup
  delete rgb2yuv;
  if (videoFrame)
    av_frame_free(&videoFrame);

  if (paStream)
  {
    pa_simple_free(paStream);
    paStream = nullptr;
  }

  glXDestroyContext(captureDisplay, captureGLContext);
  XCloseDisplay(captureDisplay);

  return 0;
}

// -------------------- IMPLEMENTATIONS --------------------

// Initialize X11, GLX, and set up OpenGL context for capturing
bool initVideoCapture()
{
  // Open a separate display connection for capturing
  captureDisplay = XOpenDisplay(nullptr);
  if (!captureDisplay)
  {
    std::cerr << "Cannot open capture display" << std::endl;
    return false;
  }

  Window captureRoot = DefaultRootWindow(captureDisplay);
  displayHeight = DisplayHeight(captureDisplay, 0);
  int screen = DefaultScreen(captureDisplay);

  // Choose a visual
  GLint att[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  XVisualInfo *vi = glXChooseVisual(captureDisplay, screen, att);
  if (!vi)
  {
    std::cerr << "No suitable visual found for capture" << std::endl;
    return false;
  }

  // Create a GLX context for capturing
  captureGLContext = glXCreateContext(captureDisplay, vi, NULL, GL_TRUE);
  if (!captureGLContext)
  {
    std::cerr << "Cannot create GL context for capture" << std::endl;
    return false;
  }

  // Note: We do NOT call glXMakeCurrent here permanently.
  // We'll do that in captureFrame() when we actually need to read pixels.

  // Store other global variables as needed:
  captureRootWindow = captureRoot;
  return true;
}

// Capture a frame from the desktop using glReadPixels
// This code is adapted from the original screen cast app's video capture logic.
bool captureFrame(uint8_t *rgbData)
{
  // Before reading pixels, make the capture context current
  if (!glXMakeCurrent(captureDisplay, captureRootWindow, captureGLContext))
  {
    std::cerr << "Failed to make capture GL context current" << std::endl;
    return false;
  }

  // At this point, glReadPixels will read from the root window's front buffer,
  // assuming a compositor or other mechanism is in place. If not, you may see black frames.
  glReadBuffer(GL_FRONT);
  glReadPixels(x, displayHeight - height + y, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgbData);

  const auto cursorImage = XFixesGetCursorImage(captureDisplay);
  if (cursorImage)
  {
    const auto cursorX = cursorImage->x - cursorImage->xhot - x;
    const auto cursorY = cursorImage->y - cursorImage->yhot - y;

    for (int j = 0; j < (int)cursorImage->height; ++j)
    {
      const int imgY = cursorY + j;
      if (imgY < 0 || imgY >= height)
        continue;

      for (int i = 0; i < (int)cursorImage->width; ++i)
      {
        const int imgX = cursorX + i;
        if (imgX < 0 || imgX >= width)
          continue;

        const unsigned long cursorPixel = cursorImage->pixels[j * cursorImage->width + i];
        const auto alpha = (cursorPixel >> 24) & 0xff;
        if (alpha == 0)
          continue;

        const auto cr = static_cast<uint8_t>((cursorPixel >> 16) & 0xff);
        const auto cg = static_cast<uint8_t>((cursorPixel >> 8) & 0xff);
        const auto cb = static_cast<uint8_t>(cursorPixel & 0xff);

        // rgbData is (width*height*3)
        const auto imageIndex = ((height - imgY) * width + imgX) * 3;

        const auto ir = rgbData[imageIndex];
        const auto ig = rgbData[imageIndex + 1];
        const auto ib = rgbData[imageIndex + 2];

        const auto nr = static_cast<uint8_t>((cr * alpha + ir * (255 - alpha)) / 255);
        const auto ng = static_cast<uint8_t>((cg * alpha + ig * (255 - alpha)) / 255);
        const auto nb = static_cast<uint8_t>((cb * alpha + ib * (255 - alpha)) / 255);

        rgbData[imageIndex] = nr;
        rgbData[imageIndex + 1] = ng;
        rgbData[imageIndex + 2] = nb;
      }
    }
    XFree(cursorImage);
  }

  return true;
}

// Initialize PulseAudio for capturing microphone audio
// Code adapted from original web-socket-session.cpp::initAudio()
bool initAudioCapture()
{
  pa_sample_spec ss;
  ss.format = PA_SAMPLE_S16LE; // 16-bit PCM
  ss.rate = 48000;             // 48kHz sample rate
  ss.channels = 2;             // Stereo

  pa_buffer_attr buffer_attr;
  buffer_attr.maxlength = (uint32_t)-1; // Default maximum buffer size
  buffer_attr.tlength = (uint32_t)-1;   // Not used for recording
  buffer_attr.prebuf = (uint32_t)-1;    // Not used for recording
  buffer_attr.minreq = (uint32_t)-1;    // Default minimum request size
  buffer_attr.fragsize = (uint32_t)-1;

  int error;
  paStream = pa_simple_new(NULL,             // Default server
                           "Streamer",       // Application name
                           PA_STREAM_RECORD, // Record stream
                           NULL,             // Default device (microphone)
                           "record",         // Stream description
                           &ss,              // Sample format spec
                           NULL,             // Default channel map
                           &buffer_attr,     // Buffer attributes
                           &error);
  if (!paStream)
  {
    std::cerr << "pa_simple_new() failed: " << pa_strerror(error) << std::endl;
    return false;
  }

  return true;
}

// Capture audio samples from the microphone
// using pa_simple_read (adapted from original code).
bool captureAudio(int16_t *samples, int nbSamples)
{
  if (!paStream)
    return false;

  int error;
  size_t bytesToRead = nbSamples * 2 * sizeof(int16_t); // stereo * 2 bytes per sample
  if (pa_simple_read(paStream, samples, bytesToRead, &error) < 0)
  {
    std::cerr << "pa_simple_read() failed: " << pa_strerror(error) << std::endl;
    // On error, zero out samples to avoid NaN.
    memset(samples, 0, bytesToRead);
    return false;
  }
  return true;
}
