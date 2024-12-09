# Streamer App

This application enables you to:

1. **Capture and stream your desktop** (video) and microphone/desktop audio to an RTMP endpoint (commonly Twitch).
2. **Connect to a Twitch channel chat**, and play a notification sound on new chat messages.

## Features

- **Desktop Capture (X11, OpenGL, GLX)**: Captures the current desktop frame from the primary display on Linux using GLX and X11.
- **Cursor Overlay**: Captures and composites the cursor image into the video stream.
- **Audio Capture**: Records from both the default microphone and the desktop audio monitor via PulseAudio.
- **Encoding**: Uses FFmpeg libraries (libavcodec, libavformat, libavutil, libswresample) to encode video (H.264) and audio (AAC), then sends them to the given RTMP endpoint.
- **Twitch Integration**: Connects to Twitch IRC (chat) to listen for channel messages. When a message arrives, it can trigger an audio notification and print the message to the log.
- **GUI with ImGui & SDL2**: Provides a GUI where you can enter your RTMP URL, stream key, Twitch username, OAuth key, and channel. Also provides start/stop streaming controls.

## Dependencies

You will need:

- A **Linux** environment with:
  - **X11** development headers and libraries.
  - **GLX** and OpenGL headers and libraries.
- **FFmpeg** (development libraries):
  Make sure you have `libavcodec-dev`, `libavformat-dev`, `libavutil-dev`, `libswresample-dev` packages.
- **PulseAudio** development headers: `libpulse-dev`.
- **SDL2** development package: `libsdl2-dev`.
- **UV** (libuv) for asynchronous I/O with Twitch IRC.

## **Getting Started**

### **Building Instructions on Ubuntu 22.04**

1. **Install Dependencies**
   ```bash
   sudo apt update
   sudo apt install -y \
   clang \
   git \
   libavcodec-dev \
   libavformat-dev \
   libavutil-dev \
   libgl-dev \
   libglx-dev \
   libpulse-dev \
   libsdl2-dev \
   libswresample-dev \
   libuv-dev \
   libuv1-dev \
   libx11-dev \
   libxext-dev \
   libxfixes-dev \
   pkg-config
   ```

2. **Install and Build `coddle` (Build Tool)**
   ```bash
   git clone https://github.com/coddle-cpp/coddle.git
   cd coddle
   ./build.sh
   sudo ./deploy.sh
   cd ..
   ```

3. **Clone and Build Streamer App**
   ```bash
   git clone https://github.com/mika314/streamer.git
   cd streamer
   make
   ```

---

## Running

1. Make sure you have a Twitch OAuth key for your Twitch user (https://dev.twitch.tv/docs/irc/auth).
   
2. Run the `streamer` binary:
   ```sh
   ./streamer
   ```

3. A GUI window should appear with fields:
   - **RTMP URL**: The RTMP endpoint. For Twitch, it's often `rtmp://<region>.contribute.live-video.net/app`
   - **Stream Key**: Your Twitch stream key.
   - **User**: Your Twitch username (e.g., `myusername`).
   - **Key**: Your Twitch OAuth key (e.g. `oauth:xxxxxxxxxxxxxx`).
   - **Channel**: The Twitch channel to connect to (e.g., `myusername`).

   These preferences are saved into `preferences.json`.

4. Click **Start Streaming** to begin sending the video and audio to your RTMP endpoint. It will also connect to Twitch chat. Any incoming chat messages will be logged to the console and trigger a notification sound.

5. Click **Stop Streaming** to end the stream.

## Additional Notes

- **Preferences**: The app stores the RTMP URL, stream key, Twitch username, key, and channel in `preferences.json`. This file will be automatically created and saved on exit, and loaded on next run.
  
- **Window Capture**: This uses the root window of X11 and `glReadPixels` to capture the desktop. It requires a running X11 environment and may not work in a Wayland-only setup without XWayland.

- **Performance**: The code uses hardware-accelerated OpenGL reads and AVX2 vectorization (in `rgb2yuv.cpp`) to speed up pixel conversions. If your CPU doesn't support AVX2, you may need to adjust the code.

- **Error Handling**: Basic error handling is included. If something fails (e.g., cannot connect to Twitch), it will retry with exponential backoff. For RTMP streaming, if the connection fails to open or encode, it will log the error.

## Troubleshooting

- If the program fails to load dependencies, verify that you have all the required libraries and headers.
- If audio capture doesn't work, ensure that PulseAudio is running and accessible. The code reads from `@DEFAULT_SINK@.monitor` for desktop audio and the default microphone input.
  You can adjust source names in the code if needed.
- Twitch chat requires a valid OAuth token. You can generate it from Twitch’s recommended tools. The format is usually `oauth:xxxxxx`.
- If video is not captured correctly, ensure you have a working GLX setup and no errors appear in the logs.

## License

This code is provided with the MIT license. The dependencies (FFmpeg, SDL2, ImGui, etc.) come with their own licenses.


**Enjoy streaming!** If you face issues, check the logs or console output for hints.
