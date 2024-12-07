#pragma once
#include <string>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}

class Streamer
{
public:
  Streamer();
  ~Streamer();
  bool start(const std::string &url, int width, int height);
  void stop();
  bool isRunning() const { return running; }

  // Encode and send video frame. Frame should be YUV420P in frame->data.
  int sendVideoFrame(AVFrame *frame);

  // Encode and send audio frame (PCM -> AAC)
  int sendAudioFrame(const uint8_t *pcmData, int nbSamples);

private:
  bool running = false;
  AVFormatContext *fmtCtx = nullptr;
  AVCodecContext *videoEncCtx = nullptr;
  AVCodecContext *audioEncCtx = nullptr;
  AVStream *videoSt = nullptr;
  AVStream *audioSt = nullptr;
  int64_t videoPts = 0;
  int64_t audioPts = 0;
  SwrContext *swrCtx = nullptr; // for audio format conversion

  // Internal functions to init encoders, add streams
  bool initVideoStream(int width, int height);
  bool initAudioStream();
  int encodeAndWrite(AVCodecContext *encCtx, AVFrame *frame, AVStream *st);
};
