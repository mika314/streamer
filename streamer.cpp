#include "streamer.hpp"
#include <iostream>
#include <log/log.hpp>

#define TEST

Streamer::Streamer()
{
  avformat_network_init();
}

Streamer::~Streamer()
{
  stop();
}

bool Streamer::start(const std::string &url, int width, int height)
{
  if (running)
    return true;

#ifndef TEST
  int ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", url.c_str());
#else
  int ret = avformat_alloc_output_context2(&fmtCtx, nullptr, "flv", "test.flv");
#endif
  if (ret < 0)
  {
    std::cerr << "Could not create output context\n";
    return false;
  }

  if (!initVideoStream(width, height))
    return false;
  if (!initAudioStream())
    return false;

  if (!(fmtCtx->oformat->flags & AVFMT_NOFILE))
  {
    LOG("format is file format, open file");
#ifndef TEST
    ret = avio_open(&fmtCtx->pb, url.c_str(), AVIO_FLAG_WRITE);
#else
    ret = avio_open(&fmtCtx->pb, "test.flv", AVIO_FLAG_WRITE);
#endif
    if (ret < 0)
    {
      std::cerr << "Could not open output URL\n";
      return false;
    }
  }

  ret = avformat_write_header(fmtCtx, nullptr);
  if (ret < 0)
  {
    std::cerr << "Error occurred when opening output URL\n";
    return false;
  }

  running = true;
  return true;
}

void Streamer::stop()
{
  if (!running)
    return;
  av_write_trailer(fmtCtx);
  if (!(fmtCtx->oformat->flags & AVFMT_NOFILE))
    avio_closep(&fmtCtx->pb);
  if (videoEncCtx)
    avcodec_free_context(&videoEncCtx);
  if (audioEncCtx)
    avcodec_free_context(&audioEncCtx);

  // Free SwrContext
  if (swrCtx)
  {
    swr_free(&swrCtx);
    swrCtx = nullptr;
  }

  avformat_free_context(fmtCtx);
  fmtCtx = nullptr;
  running = false;
}

bool Streamer::initVideoStream(int width, int height)
{
  AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!codec)
  {
    std::cerr << "H.264 encoder not found\n";
    return false;
  }
  videoSt = avformat_new_stream(fmtCtx, codec);
  if (!videoSt)
    return false;
  videoEncCtx = avcodec_alloc_context3(codec);
  videoEncCtx->width = width;
  videoEncCtx->height = height;
  videoEncCtx->time_base = {1, 60};
  videoEncCtx->framerate = {60, 1};
  videoEncCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  // Tune for streaming
  av_opt_set(videoEncCtx->priv_data, "preset", "ultrafast", 0);
  av_opt_set(videoEncCtx->priv_data, "tune", "zerolatency", 0);
  // Bitrate, profile, etc.
  videoEncCtx->bit_rate = 2000000; // 2Mbps
  if (avcodec_open2(videoEncCtx, codec, nullptr) < 0)
    return false;
  avcodec_parameters_from_context(videoSt->codecpar, videoEncCtx);
  return true;
}

bool Streamer::initAudioStream()
{
  AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
  if (!codec)
  {
    std::cerr << "AAC encoder not found\n";
    return false;
  }
  audioSt = avformat_new_stream(fmtCtx, codec);
  audioEncCtx = avcodec_alloc_context3(codec);
  audioEncCtx->sample_rate = 48000;
  audioEncCtx->channel_layout = AV_CH_LAYOUT_STEREO;
  audioEncCtx->channels = 2;
  audioEncCtx->sample_fmt = codec->sample_fmts[0]; // pick first supported
  audioEncCtx->bit_rate = 128000;
  audioEncCtx->time_base = {1, audioEncCtx->sample_rate};
  if (avcodec_open2(audioEncCtx, codec, nullptr) < 0)
    return false;
  avcodec_parameters_from_context(audioSt->codecpar, audioEncCtx);

  // After audioEncCtx is open:
  SwrContext *swrCtx =
    swr_alloc_set_opts(nullptr,
                       audioEncCtx->channel_layout, // output layout
                       audioEncCtx->sample_fmt,     // output format (FLTP)
                       audioEncCtx->sample_rate,    // output sample rate
                       audioEncCtx->channel_layout, // input layout (S16LE from pa_simple)
                       AV_SAMPLE_FMT_S16,           // input format from PulseAudio
                       audioEncCtx->sample_rate,    // input sample rate (assuming same)
                       0,
                       nullptr);

  if (!swrCtx || swr_init(swrCtx) < 0)
  {
    std::cerr << "Failed to initialize SwrContext\n";
    avcodec_free_context(&audioEncCtx);
    return false;
  }

  // Store swrCtx as a class member so we can use it in sendAudioFrame()
  this->swrCtx = swrCtx;

  return true;
}

int Streamer::encodeAndWrite(AVCodecContext *encCtx, AVFrame *frame, AVStream *st)
{
  if (frame)
    frame->pts = (st == videoSt) ? videoPts++ : audioPts++;
  int ret = avcodec_send_frame(encCtx, frame);
  if (ret < 0 && ret != AVERROR_EOF && ret != AVERROR(EAGAIN))
    return ret;

  AVPacket *pkt = av_packet_alloc();
  if (!pkt)
    return AVERROR(ENOMEM);

  while (true)
  {
    ret = avcodec_receive_packet(encCtx, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    {
      av_packet_free(&pkt);
      return 0;
    }
    else if (ret < 0)
    {
      av_packet_free(&pkt);
      return ret;
    }
    pkt->stream_index = st->index;
    av_packet_rescale_ts(pkt, encCtx->time_base, st->time_base);
    LOG("write frame pts:", pkt->pts, "stream_index:", pkt->stream_index);
    ret = av_interleaved_write_frame(fmtCtx, pkt);
    av_packet_unref(pkt);
    if (ret < 0)
    {
      av_packet_free(&pkt);
      return ret;
    }
  }

  // Unreachable, but for clarity:
  av_packet_free(&pkt);
  return 0;
}

int Streamer::sendVideoFrame(AVFrame *frame)
{
  return encodeAndWrite(videoEncCtx, frame, videoSt);
}

// For audio, you need to convert your captured PCM (interleaved int16_t) to AVFrame samples.
// This code assumes your PCM is int16_t stereo, and we need to convert it to the encoder’s format
// (likely FLTP). In a real scenario, use SwrContext to resample from S16 to FLTP if needed.

int Streamer::sendAudioFrame(const uint8_t *pcmData, int nbSamples)
{
  if (!swrCtx)
  {
    std::cerr << "No SwrContext initialized\n";
    return -1;
  }

  // Allocate a frame for the converted samples
  AVFrame *af = av_frame_alloc();
  af->nb_samples = nbSamples;
  af->channel_layout = audioEncCtx->channel_layout;
  af->format = audioEncCtx->sample_fmt;
  af->sample_rate = audioEncCtx->sample_rate;

  if (av_frame_get_buffer(af, 0) < 0)
  {
    std::cerr << "Could not allocate audio frame samples\n";
    av_frame_free(&af);
    return -1;
  }

  if (av_frame_make_writable(af) < 0)
  {
    std::cerr << "Could not make audio frame writable\n";
    av_frame_free(&af);
    return -1;
  }

  // Input from pa_simple_read is interleaved S16
  const uint8_t *inData[1] = {pcmData};

  // Convert S16 interleaved to FLTP planar
  int convertedSamples = swr_convert(swrCtx,
                                     af->data, // output buffers
                                     nbSamples,
                                     inData, // input buffers
                                     nbSamples);

  if (convertedSamples < 0)
  {
    std::cerr << "Error converting audio samples\n";
    av_frame_free(&af);
    return -1;
  }

  // Set pts (in samples)
  af->pts = audioPts;
  audioPts += convertedSamples; // increment pts by the number of samples

  // Now encode and send
  int ret = encodeAndWrite(audioEncCtx, af, audioSt);
  av_frame_free(&af);
  return ret;
}
