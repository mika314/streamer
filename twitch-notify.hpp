#pragma once
#include "twitch-sink.hpp"
#include <sdlpp/sdlpp.hpp>

class TwitchNotify final : public TwitchSink
{
public:
  TwitchNotify(sdl::Audio &);
  auto onMsg(Msg) -> void final;

private:
  std::reference_wrapper<sdl::Audio> audio;
  std::vector<int16_t> notif;
};
