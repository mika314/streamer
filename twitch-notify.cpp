#include "twitch-notify.hpp"
#include <fstream>
#include <log/log.hpp>

static std::vector<int16_t> loadNotifSnd()
{
  std::vector<int16_t> ret;
  std::ifstream f("chat.s16le", std::ios::binary);
  if (!f)
    std::cerr << "File chat.s16le is missing\n";
  while (f)
  {
    int16_t v;
    f.read((char *)&v, sizeof(v));
    ret.push_back(v);
  }
  return ret;
}

auto TwitchNotify::onMsg(Msg) -> void
{
  audio.get().queue(notif.data(), notif.size() * sizeof(int16_t));
}

TwitchNotify::TwitchNotify(sdl::Audio &audio) : audio(audio), notif(loadNotifSnd()) {}
