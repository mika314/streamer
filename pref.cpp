#include "gui.hpp"
#include <fstream>
#include <json-ser/json-ser.hpp>

static const char *PrefFile = "preferences.json";

auto loadPref(Gui &state) -> void
{
  auto ifs = std::ifstream{PrefFile};
  if (!ifs)
    return;
  jsonDeser(ifs, state);
}

auto savePref(const Gui &state) -> void
{
  auto ofs = std::ofstream{PrefFile};
  jsonSer(ofs, state);
}
