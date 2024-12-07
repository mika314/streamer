#include "gui.hpp"
#include <fstream>
#include <json-ser/json-ser.hpp>

static const char *PREF_FILE = "preferences.json";

void LoadPreferences(GuiState &state)
{
  std::ifstream ifs(PREF_FILE);
  if (!ifs)
  {
    return;
  }
  jsonDeser(ifs, state);
}

void SavePreferences(const GuiState &state)
{
  std::ofstream ofs(PREF_FILE);
  jsonSer(ofs, state);
}
