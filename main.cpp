#include "gui.hpp"
#include "pref.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  GuiState guiState;
  LoadPreferences(guiState);
  guiState.RunGUI();
}
