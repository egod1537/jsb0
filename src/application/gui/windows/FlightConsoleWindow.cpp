#include "application/gui/windows/FlightConsoleWindow.hpp"
#include "flightui/FlightUI.hpp"
#include "application/gui/GUI.hpp"
#include "application/sim/SimulationConfig.h"

namespace gui {
namespace UI = FlightUI;

FlightConsoleWindow::FlightConsoleWindow() : Window("Flight Console") {}

void FlightConsoleWindow::OnRender(GUI &gui) {
  const sim::SimulationConfig &config = gui.GetPrimarySimulation().GetConfig();
  const sim::InitialCondition &initialCondition =
      gui.GetPrimarySimulation().GetDefaultInitialCondition();

  // clang-format off
  FlightUI::UIElement content =
      UI::VerticalLayout()
      [
        +UI::Heading("JSB Flight Console")
        + UI::Text("Aircraft: " + config.aircraftName)
        + UI::ValueLabel("Simulation", config.simulationHz, "%.1f Hz")
        + UI::ValueLabel("Initial altitude", initialCondition.altitudeFt, "%.0f ft")
        + UI::ValueLabel("Initial airspeed", initialCondition.airspeedKts,
                         "%.0f kt")
      ];
  // clang-format on

  content.Render();
}
} // namespace gui
