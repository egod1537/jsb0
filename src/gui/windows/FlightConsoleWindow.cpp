#include "gui/windows/FlightConsoleWindow.hpp"
#include "flightui/FlightUI.hpp"
#include "sim/SimulationConfig.h"
#include "sim/runtime/SimulationContracts.hpp"

namespace gui {
namespace UI = FlightUI;

FlightConsoleWindow::FlightConsoleWindow() : Window("Flight Console") {}

void FlightConsoleWindow::OnRender(const sim::SimulationSnapshot &snapshot) {
  const sim::SimulationConfig &config = snapshot.config;
  const sim::InitialCondition &initialCondition =
      snapshot.defaultInitialCondition;

  // clang-format off
  FlightUI::UIElement content =
      UI::VerticalLayout()
      [
        +UI::Heading("JSB Flight Console")
        + UI::Text("Aircraft: " + config.aircraftName)
        + UI::ValueLabel("Simulation", config.simulationHz, "%.1f Hz")
        + UI::ValueLabel("Initial altitude ASL", initialCondition.altitudeAslM,
                         "%.1f m")
        + UI::ValueLabel("Initial CAS",
                         initialCondition.calibratedAirspeedMps, "%.1f m/s")
      ];
  // clang-format on

  content.Render();
}
} // namespace gui
