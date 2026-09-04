#include "gui/windows/FlightConsoleWindow.hpp"
#include "flightui/FlightUI.hpp"
#include "sim/runtime/SimContracts.hpp"

namespace gui {

FlightConsoleWindow::FlightConsoleWindow() : Window("Flight Console") {}

void FlightConsoleWindow::OnRender(const sim::SimSnapshot &snapshot) {
  const sim::InitialCondition &initialCondition =
      snapshot.defaultInitialCondition;

  // clang-format off
  ui::UIElement content =
      ui::VerticalLayout()
      [
        +ui::Heading("JSB Flight Console")
        + ui::Text("Aircraft: " + snapshot.aircraftName)
        + ui::ValueLabel("Simulation", snapshot.simulationHz, "%.1f Hz")
        + ui::ValueLabel("Initial altitude ASL", initialCondition.altitudeAslM,
                         "%.1f m")
        + ui::ValueLabel("Initial CAS",
                         initialCondition.calibratedAirspeedMps, "%.1f m/s")
      ];
  // clang-format on

  content.Render();
}
} // namespace gui
