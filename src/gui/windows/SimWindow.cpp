#include "gui/windows/SimWindow.hpp"

#include "common/math/Math.hpp"
#include "flightui/FlightUI.hpp"

#include <string>
#include <vector>

namespace gui {

namespace {
constexpr float InputWidth = 180.0F;
constexpr float LayoutSpacing = 8.0F;
} // namespace

SimWindow::SimWindow(SimController &controller)
    : Window("Simulation", editor_icon_aliases::Simulation),
      controller_(controller) {}

void SimWindow::OnRender(const sim::SimSnapshot &snapshot) {
  controller_.Synchronize(snapshot);

  ui::TabGroup(
      "SimulationTabs")[+ui::Tab(
                            "Initial Condition")[ui::Custom([this, &snapshot] {
    DrawInitialConditionTab(snapshot);
  })] + ui::Tab("Diagnostics")[ui::Custom([this, &snapshot] {
    DrawDiagnosticsTab(snapshot);
  })] + ui::Tab("Environment")[ui::Custom([this] { DrawEnvironmentTab(); })]
                        + ui::Tab("Aircraft")[ui::Custom(
                            [this, &snapshot] { DrawAircraftTab(snapshot); })]]
      .Render();
}

void SimWindow::DrawInitialConditionTab(
    const sim::SimSnapshot &snapshot) {
  ui::VerticalLayout()
      .Spacing(LayoutSpacing)[+DrawInitialConditionFields()
                              + DrawInitialConditionActions(snapshot)
                              + DrawLastError(snapshot)]
      .Render();
}

void SimWindow::DrawDiagnosticsTab(
    const sim::SimSnapshot &snapshot) {
  const SimTransportProps transport = controller_.GetTransportProps();
  const double tickSizeSec = snapshot.simulationHz > 0.0
                                 ? 1.0 / snapshot.simulationHz
                                 : 0.0;

  ui::VerticalLayout()
      .Spacing(
          LayoutSpacing)[+ui::Heading("Diagnostics")
                         + ui::ValueLabel("Simulation Time",
                             snapshot.primary.aircraft.simulationTimeSec,
                             "%.2f s")
                         + ui::ValueLabel("Tick Size", tickSizeSec, "%.6f s")
                         + ui::ValueLabel("Pending Ticks",
                             static_cast<int>(transport.pendingTickCount),
                             "%d")
                         + DrawLastError(snapshot)]
      .Render();
}

void SimWindow::DrawEnvironmentTab() {
  ui::TextDisabled("Wind and atmosphere configuration will be added here.")
      .Render();
}

void SimWindow::DrawAircraftTab(
    const sim::SimSnapshot &snapshot) {
  const auto &engineStates = snapshot.primary.engines;

  ui::VerticalLayoutBuilder layout = ui::VerticalLayout().Spacing(LayoutSpacing)
                                     + ui::Heading("Aircraft")
                                     + ui::ValueLabel("Engine Count",
                                         static_cast<int>(engineStates.size()),
                                         "%d");

  if (engineStates.empty()) {
    layout =
        layout + ui::TextDisabled("No engines are defined for this aircraft.");
    static_cast<ui::UIElement>(layout).Render();
    return;
  }

  for (const sim::EngineState &engineState : engineStates) {
    layout =
        layout
        + ui::VerticalLayout().Spacing(
            2.0F)[+ui::Text("Engine " + std::to_string(engineState.index))
                  + ui::Text(std::string("Status: ")
                             + (engineState.running ? "Running" : "Stopped"))
                  + ui::ValueLabel("RPM", engineState.rpm, "%.2f")
                  + ui::ValueLabel("Throttle",
                      engineState.throttleCommand,
                      "%.3f")];
  }

  static_cast<ui::UIElement>(layout).Render();
}

ui::UIElement SimWindow::DrawInitialConditionFields() {
  const sim::InitialCondition &initialCondition =
      controller_.GetInitialConditionModel().pending;
  const double latitudeDeg = math::RadToDeg(initialCondition.latitudeRad);
  const double longitudeDeg = math::RadToDeg(initialCondition.longitudeRad);
  const double rollDeg = math::RadToDeg(initialCondition.rollRad);
  const double pitchDeg = math::RadToDeg(initialCondition.pitchRad);
  const double headingDeg = math::RadToDeg(initialCondition.headingRad);
  return ui::VerticalLayout().Spacing(LayoutSpacing)
      [+ui::Heading("Position")
          + ui::InputDouble("Latitude (deg)", latitudeDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent({InitialConditionField::LatitudeDeg, value});
              })
          + ui::InputDouble("Longitude (deg)", longitudeDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent(
                    {InitialConditionField::LongitudeDeg, value});
              })
          + ui::InputDouble("Altitude ASL (m)", initialCondition.altitudeAslM)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent(
                    {InitialConditionField::AltitudeAslM, value});
              })
          + ui::Heading("Attitude")
          + ui::InputDouble("Roll (deg)", rollDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent({InitialConditionField::RollDeg, value});
              })
          + ui::InputDouble("Pitch (deg)", pitchDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent({InitialConditionField::PitchDeg, value});
              })
          + ui::InputDouble("Heading (deg)", headingDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent({InitialConditionField::HeadingDeg, value});
              })
          + ui::Heading("Velocity")
          + ui::InputDouble(
                "Calibrated Airspeed (m/s)",
                initialCondition.calibratedAirspeedMps)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                controller_.OnEvent(
                    {InitialConditionField::CalibratedAirspeedMps, value});
              })];
}

ui::UIElement SimWindow::DrawInitialConditionActions(
    const sim::SimSnapshot &snapshot) {
  const sim::InitialCondition currentCondition =
      snapshot.primary.currentCondition;
  const sim::InitialCondition defaultCondition =
      snapshot.defaultInitialCondition;

  return ui::HorizontalLayout().Spacing(
      LayoutSpacing)[+ui::Button("Reset With IC").OnAction([this] {
    controller_.OnEvent(ResetWithInitialConditionRequested{});
  }) + ui::Button("Use Current State").OnAction([this, currentCondition] {
    controller_.OnEvent(UseCurrentInitialConditionRequested{currentCondition});
  }) + ui::Button("Reset Default").OnAction([this, defaultCondition] {
    controller_.OnEvent(
        RestoreDefaultInitialConditionRequested{defaultCondition});
  })];
}

ui::UIElement SimWindow::DrawLastError(
    const sim::SimSnapshot &snapshot) const {
  const std::string &lastError = snapshot.status.lastError;
  if (lastError.empty()) {
    return {};
  }

  return ui::TextWrapped("Error: " + lastError);
}
} // namespace gui
