#include "application/gui/windows/SimulationWindow.hpp"

#include "flightui/FlightUI.hpp"
#include "application/gui/GUI.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/EngineState.hpp"
#include "application/sim/Simulation.hpp"

#include <string>
#include <vector>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float InputWidth = 180.0F;
constexpr float LayoutSpacing = 8.0F;
} // namespace

SimulationWindow::SimulationWindow()
    : Window("Simulation", EditorIconAliases::Simulation) {}

void SimulationWindow::OnRender(gui::GUI &gui) {
  if (!initialConditionLoaded_) {
    initialCondition_ =
        gui.GetPrimarySimulation().GetDefaultInitialCondition();
    initialConditionLoaded_ = true;
  }

  UI::TabGroup(
      "SimulationTabs")[+UI::Tab("Initial Condition")[UI::Custom([this, &gui] {
    DrawInitialConditionTab(gui);
  })] + UI::Tab("Diagnostics")[UI::Custom([this, &gui] {
    DrawDiagnosticsTab(gui);
  })] + UI::Tab("Environment")[UI::Custom([this] {
    DrawEnvironmentTab();
  })] + UI::Tab("Aircraft")[UI::Custom([this, &gui] { DrawAircraftTab(gui); })]]
      .Render();
}

void SimulationWindow::DrawInitialConditionTab(gui::GUI &gui) {
  UI::VerticalLayout()
      .Spacing(LayoutSpacing)[+DrawInitialConditionFields()
                              + DrawInitialConditionActions(gui)
                              + DrawLastError(gui)]
      .Render();
}

void SimulationWindow::DrawDiagnosticsTab(gui::GUI &gui) {
  auto &simulation = gui.GetPrimarySimulation();
  const auto &executionControl = gui.GetSimulationExecutionControl();
  const sim::AircraftState aircraftState =
      simulation.GetAircraft().GetAircraftState();

  UI::VerticalLayout()
      .Spacing(LayoutSpacing)[+UI::Heading("Diagnostics")
                              + UI::ValueLabel("Simulation Time",
                                  aircraftState.simulationTimeSec,
                                  "%.2f s")
                              + UI::ValueLabel("Tick Size",
                                  simulation.GetTickSizeSec(),
                                  "%.6f s")
                              + UI::ValueLabel("Pending Ticks",
                                  static_cast<int>(executionControl
                                          .GetPendingSimulationTickCount()),
                                  "%d")
                              + DrawLastError(gui)]
      .Render();
}

void SimulationWindow::DrawEnvironmentTab() {
  UI::TextDisabled("Wind and atmosphere configuration will be added here.")
      .Render();
}

void SimulationWindow::DrawAircraftTab(gui::GUI &gui) {
  const auto engineStates =
      gui.GetPrimarySimulation().GetAircraft().GetEngines().GetEngineStates();

  UI::VerticalLayoutBuilder layout = UI::VerticalLayout().Spacing(LayoutSpacing)
                                     + UI::Heading("Aircraft")
                                     + UI::ValueLabel("Engine Count",
                                         static_cast<int>(engineStates.size()),
                                         "%d");

  if (engineStates.empty()) {
    layout =
        layout + UI::TextDisabled("No engines are defined for this aircraft.");
    static_cast<UI::UIElement>(layout).Render();
    return;
  }

  for (const sim::EngineState &engineState : engineStates) {
    layout =
        layout
        + UI::VerticalLayout().Spacing(
            2.0F)[+UI::Text("Engine " + std::to_string(engineState.index))
                  + UI::Text(std::string("Status: ")
                             + (engineState.running ? "Running" : "Stopped"))
                  + UI::ValueLabel("RPM", engineState.rpm, "%.2f")
                  + UI::ValueLabel("Throttle",
                      engineState.throttleCommand,
                      "%.3f")];
  }

  static_cast<UI::UIElement>(layout).Render();
}

UI::UIElement SimulationWindow::DrawInitialConditionFields() {
  return UI::VerticalLayout().Spacing(LayoutSpacing)
      [+UI::Heading("Position")
          + UI::InputDouble("Latitude (deg)", initialCondition_.latitudeDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                initialCondition_.latitudeDeg = value;
              })
          + UI::InputDouble("Longitude (deg)", initialCondition_.longitudeDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                initialCondition_.longitudeDeg = value;
              })
          + UI::InputDouble("Altitude (ft)", initialCondition_.altitudeFt)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                initialCondition_.altitudeFt = value;
              })
          + UI::Heading("Attitude")
          + UI::InputDouble("Roll (deg)", initialCondition_.rollDeg)
              .Width(InputWidth)
              .OnChanged(
                  [this](double value) { initialCondition_.rollDeg = value; })
          + UI::InputDouble("Pitch (deg)", initialCondition_.pitchDeg)
              .Width(InputWidth)
              .OnChanged(
                  [this](double value) { initialCondition_.pitchDeg = value; })
          + UI::InputDouble("Heading (deg)", initialCondition_.headingDeg)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                initialCondition_.headingDeg = value;
              })
          + UI::Heading("Velocity")
          + UI::InputDouble("Airspeed (kt)", initialCondition_.airspeedKts)
              .Width(InputWidth)
              .OnChanged([this](double value) {
                initialCondition_.airspeedKts = value;
              })];
}

UI::UIElement SimulationWindow::DrawInitialConditionActions(gui::GUI &gui) {
  auto &simulation = gui.GetPrimarySimulation();
  auto &executionControl = gui.GetSimulationExecutionControl();

  return UI::HorizontalLayout().Spacing(LayoutSpacing)
      [+UI::Button("Reset With IC")
              .OnAction([this, &simulation, &executionControl] {
                const bool resumeAfterReset =
                    executionControl.GetSimulationExecutionState()
                    == application::SimulationExecutionState::Running;
                executionControl.PauseSimulation();
                if (executionControl.ResetSimulation(initialCondition_)
                    && resumeAfterReset) {
                  executionControl.ResumeSimulation();
                }
              })
          + UI::Button("Use Current State").OnAction([this, &simulation] {
              initialCondition_ = simulation.GetCurrentCondition();
            })
          + UI::Button("Reset Default").OnAction([this, &simulation] {
              initialCondition_ = simulation.GetDefaultInitialCondition();
              simulation.GetErrorTracker().ClearError();
            })];
}

UI::UIElement SimulationWindow::DrawLastError(gui::GUI &gui) const {
  const auto &lastError =
      gui.GetPrimarySimulation().GetErrorTracker().GetLastError();
  if (!lastError.has_value()) {
    return {};
  }

  return UI::TextWrapped("Error: " + *lastError);
}
} // namespace gui
