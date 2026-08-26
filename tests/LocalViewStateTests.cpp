#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/panels/BaselineAutopilotPanel.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"
#include "application/gui/windows/GNCWindow.hpp"
#include "application/gui/windows/monitor/RollTrackingAcceptance.hpp"
#include "application/gui/windows/viz/FlightVizWindow.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/gnc/autopilot/MyAutopilot.hpp"
#include "application/sim/gnc/autopilot/PX4Autopilot.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <type_traits>

namespace {
template <typename T>
concept HasBaselineRollHoldTuningState = requires(T &state) {
  state.rollHold;
  state.rollTargetDeg;
  state.px4RollTimeConstantSec;
  state.px4RollTuningOpen;
  state.px4RollDiagnosticsOpen;
};

template <typename T>
concept HasLegacyPx4ReferenceState = requires(T &state) {
  state.legacyPx4ReferenceOpen;
};

template <typename T>
concept HasAircraftViewMode = requires(T &visualizer) {
  visualizer.GetAircraftViewMode();
  visualizer.CycleAircraftViewMode();
};

template <typename T>
concept HasMutableSimulationSources =
    requires(T &view, sim::Simulation *simulation) {
      view.SetMainSimulation(simulation);
      view.SetShadowSimulation(simulation);
    };

static_assert(!HasBaselineRollHoldTuningState<gui::AutopilotPanelState>);
static_assert(
    HasBaselineRollHoldTuningState<gui::BaselineAutopilotPanelState>);
static_assert(
    !HasLegacyPx4ReferenceState<gui::BaselineAutopilotPanelState>);
static_assert(!HasAircraftViewMode<viz::FlightVisualizer>);
static_assert(!HasMutableSimulationSources<viz::FlightVisualizer>);
static_assert(!HasMutableSimulationSources<gui::FlightVizWindow>);

void TestLocalViewStateDefaults() {
  gui::GNCWindow gncWindow;

  assert(gncWindow.GetAutopilotViewState().GetSelection()
         == gui::AutopilotSelection::Primary);
}

void TestBaselineUnavailableIsSafe() {
  gui::AutopilotViewState autopilotView;
  assert(!autopilotView.Select(gui::AutopilotSelection::Baseline, false));
  assert(autopilotView.GetSelection() == gui::AutopilotSelection::Primary);

  viz::FlightVisualizer visualizer;
  assert(visualizer.GetMainSimulation() == nullptr);
  assert(visualizer.GetShadowSimulation() == nullptr);
  visualizer.SetShadowEnabled(true);
  assert(visualizer.IsShadowEnabled());
  assert(!visualizer.Tick());
}

void TestFlightVizWindowsUseIndependentSourcesAndIds() {
  sim::Simulation primary(std::make_unique<gnc::MyAutopilot>());
  sim::Simulation baseline(std::make_unique<gnc::PX4Autopilot>());
  gui::FlightVizWindow primaryWindow(gui::SimulationSlot::Primary,
      &primary,
      &baseline);
  gui::FlightVizWindow baselineWindow(gui::SimulationSlot::Baseline,
      &baseline,
      &primary);

  assert(primaryWindow.GetTitle() == "Flight Viz · Primary");
  assert(primaryWindow.GetWindowId() == "FlightVizPrimary");
  assert(primaryWindow.GetSimulationSlot() == gui::SimulationSlot::Primary);
  assert(primaryWindow.GetMainSimulation() == &primary);
  assert(primaryWindow.GetShadowSimulation() == &baseline);
  assert(baselineWindow.GetTitle() == "Flight Viz · Baseline");
  assert(baselineWindow.GetWindowId() == "FlightVizBaseline");
  assert(baselineWindow.GetSimulationSlot() == gui::SimulationSlot::Baseline);
  assert(baselineWindow.GetMainSimulation() == &baseline);
  assert(baselineWindow.GetShadowSimulation() == &primary);
  assert(&primaryWindow.GetVisualizer() != &baselineWindow.GetVisualizer());

  primaryWindow.GetVisualizer().SetViewMode(viz::ViewMode::ThirdPerson);
  assert(primaryWindow.GetVisualizer().GetViewMode()
         == viz::ViewMode::ThirdPerson);
  assert(baselineWindow.GetVisualizer().GetViewMode() == viz::ViewMode::Orbit);
  primaryWindow.GetVisualizer().SetShadowEnabled(true);
  assert(primaryWindow.GetVisualizer().IsShadowEnabled());
  assert(!baselineWindow.GetVisualizer().IsShadowEnabled());
}

void TestShadowUsesFixedWorldProjection() {
  constexpr double EarthRadiusMeters = 6'371'000.0;
  constexpr double MetersPerVizUnit = 75.0 * 0.3048;
  constexpr double LatitudeOffsetRad = 0.00001;
  constexpr double LongitudeOffsetRad = 0.00002;

  sim::Simulation primary(std::make_unique<gnc::MyAutopilot>());
  sim::Simulation baseline(std::make_unique<gnc::PX4Autopilot>());
  sim::SimulationConfig config{};
  assert(primary.Initialize(config));
  assert(baseline.Initialize(config));

  auto &primaryProperties = primary.GetAircraft().GetProperties();
  auto &baselineProperties = baseline.GetAircraft().GetProperties();
  const double originLatitudeRad = primaryProperties.Latitude().Rad();
  const double originLongitudeRad = primaryProperties.Longitude().Rad();

  baselineProperties.Set("position/lat-gc-rad",
      originLatitudeRad + LatitudeOffsetRad);
  baselineProperties.Set("position/long-gc-rad",
      originLongitudeRad + LongitudeOffsetRad);

  viz::FlightVisualizer visualizer(&primary, &baseline);
  visualizer.SetShadowEnabled(true);
  assert(visualizer.Tick());

  const viz::FrameSnapshot &firstSnapshot = visualizer.GetFrameSnapshot();
  assert(firstSnapshot.shadowEnabled);
  assert(firstSnapshot.shadowAircraft.available);
  const double expectedShadowNorth =
      LatitudeOffsetRad * EarthRadiusMeters / MetersPerVizUnit;
  const double expectedShadowEast = LongitudeOffsetRad
                                    * std::cos(originLatitudeRad)
                                    * EarthRadiusMeters / MetersPerVizUnit;
  assert(std::abs(firstSnapshot.shadowAircraft.position.x - expectedShadowNorth)
         < 0.01);
  assert(std::abs(firstSnapshot.shadowAircraft.position.y - expectedShadowEast)
         < 0.01);

  primaryProperties.Set("position/lat-gc-rad",
      originLatitudeRad + LatitudeOffsetRad * 0.5);
  assert(visualizer.Tick());
  const viz::FrameSnapshot &secondSnapshot = visualizer.GetFrameSnapshot();
  assert(
      std::abs(secondSnapshot.aircraft.position.x - expectedShadowNorth * 0.5)
      < 0.01);
  assert(
      std::abs(secondSnapshot.shadowAircraft.position.x - expectedShadowNorth)
      < 0.01);

  viz::FlightVisualizer primaryOnlyVisualizer(&primary, nullptr);
  primaryOnlyVisualizer.SetShadowEnabled(true);
  assert(primaryOnlyVisualizer.Tick());
  assert(primaryOnlyVisualizer.GetFrameSnapshot().aircraft.available);
  assert(!primaryOnlyVisualizer.GetFrameSnapshot().shadowAircraft.available);
}

void TestComponentSelectionsAreIndependent() {
  gui::GNCWindow gncWindow;
  viz::FlightVisualizer visualizer;
  visualizer.SetViewMode(viz::ViewMode::ThirdPerson);

  assert(gncWindow.GetAutopilotViewState().Select(
      gui::AutopilotSelection::Baseline,
      true));
  assert(visualizer.GetViewMode() == viz::ViewMode::ThirdPerson);
  assert(gncWindow.GetAutopilotViewState().GetSelection()
         == gui::AutopilotSelection::Baseline);
}

void TestBaselineRollHoldStateSurvivesSelectionChanges() {
  gui::AutopilotViewState autopilotView;
  gui::BaselineAutopilotPanelState baselineState;
  baselineState.rollHold = true;
  baselineState.rollTargetDeg = 8.0;
  baselineState.px4RollTimeConstantSec = 0.91;
  baselineState.px4RollTuningOpen = true;
  baselineState.px4RollDiagnosticsOpen = false;

  assert(autopilotView.Select(gui::AutopilotSelection::Baseline, true));
  assert(autopilotView.Select(gui::AutopilotSelection::Primary, true));
  assert(autopilotView.Select(gui::AutopilotSelection::Baseline, true));

  assert(baselineState.rollHold);
  assert(baselineState.rollTargetDeg == 8.0);
  assert(baselineState.px4RollTimeConstantSec == 0.91);
  assert(baselineState.px4RollTuningOpen);
  assert(!baselineState.px4RollDiagnosticsOpen);
}

void TestRollTrackingAcceptanceIsCommandRelative() {
  constexpr double CommandedRollDeg = -7.25;
  constexpr gui::RollTrackingAcceptance acceptance =
      gui::MakeRollTrackingAcceptance(CommandedRollDeg);
  static_assert(acceptance.settlingUpperDeg == -6.75);
  static_assert(acceptance.settlingLowerDeg == -7.75);
  static_assert(acceptance.overshootLimitDeg == -6.25);
  static_assert(acceptance.undershootLimitDeg == -8.25);
}
} // namespace

int main() {
  TestLocalViewStateDefaults();
  TestBaselineUnavailableIsSafe();
  TestFlightVizWindowsUseIndependentSourcesAndIds();
  TestShadowUsesFixedWorldProjection();
  TestComponentSelectionsAreIndependent();
  TestBaselineRollHoldStateSurvivesSelectionChanges();
  TestRollTrackingAcceptanceIsCommandRelative();
  return 0;
}
