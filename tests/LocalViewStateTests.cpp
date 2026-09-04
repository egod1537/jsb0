#include "gui/panels/AutopilotPanel.hpp"
#include "gui/panels/BaselineAutopilotPanel.hpp"
#include "common/math/Math.hpp"
#include "gui/features/monitor/MonitorConfig.hpp"
#include "gui/features/flightviz/FlightVisualizer.hpp"
#include "gui/windows/GNCWindow.hpp"
#include "gui/features/monitor/plots/CourseTrackingAcceptance.hpp"
#include "gui/features/monitor/plots/PitchTrackingAcceptance.hpp"
#include "gui/features/monitor/plots/RollTrackingAcceptance.hpp"
#include "gui/windows/viz/FlightVizWindow.hpp"
#include "sim/runtime/SimContracts.hpp"

#include <cassert>
#include <cmath>

namespace {
template <typename T>
concept HasBaselineRollHoldTuningState = requires(T &state) {
  state.rollHold;
  state.rollTargetDeg;
  state.px4RollTimeConstantSec;
  state.px4RollTuningOpen;
  state.px4RollDiagnosticsOpen;
  state.pitchHold;
  state.pitchTargetDeg;
  state.px4PitchTimeConstantSec;
  state.px4PitchTuningOpen;
  state.px4PitchDiagnosticsOpen;
};

template <typename T>
concept HasLegacyPx4ReferenceState =
    requires(T &state) { state.legacyPx4ReferenceOpen; };

template <typename T>
concept HasAircraftViewMode = requires(T &visualizer) {
  visualizer.GetAircraftViewMode();
  visualizer.CycleAircraftViewMode();
};

template <typename T>
concept HasMutableSimulationSources = requires(T &view, void *source) {
  view.SetMainSimulation(source);
  view.SetShadowSimulation(source);
};

static_assert(!HasBaselineRollHoldTuningState<gui::AutopilotPanelState>);
static_assert(HasBaselineRollHoldTuningState<gui::BaselineAutopilotPanelState>);
static_assert(!HasLegacyPx4ReferenceState<gui::BaselineAutopilotPanelState>);
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
  visualizer.SetShadowEnabled(true);
  assert(visualizer.IsShadowEnabled());
  assert(!visualizer.Tick(nullptr));
}

void TestFlightVizWindowsUseIndependentSlotsAndIds() {
  gui::FlightVizWindow primaryWindow(sim::SimSlot::Primary);
  gui::FlightVizWindow baselineWindow(sim::SimSlot::Baseline);

  assert(primaryWindow.GetTitle() == "Flight Viz · Primary");
  assert(primaryWindow.GetWindowId() == "FlightVizPrimary");
  assert(primaryWindow.GetSimSlot() == sim::SimSlot::Primary);
  assert(baselineWindow.GetTitle() == "Flight Viz · Baseline");
  assert(baselineWindow.GetWindowId() == "FlightVizBaseline");
  assert(baselineWindow.GetSimSlot() == sim::SimSlot::Baseline);
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
  constexpr double MetersPerVizUnit = math::FeetToMeters(75.0);
  constexpr double LatitudeOffsetRad = 0.00001;
  constexpr double LongitudeOffsetRad = 0.00002;
  constexpr double OriginLatitudeRad = 0.65;
  constexpr double OriginLongitudeRad = 2.2;

  sim::SimInstanceSnapshot primary;
  primary.available = true;
  primary.fdmState.state.latitudeRad = OriginLatitudeRad;
  primary.fdmState.state.longitudeRad = OriginLongitudeRad;
  primary.aircraft.altitudeAslM = math::FeetToMeters(4000.0);

  sim::SimInstanceSnapshot baseline = primary;
  baseline.fdmState.state.latitudeRad += LatitudeOffsetRad;
  baseline.fdmState.state.longitudeRad += LongitudeOffsetRad;

  viz::FlightVisualizer visualizer;
  visualizer.SetShadowEnabled(true);
  assert(visualizer.Tick(&primary, &baseline));

  const viz::FrameSnapshot &firstSnapshot = visualizer.GetFrameSnapshot();
  assert(firstSnapshot.shadowEnabled);
  assert(firstSnapshot.shadowAircraft.available);
  const double expectedShadowNorth =
      LatitudeOffsetRad * EarthRadiusMeters / MetersPerVizUnit;
  const double expectedShadowEast = LongitudeOffsetRad
                                    * std::cos(OriginLatitudeRad)
                                    * EarthRadiusMeters / MetersPerVizUnit;
  assert(std::abs(firstSnapshot.shadowAircraft.position.x - expectedShadowNorth)
         < 0.01);
  assert(std::abs(firstSnapshot.shadowAircraft.position.y - expectedShadowEast)
         < 0.01);

  primary.fdmState.state.latitudeRad += LatitudeOffsetRad * 0.5;
  assert(visualizer.Tick(&primary, &baseline));
  const viz::FrameSnapshot &secondSnapshot = visualizer.GetFrameSnapshot();
  assert(
      std::abs(secondSnapshot.aircraft.position.x - expectedShadowNorth * 0.5)
      < 0.01);
  assert(
      std::abs(secondSnapshot.shadowAircraft.position.x - expectedShadowNorth)
      < 0.01);

  viz::FlightVisualizer primaryOnlyVisualizer;
  primaryOnlyVisualizer.SetShadowEnabled(true);
  assert(primaryOnlyVisualizer.Tick(&primary));
  assert(primaryOnlyVisualizer.GetFrameSnapshot().aircraft.available);
  assert(!primaryOnlyVisualizer.GetFrameSnapshot().shadowAircraft.available);
}

void TestFlightVizPreservesSiAircraftStateAndControls() {
  sim::SimInstanceSnapshot source;
  source.available = true;
  source.aircraft.simulationTimeSec = 1.0;
  source.aircraft.altitudeAglM = 365.76;
  source.aircraft.altitudeAslM = math::FeetToMeters(4000.0);
  source.aircraft.calibratedAirspeedMps = 41.16;
  source.aircraft.trueAirspeedMps = 41.8;
  source.aircraft.rollRad = math::DegToRad(-0.2);
  source.aircraft.pitchRad = math::DegToRad(2.2);
  source.aircraft.headingRad = math::DegToRad(359.0);
  source.aircraft.courseRad = math::DegToRad(1.5);
  source.controlInput = {
      .elevator = 0.02,
      .aileron = -0.11,
      .rudder = 0.01,
      .throttle = 0.64,
  };
  source.fdmState.state.latitudeRad = 0.65;
  source.fdmState.state.longitudeRad = 2.2;

  viz::FlightVisualizer visualizer;
  assert(visualizer.Tick(&source));
  const viz::AircraftSnapshot &captured =
      visualizer.GetFrameSnapshot().aircraft;

  assert(captured.state.altitudeAglM == source.aircraft.altitudeAglM);
  assert(captured.state.altitudeAslM == source.aircraft.altitudeAslM);
  assert(captured.state.calibratedAirspeedMps
         == source.aircraft.calibratedAirspeedMps);
  assert(captured.state.trueAirspeedMps == source.aircraft.trueAirspeedMps);
  assert(captured.state.rollRad == source.aircraft.rollRad);
  assert(captured.state.pitchRad == source.aircraft.pitchRad);
  assert(captured.state.headingRad == source.aircraft.headingRad);
  assert(captured.state.courseRad == source.aircraft.courseRad);
  assert(captured.controlInput.aileron == source.controlInput.aileron);
  assert(captured.controlInput.elevator == source.controlInput.elevator);
  assert(captured.controlInput.rudder == source.controlInput.rudder);
  assert(captured.controlInput.throttle == source.controlInput.throttle);

  const double expectedVisualAltitude =
      source.aircraft.altitudeAglM / math::FeetToMeters(75.0);
  assert(std::abs(captured.visualAltitude - expectedVisualAltitude) < 1.0e-5);
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
  baselineState.courseHold = true;
  baselineState.targetCourseDeg = -179.0;
  baselineState.px4RollTimeConstantSec = 0.91;
  baselineState.px4RollTuningOpen = true;
  baselineState.px4RollDiagnosticsOpen = false;
  baselineState.pitchHold = true;
  baselineState.pitchTargetDeg = 4.0;
  baselineState.px4PitchTimeConstantSec = 0.73;
  baselineState.px4PitchTuningOpen = true;
  baselineState.px4PitchDiagnosticsOpen = false;

  assert(autopilotView.Select(gui::AutopilotSelection::Baseline, true));
  assert(autopilotView.Select(gui::AutopilotSelection::Primary, true));
  assert(autopilotView.Select(gui::AutopilotSelection::Baseline, true));

  assert(baselineState.rollHold);
  assert(baselineState.rollTargetDeg == 8.0);
  assert(baselineState.courseHold);
  assert(baselineState.targetCourseDeg == -179.0);
  assert(baselineState.px4RollTimeConstantSec == 0.91);
  assert(baselineState.px4RollTuningOpen);
  assert(!baselineState.px4RollDiagnosticsOpen);
  assert(baselineState.pitchHold);
  assert(baselineState.pitchTargetDeg == 4.0);
  assert(baselineState.px4PitchTimeConstantSec == 0.73);
  assert(baselineState.px4PitchTuningOpen);
  assert(!baselineState.px4PitchDiagnosticsOpen);
}

void TestRollTrackingAcceptanceIsCommandRelative() {
  constexpr double CommandedRollDeg = -7.25;
  constexpr gui::MonitorConfig Config;
  constexpr gui::RollTrackingAcceptance acceptance =
      gui::MakeRollTrackingAcceptance(CommandedRollDeg,
          0.5,
          Config.rollTrackingToleranceDeg);
  static_assert(acceptance.settlingUpperDeg == -6.75);
  static_assert(acceptance.settlingLowerDeg == -7.75);
  static_assert(acceptance.overshootLimitDeg
                == CommandedRollDeg + Config.rollTrackingToleranceDeg);
  static_assert(acceptance.undershootLimitDeg
                == CommandedRollDeg - Config.rollTrackingToleranceDeg);
}

void TestCourseTrackingToleranceBandIsCommandRelative() {
  constexpr gui::MonitorConfig Config;
  constexpr gui::CourseTrackingToleranceBand band =
      gui::MakeCourseTrackingToleranceBand(5.0,
          Config.courseTrackingToleranceDeg);
  static_assert(band.lowerDeg == 4.0);
  static_assert(band.upperDeg == 6.0);
}

void TestPitchTrackingToleranceBandIsCommandRelative() {
  constexpr gui::MonitorConfig Config;
  constexpr gui::PitchTrackingToleranceBand band =
      gui::MakePitchTrackingToleranceBand(-3.0,
          Config.pitchTrackingToleranceDeg);
  static_assert(band.lowerDeg == -3.1);
  static_assert(band.upperDeg == -2.9);
}

void TestTrackingToleranceConfigDefaults() {
  const gui::MonitorConfig config;
  assert(config.rollTrackingToleranceDeg == 0.1);
  assert(config.pitchTrackingToleranceDeg == 0.1);
  assert(config.courseTrackingToleranceDeg == 1.0);
}
} // namespace

int main() {
  TestLocalViewStateDefaults();
  TestBaselineUnavailableIsSafe();
  TestFlightVizWindowsUseIndependentSlotsAndIds();
  TestShadowUsesFixedWorldProjection();
  TestFlightVizPreservesSiAircraftStateAndControls();
  TestComponentSelectionsAreIndependent();
  TestBaselineRollHoldStateSurvivesSelectionChanges();
  TestRollTrackingAcceptanceIsCommandRelative();
  TestCourseTrackingToleranceBandIsCommandRelative();
  TestPitchTrackingToleranceBandIsCommandRelative();
  TestTrackingToleranceConfigDefaults();
  return 0;
}
