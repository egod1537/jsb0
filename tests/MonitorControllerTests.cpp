#include "gui/features/monitor/MonitorController.hpp"
#include "gui/features/monitor/MonitorSignalCatalog.hpp"
#include "gui/features/monitor/catalog/MonitorPlotPresetCatalog.hpp"
#include "gui/features/monitor/view/MonitorPlotDialogModel.hpp"
#include "sim/linearization/DynamicModeContracts.hpp"
#include "sim/telemetry/AutopilotTelemetry.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"

#include <cassert>
#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <set>
#include <string_view>

namespace {
constexpr double Tolerance = 1.0e-9;

void RequireNear(double actual, double expected) {
  assert(std::abs(actual - expected) <= Tolerance);
}

gui::MonitorController MakeControllerWithRange(double minimum, double maximum) {
  gui::MonitorController controller;
  controller.Handle(gui::MonitorTelemetryRangeChanged{{minimum, maximum}});
  return controller;
}

void TestLiveTelemetryExtendsSharedRanges() {
  gui::MonitorController controller = MakeControllerWithRange(0.0, 100.0);
  const gui::MonitorTimelineState &timeline = controller.GetState().timeline;
  assert(timeline.live);
  RequireNear(timeline.totalRange.maxSec, 100.0);
  RequireNear(timeline.viewRange.minSec, 60.0);
  RequireNear(timeline.viewRange.maxSec, 100.0);
  RequireNear(timeline.visibleRange.minSec, 90.0);
  RequireNear(timeline.visibleRange.maxSec, 100.0);
  RequireNear(timeline.cursorTimeSec, 100.0);
}

void TestDisplayModeDefaultsToCompare() {
  gui::MonitorController controller;
  assert(controller.GetState().displayMode == gui::MonitorDisplayMode::Compare);
  static_assert(gui::MonitorDisplaysBaseline(gui::MonitorDisplayMode::Compare));
  static_assert(gui::MonitorDisplaysPrimary(gui::MonitorDisplayMode::Compare));
  static_assert(
      gui::MonitorDisplaysBaseline(gui::MonitorDisplayMode::Baseline));
  static_assert(
      !gui::MonitorDisplaysPrimary(gui::MonitorDisplayMode::Baseline));
  static_assert(
      !gui::MonitorDisplaysBaseline(gui::MonitorDisplayMode::Primary));
  static_assert(gui::MonitorDisplaysPrimary(gui::MonitorDisplayMode::Primary));

  gui::MonitorState state = controller.GetState();
  state.displayMode = gui::MonitorDisplayMode::Baseline;
  controller.Handle(gui::MonitorStateChanged{state});
  assert(
      controller.GetState().displayMode == gui::MonitorDisplayMode::Baseline);
}

void TestDisablingLiveFreezesExpectedRange() {
  gui::MonitorController controller = MakeControllerWithRange(0.0, 100.0);
  controller.Handle(gui::MonitorLiveChanged{false});
  const gui::MonitorTimeRange frozen =
      controller.GetState().timeline.visibleRange;

  controller.Handle(gui::MonitorTelemetryRangeChanged{{0.0, 120.0}});

  assert(!controller.GetState().timeline.live);
  RequireNear(controller.GetState().timeline.visibleRange.minSec,
      frozen.minSec);
  RequireNear(controller.GetState().timeline.visibleRange.maxSec,
      frozen.maxSec);
  RequireNear(controller.GetState().timeline.totalRange.maxSec, 120.0);
}

void TestZoomUpdatesSharedTimeline() {
  gui::MonitorController controller = MakeControllerWithRange(0.0, 100.0);
  controller.Handle(gui::MonitorLiveChanged{false});
  const double oldDuration = controller.GetState().timeline.viewRange.maxSec
                             - controller.GetState().timeline.viewRange.minSec;

  controller.Handle(gui::MonitorZoomRequested{1.0, 80.0});

  const gui::MonitorTimelineState &timeline = controller.GetState().timeline;
  const double newDuration =
      timeline.viewRange.maxSec - timeline.viewRange.minSec;
  assert(newDuration < oldDuration);
  assert(newDuration
         >= timeline.visibleRange.maxSec - timeline.visibleRange.minSec);
}

void TestPanUpdatesSharedRanges() {
  gui::MonitorController controller = MakeControllerWithRange(0.0, 200.0);
  controller.Handle(gui::MonitorLiveChanged{false});
  controller.Handle(gui::MonitorViewRangeChanged{{50.0, 90.0}});
  controller.Handle(gui::MonitorVisibleRangeChanged{{60.0, 70.0}});

  controller.Handle(gui::MonitorPanRequested{5.0});

  RequireNear(controller.GetState().timeline.viewRange.minSec, 55.0);
  RequireNear(controller.GetState().timeline.viewRange.maxSec, 95.0);
  RequireNear(controller.GetState().timeline.visibleRange.minSec, 65.0);
  RequireNear(controller.GetState().timeline.visibleRange.maxSec, 75.0);
}

void TestCursorMovementPropagatesThroughController() {
  gui::MonitorController controller = MakeControllerWithRange(0.0, 100.0);
  controller.Handle(gui::MonitorCursorMoved{42.5});
  assert(controller.GetState().timeline.cursorInitialized);
  RequireNear(controller.GetState().timeline.cursorTimeSec, 42.5);
}

void TestEveryPlotReceivesTheSameTimelineProps() {
  gui::MonitorController controller;
  gui::MonitorState state = controller.GetState();
  state.plots.push_back({.id = 1,
      .title = "Roll",
      .hiddenSeries = {"Primary/aircraft/rates/p"},
      .showLegend = false,
      .manualYAxis = true,
      .yAxisMinimum = -10.0,
      .yAxisMaximum = 10.0});
  state.plots.push_back({.id = 2, .title = "Pitch"});
  controller.Handle(gui::MonitorStateChanged{state});

  const std::vector<gui::MonitorPlotProps> props = controller.BuildPlotProps();
  assert(props.size() == 2);
  assert(props[0].timeline == props[1].timeline);
  assert(props[0].timeline == &controller.GetState().timeline);
  assert(props[0].plot->hiddenSeries
         == std::vector<std::string>{"Primary/aircraft/rates/p"});
  assert(!props[0].plot->showLegend);
  assert(props[0].plot->manualYAxis);
  RequireNear(props[0].plot->yAxisMinimum, -10.0);
  RequireNear(props[0].plot->yAxisMaximum, 10.0);
}

void TestInputUsesProvidedSnapshotDataOnly() {
  auto telemetry = std::make_shared<telemetry::TelemetrySnapshot>();
  telemetry->available = true;
  telemetry->publishedTimeRange = telemetry::TelemetryTimeRange{2.0, 25.0};
  const std::vector<gnc::DynamicModeSnapshot> dynamicModes(1);
  gui::MonitorController controller;

  controller.SetInput({
      .primary = telemetry,
      .dynamicModes = {.history = dynamicModes, .available = true},
  });

  assert(controller.GetInput().primary == telemetry);
  assert(
      controller.GetInput().dynamicModes.history.data() == dynamicModes.data());
  RequireNear(controller.GetState().timeline.totalRange.minSec, 0.0);
  RequireNear(controller.GetState().timeline.totalRange.maxSec, 25.0);
}

void TestApplicationIntentIsEmittedUpward() {
  bool enabled = false;
  gui::MonitorController controller(
      gui::architecture::EventSink<gui::MonitorAutomaticLinearizationChanged>{
          [&enabled](const auto &event) { enabled = event.enabled; }});

  controller.Handle(gui::MonitorAutomaticLinearizationChanged{true});

  assert(enabled);
}

void TestPlotLayoutsExposeExpectedSlots() {
  using gui::MonitorPlotLayout;
  assert(gui::GetMonitorPlotSlotCount(MonitorPlotLayout::Grid1x1) == 1);
  assert(gui::GetMonitorPlotSlotCount(MonitorPlotLayout::Grid1x2) == 2);
  assert(gui::GetMonitorPlotSlotCount(MonitorPlotLayout::Grid2x2) == 4);
  assert(gui::GetMonitorPlotSlotCount(MonitorPlotLayout::Grid2x3) == 6);
  assert(gui::GetMonitorPlotSlotCount(MonitorPlotLayout::Grid3x3) == 9);
  assert(gui::GetMonitorTrailingEmptySlotCount(MonitorPlotLayout::Grid2x2, 0)
         == 4);
  assert(gui::GetMonitorTrailingEmptySlotCount(MonitorPlotLayout::Grid2x2, 3)
         == 1);
  assert(gui::GetMonitorTrailingEmptySlotCount(MonitorPlotLayout::Grid2x2, 4)
         == 0);
  assert(gui::GetMonitorTrailingEmptySlotCount(MonitorPlotLayout::Grid3x3, 8)
         == 1);
}

void TestCustomPlotSlotInsertionAndRemovalAreDeterministic() {
  gui::MonitorState state;
  state.plotLayout = gui::MonitorPlotLayout::Grid2x2;
  assert(gui::FindFirstEmptyMonitorPlotSlot(state) == 0);

  gui::MonitorPlotState clickedSlotPlot{.title = "Roll",
      .channels = {"aircraft/attitude/roll"}};
  assert(gui::AddMonitorPlotToSlot(state, clickedSlotPlot, 3));
  assert(state.customPlotSlots[3].has_value());
  assert(gui::FindFirstEmptyMonitorPlotSlot(state) == 0);

  for (std::size_t slot = 0; slot < 3; ++slot) {
    gui::MonitorPlotState toolbarPlot{.title = "Plot",
        .channels = {"aircraft/rates/p"}};
    assert(gui::AddMonitorPlotToSlot(state, toolbarPlot, slot));
  }
  assert(!gui::FindFirstEmptyMonitorPlotSlot(state).has_value());

  const std::uint64_t removedId = *state.customPlotSlots[1];
  assert(gui::RemoveMonitorPlot(state, removedId));
  assert(!state.customPlotSlots[1].has_value());
  assert(gui::FindFirstEmptyMonitorPlotSlot(state) == 1);
}

void TestCustomPlotRejectsOccupiedAndOutOfLayoutSlots() {
  gui::MonitorState state;
  state.plotLayout = gui::MonitorPlotLayout::Grid1x2;
  const gui::MonitorPlotState plot{.title = "Roll",
      .channels = {"aircraft/attitude/roll"}};
  assert(gui::AddMonitorPlotToSlot(state, plot, 1));
  assert(!gui::AddMonitorPlotToSlot(state, plot, 1));
  assert(!gui::AddMonitorPlotToSlot(state, plot, 2));
  assert(state.plots.size() == 1);

  state.plotLayout = gui::MonitorPlotLayout::Grid1x1;
  assert(gui::FindFirstEmptyMonitorPlotSlot(state) == 0);
  state.plotLayout = gui::MonitorPlotLayout::Grid1x2;
  assert(gui::FindFirstEmptyMonitorPlotSlot(state) == 0);
}

void TestPresetCatalogBuildsStableTemplatesAndMasks() {
  const std::vector<gui::MonitorPlotState> templates =
      gui::BuildDefaultMonitorPlotTemplates();
  assert(templates.size()
         == static_cast<std::size_t>(gui::DefaultTelemetryPlot::Count));

  std::set<std::string> templatePaths;
  for (const gui::MonitorPlotState &plot : templates) {
    assert(!plot.custom);
    assert(!plot.title.empty());
    assert(!plot.telemetryGroupPath.empty());
    assert(!plot.channels.empty());
    assert(templatePaths.insert(plot.telemetryGroupPath).second);
  }

  const auto presets = gui::GetMonitorPresetDefinitions();
  assert(presets.size() == static_cast<std::size_t>(gui::MonitorPreset::Count));
  const std::uint32_t allMask = gui::GetAllMonitorPresetMask();
  for (std::size_t index = 0; index < presets.size(); ++index) {
    assert(gui::IsMonitorPresetActive(allMask, index));
  }
  assert(!gui::IsMonitorPresetActive(allMask, presets.size()));

  const std::uint32_t tecsMask = gui::GetPresetBit(gui::MonitorPreset::Px4Tecs);
  assert(gui::IsMonitorPlotVisibleByPreset("preset/px4_tecs/energy", tecsMask));
  assert(!gui::IsMonitorPlotVisibleByPreset("preset/roll_hold/roll_tracking",
      tecsMask));
}

void TestPlotDialogModelSeparatesAddEditAndRuntimeReset() {
  gui::MonitorPlotDialogModel dialog;
  dialog.BeginAdd(2);
  assert(dialog.openRequested);
  assert(dialog.targetSlot == 2);
  assert(!dialog.editingPlotId.has_value());
  assert(dialog.selectedSignalIds.empty());
  assert(!dialog.manualYAxis);
  assert(dialog.showLegend);

  const gui::MonitorPlotState plot{.id = 17,
      .title = "Pitch tracking",
      .channels = {"autopilot/pitch_hold/pitch"},
      .showLegend = false,
      .manualYAxis = true,
      .yAxisMinimum = -0.2,
      .yAxisMaximum = 0.2,
      .templateId = "preset/px4_pitch_hold/pitch_tracking"};
  dialog.BeginEdit(plot);
  assert(dialog.editingPlotId == 17);
  assert(!dialog.targetSlot.has_value());
  assert(std::string(dialog.title.data()) == plot.title);
  assert(dialog.selectedSignalIds == plot.channels);
  assert(dialog.selectedTemplateId == plot.templateId);
  assert(dialog.manualYAxis);
  assert(!dialog.showLegend);
  RequireNear(dialog.yAxisMinimum, -0.2);
  RequireNear(dialog.yAxisMaximum, 0.2);

  dialog.focusSearch = true;
  dialog.Close();
  assert(!dialog.editingPlotId.has_value());
  assert(!dialog.targetSlot.has_value());
  assert(!dialog.focusSearch);
}

void TestPlotYAxisValidation() {
  assert(gui::IsValidMonitorManualYAxis(-1.0, 1.0));
  assert(!gui::IsValidMonitorManualYAxis(1.0, 1.0));
  assert(!gui::IsValidMonitorManualYAxis(2.0, 1.0));
  assert(
      !gui::IsValidMonitorManualYAxis(std::numeric_limits<double>::quiet_NaN(),
          1.0));

  gui::MonitorState state;
  gui::MonitorPlotState invalidPlot{.title = "Invalid",
      .channels = {"aircraft/rates/p"},
      .manualYAxis = true,
      .yAxisMinimum = 1.0,
      .yAxisMaximum = 1.0};
  assert(!gui::AddMonitorPlotToSlot(state, invalidPlot, 0));
}

void TestSignalCatalogMetadataSearchAndTitleFallback() {
  const std::vector<gui::MonitorPlotState> templates{
      {.title = "Roll",
          .channels = {"autopilot/roll_hold/commanded_roll",
              "aircraft/attitude/roll"},
          .telemetryGroupPath = "preset/roll",
          .yAxisLabel = "rad"},
      {.title = "Roll Rate",
          .channels = {"autopilot/roll_hold/commanded_roll_rate",
              "autopilot/roll_hold/roll_rate"},
          .telemetryGroupPath = "preset/roll_rate",
          .yAxisLabel = "rad/s"}};
  const std::vector<std::string_view> paths{
      "autopilot/roll_hold/commanded_roll",
      "aircraft/attitude/roll",
      "autopilot/roll_hold/commanded_roll_rate",
      "autopilot/roll_hold/roll_rate"};
  const std::vector<gui::MonitorSignalDescriptor> catalog =
      gui::BuildMonitorSignalCatalog(paths, templates);
  assert(catalog.size() == 4);
  assert(gui::FilterMonitorSignalCatalog(catalog, "roll").size() == 4);
  assert(
      gui::FilterMonitorSignalCatalog(catalog, "commanded_roll").size() == 2);
  assert(gui::FilterMonitorSignalCatalog(catalog, "rad/s").size() == 2);
  assert(gui::FilterMonitorSignalCatalog(catalog, "\xCF\x86").size() == 1);

  const std::vector<std::string> selection{"aircraft/attitude/roll",
      "autopilot/roll_hold/commanded_roll"};
  const auto filtered = gui::FilterMonitorSignalCatalog(catalog, "rad/s");
  assert(filtered.size() == 2);
  assert(selection.size() == 2);
  assert(gui::CanAddMonitorSignal(catalog,
      selection,
      "autopilot/roll_hold/commanded_roll"));
  assert(!gui::CanAddMonitorSignal(catalog,
      selection,
      "autopilot/roll_hold/roll_rate"));
  assert(gui::ResolveMonitorPlotTitle("", selection, catalog) == "Roll");
  assert(gui::ResolveMonitorPlotTitle("   ", selection, catalog) == "Roll");
  assert(gui::ResolveMonitorPlotTitle("Lateral attitude", selection, catalog)
         == "Lateral attitude");
}

void TestPitchHoldSignalCatalogMetadata() {
  const std::vector<gui::MonitorPlotState> templates{
      {.title = "Pitch Tracking",
          .channels = {"autopilot/pitch_hold/commanded_pitch",
              "autopilot/pitch_hold/pitch"},
          .telemetryGroupPath = "preset/px4_pitch_hold/pitch_tracking",
          .yAxisLabel = "rad"},
      {.title = "Pitch Rate Tracking",
          .channels = {"autopilot/pitch_hold/commanded_pitch_rate",
              "autopilot/pitch_hold/pitch_rate"},
          .telemetryGroupPath = "preset/px4_pitch_hold/pitch_rate_tracking",
          .yAxisLabel = "rad/s"},
      {.title = "Elevator",
          .channels = {"autopilot/pitch_hold/elevator_command",
              "autopilot/pitch_hold/trim_elevator_command",
              "aircraft/control/elevator"},
          .telemetryGroupPath = "preset/px4_pitch_hold/elevator",
          .yAxisLabel = "normalized"}};
  const std::vector<std::string_view> paths{
      "autopilot/pitch_hold/commanded_pitch",
      "autopilot/pitch_hold/pitch",
      "autopilot/pitch_hold/commanded_pitch_rate",
      "autopilot/pitch_hold/pitch_rate",
      "autopilot/pitch_hold/elevator_command",
      "autopilot/pitch_hold/trim_elevator_command",
      "aircraft/control/elevator"};
  const std::vector<gui::MonitorSignalDescriptor> catalog =
      gui::BuildMonitorSignalCatalog(paths, templates);

  assert(gui::FindMonitorSignal(catalog, "autopilot/pitch_hold/commanded_pitch")
             ->name
         == "Commanded Pitch");
  assert(gui::FindMonitorSignal(catalog,
             "autopilot/pitch_hold/commanded_pitch_rate")
             ->name
         == "Commanded Pitch Rate");
  assert(
      gui::FindMonitorSignal(catalog, "autopilot/pitch_hold/elevator_command")
          ->name
      == "Pitch Hold Elevator Command");
  assert(gui::FindMonitorSignal(catalog,
             "autopilot/pitch_hold/trim_elevator_command")
             ->name
         == "Elevator Trim");
  assert(gui::FilterMonitorSignalCatalog(catalog, "pitch").size() == 6);
  assert(gui::FilterMonitorSignalCatalog(catalog, "elevator").size() == 3);
}

void TestTecsSignalCatalogMetadata() {
  const std::vector<gui::MonitorPlotState> templates{
      {.title = "TECS Altitude",
          .channels = {std::string(
                           telemetry::paths::AutopilotTecsTargetAltitude),
              std::string(
                  telemetry::paths::AutopilotTecsInternalAltitudeSetpoint),
              std::string(telemetry::paths::AutopilotTecsAltitude)},
          .telemetryGroupPath = "preset/px4_tecs/altitude",
          .yAxisLabel = "m"},
      {.title = "TECS Energy",
          .channels = {"autopilot/tecs/total_energy_error",
              "autopilot/tecs/energy_balance_error"},
          .telemetryGroupPath = "preset/px4_tecs/energy",
          .yAxisLabel = "m^2/s^2"}};
  const std::vector<std::string_view> paths{
      telemetry::paths::AutopilotTecsTargetAltitude,
      telemetry::paths::AutopilotTecsInternalAltitudeSetpoint,
      telemetry::paths::AutopilotTecsAltitude,
      "autopilot/tecs/total_energy_error",
      "autopilot/tecs/energy_balance_error",
      "autopilot/tecs/underspeed_protection"};
  const auto catalog = gui::BuildMonitorSignalCatalog(paths, templates);
  const auto *targetAltitude = gui::FindMonitorSignal(catalog,
      telemetry::paths::AutopilotTecsTargetAltitude);
  assert(targetAltitude->name == "Target Altitude AGL");
  assert(targetAltitude->unit == "m");
  assert(gui::FindMonitorSignal(catalog,
             telemetry::paths::AutopilotTecsInternalAltitudeSetpoint)
             ->name
         == "Internal Altitude Setpoint AGL");
  assert(
      gui::FindMonitorSignal(catalog, telemetry::paths::AutopilotTecsAltitude)
          ->unit
      == "m");
  const std::vector<std::string_view> airspeedPaths{
      telemetry::paths::AutopilotTecsTargetAirspeed,
      telemetry::paths::AutopilotTecsAirspeed};
  const auto airspeedCatalog =
      gui::BuildMonitorSignalCatalog(airspeedPaths, {});
  assert(gui::FindMonitorSignal(airspeedCatalog,
             telemetry::paths::AutopilotTecsTargetAirspeed)
             ->name
         == "Target Airspeed CAS");
  assert(gui::FindMonitorSignal(airspeedCatalog,
             telemetry::paths::AutopilotTecsTargetAirspeed)
             ->unit
         == "m/s");
  const std::vector<std::string_view> commandPaths{
      telemetry::paths::AutopilotTecsTargetPitch,
      telemetry::paths::AutopilotTecsTargetThrottle};
  const auto commandCatalog = gui::BuildMonitorSignalCatalog(commandPaths, {});
  assert(gui::FindMonitorSignal(commandCatalog,
             telemetry::paths::AutopilotTecsTargetPitch)
             ->unit
         == "rad");
  assert(gui::FindMonitorSignal(commandCatalog,
             telemetry::paths::AutopilotTecsTargetThrottle)
             ->unit
         == "normalized");
  assert(gui::FindMonitorSignal(catalog, "autopilot/tecs/underspeed_protection")
             ->name
         == "Underspeed Protection");
  assert(gui::FilterMonitorSignalCatalog(catalog, "energy").size() == 2);
}
} // namespace

int main() {
  TestDisplayModeDefaultsToCompare();
  TestLiveTelemetryExtendsSharedRanges();
  TestDisablingLiveFreezesExpectedRange();
  TestZoomUpdatesSharedTimeline();
  TestPanUpdatesSharedRanges();
  TestCursorMovementPropagatesThroughController();
  TestEveryPlotReceivesTheSameTimelineProps();
  TestInputUsesProvidedSnapshotDataOnly();
  TestApplicationIntentIsEmittedUpward();
  TestPlotLayoutsExposeExpectedSlots();
  TestCustomPlotSlotInsertionAndRemovalAreDeterministic();
  TestCustomPlotRejectsOccupiedAndOutOfLayoutSlots();
  TestPresetCatalogBuildsStableTemplatesAndMasks();
  TestPlotDialogModelSeparatesAddEditAndRuntimeReset();
  TestPlotYAxisValidation();
  TestSignalCatalogMetadataSearchAndTitleFallback();
  TestPitchHoldSignalCatalogMetadata();
  TestTecsSignalCatalogMetadata();
  return 0;
}
