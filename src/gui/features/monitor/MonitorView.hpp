#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/monitor/MonitorConfig.hpp"
#include "gui/features/monitor/MonitorEvents.hpp"
#include "gui/features/monitor/MonitorInput.hpp"
#include "gui/features/monitor/MonitorSignalCatalog.hpp"
#include "gui/features/monitor/view/MonitorPlotDialogModel.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace telemetry {
struct TelemetrySnapshot;
}

namespace gui {
class MonitorView {
public:
  explicit MonitorView(MonitorConfig config = {});

  void Render(const MonitorInput &input, const MonitorState &state,
      architecture::EventSink<MonitorEvent> events);

private:
  using TelemetrySources = MonitorInput;
  using MonitorPlot = MonitorPlotState;
  using TimelineRange = MonitorTimeRange;
  using TimelineDragMode = MonitorTimelineDragMode;
  using TimelineDragTarget = MonitorTimelineDragTarget;

  // Workspace setup and plot management
  void InitializePresetWorkspace();
  MonitorPlot *FindPlot(std::uint64_t plotId);
  void RequestAddPlot(std::optional<std::size_t> slotIndex);
  void RequestEditPlot(MonitorPlot &plot);
  void CommitPlotDialog(std::span<const MonitorSignalDescriptor> signalCatalog);
  void DeletePlot(std::uint64_t plotId);
  std::size_t GetAvailablePlotSlotCount() const;
  std::optional<std::size_t> FindFirstEmptyPlotSlot() const;

  // Workspace rendering
  void DrawWindow(const TelemetrySources &sources,
      std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory);
  void DrawDynamicModes(const MonitorDynamicModeInput &dynamicModes);
  void DrawToolbar(const telemetry::TelemetrySnapshot &telemetry);
  void DrawPresetPaneHeader();
  void DrawPresetPanel();
  void DrawPlotWorkspace(const TelemetrySources &sources,
      std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory);
  void DrawPlotScrollRegion(const TelemetrySources &sources);
  void DrawTimelineHeader();
  void DrawTimeline(
      std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory);
  void DrawTimelineOverview(const TimelineRange &historyRange);
  void DrawTimelineDetail();
  void DrawLinearizationTrack(
      std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory);
  void DrawPlotLayoutSelector();
  void DrawDisplayModeSelector();
  void DrawPlotList(const TelemetrySources &sources);
  void DrawPlotGrid(const TelemetrySources &sources, int dimension);
  void DrawPlotTable(const TelemetrySources &sources, int columnCount,
      float plotHeight, const char *tableId);
  void DrawEmptyPlotSlot(std::size_t slotIndex, float plotHeight);
  float CalculateGridPlotHeight(int rowCount) const;
  void DrawPlotCard(MonitorPlot &plot, const TelemetrySources &sources,
      float plotHeight);
  void DrawPlotConfigurationDialog(const TelemetrySources &sources);
  std::vector<MonitorSignalDescriptor> BuildSignalCatalog(
      const TelemetrySources &sources) const;

  // Visibility composition
  bool IsPlotVisible(const MonitorPlot &plot) const;
  bool IsPlotVisibleByPreset(const MonitorPlot &plot) const;
  bool IsPresetActive(std::size_t presetIndex) const;
  void SetPresetActive(std::size_t presetIndex, bool active);

  // Shared viewport and cursor
  std::optional<TimelineRange> GetTelemetryHistoryRange(
      const telemetry::TelemetrySnapshot &telemetry) const;
  void SynchronizeTimelineState(const telemetry::TelemetrySnapshot &telemetry);
  TimelineRange GetEffectiveHistoryRange(
      const TimelineRange &historyRange) const;
  void ClampTimelineViewRangeToHistory();
  void ClampVisibleTimeRangeToHistory();
  void EnsureVisibleTimeRangeInTimelineView();
  void UpdateSharedXAxisTicks();
  void UpdateLiveTimeRanges();
  void SetLiveView(bool enabled);
  void SelectTimelineTime(double timeSec, bool disableLive);
  void ZoomTimelineView(double wheelDelta, double anchorSec);
  std::optional<double> DrawPlotOverlay();

  // Configuration
  MonitorConfig config_;

  // Per-frame render state. The controller remains the authoritative owner.
  MonitorState renderState_;
  architecture::EventSink<MonitorEvent> events_;

  MonitorTimelineState &timelineModel_;
  TimelineRange &timelineViewRange_;
  TimelineRange &visibleTimeRange_;
  std::optional<TimelineRange> &telemetryHistoryRange_;
  std::vector<double> &sharedXAxisTicks_;
  double &timelineViewWindowSec_;
  double &liveWindowSec_;
  double &selectedTimeSec_;
  bool &liveView_;
  bool &selectedTimeInitialized_;

  std::vector<MonitorPlot> &plots_;
  std::uint64_t &nextPlotId_;
  MonitorPlotLayout &plotLayout_;
  MonitorDisplayMode &displayMode_;
  std::uint32_t &activePresetMask_;

  float &presetPaneWidth_;
  float &timelinePaneHeight_;
  bool &presetPaneOpen_;
  bool &timelinePaneOpen_;

  TimelineDragMode &timelineDragMode_;
  TimelineDragTarget &timelineDragTarget_;
  TimelineRange &timelineDragInitialRange_;
  TimelineRange &timelineDragAxisRange_;
  double &timelineDragAnchorSec_;
  std::optional<double> &linearizationTrackSnapTimeSec_;

  std::optional<std::size_t> &selectedDynamicModeIndex_;
  std::optional<double> &selectedDynamicModeSnapshotTimeSec_;

  // View-local interaction state
  MonitorPlotDialogModel plotDialog_;
  std::optional<std::uint64_t> plotToRemove_;
  bool noEmptySlotMessage_ = false;
};
} // namespace gui
