#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace gui {
struct MonitorTimeRange {
  double minSec = 0.0;
  double maxSec = 10.0;
};

struct MonitorTimelineState {
  bool live = true;
  bool cursorInitialized = false;
  MonitorTimeRange totalRange;
  std::optional<MonitorTimeRange> historyRange;
  MonitorTimeRange viewRange{0.0, 40.0};
  MonitorTimeRange visibleRange;
  std::optional<MonitorTimeRange> selectedRange;
  std::vector<double> sharedXAxisTicks;
  double viewWindowSec = 40.0;
  double liveWindowSec = 10.0;
  double cursorTimeSec = 0.0;
};

struct MonitorPlotState {
  std::uint64_t id = 0;
  std::string title;
  std::vector<std::string> channels;
  std::string telemetryGroupPath;
  std::string yAxisLabel = "Value";
  std::vector<std::string> hiddenSeries;
  bool custom = false;
  bool showLegend = true;
  bool manualYAxis = false;
  double yAxisMinimum = 0.0;
  double yAxisMaximum = 1.0;
  std::string templateId;
};

struct MonitorPlotProps {
  const MonitorPlotState *plot = nullptr;
  const MonitorTimelineState *timeline = nullptr;
};

enum class MonitorPlotLayout {
  List,
  Grid1x1,
  Grid1x2,
  Grid2x2,
  Grid2x3,
  Grid3x3,
};

struct MonitorPlotLayoutDimensions {
  std::size_t columns = 1;
  std::size_t rows = 1;
};

constexpr MonitorPlotLayoutDimensions GetMonitorPlotLayoutDimensions(
    MonitorPlotLayout layout) {
  switch (layout) {
  case MonitorPlotLayout::Grid1x2:
    return {2, 1};
  case MonitorPlotLayout::Grid2x2:
    return {2, 2};
  case MonitorPlotLayout::Grid2x3:
    return {3, 2};
  case MonitorPlotLayout::Grid3x3:
    return {3, 3};
  case MonitorPlotLayout::List:
  case MonitorPlotLayout::Grid1x1:
  default:
    return {1, 1};
  }
}

constexpr std::size_t GetMonitorPlotSlotCount(MonitorPlotLayout layout) {
  const MonitorPlotLayoutDimensions dimensions =
      GetMonitorPlotLayoutDimensions(layout);
  return dimensions.columns * dimensions.rows;
}

constexpr std::size_t GetMonitorTrailingEmptySlotCount(MonitorPlotLayout layout,
    std::size_t visiblePresetPlotCount) {
  const std::size_t slotCount = GetMonitorPlotSlotCount(layout);
  if (visiblePresetPlotCount == 0) {
    return slotCount;
  }
  const std::size_t occupiedTrailingSlots = visiblePresetPlotCount % slotCount;
  return occupiedTrailingSlots == 0 ? 0 : slotCount - occupiedTrailingSlots;
}

enum class MonitorDisplayMode {
  Baseline,
  Primary,
  Compare,
};

constexpr bool MonitorDisplaysBaseline(MonitorDisplayMode mode) {
  return mode == MonitorDisplayMode::Baseline
         || mode == MonitorDisplayMode::Compare;
}

constexpr bool MonitorDisplaysPrimary(MonitorDisplayMode mode) {
  return mode == MonitorDisplayMode::Primary
         || mode == MonitorDisplayMode::Compare;
}

enum class MonitorTimelineDragMode {
  None,
  Start,
  End,
  Window,
};

enum class MonitorTimelineDragTarget {
  None,
  TimelineView,
  PlotVisible,
};

struct MonitorState {
  // Shared timeline
  MonitorTimelineState timeline;

  // Preset workspace
  std::vector<MonitorPlotState> plots;
  std::array<std::optional<std::uint64_t>, 9> customPlotSlots;
  std::uint64_t nextPlotId = 1;
  MonitorPlotLayout plotLayout = MonitorPlotLayout::Grid2x2;
  MonitorDisplayMode displayMode = MonitorDisplayMode::Compare;
  std::uint32_t activePresetMask = 0;
  bool workspaceInitialized = false;

  // Pane layout
  float presetPaneWidth = 270.0F;
  float timelinePaneHeight = 210.0F;
  bool presetPaneOpen = true;
  bool timelinePaneOpen = true;

  // Timeline interaction
  MonitorTimelineDragMode timelineDragMode = MonitorTimelineDragMode::None;
  MonitorTimelineDragTarget timelineDragTarget =
      MonitorTimelineDragTarget::None;
  MonitorTimeRange timelineDragInitialRange;
  MonitorTimeRange timelineDragAxisRange;
  double timelineDragAnchorSec = 0.0;
  std::optional<double> linearizationTrackSnapTimeSec;

  // Dynamic-mode selection
  std::optional<std::size_t> selectedDynamicModeIndex;
  std::optional<double> selectedDynamicModeSnapshotTimeSec;
};

bool IsValidMonitorManualYAxis(double minimum, double maximum);
std::optional<std::size_t> FindFirstEmptyMonitorPlotSlot(
    const MonitorState &state);
bool AddMonitorPlotToSlot(MonitorState &state, MonitorPlotState plot,
    std::size_t slotIndex);
bool RemoveMonitorPlot(MonitorState &state, std::uint64_t plotId);
} // namespace gui
