#pragma once

#include "application/gui/Window.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace telemetry {
class TelemetryRegistry;
}

namespace sim {
class Simulation;
}

namespace gnc {
class DynamicModeHistory;
}

namespace gui {
class FlightDataMonitorWindow final : public gui::Window {
public:
  FlightDataMonitorWindow();

protected:
  ImGuiWindowFlags GetWindowFlags() const override;
  void OnRender(gui::GUI &gui) override;

private:
  struct TelemetrySources {
    const telemetry::TelemetryRegistry *primary = nullptr;
    const telemetry::TelemetryRegistry *baseline = nullptr;
  };

  struct MonitorPlot {
    std::uint64_t id = 0;
    std::string title;
    std::vector<std::string> channels;
    std::string telemetryGroupPath;
    std::string yAxisLabel = "Value";
    bool manualVisible = true;
  };

  struct TimelineRange {
    double minSec = 0.0;
    double maxSec = 10.0;
  };

  struct BrowserNode {
    std::string name;
    std::string fullPath;
    std::map<std::string, BrowserNode, std::less<>> children;
    bool isChannel = false;
  };

  enum class MonitorPlotLayout {
    List,
    Grid2x2,
    Grid3x3,
  };

  enum class TimelineDragMode {
    None,
    Start,
    End,
    Window,
  };

  enum class TimelineDragTarget {
    None,
    TimelineView,
    PlotVisible,
  };

  // Workspace setup and plot management
  TelemetrySources ResolveTelemetrySources(const gui::GUI &gui) const;
  void CreateDefaultPreset();
  MonitorPlot &CreatePlot(std::string title,
      std::string telemetryGroupPath = {}, std::string yAxisLabel = "Value");
  MonitorPlot *FindBoundPlot(std::string_view telemetryNodePath);
  void DeletePlot(std::uint64_t plotId);
  void SetChannelEnabled(MonitorPlot &plot, std::string_view channelPath,
      bool enabled);

  // Workspace rendering
  void DrawWindow(const TelemetrySources &sources,
      const gnc::DynamicModeHistory *dynamicModeHistory);
  void DrawDynamicModes(sim::Simulation &simulation);
  void DrawToolbar(const telemetry::TelemetryRegistry &telemetryRegistry);
  void DrawExplorerHeader();
  void DrawTelemetryBrowser(
      const telemetry::TelemetryRegistry &telemetryRegistry);
  void AddBrowserPath(BrowserNode &root, std::string_view path) const;
  void DrawBrowserNode(const BrowserNode &node, bool expandAll);
  void DrawBrowserChannel(std::string_view label, std::string_view channelPath);
  void DrawPresetPanel();
  void DrawPlotWorkspace(const TelemetrySources &sources,
      const gnc::DynamicModeHistory *dynamicModeHistory);
  void DrawPlotScrollRegion(const TelemetrySources &sources);
  void DrawTimelineHeader();
  void DrawTimeline(const gnc::DynamicModeHistory *dynamicModeHistory);
  void DrawTimelineOverview(const TimelineRange &historyRange);
  void DrawTimelineDetail();
  void DrawLinearizationTrack(
      const gnc::DynamicModeHistory *dynamicModeHistory);
  void DrawPlotLayoutSelector();
  void DrawPlotList(const TelemetrySources &sources);
  void DrawPlotGrid(const TelemetrySources &sources, int dimension);
  void DrawPlotTable(const TelemetrySources &sources, int columnCount,
      float plotHeight, const char *tableId);
  float CalculateGridPlotHeight(int rowCount) const;
  bool DrawPlotCard(MonitorPlot &plot, const TelemetrySources &sources,
      float plotHeight);
  void DrawTelemetryPlot(const MonitorPlot &plot,
      const TelemetrySources &sources, float plotHeight);
  void DrawRollTrackingAcceptanceUnderlay(const MonitorPlot &plot,
      const telemetry::TelemetryRegistry &telemetryRegistry,
      std::size_t maximumRenderedSampleCount) const;

  // Visibility composition
  bool IsPlotVisible(const MonitorPlot &plot) const;
  bool IsPlotVisibleByPreset(const MonitorPlot &plot) const;
  bool IsPresetActive(std::size_t presetIndex) const;
  void SetPresetActive(std::size_t presetIndex, bool active);

  // Shared viewport and cursor
  std::optional<TimelineRange> GetTelemetryHistoryRange(
      const telemetry::TelemetryRegistry &telemetryRegistry) const;
  void SynchronizeTimelineState(
      const telemetry::TelemetryRegistry &telemetryRegistry);
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
  void DrawPlotOverlay(const MonitorPlot &plot,
      const telemetry::TelemetryRegistry &telemetryRegistry);

  // Plot workspace state
  std::vector<MonitorPlot> plots_;
  std::uint64_t nextPlotId_ = 1;
  std::uint64_t selectedPlotId_ = 0;
  MonitorPlotLayout plotLayout_ = MonitorPlotLayout::Grid2x2;
  std::uint32_t activePresetMask_ = 0;

  // Pane layout state
  float explorerPaneWidth_ = 270.0F;
  float timelinePaneHeight_ = 210.0F;
  bool explorerPaneOpen_ = true;
  bool timelinePaneOpen_ = true;

  // Telemetry browser state
  std::array<char, 128> channelSearch_{};
  std::string selectedChannelPath_;

  // Shared viewport state
  TimelineRange timelineViewRange_{0.0, 40.0};
  TimelineRange visibleTimeRange_;
  std::optional<TimelineRange> telemetryHistoryRange_;
  std::vector<double> sharedXAxisTicks_;
  double timelineViewWindowSec_ = 40.0;
  double liveWindowSec_ = 10.0;
  bool liveView_ = true;

  // Timeline interaction state
  TimelineDragMode timelineDragMode_ = TimelineDragMode::None;
  TimelineDragTarget timelineDragTarget_ = TimelineDragTarget::None;
  TimelineRange timelineDragInitialRange_;
  TimelineRange timelineDragAxisRange_;
  double timelineDragAnchorSec_ = 0.0;
  std::optional<double> linearizationTrackSnapTimeSec_;

  // Shared time selection
  double selectedTimeSec_ = 0.0;
  bool selectedTimeInitialized_ = false;

  // Dynamic-mode inspection state
  std::optional<std::size_t> selectedDynamicModeIndex_;
  std::optional<double> selectedDynamicModeSnapshotTimeSec_;
};
} // namespace gui
