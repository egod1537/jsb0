#include "gui/features/monitor/MonitorView.hpp"

#include "gui/features/monitor/catalog/MonitorPlotPresetCatalog.hpp"

#include "sim/linearization/DynamicModeContracts.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"
#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace gui {

namespace {
constexpr float PresetPaneMinWidth = 180.0F;
constexpr float PresetPaneMaxWidth = 600.0F;
constexpr float PresetPaneMinimumPlotWidth = 320.0F;
constexpr float PresetPaneCollapsedWidth = 30.0F;
constexpr float PaneSplitterThickness = 6.0F;
constexpr float TimelineMinHeight = 200.0F;
constexpr float TimelineMaxHeight = 300.0F;
constexpr float TimelineCollapsedHeight = 32.0F;

template <typename T>
T ClampToOrderedRange(T value, T firstBound, T secondBound) {
  const T minimum = std::min(firstBound, secondBound);
  const T maximum = std::max(firstBound, secondBound);
  return std::min(std::max(value, minimum), maximum);
}

void DrawVerticalPaneSplitter(const char *id, float height, float &sizeLogical,
    float minLogical, float maxLogical) {
  const float splitterWidth = ui::Ui(PaneSplitterThickness);
  const ImVec2 splitterMin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id,
      ImVec2(splitterWidth, std::max(height, 1.0F)),
      ImGuiButtonFlags_MouseButtonLeft);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  if (hovered || active) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
  }
  if (active) {
    const float uiScale = std::max(ui::GetUIScale(), 0.001F);
    sizeLogical =
        ClampToOrderedRange(sizeLogical + ImGui::GetIO().MouseDelta.x / uiScale,
            minLogical,
            maxLogical);
  }

  const ImU32 color = ImGui::GetColorU32(active    ? ImGuiCol_SeparatorActive
                                         : hovered ? ImGuiCol_SeparatorHovered
                                                   : ImGuiCol_Separator);
  const float centerX = splitterMin.x + splitterWidth * 0.5F;
  ImGui::GetWindowDrawList()->AddLine(ImVec2(centerX, splitterMin.y),
      ImVec2(centerX, splitterMin.y + height),
      color,
      active || hovered ? ui::Ui(2.0F) : ui::Ui(1.0F));
}

void DrawHorizontalPaneSplitter(const char *id, float width,
    float &bottomSizeLogical, float minLogical, float maxLogical) {
  const float splitterHeight = ui::Ui(PaneSplitterThickness);
  const ImVec2 splitterMin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id,
      ImVec2(std::max(width, 1.0F), splitterHeight),
      ImGuiButtonFlags_MouseButtonLeft);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  if (hovered || active) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
  }
  if (active) {
    const float uiScale = std::max(ui::GetUIScale(), 0.001F);
    bottomSizeLogical = ClampToOrderedRange(
        bottomSizeLogical - ImGui::GetIO().MouseDelta.y / uiScale,
        minLogical,
        maxLogical);
  }

  const ImU32 color = ImGui::GetColorU32(active    ? ImGuiCol_SeparatorActive
                                         : hovered ? ImGuiCol_SeparatorHovered
                                                   : ImGuiCol_Separator);
  const float centerY = splitterMin.y + splitterHeight * 0.5F;
  ImGui::GetWindowDrawList()->AddLine(ImVec2(splitterMin.x, centerY),
      ImVec2(splitterMin.x + width, centerY),
      color,
      active || hovered ? ui::Ui(2.0F) : ui::Ui(1.0F));
}
} // namespace

MonitorView::MonitorView(MonitorConfig config)
    : config_(std::move(config)), timelineModel_(renderState_.timeline),
      timelineViewRange_(timelineModel_.viewRange),
      visibleTimeRange_(timelineModel_.visibleRange),
      telemetryHistoryRange_(timelineModel_.historyRange),
      sharedXAxisTicks_(timelineModel_.sharedXAxisTicks),
      timelineViewWindowSec_(timelineModel_.viewWindowSec),
      liveWindowSec_(timelineModel_.liveWindowSec),
      selectedTimeSec_(timelineModel_.cursorTimeSec),
      liveView_(timelineModel_.live),
      selectedTimeInitialized_(timelineModel_.cursorInitialized),
      plots_(renderState_.plots), nextPlotId_(renderState_.nextPlotId),
      plotLayout_(renderState_.plotLayout),
      displayMode_(renderState_.displayMode),
      activePresetMask_(renderState_.activePresetMask),
      presetPaneWidth_(renderState_.presetPaneWidth),
      timelinePaneHeight_(renderState_.timelinePaneHeight),
      presetPaneOpen_(renderState_.presetPaneOpen),
      timelinePaneOpen_(renderState_.timelinePaneOpen),
      timelineDragMode_(renderState_.timelineDragMode),
      timelineDragTarget_(renderState_.timelineDragTarget),
      timelineDragInitialRange_(renderState_.timelineDragInitialRange),
      timelineDragAxisRange_(renderState_.timelineDragAxisRange),
      timelineDragAnchorSec_(renderState_.timelineDragAnchorSec),
      linearizationTrackSnapTimeSec_(
          renderState_.linearizationTrackSnapTimeSec),
      selectedDynamicModeIndex_(renderState_.selectedDynamicModeIndex),
      selectedDynamicModeSnapshotTimeSec_(
          renderState_.selectedDynamicModeSnapshotTimeSec) {}

void MonitorView::Render(const MonitorInput &input, const MonitorState &state,
    architecture::EventSink<MonitorEvent> events) {
  renderState_ = state;
  events_ = std::move(events);
  if (!renderState_.workspaceInitialized) {
    InitializePresetWorkspace();
    renderState_.workspaceInitialized = true;
  }

  const TelemetrySources &sources = input;
  if (sources.primary == nullptr) {
    ImGui::TextDisabled("Primary telemetry is unavailable.");
    events_.Emit(MonitorStateChanged{renderState_});
    return;
  }

  const telemetry::TelemetrySnapshot &telemetry = *sources.primary;
  SynchronizeTimelineState(telemetry);

  const std::span dynamicModeHistory = input.dynamicModes.history;

  if (!ImGui::BeginTabBar("MonitorViews")) {
    events_.Emit(MonitorStateChanged{renderState_});
    return;
  }

  if (ImGui::BeginTabItem("Plots")) {
    DrawWindow(sources, dynamicModeHistory);
    ImGui::EndTabItem();
  }
  if (ImGui::BeginTabItem("Dynamic Modes")) {
    DrawDynamicModes(input.dynamicModes);
    ImGui::EndTabItem();
  }
  ImGui::EndTabBar();
  events_.Emit(MonitorStateChanged{renderState_});
}

void MonitorView::InitializePresetWorkspace() {
  plots_ = BuildDefaultMonitorPlotTemplates();
  renderState_.customPlotSlots.fill(std::nullopt);
  nextPlotId_ = 1;
  for (MonitorPlot &plot : plots_) {
    plot.id = nextPlotId_++;
  }
  activePresetMask_ = GetPresetBit(MonitorPreset::AircraftState);
}
MonitorView::MonitorPlot *MonitorView::FindPlot(std::uint64_t plotId) {
  const auto plot = std::find_if(plots_.begin(),
      plots_.end(),
      [plotId](
          const MonitorPlot &candidate) { return candidate.id == plotId; });
  return plot == plots_.end() ? nullptr : &*plot;
}

void MonitorView::DeletePlot(std::uint64_t plotId) {
  if (RemoveMonitorPlot(renderState_, plotId)) {
    noEmptySlotMessage_ = false;
  }
}

std::size_t MonitorView::GetAvailablePlotSlotCount() const {
  if (plotLayout_ == MonitorPlotLayout::List) {
    return 0;
  }
  const std::size_t visiblePresetPlotCount =
      static_cast<std::size_t>(std::count_if(plots_.begin(),
          plots_.end(),
          [this](const MonitorPlot &plot) {
            return !plot.custom && IsPlotVisible(plot);
          }));
  return GetMonitorTrailingEmptySlotCount(plotLayout_, visiblePresetPlotCount);
}

std::optional<std::size_t> MonitorView::FindFirstEmptyPlotSlot() const {
  const std::size_t slotCount = GetAvailablePlotSlotCount();
  for (std::size_t slotIndex = 0; slotIndex < slotCount; ++slotIndex) {
    if (!renderState_.customPlotSlots[slotIndex].has_value()) {
      return slotIndex;
    }
  }
  return std::nullopt;
}

void MonitorView::DrawWindow(const TelemetrySources &sources,
    std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory) {
  const ImVec2 workspaceSize = ImGui::GetContentRegionAvail();
  const float uiScale = std::max(ui::GetUIScale(), 0.001F);
  if (presetPaneOpen_) {
    const float availableWidthLogical = workspaceSize.x / uiScale;
    const float maximumPresetPaneWidth = std::max(PresetPaneMinWidth,
        std::min(PresetPaneMaxWidth,
            availableWidthLogical - PresetPaneMinimumPlotWidth
                - PaneSplitterThickness));
    presetPaneWidth_ = ClampToOrderedRange(presetPaneWidth_,
        PresetPaneMinWidth,
        maximumPresetPaneWidth);

    if (ImGui::BeginChild("MonitorPresetPane",
            ImVec2(ui::Ui(presetPaneWidth_), 0.0F),
            false,
            ImGuiWindowFlags_NoScrollbar
                | ImGuiWindowFlags_NoScrollWithMouse)) {
      DrawPresetPaneHeader();
      if (ImGui::BeginChild("MonitorPresetList", ImVec2(0.0F, 0.0F), true)) {
        DrawPresetPanel();
      }
      ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::SameLine(0.0F, 0.0F);
    DrawVerticalPaneSplitter("##MonitorPresetSplitter",
        workspaceSize.y,
        presetPaneWidth_,
        PresetPaneMinWidth,
        maximumPresetPaneWidth);
  } else {
    if (ImGui::BeginChild("MonitorPresetPaneCollapsed",
            ImVec2(ui::Ui(PresetPaneCollapsedWidth), 0.0F),
            true,
            ImGuiWindowFlags_NoScrollbar
                | ImGuiWindowFlags_NoScrollWithMouse)) {
      if (ImGui::Button(">##OpenMonitorPresets", ImVec2(-1.0F, 0.0F))) {
        presetPaneOpen_ = true;
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Open Monitor Presets");
      }
    }
    ImGui::EndChild();
  }

  ImGui::SameLine(0.0F, 0.0F);
  if (ImGui::BeginChild("MonitorPlotPane",
          ImVec2(0.0F, 0.0F),
          false,
          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
    DrawPlotWorkspace(sources, dynamicModeHistory);
  }
  ImGui::EndChild();
}

void MonitorView::DrawPlotWorkspace(const TelemetrySources &sources,
    std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory) {
  const telemetry::TelemetrySnapshot &telemetry = *sources.primary;
  DrawToolbar(telemetry);
  ImGui::Separator();

  const ImVec2 availableSize = ImGui::GetContentRegionAvail();
  const ImGuiStyle &style = ImGui::GetStyle();
  float plotRegionHeight = availableSize.y;
  float timelineRegionHeight = ui::Ui(TimelineCollapsedHeight);
  float maximumTimelineHeight = TimelineMinHeight;
  if (timelinePaneOpen_) {
    const float uiScale = std::max(ui::GetUIScale(), 0.001F);
    maximumTimelineHeight = std::max(TimelineMinHeight,
        std::min(TimelineMaxHeight, availableSize.y / uiScale * 0.5F));
    timelinePaneHeight_ = ClampToOrderedRange(timelinePaneHeight_,
        TimelineMinHeight,
        maximumTimelineHeight);
    timelineRegionHeight = ui::Ui(timelinePaneHeight_);
    plotRegionHeight = std::max(1.0F,
        availableSize.y - timelineRegionHeight - ui::Ui(PaneSplitterThickness)
            - style.ItemSpacing.y * 2.0F);
  } else {
    plotRegionHeight = std::max(1.0F,
        availableSize.y - timelineRegionHeight - style.ItemSpacing.y);
  }

  if (ImGui::BeginChild("PlotScrollRegion",
          ImVec2(0.0F, plotRegionHeight),
          true,
          ImGuiWindowFlags_AlwaysVerticalScrollbar)) {
    DrawPlotScrollRegion(sources);
  }
  ImGui::EndChild();

  if (timelinePaneOpen_) {
    DrawHorizontalPaneSplitter("##TimelineSplitter",
        availableSize.x,
        timelinePaneHeight_,
        TimelineMinHeight,
        maximumTimelineHeight);
  }

  constexpr ImGuiWindowFlags TimelineRegionFlags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
  if (ImGui::BeginChild("TimelineRegion",
          ImVec2(0.0F, 0.0F),
          true,
          TimelineRegionFlags)) {
    DrawTimelineHeader();
    if (timelinePaneOpen_) {
      DrawTimeline(dynamicModeHistory);
    }
  }
  ImGui::EndChild();

  DrawPlotConfigurationDialog(sources);
}

void MonitorView::DrawPlotScrollRegion(const TelemetrySources &sources) {
  if (plotLayout_ == MonitorPlotLayout::List) {
    DrawPlotList(sources);
  } else if (plotLayout_ == MonitorPlotLayout::Grid1x1) {
    DrawPlotTable(sources, 1, CalculateGridPlotHeight(1), "MonitorPlotGrid1x1");
  } else if (plotLayout_ == MonitorPlotLayout::Grid1x2) {
    DrawPlotTable(sources, 2, CalculateGridPlotHeight(1), "MonitorPlotGrid1x2");
  } else if (plotLayout_ == MonitorPlotLayout::Grid2x2) {
    DrawPlotGrid(sources, 2);
  } else if (plotLayout_ == MonitorPlotLayout::Grid2x3) {
    DrawPlotTable(sources, 3, CalculateGridPlotHeight(2), "MonitorPlotGrid2x3");
  } else {
    DrawPlotGrid(sources, 3);
  }
}

} // namespace gui
