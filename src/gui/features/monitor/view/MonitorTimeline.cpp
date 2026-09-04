#include "gui/features/monitor/MonitorView.hpp"

#include "sim/linearization/DynamicModeContracts.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace gui {

namespace {
constexpr float TimelineOverviewBarHeight = 12.0F;
constexpr float TimelineDetailBarHeight = 18.0F;
constexpr float LinearizationTrackHeight = 14.0F;
constexpr float LinearizationMarkerRadius = 3.5F;
constexpr float LinearizationMarkerHitRadius = 7.0F;
constexpr float TimelineHandleWidth = 10.0F;
constexpr float TimelineHorizontalPadding = 12.0F;
constexpr float TimelineRowSpacing = 7.0F;
constexpr double MinimumTimelineWindowSec = 0.1;
constexpr double TimelineZoomFactor = 1.15;
constexpr int TargetTimelineTickCount = 6;

template <typename T>
T ClampToOrderedRange(T value, T firstBound, T secondBound) {
  const T minimum = std::min(firstBound, secondBound);
  const T maximum = std::max(firstBound, secondBound);
  return std::min(std::max(value, minimum), maximum);
}

double CalculateTimelineTickSpacing(double durationSec) {
  if (!std::isfinite(durationSec) || durationSec <= 0.0) {
    return 1.0;
  }
  const double rawSpacing =
      durationSec / static_cast<double>(TargetTimelineTickCount - 1);
  const double magnitude = std::pow(10.0, std::floor(std::log10(rawSpacing)));
  const double normalized = rawSpacing / magnitude;
  const double niceNormalized = normalized <= 1.0   ? 1.0
                                : normalized <= 2.0 ? 2.0
                                : normalized <= 5.0 ? 5.0
                                                    : 10.0;
  return niceNormalized * magnitude;
}

std::vector<double> CalculateTimelineTicks(double minSec, double maxSec) {
  std::vector<double> ticks;
  const double spacing = CalculateTimelineTickSpacing(maxSec - minSec);
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    return ticks;
  }
  const double firstTick = std::ceil(minSec / spacing) * spacing;
  constexpr std::size_t MaximumTickCount = 64;
  for (double tick = firstTick;
      tick <= maxSec + spacing * 1.0e-6 && ticks.size() < MaximumTickCount;
      tick += spacing) {
    ticks.push_back(std::abs(tick) < spacing * 1.0e-9 ? 0.0 : tick);
  }
  return ticks;
}
} // namespace

void MonitorView::DrawTimelineHeader() {
  const bool wasOpen = timelinePaneOpen_;
  bool live = liveView_;
  const ui::UIElement toolbar =
      ui::Toolbar()
          .Id("TimelineHeader")
          .Compact()
          .Height(26.0F)
          .Left(ui::HorizontalLayout().Spacing(
              4.0F)[+ui::Button(
                        wasOpen ? "v##CollapseTimeline" : "^##OpenTimeline")
                        .Tooltip(
                            wasOpen ? "Collapse Timeline" : "Open Timeline")
                        .OnAction(
                            [this, wasOpen] { timelinePaneOpen_ = !wasOpen; })
                    + ui::Text("Timeline")])
          .Right(ui::Toggle("Live##Timeline", live)
                  .OnChanged([this](bool enabled) { SetLiveView(enabled); }));
  toolbar.Render();
}

void MonitorView::DrawTimeline(
    std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory) {
  if (selectedTimeInitialized_) {
    ImGui::Text("View %.2f - %.2f s  |  Plot %.2f - %.2f s  |  Selected %.2f s",
        timelineViewRange_.minSec,
        timelineViewRange_.maxSec,
        visibleTimeRange_.minSec,
        visibleTimeRange_.maxSec,
        selectedTimeSec_);
  } else {
    ImGui::Text("View %.2f - %.2f s  |  Plot %.2f - %.2f s",
        timelineViewRange_.minSec,
        timelineViewRange_.maxSec,
        visibleTimeRange_.minSec,
        visibleTimeRange_.maxSec);
  }

  if (!telemetryHistoryRange_) {
    ImGui::TextDisabled("Waiting for telemetry history.");
    return;
  }

  DrawTimelineOverview(*telemetryHistoryRange_);
  ImGui::Dummy(ImVec2(0.0F, ui::Ui(TimelineRowSpacing)));
  DrawTimelineDetail();
  ImGui::Dummy(ImVec2(0.0F, ui::Ui(TimelineRowSpacing)));
  DrawLinearizationTrack(dynamicModeHistory);
}

void MonitorView::DrawTimelineOverview(const TimelineRange &historyRange) {
  ImGui::TextDisabled("Overview  History %.2f - %.2f s",
      historyRange.minSec,
      historyRange.maxSec);

  const TimelineRange trackRange = GetEffectiveHistoryRange(historyRange);

  const float barWidth = std::max(ui::Ui(80.0F),
      ImGui::GetContentRegionAvail().x
          - ui::Ui(TimelineHorizontalPadding) * 2.0F);
  const float barHeight = ui::Ui(TimelineOverviewBarHeight);
  const ImVec2 cursorPosition = ImGui::GetCursorScreenPos();
  const ImVec2 barMin(cursorPosition.x + ui::Ui(TimelineHorizontalPadding),
      cursorPosition.y);
  const ImVec2 barMax(barMin.x + barWidth, barMin.y + barHeight);
  const double historyDuration = trackRange.maxSec - trackRange.minSec;

  const auto timeToX = [&](double timeSec) {
    const double ratio =
        ClampToOrderedRange((timeSec - trackRange.minSec) / historyDuration,
            0.0,
            1.0);
    return barMin.x + static_cast<float>(ratio) * barWidth;
  };
  const auto xToTime = [&](float x) {
    const double ratio =
        ClampToOrderedRange(static_cast<double>((x - barMin.x) / barWidth),
            0.0,
            1.0);
    return trackRange.minSec + ratio * historyDuration;
  };

  const ImGuiIO &io = ImGui::GetIO();
  const bool barHovered = io.MousePos.x >= barMin.x && io.MousePos.x <= barMax.x
                          && io.MousePos.y >= barMin.y
                          && io.MousePos.y <= barMax.y;
  if (barHovered && io.KeyCtrl && io.MouseWheel != 0.0F
      && timelineDragMode_ == TimelineDragMode::None) {
    ZoomTimelineView(io.MouseWheel, xToTime(io.MousePos.x));
  }

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(barMin,
      barMax,
      ImGui::GetColorU32(ImGuiCol_FrameBg),
      ui::Ui(3.0F));

  const float selectionMinX = timeToX(timelineViewRange_.minSec);
  const float selectionMaxX = timeToX(timelineViewRange_.maxSec);
  drawList->AddRectFilled(ImVec2(selectionMinX, barMin.y),
      ImVec2(selectionMaxX, barMax.y),
      ImGui::GetColorU32(ImGuiCol_FrameBgHovered),
      ui::Ui(3.0F));

  const float handleWidth = ui::Ui(TimelineHandleWidth);
  const ImU32 viewHandleColor = ImGui::GetColorU32(ImGuiCol_TextDisabled);
  drawList->AddLine(ImVec2(selectionMinX, barMin.y - ui::Ui(2.0F)),
      ImVec2(selectionMinX, barMax.y + ui::Ui(2.0F)),
      viewHandleColor,
      ui::Ui(1.5F));
  drawList->AddLine(ImVec2(selectionMaxX, barMin.y - ui::Ui(2.0F)),
      ImVec2(selectionMaxX, barMax.y + ui::Ui(2.0F)),
      viewHandleColor,
      ui::Ui(1.5F));

  ImGui::SetCursorScreenPos(ImVec2(barMin.x - handleWidth * 0.5F, barMin.y));
  ImGui::InvisibleButton("##TimelineOverviewInteraction",
      ImVec2(barWidth + handleWidth, barHeight),
      ImGuiButtonFlags_MouseButtonLeft);
  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive();
  const float mouseX = ImGui::GetIO().MousePos.x;
  const float handleHitRadius = handleWidth;
  const float startHandleDistance = std::abs(mouseX - selectionMinX);
  const float endHandleDistance = std::abs(mouseX - selectionMaxX);
  const bool isHandleHovered =
      std::min(startHandleDistance, endHandleDistance) <= handleHitRadius;
  const bool isSelectionHovered =
      mouseX > selectionMinX && mouseX < selectionMaxX;
  if (isHovered || isActive) {
    ImGui::SetMouseCursor(isHandleHovered      ? ImGuiMouseCursor_ResizeEW
                          : isSelectionHovered ? ImGuiMouseCursor_ResizeAll
                                               : ImGuiMouseCursor_Arrow);
  }

  if (ImGui::IsItemActivated()) {
    if (isHandleHovered) {
      timelineDragMode_ = startHandleDistance <= endHandleDistance
                              ? TimelineDragMode::Start
                              : TimelineDragMode::End;
    } else if (isSelectionHovered) {
      timelineDragMode_ = TimelineDragMode::Window;
    } else {
      timelineDragMode_ = TimelineDragMode::Window;
    }
    timelineDragTarget_ = TimelineDragTarget::TimelineView;
    timelineDragAnchorSec_ = xToTime(mouseX);
    timelineDragInitialRange_ = timelineViewRange_;
    timelineDragAxisRange_ = trackRange;
    SetLiveView(false);
  }

  if (isActive && timelineDragTarget_ == TimelineDragTarget::TimelineView) {
    const double mouseRatio = ClampToOrderedRange(
        static_cast<double>((ImGui::GetIO().MousePos.x - barMin.x) / barWidth),
        0.0,
        1.0);
    const double mouseTime =
        timelineDragAxisRange_.minSec
        + mouseRatio
              * (timelineDragAxisRange_.maxSec - timelineDragAxisRange_.minSec);
    const double minimumViewDuration = MinimumTimelineWindowSec;
    if (timelineDragMode_ == TimelineDragMode::Start) {
      const double maximumStartSec = std::max(trackRange.minSec,
          timelineViewRange_.maxSec - minimumViewDuration);
      timelineViewRange_.minSec =
          ClampToOrderedRange(mouseTime, trackRange.minSec, maximumStartSec);
    } else if (timelineDragMode_ == TimelineDragMode::End) {
      const double minimumEndSec = std::min(trackRange.maxSec,
          timelineViewRange_.minSec + minimumViewDuration);
      timelineViewRange_.maxSec =
          ClampToOrderedRange(mouseTime, minimumEndSec, trackRange.maxSec);
    } else {
      const double duration = ClampToOrderedRange(
          timelineDragInitialRange_.maxSec - timelineDragInitialRange_.minSec,
          MinimumTimelineWindowSec,
          historyDuration);
      const double desiredMin =
          timelineDragInitialRange_.minSec + mouseTime - timelineDragAnchorSec_;
      const double maximumMinSec =
          std::max(trackRange.minSec, trackRange.maxSec - duration);
      const double minSec =
          ClampToOrderedRange(desiredMin, trackRange.minSec, maximumMinSec);
      timelineViewRange_ = {minSec, minSec + duration};
    }
    ClampTimelineViewRangeToHistory();
    timelineViewWindowSec_ =
        timelineViewRange_.maxSec - timelineViewRange_.minSec;
  }
  if (ImGui::IsItemDeactivated()) {
    timelineDragMode_ = TimelineDragMode::None;
    timelineDragTarget_ = TimelineDragTarget::None;
  }
}

void MonitorView::DrawTimelineDetail() {
  ImGui::TextDisabled("Detail  View %.2f - %.2f s",
      timelineViewRange_.minSec,
      timelineViewRange_.maxSec);

  const TimelineRange detailRange = timelineViewRange_;
  const double viewDuration = detailRange.maxSec - detailRange.minSec;
  if (!std::isfinite(viewDuration) || viewDuration <= 0.0) {
    return;
  }

  const float barWidth = std::max(ui::Ui(80.0F),
      ImGui::GetContentRegionAvail().x
          - ui::Ui(TimelineHorizontalPadding) * 2.0F);
  const float barHeight = ui::Ui(TimelineDetailBarHeight);
  const ImVec2 cursorPosition = ImGui::GetCursorScreenPos();
  const ImVec2 barMin(cursorPosition.x + ui::Ui(TimelineHorizontalPadding),
      cursorPosition.y + ImGui::GetTextLineHeight());
  const ImVec2 barMax(barMin.x + barWidth, barMin.y + barHeight);

  const auto timeToX = [&](double timeSec) {
    const double ratio =
        ClampToOrderedRange((timeSec - detailRange.minSec) / viewDuration,
            0.0,
            1.0);
    return barMin.x + static_cast<float>(ratio) * barWidth;
  };
  const auto xToTime = [&](float x) {
    const double ratio =
        ClampToOrderedRange(static_cast<double>((x - barMin.x) / barWidth),
            0.0,
            1.0);
    return detailRange.minSec + ratio * viewDuration;
  };

  const ImGuiIO &io = ImGui::GetIO();
  const bool barHovered = io.MousePos.x >= barMin.x && io.MousePos.x <= barMax.x
                          && io.MousePos.y >= barMin.y
                          && io.MousePos.y <= barMax.y;
  if (barHovered && io.KeyCtrl && io.MouseWheel != 0.0F
      && timelineDragMode_ == TimelineDragMode::None) {
    ZoomTimelineView(io.MouseWheel, xToTime(io.MousePos.x));
  }

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(barMin,
      barMax,
      ImGui::GetColorU32(ImGuiCol_FrameBg),
      ui::Ui(3.0F));
  const std::vector<double> ticks =
      CalculateTimelineTicks(detailRange.minSec, detailRange.maxSec);
  for (double tick : ticks) {
    const float tickX = timeToX(tick);
    drawList->AddLine(ImVec2(tickX, barMin.y),
        ImVec2(tickX, barMax.y),
        ImGui::GetColorU32(ImGuiCol_Border));
    char label[32]{};
    std::snprintf(label, sizeof(label), "%.3g s", tick);
    const ImVec2 labelSize = ImGui::CalcTextSize(label);
    drawList->AddText(ImVec2(tickX - labelSize.x * 0.5F,
                          barMin.y - labelSize.y - ui::Ui(2.0F)),
        ImGui::GetColorU32(ImGuiCol_TextDisabled),
        label);
  }

  const float selectionMinX = timeToX(visibleTimeRange_.minSec);
  const float selectionMaxX = timeToX(visibleTimeRange_.maxSec);
  drawList->AddRectFilled(ImVec2(selectionMinX, barMin.y),
      ImVec2(selectionMaxX, barMax.y),
      ImGui::GetColorU32(ImGuiCol_SliderGrabActive),
      ui::Ui(3.0F));

  const float handleWidth = ui::Ui(TimelineHandleWidth);
  drawList->AddRectFilled(ImVec2(selectionMinX - handleWidth * 0.5F, barMin.y),
      ImVec2(selectionMinX + handleWidth * 0.5F, barMax.y),
      ImGui::GetColorU32(ImGuiCol_SliderGrab),
      ui::Ui(2.0F));
  drawList->AddRectFilled(ImVec2(selectionMaxX - handleWidth * 0.5F, barMin.y),
      ImVec2(selectionMaxX + handleWidth * 0.5F, barMax.y),
      ImGui::GetColorU32(ImGuiCol_SliderGrab),
      ui::Ui(2.0F));

  if (selectedTimeInitialized_ && selectedTimeSec_ >= detailRange.minSec
      && selectedTimeSec_ <= detailRange.maxSec) {
    const float cursorX = timeToX(selectedTimeSec_);
    drawList->AddLine(ImVec2(cursorX, barMin.y - ui::Ui(3.0F)),
        ImVec2(cursorX, barMax.y + ui::Ui(3.0F)),
        ImGui::GetColorU32(ImVec4(0.95F, 0.75F, 0.25F, 0.9F)),
        ui::Ui(1.5F));
  }

  ImGui::SetCursorScreenPos(ImVec2(barMin.x - handleWidth * 0.5F, barMin.y));
  ImGui::InvisibleButton("##TimelineDetailInteraction",
      ImVec2(barWidth + handleWidth, barHeight),
      ImGuiButtonFlags_MouseButtonLeft);
  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive();
  const float mouseX = ImGui::GetIO().MousePos.x;
  const float handleHitRadius = handleWidth;
  const float startHandleDistance = std::abs(mouseX - selectionMinX);
  const float endHandleDistance = std::abs(mouseX - selectionMaxX);
  const bool isHandleHovered =
      std::min(startHandleDistance, endHandleDistance) <= handleHitRadius;
  const bool isSelectionHovered =
      mouseX > selectionMinX && mouseX < selectionMaxX;
  if (isHovered || isActive) {
    ImGui::SetMouseCursor(isHandleHovered      ? ImGuiMouseCursor_ResizeEW
                          : isSelectionHovered ? ImGuiMouseCursor_ResizeAll
                                               : ImGuiMouseCursor_Hand);
  }

  if (ImGui::IsItemActivated()) {
    if (isHandleHovered) {
      timelineDragMode_ = startHandleDistance <= endHandleDistance
                              ? TimelineDragMode::Start
                              : TimelineDragMode::End;
      timelineDragTarget_ = TimelineDragTarget::PlotVisible;
      timelineDragInitialRange_ = visibleTimeRange_;
    } else if (isSelectionHovered) {
      timelineDragMode_ = TimelineDragMode::Window;
      timelineDragTarget_ = TimelineDragTarget::PlotVisible;
      timelineDragInitialRange_ = visibleTimeRange_;
    } else {
      timelineDragMode_ = TimelineDragMode::Window;
      timelineDragTarget_ = TimelineDragTarget::TimelineView;
      timelineDragInitialRange_ = timelineViewRange_;
    }
    timelineDragAnchorSec_ = xToTime(mouseX);
    timelineDragAxisRange_ = timelineViewRange_;
    SetLiveView(false);
  }

  if (isActive && timelineDragTarget_ != TimelineDragTarget::None) {
    const double mouseRatio = ClampToOrderedRange(
        static_cast<double>((ImGui::GetIO().MousePos.x - barMin.x) / barWidth),
        0.0,
        1.0);
    const double mouseTime =
        timelineDragAxisRange_.minSec
        + mouseRatio
              * (timelineDragAxisRange_.maxSec - timelineDragAxisRange_.minSec);
    if (timelineDragTarget_ == TimelineDragTarget::TimelineView) {
      const double duration =
          timelineDragInitialRange_.maxSec - timelineDragInitialRange_.minSec;
      const double desiredMin =
          timelineDragInitialRange_.minSec + mouseTime - timelineDragAnchorSec_;
      timelineViewRange_ = {desiredMin, desiredMin + duration};
      ClampTimelineViewRangeToHistory();
      timelineViewWindowSec_ =
          timelineViewRange_.maxSec - timelineViewRange_.minSec;
    } else if (timelineDragMode_ == TimelineDragMode::Start) {
      visibleTimeRange_.minSec = std::min(mouseTime,
          visibleTimeRange_.maxSec - MinimumTimelineWindowSec);
      ClampVisibleTimeRangeToHistory();
      EnsureVisibleTimeRangeInTimelineView();
      liveWindowSec_ = visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
      timelineViewWindowSec_ =
          timelineViewRange_.maxSec - timelineViewRange_.minSec;
      UpdateSharedXAxisTicks();
    } else if (timelineDragMode_ == TimelineDragMode::End) {
      visibleTimeRange_.maxSec = std::max(mouseTime,
          visibleTimeRange_.minSec + MinimumTimelineWindowSec);
      ClampVisibleTimeRangeToHistory();
      EnsureVisibleTimeRangeInTimelineView();
      liveWindowSec_ = visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
      timelineViewWindowSec_ =
          timelineViewRange_.maxSec - timelineViewRange_.minSec;
      UpdateSharedXAxisTicks();
    } else {
      const double duration =
          timelineDragInitialRange_.maxSec - timelineDragInitialRange_.minSec;
      const double desiredMin =
          timelineDragInitialRange_.minSec + mouseTime - timelineDragAnchorSec_;
      visibleTimeRange_ = {desiredMin, desiredMin + duration};
      ClampVisibleTimeRangeToHistory();
      EnsureVisibleTimeRangeInTimelineView();
      liveWindowSec_ = visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
      timelineViewWindowSec_ =
          timelineViewRange_.maxSec - timelineViewRange_.minSec;
      UpdateSharedXAxisTicks();
    }
  }
  if (ImGui::IsItemDeactivated()) {
    timelineDragMode_ = TimelineDragMode::None;
    timelineDragTarget_ = TimelineDragTarget::None;
  }
}

void MonitorView::DrawLinearizationTrack(
    std::span<const gnc::DynamicModeSnapshot> dynamicModeHistory) {
  ImGui::TextDisabled("Linearization");

  const TimelineRange trackRange = timelineViewRange_;
  const double trackDuration = trackRange.maxSec - trackRange.minSec;
  if (!std::isfinite(trackDuration) || trackDuration <= 0.0) {
    return;
  }

  const float trackWidth = std::max(ui::Ui(80.0F),
      ImGui::GetContentRegionAvail().x
          - ui::Ui(TimelineHorizontalPadding) * 2.0F);
  const float trackHeight = ui::Ui(LinearizationTrackHeight);
  const ImVec2 cursorPosition = ImGui::GetCursorScreenPos();
  const ImVec2 trackMin(cursorPosition.x + ui::Ui(TimelineHorizontalPadding),
      cursorPosition.y);
  const ImVec2 trackMax(trackMin.x + trackWidth, trackMin.y + trackHeight);
  const auto timeToX = [&](double timeSec) {
    const double ratio =
        ClampToOrderedRange((timeSec - trackRange.minSec) / trackDuration,
            0.0,
            1.0);
    return trackMin.x + static_cast<float>(ratio) * trackWidth;
  };
  const auto xToTime = [&](float x) {
    const double ratio =
        ClampToOrderedRange(static_cast<double>((x - trackMin.x) / trackWidth),
            0.0,
            1.0);
    return trackRange.minSec + ratio * trackDuration;
  };

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(trackMin,
      trackMax,
      ImGui::GetColorU32(ImGuiCol_FrameBg),
      ui::Ui(3.0F));

  if (!dynamicModeHistory.empty()) {
    const ImU32 markerColor = ImGui::GetColorU32(ImGuiCol_PlotHistogram);
    for (const gnc::DynamicModeSnapshot &snapshot : dynamicModeHistory) {
      if (snapshot.simulationTimeSec < trackRange.minSec
          || snapshot.simulationTimeSec > trackRange.maxSec) {
        continue;
      }
      const float markerX = timeToX(snapshot.simulationTimeSec);
      const ImVec2 markerCenter(markerX, (trackMin.y + trackMax.y) * 0.5F);
      drawList->AddLine(ImVec2(markerX, trackMin.y + ui::Ui(2.0F)),
          ImVec2(markerX, trackMax.y - ui::Ui(2.0F)),
          markerColor,
          ui::Ui(1.0F));
      drawList->AddCircleFilled(markerCenter,
          ui::Ui(LinearizationMarkerRadius),
          markerColor);
    }
  }

  if (selectedTimeInitialized_ && selectedTimeSec_ >= trackRange.minSec
      && selectedTimeSec_ <= trackRange.maxSec) {
    const float selectedX = timeToX(selectedTimeSec_);
    drawList->AddLine(ImVec2(selectedX, trackMin.y - ui::Ui(2.0F)),
        ImVec2(selectedX, trackMax.y + ui::Ui(2.0F)),
        ImGui::GetColorU32(ImVec4(0.95F, 0.75F, 0.25F, 0.9F)),
        ui::Ui(1.5F));
  }

  ImGui::SetCursorScreenPos(trackMin);
  ImGui::InvisibleButton("##LinearizationTrackInteraction",
      ImVec2(trackWidth, trackHeight),
      ImGuiButtonFlags_MouseButtonLeft);
  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive();

  const gnc::DynamicModeSnapshot *hoveredSnapshot = nullptr;
  float nearestDistance = ui::Ui(LinearizationMarkerHitRadius) + 1.0F;
  if ((isHovered || isActive) && !dynamicModeHistory.empty()) {
    const float mouseX = ImGui::GetIO().MousePos.x;
    for (const gnc::DynamicModeSnapshot &snapshot : dynamicModeHistory) {
      if (snapshot.simulationTimeSec < trackRange.minSec
          || snapshot.simulationTimeSec > trackRange.maxSec) {
        continue;
      }
      const float distance =
          std::abs(mouseX - timeToX(snapshot.simulationTimeSec));
      if (distance < nearestDistance) {
        nearestDistance = distance;
        hoveredSnapshot = &snapshot;
      }
    }
  }

  if (isHovered) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    if (hoveredSnapshot != nullptr) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted("Linearization");
      ImGui::Text("Time: %.3f s", hoveredSnapshot->simulationTimeSec);
      ImGui::EndTooltip();
    }
  }

  if (ImGui::IsItemActivated()) {
    if (hoveredSnapshot != nullptr) {
      linearizationTrackSnapTimeSec_ = hoveredSnapshot->simulationTimeSec;
      SelectTimelineTime(*linearizationTrackSnapTimeSec_, true);
    } else {
      linearizationTrackSnapTimeSec_.reset();
      SelectTimelineTime(xToTime(ImGui::GetIO().MousePos.x), true);
    }
  } else if (isActive) {
    const float dragDistance =
        std::abs(ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x);
    if (dragDistance > ImGui::GetIO().MouseDragThreshold) {
      linearizationTrackSnapTimeSec_.reset();
    }
    if (!linearizationTrackSnapTimeSec_) {
      SelectTimelineTime(xToTime(ImGui::GetIO().MousePos.x), true);
    }
  }
  if (ImGui::IsItemDeactivated()) {
    linearizationTrackSnapTimeSec_.reset();
  }
}

std::optional<MonitorView::TimelineRange> MonitorView::GetTelemetryHistoryRange(
    const telemetry::TelemetrySnapshot &telemetry) const {
  const std::optional<telemetry::TelemetryTimeRange> range =
      telemetry.publishedTimeRange;
  if (!range) {
    return std::nullopt;
  }

  return TimelineRange{std::min(0.0, range->minSec), range->maxSec};
}

void MonitorView::SynchronizeTimelineState(
    const telemetry::TelemetrySnapshot &telemetry) {
  telemetryHistoryRange_ = GetTelemetryHistoryRange(telemetry);
  if (!telemetryHistoryRange_) {
    selectedTimeInitialized_ = false;
    return;
  }

  if (liveView_) {
    UpdateLiveTimeRanges();
    selectedTimeSec_ = telemetryHistoryRange_->maxSec;
    selectedTimeInitialized_ = true;
  } else {
    ClampTimelineViewRangeToHistory();
    ClampVisibleTimeRangeToHistory();
    if (!selectedTimeInitialized_) {
      selectedTimeSec_ = visibleTimeRange_.maxSec;
      selectedTimeInitialized_ = true;
    } else {
      selectedTimeSec_ = ClampToOrderedRange(selectedTimeSec_,
          telemetryHistoryRange_->minSec,
          telemetryHistoryRange_->maxSec);
    }
  }
  UpdateSharedXAxisTicks();
}

MonitorView::TimelineRange MonitorView::GetEffectiveHistoryRange(
    const TimelineRange &historyRange) const {
  if (historyRange.maxSec - historyRange.minSec >= MinimumTimelineWindowSec) {
    return historyRange;
  }
  return {historyRange.minSec, historyRange.minSec + MinimumTimelineWindowSec};
}

void MonitorView::ClampTimelineViewRangeToHistory() {
  if (!telemetryHistoryRange_) {
    return;
  }

  const TimelineRange historyRange =
      GetEffectiveHistoryRange(*telemetryHistoryRange_);
  const double historyDuration = historyRange.maxSec - historyRange.minSec;
  double duration = timelineViewRange_.maxSec - timelineViewRange_.minSec;
  const bool isFiniteRange = std::isfinite(timelineViewRange_.minSec)
                             && std::isfinite(timelineViewRange_.maxSec);
  const bool isInsideHistory =
      isFiniteRange && duration >= MinimumTimelineWindowSec
      && duration <= historyDuration
      && timelineViewRange_.minSec >= historyRange.minSec
      && timelineViewRange_.maxSec <= historyRange.maxSec;
  if (isInsideHistory) {
    return;
  }

  if (!std::isfinite(duration) || duration < MinimumTimelineWindowSec) {
    duration = MinimumTimelineWindowSec;
  }
  duration = std::min(duration, historyDuration);
  double minSec = timelineViewRange_.minSec;
  if (!std::isfinite(minSec)) {
    minSec = historyRange.minSec;
  }
  const double maximumMinSec = historyRange.maxSec - duration;
  minSec = ClampToOrderedRange(minSec, historyRange.minSec, maximumMinSec);
  timelineViewRange_ = {minSec, minSec + duration};
}

void MonitorView::ClampVisibleTimeRangeToHistory() {
  if (!telemetryHistoryRange_) {
    return;
  }

  const TimelineRange historyRange =
      GetEffectiveHistoryRange(*telemetryHistoryRange_);
  const double historyDuration = std::max(MinimumTimelineWindowSec,
      historyRange.maxSec - historyRange.minSec);
  double duration = visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
  const bool isFiniteRange = std::isfinite(visibleTimeRange_.minSec)
                             && std::isfinite(visibleTimeRange_.maxSec);
  const bool hasValidDuration = isFiniteRange
                                && duration >= MinimumTimelineWindowSec
                                && duration <= historyDuration;
  const bool isInsideHistory =
      hasValidDuration && visibleTimeRange_.minSec >= historyRange.minSec
      && visibleTimeRange_.maxSec <= historyRange.maxSec;
  if (isInsideHistory) {
    return;
  }

  if (!std::isfinite(duration) || duration < MinimumTimelineWindowSec) {
    duration = MinimumTimelineWindowSec;
  }
  duration = std::min(duration, historyDuration);

  double minSec = visibleTimeRange_.minSec;
  if (!std::isfinite(minSec)) {
    minSec = historyRange.minSec;
  }
  const double maximumMinSec =
      std::max(historyRange.minSec, historyRange.maxSec - duration);
  minSec = ClampToOrderedRange(minSec, historyRange.minSec, maximumMinSec);
  visibleTimeRange_ = {minSec, minSec + duration};
}

void MonitorView::EnsureVisibleTimeRangeInTimelineView() {
  const double visibleDuration =
      visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
  double viewDuration = timelineViewRange_.maxSec - timelineViewRange_.minSec;
  if (visibleDuration > viewDuration) {
    timelineViewRange_ = visibleTimeRange_;
    viewDuration = visibleDuration;
  } else if (visibleTimeRange_.minSec < timelineViewRange_.minSec) {
    timelineViewRange_.minSec = visibleTimeRange_.minSec;
    timelineViewRange_.maxSec = timelineViewRange_.minSec + viewDuration;
  } else if (visibleTimeRange_.maxSec > timelineViewRange_.maxSec) {
    timelineViewRange_.maxSec = visibleTimeRange_.maxSec;
    timelineViewRange_.minSec = timelineViewRange_.maxSec - viewDuration;
  }
  ClampTimelineViewRangeToHistory();
}

void MonitorView::UpdateSharedXAxisTicks() {
  sharedXAxisTicks_ = CalculateTimelineTicks(visibleTimeRange_.minSec,
      visibleTimeRange_.maxSec);
}

void MonitorView::UpdateLiveTimeRanges() {
  if (!telemetryHistoryRange_) {
    timelineViewRange_ = {0.0, timelineViewWindowSec_};
    visibleTimeRange_ = {0.0, liveWindowSec_};
    return;
  }

  const TimelineRange historyRange =
      GetEffectiveHistoryRange(*telemetryHistoryRange_);
  const double historyDuration = std::max(MinimumTimelineWindowSec,
      historyRange.maxSec - historyRange.minSec);
  const double visibleDuration = std::min(liveWindowSec_, historyDuration);
  const double viewDuration =
      std::min(std::max(timelineViewWindowSec_, visibleDuration),
          historyDuration);
  timelineViewRange_.maxSec = historyRange.maxSec;
  timelineViewRange_.minSec = historyRange.maxSec - viewDuration;
  visibleTimeRange_.maxSec = historyRange.maxSec;
  visibleTimeRange_.minSec = historyRange.maxSec - visibleDuration;
}

void MonitorView::SetLiveView(bool enabled) {
  if (liveView_ == enabled) {
    return;
  }

  liveView_ = enabled;
  events_.Emit(MonitorLiveChanged{enabled});
  if (liveView_) {
    UpdateLiveTimeRanges();
    if (telemetryHistoryRange_) {
      selectedTimeSec_ = telemetryHistoryRange_->maxSec;
      selectedTimeInitialized_ = true;
    }
    UpdateSharedXAxisTicks();
  }
}

void MonitorView::SelectTimelineTime(double timeSec, bool disableLive) {
  if (!telemetryHistoryRange_ || !std::isfinite(timeSec)) {
    return;
  }

  if (disableLive) {
    SetLiveView(false);
  }
  selectedTimeSec_ = ClampToOrderedRange(timeSec,
      telemetryHistoryRange_->minSec,
      telemetryHistoryRange_->maxSec);
  selectedTimeInitialized_ = true;
  events_.Emit(MonitorCursorMoved{selectedTimeSec_});
}

void MonitorView::ZoomTimelineView(double wheelDelta, double anchorSec) {
  if (!telemetryHistoryRange_ || !std::isfinite(wheelDelta)
      || wheelDelta == 0.0) {
    return;
  }
  events_.Emit(MonitorZoomRequested{wheelDelta, anchorSec});

  const TimelineRange historyRange =
      GetEffectiveHistoryRange(*telemetryHistoryRange_);
  const double historyDuration = std::max(MinimumTimelineWindowSec,
      historyRange.maxSec - historyRange.minSec);
  const double visibleDuration =
      visibleTimeRange_.maxSec - visibleTimeRange_.minSec;
  const double minimumViewDuration = std::min(historyDuration,
      std::max(MinimumTimelineWindowSec, visibleDuration));
  const double currentDuration =
      ClampToOrderedRange(timelineViewRange_.maxSec - timelineViewRange_.minSec,
          MinimumTimelineWindowSec,
          historyDuration);
  const double zoomMultiplier = std::pow(TimelineZoomFactor, -wheelDelta);
  const double newDuration =
      ClampToOrderedRange(currentDuration * zoomMultiplier,
          minimumViewDuration,
          historyDuration);

  timelineViewWindowSec_ = newDuration;
  if (liveView_) {
    timelineViewRange_.maxSec = historyRange.maxSec;
    timelineViewRange_.minSec = historyRange.maxSec - newDuration;
  } else {
    const double anchorRatio =
        (anchorSec - timelineViewRange_.minSec) / currentDuration;
    timelineViewRange_.minSec = anchorSec - newDuration * anchorRatio;
    timelineViewRange_.maxSec = timelineViewRange_.minSec + newDuration;
    ClampTimelineViewRangeToHistory();
  }
}

std::optional<double> MonitorView::DrawPlotOverlay() {
  const bool isHovered = ImPlot::IsPlotHovered();
  std::optional<double> hoverTimeSec;
  if (isHovered) {
    const ImPlotRect limits = ImPlot::GetPlotLimits();
    hoverTimeSec = ClampToOrderedRange(ImPlot::GetPlotMousePos().x,
        limits.X.Min,
        limits.X.Max);
    SelectTimelineTime(*hoverTimeSec, false);
  }

  if (selectedTimeInitialized_) {
    ImPlotSpec cursorSpec;
    cursorSpec.LineColor = ImVec4(0.95F, 0.75F, 0.25F, 0.9F);
    cursorSpec.Flags = ImPlotItemFlags_NoLegend;
    ImPlot::PlotInfLines("##SharedTimeCursor",
        &selectedTimeSec_,
        1,
        cursorSpec);
  }

  return hoverTimeSec;
}
} // namespace gui