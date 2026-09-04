#include "gui/features/monitor/MonitorView.hpp"

#include "sim/telemetry/TelemetryContracts.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <optional>

namespace gui {
namespace UI = FlightUI;

void MonitorView::DrawToolbar(const telemetry::TelemetrySnapshot &telemetry) {
  if (ImGui::Button("+ Plot")) {
    const std::optional<std::size_t> slot = FindFirstEmptyPlotSlot();
    RequestAddPlot(slot);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Add a telemetry plot to the first empty slot");
  }
  ImGui::SameLine();

  bool live = liveView_;
  if (ImGui::Checkbox("Live##Toolbar", &live)) {
    SetLiveView(live);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(liveView_
                          ? "Following the latest telemetry time"
                          : "Viewport paused; telemetry continues recording");
  }
  ImGui::SameLine();

  DrawPlotLayoutSelector();

  ImGui::SameLine();
  DrawDisplayModeSelector();

  ImGui::SameLine();
  if (noEmptySlotMessage_) {
    ImGui::TextColored(
        UI::GetDarkEditorSemanticColor(UI::SemanticColor::Warning),
        "No empty plot slots. Change the layout or remove a plot.");
    ImGui::SameLine();
  }
  const std::size_t channelCount = telemetry.GetChannelPaths().size();
  ImGui::TextDisabled("%zu channels | %.2f - %.2f s%s",
      channelCount,
      visibleTimeRange_.minSec,
      visibleTimeRange_.maxSec,
      liveView_ ? " | following latest" : " | view paused");
}

void MonitorView::DrawPlotLayoutSelector() {
  ImGui::TextUnformatted("Layout:");
  const auto drawLayoutButton = [this](const char *label,
                                    MonitorPlotLayout layout) {
    ImGui::SameLine();
    const bool isSelected = plotLayout_ == layout;
    if (isSelected) {
      ImGui::PushStyleColor(ImGuiCol_Button,
          ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    }
    if (ImGui::Button(label)) {
      plotLayout_ = layout;
      noEmptySlotMessage_ = false;
    }
    if (isSelected) {
      ImGui::PopStyleColor();
    }
  };

  drawLayoutButton("List", MonitorPlotLayout::List);
  drawLayoutButton("1x1", MonitorPlotLayout::Grid1x1);
  drawLayoutButton("1x2", MonitorPlotLayout::Grid1x2);
  drawLayoutButton("2x2", MonitorPlotLayout::Grid2x2);
  drawLayoutButton("2x3", MonitorPlotLayout::Grid2x3);
  drawLayoutButton("3x3", MonitorPlotLayout::Grid3x3);
}

void MonitorView::DrawDisplayModeSelector() {
  ImGui::TextUnformatted("Mode:");
  const auto drawModeButton = [this](const char *label,
                                  MonitorDisplayMode mode) {
    ImGui::SameLine(0.0F, UI::Ui(2.0F));
    const bool isSelected = displayMode_ == mode;
    if (isSelected) {
      ImGui::PushStyleColor(ImGuiCol_Button,
          ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    }
    if (ImGui::Button(label)) {
      displayMode_ = mode;
    }
    if (isSelected) {
      ImGui::PopStyleColor();
    }
  };

  drawModeButton("Baseline", MonitorDisplayMode::Baseline);
  drawModeButton("Primary", MonitorDisplayMode::Primary);
  drawModeButton("Compare", MonitorDisplayMode::Compare);
}
} // namespace gui