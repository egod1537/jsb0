#include "gui/features/monitor/MonitorView.hpp"

#include "gui/features/monitor/plotting/MonitorPlotRenderer.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <algorithm>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float PlotHeight = 245.0F;
constexpr float MinimumGridPlotHeight = 105.0F;
constexpr float WorkspaceSpacing = 8.0F;
constexpr float PlotCardTopMargin = 3.0F;
constexpr float PlotTitleFrameSpacing = 5.0F;
constexpr float PlotCardBottomMargin = 12.0F;
constexpr float PlotGridCellPadding = 4.0F;
} // namespace

void MonitorView::DrawPlotList(const TelemetrySources &sources) {
  DrawPlotTable(sources, 1, PlotHeight, "MonitorPlotList");
}

void MonitorView::DrawPlotGrid(const TelemetrySources &sources, int dimension) {
  const char *tableId =
      dimension == 2 ? "MonitorPlotGrid2x2" : "MonitorPlotGrid3x3";
  DrawPlotTable(sources,
      dimension,
      CalculateGridPlotHeight(dimension),
      tableId);
}

void MonitorView::DrawPlotTable(const TelemetrySources &sources,
    int columnCount, float plotHeight, const char *tableId) {
  const bool listLayout = plotLayout_ == MonitorPlotLayout::List;
  if (listLayout
      && std::none_of(plots_.begin(), plots_.end(), [this](const auto &plot) {
           return IsPlotVisible(plot);
         })) {
    ImGui::TextDisabled("Select a Monitor Preset in the left panel.");
    return;
  }

  constexpr ImGuiTableFlags Flags =
      ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoSavedSettings;
  ImGui::PushStyleVar(ImGuiStyleVar_CellPadding,
      ImVec2(UI::Ui(PlotGridCellPadding), 0.0F));
  if (!ImGui::BeginTable(tableId, columnCount, Flags)) {
    ImGui::PopStyleVar();
    return;
  }
  for (int column = 0; column < columnCount; ++column) {
    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch);
  }

  std::size_t visibleIndex = 0;
  for (MonitorPlot &plot : plots_) {
    if ((!listLayout && plot.custom) || !IsPlotVisible(plot)) {
      continue;
    }

    if (visibleIndex % static_cast<std::size_t>(columnCount) == 0) {
      ImGui::TableNextRow();
    }
    ImGui::TableNextColumn();
    DrawPlotCard(plot, sources, plotHeight);
    ++visibleIndex;
  }

  const std::size_t slotCount = listLayout ? 0 : GetAvailablePlotSlotCount();
  for (std::size_t slotIndex = 0; slotIndex < slotCount; ++slotIndex) {
    if (visibleIndex % static_cast<std::size_t>(columnCount) == 0) {
      ImGui::TableNextRow();
    }
    ImGui::TableNextColumn();
    const std::optional<std::uint64_t> plotId =
        renderState_.customPlotSlots[slotIndex];
    if (plotId) {
      if (MonitorPlot *plot = FindPlot(*plotId)) {
        DrawPlotCard(*plot, sources, plotHeight);
      } else {
        renderState_.customPlotSlots[slotIndex].reset();
        DrawEmptyPlotSlot(slotIndex, plotHeight);
      }
    } else {
      DrawEmptyPlotSlot(slotIndex, plotHeight);
    }
    ++visibleIndex;
  }
  ImGui::EndTable();
  ImGui::PopStyleVar();

  if (plotToRemove_) {
    DeletePlot(*plotToRemove_);
    plotToRemove_.reset();
  }
}

void MonitorView::DrawEmptyPlotSlot(std::size_t slotIndex, float plotHeight) {
  ImGui::PushID(static_cast<int>(slotIndex));
  const float slotHeight = UI::Ui(plotHeight) + ImGui::GetTextLineHeight()
                           + UI::Ui(PlotTitleFrameSpacing);
  if (ImGui::BeginChild("EmptyPlotSlot",
          ImVec2(0.0F, slotHeight),
          true,
          ImGuiWindowFlags_NoScrollbar)) {
    const char *label = "+ Add Plot";
    const ImVec2 textSize = ImGui::CalcTextSize(label);
    const ImVec2 buttonSize(textSize.x
                                + ImGui::GetStyle().FramePadding.x * 2.0F,
        textSize.y + ImGui::GetStyle().FramePadding.y * 2.0F);
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 cursor = ImGui::GetCursorPos();
    ImGui::SetCursorPos(
        ImVec2(cursor.x + std::max(0.0F, (available.x - buttonSize.x) * 0.5F),
            cursor.y + std::max(0.0F, (available.y - buttonSize.y) * 0.5F)));
    if (ImGui::Button(label)) {
      RequestAddPlot(slotIndex);
    }
  }
  ImGui::EndChild();
  ImGui::PopID();
}

float MonitorView::CalculateGridPlotHeight(int rowCount) const {
  const float availableHeight = ImGui::GetContentRegionAvail().y;
  const float cardChromeHeight =
      UI::Ui(PlotCardTopMargin) + ImGui::GetTextLineHeight()
      + UI::Ui(PlotTitleFrameSpacing) + UI::Ui(PlotCardBottomMargin);
  const float plotHeightPixels =
      availableHeight / static_cast<float>(rowCount) - cardChromeHeight;
  const float uiScale = std::max(UI::GetUIScale(), 0.001F);
  return std::max(MinimumGridPlotHeight, plotHeightPixels / uiScale);
}

void MonitorView::DrawPlotCard(MonitorPlot &plot,
    const TelemetrySources &sources, float plotHeight) {
  ImGui::PushID(static_cast<int>(plot.id));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
      ImVec2(UI::Ui(WorkspaceSpacing), 0.0F));
  ImGui::BeginGroup();

  ImGui::Dummy(ImVec2(0.0F, UI::Ui(PlotCardTopMargin)));

  ImGui::TextUnformatted(plot.title.c_str());
  if (plot.custom) {
    const float controlsWidth = ImGui::CalcTextSize("Edit").x
                                + ImGui::CalcTextSize("x").x
                                + ImGui::GetStyle().FramePadding.x * 4.0F
                                + ImGui::GetStyle().ItemSpacing.x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX(),
        ImGui::GetWindowContentRegionMax().x - controlsWidth));
    if (ImGui::SmallButton("Edit")) {
      RequestEditPlot(plot);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Edit plot");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("x")) {
      plotToRemove_ = plot.id;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Remove plot");
    }
  }
  ImGui::Dummy(ImVec2(0.0F, UI::Ui(PlotTitleFrameSpacing)));
  plotting::MonitorPlotRenderer::Draw(plot,
      {.config = config_,
          .sources = sources,
          .displayMode = displayMode_,
          .visibleTimeRange = visibleTimeRange_,
          .sharedXAxisTicks = sharedXAxisTicks_,
          .drawOverlay = [this] { return DrawPlotOverlay(); }},
      plotHeight);
  ImGui::Dummy(ImVec2(0.0F, UI::Ui(PlotCardBottomMargin)));

  ImGui::EndGroup();
  ImGui::PopStyleVar();
  ImGui::PopID();
}
} // namespace gui