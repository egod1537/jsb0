#include "gui/features/monitor/plotting/MonitorPlotSeries.hpp"

#include "gui/features/monitor/MonitorSignalCatalog.hpp"

#include "flightui/FlightUI.hpp"

#include <algorithm>
#include <optional>

namespace gui::plotting {

namespace {
constexpr float LegendMarkerRadius = 3.5F;
constexpr float LegendMarkerTextSpacing = 6.0F;
constexpr float LegendItemHorizontalPadding = 3.0F;
constexpr float LegendItemSpacing = 10.0F;
constexpr float LegendPlotSpacing = 4.0F;
} // namespace

std::string MakeTelemetrySeriesDisplayLabel(std::string_view path,
    std::string_view sourceName) {
  return std::string(sourceName) + " · " + GetMonitorSignalDisplayName(path);
}

std::string MakeTelemetrySeriesKey(std::string_view path,
    std::string_view sourceName) {
  return std::string(sourceName) + "/" + std::string(path);
}

bool IsTelemetrySeriesVisible(const MonitorPlotState &plot,
    std::string_view key) {
  return std::find(plot.hiddenSeries.begin(), plot.hiddenSeries.end(), key)
         == plot.hiddenSeries.end();
}

void SetTelemetrySeriesVisible(MonitorPlotState &plot, std::string_view key,
    bool visible) {
  const auto hidden =
      std::find(plot.hiddenSeries.begin(), plot.hiddenSeries.end(), key);
  if (visible) {
    if (hidden != plot.hiddenSeries.end()) {
      plot.hiddenSeries.erase(hidden);
    }
  } else if (hidden == plot.hiddenSeries.end()) {
    plot.hiddenSeries.emplace_back(key);
  }
}

void DrawTelemetryLegend(MonitorPlotState &plot,
    std::vector<TelemetryRenderSeries> &seriesList) {
  if (seriesList.empty()) {
    return;
  }

  const float availableWidth = std::max(1.0F, ImGui::GetContentRegionAvail().x);
  const float markerRadius = ui::Ui(LegendMarkerRadius);
  const float markerDiameter = markerRadius * 2.0F;
  const float markerTextSpacing = ui::Ui(LegendMarkerTextSpacing);
  const float horizontalPadding = ui::Ui(LegendItemHorizontalPadding);
  const float itemSpacing = ui::Ui(LegendItemSpacing);
  const float itemHeight = std::max(ImGui::GetTextLineHeight(), markerDiameter)
                           + ImGui::GetStyle().FramePadding.y * 2.0F;
  float usedWidth = 0.0F;

  for (std::size_t index = 0; index < seriesList.size(); ++index) {
    TelemetryRenderSeries &series = seriesList[index];
    const float textWidth = ImGui::CalcTextSize(series.displayLabel.c_str()).x;
    const float desiredWidth = horizontalPadding * 2.0F + markerDiameter
                               + markerTextSpacing + textWidth;
    const float itemWidth = std::min(desiredWidth, availableWidth);
    const bool wrap =
        index > 0 && usedWidth + itemSpacing + itemWidth > availableWidth;
    if (wrap) {
      usedWidth = 0.0F;
    } else if (index > 0) {
      ImGui::SameLine(0.0F, itemSpacing);
      usedWidth += itemSpacing;
    }

    ImGui::PushID(series.key.c_str());
    ImGui::InvisibleButton("##TelemetryLegendItem",
        ImVec2(itemWidth, itemHeight));
    const bool hovered = ImGui::IsItemHovered();
    if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
      series.visible = !series.visible;
      SetTelemetrySeriesVisible(plot, series.key, series.visible);
    }
    if (hovered) {
      ImGui::SetTooltip("%s\n%s",
          series.displayLabel.c_str(),
          series.visible ? "Click to hide this series"
                         : "Click to show this series");
    }

    const ImVec2 itemMin = ImGui::GetItemRectMin();
    const ImVec2 itemMax = ImGui::GetItemRectMax();
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    if (hovered) {
      drawList->AddRectFilled(itemMin,
          itemMax,
          ImGui::GetColorU32(ImGuiCol_HeaderHovered),
          ImGui::GetStyle().FrameRounding);
    }

    ImVec4 markerColor = series.color;
    if (!series.visible) {
      markerColor.w *= 0.35F;
    }
    const ImVec2 markerCenter(itemMin.x + horizontalPadding + markerRadius,
        (itemMin.y + itemMax.y) * 0.5F);
    drawList->AddCircleFilled(markerCenter,
        markerRadius,
        ImGui::ColorConvertFloat4ToU32(markerColor));
    const ImVec2 textPosition(itemMin.x + horizontalPadding + markerDiameter
                                  + markerTextSpacing,
        itemMin.y + (itemHeight - ImGui::GetTextLineHeight()) * 0.5F);
    drawList->PushClipRect(itemMin, itemMax, true);
    drawList->AddText(textPosition,
        ImGui::GetColorU32(
            series.visible ? ImGuiCol_Text : ImGuiCol_TextDisabled),
        series.displayLabel.c_str());
    drawList->PopClipRect();
    ImGui::PopID();
    usedWidth += itemWidth;
  }

  ImGui::Dummy(ImVec2(0.0F, ui::Ui(LegendPlotSpacing)));
}

void DrawTelemetryHoverTooltip(
    std::span<const TelemetryRenderSeries> seriesList, double timeSec,
    const std::string &valueUnit) {
  ImGui::BeginTooltip();
  ImGui::Text("Time: %.3f s", timeSec);
  ImGui::Separator();

  for (const TelemetryRenderSeries &series : seriesList) {
    if (!series.visible || series.sourceSeries == nullptr) {
      continue;
    }

    const std::optional<telemetry::TelemetrySample> sample =
        telemetry::FindClosestTelemetrySample(*series.sourceSeries, timeSec);
    if (!sample) {
      continue;
    }

    ImGui::PushID(series.key.c_str());
    ImGui::ColorButton("##SeriesColor",
        series.color,
        ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
        ImVec2(ui::Ui(9.0F), ui::Ui(9.0F)));
    ImGui::SameLine();
    if (valueUnit.empty()) {
      ImGui::Text("%s: %.6g", series.displayLabel.c_str(), sample->value);
    } else {
      ImGui::Text("%s: %.6g %s",
          series.displayLabel.c_str(),
          sample->value,
          valueUnit.c_str());
    }
    ImGui::PopID();
  }

  ImGui::EndTooltip();
}
} // namespace gui::plotting
