#pragma once

#include "gui/features/monitor/MonitorModel.hpp"

#include "sim/telemetry/TelemetryContracts.hpp"

#include <imgui.h>

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace gui::plotting {
struct TelemetryRenderSeries {
  std::string key;
  std::string displayLabel;
  std::string plotLabel;
  ImVec4 color;
  const telemetry::TelemetrySeries *sourceSeries = nullptr;
  std::vector<telemetry::TelemetrySample> samples;
  bool visible = true;
};

std::string MakeTelemetrySeriesDisplayLabel(std::string_view path,
    std::string_view sourceName);
std::string MakeTelemetrySeriesKey(std::string_view path,
    std::string_view sourceName);

bool IsTelemetrySeriesVisible(const MonitorPlotState &plot,
    std::string_view key);
void SetTelemetrySeriesVisible(MonitorPlotState &plot, std::string_view key,
    bool visible);

void DrawTelemetryLegend(MonitorPlotState &plot,
    std::vector<TelemetryRenderSeries> &seriesList);
void DrawTelemetryHoverTooltip(
    std::span<const TelemetryRenderSeries> seriesList, double timeSec,
    const std::string &valueUnit);
} // namespace gui::plotting
