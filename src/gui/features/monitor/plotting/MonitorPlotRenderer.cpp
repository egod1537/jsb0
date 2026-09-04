#include "gui/features/monitor/plotting/MonitorPlotRenderer.hpp"

#include "gui/features/monitor/catalog/MonitorPlotPresetCatalog.hpp"
#include "gui/features/monitor/plotting/MonitorPlotSeries.hpp"

#include "sim/telemetry/AutopilotTelemetry.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace gui::plotting {
namespace UI = FlightUI;

namespace {
constexpr std::size_t MinimumRenderedSamplesPerChannel = 512;
constexpr std::size_t MaximumRenderedSamplesPerChannel = 4096;
constexpr float TrackingToleranceBandAlpha = 0.30F;
constexpr float TrackingToleranceBoundaryAlpha = 0.68F;
constexpr float TrackingToleranceBoundaryDashLength = 5.0F;
constexpr float TrackingToleranceBoundaryGapLength = 3.0F;
constexpr float TrackingToleranceBoundaryThickness = 1.0F;

struct TrackingToleranceBandBoundary {
  const std::vector<telemetry::TelemetrySample> *samples = nullptr;
  double tolerance = 0.0;
  bool upper = false;
};

struct AcceptanceBandDefinition {
  DefaultTelemetryPlot plot;
  std::string_view commandPath;
  double tolerance;
  std::string_view idSuffix;
};

ImPlotPoint GetTrackingToleranceBandBoundaryPoint(int index,
    void *boundaryData) {
  const auto &boundary =
      *static_cast<const TrackingToleranceBandBoundary *>(boundaryData);
  const telemetry::TelemetrySample &sample =
      (*boundary.samples)[static_cast<std::size_t>(index)];
  return ImPlotPoint(sample.timeSec,
      sample.value
          + (boundary.upper ? boundary.tolerance : -boundary.tolerance));
}

void DrawDashedPlotLine(const std::vector<telemetry::TelemetrySample> &samples,
    double valueOffset, ImU32 color, float dashLength, float gapLength,
    float thickness) {
  if (samples.size() < 2) {
    return;
  }

  ImDrawList *drawList = ImPlot::GetPlotDrawList();
  const float patternLength = dashLength + gapLength;
  float patternOffset = 0.0F;
  for (std::size_t sampleIndex = 1; sampleIndex < samples.size();
      ++sampleIndex) {
    const telemetry::TelemetrySample &previous = samples[sampleIndex - 1];
    const telemetry::TelemetrySample &current = samples[sampleIndex];
    if (!std::isfinite(previous.timeSec) || !std::isfinite(previous.value)
        || !std::isfinite(current.timeSec) || !std::isfinite(current.value)) {
      continue;
    }

    const ImVec2 start =
        ImPlot::PlotToPixels(previous.timeSec, previous.value + valueOffset);
    const ImVec2 end =
        ImPlot::PlotToPixels(current.timeSec, current.value + valueOffset);
    const ImVec2 delta(end.x - start.x, end.y - start.y);
    const float segmentLength =
        std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (segmentLength <= 0.001F) {
      continue;
    }

    float segmentOffset = 0.0F;
    while (segmentOffset < segmentLength) {
      const float positionInPattern = std::fmod(patternOffset, patternLength);
      const bool drawing = positionInPattern < dashLength;
      const float runLength = drawing ? dashLength - positionInPattern
                                      : patternLength - positionInPattern;
      const float nextOffset =
          std::min(segmentLength, segmentOffset + runLength);
      if (drawing && nextOffset > segmentOffset) {
        const float startRatio = segmentOffset / segmentLength;
        const float endRatio = nextOffset / segmentLength;
        drawList->AddLine(ImVec2(start.x + delta.x * startRatio,
                              start.y + delta.y * startRatio),
            ImVec2(start.x + delta.x * endRatio, start.y + delta.y * endRatio),
            color,
            thickness);
      }
      const float advanced = nextOffset - segmentOffset;
      segmentOffset = nextOffset;
      patternOffset += advanced;
    }
  }
}

void DrawTrackingToleranceBand(
    const std::vector<telemetry::TelemetrySample> &samples, double tolerance,
    const ImVec4 &seriesColor, const char *bandId) {
  if (samples.size() < 2 || !std::isfinite(tolerance) || tolerance < 0.0) {
    return;
  }

  ImPlotSpec bandSpec;
  bandSpec.FillColor = seriesColor;
  bandSpec.FillAlpha = TrackingToleranceBandAlpha;
  bandSpec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
  TrackingToleranceBandBoundary upperBoundary{&samples, tolerance, true};
  TrackingToleranceBandBoundary lowerBoundary{&samples, tolerance, false};
  ImPlot::PlotShadedG(bandId,
      GetTrackingToleranceBandBoundaryPoint,
      &upperBoundary,
      GetTrackingToleranceBandBoundaryPoint,
      &lowerBoundary,
      static_cast<int>(samples.size()),
      bandSpec);

  ImVec4 boundaryColor = seriesColor;
  boundaryColor.w = TrackingToleranceBoundaryAlpha;
  const ImU32 packedBoundaryColor =
      ImGui::ColorConvertFloat4ToU32(boundaryColor);
  ImPlot::PushPlotClipRect();
  DrawDashedPlotLine(samples,
      tolerance,
      packedBoundaryColor,
      UI::Ui(TrackingToleranceBoundaryDashLength),
      UI::Ui(TrackingToleranceBoundaryGapLength),
      std::max(1.0F, UI::Ui(TrackingToleranceBoundaryThickness)));
  DrawDashedPlotLine(samples,
      -tolerance,
      packedBoundaryColor,
      UI::Ui(TrackingToleranceBoundaryDashLength),
      UI::Ui(TrackingToleranceBoundaryGapLength),
      std::max(1.0F, UI::Ui(TrackingToleranceBoundaryThickness)));
  ImPlot::PopPlotClipRect();
}

void DrawAcceptanceBand(const MonitorPlotState &plot,
    const MonitorPlotRenderContext &context,
    const AcceptanceBandDefinition &definition,
    std::size_t maximumRenderedSampleCount) {
  const std::string_view trackingPath =
      GetTelemetryPlotBinding(definition.plot).nodePath;
  if (plot.telemetryGroupPath != trackingPath) {
    return;
  }

  const auto commandChannelPath = std::find(plot.channels.begin(),
      plot.channels.end(),
      definition.commandPath);
  if (commandChannelPath == plot.channels.end()) {
    return;
  }
  const std::size_t commandChannelIndex = static_cast<std::size_t>(
      std::distance(plot.channels.begin(), commandChannelPath));

  const auto drawSource = [&](const telemetry::TelemetrySnapshot *snapshot,
                              std::string_view sourceName,
                              std::size_t sourceIndex) {
    if (snapshot == nullptr
        || !IsTelemetrySeriesVisible(plot,
            MakeTelemetrySeriesKey(definition.commandPath, sourceName))) {
      return;
    }
    const telemetry::TelemetrySeries *commandChannel =
        snapshot->Find(definition.commandPath);
    if (commandChannel == nullptr) {
      return;
    }
    const std::vector<telemetry::TelemetrySample> samples =
        telemetry::ReadTelemetrySamples(*commandChannel,
            context.visibleTimeRange.minSec,
            context.visibleTimeRange.maxSec,
            maximumRenderedSampleCount);
    if (samples.size() < 2) {
      return;
    }
    const int colorIndex = static_cast<int>(
        sourceIndex * plot.channels.size() + commandChannelIndex);
    const std::string bandId =
        "##" + std::string(sourceName) + std::string(definition.idSuffix);
    DrawTrackingToleranceBand(samples,
        definition.tolerance,
        ImPlot::GetColormapColor(colorIndex),
        bandId.c_str());
  };

  if (MonitorDisplaysPrimary(context.displayMode)) {
    drawSource(context.sources.primary.get(), "Primary", 0);
  }
  if (MonitorDisplaysBaseline(context.displayMode)) {
    drawSource(context.sources.baseline.get(), "Baseline", 1);
  }
}

void DrawAcceptanceBands(const MonitorPlotState &plot,
    const MonitorPlotRenderContext &context,
    std::size_t maximumRenderedSampleCount) {
  const std::array definitions{
      AcceptanceBandDefinition{DefaultTelemetryPlot::CourseHoldCourseTracking,
          telemetry::paths::AutopilotCourseHoldCommandedCourse,
          context.config.courseTrackingToleranceDeg,
          "CommandedCourseToleranceBand"},
      AcceptanceBandDefinition{DefaultTelemetryPlot::RollHoldRollTracking,
          telemetry::paths::AutopilotRollHoldCommandedRoll,
          context.config.rollTrackingToleranceDeg,
          "CommandedRollToleranceBand"},
      AcceptanceBandDefinition{DefaultTelemetryPlot::PitchHoldPitchTracking,
          telemetry::paths::AutopilotPitchHoldCommandedPitch,
          context.config.pitchTrackingToleranceDeg,
          "CommandedPitchToleranceBand"},
  };
  for (const AcceptanceBandDefinition &definition : definitions) {
    DrawAcceptanceBand(plot, context, definition, maximumRenderedSampleCount);
  }
}
} // namespace

void MonitorPlotRenderer::Draw(MonitorPlotState &plot,
    const MonitorPlotRenderContext &context, float plotHeight) {
  const float availablePlotWidth = ImGui::GetContentRegionAvail().x;
  const std::size_t maximumRenderedSampleCount = std::clamp(
      static_cast<std::size_t>(std::max(0.0F, availablePlotWidth) * 2.0F),
      MinimumRenderedSamplesPerChannel,
      MaximumRenderedSamplesPerChannel);

  std::vector<TelemetryRenderSeries> renderedSeries;
  renderedSeries.reserve(plot.channels.size() * 2);
  const auto addSource = [&](const telemetry::TelemetrySnapshot *snapshot,
                             std::string_view sourceName,
                             std::size_t sourceIndex) {
    if (snapshot == nullptr) {
      return;
    }
    for (std::size_t channelIndex = 0; channelIndex < plot.channels.size();
        ++channelIndex) {
      const std::string &path = plot.channels[channelIndex];
      const telemetry::TelemetrySeries *channel = snapshot->Find(path);
      if (channel == nullptr || channel->samples.empty()) {
        continue;
      }
      std::vector<telemetry::TelemetrySample> samples =
          telemetry::ReadTelemetrySamples(*channel,
              context.visibleTimeRange.minSec,
              context.visibleTimeRange.maxSec,
              maximumRenderedSampleCount);
      if (samples.empty()) {
        continue;
      }
      const std::string key = MakeTelemetrySeriesKey(path, sourceName);
      const std::string displayLabel =
          MakeTelemetrySeriesDisplayLabel(path, sourceName);
      const int colorIndex =
          static_cast<int>(sourceIndex * plot.channels.size() + channelIndex);
      renderedSeries.push_back(TelemetryRenderSeries{key,
          displayLabel,
          displayLabel + "##" + key,
          ImPlot::GetColormapColor(colorIndex),
          channel,
          std::move(samples),
          IsTelemetrySeriesVisible(plot, key)});
    }
  };
  if (MonitorDisplaysPrimary(context.displayMode)) {
    addSource(context.sources.primary.get(), "Primary", 0);
  }
  if (MonitorDisplaysBaseline(context.displayMode)) {
    addSource(context.sources.baseline.get(), "Baseline", 1);
  }

  if (plot.showLegend) {
    DrawTelemetryLegend(plot, renderedSeries);
  }

  UI::PlotBuilder plotBuilder =
      UI::Plot("##TelemetryPlot" + std::to_string(plot.id))
          .Height(plotHeight)
          .Flags(ImPlotFlags_NoTitle | ImPlotFlags_NoInputs
                 | ImPlotFlags_NoBoxSelect)
          .LegendVisible(false)
          .XAxisLinks(context.visibleTimeRange.minSec,
              context.visibleTimeRange.maxSec)
          .XAxisTicks(context.sharedXAxisTicks)
          .XAxisLabel("Time (s)")
          .YAxisLabel(plot.yAxisLabel);
  if (plot.manualYAxis) {
    plotBuilder.YAxisLimitsAlways(plot.yAxisMinimum, plot.yAxisMaximum);
  } else {
    plotBuilder.FocusedYAxis();
  }
  plotBuilder.Underlay([&plot, &context, maximumRenderedSampleCount] {
    DrawAcceptanceBands(plot, context, maximumRenderedSampleCount);
  });

  for (const TelemetryRenderSeries &series : renderedSeries) {
    if (!series.visible) {
      continue;
    }
    const telemetry::TelemetrySample *data = series.samples.data();
    plotBuilder.AddLine(series.plotLabel,
        UI::DataView(&data->timeSec,
            series.samples.size(),
            sizeof(telemetry::TelemetrySample)),
        UI::DataView(&data->value,
            series.samples.size(),
            sizeof(telemetry::TelemetrySample)),
        series.color);
  }

  plotBuilder.Overlay([&plot, &context, &renderedSeries] {
    const std::optional<double> hoverTimeSec = context.drawOverlay();
    if (hoverTimeSec) {
      DrawTelemetryHoverTooltip(renderedSeries, *hoverTimeSec, plot.yAxisLabel);
    }
  });
  UI::UIElement plotElement = plotBuilder;
  plotElement.Render();

  if (plot.channels.empty()) {
    ImGui::TextDisabled("No channels assigned.");
  } else if (renderedSeries.empty()) {
    ImGui::TextDisabled("Waiting for assigned telemetry channels.");
  }
}
} // namespace gui::plotting
