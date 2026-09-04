#pragma once

#include "gui/features/monitor/MonitorConfig.hpp"
#include "gui/features/monitor/MonitorInput.hpp"
#include "gui/features/monitor/MonitorModel.hpp"

#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace gui::plotting {
struct MonitorPlotRenderContext {
  const MonitorConfig &config;
  const MonitorInput &sources;
  MonitorDisplayMode displayMode;
  MonitorTimeRange &visibleTimeRange;
  const std::vector<double> &sharedXAxisTicks;
  std::function<std::optional<double>()> drawOverlay;
};

class MonitorPlotRenderer {
public:
  static void Draw(MonitorPlotState &plot,
      const MonitorPlotRenderContext &context, float plotHeight);
};
} // namespace gui::plotting
