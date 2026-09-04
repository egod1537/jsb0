#include "gui/features/monitor/MonitorModel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gui {
bool IsValidMonitorManualYAxis(double minimum, double maximum) {
  return std::isfinite(minimum) && std::isfinite(maximum) && minimum < maximum;
}

std::optional<std::size_t> FindFirstEmptyMonitorPlotSlot(
    const MonitorState &state) {
  const std::size_t slotCount = GetMonitorPlotSlotCount(state.plotLayout);
  for (std::size_t slotIndex = 0; slotIndex < slotCount; ++slotIndex) {
    if (!state.customPlotSlots[slotIndex].has_value()) {
      return slotIndex;
    }
  }
  return std::nullopt;
}

bool AddMonitorPlotToSlot(MonitorState &state, MonitorPlotState plot,
    std::size_t slotIndex) {
  if (slotIndex >= GetMonitorPlotSlotCount(state.plotLayout)
      || state.customPlotSlots[slotIndex].has_value() || plot.channels.empty()
      || (plot.manualYAxis
          && !IsValidMonitorManualYAxis(plot.yAxisMinimum,
              plot.yAxisMaximum))) {
    return false;
  }

  plot.id = state.nextPlotId++;
  plot.custom = true;
  state.customPlotSlots[slotIndex] = plot.id;
  state.plots.push_back(std::move(plot));
  return true;
}

bool RemoveMonitorPlot(MonitorState &state, std::uint64_t plotId) {
  const auto plot = std::find_if(state.plots.begin(),
      state.plots.end(),
      [plotId](const MonitorPlotState &candidate) {
        return candidate.id == plotId && candidate.custom;
      });
  if (plot == state.plots.end()) {
    return false;
  }

  state.plots.erase(plot);
  for (std::optional<std::uint64_t> &slot : state.customPlotSlots) {
    if (slot == plotId) {
      slot.reset();
      break;
    }
  }
  return true;
}
} // namespace gui