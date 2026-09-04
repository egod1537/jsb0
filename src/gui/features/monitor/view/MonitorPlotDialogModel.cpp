#include "gui/features/monitor/view/MonitorPlotDialogModel.hpp"

#include <cstdio>

namespace gui {
void MonitorPlotDialogModel::BeginAdd(std::optional<std::size_t> slotIndex) {
  title.fill('\0');
  search.fill('\0');
  selectedSignalIds.clear();
  selectedTemplateId.clear();
  validationMessage.clear();
  targetSlot = slotIndex;
  editingPlotId.reset();
  yAxisMinimum = 0.0;
  yAxisMaximum = 1.0;
  manualYAxis = false;
  showLegend = true;
  openRequested = true;
}

void MonitorPlotDialogModel::BeginEdit(const MonitorPlotState &plot) {
  std::snprintf(title.data(), title.size(), "%s", plot.title.c_str());
  search.fill('\0');
  selectedSignalIds = plot.channels;
  targetSlot.reset();
  editingPlotId = plot.id;
  selectedTemplateId = plot.templateId;
  validationMessage.clear();
  yAxisMinimum = plot.yAxisMinimum;
  yAxisMaximum = plot.yAxisMaximum;
  manualYAxis = plot.manualYAxis;
  showLegend = plot.showLegend;
  openRequested = true;
}

void MonitorPlotDialogModel::Close() {
  targetSlot.reset();
  editingPlotId.reset();
  focusSearch = false;
}
} // namespace gui
