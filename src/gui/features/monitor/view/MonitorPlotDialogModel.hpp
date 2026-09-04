#pragma once

#include "gui/features/monitor/MonitorModel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace gui {
struct MonitorPlotDialogModel {
  static constexpr std::size_t TitleCapacity = 128;
  static constexpr std::size_t SearchCapacity = 160;

  std::array<char, TitleCapacity> title{};
  std::array<char, SearchCapacity> search{};
  std::vector<std::string> selectedSignalIds;
  std::optional<std::size_t> targetSlot;
  std::optional<std::uint64_t> editingPlotId;
  std::string selectedTemplateId;
  std::string validationMessage;
  double yAxisMinimum = 0.0;
  double yAxisMaximum = 1.0;
  bool manualYAxis = false;
  bool showLegend = true;
  bool openRequested = false;
  bool focusSearch = false;

  void BeginAdd(std::optional<std::size_t> slotIndex);
  void BeginEdit(const MonitorPlotState &plot);
  void Close();
};
} // namespace gui
