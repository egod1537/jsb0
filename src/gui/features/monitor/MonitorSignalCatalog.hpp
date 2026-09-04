#pragma once

#include "gui/features/monitor/MonitorModel.hpp"

#include "sim/telemetry/TelemetryContracts.hpp"

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace gui {
struct MonitorSignalDescriptor {
  std::string id;
  std::string name;
  std::string symbol;
  std::string unit;
  std::string group;
  std::string subgroup;
};

std::string GetMonitorSignalDisplayName(std::string_view signalId);
std::vector<MonitorSignalDescriptor> BuildMonitorSignalCatalog(
    std::span<const std::string_view> channelPaths,
    std::span<const MonitorPlotState> plotTemplates);
std::vector<MonitorSignalDescriptor> BuildMonitorSignalCatalog(
    std::span<const telemetry::TelemetrySignalMetadata> metadata,
    std::span<const MonitorPlotState> plotTemplates);
std::vector<const MonitorSignalDescriptor *> FilterMonitorSignalCatalog(
    std::span<const MonitorSignalDescriptor> catalog, std::string_view search);
const MonitorSignalDescriptor *FindMonitorSignal(
    std::span<const MonitorSignalDescriptor> catalog, std::string_view id);
bool CanAddMonitorSignal(std::span<const MonitorSignalDescriptor> catalog,
    std::span<const std::string> selectedSignalIds,
    std::string_view candidateSignalId);
std::string ResolveMonitorPlotTitle(std::string_view requestedTitle,
    std::span<const std::string> selectedSignalIds,
    std::span<const MonitorSignalDescriptor> catalog);
} // namespace gui
