#include "gui/features/monitor/MonitorSignalCatalog.hpp"

#include "sim/telemetry/TelemetryMetadata.hpp"

#include <algorithm>
#include <cctype>
#include <tuple>

namespace gui {
namespace {
std::string ToDisplayName(std::string_view value) {
  std::string result;
  result.reserve(value.size());
  bool capitalize = true;
  for (const char character : value) {
    if (character == '_' || character == '-') {
      result.push_back(' ');
      capitalize = true;
      continue;
    }
    result.push_back(capitalize ? static_cast<char>(std::toupper(
                                      static_cast<unsigned char>(character)))
                                : character);
    capitalize = false;
  }
  return result;
}

std::string ToLower(std::string_view value) {
  std::string result(value);
  std::transform(result.begin(), result.end(), result.begin(), [](char value) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
  });
  return result;
}

bool ContainsCaseInsensitive(std::string_view value, std::string_view search) {
  return ToLower(value).find(ToLower(search)) != std::string::npos;
}
} // namespace

std::string GetMonitorSignalDisplayName(std::string_view signalId) {
  return telemetry::ResolveTelemetrySignalMetadata(signalId).displayName;
}

std::vector<MonitorSignalDescriptor> BuildMonitorSignalCatalog(
    std::span<const std::string_view> channelPaths,
    std::span<const MonitorPlotState> plotTemplates) {
  std::vector<telemetry::TelemetrySignalMetadata> metadata;
  metadata.reserve(channelPaths.size());
  for (const std::string_view path : channelPaths) {
    metadata.push_back(telemetry::ResolveTelemetrySignalMetadata(path));
  }
  return BuildMonitorSignalCatalog(metadata, plotTemplates);
}

std::vector<MonitorSignalDescriptor> BuildMonitorSignalCatalog(
    std::span<const telemetry::TelemetrySignalMetadata> metadata,
    std::span<const MonitorPlotState> plotTemplates) {
  std::vector<MonitorSignalDescriptor> catalog;
  catalog.reserve(metadata.size());
  for (const telemetry::TelemetrySignalMetadata &signalMetadata : metadata) {
    const std::string_view path = signalMetadata.path;
    const std::size_t nameSeparator = path.rfind('/');
    const std::size_t groupSeparator = path.find('/');
    const std::string_view rawGroup = path.substr(0, groupSeparator);
    std::string_view rawSubgroup;
    if (groupSeparator != std::string_view::npos
        && nameSeparator != std::string_view::npos
        && groupSeparator < nameSeparator) {
      rawSubgroup =
          path.substr(groupSeparator + 1, nameSeparator - groupSeparator - 1);
    }

    std::string unit = signalMetadata.unit;
    if (unit.empty()) {
      for (const MonitorPlotState &plot : plotTemplates) {
        if (plot.custom
            || std::find(plot.channels.begin(), plot.channels.end(), path)
                   == plot.channels.end()) {
          continue;
        }
        unit = plot.yAxisLabel == "Value" ? std::string{} : plot.yAxisLabel;
        break;
      }
    }

    catalog.push_back({.id = std::string(path),
        .name = signalMetadata.displayName.empty()
                    ? GetMonitorSignalDisplayName(path)
                    : signalMetadata.displayName,
        .symbol = signalMetadata.symbol,
        .unit = std::move(unit),
        .group = ToDisplayName(rawGroup),
        .subgroup = ToDisplayName(rawSubgroup)});
  }
  std::sort(catalog.begin(),
      catalog.end(),
      [](const MonitorSignalDescriptor &left,
          const MonitorSignalDescriptor &right) {
        return std::tie(left.group, left.subgroup, left.name, left.id)
               < std::tie(right.group, right.subgroup, right.name, right.id);
      });
  catalog.erase(std::unique(catalog.begin(),
                    catalog.end(),
                    [](const MonitorSignalDescriptor &left,
                        const MonitorSignalDescriptor &right) {
                      return left.id == right.id;
                    }),
      catalog.end());
  return catalog;
}

std::vector<const MonitorSignalDescriptor *> FilterMonitorSignalCatalog(
    std::span<const MonitorSignalDescriptor> catalog, std::string_view search) {
  std::vector<const MonitorSignalDescriptor *> result;
  for (const MonitorSignalDescriptor &signal : catalog) {
    if (search.empty() || ContainsCaseInsensitive(signal.name, search)
        || ContainsCaseInsensitive(signal.id, search)
        || ContainsCaseInsensitive(signal.symbol, search)
        || ContainsCaseInsensitive(signal.unit, search)) {
      result.push_back(&signal);
    }
  }
  return result;
}

const MonitorSignalDescriptor *FindMonitorSignal(
    std::span<const MonitorSignalDescriptor> catalog, std::string_view id) {
  const auto signal = std::find_if(catalog.begin(),
      catalog.end(),
      [id](const auto &candidate) { return candidate.id == id; });
  return signal == catalog.end() ? nullptr : &*signal;
}

bool CanAddMonitorSignal(std::span<const MonitorSignalDescriptor> catalog,
    std::span<const std::string> selectedSignalIds,
    std::string_view candidateSignalId) {
  const MonitorSignalDescriptor *candidate =
      FindMonitorSignal(catalog, candidateSignalId);
  if (candidate == nullptr || candidate->unit.empty()) {
    return candidate != nullptr;
  }
  for (const std::string &selectedId : selectedSignalIds) {
    const MonitorSignalDescriptor *selected =
        FindMonitorSignal(catalog, selectedId);
    if (selected != nullptr && !selected->unit.empty()) {
      return selected->unit == candidate->unit;
    }
  }
  return true;
}

std::string ResolveMonitorPlotTitle(std::string_view requestedTitle,
    std::span<const std::string> selectedSignalIds,
    std::span<const MonitorSignalDescriptor> catalog) {
  const bool blank =
      std::all_of(requestedTitle.begin(), requestedTitle.end(), [](char value) {
        return std::isspace(static_cast<unsigned char>(value)) != 0;
      });
  if (!blank) {
    return std::string(requestedTitle);
  }
  if (selectedSignalIds.empty()) {
    return {};
  }
  const MonitorSignalDescriptor *signal =
      FindMonitorSignal(catalog, selectedSignalIds.front());
  return signal == nullptr ? selectedSignalIds.front() : signal->name;
}
} // namespace gui
