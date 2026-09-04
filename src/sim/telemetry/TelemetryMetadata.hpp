#pragma once

#include "sim/telemetry/TelemetryContracts.hpp"

#include <string_view>

namespace telemetry {
TelemetrySignalMetadata ResolveTelemetrySignalMetadata(std::string_view path);
} // namespace telemetry
