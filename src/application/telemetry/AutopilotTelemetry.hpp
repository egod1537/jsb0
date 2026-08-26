#pragma once

#include <string_view>

namespace telemetry::paths {
inline constexpr std::string_view AutopilotRollHoldCommandedRoll =
    "autopilot/roll_hold/commanded_roll";
inline constexpr std::string_view AutopilotRollHoldAileronCommand =
    "autopilot/roll_hold/aileron_command";
} // namespace telemetry::paths
