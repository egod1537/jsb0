#pragma once

#include <string_view>

namespace opts {

namespace simulation {
inline constexpr std::string_view AircraftName = "c172x";
inline constexpr double Hz = 120.0;
inline constexpr double DtSec = 1.0 / Hz;
} // namespace simulation

namespace gui {
inline constexpr int WindowWidth = 1280;
inline constexpr int WindowHeight = 720;
inline constexpr std::string_view WindowTitle = "JSB Flight Console";
inline constexpr double RenderHz = 60.0;
inline constexpr double RenderDtSec = 1.0 / RenderHz;
} // namespace gui

namespace debug {
inline constexpr bool PrintAircraftState = false;
inline constexpr bool PrintControllerOutput = false;
} // namespace debug

namespace experiment {
inline constexpr bool ValidateLinearModel = false;
inline constexpr double ValidationIntervalSec = 0.2;
} // namespace experiment

} // namespace opts
