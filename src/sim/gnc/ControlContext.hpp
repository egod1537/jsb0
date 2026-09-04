#pragma once

#include "sim/gnc/control/legacy/PitchDynamics.hpp"
#include "sim/gnc/control/legacy/YawDynamics.hpp"

#include <optional>

namespace gnc {
struct ControlContext {
  std::optional<PitchDynamics> pitchDynamics;
  std::optional<YawDynamics> yawDynamics;
};
} // namespace gnc
