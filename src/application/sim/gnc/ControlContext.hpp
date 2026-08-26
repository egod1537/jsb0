#pragma once

#include "application/sim/gnc/hold/PitchDynamics.hpp"
#include "application/sim/gnc/hold/RollDynamics.hpp"
#include "application/sim/gnc/hold/RollHoldSettings.hpp"
#include "application/sim/gnc/hold/YawDynamics.hpp"

#include <optional>

namespace gnc {
struct ControlContext {
  std::optional<RollDynamics> rollDynamics;
  std::optional<PitchDynamics> pitchDynamics;
  std::optional<YawDynamics> yawDynamics;
  std::optional<RollHoldSettings> rollHoldSettings;
};
} // namespace gnc
