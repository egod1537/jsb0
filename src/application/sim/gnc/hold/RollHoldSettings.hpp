#pragma once

namespace gnc {
struct RollHoldSettings {
  double targetRollRad{};
  double dampingRatio{};
  double naturalFrequencyRadPerSec{};
};
} // namespace gnc
