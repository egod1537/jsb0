#pragma once

#include "sim/runtime/SimContracts.hpp"

namespace gui {
struct PrimaryRollHoldConfigChanged {
  sim::PrimaryRollHoldConfig config;
};

enum class PrimaryRollHoldField {
  Enabled,
  TargetDeg,
  AngleProportionalGain,
  RateProportionalGain,
};

struct PrimaryRollHoldValueChanged {
  PrimaryRollHoldField field = PrimaryRollHoldField::Enabled;
  double value = 0.0;
};

struct ExperimentalViewStateChanged {
  bool primaryParametersOpen = true;
};
} // namespace gui
