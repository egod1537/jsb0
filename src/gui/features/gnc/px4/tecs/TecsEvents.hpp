#pragma once

#include "sim/gnc/tecs/Px4TecsParameterMetadata.hpp"

namespace gui {
enum class BaselineTecsField {
  Enabled,
  TargetAltitudeM,
  TargetAirspeedMps,
};

struct BaselineTecsValueChanged {
  BaselineTecsField field = BaselineTecsField::Enabled;
  double value = 0.0;
};

struct BaselineTecsParameterChanged {
  gnc::Px4TecsParameter parameter = gnc::Px4TecsParameter::MinimumPitch;
  double value = 0.0;
};

struct BaselineTecsTuningResetRequested {};

struct BaselineTecsAltitudeCaptureRequested {
  double currentAltitudeAglM = 0.0;
};

struct BaselineTecsAirspeedCaptureRequested {
  double currentCalibratedAirspeedMps = 0.0;
};
} // namespace gui
