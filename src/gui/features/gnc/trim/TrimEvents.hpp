#pragma once

#include "sim/gnc/TrimTypes.hpp"

namespace gui {
struct TrimRequested {
  gnc::TrimRequest request;
  bool fromCurrentState = false;
};

enum class TrimRequestField {
  Mode,
  CalibratedAirspeedMps,
  AltitudeAslM,
  FlightPathAngleRad,
};

struct TrimRequestValueChanged {
  TrimRequestField field = TrimRequestField::Mode;
  double value = 0.0;
};

struct TrimExecutionRequested {
  bool fromCurrentState = false;
};

struct TrimViewStateChanged {
  bool resultOpen = true;
  bool residualOpen = true;
};
} // namespace gui
