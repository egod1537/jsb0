#pragma once

namespace gnc {
struct LateralSetpoint {
  double courseRad = 0.0;
  double rollRad = 0.0;
};

struct LongitudinalSetpoint {
  double altitudeAglM = 0.0;
  double calibratedAirspeedMps = 0.0;
  double pitchRad = 0.0;
};

struct FixedWingSetpoint {
  LateralSetpoint lateral;
  LongitudinalSetpoint longitudinal;
};
} // namespace gnc
