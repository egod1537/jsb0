#pragma once

#include "common/math/Math.hpp"

#include <string>

namespace gnc {
enum class TrimMode {
  Longitudinal,
  Full,
  Ground,
};

struct TrimRequest {
  TrimMode mode = TrimMode::Longitudinal;

  double calibratedAirspeedMps = math::KnotsToMetersPerSecond(100.0);
  double altitudeAslM = math::FeetToMeters(3000.0);
  double flightPathAngleRad = 0.0;
};

struct TrimResult {
  bool success = false;
  std::string message;

  double alphaRad = 0.0;
  double betaRad = 0.0;
  double rollRad = 0.0;
  double pitchRad = 0.0;

  double throttle = 0.0;
  double elevator = 0.0;
  double pitchTrim = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;

  double uDotMps2 = 0.0;
  double vDotMps2 = 0.0;
  double wDotMps2 = 0.0;

  double pDotRadPerSec2 = 0.0;
  double qDotRadPerSec2 = 0.0;
  double rDotRadPerSec2 = 0.0;
};
} // namespace gnc
