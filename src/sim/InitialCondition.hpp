#pragma once

#include "common/math/Math.hpp"

#include <string>

namespace sim {
struct InitialCondition {
  double latitudeRad = 0.0;
  double longitudeRad = 0.0;
  double altitudeAslM = math::FeetToMeters(1000.0);

  double rollRad = 0.0;
  double pitchRad = 0.0;
  double headingRad = 0.0;

  double calibratedAirspeedMps = math::KnotsToMetersPerSecond(80.0);

  double pRadPerSec = 0.0;
  double qRadPerSec = 0.0;
  double rRadPerSec = 0.0;

  bool operator==(const InitialCondition &) const = default;
};

bool ValidateInitialCondition(const InitialCondition &initialCondition,
    std::string *errorMessage = nullptr);
} // namespace sim
