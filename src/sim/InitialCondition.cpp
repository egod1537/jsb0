#include "sim/InitialCondition.hpp"

#include <cmath>
#include <numbers>

namespace {
bool ValidationFailed(std::string *errorMessage, const char *message) {
  if (errorMessage != nullptr) {
    *errorMessage = message;
  }

  return false;
}
} // namespace

namespace sim {
bool ValidateInitialCondition(const InitialCondition &initialCondition,
    std::string *errorMessage) {
  if (errorMessage != nullptr) {
    errorMessage->clear();
  }

  constexpr double HalfPi = std::numbers::pi_v<double> / 2.0;
  constexpr double Pi = std::numbers::pi_v<double>;
  if (!std::isfinite(initialCondition.latitudeRad)
      || initialCondition.latitudeRad < -HalfPi
      || initialCondition.latitudeRad > HalfPi) {
    return ValidationFailed(errorMessage,
        "Latitude must be finite and between -pi/2 and pi/2 radians.");
  }

  if (!std::isfinite(initialCondition.longitudeRad)
      || initialCondition.longitudeRad < -Pi
      || initialCondition.longitudeRad > Pi) {
    return ValidationFailed(errorMessage,
        "Longitude must be finite and between -pi and pi radians.");
  }

  if (!std::isfinite(initialCondition.altitudeAslM)) {
    return ValidationFailed(errorMessage, "Altitude must be finite.");
  }

  if (!std::isfinite(initialCondition.rollRad)
      || !std::isfinite(initialCondition.pitchRad)
      || !std::isfinite(initialCondition.headingRad)) {
    return ValidationFailed(errorMessage, "Attitude values must be finite.");
  }

  if (!std::isfinite(initialCondition.calibratedAirspeedMps)
      || initialCondition.calibratedAirspeedMps < 0.0) {
    return ValidationFailed(errorMessage,
        "Airspeed must be finite and non-negative.");
  }

  if (!std::isfinite(initialCondition.pRadPerSec)
      || !std::isfinite(initialCondition.qRadPerSec)
      || !std::isfinite(initialCondition.rRadPerSec)) {
    return ValidationFailed(errorMessage, "Angular rates must be finite.");
  }

  return true;
}
} // namespace sim
