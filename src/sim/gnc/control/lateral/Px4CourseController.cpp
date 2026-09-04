#include "sim/gnc/control/lateral/Px4CourseController.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Tick.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
constexpr double MinimumUsableGroundSpeedMps = 1.0;
}

namespace gnc {
double GetPx4CourseHoldParameterValue(const Px4CourseHoldSettings &settings,
    Px4CourseHoldParameter parameter) {
  return GetBoundParameterValue(settings,
      parameter,
      Px4CourseHoldParameterBindings);
}

bool SetPx4CourseHoldParameterValue(Px4CourseHoldSettings &settings,
    Px4CourseHoldParameter parameter, double value) {
  return SetBoundParameterValue(settings,
      parameter,
      value,
      Px4CourseHoldParameters,
      Px4CourseHoldParameterBindings);
}

void ResetPx4CourseHoldParametersToDefaults(Px4CourseHoldSettings &settings) {
  ResetBoundParametersToDefaults(settings,
      Px4CourseHoldParameters,
      Px4CourseHoldParameterBindings);
}

void Px4CourseController::Reset() {
  rollSetpointInitialized_ = false;
  previousRollSetpointRad_ = 0.0;
  diagnostics_ = {};
}

bool Px4CourseController::IsEnabled() const { return enabled_; }

void Px4CourseController::SetEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }

  enabled_ = enabled;
  Reset();
}

const Px4CourseHoldSettings &Px4CourseController::GetSettings() const {
  return settings_;
}

void Px4CourseController::SetSettings(
    const Px4CourseHoldSettings &settings) {
  settings_ = settings;
  NormalizeBoundParameters(settings_,
      Px4CourseHoldParameters,
      Px4CourseHoldParameterBindings);
}

std::optional<double> Px4CourseController::OnTick(
    const sim::Aircraft &aircraft, const sim::Tick &tick,
    double targetCourseRad) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &properties = aircraft.GetProperties();
  const double northVelocityMps = properties.NorthVelocity().Mps();
  const double eastVelocityMps = properties.EastVelocity().Mps();
  const double groundSpeedMps = std::hypot(northVelocityMps, eastVelocityMps);
  const double currentCourseRad = properties.Course().Rad();
  const double currentRollRad = properties.Roll().Rad();
  const double gravityMps2 = properties.GravityMps2();

  if (!std::isfinite(targetCourseRad) || !std::isfinite(currentCourseRad)
      || !std::isfinite(currentRollRad) || !std::isfinite(groundSpeedMps)
      || !std::isfinite(gravityMps2) || gravityMps2 <= 0.0) {
    diagnostics_ = {};
    return std::nullopt;
  }

  targetCourseRad = math::WrapAngleRad(targetCourseRad);
  const double courseErrorRad =
      math::DeltaAngleRad(currentCourseRad, targetCourseRad);
  const double directionGainPerSec = ComputeDirectionGainPerSec();
  const bool groundSpeedValid = groundSpeedMps >= MinimumUsableGroundSpeedMps;

  double rawLateralAccelerationMps2 = 0.0;
  if (groundSpeedValid) {
    const double targetNorth = std::cos(targetCourseRad);
    const double targetEast = std::sin(targetCourseRad);
    const double velocityDotTarget =
        northVelocityMps * targetNorth + eastVelocityMps * targetEast;
    const double velocityCrossTarget =
        northVelocityMps * targetEast - eastVelocityMps * targetNorth;
    const double directionErrorVelocity =
        velocityDotTarget < 0.0
            ? std::copysign(groundSpeedMps, velocityCrossTarget)
            : velocityCrossTarget;
    rawLateralAccelerationMps2 = directionGainPerSec * directionErrorVelocity;
  }

  const double maxLateralAccelerationMps2 =
      std::tan(settings_.maxRollRad) * gravityMps2;
  const double limitedLateralAccelerationMps2 =
      std::clamp(rawLateralAccelerationMps2,
          -maxLateralAccelerationMps2,
          maxLateralAccelerationMps2);
  const bool rollLimited =
      limitedLateralAccelerationMps2 != rawLateralAccelerationMps2;
  const double rawRollSetpointRad =
      std::atan(rawLateralAccelerationMps2 / gravityMps2);
  const double rollLimitedSetpointRad =
      std::atan(limitedLateralAccelerationMps2 / gravityMps2);

  bool rateLimited = false;
  const double limitedRollSetpointRad =
      ApplyRollSetpointRateLimit(rollLimitedSetpointRad,
          currentRollRad,
          tick.dtSec,
          rateLimited);

  diagnostics_ = {
      .controlOutputValid = true,
      .groundSpeedValid = groundSpeedValid,
      .targetCourseRad = targetCourseRad,
      .currentCourseRad = currentCourseRad,
      .courseErrorRad = courseErrorRad,
      .groundSpeedMps = groundSpeedMps,
      .directionGainPerSec = directionGainPerSec,
      .rawLateralAccelerationMps2 = rawLateralAccelerationMps2,
      .limitedLateralAccelerationMps2 = limitedLateralAccelerationMps2,
      .rawRollSetpointRad = rawRollSetpointRad,
      .rollLimitedSetpointRad = rollLimitedSetpointRad,
      .limitedRollSetpointRad = limitedRollSetpointRad,
      .rollLimited = rollLimited,
      .rollSetpointRateLimited = rateLimited,
  };
  return limitedRollSetpointRad;
}

const Px4CourseHoldDiagnostics &
Px4CourseController::GetDiagnostics() const {
  return diagnostics_;
}

double Px4CourseController::ComputeDirectionGainPerSec() const {
  return 4.0 * std::numbers::pi * settings_.guidanceDampingRatio
         / settings_.guidancePeriodSec;
}

double Px4CourseController::ApplyRollSetpointRateLimit(
    double rollSetpointRad, double currentRollRad, double dtSec,
    bool &rateLimited) {
  if (!rollSetpointInitialized_) {
    previousRollSetpointRad_ = currentRollRad;
    rollSetpointInitialized_ = true;
  }

  if (settings_.maxRollSetpointRateRadPerSec <= 0.0) {
    previousRollSetpointRad_ = rollSetpointRad;
    rateLimited = false;
    return rollSetpointRad;
  }

  const double maxStepRad =
      settings_.maxRollSetpointRateRadPerSec * std::max(dtSec, 0.0);
  const double setpointDeltaRad = rollSetpointRad - previousRollSetpointRad_;
  const double limitedDeltaRad =
      std::clamp(setpointDeltaRad, -maxStepRad, maxStepRad);
  rateLimited = limitedDeltaRad != setpointDeltaRad;
  previousRollSetpointRad_ += limitedDeltaRad;
  return previousRollSetpointRad_;
}
} // namespace gnc
