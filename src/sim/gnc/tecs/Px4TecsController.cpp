#include "sim/gnc/tecs/Px4TecsController.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double MinimumDtSec = 0.001;
constexpr double MaximumDtSec = 0.25;
constexpr double MinimumGravityMps2 = 1.0;
constexpr double MinimumEnergyDenominator = 1.0e-6;

double Approach(double current, double target, double maximumDelta) {
  return current + std::clamp(target - current, -maximumDelta, maximumDelta);
}
} // namespace

namespace gnc {
double GetPx4TecsParameterValue(const Px4TecsSettings &settings,
    Px4TecsParameter parameter) {
  return GetBoundParameterValue(settings, parameter, Px4TecsParameterBindings);
}

void SetPx4TecsParameterValue(Px4TecsSettings &settings,
    Px4TecsParameter parameter, double value) {
  TrySetPx4TecsParameterValue(settings, parameter, value);
}

bool TrySetPx4TecsParameterValue(Px4TecsSettings &settings,
    Px4TecsParameter parameter, double value) {
  return SetBoundParameterValue(settings,
      parameter,
      value,
      Px4TecsParameters,
      Px4TecsParameterBindings);
}

void ResetPx4TecsParametersToDefaults(Px4TecsSettings &settings) {
  ResetBoundParametersToDefaults(settings,
      Px4TecsParameters,
      Px4TecsParameterBindings);
}

void Px4TecsController::Reset() {
  initialized_ = false;
  internalAltitudeSetpointM_ = 0.0;
  previousAirspeedMps_ = 0.0;
  filteredAirspeedRateMps2_ = 0.0;
  filteredTotalEnergyRate_ = 0.0;
  throttleIntegrator_ = 0.0;
  pitchIntegratorRad_ = 0.0;
  targetPitchRad_ = 0.0;
  targetThrottle_ = 0.0;
  diagnostics_ = {};
}

void Px4TecsController::Synchronize(const Px4TecsInput &input) {
  if (!IsInputValid(input)) {
    Reset();
    return;
  }

  initialized_ = true;
  internalAltitudeSetpointM_ = input.altitudeM;
  previousAirspeedMps_ = input.calibratedAirspeedMps;
  filteredAirspeedRateMps2_ = 0.0;
  filteredTotalEnergyRate_ = input.gravityMps2 * input.verticalSpeedMps;
  targetPitchRad_ = std::clamp(input.currentPitchRad,
      settings_.minimumPitchRad,
      settings_.maximumPitchRad);
  targetThrottle_ = std::clamp(input.currentThrottle,
      settings_.minimumThrottle,
      settings_.maximumThrottle);
  pitchIntegratorRad_ = targetPitchRad_;
  throttleIntegrator_ = targetThrottle_ - settings_.trimThrottle;
  diagnostics_ = {};
}

const Px4TecsSettings &Px4TecsController::GetSettings() const {
  return settings_;
}

void Px4TecsController::SetSettings(const Px4TecsSettings &settings) {
  settings_ = settings;
  NormalizeBoundParameters(settings_,
      Px4TecsParameters,
      Px4TecsParameterBindings);
  if (settings_.minimumPitchRad > settings_.maximumPitchRad) {
    std::swap(settings_.minimumPitchRad, settings_.maximumPitchRad);
  }
  if (settings_.minimumThrottle > settings_.maximumThrottle) {
    std::swap(settings_.minimumThrottle, settings_.maximumThrottle);
  }
  if (settings_.minimumAirspeedMps > settings_.maximumAirspeedMps) {
    std::swap(settings_.minimumAirspeedMps, settings_.maximumAirspeedMps);
  }
  settings_.trimThrottle = std::clamp(settings_.trimThrottle,
      settings_.minimumThrottle,
      settings_.maximumThrottle);

  targetPitchRad_ = std::clamp(targetPitchRad_,
      settings_.minimumPitchRad,
      settings_.maximumPitchRad);
  targetThrottle_ = std::clamp(targetThrottle_,
      settings_.minimumThrottle,
      settings_.maximumThrottle);
}

Px4TecsOutput Px4TecsController::Update(const Px4TecsInput &input) {
  if (!IsInputValid(input)) {
    diagnostics_ = {};
    return {};
  }

  const bool synchronizeThisUpdate = !initialized_;
  if (synchronizeThisUpdate) {
    Synchronize(input);
  }

  const double dtSec = input.dtSec;
  const double gravityMps2 = std::max(input.gravityMps2, MinimumGravityMps2);
  const double altitudeReferenceRateMps =
      UpdateAltitudeSetpoint(input.targetAltitudeM, dtSec);
  const double airspeedRateMps2 =
      UpdateAirspeedRate(input.calibratedAirspeedMps, dtSec);
  const double airspeedSetpointMps = std::clamp(input.targetAirspeedMps,
      settings_.minimumAirspeedMps,
      settings_.maximumAirspeedMps);

  const double targetVerticalSpeedMps =
      std::clamp((internalAltitudeSetpointM_ - input.altitudeM)
                         * settings_.altitudeErrorGain
                     + altitudeReferenceRateMps,
          -settings_.maximumSinkRateMps,
          settings_.maximumClimbRateMps);
  const double energyRateMaximum = gravityMps2 * settings_.maximumClimbRateMps;
  const double energyRateMinimum = -gravityMps2 * settings_.maximumSinkRateMps;
  const double safeAirspeedMps = std::max(input.calibratedAirspeedMps, 1.0);
  const double targetAirspeedRateMps2 =
      std::clamp((airspeedSetpointMps - input.calibratedAirspeedMps)
                     * settings_.airspeedErrorGain,
          0.5 * energyRateMinimum / safeAirspeedMps,
          0.5 * energyRateMaximum / safeAirspeedMps);

  const double potentialEnergy = gravityMps2 * input.altitudeM;
  const double potentialEnergySetpoint =
      gravityMps2 * internalAltitudeSetpointM_;
  const double kineticEnergy =
      0.5 * input.calibratedAirspeedMps * input.calibratedAirspeedMps;
  const double kineticEnergySetpoint =
      0.5 * airspeedSetpointMps * airspeedSetpointMps;
  const double potentialEnergyError = potentialEnergySetpoint - potentialEnergy;
  const double kineticEnergyError = kineticEnergySetpoint - kineticEnergy;

  const double potentialEnergyRate = gravityMps2 * input.verticalSpeedMps;
  const double kineticEnergyRate =
      input.calibratedAirspeedMps * airspeedRateMps2;
  const double rawTotalEnergyRate = potentialEnergyRate + kineticEnergyRate;
  const double filterTimeConstant =
      settings_.totalEnergyRateFilterTimeConstantSec;
  const double filterAlpha =
      filterTimeConstant > 0.0 ? dtSec / (filterTimeConstant + dtSec) : 1.0;
  filteredTotalEnergyRate_ +=
      filterAlpha * (rawTotalEnergyRate - filteredTotalEnergyRate_);

  const double potentialEnergyRateSetpoint =
      gravityMps2 * targetVerticalSpeedMps;
  const double kineticEnergyRateSetpoint =
      airspeedSetpointMps * targetAirspeedRateMps2;
  const double totalEnergyRateSetpoint =
      std::clamp(potentialEnergyRateSetpoint + kineticEnergyRateSetpoint,
          energyRateMinimum,
          energyRateMaximum);
  const double totalEnergyRateError =
      totalEnergyRateSetpoint - filteredTotalEnergyRate_;

  const double underspeedStart = settings_.minimumAirspeedMps * 1.05;
  const double underspeedFull = settings_.minimumAirspeedMps * 0.90;
  const double underspeedRatio =
      std::clamp((underspeedStart - input.calibratedAirspeedMps)
                     / std::max(underspeedStart - underspeedFull,
                         MinimumEnergyDenominator),
          0.0,
          1.0);
  const double overspeedStart = settings_.maximumAirspeedMps * 0.98;
  const double overspeedFull = settings_.maximumAirspeedMps * 1.05;
  const double overspeedRatio = std::clamp(
      (input.calibratedAirspeedMps - overspeedStart)
          / std::max(overspeedFull - overspeedStart, MinimumEnergyDenominator),
      0.0,
      1.0);
  const double speedProtectionRatio = std::max(underspeedRatio, overspeedRatio);

  const double energyRateRange =
      std::max(energyRateMaximum - energyRateMinimum, MinimumEnergyDenominator);
  double throttleFeedForward = settings_.trimThrottle;
  if (totalEnergyRateSetpoint >= 0.0) {
    throttleFeedForward +=
        totalEnergyRateSetpoint
        * (settings_.maximumThrottle - settings_.trimThrottle)
        / std::max(energyRateMaximum, MinimumEnergyDenominator);
  } else {
    throttleFeedForward -=
        totalEnergyRateSetpoint
        * (settings_.trimThrottle - settings_.minimumThrottle)
        / std::min(energyRateMinimum, -MinimumEnergyDenominator);
  }
  const double throttleRateTerm =
      settings_.throttleDampingGain * totalEnergyRateError / energyRateRange;

  const double throttleIntegratorInput =
      totalEnergyRateError * settings_.throttleIntegralGain / energyRateRange;
  double limitedThrottleIntegratorInput = throttleIntegratorInput;
  if (targetThrottle_ >= settings_.maximumThrottle) {
    limitedThrottleIntegratorInput =
        std::min(limitedThrottleIntegratorInput, 0.0);
  } else if (targetThrottle_ <= settings_.minimumThrottle) {
    limitedThrottleIntegratorInput =
        std::max(limitedThrottleIntegratorInput, 0.0);
  }
  limitedThrottleIntegratorInput *= 1.0 - underspeedRatio;
  throttleIntegrator_ += limitedThrottleIntegratorInput * dtSec;
  const double throttleIntegratorMinimum =
      settings_.minimumThrottle - settings_.trimThrottle;
  const double throttleIntegratorMaximum =
      settings_.maximumThrottle - settings_.trimThrottle;
  const double unclampedThrottleIntegrator = throttleIntegrator_;
  throttleIntegrator_ = std::clamp(throttleIntegrator_,
      throttleIntegratorMinimum,
      throttleIntegratorMaximum);
  const bool throttleIntegratorLimited =
      throttleIntegrator_ != unclampedThrottleIntegrator;

  double unclampedThrottle =
      throttleFeedForward + throttleRateTerm + throttleIntegrator_;
  unclampedThrottle = underspeedRatio * settings_.maximumThrottle
                      + (1.0 - underspeedRatio) * unclampedThrottle;
  unclampedThrottle = overspeedRatio * settings_.minimumThrottle
                      + (1.0 - overspeedRatio) * unclampedThrottle;
  const double clampedThrottle = std::clamp(unclampedThrottle,
      settings_.minimumThrottle,
      settings_.maximumThrottle);
  const bool throttleUpperSaturated =
      unclampedThrottle > settings_.maximumThrottle;
  const bool throttleLowerSaturated =
      unclampedThrottle < settings_.minimumThrottle;

  // PX4 uses equal potential/kinetic weighting in normal cruise and moves to
  // full speed priority during underspeed. Overspeed uses the same exchange
  // mechanism in the opposite direction.
  const double kineticEnergyWeight = 1.0 + speedProtectionRatio;
  const double potentialEnergyWeight = 2.0 - kineticEnergyWeight;
  const double energyBalanceRate = potentialEnergyWeight * potentialEnergyRate
                                   - kineticEnergyWeight * kineticEnergyRate;
  const double energyBalanceRateSetpoint =
      potentialEnergyWeight * potentialEnergyRateSetpoint
      - kineticEnergyWeight * kineticEnergyRateSetpoint;
  const double energyBalanceRateError =
      energyBalanceRateSetpoint - energyBalanceRate;
  const double pitchDenominator =
      std::max(safeAirspeedMps * gravityMps2, MinimumEnergyDenominator);
  const double pitchProportionalTerm = settings_.energyBalanceFeedForwardGain
                                       * energyBalanceRateSetpoint
                                       / pitchDenominator;
  const double pitchRateTerm =
      settings_.pitchDampingGain * energyBalanceRateError / pitchDenominator;

  double pitchIntegratorInput =
      settings_.pitchIntegralGain * energyBalanceRateError / pitchDenominator;
  if (targetPitchRad_ >= settings_.maximumPitchRad) {
    pitchIntegratorInput = std::min(pitchIntegratorInput, 0.0);
  } else if (targetPitchRad_ <= settings_.minimumPitchRad) {
    pitchIntegratorInput = std::max(pitchIntegratorInput, 0.0);
  }
  pitchIntegratorRad_ += pitchIntegratorInput * dtSec;
  const double unclampedPitchIntegrator = pitchIntegratorRad_;
  pitchIntegratorRad_ = std::clamp(pitchIntegratorRad_,
      settings_.minimumPitchRad,
      settings_.maximumPitchRad);
  const bool pitchIntegratorLimited =
      pitchIntegratorRad_ != unclampedPitchIntegrator;

  double unclampedPitch =
      pitchIntegratorRad_ + pitchProportionalTerm + pitchRateTerm;
  const double underspeedPitchLimit = std::min(input.currentPitchRad, 0.0);
  unclampedPitch =
      underspeedRatio * std::min(unclampedPitch, underspeedPitchLimit)
      + (1.0 - underspeedRatio) * unclampedPitch;
  const double clampedPitch = std::clamp(unclampedPitch,
      settings_.minimumPitchRad,
      settings_.maximumPitchRad);
  const bool pitchUpperLimited = unclampedPitch > settings_.maximumPitchRad;
  const bool pitchLowerLimited = unclampedPitch < settings_.minimumPitchRad;

  const double previousPitchTarget = targetPitchRad_;
  const double previousThrottleTarget = targetThrottle_;
  if (synchronizeThisUpdate) {
    targetPitchRad_ = std::clamp(input.currentPitchRad,
        settings_.minimumPitchRad,
        settings_.maximumPitchRad);
    targetThrottle_ = std::clamp(input.currentThrottle,
        settings_.minimumThrottle,
        settings_.maximumThrottle);
  } else {
    targetPitchRad_ = Approach(targetPitchRad_,
        clampedPitch,
        settings_.pitchSlewRateRadPerSec * dtSec);
    const double throttleDelta =
        settings_.throttleSlewRatePerSec > 0.0
            ? settings_.throttleSlewRatePerSec * dtSec
            : settings_.maximumThrottle - settings_.minimumThrottle;
    targetThrottle_ = Approach(targetThrottle_, clampedThrottle, throttleDelta);
  }

  diagnostics_ = {
      .controlOutputValid = true,
      .initialized = initialized_,
      .altitudeM = input.altitudeM,
      .targetAltitudeM = input.targetAltitudeM,
      .internalAltitudeSetpointM = internalAltitudeSetpointM_,
      .airspeedMps = input.calibratedAirspeedMps,
      .targetAirspeedMps = airspeedSetpointMps,
      .verticalSpeedMps = input.verticalSpeedMps,
      .airspeedRateMps2 = airspeedRateMps2,
      .targetVerticalSpeedMps = targetVerticalSpeedMps,
      .targetAirspeedRateMps2 = targetAirspeedRateMps2,
      .potentialEnergy = potentialEnergy,
      .potentialEnergySetpoint = potentialEnergySetpoint,
      .potentialEnergyError = potentialEnergyError,
      .kineticEnergy = kineticEnergy,
      .kineticEnergySetpoint = kineticEnergySetpoint,
      .kineticEnergyError = kineticEnergyError,
      .totalEnergy = potentialEnergy + kineticEnergy,
      .totalEnergySetpoint = potentialEnergySetpoint + kineticEnergySetpoint,
      .totalEnergyError = potentialEnergyError + kineticEnergyError,
      .energyBalance = potentialEnergy - kineticEnergy,
      .energyBalanceSetpoint = potentialEnergySetpoint - kineticEnergySetpoint,
      .energyBalanceError = potentialEnergyError - kineticEnergyError,
      .totalEnergyRate = filteredTotalEnergyRate_,
      .totalEnergyRateSetpoint = totalEnergyRateSetpoint,
      .totalEnergyRateError = totalEnergyRateError,
      .energyBalanceRate = energyBalanceRate,
      .energyBalanceRateSetpoint = energyBalanceRateSetpoint,
      .energyBalanceRateError = energyBalanceRateError,
      .throttleFeedForwardTerm = throttleFeedForward,
      .throttleProportionalTerm = throttleFeedForward - settings_.trimThrottle,
      .throttleIntegralTerm = throttleIntegrator_,
      .throttleRateTerm = throttleRateTerm,
      .pitchProportionalTerm = pitchProportionalTerm,
      .pitchIntegralTerm = pitchIntegratorRad_,
      .pitchRateTerm = pitchRateTerm,
      .unclampedPitchRad = unclampedPitch,
      .unclampedThrottle = unclampedThrottle,
      .targetPitchRad = targetPitchRad_,
      .targetThrottle = targetThrottle_,
      .pitchUpperLimited = pitchUpperLimited,
      .pitchLowerLimited = pitchLowerLimited,
      .pitchRateLimited = !synchronizeThisUpdate
                          && targetPitchRad_ != clampedPitch
                          && targetPitchRad_ != previousPitchTarget,
      .throttleUpperSaturated = throttleUpperSaturated,
      .throttleLowerSaturated = throttleLowerSaturated,
      .throttleRateLimited = !synchronizeThisUpdate
                             && targetThrottle_ != clampedThrottle
                             && targetThrottle_ != previousThrottleTarget,
      .underspeedProtectionActive = underspeedRatio > 0.0,
      .overspeedProtectionActive = overspeedRatio > 0.0,
      .throttleIntegratorLimited = throttleIntegratorLimited,
      .pitchIntegratorLimited = pitchIntegratorLimited,
  };
  return Px4TecsOutput{true, targetPitchRad_, targetThrottle_};
}

const Px4TecsDiagnostics &Px4TecsController::GetDiagnostics() const {
  return diagnostics_;
}

bool Px4TecsController::IsInputValid(const Px4TecsInput &input) const {
  return std::isfinite(input.altitudeM) && std::isfinite(input.verticalSpeedMps)
         && std::isfinite(input.calibratedAirspeedMps)
         && input.calibratedAirspeedMps > 0.0
         && std::isfinite(input.targetAltitudeM)
         && std::isfinite(input.targetAirspeedMps)
         && input.targetAirspeedMps > 0.0
         && std::isfinite(input.currentPitchRad)
         && std::isfinite(input.currentThrottle)
         && std::isfinite(input.gravityMps2)
         && input.gravityMps2 >= MinimumGravityMps2
         && std::isfinite(input.dtSec) && input.dtSec >= MinimumDtSec
         && input.dtSec <= MaximumDtSec;
}

double Px4TecsController::UpdateAltitudeSetpoint(double targetAltitudeM,
    double dtSec) {
  const double delta = targetAltitudeM - internalAltitudeSetpointM_;
  const double maximumDelta = (delta >= 0.0 ? settings_.maximumClimbRateMps
                                            : settings_.maximumSinkRateMps)
                              * dtSec;
  const double previous = internalAltitudeSetpointM_;
  internalAltitudeSetpointM_ =
      Approach(internalAltitudeSetpointM_, targetAltitudeM, maximumDelta);
  return (internalAltitudeSetpointM_ - previous) / dtSec;
}

double Px4TecsController::UpdateAirspeedRate(double airspeedMps, double dtSec) {
  const double rawRate = (airspeedMps - previousAirspeedMps_) / dtSec;
  previousAirspeedMps_ = airspeedMps;
  const double timeConstant = settings_.totalEnergyRateFilterTimeConstantSec;
  const double alpha =
      timeConstant > 0.0 ? dtSec / (timeConstant + dtSec) : 1.0;
  filteredAirspeedRateMps2_ += alpha * (rawRate - filteredAirspeedRateMps2_);
  return filteredAirspeedRateMps2_;
}
} // namespace gnc
