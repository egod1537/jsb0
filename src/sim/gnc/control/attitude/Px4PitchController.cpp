#include "sim/gnc/control/attitude/Px4PitchController.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Tick.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double MinimumPx4DtSec = 0.002;
constexpr double MaximumPx4DtSec = 0.040;
constexpr double IntegralReductionRateRadPerSec = math::DegToRad(400.0);
} // namespace

namespace gnc {
double GetPx4PitchHoldParameterValue(const Px4PitchHoldSettings &settings,
    Px4PitchHoldParameter parameter) {
  return GetBoundParameterValue(settings,
      parameter,
      Px4PitchHoldParameterBindings);
}

bool SetPx4PitchHoldParameterValue(Px4PitchHoldSettings &settings,
    Px4PitchHoldParameter parameter, double value) {
  return SetBoundParameterValue(settings,
      parameter,
      value,
      Px4PitchHoldParameters,
      Px4PitchHoldParameterBindings);
}

void ResetPx4PitchHoldParametersToDefaults(Px4PitchHoldSettings &settings) {
  ResetBoundParametersToDefaults(settings,
      Px4PitchHoldParameters,
      Px4PitchHoldParameterBindings);
}

void Px4PitchController::Reset() {
  rateIntegrator_ = 0.0;
  diagnostics_ = {};
}

bool Px4PitchController::IsEnabled() const { return enabled_; }

void Px4PitchController::SetEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }

  enabled_ = enabled;
  Reset();
}

const Px4PitchHoldSettings &Px4PitchController::GetSettings() const {
  return settings_;
}

void Px4PitchController::SetSettings(const Px4PitchHoldSettings &settings) {
  settings_ = settings;
  NormalizeBoundParameters(settings_,
      Px4PitchHoldParameters,
      Px4PitchHoldParameterBindings);
  settings_.trimAirspeedMps = std::isfinite(settings_.trimAirspeedMps)
                                  ? std::max(settings_.trimAirspeedMps, 0.1)
                                  : 15.0;
  settings_.stallAirspeedMps = std::isfinite(settings_.stallAirspeedMps)
                                   ? std::max(settings_.stallAirspeedMps, 0.1)
                                   : 7.0;
  settings_.trimElevatorCommand =
      std::isfinite(settings_.trimElevatorCommand)
          ? std::clamp(settings_.trimElevatorCommand, -1.0, 1.0)
          : 0.0;
}

std::optional<double> Px4PitchController::OnTick(
    const sim::Aircraft &aircraft, const sim::Tick &tick,
    double targetPitchRad) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &properties = aircraft.GetProperties();
  const double pitchRad = properties.Pitch().Rad();
  const double pitchRateRadPerSec = properties.Q().RadPerSec();
  const double pitchAccelerationRadPerSec2 = properties.Q().DotRadPerSec2();
  const double calibratedAirspeedMps = properties.CalibratedAirspeed().Mps();
  if (!std::isfinite(targetPitchRad) || !std::isfinite(pitchRad)
      || !std::isfinite(pitchRateRadPerSec)
      || !std::isfinite(pitchAccelerationRadPerSec2)
      || !std::isfinite(calibratedAirspeedMps)) {
    diagnostics_ = {};
    return std::nullopt;
  }

  const double pitchErrorRad = targetPitchRad - pitchRad;
  const double requestedRateSetpoint =
      pitchErrorRad / std::max(settings_.timeConstantSec, 1.0e-6);
  const double rateSetpoint = std::clamp(requestedRateSetpoint,
      -std::max(settings_.maximumNegativePitchRateRadPerSec, 0.0),
      std::max(settings_.maximumPositivePitchRateRadPerSec, 0.0));
  const double rateErrorRadPerSec = rateSetpoint - pitchRateRadPerSec;

  const double minimumAirspeed = std::max(settings_.stallAirspeedMps, 0.1);
  const double constrainedAirspeed =
      std::max(calibratedAirspeedMps, minimumAirspeed);
  const double airspeedScaling =
      settings_.trimAirspeedMps / constrainedAirspeed;
  const double scaledFeedForward =
      settings_.rateFeedForwardGain >= 0.0
          ? settings_.rateFeedForwardGain / std::max(airspeedScaling, 1.0e-6)
          : settings_.rateFeedForwardGain;

  const double rateProportionalTerm =
      settings_.rateProportionalGain * rateErrorRadPerSec;
  const double rateIntegralTerm = rateIntegrator_;
  const double rateDerivativeTerm =
      -settings_.rateDerivativeGain * pitchAccelerationRadPerSec2;
  const double rateFeedForwardTerm = scaledFeedForward * rateSetpoint;
  const double unscaledTorqueCommand = rateProportionalTerm + rateIntegralTerm
                                       + rateDerivativeTerm
                                       + rateFeedForwardTerm;
  const double scalingSquared = airspeedScaling * airspeedScaling;
  const double trimPitchTorqueCommand = -settings_.trimElevatorCommand;
  const double rawTorqueCommand =
      (unscaledTorqueCommand + trimPitchTorqueCommand) * scalingSquared;
  const double pitchTorqueCommand = std::clamp(rawTorqueCommand, -1.0, 1.0);
  const bool positiveSaturation = rawTorqueCommand > 1.0;
  const bool negativeSaturation = rawTorqueCommand < -1.0;

  // JSBSim's C172x elevator command is opposite the positive body-pitch
  // torque convention used by PX4.
  const double elevatorCommand = -pitchTorqueCommand;

  const double dtSec =
      std::isfinite(tick.dtSec)
          ? std::clamp(tick.dtSec, MinimumPx4DtSec, MaximumPx4DtSec)
          : MinimumPx4DtSec;
  UpdateIntegrator(rateErrorRadPerSec,
      dtSec,
      positiveSaturation,
      negativeSaturation);
  const double integratorLimit = std::max(settings_.integratorLimit, 0.0);
  const bool integratorLimited =
      integratorLimit > 0.0 && std::abs(rateIntegrator_) >= integratorLimit;

  diagnostics_ = {
      .controlOutputValid = true,
      .targetPitchRad = targetPitchRad,
      .pitchErrorRad = pitchErrorRad,
      .bodyRateSetpointRadPerSec = rateSetpoint,
      .bodyRateErrorRadPerSec = rateErrorRadPerSec,
      .rateProportionalTerm = rateProportionalTerm,
      .rateIntegralTerm = rateIntegralTerm,
      .rateDerivativeTerm = rateDerivativeTerm,
      .rateFeedForwardTerm = rateFeedForwardTerm,
      .rateIntegrator = rateIntegrator_,
      .airspeedScaling = airspeedScaling,
      .unscaledTorqueCommand = unscaledTorqueCommand,
      .rawTorqueCommand = rawTorqueCommand,
      .pitchTorqueCommand = pitchTorqueCommand,
      .elevatorCommand = elevatorCommand,
      .trimElevatorCommand = settings_.trimElevatorCommand,
      .positiveSaturation = positiveSaturation,
      .negativeSaturation = negativeSaturation,
      .integratorLimited = integratorLimited,
  };
  return elevatorCommand;
}

const Px4PitchHoldDiagnostics &Px4PitchController::GetDiagnostics() const {
  return diagnostics_;
}

void Px4PitchController::UpdateIntegrator(double rateErrorRadPerSec,
    double dtSec, bool positiveSaturation, bool negativeSaturation) {
  if (positiveSaturation) {
    rateErrorRadPerSec = std::min(rateErrorRadPerSec, 0.0);
  }
  if (negativeSaturation) {
    rateErrorRadPerSec = std::max(rateErrorRadPerSec, 0.0);
  }

  const double normalizedError =
      rateErrorRadPerSec / IntegralReductionRateRadPerSec;
  const double integralFactor =
      std::max(0.0, 1.0 - normalizedError * normalizedError);
  const double nextIntegrator = rateIntegrator_
                                + integralFactor * settings_.rateIntegralGain
                                      * rateErrorRadPerSec * dtSec;
  if (std::isfinite(nextIntegrator)) {
    const double limit = std::max(settings_.integratorLimit, 0.0);
    rateIntegrator_ = std::clamp(nextIntegrator, -limit, limit);
  }
}
} // namespace gnc
