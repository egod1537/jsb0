#include "sim/gnc/control/yaw/Px4YawRateController.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Tick.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double MinimumPx4DtSec = 0.002;
constexpr double MaximumPx4DtSec = 0.040;
constexpr double IntegralReductionRateRadPerSec = math::DegToRad(400.0);
constexpr double StandardGravityMps2 = 9.80665;
constexpr double FullCoordinationTiltRad = math::DegToRad(70.0);
constexpr double ZeroCoordinationTiltRad = math::DegToRad(75.0);

double CoordinationScale(double tiltRad) {
  if (tiltRad <= FullCoordinationTiltRad) {
    return 1.0;
  }
  if (tiltRad >= ZeroCoordinationTiltRad) {
    return 0.0;
  }
  return (ZeroCoordinationTiltRad - tiltRad)
         / (ZeroCoordinationTiltRad - FullCoordinationTiltRad);
}
} // namespace

namespace gnc {
double GetPx4YawRateParameterValue(const Px4YawRateSettings &settings,
    Px4YawRateParameter parameter) {
  return GetBoundParameterValue(settings,
      parameter,
      Px4YawRateParameterBindings);
}

bool SetPx4YawRateParameterValue(Px4YawRateSettings &settings,
    Px4YawRateParameter parameter, double value) {
  return SetBoundParameterValue(settings,
      parameter,
      value,
      Px4YawRateParameters,
      Px4YawRateParameterBindings);
}

void ResetPx4YawRateParametersToDefaults(Px4YawRateSettings &settings) {
  ResetBoundParametersToDefaults(settings,
      Px4YawRateParameters,
      Px4YawRateParameterBindings);
}

void Px4YawRateController::Reset() {
  rateIntegrator_ = 0.0;
  yawRateResidualLowPassRadPerSec_ = 0.0;
  diagnostics_ = {};
}

bool Px4YawRateController::IsEnabled() const { return enabled_; }

void Px4YawRateController::SetEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }
  enabled_ = enabled;
  Reset();
}

const Px4YawRateSettings &Px4YawRateController::GetSettings() const {
  return settings_;
}

void Px4YawRateController::SetSettings(const Px4YawRateSettings &settings) {
  if (settings_.setpointMode != settings.setpointMode
      || settings_.yawRateWashoutTimeConstantSec
             != settings.yawRateWashoutTimeConstantSec) {
    rateIntegrator_ = 0.0;
    yawRateResidualLowPassRadPerSec_ = 0.0;
    diagnostics_ = {};
  }
  settings_ = settings;
  NormalizeBoundParameters(settings_,
      Px4YawRateParameters,
      Px4YawRateParameterBindings);
  settings_.trimAirspeedMps = std::isfinite(settings_.trimAirspeedMps)
                                  ? std::max(settings_.trimAirspeedMps, 0.1)
                                  : 15.0;
  settings_.stallAirspeedMps = std::isfinite(settings_.stallAirspeedMps)
                                   ? std::max(settings_.stallAirspeedMps, 0.1)
                                   : 7.0;
  settings_.trimRudderCommand =
      std::isfinite(settings_.trimRudderCommand)
          ? std::clamp(settings_.trimRudderCommand, -1.0, 1.0)
          : 0.0;
}

std::optional<double> Px4YawRateController::OnTick(
    const sim::Aircraft &aircraft, const sim::Tick &tick,
    double rollControlCommand) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &properties = aircraft.GetProperties();
  const double minimumAirspeed = std::max(settings_.stallAirspeedMps, 0.1);
  const double constrainedAirspeed =
      std::max(properties.CalibratedAirspeed().Mps(), minimumAirspeed);
  const double airspeedScaling =
      settings_.trimAirspeedMps / constrainedAirspeed;
  const double coordinatedRateSetpoint =
      GenerateCoordinatedRateSetpoint(aircraft, constrainedAirspeed);
  const double sideslip = properties.Beta().Rad();
  const double sideslipRateCorrection =
      settings_.sideslipToYawRateGain * sideslip;
  const double maximumRate = std::max(settings_.maximumYawRateRadPerSec, 0.0);
  const double rateSetpoint =
      std::clamp(coordinatedRateSetpoint + sideslipRateCorrection,
          -maximumRate,
          maximumRate);
  const double bodyRate = properties.R().RadPerSec();
  const double dtSec = std::clamp(tick.dtSec, MinimumPx4DtSec, MaximumPx4DtSec);
  double feedbackBodyRate = bodyRate;
  const double washoutTimeConstant =
      std::max(settings_.yawRateWashoutTimeConstantSec, 0.0);
  if (washoutTimeConstant > 0.0) {
    const double residualRate = bodyRate - coordinatedRateSetpoint;
    const double lowPassAlpha = dtSec / (washoutTimeConstant + dtSec);
    yawRateResidualLowPassRadPerSec_ +=
        lowPassAlpha * (residualRate - yawRateResidualLowPassRadPerSec_);
    feedbackBodyRate = coordinatedRateSetpoint + residualRate
                       - yawRateResidualLowPassRadPerSec_;
  } else {
    yawRateResidualLowPassRadPerSec_ = 0.0;
  }
  const double rateError = rateSetpoint - feedbackBodyRate;

  const double feedForwardGain =
      settings_.rateFeedForwardGain >= 0.0
          ? settings_.rateFeedForwardGain / std::max(airspeedScaling, 1.0e-6)
          : settings_.rateFeedForwardGain;
  const double rateProportionalTerm =
      settings_.rateProportionalGain * rateError;
  const double rateIntegralTerm = rateIntegrator_;
  const double rateDerivativeTerm =
      -settings_.rateDerivativeGain * properties.R().DotRadPerSec2();
  const double rateFeedForwardTerm = feedForwardGain * rateSetpoint;
  const double unscaledTorque = rateProportionalTerm + rateIntegralTerm
                                + rateDerivativeTerm + rateFeedForwardTerm;
  const double scalingSquared = airspeedScaling * airspeedScaling;
  const double rollToYawFeedForwardTerm =
      settings_.rollToYawFeedForwardGain * rollControlCommand;
  const double rawTorqueCommand =
      unscaledTorque * scalingSquared + rollToYawFeedForwardTerm;

  // The C172x JSBSim rudder channel has the opposite allocation sign from
  // PX4 positive yaw torque (Cndr < 0). Keep the PID in PX4 torque coordinates
  // and apply the sign only at the actuator-allocation boundary.
  const double rawRudderCommand =
      settings_.trimRudderCommand - rawTorqueCommand;
  const double rudderCommand = std::clamp(rawRudderCommand, -1.0, 1.0);
  const double yawTorqueCommand = settings_.trimRudderCommand - rudderCommand;
  const bool positiveSaturation = rawRudderCommand < -1.0;
  const bool negativeSaturation = rawRudderCommand > 1.0;

  UpdateIntegrator(rateError, dtSec, positiveSaturation, negativeSaturation);
  const double integratorLimit = std::max(settings_.integratorLimit, 0.0);
  const bool integratorLimited =
      integratorLimit > 0.0 && std::abs(rateIntegrator_) >= integratorLimit;

  diagnostics_ = {
      .controlOutputValid = true,
      .setpointMode = settings_.setpointMode,
      .bodyRateSetpointRadPerSec = rateSetpoint,
      .coordinatedRateSetpointRadPerSec = coordinatedRateSetpoint,
      .sideslipRad = sideslip,
      .sideslipRateCorrectionRadPerSec = sideslipRateCorrection,
      .bodyRateRadPerSec = bodyRate,
      .feedbackBodyRateRadPerSec = feedbackBodyRate,
      .bodyRateErrorRadPerSec = rateError,
      .rateProportionalTerm = rateProportionalTerm,
      .rateIntegralTerm = rateIntegralTerm,
      .rateDerivativeTerm = rateDerivativeTerm,
      .rateFeedForwardTerm = rateFeedForwardTerm,
      .rollToYawFeedForwardTerm = rollToYawFeedForwardTerm,
      .rateIntegrator = rateIntegrator_,
      .airspeedScaling = airspeedScaling,
      .unscaledTorqueCommand = unscaledTorque,
      .rawTorqueCommand = rawTorqueCommand,
      .yawTorqueCommand = yawTorqueCommand,
      .rawRudderCommand = rawRudderCommand,
      .rudderCommand = rudderCommand,
      .trimRudderCommand = settings_.trimRudderCommand,
      .positiveSaturation = positiveSaturation,
      .negativeSaturation = negativeSaturation,
      .integratorLimited = integratorLimited,
  };
  return rudderCommand;
}

const Px4YawRateDiagnostics &Px4YawRateController::GetDiagnostics() const {
  return diagnostics_;
}

double Px4YawRateController::GenerateCoordinatedRateSetpoint(
    const sim::Aircraft &aircraft, double constrainedAirspeedMps) const {
  if (settings_.setpointMode == Px4YawRateSetpointMode::DampingOnly) {
    return 0.0;
  }

  const auto &properties = aircraft.GetProperties();
  const double roll = properties.Roll().Rad();
  const double pitch = properties.Pitch().Rad();
  const double cosTilt =
      std::clamp(std::cos(roll) * std::cos(pitch), -1.0, 1.0);
  const double tilt = std::acos(cosTilt);
  const double yawRate = CoordinationScale(tilt) * StandardGravityMps2
                         * std::sin(roll) * std::cos(pitch)
                         / std::max(constrainedAirspeedMps, 0.1);
  return yawRate;
}

void Px4YawRateController::UpdateIntegrator(double rateErrorRadPerSec,
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
