#pragma once

#include "common/math/Math.hpp"
#include "sim/gnc/Controller.hpp"
#include "sim/gnc/control/yaw/Px4YawRateParameterMetadata.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
enum class Px4YawRateSetpointMode {
  DampingOnly,
  CoordinatedTurn,
};

struct Px4YawRateSettings {
  Px4YawRateSetpointMode setpointMode = Px4YawRateSetpointMode::DampingOnly;
  double maximumYawRateRadPerSec = math::DegToRad(
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::MaximumYawRate)
          .defaultValue);
  double rateProportionalGain =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::RateProportionalGain)
          .defaultValue;
  double rateIntegralGain =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::RateIntegralGain)
          .defaultValue;
  double rateDerivativeGain =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::RateDerivativeGain)
          .defaultValue;
  double rateFeedForwardGain =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::RateFeedForwardGain)
          .defaultValue;
  double integratorLimit =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::IntegratorLimit)
          .defaultValue;
  double rollToYawFeedForwardGain = GetPx4YawRateParameterMetadata(
      Px4YawRateParameter::RollToYawFeedForwardGain)
                                        .defaultValue;
  // JSB0 lateral augmentation; zero keeps the PX4 mainline signal path.
  double sideslipToYawRateGain =
      GetPx4YawRateParameterMetadata(Px4YawRateParameter::SideslipToYawRateGain)
          .defaultValue;
  double yawRateWashoutTimeConstantSec = GetPx4YawRateParameterMetadata(
      Px4YawRateParameter::YawRateWashoutTimeConstant)
                                             .defaultValue;
  double trimAirspeedMps = 15.0;
  double stallAirspeedMps = 7.0;
  double trimRudderCommand = 0.0;
};

inline constexpr std::array<
    ParameterBinding<Px4YawRateParameter, Px4YawRateSettings>,
    static_cast<std::size_t>(Px4YawRateParameter::Count)>
    Px4YawRateParameterBindings{{
        {Px4YawRateParameter::MaximumYawRate,
            &Px4YawRateSettings::maximumYawRateRadPerSec,
            ParameterValueTransform::DegreesToRadians},
        {Px4YawRateParameter::RateProportionalGain,
            &Px4YawRateSettings::rateProportionalGain},
        {Px4YawRateParameter::RateIntegralGain,
            &Px4YawRateSettings::rateIntegralGain},
        {Px4YawRateParameter::RateDerivativeGain,
            &Px4YawRateSettings::rateDerivativeGain},
        {Px4YawRateParameter::RateFeedForwardGain,
            &Px4YawRateSettings::rateFeedForwardGain},
        {Px4YawRateParameter::IntegratorLimit,
            &Px4YawRateSettings::integratorLimit},
        {Px4YawRateParameter::RollToYawFeedForwardGain,
            &Px4YawRateSettings::rollToYawFeedForwardGain},
        {Px4YawRateParameter::SideslipToYawRateGain,
            &Px4YawRateSettings::sideslipToYawRateGain},
        {Px4YawRateParameter::YawRateWashoutTimeConstant,
            &Px4YawRateSettings::yawRateWashoutTimeConstantSec},
    }};

static_assert(
    ValidateParameterSchema(Px4YawRateParameters, Px4YawRateParameterBindings));

double GetPx4YawRateParameterValue(const Px4YawRateSettings &settings,
    Px4YawRateParameter parameter);
bool SetPx4YawRateParameterValue(Px4YawRateSettings &settings,
    Px4YawRateParameter parameter, double value);
void ResetPx4YawRateParametersToDefaults(Px4YawRateSettings &settings);

struct Px4YawRateDiagnostics {
  bool controlOutputValid = false;
  Px4YawRateSetpointMode setpointMode = Px4YawRateSetpointMode::DampingOnly;
  double bodyRateSetpointRadPerSec = 0.0;
  double coordinatedRateSetpointRadPerSec = 0.0;
  double sideslipRad = 0.0;
  double sideslipRateCorrectionRadPerSec = 0.0;
  double bodyRateRadPerSec = 0.0;
  double feedbackBodyRateRadPerSec = 0.0;
  double bodyRateErrorRadPerSec = 0.0;

  // Rate-controller contribution breakdown in normalized yaw-torque units.
  double rateProportionalTerm = 0.0;
  double rateIntegralTerm = 0.0;
  double rateDerivativeTerm = 0.0;
  double rateFeedForwardTerm = 0.0;
  double rollToYawFeedForwardTerm = 0.0;

  // Output pipeline and controller state.
  double rateIntegrator = 0.0;
  double airspeedScaling = 1.0;
  double unscaledTorqueCommand = 0.0;
  double rawTorqueCommand = 0.0;
  double yawTorqueCommand = 0.0;
  double rawRudderCommand = 0.0;
  double rudderCommand = 0.0;
  double trimRudderCommand = 0.0;

  // Limiting state in yaw-torque coordinates.
  bool positiveSaturation = false;
  bool negativeSaturation = false;
  bool integratorLimited = false;
};

class Px4YawRateController final : public Controller {
public:
  // Lifecycle
  void Reset() override;

  // Execution mode
  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  // PX4 parameter snapshot
  const Px4YawRateSettings &GetSettings() const;
  void SetSettings(const Px4YawRateSettings &settings);

  // Yaw control output and diagnostics
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, double rollControlCommand);
  const Px4YawRateDiagnostics &GetDiagnostics() const;

private:
  // Setpoint and integrator updates
  double GenerateCoordinatedRateSetpoint(const sim::Aircraft &aircraft,
      double constrainedAirspeedMps) const;
  void UpdateIntegrator(double rateErrorRadPerSec, double dtSec,
      bool positiveSaturation, bool negativeSaturation);

  // Mode and PX4 parameter snapshot
  bool enabled_ = false;
  Px4YawRateSettings settings_;

  // Controller state and last result
  double rateIntegrator_ = 0.0;
  double yawRateResidualLowPassRadPerSec_ = 0.0;
  Px4YawRateDiagnostics diagnostics_;
};
} // namespace gnc
