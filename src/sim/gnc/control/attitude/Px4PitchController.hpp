#pragma once

#include "common/math/Math.hpp"
#include "sim/gnc/Controller.hpp"
#include "sim/gnc/control/attitude/Px4PitchParameterMetadata.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct Px4PitchHoldSettings {
  double timeConstantSec =
      GetPx4PitchHoldParameterMetadata(Px4PitchHoldParameter::TimeConstant)
          .defaultValue;
  double maximumPositivePitchRateRadPerSec =
      math::DegToRad(GetPx4PitchHoldParameterMetadata(
          Px4PitchHoldParameter::MaximumPositivePitchRate)
              .defaultValue);
  double maximumNegativePitchRateRadPerSec =
      math::DegToRad(GetPx4PitchHoldParameterMetadata(
          Px4PitchHoldParameter::MaximumNegativePitchRate)
              .defaultValue);
  double rateProportionalGain = GetPx4PitchHoldParameterMetadata(
      Px4PitchHoldParameter::RateProportionalGain)
                                    .defaultValue;
  double rateIntegralGain =
      GetPx4PitchHoldParameterMetadata(Px4PitchHoldParameter::RateIntegralGain)
          .defaultValue;
  double rateDerivativeGain = GetPx4PitchHoldParameterMetadata(
      Px4PitchHoldParameter::RateDerivativeGain)
                                  .defaultValue;
  double rateFeedForwardGain = GetPx4PitchHoldParameterMetadata(
      Px4PitchHoldParameter::RateFeedForwardGain)
                                   .defaultValue;
  double integratorLimit =
      GetPx4PitchHoldParameterMetadata(Px4PitchHoldParameter::IntegratorLimit)
          .defaultValue;
  double trimAirspeedMps = 15.0;
  double stallAirspeedMps = 7.0;
  double trimElevatorCommand = 0.0;
};

inline constexpr std::array<
    ParameterBinding<Px4PitchHoldParameter, Px4PitchHoldSettings>,
    static_cast<std::size_t>(Px4PitchHoldParameter::Count)>
    Px4PitchHoldParameterBindings{{
        {Px4PitchHoldParameter::TimeConstant,
            &Px4PitchHoldSettings::timeConstantSec},
        {Px4PitchHoldParameter::MaximumPositivePitchRate,
            &Px4PitchHoldSettings::maximumPositivePitchRateRadPerSec,
            ParameterValueTransform::DegreesToRadians},
        {Px4PitchHoldParameter::MaximumNegativePitchRate,
            &Px4PitchHoldSettings::maximumNegativePitchRateRadPerSec,
            ParameterValueTransform::DegreesToRadians},
        {Px4PitchHoldParameter::RateProportionalGain,
            &Px4PitchHoldSettings::rateProportionalGain},
        {Px4PitchHoldParameter::RateIntegralGain,
            &Px4PitchHoldSettings::rateIntegralGain},
        {Px4PitchHoldParameter::RateDerivativeGain,
            &Px4PitchHoldSettings::rateDerivativeGain},
        {Px4PitchHoldParameter::RateFeedForwardGain,
            &Px4PitchHoldSettings::rateFeedForwardGain},
        {Px4PitchHoldParameter::IntegratorLimit,
            &Px4PitchHoldSettings::integratorLimit},
    }};

static_assert(ValidateParameterSchema(Px4PitchHoldParameters,
    Px4PitchHoldParameterBindings));

double GetPx4PitchHoldParameterValue(const Px4PitchHoldSettings &settings,
    Px4PitchHoldParameter parameter);
bool SetPx4PitchHoldParameterValue(Px4PitchHoldSettings &settings,
    Px4PitchHoldParameter parameter, double value);
void ResetPx4PitchHoldParametersToDefaults(Px4PitchHoldSettings &settings);

struct Px4PitchHoldDiagnostics {
  bool controlOutputValid = false;
  double targetPitchRad = 0.0;
  double pitchErrorRad = 0.0;
  double bodyRateSetpointRadPerSec = 0.0;
  double bodyRateErrorRadPerSec = 0.0;

  // Rate-controller contribution breakdown
  double rateProportionalTerm = 0.0;
  double rateIntegralTerm = 0.0;
  double rateDerivativeTerm = 0.0;
  double rateFeedForwardTerm = 0.0;

  // Integrator state and control-output pipeline
  double rateIntegrator = 0.0;
  double airspeedScaling = 1.0;
  double unscaledTorqueCommand = 0.0;
  double rawTorqueCommand = 0.0;
  double pitchTorqueCommand = 0.0;
  double elevatorCommand = 0.0;
  double trimElevatorCommand = 0.0;

  // Limiting state
  bool positiveSaturation = false;
  bool negativeSaturation = false;
  bool integratorLimited = false;
};

class Px4PitchController final : public Controller {
public:
  // Lifecycle
  void Reset() override;

  // Baseline Pitch Hold execution
  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  // PX4 parameter snapshot
  const Px4PitchHoldSettings &GetSettings() const;
  void SetSettings(const Px4PitchHoldSettings &settings);

  // Baseline output and diagnostics
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, double targetPitchRad);
  const Px4PitchHoldDiagnostics &GetDiagnostics() const;

private:
  // Rate-loop state
  void UpdateIntegrator(double rateErrorRadPerSec, double dtSec,
      bool positiveSaturation, bool negativeSaturation);

  // Mode and PX4 parameter snapshot
  bool enabled_ = false;
  Px4PitchHoldSettings settings_;

  // Controller state and last result
  double rateIntegrator_ = 0.0;
  Px4PitchHoldDiagnostics diagnostics_;
};
} // namespace gnc
