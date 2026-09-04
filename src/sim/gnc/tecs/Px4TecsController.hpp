#pragma once

#include "common/math/Math.hpp"
#include "sim/gnc/Controller.hpp"
#include "sim/gnc/tecs/Px4TecsParameterMetadata.hpp"

namespace gnc {
struct Px4TecsInput {
  // Altitude is metres AGL; airspeed is metres per second CAS.
  double altitudeM = 0.0;
  double verticalSpeedMps = 0.0;
  double calibratedAirspeedMps = 0.0;
  double targetAltitudeM = 0.0;
  double targetAirspeedMps = 0.0;
  double currentPitchRad = 0.0;
  double currentThrottle = 0.0;
  double gravityMps2 = 9.80665;
  double dtSec = 0.0;
};

struct Px4TecsOutput {
  bool valid = false;
  // Pitch is radians and throttle is normalized to [0, 1].
  double targetPitchRad = 0.0;
  double targetThrottle = 0.0;
};

struct Px4TecsSettings {
  double minimumPitchRad =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MinimumPitch).defaultValue;
  double maximumPitchRad =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MaximumPitch).defaultValue;
  double minimumThrottle =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MinimumThrottle)
          .defaultValue;
  double maximumThrottle =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MaximumThrottle)
          .defaultValue;
  double trimThrottle =
      GetPx4TecsParameterMetadata(Px4TecsParameter::TrimThrottle).defaultValue;
  double minimumAirspeedMps =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MinimumAirspeed)
          .defaultValue;
  double maximumAirspeedMps =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MaximumAirspeed)
          .defaultValue;
  double maximumClimbRateMps =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MaximumClimbRate)
          .defaultValue;
  double maximumSinkRateMps =
      GetPx4TecsParameterMetadata(Px4TecsParameter::MaximumSinkRate)
          .defaultValue;
  double altitudeErrorGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::AltitudeErrorGain)
          .defaultValue;
  double airspeedErrorGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::AirspeedErrorGain)
          .defaultValue;
  double throttleDampingGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::ThrottleDampingGain)
          .defaultValue;
  double throttleIntegralGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::ThrottleIntegralGain)
          .defaultValue;
  double pitchDampingGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::PitchDampingGain)
          .defaultValue;
  double pitchIntegralGain =
      GetPx4TecsParameterMetadata(Px4TecsParameter::PitchIntegralGain)
          .defaultValue;
  double energyBalanceFeedForwardGain = GetPx4TecsParameterMetadata(
      Px4TecsParameter::EnergyBalanceFeedForwardGain)
                                            .defaultValue;
  double totalEnergyRateFilterTimeConstantSec = GetPx4TecsParameterMetadata(
      Px4TecsParameter::TotalEnergyRateFilterTimeConstant)
                                                    .defaultValue;
  double pitchSlewRateRadPerSec =
      GetPx4TecsParameterMetadata(Px4TecsParameter::PitchSlewRate).defaultValue;
  double throttleSlewRatePerSec =
      GetPx4TecsParameterMetadata(Px4TecsParameter::ThrottleSlewRate)
          .defaultValue;
};

inline constexpr std::array<ParameterBinding<Px4TecsParameter, Px4TecsSettings>,
    static_cast<std::size_t>(Px4TecsParameter::Count)>
    Px4TecsParameterBindings{{
        {Px4TecsParameter::MinimumPitch, &Px4TecsSettings::minimumPitchRad},
        {Px4TecsParameter::MaximumPitch, &Px4TecsSettings::maximumPitchRad},
        {Px4TecsParameter::MinimumThrottle, &Px4TecsSettings::minimumThrottle},
        {Px4TecsParameter::MaximumThrottle, &Px4TecsSettings::maximumThrottle},
        {Px4TecsParameter::TrimThrottle, &Px4TecsSettings::trimThrottle},
        {Px4TecsParameter::MinimumAirspeed,
            &Px4TecsSettings::minimumAirspeedMps},
        {Px4TecsParameter::MaximumAirspeed,
            &Px4TecsSettings::maximumAirspeedMps},
        {Px4TecsParameter::MaximumClimbRate,
            &Px4TecsSettings::maximumClimbRateMps},
        {Px4TecsParameter::MaximumSinkRate,
            &Px4TecsSettings::maximumSinkRateMps},
        {Px4TecsParameter::AltitudeErrorGain,
            &Px4TecsSettings::altitudeErrorGain},
        {Px4TecsParameter::AirspeedErrorGain,
            &Px4TecsSettings::airspeedErrorGain},
        {Px4TecsParameter::ThrottleDampingGain,
            &Px4TecsSettings::throttleDampingGain},
        {Px4TecsParameter::ThrottleIntegralGain,
            &Px4TecsSettings::throttleIntegralGain},
        {Px4TecsParameter::PitchDampingGain,
            &Px4TecsSettings::pitchDampingGain},
        {Px4TecsParameter::PitchIntegralGain,
            &Px4TecsSettings::pitchIntegralGain},
        {Px4TecsParameter::EnergyBalanceFeedForwardGain,
            &Px4TecsSettings::energyBalanceFeedForwardGain},
        {Px4TecsParameter::TotalEnergyRateFilterTimeConstant,
            &Px4TecsSettings::totalEnergyRateFilterTimeConstantSec},
        {Px4TecsParameter::PitchSlewRate,
            &Px4TecsSettings::pitchSlewRateRadPerSec},
        {Px4TecsParameter::ThrottleSlewRate,
            &Px4TecsSettings::throttleSlewRatePerSec},
    }};

static_assert(
    ValidateParameterSchema(Px4TecsParameters, Px4TecsParameterBindings));

struct Px4TecsDiagnostics {
  bool controlOutputValid = false;
  bool initialized = false;

  // Inputs and shaped setpoints
  double altitudeM = 0.0;
  double targetAltitudeM = 0.0;
  double internalAltitudeSetpointM = 0.0;
  double airspeedMps = 0.0;
  double targetAirspeedMps = 0.0;
  double verticalSpeedMps = 0.0;
  double airspeedRateMps2 = 0.0;
  double targetVerticalSpeedMps = 0.0;
  double targetAirspeedRateMps2 = 0.0;

  // Specific energy and energy-rate state
  double potentialEnergy = 0.0;
  double potentialEnergySetpoint = 0.0;
  double potentialEnergyError = 0.0;
  double kineticEnergy = 0.0;
  double kineticEnergySetpoint = 0.0;
  double kineticEnergyError = 0.0;
  double totalEnergy = 0.0;
  double totalEnergySetpoint = 0.0;
  double totalEnergyError = 0.0;
  double energyBalance = 0.0;
  double energyBalanceSetpoint = 0.0;
  double energyBalanceError = 0.0;
  double totalEnergyRate = 0.0;
  double totalEnergyRateSetpoint = 0.0;
  double totalEnergyRateError = 0.0;
  double energyBalanceRate = 0.0;
  double energyBalanceRateSetpoint = 0.0;
  double energyBalanceRateError = 0.0;

  // Controller contributions and outputs
  double throttleFeedForwardTerm = 0.0;
  double throttleProportionalTerm = 0.0;
  double throttleIntegralTerm = 0.0;
  double throttleRateTerm = 0.0;
  double pitchProportionalTerm = 0.0;
  double pitchIntegralTerm = 0.0;
  double pitchRateTerm = 0.0;
  double unclampedPitchRad = 0.0;
  double unclampedThrottle = 0.0;
  double targetPitchRad = 0.0;
  double targetThrottle = 0.0;

  // Protection and limiting state
  bool pitchUpperLimited = false;
  bool pitchLowerLimited = false;
  bool pitchRateLimited = false;
  bool throttleUpperSaturated = false;
  bool throttleLowerSaturated = false;
  bool throttleRateLimited = false;
  bool underspeedProtectionActive = false;
  bool overspeedProtectionActive = false;
  bool throttleIntegratorLimited = false;
  bool pitchIntegratorLimited = false;
};

double GetPx4TecsParameterValue(const Px4TecsSettings &settings,
    Px4TecsParameter parameter);
void SetPx4TecsParameterValue(Px4TecsSettings &settings,
    Px4TecsParameter parameter, double value);
bool TrySetPx4TecsParameterValue(Px4TecsSettings &settings,
    Px4TecsParameter parameter, double value);
void ResetPx4TecsParametersToDefaults(Px4TecsSettings &settings);

class Px4TecsController final : public Controller {
public:
  // Lifecycle and bumpless initialization
  void Reset() override;
  void Synchronize(const Px4TecsInput &input);

  // Aircraft-specific parameter snapshot
  const Px4TecsSettings &GetSettings() const;
  void SetSettings(const Px4TecsSettings &settings);

  // Longitudinal outer-loop execution
  Px4TecsOutput Update(const Px4TecsInput &input);
  const Px4TecsDiagnostics &GetDiagnostics() const;

private:
  // Input conditioning and reference shaping
  bool IsInputValid(const Px4TecsInput &input) const;
  double UpdateAltitudeSetpoint(double targetAltitudeM, double dtSec);
  double UpdateAirspeedRate(double airspeedMps, double dtSec);

  // Energy controller state
  bool initialized_ = false;
  double internalAltitudeSetpointM_ = 0.0;
  double previousAirspeedMps_ = 0.0;
  double filteredAirspeedRateMps2_ = 0.0;
  double filteredTotalEnergyRate_ = 0.0;
  double throttleIntegrator_ = 0.0;
  double pitchIntegratorRad_ = 0.0;
  double targetPitchRad_ = 0.0;
  double targetThrottle_ = 0.0;

  // Configuration and last result
  Px4TecsSettings settings_;
  Px4TecsDiagnostics diagnostics_;
};
} // namespace gnc
