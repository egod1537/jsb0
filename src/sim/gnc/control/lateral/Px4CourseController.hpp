#pragma once

#include "common/math/Math.hpp"
#include "sim/gnc/Controller.hpp"
#include "sim/gnc/control/lateral/Px4CourseParameterMetadata.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct Px4CourseHoldSettings {
  double guidancePeriodSec =
      GetPx4CourseHoldParameterMetadata(Px4CourseHoldParameter::GuidancePeriod)
          .defaultValue;
  double guidanceDampingRatio =
      GetPx4CourseHoldParameterMetadata(Px4CourseHoldParameter::GuidanceDamping)
          .defaultValue;
  double maxRollRad = math::DegToRad(
      GetPx4CourseHoldParameterMetadata(Px4CourseHoldParameter::MaximumRoll)
          .defaultValue);
  double maxRollSetpointRateRadPerSec =
      math::DegToRad(GetPx4CourseHoldParameterMetadata(
          Px4CourseHoldParameter::MaximumRollSetpointRate)
              .defaultValue);
};

inline constexpr std::array<
    ParameterBinding<Px4CourseHoldParameter, Px4CourseHoldSettings>,
    static_cast<std::size_t>(Px4CourseHoldParameter::Count)>
    Px4CourseHoldParameterBindings{{
        {Px4CourseHoldParameter::GuidancePeriod,
            &Px4CourseHoldSettings::guidancePeriodSec},
        {Px4CourseHoldParameter::GuidanceDamping,
            &Px4CourseHoldSettings::guidanceDampingRatio},
        {Px4CourseHoldParameter::MaximumRoll,
            &Px4CourseHoldSettings::maxRollRad,
            ParameterValueTransform::DegreesToRadians},
        {Px4CourseHoldParameter::MaximumRollSetpointRate,
            &Px4CourseHoldSettings::maxRollSetpointRateRadPerSec,
            ParameterValueTransform::DegreesToRadians},
    }};

static_assert(ValidateParameterSchema(Px4CourseHoldParameters,
    Px4CourseHoldParameterBindings));

double GetPx4CourseHoldParameterValue(const Px4CourseHoldSettings &settings,
    Px4CourseHoldParameter parameter);
bool SetPx4CourseHoldParameterValue(Px4CourseHoldSettings &settings,
    Px4CourseHoldParameter parameter, double value);
void ResetPx4CourseHoldParametersToDefaults(Px4CourseHoldSettings &settings);

struct Px4CourseHoldDiagnostics {
  bool controlOutputValid = false;
  bool groundSpeedValid = false;
  double targetCourseRad = 0.0;
  double currentCourseRad = 0.0;
  double courseErrorRad = 0.0;
  double groundSpeedMps = 0.0;
  double directionGainPerSec = 0.0;
  double rawLateralAccelerationMps2 = 0.0;
  double limitedLateralAccelerationMps2 = 0.0;
  double rawRollSetpointRad = 0.0;
  double rollLimitedSetpointRad = 0.0;
  double limitedRollSetpointRad = 0.0;
  bool rollLimited = false;
  bool rollSetpointRateLimited = false;
};

class Px4CourseController final : public Controller {
public:
  // Lifecycle
  void Reset() override;

  // Execution mode
  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  // PX4 lateral-guidance parameter snapshot
  const Px4CourseHoldSettings &GetSettings() const;
  void SetSettings(const Px4CourseHoldSettings &settings);

  // Course-to-roll outer-loop output and diagnostics
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, double targetCourseRad);
  const Px4CourseHoldDiagnostics &GetDiagnostics() const;

private:
  // Guidance stages
  double ComputeDirectionGainPerSec() const;
  double ApplyRollSetpointRateLimit(double rollSetpointRad,
      double currentRollRad, double dtSec, bool &rateLimited);

  // Mode and PX4 parameter snapshot
  bool enabled_ = false;
  Px4CourseHoldSettings settings_;

  // Slew state and last result
  bool rollSetpointInitialized_ = false;
  double previousRollSetpointRad_ = 0.0;
  Px4CourseHoldDiagnostics diagnostics_;
};
} // namespace gnc
