#pragma once

#include "sim/gnc/parameters/Parameter.hpp"

#include <array>
#include <cstddef>

namespace gnc {
enum class Px4RollHoldParameter : std::size_t {
  TimeConstant,
  MaximumRollRate,
  RateProportionalGain,
  RateIntegralGain,
  RateDerivativeGain,
  RateFeedForwardGain,
  IntegratorLimit,
  Count,
};

using Px4RollHoldParameterMetadata = ParameterMetadata<Px4RollHoldParameter>;

// PX4 v1.17 parameter ranges with C172x-tuned runtime defaults.
inline constexpr std::array<Px4RollHoldParameterMetadata,
    static_cast<std::size_t>(Px4RollHoldParameter::Count)>
    Px4RollHoldParameters{{
        {Px4RollHoldParameter::TimeConstant,
            "FW_R_TC",
            "Roll time constant",
            UnitId::Second,
            UnitId::Second,
            0.2,
            1.0,
            0.4,
            0.05,
            "Time constant used to convert roll angle error to a roll-rate "
            "setpoint."},
        {Px4RollHoldParameter::MaximumRollRate,
            "FW_R_RMAX",
            "Maximum roll rate",
            UnitId::DegreePerSecond,
            UnitId::DegreePerSecond,
            0.0,
            180.0,
            70.0,
            0.5,
            "Maximum roll-rate setpoint produced by the attitude loop."},
        {Px4RollHoldParameter::RateProportionalGain,
            "FW_RR_P",
            "Roll rate proportional gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            1.9,
            0.005,
            "Proportional gain applied to roll-rate error."},
        {Px4RollHoldParameter::RateIntegralGain,
            "FW_RR_I",
            "Roll rate integrator gain",
            UnitId::NormalizedPerRadian,
            UnitId::NormalizedPerRadian,
            0.0,
            10.0,
            0.25,
            0.01,
            "Integral gain applied to accumulated roll-rate error."},
        {Px4RollHoldParameter::RateDerivativeGain,
            "FW_RR_D",
            "Roll rate derivative gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            0.0,
            0.005,
            "Derivative gain applied by the roll-rate controller."},
        {Px4RollHoldParameter::RateFeedForwardGain,
            "FW_RR_FF",
            "Roll rate feed forward",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            1.2,
            0.05,
            "Feed-forward gain applied to the roll-rate setpoint."},
        {Px4RollHoldParameter::IntegratorLimit,
            "FW_RR_IMAX",
            "Roll integrator limit",
            UnitId::Normalized,
            UnitId::Normalized,
            0.0,
            1.0,
            0.2,
            0.05,
            "Absolute normalized limit for the roll-rate integrator."},
    }};

constexpr const Px4RollHoldParameterMetadata &GetPx4RollHoldParameterMetadata(
    Px4RollHoldParameter parameter) {
  return GetParameterMetadata(parameter, Px4RollHoldParameters);
}
} // namespace gnc
