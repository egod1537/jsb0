#pragma once

#include "sim/gnc/parameters/Parameter.hpp"

#include <array>
#include <cstddef>

namespace gnc {
enum class Px4PitchHoldParameter : std::size_t {
  TimeConstant,
  MaximumPositivePitchRate,
  MaximumNegativePitchRate,
  RateProportionalGain,
  RateIntegralGain,
  RateDerivativeGain,
  RateFeedForwardGain,
  IntegratorLimit,
  Count,
};

using Px4PitchHoldParameterMetadata = ParameterMetadata<Px4PitchHoldParameter>;

// PX4 v1.17 parameter ranges with C172x-tuned runtime defaults.
inline constexpr std::array<Px4PitchHoldParameterMetadata,
    static_cast<std::size_t>(Px4PitchHoldParameter::Count)>
    Px4PitchHoldParameters{{
        {Px4PitchHoldParameter::TimeConstant,
            "FW_P_TC",
            "Pitch attitude time constant",
            UnitId::Second,
            UnitId::Second,
            0.2,
            1.0,
            0.2,
            0.05},
        {Px4PitchHoldParameter::MaximumPositivePitchRate,
            "FW_P_RMAX_POS",
            "Maximum positive pitch-rate setpoint",
            UnitId::DegreePerSecond,
            UnitId::DegreePerSecond,
            0.0,
            180.0,
            14.0,
            0.5},
        {Px4PitchHoldParameter::MaximumNegativePitchRate,
            "FW_P_RMAX_NEG",
            "Maximum negative pitch-rate setpoint",
            UnitId::DegreePerSecond,
            UnitId::DegreePerSecond,
            0.0,
            180.0,
            10.0,
            0.5},
        {Px4PitchHoldParameter::RateProportionalGain,
            "FW_PR_P",
            "Pitch-rate proportional gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            4.5,
            0.005},
        {Px4PitchHoldParameter::RateIntegralGain,
            "FW_PR_I",
            "Pitch-rate integrator gain",
            UnitId::NormalizedPerRadian,
            UnitId::NormalizedPerRadian,
            0.0,
            10.0,
            4.5,
            0.005},
        {Px4PitchHoldParameter::RateDerivativeGain,
            "FW_PR_D",
            "Pitch-rate derivative gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            0.0,
            0.005},
        {Px4PitchHoldParameter::RateFeedForwardGain,
            "FW_PR_FF",
            "Pitch-rate feed forward",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            -10.0,
            10.0,
            1.2,
            0.05},
        {Px4PitchHoldParameter::IntegratorLimit,
            "FW_PR_IMAX",
            "Pitch-rate integrator limit",
            UnitId::Normalized,
            UnitId::Normalized,
            0.0,
            1.0,
            0.4,
            0.05},
    }};

constexpr const Px4PitchHoldParameterMetadata &GetPx4PitchHoldParameterMetadata(
    Px4PitchHoldParameter parameter) {
  return GetParameterMetadata(parameter, Px4PitchHoldParameters);
}
} // namespace gnc
