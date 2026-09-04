#pragma once

#include "sim/gnc/parameters/Parameter.hpp"

#include <array>
#include <cstddef>

namespace gnc {
enum class Px4YawRateParameter : std::size_t {
  MaximumYawRate,
  RateProportionalGain,
  RateIntegralGain,
  RateDerivativeGain,
  RateFeedForwardGain,
  IntegratorLimit,
  RollToYawFeedForwardGain,
  SideslipToYawRateGain,
  YawRateWashoutTimeConstant,
  Count,
};

using Px4YawRateParameterMetadata = ParameterMetadata<Px4YawRateParameter>;

inline constexpr std::array<Px4YawRateParameterMetadata,
    static_cast<std::size_t>(Px4YawRateParameter::Count)>
    Px4YawRateParameters{{
        {Px4YawRateParameter::MaximumYawRate,
            "FW_Y_RMAX",
            "Maximum yaw rate",
            UnitId::DegreePerSecond,
            UnitId::DegreePerSecond,
            0.0,
            90.0,
            50.0,
            0.5},
        {Px4YawRateParameter::RateProportionalGain,
            "FW_YR_P",
            "Yaw rate proportional gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            0.05,
            0.005},
        {Px4YawRateParameter::RateIntegralGain,
            "FW_YR_I",
            "Yaw rate integrator gain",
            UnitId::NormalizedPerRadian,
            UnitId::NormalizedPerRadian,
            0.0,
            10.0,
            0.1,
            0.1},
        {Px4YawRateParameter::RateDerivativeGain,
            "FW_YR_D",
            "Yaw rate derivative gain",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            0.0,
            10.0,
            0.0,
            0.005},
        {Px4YawRateParameter::RateFeedForwardGain,
            "FW_YR_FF",
            "Yaw rate setpoint feed forward",
            UnitId::NormalizedPerRadianPerSecond,
            UnitId::NormalizedPerRadianPerSecond,
            -10.0,
            10.0,
            0.3,
            0.05},
        {Px4YawRateParameter::IntegratorLimit,
            "FW_YR_IMAX",
            "Yaw rate integrator limit",
            UnitId::Normalized,
            UnitId::Normalized,
            0.0,
            1.0,
            0.2,
            0.05},
        {Px4YawRateParameter::RollToYawFeedForwardGain,
            "FW_RLL_TO_YAW_FF",
            "Roll control to yaw control feed forward",
            UnitId::Dimensionless,
            UnitId::Dimensionless,
            0.0,
            10.0,
            0.0,
            0.01},
        {Px4YawRateParameter::SideslipToYawRateGain,
            "JSB_BETA_YR",
            "Sideslip feedback gain",
            UnitId::PerSecond,
            UnitId::PerSecond,
            -20.0,
            20.0,
            0.0,
            0.5},
        {Px4YawRateParameter::YawRateWashoutTimeConstant,
            "JSB_YR_WO_TC",
            "Yaw-rate residual washout time constant; zero disables",
            UnitId::Second,
            UnitId::Second,
            0.0,
            10.0,
            0.0,
            0.25},
    }};

constexpr const Px4YawRateParameterMetadata &GetPx4YawRateParameterMetadata(
    Px4YawRateParameter parameter) {
  return GetParameterMetadata(parameter, Px4YawRateParameters);
}
} // namespace gnc
