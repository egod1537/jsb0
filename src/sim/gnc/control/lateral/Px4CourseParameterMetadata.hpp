#pragma once

#include "sim/gnc/parameters/Parameter.hpp"

#include <array>
#include <cstddef>

namespace gnc {
enum class Px4CourseHoldParameter : std::size_t {
  GuidancePeriod,
  GuidanceDamping,
  MaximumRoll,
  MaximumRollSetpointRate,
  Count,
};

using Px4CourseHoldParameterMetadata =
    ParameterMetadata<Px4CourseHoldParameter>;

inline constexpr std::array<Px4CourseHoldParameterMetadata,
    static_cast<std::size_t>(Px4CourseHoldParameter::Count)>
    Px4CourseHoldParameters{{
        {Px4CourseHoldParameter::GuidancePeriod,
            "NPFG_PERIOD",
            "Lateral guidance response period",
            UnitId::Second,
            UnitId::Second,
            1.0,
            100.0,
            10.0,
            0.1},
        {Px4CourseHoldParameter::GuidanceDamping,
            "NPFG_DAMPING",
            "Lateral guidance damping ratio",
            UnitId::Dimensionless,
            UnitId::Dimensionless,
            0.1,
            1.0,
            0.7,
            0.01},
        {Px4CourseHoldParameter::MaximumRoll,
            "FW_R_LIM",
            "Maximum course-control roll setpoint",
            UnitId::Degree,
            UnitId::Degree,
            0.0,
            75.0,
            20.0,
            0.5},
        {Px4CourseHoldParameter::MaximumRollSetpointRate,
            "FW_PN_R_SLEW_MAX",
            "Maximum course-control roll setpoint slew rate; zero disables",
            UnitId::DegreePerSecond,
            UnitId::DegreePerSecond,
            0.0,
            180.0,
            30.0,
            1.0},
    }};

constexpr const Px4CourseHoldParameterMetadata &
GetPx4CourseHoldParameterMetadata(Px4CourseHoldParameter parameter) {
  return GetParameterMetadata(parameter, Px4CourseHoldParameters);
}
} // namespace gnc
