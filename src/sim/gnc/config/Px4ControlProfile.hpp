#pragma once

#include "sim/gnc/control/attitude/Px4PitchController.hpp"
#include "sim/gnc/control/attitude/Px4RollController.hpp"
#include "sim/gnc/control/lateral/Px4CourseController.hpp"
#include "sim/gnc/control/yaw/Px4YawRateController.hpp"
#include "sim/gnc/tecs/Px4TecsController.hpp"

#include <string_view>

namespace gnc {
struct Px4ControlProfile {
  std::string_view aircraftId;
  Px4CourseHoldSettings course;
  Px4RollHoldReferenceSettings roll;
  Px4PitchHoldSettings pitch;
  Px4YawRateSettings yaw;
  Px4TecsSettings tecs;
};

Px4ControlProfile MakeC172xPx4ControlProfile();
const Px4ControlProfile &GetC172xPx4ControlProfile();
} // namespace gnc
