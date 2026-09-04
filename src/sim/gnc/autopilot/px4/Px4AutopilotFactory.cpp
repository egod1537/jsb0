#include "sim/gnc/autopilot/px4/Px4AutopilotFactory.hpp"

#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/gnc/config/Px4ControlProfile.hpp"

namespace gnc {
std::unique_ptr<IAutopilot> CreateC172xPx4Autopilot() {
  return CreatePx4Autopilot(MakeC172xPx4ControlProfile());
}

std::unique_ptr<IAutopilot> CreatePx4Autopilot(
    const Px4ControlProfile &profile) {
  return std::make_unique<PX4Autopilot>(profile);
}

bool IsPx4Autopilot(const IAutopilot &autopilot) {
  return dynamic_cast<const PX4Autopilot *>(&autopilot) != nullptr;
}
} // namespace gnc
