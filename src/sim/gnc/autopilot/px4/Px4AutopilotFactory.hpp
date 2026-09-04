#pragma once

#include <memory>

namespace gnc {
class IAutopilot;
struct Px4ControlProfile;

std::unique_ptr<IAutopilot> CreateC172xPx4Autopilot();
std::unique_ptr<IAutopilot> CreatePx4Autopilot(
    const Px4ControlProfile &profile);
bool IsPx4Autopilot(const IAutopilot &autopilot);
} // namespace gnc
