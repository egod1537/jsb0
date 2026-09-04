#pragma once

#include <memory>

namespace gnc {
class IAutopilot;

std::unique_ptr<IAutopilot> CreateExperimentalAutopilot();
bool IsExperimentalAutopilot(const IAutopilot &autopilot);
} // namespace gnc
