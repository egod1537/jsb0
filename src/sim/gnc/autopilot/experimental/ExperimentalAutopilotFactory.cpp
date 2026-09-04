#include "sim/gnc/autopilot/experimental/ExperimentalAutopilotFactory.hpp"

#include "sim/gnc/autopilot/MyAutopilot.hpp"

namespace gnc {
std::unique_ptr<IAutopilot> CreateExperimentalAutopilot() {
  return std::make_unique<MyAutopilot>();
}

bool IsExperimentalAutopilot(const IAutopilot &autopilot) {
  return dynamic_cast<const MyAutopilot *>(&autopilot) != nullptr;
}
} // namespace gnc
