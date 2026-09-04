#pragma once

#include "sim/linearization/DynamicModeHistory.hpp"
#include "sim/linearization/LinearizationResult.hpp"

#include <optional>
#include <string>

namespace sim {
class Simulation;

struct LinearizationServiceState {
  bool available = false;
  bool automaticUpdatesEnabled = false;
  bool updateInProgress = false;
  std::string errorMessage;
  std::optional<gnc::LinearizationResult> result;
  gnc::DynamicModeHistory dynamicModeHistory;
};

class LinearizationService {
public:
  bool SetAutomaticUpdates(Simulation &simulation, bool enabled) const;
  LinearizationServiceState Capture(const Simulation &simulation) const;
};
} // namespace sim
