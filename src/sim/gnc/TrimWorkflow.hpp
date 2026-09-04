#pragma once

#include "sim/gnc/TrimTypes.hpp"

namespace control {
class FlightControlManager;
}

namespace sim {
class Aircraft;
struct InitialCondition;
} // namespace sim

namespace gnc {
class TrimService;

struct TrimWorkflowOptions {
  bool fromCurrentState = false;
  bool resetSimulationTime = false;
};

class TrimWorkflow {
public:
  static TrimRequest MakeRequest(const sim::InitialCondition &initialCondition,
      TrimMode mode);
  static bool Execute(TrimService &trimService, sim::Aircraft &aircraft,
      control::FlightControlManager &flightControls, const TrimRequest &request,
      const TrimWorkflowOptions &options = {});
};
} // namespace gnc
