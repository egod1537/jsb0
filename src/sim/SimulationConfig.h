#pragma once
#include <string>

namespace sim {
inline constexpr double DefaultSimulationHz = 120.0;
inline constexpr double DefaultSimulationDtSec = 1.0 / DefaultSimulationHz;

struct SimulationConfig {
  std::string aircraftName = "c172x";

  double simulationHz = DefaultSimulationHz;

  double GetDT() const { return 1.0 / simulationHz; }
};
} // namespace sim
