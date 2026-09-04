#pragma once

#include "sim/runtime/SimulationContracts.hpp"

namespace sim {
class Simulation;

class SimulationSnapshotBuilder {
public:
  static SimulationInstanceSnapshot CaptureInstance(
      const Simulation &simulation);
  static AutopilotSnapshot CaptureAutopilot(const Simulation &simulation);
  static LinearizationSnapshot CaptureLinearization(
      const Simulation &simulation);
};
} // namespace sim
