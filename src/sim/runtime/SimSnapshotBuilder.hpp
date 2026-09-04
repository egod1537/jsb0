#pragma once

#include "sim/runtime/SimContracts.hpp"

namespace sim {
class Simulation;

class SimSnapshotBuilder {
public:
  static SimInstanceSnapshot CaptureInstance(
      const Simulation &simulation);
  static AutopilotSnapshot CaptureAutopilot(const Simulation &simulation);
  static LinearizationSnapshot CaptureLinearization(
      const Simulation &simulation);
};
} // namespace sim
