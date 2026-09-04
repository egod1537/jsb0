#pragma once

#include <string>

namespace sim {
class Simulation;
struct InitialCondition;
struct SimulationConfig;

class SimulationInstanceSet {
public:
  // Shared lifecycle for the interactive primary/baseline pair
  SimulationInstanceSet(Simulation &primary, Simulation *baseline);

  bool Initialize(const SimulationConfig &config, std::string &error) const;
  void Shutdown() const;
  bool Reset(const InitialCondition *initialCondition,
      std::string &error) const;
  bool Step(double dtSec, std::string &error) const;

private:
  // Per-tick synchronization
  bool SynchronizeControlState(std::string &error) const;

  // Non-owning instance references
  Simulation &primary_;
  Simulation *baseline_ = nullptr;
};
} // namespace sim
