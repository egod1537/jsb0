#pragma once

#include <string>
#include <string_view>

namespace sim {
class Simulation;
struct InitialCondition;

class SimInstanceSet {
public:
  // Shared lifecycle for the interactive primary/baseline pair
  SimInstanceSet(Simulation &primary, Simulation *baseline);

  bool Initialize(std::string_view aircraftName, double simulationHz,
      std::string &error) const;
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
