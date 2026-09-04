#pragma once

#include "sim/Aircraft.hpp"
#include "sim/FDMState.hpp"

namespace gnc {
struct LinearizationResult;
}

namespace sim {
class AircraftLinearizer {
public:
  bool Initialize(std::string_view aircraftName, double simulationHz,
      const InitialCondition &initialCondition);
  gnc::LinearizationResult Linearize(const FDMState &sourceState);

private:
  void SyncFrom(const FDMState &sourceState);

  Aircraft aircraft_;
  bool initialized_ = false;
};
} // namespace sim
