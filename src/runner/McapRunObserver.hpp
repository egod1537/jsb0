#pragma once

#include "SimulationRunner.hpp"

namespace runner {
class McapRunObserver final : public ISimulationRunObserver {
public:
  bool OnRunStarted(const SimulationRunInfo &info,
      sim::SimulationRuntime &runtime, std::string &error) override;
  bool OnSimulationStep(const SimulationRunInfo &info,
      sim::SimulationRuntime &runtime, std::string &error) override;
  bool OnRunFinished(const SimulationRunInfo &info,
      sim::SimulationRuntime &runtime, const RunnerResult &result,
      std::string &error) override;

private:
  bool started_ = false;
};
} // namespace runner
