#pragma once

#include "sim/scenario/SimulationScenario.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace sim {
class Simulation;

enum class ScenarioExecutorState {
  Idle,
  Running,
  Completed,
  Stopped,
  Failed,
};

struct ScenarioStepResult {
  bool succeeded = false;
  bool completed = false;
  std::optional<double> commandActivationTimeSec;
};

class ScenarioExecutor {
public:
  ScenarioExecutor(Simulation &primary, Simulation *baseline = nullptr);

  // Execution lifecycle
  bool Start(const SimulationScenario &scenario, double dtSec);
  ScenarioStepResult Step();
  void Stop();

  // Run state
  ScenarioExecutorState GetState() const;
  bool IsRunning() const;
  bool IsFinished() const;
  double GetElapsedSec() const;
  double GetStepSizeSec() const;
  std::uint64_t GetStepCount() const;
  std::uint64_t GetTargetStepCount() const;
  const SimulationScenario *GetScenario() const;
  const std::string &GetLastError() const;
  std::optional<double> TakeCommandActivationTime();

  // Deterministic duration policy
  static std::optional<std::uint64_t> CalculateStepCount(double durationSec,
      double dtSec);

private:
  bool ResetSimulations();
  bool ApplyControlState();
  void DisableRollHold();
  bool Fail(std::string message);

  // Simulation dependencies
  Simulation &primary_;
  Simulation *baseline_ = nullptr;

  // Run configuration
  SimulationScenario scenario_;
  double dtSec_ = 0.0;
  std::uint64_t targetStepCount_ = 0;

  // Runtime state
  ScenarioExecutorState state_ = ScenarioExecutorState::Idle;
  std::uint64_t stepCount_ = 0;
  bool commandActive_ = false;
  std::optional<double> pendingCommandActivationTimeSec_;
  std::string lastError_;
};
} // namespace sim
