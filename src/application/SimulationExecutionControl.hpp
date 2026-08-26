#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace sim {
struct InitialCondition;
struct SimulationScenario;
}

namespace application {
inline constexpr double MinimumAutomaticSimulationHz = 1.0;
inline constexpr double MaximumAutomaticSimulationHz = 1000.0;

enum class SimulationExecutionState {
  Running,
  Paused,
  Stopped,
};

struct ScenarioExecutionStatus {
  std::string name;
  double elapsedSec = 0.0;
  double durationSec = 0.0;
};

inline const char *ToString(SimulationExecutionState state) {
  switch (state) {
  case SimulationExecutionState::Running:
    return "Running";
  case SimulationExecutionState::Paused:
    return "Paused";
  case SimulationExecutionState::Stopped:
    return "Stopped";
  }

  return "Unknown";
}

class SimulationExecutionControl {
public:
  virtual ~SimulationExecutionControl() = default;

  virtual bool RunScenario(const sim::SimulationScenario &scenario) = 0;
  virtual std::optional<ScenarioExecutionStatus>
  GetScenarioExecutionStatus() const = 0;
  virtual SimulationExecutionState GetSimulationExecutionState() const = 0;
  virtual void StartSimulation() = 0;
  virtual void StopSimulation() = 0;
  virtual void PauseSimulation() = 0;
  virtual void ResumeSimulation() = 0;
  virtual void RequestSimulationTick() = 0;
  virtual double GetAutomaticSimulationHz() const = 0;
  virtual void SetAutomaticSimulationHz(double hz) = 0;
  virtual bool IsMaximumSimulationSpeedEnabled() const = 0;
  virtual void SetMaximumSimulationSpeedEnabled(bool enabled) = 0;
  virtual bool ResetSimulation() = 0;
  virtual bool ResetSimulation(
      const sim::InitialCondition &initialCondition) = 0;
  virtual std::uint32_t GetPendingSimulationTickCount() const = 0;
};
} // namespace application
