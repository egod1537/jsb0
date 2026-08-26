#pragma once

#include "application/SimulationExecutionControl.hpp"
#include "application/flightgear/FlightGearSystem.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/scenario/SimulationScenario.hpp"

#include <csignal>
#include <cstdint>
#include <memory>

namespace sim {
class Simulation;
}
namespace gui {
class GUI;
}

class Application : public application::SimulationExecutionControl {
  struct ActiveScenarioExecution {
    sim::SimulationScenario scenario;
    application::ScenarioExecutionStatus status;
  };

public:
  // Lifetime and main loop
  Application();
  ~Application();
  Application(std::unique_ptr<gui::GUI> gui,
      std::unique_ptr<sim::Simulation> primarySimulation,
      sim::SimulationConfig simConfig,
      std::unique_ptr<sim::Simulation> baselineSimulation = nullptr);
  bool Run(const volatile std::sig_atomic_t &running);

  // Simulation execution control
  bool RunScenario(const sim::SimulationScenario &scenario) override;
  std::optional<application::ScenarioExecutionStatus>
  GetScenarioExecutionStatus() const override {
    return activeScenario_.has_value() ? std::optional(activeScenario_->status)
                                       : std::nullopt;
  }
  application::SimulationExecutionState
  GetSimulationExecutionState() const override {
    return simulationExecutionState_;
  }
  void StartSimulation() override;
  void StopSimulation() override;
  void PauseSimulation() override;
  void ResumeSimulation() override;
  void RequestSimulationTick() override;
  double GetAutomaticSimulationHz() const override {
    return automaticSimulationHz_;
  }
  void SetAutomaticSimulationHz(double hz) override;
  bool IsMaximumSimulationSpeedEnabled() const override {
    return maximumSimulationSpeedEnabled_;
  }
  void SetMaximumSimulationSpeedEnabled(bool enabled) override;
  bool ResetSimulation() override;
  bool ResetSimulation(const sim::InitialCondition &initialCondition) override;
  std::uint32_t GetPendingSimulationTickCount() const override {
    return pendingSimulationTicks_;
  }

private:
  // Application lifecycle
  bool Start();
  bool TickSimulation();
  void TickGUI();
  void Exit();

  // Baseline coordination
  bool SynchronizeBaselineControlState();
  bool ResetSimulations(const sim::InitialCondition *initialCondition);

  // Scenario execution
  bool ApplyScenarioControlState();
  void AdvanceScenarioClock(double dtSec);
  void FinishScenario();

  // Owned services
  std::unique_ptr<sim::Simulation> primarySimulation_;
  std::unique_ptr<sim::Simulation> baselineSimulation_;
  std::unique_ptr<gui::GUI> gui_;
  flightgear::FlightGearSystem flightGear_;

  // Configuration
  sim::SimulationConfig simConfig_;

  // Execution state
  std::optional<ActiveScenarioExecution> activeScenario_;
  application::SimulationExecutionState simulationExecutionState_ =
      application::SimulationExecutionState::Stopped;
  double automaticSimulationHz_ = sim::DefaultSimulationHz;
  bool maximumSimulationSpeedEnabled_ = false;
  std::uint32_t pendingSimulationTicks_ = 0;
};
