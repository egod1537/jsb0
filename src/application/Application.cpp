#include "Application.hpp"
#include "application/gui/GUI.hpp"
#include "application/input/Input.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlManager.hpp"
#include "application/sim/gnc/autopilot/IRollHoldAutopilot.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

namespace {
using Clock = std::chrono::steady_clock;

Clock::duration ToClockDuration(double seconds) {
  return std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
}

double ClampAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return application::MinimumAutomaticSimulationHz;
  }

  return std::clamp(hz,
      application::MinimumAutomaticSimulationHz,
      application::MaximumAutomaticSimulationHz);
}

Clock::duration ToSimulationInterval(double hz) {
  return ToClockDuration(1.0 / ClampAutomaticSimulationHz(hz));
}

sim::InitialCondition MakeScenarioInitialCondition(
    const sim::SimulationScenario &scenario,
    const sim::InitialCondition &reference) {
  sim::InitialCondition initialCondition = reference;
  initialCondition.altitudeFt = scenario.altitudeFt;
  initialCondition.airspeedKts = scenario.airspeedKts;
  initialCondition.rollDeg = scenario.initialRollDeg;
  initialCondition.pitchDeg = scenario.initialPitchDeg;
  initialCondition.headingDeg = scenario.initialHeadingDeg;
  initialCondition.pRadPerSec = 0.0;
  initialCondition.qRadPerSec = 0.0;
  initialCondition.rRadPerSec = 0.0;
  return initialCondition;
}

sim::FDMEnvironmentState MakeScenarioEnvironment(
    const sim::Simulation &reference, bool windEnabled) {
  sim::FDMEnvironmentState environment =
      reference.GetAircraft()
          .ExtractFDMState(sim::FDMStateFlags::Environment)
          .environment;
  if (!windEnabled) {
    environment.windNedFps.fill(0.0);
    environment.gustNedFps.fill(0.0);
    environment.turbulenceNedFps.fill(0.0);
    environment.turbulenceType = 0;
    environment.turbulenceGain = 0.0;
    environment.turbulenceRate = 0.0;
    environment.turbulenceRhythmicity = 0.0;
    environment.windSpeedAt20FtFps = 0.0;
  }
  return environment;
}

gnc::IRollHoldAutopilot *FindScenarioRollHold(sim::Simulation &simulation) {
  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  if (manager == nullptr) {
    return nullptr;
  }

  return dynamic_cast<gnc::IRollHoldAutopilot *>(&manager->GetAutopilot());
}

bool ConfigureScenarioRollHold(sim::Simulation &simulation,
    double targetRollRad, bool enabled) {
  auto *rollHold = FindScenarioRollHold(simulation);
  if (rollHold == nullptr) {
    return false;
  }

  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  rollHold->SetTargetRollRad(targetRollRad);
  rollHold->SetRollHoldEnabled(enabled);
  manager->SetMode(enabled ? control::FlightControlMode::Autopilot
                           : control::FlightControlMode::Manual);
  return true;
}

bool SupportsScenarioRollHold(sim::Simulation &simulation) {
  return FindScenarioRollHold(simulation) != nullptr;
}
} // namespace

Application::Application(std::unique_ptr<gui::GUI> gui,
    std::unique_ptr<sim::Simulation> primarySimulation,
    sim::SimulationConfig simConfig,
    std::unique_ptr<sim::Simulation> baselineSimulation)
    : primarySimulation_(std::move(primarySimulation)),
      baselineSimulation_(std::move(baselineSimulation)), gui_(std::move(gui)),
      simConfig_(std::move(simConfig)),
      automaticSimulationHz_(
          ClampAutomaticSimulationHz(simConfig_.simulationHz)) {}

Application::Application() = default;

Application::~Application() = default;

bool Application::Run(const volatile std::sig_atomic_t &running) {
  if (!Start()) {
    Exit();
    return false;
  }

  bool succeeded = true;
  double scheduledSimulationHz = automaticSimulationHz_;
  bool scheduledMaximumSimulationSpeed = maximumSimulationSpeedEnabled_;
  Clock::duration simulationInterval =
      ToSimulationInterval(scheduledSimulationHz);
  const double guiDt = gui_->GetConfig().GetRenderDT();
  const Clock::duration guiInterval =
      guiDt > 0.0 ? ToClockDuration(guiDt) : simulationInterval;

  auto nextSimulationTick = Clock::now();
  auto nextGUITick = nextSimulationTick;

  while (succeeded && running && !gui_->ShouldClose()) {
    auto now = Clock::now();

    if (maximumSimulationSpeedEnabled_ != scheduledMaximumSimulationSpeed) {
      scheduledMaximumSimulationSpeed = maximumSimulationSpeedEnabled_;
      nextSimulationTick = now + simulationInterval;
    }

    if (automaticSimulationHz_ != scheduledSimulationHz) {
      scheduledSimulationHz = automaticSimulationHz_;
      simulationInterval = ToSimulationInterval(scheduledSimulationHz);
      if (!maximumSimulationSpeedEnabled_) {
        nextSimulationTick = now + simulationInterval;
      }
    }

    const bool hasPendingManualTick =
        simulationExecutionState_
            == application::SimulationExecutionState::Paused
        && pendingSimulationTicks_ > 0;

    const bool runAtMaximumSpeed =
        maximumSimulationSpeedEnabled_
        && simulationExecutionState_
               == application::SimulationExecutionState::Running;

    if (hasPendingManualTick) {
      if (!TickSimulation()) {
        succeeded = false;
      }
    } else if (runAtMaximumSpeed) {
      do {
        if (!TickSimulation()) {
          succeeded = false;
          break;
        }
        now = Clock::now();
      } while (running && now < nextGUITick);
    } else if (simulationExecutionState_
               == application::SimulationExecutionState::Paused) {
      nextSimulationTick = now + simulationInterval;
    } else {
      while (now >= nextSimulationTick) {
        if (!TickSimulation()) {
          succeeded = false;
          break;
        }

        nextSimulationTick += simulationInterval;
        now = Clock::now();
      }
    }

    if (!succeeded) {
      break;
    }

    if (now >= nextGUITick) {
      TickGUI();
      now = Clock::now();
      do {
        nextGUITick += guiInterval;
      } while (nextGUITick <= now);
    }

    if (!runAtMaximumSpeed) {
      std::this_thread::sleep_until(std::min(nextSimulationTick, nextGUITick));
    }
  }

  Exit();
  return succeeded;
}

bool Application::Start() {
  if (gui_ == nullptr || primarySimulation_ == nullptr) {
    std::cerr << "Application requires GUI and simulation instances\n";
    return false;
  }

  gui_->SetSimulationExecutionControl(this);

  if (!application::Input::Initialize()) {
    std::cerr << "Failed to initialize input\n";
    return false;
  }

  if (!primarySimulation_->Initialize(simConfig_)) {
    std::cerr << "Failed to initialize simulation\n";
    return false;
  }

  if (baselineSimulation_ != nullptr
      && !baselineSimulation_->Initialize(simConfig_)) {
    std::cerr << "Failed to initialize baseline simulation\n";
    return false;
  }
  if (!flightGear_.Initialize()) {
    return false;
  }

  if (!gui_->Start()) {
    std::cerr << "Failed to start GUI\n";
    return false;
  }

  simulationExecutionState_ = application::SimulationExecutionState::Stopped;
  pendingSimulationTicks_ = 0;
  return true;
}

bool Application::TickSimulation() {
  const bool isPaused = simulationExecutionState_
                        == application::SimulationExecutionState::Paused;
  if (isPaused && pendingSimulationTicks_ == 0) {
    return true;
  }

  if (simulationExecutionState_
          != application::SimulationExecutionState::Running
      && !isPaused) {
    return true;
  }

  application::Input::Update();

  if (activeScenario_.has_value() ? !ApplyScenarioControlState()
                                  : !SynchronizeBaselineControlState()) {
    return false;
  }

  const double sharedDtSec = primarySimulation_->GetTickSizeSec();
  if (!primarySimulation_->Step(sharedDtSec)) {
    return false;
  }

  if (baselineSimulation_ != nullptr) {
    if (std::abs(baselineSimulation_->GetTickSizeSec() - sharedDtSec)
        > 1.0e-12) {
      std::cerr << "Primary and Baseline simulation dt do not match\n";
      return false;
    }
    if (!baselineSimulation_->Step(sharedDtSec)) {
      return false;
    }
  }

  flightGear_.Update(primarySimulation_->GetAircraft());

  if (activeScenario_.has_value()) {
    AdvanceScenarioClock(sharedDtSec);
    if (activeScenario_->status.elapsedSec
        >= activeScenario_->scenario.durationSec) {
      FinishScenario();
    }
  }

  if (isPaused) {
    --pendingSimulationTicks_;
  }

  return true;
}

bool Application::RunScenario(const sim::SimulationScenario &scenario) {
  if (simulationExecutionState_
          != application::SimulationExecutionState::Stopped
      || primarySimulation_ == nullptr || !primarySimulation_->IsInitialized()
      || !sim::ValidateSimulationScenario(scenario)
      || !SupportsScenarioRollHold(*primarySimulation_)
      || (baselineSimulation_ != nullptr
          && (!baselineSimulation_->IsInitialized()
              || !SupportsScenarioRollHold(*baselineSimulation_)))) {
    return false;
  }

  const sim::InitialCondition initialCondition =
      MakeScenarioInitialCondition(scenario,
          primarySimulation_->GetDefaultInitialCondition());
  const sim::FDMEnvironmentState environment =
      MakeScenarioEnvironment(*primarySimulation_, scenario.windEnabled);
  const sim::SimulationResetOptions resetOptions{
      .runTrim = scenario.runTrim,
      .trimMode = scenario.trimMode,
      .environment = environment,
  };
  if (!primarySimulation_->Reset(initialCondition, resetOptions)) {
    return false;
  }
  if (baselineSimulation_ != nullptr
      && !baselineSimulation_->Reset(initialCondition, resetOptions)) {
    return false;
  }

  activeScenario_ = ActiveScenarioExecution{
      .scenario = scenario,
      .status =
          application::ScenarioExecutionStatus{
              .name = scenario.name,
              .elapsedSec = 0.0,
              .durationSec = scenario.durationSec,
          },
  };
  pendingSimulationTicks_ = 0;
  if (!ApplyScenarioControlState()) {
    activeScenario_.reset();
    return false;
  }

  flightGear_.Update(primarySimulation_->GetAircraft());
  simulationExecutionState_ = application::SimulationExecutionState::Running;
  return true;
}

void Application::StartSimulation() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Stopped) {
    pendingSimulationTicks_ = 0;
    simulationExecutionState_ = application::SimulationExecutionState::Running;
  }
}

void Application::StopSimulation() {
  if (activeScenario_.has_value()) {
    FinishScenario();
    return;
  }

  pendingSimulationTicks_ = 0;
  simulationExecutionState_ = application::SimulationExecutionState::Stopped;
}

void Application::PauseSimulation() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Running) {
    simulationExecutionState_ = application::SimulationExecutionState::Paused;
  }
}

void Application::ResumeSimulation() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Paused) {
    pendingSimulationTicks_ = 0;
    simulationExecutionState_ = application::SimulationExecutionState::Running;
  }
}

void Application::RequestSimulationTick() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Paused) {
    ++pendingSimulationTicks_;
  }
}

void Application::SetAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return;
  }

  automaticSimulationHz_ = ClampAutomaticSimulationHz(hz);
  maximumSimulationSpeedEnabled_ = false;
}

void Application::SetMaximumSimulationSpeedEnabled(bool enabled) {
  maximumSimulationSpeedEnabled_ = enabled;
}

bool Application::ResetSimulation() {
  return !activeScenario_.has_value() && ResetSimulations(nullptr);
}

bool Application::ResetSimulation(
    const sim::InitialCondition &initialCondition) {
  return !activeScenario_.has_value() && ResetSimulations(&initialCondition);
}

bool Application::SynchronizeBaselineControlState() {
  if (baselineSimulation_ == nullptr) {
    return true;
  }

  auto *primaryManager =
      primarySimulation_->GetComponent<control::FlightControlManager>();
  auto *baselineManager =
      baselineSimulation_->GetComponent<control::FlightControlManager>();
  if (primaryManager == nullptr || baselineManager == nullptr) {
    std::cerr << "Failed to synchronize baseline flight controls\n";
    return false;
  }

  baselineManager->GetManualController().SetCommandedInput(
      primaryManager->GetManualController().GetCommandedInput());
  return true;
}

bool Application::ApplyScenarioControlState() {
  if (!activeScenario_.has_value() || primarySimulation_ == nullptr) {
    return false;
  }

  const bool commandActive = activeScenario_->status.elapsedSec
                             >= activeScenario_->scenario.commandStartSec;
  const double targetRollRad =
      math::DegToRad(activeScenario_->scenario.commandedRollDeg);
  if (!ConfigureScenarioRollHold(*primarySimulation_,
          targetRollRad,
          commandActive)) {
    return false;
  }
  return baselineSimulation_ == nullptr
         || ConfigureScenarioRollHold(*baselineSimulation_,
             targetRollRad,
             commandActive);
}

void Application::AdvanceScenarioClock(double dtSec) {
  if (!activeScenario_.has_value()) {
    return;
  }

  activeScenario_->status.elapsedSec =
      std::clamp(activeScenario_->status.elapsedSec + dtSec,
          0.0,
          activeScenario_->scenario.durationSec);
}

void Application::FinishScenario() {
  if (activeScenario_.has_value()) {
    const double targetRollRad =
        math::DegToRad(activeScenario_->scenario.commandedRollDeg);
    ConfigureScenarioRollHold(*primarySimulation_, targetRollRad, false);
    if (baselineSimulation_ != nullptr) {
      ConfigureScenarioRollHold(*baselineSimulation_, targetRollRad, false);
    }
  }

  activeScenario_.reset();
  pendingSimulationTicks_ = 0;
  simulationExecutionState_ = application::SimulationExecutionState::Stopped;
}

bool Application::ResetSimulations(
    const sim::InitialCondition *initialCondition) {
  const auto reset = [initialCondition](sim::Simulation &simulation) {
    return initialCondition != nullptr ? simulation.Reset(*initialCondition)
                                       : simulation.Reset();
  };

  if (!reset(*primarySimulation_)) {
    return false;
  }
  if (baselineSimulation_ != nullptr && !reset(*baselineSimulation_)) {
    return false;
  }

  flightGear_.Update(primarySimulation_->GetAircraft());
  return true;
}

void Application::TickGUI() { gui_->Tick(); }

void Application::Exit() {
  if (activeScenario_.has_value()) {
    FinishScenario();
  }

  simulationExecutionState_ = application::SimulationExecutionState::Stopped;
  pendingSimulationTicks_ = 0;

  if (gui_ != nullptr) {
    gui_->Exit();
  }

  flightGear_.Shutdown();

  if (baselineSimulation_ != nullptr) {
    baselineSimulation_->Shutdown();
  }

  if (primarySimulation_ != nullptr) {
    primarySimulation_->Shutdown();
  }

  application::Input::Shutdown();
}
