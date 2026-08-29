#include "sim/scenario/ScenarioExecutor.hpp"

#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/autopilot/IRollHoldAutopilot.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim {
namespace {
InitialCondition MakeInitialCondition(const SimulationScenario &scenario,
    const InitialCondition &reference) {
  InitialCondition initialCondition = reference;
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

FDMEnvironmentState MakeEnvironment(const Simulation &reference,
    bool windEnabled) {
  FDMEnvironmentState environment =
      reference.GetAircraft()
          .ExtractFDMState(FDMStateFlags::Environment)
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

gnc::IRollHoldAutopilot *FindRollHold(Simulation &simulation) {
  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  return manager != nullptr
             ? dynamic_cast<gnc::IRollHoldAutopilot *>(
                   &manager->GetAutopilot())
             : nullptr;
}

bool ConfigureRollHold(Simulation &simulation, double targetRollRad,
    bool enabled) {
  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  auto *rollHold = FindRollHold(simulation);
  if (manager == nullptr || rollHold == nullptr) {
    return false;
  }
  rollHold->SetTargetRollRad(targetRollRad);
  rollHold->SetRollHoldEnabled(enabled);
  manager->SetMode(enabled ? control::FlightControlMode::Autopilot
                           : control::FlightControlMode::Manual);
  return true;
}
} // namespace

ScenarioExecutor::ScenarioExecutor(Simulation &primary, Simulation *baseline)
    : primary_(primary), baseline_(baseline) {}

bool ScenarioExecutor::Start(const SimulationScenario &scenario,
    double dtSec) {
  if (state_ == ScenarioExecutorState::Running) {
    return Fail("scenario executor is already running");
  }
  lastError_.clear();
  pendingCommandActivationTimeSec_.reset();
  if (!primary_.IsInitialized()
      || (baseline_ != nullptr && !baseline_->IsInitialized())) {
    return Fail("all scenario simulations must be initialized");
  }
  std::string validationError;
  if (!ValidateSimulationScenario(scenario, &validationError)) {
    return Fail(validationError);
  }
  const auto targetSteps = CalculateStepCount(scenario.durationSec, dtSec);
  if (!targetSteps) {
    return Fail("scenario duration and timestep produce an invalid step count");
  }
  if (std::abs(primary_.GetTickSizeSec() - dtSec) > 1.0e-12
      || (baseline_ != nullptr
          && std::abs(baseline_->GetTickSizeSec() - dtSec) > 1.0e-12)) {
    return Fail("simulation configuration timestep does not match executor timestep");
  }
  if (FindRollHold(primary_) == nullptr
      || (baseline_ != nullptr && FindRollHold(*baseline_) == nullptr)) {
    return Fail("scenario simulation does not support Roll Hold");
  }

  scenario_ = scenario;
  dtSec_ = dtSec;
  targetStepCount_ = *targetSteps;
  stepCount_ = 0;
  commandActive_ = false;
  if (!ResetSimulations()) {
    return Fail("failed to reset scenario simulation");
  }
  state_ = ScenarioExecutorState::Running;
  if (!ApplyControlState()) {
    return Fail("failed to apply initial scenario control state");
  }
  return true;
}

ScenarioStepResult ScenarioExecutor::Step() {
  ScenarioStepResult result;
  if (state_ != ScenarioExecutorState::Running) {
    lastError_ = "scenario executor is not running";
    return result;
  }
  if (!ApplyControlState()) {
    Fail("failed to apply scenario control state");
    return result;
  }
  result.commandActivationTimeSec = TakeCommandActivationTime();
  if (!primary_.Step(dtSec_)) {
    Fail("primary simulation step failed");
    return result;
  }
  if (baseline_ != nullptr && !baseline_->Step(dtSec_)) {
    Fail("baseline simulation step failed");
    return result;
  }

  ++stepCount_;
  result.succeeded = true;
  if (stepCount_ >= targetStepCount_) {
    DisableRollHold();
    state_ = ScenarioExecutorState::Completed;
    result.completed = true;
  }
  return result;
}

void ScenarioExecutor::Stop() {
  if (state_ == ScenarioExecutorState::Running) {
    DisableRollHold();
    state_ = ScenarioExecutorState::Stopped;
  }
}

ScenarioExecutorState ScenarioExecutor::GetState() const { return state_; }

bool ScenarioExecutor::IsRunning() const {
  return state_ == ScenarioExecutorState::Running;
}

bool ScenarioExecutor::IsFinished() const {
  return state_ == ScenarioExecutorState::Completed;
}

double ScenarioExecutor::GetElapsedSec() const {
  return std::min(scenario_.durationSec,
      static_cast<double>(stepCount_) * dtSec_);
}

double ScenarioExecutor::GetStepSizeSec() const { return dtSec_; }

std::uint64_t ScenarioExecutor::GetStepCount() const { return stepCount_; }

std::uint64_t ScenarioExecutor::GetTargetStepCount() const {
  return targetStepCount_;
}

const SimulationScenario *ScenarioExecutor::GetScenario() const {
  return state_ == ScenarioExecutorState::Idle ? nullptr : &scenario_;
}

const std::string &ScenarioExecutor::GetLastError() const { return lastError_; }

std::optional<double> ScenarioExecutor::TakeCommandActivationTime() {
  return std::exchange(pendingCommandActivationTimeSec_, std::nullopt);
}

std::optional<std::uint64_t> ScenarioExecutor::CalculateStepCount(
    double durationSec, double dtSec) {
  if (!std::isfinite(durationSec) || durationSec <= 0.0
      || !std::isfinite(dtSec) || dtSec <= 0.0) {
    return std::nullopt;
  }
  const double ratio = durationSec / dtSec;
  if (!std::isfinite(ratio)
      || ratio > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::nullopt;
  }
  const double nearest = std::round(ratio);
  const double tolerance = std::numeric_limits<double>::epsilon()
                           * std::max(1.0, std::abs(ratio)) * 8.0;
  const double steps = std::abs(ratio - nearest) <= tolerance
                           ? nearest
                           : std::ceil(ratio);
  if (steps < 1.0) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(steps);
}

bool ScenarioExecutor::ResetSimulations() {
  const InitialCondition initialCondition =
      MakeInitialCondition(scenario_, primary_.GetDefaultInitialCondition());
  const FDMEnvironmentState environment =
      MakeEnvironment(primary_, scenario_.windEnabled);
  const SimulationResetOptions options{
      .runTrim = scenario_.runTrim,
      .trimMode = scenario_.trimMode,
      .environment = environment,
  };
  return primary_.Reset(initialCondition, options)
         && (baseline_ == nullptr || baseline_->Reset(initialCondition, options));
}

bool ScenarioExecutor::ApplyControlState() {
  const bool commandActive = GetElapsedSec() >= scenario_.commandStartSec;
  const double targetRollRad = math::DegToRad(scenario_.commandedRollDeg);
  if (!ConfigureRollHold(primary_, targetRollRad, commandActive)
      || (baseline_ != nullptr
          && !ConfigureRollHold(*baseline_, targetRollRad, commandActive))) {
    return false;
  }
  if (commandActive && !commandActive_) {
    commandActive_ = true;
    pendingCommandActivationTimeSec_ = primary_.GetTime();
  }
  return true;
}

void ScenarioExecutor::DisableRollHold() {
  const double targetRollRad = math::DegToRad(scenario_.commandedRollDeg);
  ConfigureRollHold(primary_, targetRollRad, false);
  if (baseline_ != nullptr) {
    ConfigureRollHold(*baseline_, targetRollRad, false);
  }
}

bool ScenarioExecutor::Fail(std::string message) {
  if (state_ == ScenarioExecutorState::Running) {
    DisableRollHold();
  }
  state_ = ScenarioExecutorState::Failed;
  lastError_ = std::move(message);
  return false;
}
} // namespace sim
