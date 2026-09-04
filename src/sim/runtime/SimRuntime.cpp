#include "sim/runtime/SimRuntime.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Simulation.hpp"
#include "sim/analysis/LinearizationService.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/gnc/TrimWorkflow.hpp"
#include "sim/runtime/AutopilotConfigurationService.hpp"
#include "sim/runtime/SimInstanceSet.hpp"
#include "sim/runtime/SimSnapshotBuilder.hpp"
#include "sim/scenario/ScenarioExecutor.hpp"
#include "sim/scenario/SimScenario.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace sim {
namespace {
double ClampAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return MinimumAutomaticSimulationHz;
  }

  return std::clamp(hz,
      MinimumAutomaticSimulationHz,
      MaximumAutomaticSimulationHz);
}

} // namespace

SimRuntime::SimRuntime(
    std::unique_ptr<Simulation> primarySimulation,
    std::unique_ptr<Simulation> baselineSimulation)
    : primarySimulation_(std::move(primarySimulation)),
      baselineSimulation_(std::move(baselineSimulation)) {}

SimRuntime::~SimRuntime() { Shutdown(); }

std::unique_ptr<SimRuntime> SimRuntime::CreateForExecution(
    const ResolvedExecutionSpec &execution, std::string &error) {
  const SimScenario &scenario = execution.scenario;
  ScenarioValidationError validationError;
  if (!ValidateSimScenario(scenario, &validationError)) {
    error = validationError.ToString();
    return nullptr;
  }
  std::unique_ptr<gnc::IAutopilot> autopilot =
      ExecutionVariantResolver::CreateAutopilot(execution.variant);
  if (autopilot == nullptr) {
    error = "failed to construct execution variant '"
            + std::string(ToString(execution.variant)) + "'";
    return nullptr;
  }
  auto runtime = std::make_unique<SimRuntime>(
      std::make_unique<Simulation>(std::move(autopilot)));
  if (!runtime->Initialize(scenario.aircraft, 1.0 / scenario.dtSec)) {
    error = runtime->GetStatus().lastError;
    return nullptr;
  }
  error.clear();
  return runtime;
}

bool SimRuntime::Initialize(
    std::string_view aircraftName, double simulationHz) {
  if (initialized_) {
    return true;
  }
  if (primarySimulation_ == nullptr) {
    lastError_ = "Simulation runtime requires a primary simulation.";
    return false;
  }

  aircraftName_ = aircraftName;
  simulationHz_ = simulationHz;
  automaticSimulationHz_ = ClampAutomaticSimulationHz(simulationHz_);
  if (!SimInstanceSet(*primarySimulation_, baselineSimulation_.get())
          .Initialize(aircraftName_, simulationHz_, lastError_)) {
    return false;
  }

  executionState_ = SimExecutionState::Stopped;
  pendingTicks_ = 0;
  initialized_ = true;
  lastError_.clear();
  return true;
}

void SimRuntime::Shutdown() {
  if (!initialized_ && primarySimulation_ == nullptr) {
    return;
  }

  if (scenarioExecutor_ != nullptr) {
    FinishScenario();
  }
  telemetryRecording_.Stop();
  executionState_ = SimExecutionState::Stopped;
  pendingTicks_ = 0;

  if (primarySimulation_ != nullptr) {
    SimInstanceSet(*primarySimulation_, baselineSimulation_.get())
        .Shutdown();
  }
  initialized_ = false;
}

void SimRuntime::Start() {
  if (initialized_ && executionState_ == SimExecutionState::Stopped) {
    pendingTicks_ = 0;
    executionState_ = SimExecutionState::Running;
  }
}

void SimRuntime::Stop() {
  if (scenarioExecutor_ != nullptr) {
    FinishScenario();
    return;
  }
  pendingTicks_ = 0;
  executionState_ = SimExecutionState::Stopped;
}

void SimRuntime::Pause() {
  if (executionState_ == SimExecutionState::Running) {
    executionState_ = SimExecutionState::Paused;
  }
}

void SimRuntime::Resume() {
  if (executionState_ == SimExecutionState::Paused) {
    pendingTicks_ = 0;
    executionState_ = SimExecutionState::Running;
  }
}

bool SimRuntime::Reset() {
  return scenarioExecutor_ == nullptr && initialized_
         && primarySimulation_ != nullptr
         && SimInstanceSet(*primarySimulation_,
             baselineSimulation_.get())
                .Reset(nullptr, lastError_);
}

bool SimRuntime::Reset(const InitialCondition &initialCondition) {
  return scenarioExecutor_ == nullptr && initialized_
         && primarySimulation_ != nullptr
         && SimInstanceSet(*primarySimulation_,
             baselineSimulation_.get())
                .Reset(&initialCondition, lastError_);
}

void SimRuntime::RequestTick() {
  if (executionState_ == SimExecutionState::Paused) {
    ++pendingTicks_;
  }
}

bool SimRuntime::Tick() {
  const bool isPaused = executionState_ == SimExecutionState::Paused;
  if (isPaused && pendingTicks_ == 0) {
    return true;
  }
  if (executionState_ != SimExecutionState::Running && !isPaused) {
    return true;
  }

  const double sharedDtSec = primarySimulation_->GetTickSizeSec();
  ScenarioStepResult scenarioStep;
  if (scenarioExecutor_ != nullptr) {
    scenarioStep = scenarioExecutor_->Step();
    if (!scenarioStep.succeeded) {
      lastError_ = scenarioExecutor_->GetLastError();
      std::cerr << "Scenario step failed: " << lastError_ << '\n';
      return false;
    }
    RecordPendingScenarioCommandEvent();
  } else {
    if (!SimInstanceSet(*primarySimulation_, baselineSimulation_.get())
            .Step(sharedDtSec, lastError_)) {
      return false;
    }
  }

  telemetryRecording_.Consume(primarySimulation_->GetTime(),
      primarySimulation_->GetTelemetryRegistry(),
      scenarioExecutor_ == nullptr && baselineSimulation_ != nullptr
          ? &baselineSimulation_->GetTelemetryRegistry()
          : nullptr);

  if (scenarioExecutor_ != nullptr && scenarioStep.completed) {
    FinishScenario();
  }
  if (isPaused) {
    --pendingTicks_;
  }

  lastError_.clear();
  return true;
}

bool SimRuntime::RunExecution(const ResolvedExecutionSpec &execution) {
  const SimScenario &scenario = execution.scenario;
  if (!initialized_ || executionState_ != SimExecutionState::Stopped
      || primarySimulation_ == nullptr || !primarySimulation_->IsInitialized()
      || (baselineSimulation_ != nullptr
          && !baselineSimulation_->IsInitialized())) {
    return false;
  }

  std::string validationError;
  if (!ValidateSimScenario(scenario, &validationError)) {
    lastError_ = validationError;
    return false;
  }
  if (scenario.aircraft != aircraftName_
      || std::abs(scenario.dtSec - 1.0 / simulationHz_) > 1.0e-12) {
    if (!ReinitializeForScenario(scenario)) {
      return false;
    }
  }
  if (!SelectExecutionVariant(execution.variant)) {
    return false;
  }
  if (!AutopilotConfigurationService::ApplyExecutionParameters(
          *primarySimulation_,
          execution,
          lastError_)) {
    RestoreInteractiveSimulationOrder();
    return false;
  }
  auto executor = std::make_unique<ScenarioExecutor>(*primarySimulation_);
  if (!executor->Start(scenario, primarySimulation_->GetTickSizeSec())) {
    lastError_ = executor->GetLastError();
    RestoreInteractiveSimulationOrder();
    return false;
  }
  scenarioExecutor_ = std::move(executor);
  resolvedExecution_ = execution;
  pendingScenarioEvents_.clear();
  const telemetry::recording::ScenarioEvent startEvent{
      .simulationTimeSec = primarySimulation_->GetTime(),
      .type = "scenario_start",
      .targetRollRad = std::nullopt,
  };
  telemetryRecording_.RecordScenarioEvent(startEvent);
  pendingScenarioEvents_.push_back(startEvent);
  RecordPendingScenarioCommandEvent();
  pendingTicks_ = 0;
  executionState_ = SimExecutionState::Running;
  lastError_.clear();
  return true;
}

std::optional<ScenarioExecutionStatus>
SimRuntime::GetScenarioStatus() const {
  if (scenarioExecutor_ == nullptr
      || scenarioExecutor_->GetScenario() == nullptr) {
    return std::nullopt;
  }
  const SimScenario &scenario = *scenarioExecutor_->GetScenario();
  return ScenarioExecutionStatus{
      .name = scenario.name,
      .elapsedSec = scenarioExecutor_->GetElapsedSec(),
      .durationSec = scenario.durationSec,
  };
}

void SimRuntime::SetAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return;
  }
  automaticSimulationHz_ = ClampAutomaticSimulationHz(hz);
  maximumSimulationSpeedEnabled_ = false;
}

double SimRuntime::GetAutomaticSimulationHz() const {
  return automaticSimulationHz_;
}

void SimRuntime::SetMaximumSimulationSpeedEnabled(bool enabled) {
  maximumSimulationSpeedEnabled_ = enabled;
}

bool SimRuntime::IsMaximumSimulationSpeedEnabled() const {
  return maximumSimulationSpeedEnabled_;
}

SimStatus SimRuntime::GetStatus() const {
  return SimStatus{
      .executionState = executionState_,
      .scenario = GetScenarioStatus(),
      .automaticSimulationHz = automaticSimulationHz_,
      .maximumSimulationSpeedEnabled = maximumSimulationSpeedEnabled_,
      .pendingTickCount = pendingTicks_,
      .initialized = initialized_,
      .baselineAvailable = baselineSimulation_ != nullptr
                           && baselineSimulation_->IsInitialized(),
      .lastError = lastError_,
  };
}

SimSnapshot SimRuntime::GetSnapshot() const {
  SimSnapshot snapshot{
      .status = GetStatus(),
      .aircraftName = aircraftName_,
      .simulationHz = simulationHz_,
      .appliedExecution = resolvedExecution_,
      .telemetryRecording = GetTelemetryRecordingStatus(),
  };
  if (primarySimulation_ != nullptr && primarySimulation_->IsInitialized()) {
    snapshot.defaultInitialCondition =
        primarySimulation_->GetDefaultInitialCondition();
    snapshot.primary =
        SimSnapshotBuilder::CaptureInstance(*primarySimulation_);
    snapshot.primaryAutopilot =
        SimSnapshotBuilder::CaptureAutopilot(*primarySimulation_);
    if (const gnc::TrimResult *result =
            primarySimulation_->GetTrimService().GetResult()) {
      snapshot.trim.result = *result;
    }
    snapshot.linearization =
        SimSnapshotBuilder::CaptureLinearization(*primarySimulation_);
  }
  if (baselineSimulation_ != nullptr && baselineSimulation_->IsInitialized()) {
    snapshot.baseline =
        SimSnapshotBuilder::CaptureInstance(*baselineSimulation_);
    snapshot.baselineAutopilot =
        SimSnapshotBuilder::CaptureAutopilot(*baselineSimulation_);
  }
  return snapshot;
}

std::uint64_t SimRuntime::GetTelemetryVersion(
    SimSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().GetVersion()
             : 0;
}

telemetry::TelemetryFrame SimRuntime::GetLatestTelemetryFrame(
    SimSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().CaptureLatestFrame()
             : telemetry::TelemetryFrame{};
}

telemetry::TelemetrySnapshot SimRuntime::GetTelemetrySnapshot(
    SimSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().CaptureSnapshot()
             : telemetry::TelemetrySnapshot{};
}

double SimRuntime::GetSimulationTimeSec() const {
  return primarySimulation_ != nullptr && primarySimulation_->IsInitialized()
             ? primarySimulation_->GetTime()
             : 0.0;
}

std::optional<telemetry::recording::TelemetrySourceFrame>
SimRuntime::CaptureRecordingSource() const {
  if (primarySimulation_ == nullptr || !primarySimulation_->IsInitialized()) {
    return std::nullopt;
  }
  return telemetry::recording::TelemetryRecordingService::CaptureSource(
      primarySimulation_->GetTelemetryRegistry(),
      primarySimulation_->GetTime());
}

std::vector<telemetry::recording::ScenarioEvent>
SimRuntime::TakeScenarioEvents() {
  return std::exchange(pendingScenarioEvents_, {});
}

bool SimRuntime::SetManualControl(const control::ControlInput &input) {
  if (!initialized_ || primarySimulation_ == nullptr) {
    return false;
  }
  return primarySimulation_->GetFlightControlManager()
      .GetManualController()
      .SetCommandedInput(input);
}

bool SimRuntime::SetPrimaryRollHoldConfig(
    const PrimaryRollHoldConfig &config) {
  if (!initialized_ || scenarioExecutor_ != nullptr) {
    return false;
  }

  bool tuningChanged = false;
  if (!AutopilotConfigurationService::ApplyPrimary(*primarySimulation_,
          config,
          tuningChanged)) {
    return false;
  }
  if (tuningChanged) {
    telemetryRecording_.RecordPrimarySettings({
        .simulationTimeSec = primarySimulation_->GetTime(),
        .rollAngleProportionalGain = config.rollAngleProportionalGain,
        .rollRateProportionalGain = config.rollRateProportionalGain,
    });
  }
  return true;
}

bool SimRuntime::SetBaselineRollHoldConfig(
    const BaselineRollHoldConfig &config) {
  if (!initialized_ || scenarioExecutor_ != nullptr
      || baselineSimulation_ == nullptr) {
    return false;
  }

  bool tuningChanged = false;
  if (!AutopilotConfigurationService::ApplyBaseline(*baselineSimulation_,
          config,
          tuningChanged)) {
    return false;
  }
  if (tuningChanged) {
    telemetryRecording_.RecordBaselineSettings({
        .simulationTimeSec = baselineSimulation_->GetTime(),
        .rollTimeConstantSec = config.timeConstantSec,
        .maximumRollRateRadPerSec = config.maximumRollRateRadPerSec,
        .rateProportionalGain = config.rateProportionalGain,
        .rateIntegralGain = config.rateIntegralGain,
        .rateDerivativeGain = config.rateDerivativeGain,
        .rateFeedForwardGain = config.rateFeedForwardGain,
        .integratorLimit = config.integratorLimit,
    });
  }
  return true;
}

bool SimRuntime::RunTrim(const gnc::TrimRequest &request,
    bool fromCurrentState) {
  if (!initialized_ || primarySimulation_ == nullptr
      || scenarioExecutor_ != nullptr) {
    return false;
  }
  auto &manager = primarySimulation_->GetFlightControlManager();

  const bool resume = executionState_ == SimExecutionState::Running;
  Pause();
  Aircraft &aircraft = primarySimulation_->GetAircraft();
  gnc::TrimService &trimService = primarySimulation_->GetTrimService();
  const bool applied = gnc::TrimWorkflow::Execute(trimService,
      aircraft,
      manager,
      request,
      {.fromCurrentState = fromCurrentState});
  if (!applied) {
    lastError_ = "Trim request failed.";
  }
  if (resume) {
    Resume();
  }
  return applied;
}

bool SimRuntime::SetAutomaticLinearizationEnabled(bool enabled) {
  if (!initialized_ || primarySimulation_ == nullptr) {
    return false;
  }
  return LinearizationService{}.SetAutomaticUpdates(*primarySimulation_,
      enabled);
}

bool SimRuntime::StartTelemetryRecording() {
  if (!initialized_) {
    return false;
  }

  telemetry::recording::RecordingMetadata metadata;
  metadata.aircraft = aircraftName_;
  metadata.simulationDtSec = 1.0 / simulationHz_;
  metadata.primaryAutopilot = "MyAutopilot";
  metadata.baselineAutopilot =
      baselineSimulation_ != nullptr ? "PX4Autopilot" : "none";
  if (scenarioExecutor_ != nullptr
      && scenarioExecutor_->GetScenario() != nullptr) {
    const SimScenario &scenario = *scenarioExecutor_->GetScenario();
    metadata.scenarioName = scenario.name;
    if (resolvedExecution_) {
      metadata.scenarioFile = resolvedExecution_->source.file;
      metadata.scenarioDigest = resolvedExecution_->source.digestSha256;
      metadata.executionVariant =
          std::string(ToString(resolvedExecution_->variant));
      metadata.primaryAutopilot = metadata.executionVariant;
    }
    metadata.scenarioDurationSec = scenario.durationSec;
  } else {
    metadata.scenarioName = "interactive";
  }
  metadata.executionMode = "single";
  if (!telemetryRecording_.StartDefault(metadata, metadata.scenarioName)) {
    return false;
  }

  const AutopilotSnapshot primaryAutopilot =
      SimSnapshotBuilder::CaptureAutopilot(*primarySimulation_);
  if (primaryAutopilot.strategyName == "MyAutopilot") {
    telemetryRecording_.RecordPrimarySettings({
        .simulationTimeSec = primarySimulation_->GetTime(),
        .rollAngleProportionalGain =
            primaryAutopilot.primaryRollHold.rollAngleProportionalGain,
        .rollRateProportionalGain =
            primaryAutopilot.primaryRollHold.rollRateProportionalGain,
    });
  }
  if (baselineSimulation_ != nullptr) {
    const AutopilotSnapshot baselineAutopilot =
        SimSnapshotBuilder::CaptureAutopilot(*baselineSimulation_);
    if (baselineAutopilot.strategyName == "PX4Autopilot") {
      const BaselineRollHoldConfig &settings =
          baselineAutopilot.baselineRollHold;
      telemetryRecording_.RecordBaselineSettings({
          .simulationTimeSec = baselineSimulation_->GetTime(),
          .rollTimeConstantSec = settings.timeConstantSec,
          .maximumRollRateRadPerSec = settings.maximumRollRateRadPerSec,
          .rateProportionalGain = settings.rateProportionalGain,
          .rateIntegralGain = settings.rateIntegralGain,
          .rateDerivativeGain = settings.rateDerivativeGain,
          .rateFeedForwardGain = settings.rateFeedForwardGain,
          .integratorLimit = settings.integratorLimit,
      });
    }
  }
  return true;
}

bool SimRuntime::StartTelemetryRecording(
    const std::filesystem::path &path,
    const telemetry::recording::RecordingMetadata &metadata) {
  telemetry::recording::TelemetryRecordingConfig recordingConfig;
  recordingConfig.outputPath = path;
  recordingConfig.recordPrimary = true;
  recordingConfig.recordBaseline = baselineSimulation_ != nullptr;
  if (!telemetryRecording_.Start(recordingConfig, metadata)) {
    return false;
  }
  if (scenarioExecutor_ != nullptr
      && scenarioExecutor_->GetScenario() != nullptr) {
    const SimScenario &scenario = *scenarioExecutor_->GetScenario();
    telemetryRecording_.RecordScenarioEvent({
        .simulationTimeSec = primarySimulation_->GetTime(),
        .type = "scenario_start",
        .targetRollRad = std::nullopt,
    });
  }
  return true;
}

void SimRuntime::StopTelemetryRecording() { telemetryRecording_.Stop(); }

telemetry::recording::RecordingStatus
SimRuntime::GetTelemetryRecordingStatus() const {
  return telemetryRecording_.GetStatus();
}

void SimRuntime::FinishScenario() {
  if (scenarioExecutor_ != nullptr) {
    const telemetry::recording::ScenarioEvent endEvent{
        .simulationTimeSec = primarySimulation_->GetTime(),
        .type = "scenario_end",
    };
    telemetryRecording_.RecordScenarioEvent(endEvent);
    pendingScenarioEvents_.push_back(endEvent);
    scenarioExecutor_->Stop();
    scenarioExecutor_.reset();
  }
  RestoreInteractiveSimulationOrder();
  pendingTicks_ = 0;
  executionState_ = SimExecutionState::Stopped;
}

void SimRuntime::RecordPendingScenarioCommandEvent() {
  if (scenarioExecutor_ == nullptr || primarySimulation_ == nullptr) {
    return;
  }
  for (const auto &activation : scenarioExecutor_->TakeCommandActivations()) {
    const telemetry::recording::ScenarioEvent event{
        .simulationTimeSec = activation.simulationTimeSec,
        .type = "roll_command_changed",
        .targetRollRad = activation.targetRollRad,
    };
    telemetryRecording_.RecordScenarioEvent(event);
    pendingScenarioEvents_.push_back(event);
  }
}

bool SimRuntime::SelectExecutionVariant(ExecutionVariant variant) {
  const auto identify = [](Simulation *simulation) {
    if (simulation == nullptr) {
      return std::optional<ExecutionVariant>{};
    }
    return ExecutionVariantResolver::IdentifyVariant(
        simulation->GetFlightControlManager().GetAutopilot());
  };
  if (identify(primarySimulation_.get()) == variant) {
    return true;
  }
  if (identify(baselineSimulation_.get()) == variant) {
    std::swap(primarySimulation_, baselineSimulation_);
    scenarioSimulationSwapped_ = true;
    return true;
  }
  lastError_ = "initialized runtime does not contain execution variant '"
               + std::string(ToString(variant)) + "'";
  return false;
}

bool SimRuntime::ReinitializeForScenario(
    const SimScenario &scenario) {
  if (baselineSimulation_ != nullptr) {
    baselineSimulation_->Shutdown();
  }
  primarySimulation_->Shutdown();
  initialized_ = false;
  return Initialize(scenario.aircraft, 1.0 / scenario.dtSec);
}

void SimRuntime::RestoreInteractiveSimulationOrder() {
  if (scenarioSimulationSwapped_) {
    std::swap(primarySimulation_, baselineSimulation_);
    scenarioSimulationSwapped_ = false;
  }
}

Simulation *SimRuntime::GetSimulation(SimSlot slot) {
  return slot == SimSlot::Primary ? primarySimulation_.get()
                                         : baselineSimulation_.get();
}

const Simulation *SimRuntime::GetSimulation(SimSlot slot) const {
  return slot == SimSlot::Primary ? primarySimulation_.get()
                                         : baselineSimulation_.get();
}
} // namespace sim
