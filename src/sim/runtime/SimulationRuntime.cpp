#include "sim/runtime/SimulationRuntime.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Simulation.hpp"
#include "sim/analysis/LinearizationService.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/gnc/TrimWorkflow.hpp"
#include "sim/runtime/AutopilotConfigurationService.hpp"
#include "sim/runtime/SimulationInstanceSet.hpp"
#include "sim/runtime/SimulationSnapshotBuilder.hpp"
#include "sim/scenario/ScenarioExecutor.hpp"
#include "sim/scenario/SimulationScenario.hpp"

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

SimulationRuntime::SimulationRuntime(
    std::unique_ptr<Simulation> primarySimulation,
    std::unique_ptr<Simulation> baselineSimulation)
    : primarySimulation_(std::move(primarySimulation)),
      baselineSimulation_(std::move(baselineSimulation)) {}

SimulationRuntime::~SimulationRuntime() { Shutdown(); }

std::unique_ptr<SimulationRuntime> SimulationRuntime::CreateForExecution(
    const ResolvedExecutionSpec &execution, std::string &error) {
  const SimulationScenario &scenario = execution.scenario;
  ScenarioValidationError validationError;
  if (!ValidateSimulationScenario(scenario, &validationError)) {
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
  auto runtime = std::make_unique<SimulationRuntime>(
      std::make_unique<Simulation>(std::move(autopilot)));
  SimulationConfig config;
  config.aircraftName = scenario.aircraft;
  config.simulationHz = 1.0 / scenario.dtSec;
  if (!runtime->Initialize(config)) {
    error = runtime->GetStatus().lastError;
    return nullptr;
  }
  error.clear();
  return runtime;
}

bool SimulationRuntime::Initialize(const SimulationConfig &config) {
  if (initialized_) {
    return true;
  }
  if (primarySimulation_ == nullptr) {
    lastError_ = "Simulation runtime requires a primary simulation.";
    return false;
  }

  config_ = config;
  automaticSimulationHz_ = ClampAutomaticSimulationHz(config.simulationHz);
  if (!SimulationInstanceSet(*primarySimulation_, baselineSimulation_.get())
          .Initialize(config_, lastError_)) {
    return false;
  }

  executionState_ = SimulationExecutionState::Stopped;
  pendingTicks_ = 0;
  initialized_ = true;
  lastError_.clear();
  return true;
}

void SimulationRuntime::Shutdown() {
  if (!initialized_ && primarySimulation_ == nullptr) {
    return;
  }

  if (scenarioExecutor_ != nullptr) {
    FinishScenario();
  }
  telemetryRecording_.Stop();
  executionState_ = SimulationExecutionState::Stopped;
  pendingTicks_ = 0;

  if (primarySimulation_ != nullptr) {
    SimulationInstanceSet(*primarySimulation_, baselineSimulation_.get())
        .Shutdown();
  }
  initialized_ = false;
}

void SimulationRuntime::Start() {
  if (initialized_ && executionState_ == SimulationExecutionState::Stopped) {
    pendingTicks_ = 0;
    executionState_ = SimulationExecutionState::Running;
  }
}

void SimulationRuntime::Stop() {
  if (scenarioExecutor_ != nullptr) {
    FinishScenario();
    return;
  }
  pendingTicks_ = 0;
  executionState_ = SimulationExecutionState::Stopped;
}

void SimulationRuntime::Pause() {
  if (executionState_ == SimulationExecutionState::Running) {
    executionState_ = SimulationExecutionState::Paused;
  }
}

void SimulationRuntime::Resume() {
  if (executionState_ == SimulationExecutionState::Paused) {
    pendingTicks_ = 0;
    executionState_ = SimulationExecutionState::Running;
  }
}

bool SimulationRuntime::Reset() {
  return scenarioExecutor_ == nullptr && initialized_
         && primarySimulation_ != nullptr
         && SimulationInstanceSet(*primarySimulation_,
             baselineSimulation_.get())
                .Reset(nullptr, lastError_);
}

bool SimulationRuntime::Reset(const InitialCondition &initialCondition) {
  return scenarioExecutor_ == nullptr && initialized_
         && primarySimulation_ != nullptr
         && SimulationInstanceSet(*primarySimulation_,
             baselineSimulation_.get())
                .Reset(&initialCondition, lastError_);
}

void SimulationRuntime::RequestTick() {
  if (executionState_ == SimulationExecutionState::Paused) {
    ++pendingTicks_;
  }
}

bool SimulationRuntime::Tick() {
  const bool isPaused = executionState_ == SimulationExecutionState::Paused;
  if (isPaused && pendingTicks_ == 0) {
    return true;
  }
  if (executionState_ != SimulationExecutionState::Running && !isPaused) {
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
    if (!SimulationInstanceSet(*primarySimulation_, baselineSimulation_.get())
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

bool SimulationRuntime::RunExecution(const ResolvedExecutionSpec &execution) {
  const SimulationScenario &scenario = execution.scenario;
  if (!initialized_ || executionState_ != SimulationExecutionState::Stopped
      || primarySimulation_ == nullptr || !primarySimulation_->IsInitialized()
      || (baselineSimulation_ != nullptr
          && !baselineSimulation_->IsInitialized())) {
    return false;
  }

  std::string validationError;
  if (!ValidateSimulationScenario(scenario, &validationError)) {
    lastError_ = validationError;
    return false;
  }
  if (scenario.aircraft != config_.aircraftName
      || std::abs(scenario.dtSec - config_.GetDT()) > 1.0e-12) {
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
  executionState_ = SimulationExecutionState::Running;
  lastError_.clear();
  return true;
}

std::optional<ScenarioExecutionStatus>
SimulationRuntime::GetScenarioStatus() const {
  if (scenarioExecutor_ == nullptr
      || scenarioExecutor_->GetScenario() == nullptr) {
    return std::nullopt;
  }
  const SimulationScenario &scenario = *scenarioExecutor_->GetScenario();
  return ScenarioExecutionStatus{
      .name = scenario.name,
      .elapsedSec = scenarioExecutor_->GetElapsedSec(),
      .durationSec = scenario.durationSec,
  };
}

void SimulationRuntime::SetAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return;
  }
  automaticSimulationHz_ = ClampAutomaticSimulationHz(hz);
  maximumSimulationSpeedEnabled_ = false;
}

double SimulationRuntime::GetAutomaticSimulationHz() const {
  return automaticSimulationHz_;
}

void SimulationRuntime::SetMaximumSimulationSpeedEnabled(bool enabled) {
  maximumSimulationSpeedEnabled_ = enabled;
}

bool SimulationRuntime::IsMaximumSimulationSpeedEnabled() const {
  return maximumSimulationSpeedEnabled_;
}

SimulationStatus SimulationRuntime::GetStatus() const {
  return SimulationStatus{
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

SimulationSnapshot SimulationRuntime::GetSnapshot() const {
  SimulationSnapshot snapshot{
      .status = GetStatus(),
      .config = config_,
      .appliedExecution = resolvedExecution_,
      .telemetryRecording = GetTelemetryRecordingStatus(),
  };
  if (primarySimulation_ != nullptr && primarySimulation_->IsInitialized()) {
    snapshot.defaultInitialCondition =
        primarySimulation_->GetDefaultInitialCondition();
    snapshot.primary =
        SimulationSnapshotBuilder::CaptureInstance(*primarySimulation_);
    snapshot.primaryAutopilot =
        SimulationSnapshotBuilder::CaptureAutopilot(*primarySimulation_);
    if (const gnc::TrimResult *result =
            primarySimulation_->GetTrimService().GetResult()) {
      snapshot.trim.result = *result;
    }
    snapshot.linearization =
        SimulationSnapshotBuilder::CaptureLinearization(*primarySimulation_);
  }
  if (baselineSimulation_ != nullptr && baselineSimulation_->IsInitialized()) {
    snapshot.baseline =
        SimulationSnapshotBuilder::CaptureInstance(*baselineSimulation_);
    snapshot.baselineAutopilot =
        SimulationSnapshotBuilder::CaptureAutopilot(*baselineSimulation_);
  }
  return snapshot;
}

std::uint64_t SimulationRuntime::GetTelemetryVersion(
    SimulationSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().GetVersion()
             : 0;
}

telemetry::TelemetryFrame SimulationRuntime::GetLatestTelemetryFrame(
    SimulationSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().CaptureLatestFrame()
             : telemetry::TelemetryFrame{};
}

telemetry::TelemetrySnapshot SimulationRuntime::GetTelemetrySnapshot(
    SimulationSlot slot) const {
  const Simulation *simulation = GetSimulation(slot);
  return simulation != nullptr && simulation->IsInitialized()
             ? simulation->GetTelemetryRegistry().CaptureSnapshot()
             : telemetry::TelemetrySnapshot{};
}

double SimulationRuntime::GetSimulationTimeSec() const {
  return primarySimulation_ != nullptr && primarySimulation_->IsInitialized()
             ? primarySimulation_->GetTime()
             : 0.0;
}

std::optional<telemetry::recording::TelemetrySourceFrame>
SimulationRuntime::CaptureRecordingSource() const {
  if (primarySimulation_ == nullptr || !primarySimulation_->IsInitialized()) {
    return std::nullopt;
  }
  return telemetry::recording::TelemetryRecordingService::CaptureSource(
      primarySimulation_->GetTelemetryRegistry(),
      primarySimulation_->GetTime());
}

std::vector<telemetry::recording::ScenarioEvent>
SimulationRuntime::TakeScenarioEvents() {
  return std::exchange(pendingScenarioEvents_, {});
}

bool SimulationRuntime::SetManualControl(const control::ControlInput &input) {
  if (!initialized_ || primarySimulation_ == nullptr) {
    return false;
  }
  auto *manager =
      primarySimulation_->GetComponent<control::FlightControlManager>();
  return manager != nullptr
         && manager->GetManualController().SetCommandedInput(input);
}

bool SimulationRuntime::SetPrimaryRollHoldConfig(
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

bool SimulationRuntime::SetBaselineRollHoldConfig(
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

bool SimulationRuntime::RunTrim(const gnc::TrimRequest &request,
    bool fromCurrentState) {
  if (!initialized_ || primarySimulation_ == nullptr
      || scenarioExecutor_ != nullptr) {
    return false;
  }
  auto *manager =
      primarySimulation_->GetComponent<control::FlightControlManager>();
  if (manager == nullptr) {
    return false;
  }

  const bool resume = executionState_ == SimulationExecutionState::Running;
  Pause();
  Aircraft &aircraft = primarySimulation_->GetAircraft();
  gnc::TrimService &trimService = primarySimulation_->GetTrimService();
  const bool applied = gnc::TrimWorkflow::Execute(trimService,
      aircraft,
      *manager,
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

bool SimulationRuntime::SetAutomaticLinearizationEnabled(bool enabled) {
  if (!initialized_ || primarySimulation_ == nullptr) {
    return false;
  }
  return LinearizationService{}.SetAutomaticUpdates(*primarySimulation_,
      enabled);
}

bool SimulationRuntime::StartTelemetryRecording() {
  if (!initialized_) {
    return false;
  }

  telemetry::recording::RecordingMetadata metadata;
  metadata.aircraft = config_.aircraftName;
  metadata.simulationDtSec = config_.GetDT();
  metadata.primaryAutopilot = "MyAutopilot";
  metadata.baselineAutopilot =
      baselineSimulation_ != nullptr ? "PX4Autopilot" : "none";
  if (scenarioExecutor_ != nullptr
      && scenarioExecutor_->GetScenario() != nullptr) {
    const SimulationScenario &scenario = *scenarioExecutor_->GetScenario();
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
      SimulationSnapshotBuilder::CaptureAutopilot(*primarySimulation_);
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
        SimulationSnapshotBuilder::CaptureAutopilot(*baselineSimulation_);
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

bool SimulationRuntime::StartTelemetryRecording(
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
    const SimulationScenario &scenario = *scenarioExecutor_->GetScenario();
    telemetryRecording_.RecordScenarioEvent({
        .simulationTimeSec = primarySimulation_->GetTime(),
        .type = "scenario_start",
        .targetRollRad = std::nullopt,
    });
  }
  return true;
}

void SimulationRuntime::StopTelemetryRecording() { telemetryRecording_.Stop(); }

telemetry::recording::RecordingStatus
SimulationRuntime::GetTelemetryRecordingStatus() const {
  return telemetryRecording_.GetStatus();
}

void SimulationRuntime::FinishScenario() {
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
  executionState_ = SimulationExecutionState::Stopped;
}

void SimulationRuntime::RecordPendingScenarioCommandEvent() {
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

bool SimulationRuntime::SelectExecutionVariant(ExecutionVariant variant) {
  const auto identify = [](Simulation *simulation) {
    if (simulation == nullptr) {
      return std::optional<ExecutionVariant>{};
    }
    auto *manager = simulation->GetComponent<control::FlightControlManager>();
    return manager == nullptr ? std::optional<ExecutionVariant>{}
                              : ExecutionVariantResolver::IdentifyVariant(
                                    manager->GetAutopilot());
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

bool SimulationRuntime::ReinitializeForScenario(
    const SimulationScenario &scenario) {
  if (baselineSimulation_ != nullptr) {
    baselineSimulation_->Shutdown();
  }
  primarySimulation_->Shutdown();
  initialized_ = false;
  SimulationConfig scenarioConfig = config_;
  scenarioConfig.aircraftName = scenario.aircraft;
  scenarioConfig.simulationHz = 1.0 / scenario.dtSec;
  return Initialize(scenarioConfig);
}

void SimulationRuntime::RestoreInteractiveSimulationOrder() {
  if (scenarioSimulationSwapped_) {
    std::swap(primarySimulation_, baselineSimulation_);
    scenarioSimulationSwapped_ = false;
  }
}

Simulation *SimulationRuntime::GetSimulation(SimulationSlot slot) {
  return slot == SimulationSlot::Primary ? primarySimulation_.get()
                                         : baselineSimulation_.get();
}

const Simulation *SimulationRuntime::GetSimulation(SimulationSlot slot) const {
  return slot == SimulationSlot::Primary ? primarySimulation_.get()
                                         : baselineSimulation_.get();
}
} // namespace sim
