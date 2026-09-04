#include "messaging/GuiSimBridge.hpp"

#include "messaging/SimMessages.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <utility>

namespace app::messaging {
GuiSimBridge::GuiSimBridge(GuiToSimQueue &commands, SimToGuiQueue &events,
    SimToGuiTelemetryQueue &telemetry, sim::SimRuntime &runtime)
    : events_(events), telemetry_(telemetry), runtime_(runtime) {
  subscriptions_.push_back(
      commands.Subscribe<SimStartCommand>([this](const auto &) {
        runtime_.Start();
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimStopCommand>([this](const auto &) {
        runtime_.Stop();
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimPauseCommand>([this](const auto &) {
        runtime_.Pause();
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimResumeCommand>([this](const auto &) {
        runtime_.Resume();
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimStepCommand>([this](const auto &) {
        runtime_.RequestTick();
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimRateCommand>([this](const auto &command) {
        runtime_.SetAutomaticSimulationHz(command.hz);
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimMaximumSpeedCommand>([this](const auto &command) {
        runtime_.SetMaximumSimulationSpeedEnabled(command.enabled);
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<SimResetCommand>([this](const auto &command) {
        const bool succeeded = command.initialCondition
                                   ? runtime_.Reset(*command.initialCondition)
                                   : runtime_.Reset();
        events_.Enqueue(SimResetResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Simulation reset failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<ManualControlCommand>([this](const auto &command) {
        const bool succeeded = runtime_.SetManualControl(command.input);
        events_.Enqueue(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Manual control failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(commands.Subscribe<PrimaryRollHoldConfigCommand>(
      [this](const auto &command) {
        const bool succeeded =
            runtime_.SetPrimaryRollHoldConfig(command.config);
        events_.Enqueue(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError(
                                     "Primary Roll Hold configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(commands.Subscribe<BaselineRollHoldConfigCommand>(
      [this](const auto &command) {
        const bool succeeded =
            runtime_.SetBaselineRollHoldConfig(command.config);
        events_.Enqueue(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded
                         ? std::string{}
                         : GetRuntimeError(
                               "Baseline Roll Hold configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(commands.Subscribe<LinearizationConfigCommand>(
      [this](const auto &command) {
        const bool succeeded = runtime_.SetAutomaticLinearizationEnabled(
            command.automaticUpdatesEnabled);
        events_.Enqueue(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError(
                                     "Linearization configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<TrimCommand>([this](const auto &command) {
        const bool succeeded =
            runtime_.RunTrim(command.request, command.fromCurrentState);
        const sim::SimSnapshot snapshot = runtime_.GetSnapshot();
        events_.Enqueue(TrimResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .result = snapshot.trim.result,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Trim request failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      commands.Subscribe<ExecutionRunCommand>([this](const auto &command) {
        sim::ResolvedExecutionSpec execution;
        std::string resolutionError;
        const bool resolved =
            sim::ExecutionVariantResolver::Resolve(command.request,
                execution,
                resolutionError);
        const bool succeeded = resolved && runtime_.RunExecution(execution);
        events_.Enqueue(ScenarioRunResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded
                         ? std::string{}
                         : (resolved ? GetRuntimeError("Scenario start failed.")
                                     : std::move(resolutionError)),
        });
        PublishState();
      }));
  subscriptions_.push_back(commands.Subscribe<TelemetryRecordingCommand>(
      [this](const auto &command) {
        bool succeeded = true;
        if (command.enabled) {
          succeeded = runtime_.StartTelemetryRecording();
        } else {
          runtime_.StopTelemetryRecording();
        }
        const telemetry::recording::RecordingStatus status =
            runtime_.GetTelemetryRecordingStatus();
        events_.Enqueue(TelemetryRecordingResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .status = status,
            .error = succeeded ? std::string{} : status.errorMessage,
        });
        PublishState();
      }));
}

void GuiSimBridge::PublishState() {
  const sim::SimSnapshot snapshot = runtime_.GetSnapshot();
  events_.Enqueue(SimStatusEvent{.status = snapshot.status});
  events_.Enqueue(ScenarioStatusEvent{.status = snapshot.status.scenario});
  events_.Enqueue(
      TelemetryRecordingStatusEvent{.status = snapshot.telemetryRecording});
  events_.Enqueue(SimSnapshotEvent{.snapshot = snapshot});
  PublishTelemetry();
}

void GuiSimBridge::PublishTelemetry() {
  TelemetryBatch batch;
  std::uint64_t nextPrimaryVersion = primaryTelemetryVersion_;
  std::uint64_t nextBaselineVersion = baselineTelemetryVersion_;

  const std::uint64_t primaryVersion =
      runtime_.GetTelemetryVersion(sim::SimSlot::Primary);
  if (primaryVersion != primaryTelemetryVersion_) {
    batch.frames.push_back(TelemetryFrameEvent{
        .slot = sim::SimSlot::Primary,
        .frame = runtime_.GetLatestTelemetryFrame(sim::SimSlot::Primary),
    });
    nextPrimaryVersion = primaryVersion;
  }

  const std::uint64_t baselineVersion =
      runtime_.GetTelemetryVersion(sim::SimSlot::Baseline);
  if (baselineVersion != baselineTelemetryVersion_) {
    batch.frames.push_back(TelemetryFrameEvent{
        .slot = sim::SimSlot::Baseline,
        .frame = runtime_.GetLatestTelemetryFrame(sim::SimSlot::Baseline),
    });
    nextBaselineVersion = baselineVersion;
  }

  if (!batch.frames.empty() && telemetry_.Enqueue(std::move(batch))) {
    primaryTelemetryVersion_ = nextPrimaryVersion;
    baselineTelemetryVersion_ = nextBaselineVersion;
  }
}

std::string GuiSimBridge::GetRuntimeError(std::string fallback) const {
  const std::string error = runtime_.GetStatus().lastError;
  return error.empty() ? std::move(fallback) : error;
}
} // namespace app::messaging
