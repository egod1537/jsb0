#include "messaging/GuiSimBridge.hpp"

#include "messaging/SimMessages.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <utility>

namespace app::messaging {
GuiSimBridge::GuiSimBridge(MessageBus &bus,
    sim::SimRuntime &runtime)
    : bus_(bus), runtime_(runtime) {
  subscriptions_.push_back(
      bus_.Subscribe<SimStartCommand>([this](const auto &) {
        runtime_.Start();
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimStopCommand>([this](const auto &) {
        runtime_.Stop();
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimPauseCommand>([this](const auto &) {
        runtime_.Pause();
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimResumeCommand>([this](const auto &) {
        runtime_.Resume();
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimStepCommand>([this](const auto &) {
        runtime_.RequestTick();
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimRateCommand>([this](const auto &command) {
        runtime_.SetAutomaticSimulationHz(command.hz);
        PublishState();
      }));
  subscriptions_.push_back(bus_.Subscribe<SimMaximumSpeedCommand>(
      [this](const auto &command) {
        runtime_.SetMaximumSimulationSpeedEnabled(command.enabled);
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<SimResetCommand>([this](const auto &command) {
        const bool succeeded = command.initialCondition
                                   ? runtime_.Reset(*command.initialCondition)
                                   : runtime_.Reset();
        bus_.Publish(SimResetResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Simulation reset failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<ManualControlCommand>([this](const auto &command) {
        const bool succeeded = runtime_.SetManualControl(command.input);
        bus_.Publish(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Manual control failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<PrimaryRollHoldConfigCommand>([this](const auto &command) {
        const bool succeeded =
            runtime_.SetPrimaryRollHoldConfig(command.config);
        bus_.Publish(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError(
                                     "Primary Roll Hold configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(bus_.Subscribe<BaselineRollHoldConfigCommand>(
      [this](const auto &command) {
        const bool succeeded =
            runtime_.SetBaselineRollHoldConfig(command.config);
        bus_.Publish(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded
                         ? std::string{}
                         : GetRuntimeError(
                               "Baseline Roll Hold configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<LinearizationConfigCommand>([this](const auto &command) {
        const bool succeeded = runtime_.SetAutomaticLinearizationEnabled(
            command.automaticUpdatesEnabled);
        bus_.Publish(OperationResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded ? std::string{}
                               : GetRuntimeError(
                                     "Linearization configuration failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<TrimCommand>([this](const auto &command) {
        const bool succeeded =
            runtime_.RunTrim(command.request, command.fromCurrentState);
        const sim::SimSnapshot snapshot = runtime_.GetSnapshot();
        bus_.Publish(TrimResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .result = snapshot.trim.result,
            .error = succeeded ? std::string{}
                               : GetRuntimeError("Trim request failed."),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<ExecutionRunCommand>([this](const auto &command) {
        sim::ResolvedExecutionSpec execution;
        std::string resolutionError;
        const bool resolved = sim::ExecutionVariantResolver::Resolve(
            command.request, execution, resolutionError);
        const bool succeeded = resolved && runtime_.RunExecution(execution);
        bus_.Publish(ScenarioRunResultEvent{
            .requestId = command.requestId,
            .succeeded = succeeded,
            .error = succeeded
                         ? std::string{}
                         : (resolved ? GetRuntimeError("Scenario start failed.")
                                     : std::move(resolutionError)),
        });
        PublishState();
      }));
  subscriptions_.push_back(
      bus_.Subscribe<TelemetryRecordingCommand>([this](const auto &command) {
        bool succeeded = true;
        if (command.enabled) {
          succeeded = runtime_.StartTelemetryRecording();
        } else {
          runtime_.StopTelemetryRecording();
        }
        const telemetry::recording::RecordingStatus status =
            runtime_.GetTelemetryRecordingStatus();
        bus_.Publish(TelemetryRecordingResultEvent{
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
  bus_.Publish(SimStatusEvent{.status = snapshot.status});
  bus_.Publish(ScenarioStatusEvent{.status = snapshot.status.scenario});
  bus_.Publish(
      TelemetryRecordingStatusEvent{.status = snapshot.telemetryRecording});
  bus_.Publish(SimSnapshotEvent{.snapshot = snapshot});
  PublishTelemetry();
}

void GuiSimBridge::PublishTelemetry() {
  const std::uint64_t primaryVersion =
      runtime_.GetTelemetryVersion(sim::SimSlot::Primary);
  if (primaryVersion != primaryTelemetryVersion_) {
    bus_.Publish(TelemetryFrameEvent{
        .slot = sim::SimSlot::Primary,
        .frame = runtime_.GetLatestTelemetryFrame(sim::SimSlot::Primary),
    });
    primaryTelemetryVersion_ = primaryVersion;
  }

  const std::uint64_t baselineVersion =
      runtime_.GetTelemetryVersion(sim::SimSlot::Baseline);
  if (baselineVersion != baselineTelemetryVersion_) {
    bus_.Publish(TelemetryFrameEvent{
        .slot = sim::SimSlot::Baseline,
        .frame =
            runtime_.GetLatestTelemetryFrame(sim::SimSlot::Baseline),
    });
    baselineTelemetryVersion_ = baselineVersion;
  }
}

std::string GuiSimBridge::GetRuntimeError(
    std::string fallback) const {
  const std::string error = runtime_.GetStatus().lastError;
  return error.empty() ? std::move(fallback) : error;
}
} // namespace app::messaging
