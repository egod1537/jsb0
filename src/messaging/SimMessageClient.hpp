#pragma once

#include "messaging/MessageQueues.hpp"
#include "messaging/SimMessages.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace app {
inline constexpr double MinimumAutomaticSimulationHz =
    sim::MinimumAutomaticSimulationHz;
inline constexpr double MaximumAutomaticSimulationHz =
    sim::MaximumAutomaticSimulationHz;

using SimExecutionState = sim::SimExecutionState;
using ScenarioExecutionStatus = sim::ScenarioExecutionStatus;

inline const char *ToString(SimExecutionState state) {
  return sim::ToString(state);
}

class SimMessageClient final {
public:
  using CommandCompletion =
      std::function<void(bool succeeded, const std::string &error)>;

  SimMessageClient(messaging::GuiToSimQueue &commands,
      messaging::SimToGuiQueue &events,
      messaging::SimToGuiTelemetryQueue &telemetry);

  // Execution and scenarios
  // Request methods return queue acceptance; completion runs on event drain.
  bool RunExecution(const sim::ExecutionRequest &request,
      CommandCompletion completion = {});
  std::optional<ScenarioExecutionStatus> GetScenarioExecutionStatus() const;
  SimExecutionState GetSimExecutionState() const;
  void StartSimulation();
  void StopSimulation();
  void PauseSimulation();
  void ResumeSimulation();
  void RequestSimTick();
  bool ResetSimulation(CommandCompletion completion = {});
  bool ResetSimulation(const sim::InitialCondition &initialCondition,
      CommandCompletion completion = {});

  // Scheduling
  double GetAutomaticSimulationHz() const;
  void SetAutomaticSimulationHz(double hz);
  bool IsMaximumSimulationSpeedEnabled() const;
  void SetMaximumSimulationSpeedEnabled(bool enabled);
  std::uint32_t GetPendingSimTickCount() const;

  // Cached state and command publication
  sim::SimSnapshot GetSimSnapshot() const;
  std::shared_ptr<const telemetry::TelemetrySnapshot> GetTelemetrySnapshot(
      sim::SimSlot slot) const;
  bool SetManualControl(const control::ControlInput &input);
  bool SetPrimaryRollHoldConfig(const sim::PrimaryRollHoldConfig &config);
  bool SetBaselineRollHoldConfig(const sim::BaselineRollHoldConfig &config);
  bool RunTrim(const gnc::TrimRequest &request, bool fromCurrentState,
      CommandCompletion completion = {});
  bool SetAutomaticLinearizationEnabled(bool enabled);
  std::optional<std::string> GetLastCommandError() const;

  // Telemetry recording
  bool StartTelemetryRecording();
  void StopTelemetryRecording();
  telemetry::recording::RecordingStatus GetTelemetryRecordingStatus() const;
  bool OpenTelemetryRecordingsFolder() const;

private:
  // Request/result correlation
  messaging::RequestId NextRequestId(CommandCompletion completion = {});
  template <typename Command> bool EnqueueRequest(Command command) {
    AssertGuiThread();
    const messaging::RequestId requestId = command.requestId;
    if (commands_.Enqueue(std::move(command))) {
      return true;
    }
    CompleteRequest(requestId, false, "Simulation command queue is closed.");
    return false;
  }
  void CompleteRequest(messaging::RequestId requestId, bool succeeded,
      const std::string &error);

  // Event cache updates
  void ReceiveTelemetryBatch(const messaging::TelemetryBatch &batch);
  void ReceiveTelemetryFrame(const messaging::TelemetryFrameEvent &event);

  // Thread ownership
  void AssertGuiThread() const;

  // Dependencies
  messaging::GuiToSimQueue &commands_;

  // Cached event data
  sim::SimSnapshot latestSnapshot_;
  sim::SimStatus latestStatus_;
  telemetry::TelemetrySnapshot primaryTelemetryCache_;
  telemetry::TelemetrySnapshot baselineTelemetryCache_;
  mutable std::shared_ptr<const telemetry::TelemetrySnapshot> primaryTelemetry_;
  mutable std::shared_ptr<const telemetry::TelemetrySnapshot>
      baselineTelemetry_;
  telemetry::recording::RecordingStatus recordingStatus_;
  std::unordered_map<messaging::RequestId, CommandCompletion> pendingRequests_;
  std::optional<std::string> lastCommandError_;

  // GUI-thread ownership
  std::thread::id guiThreadId_ = std::this_thread::get_id();

  // Declared last so subscriptions are destroyed before callback state.
  std::vector<messaging::Subscription> subscriptions_;
};
} // namespace app
