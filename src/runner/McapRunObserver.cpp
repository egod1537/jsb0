#include "McapRunObserver.hpp"

#include "sim/gnc/autopilot/AutopilotFactory.hpp"
#include "sim/runtime/SimulationRuntime.hpp"
#include "contract/telemetry/RecordingTypes.hpp"

#include <string>

namespace runner {
namespace {
std::string RecordingError(const telemetry::recording::RecordingStatus &status,
    std::string_view context) {
  std::string error(context);
  if (!status.errorMessage.empty()) {
    error += ": ";
    error += status.errorMessage;
  }
  return error;
}
} // namespace

bool McapRunObserver::OnRunStarted(const SimulationRunInfo &info,
    sim::SimulationRuntime &runtime, std::string &error) {
  telemetry::recording::RecordingMetadata metadata;
  metadata.contractVersion = JSB_CONTRACT_VERSION;
  metadata.telemetrySchemaVersion = JSB_TELEMETRY_SCHEMA_VERSION;
  metadata.applicationVersion = JSB_APPLICATION_VERSION;
  metadata.gitCommit = JSB_GIT_COMMIT;
  metadata.runtimeBranch = JSB_RUNTIME_BRANCH;
  metadata.aircraft = info.aircraft;
  metadata.scenarioName = info.scenarioName;
  metadata.scenarioFile = info.scenarioFile;
  metadata.scenarioDigest = info.scenarioDigest;
  metadata.scenarioSchemaVersion = info.scenarioSchemaVersion;
  metadata.scenarioType = info.scenarioType;
  metadata.scenarioDurationSec = info.durationSec;
  metadata.simulationDtSec = info.dtSec;
  metadata.primaryAutopilot = gnc::ToString(info.autopilot);

  if (!runtime.StartTelemetryRecording(info.outputDirectory / "telemetry.mcap",
          metadata)) {
    error = RecordingError(runtime.GetTelemetryRecordingStatus(),
        "failed to initialize MCAP recorder for telemetry.mcap");
    return false;
  }
  started_ = true;
  return true;
}

bool McapRunObserver::OnSimulationStep(const SimulationRunInfo &,
    sim::SimulationRuntime &runtime, std::string &error) {
  if (!started_) {
    error = "MCAP recorder was not started";
    return false;
  }
  const telemetry::recording::RecordingStatus status =
      runtime.GetTelemetryRecordingStatus();
  if (status.state != telemetry::recording::RecordingState::Recording) {
    error = RecordingError(status, "failed to record telemetry.mcap");
    return false;
  }
  return true;
}

bool McapRunObserver::OnRunFinished(const SimulationRunInfo &,
    sim::SimulationRuntime &runtime, const RunnerResult &, std::string &error) {
  if (!started_) {
    return true;
  }
  runtime.StopTelemetryRecording();
  started_ = false;
  const telemetry::recording::RecordingStatus status =
      runtime.GetTelemetryRecordingStatus();
  if (status.state != telemetry::recording::RecordingState::Idle) {
    error = RecordingError(status, "failed to finalize telemetry.mcap");
    return false;
  }
  return true;
}
} // namespace runner
