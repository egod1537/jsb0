#pragma once

#include "common/Options.hpp"
#include "sim/runtime/SimContracts.hpp"
#include "sim/execution/ExecutionRequest.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"
#include "contract/telemetry/RecordingTypes.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace app::messaging {
using RequestId = std::uint64_t;

// Execution commands
struct SimStartCommand {};
struct SimStopCommand {};
struct SimPauseCommand {};
struct SimResumeCommand {};
struct SimStepCommand {};

struct SimRateCommand {
  double hz = opts::simulation::Hz;
};

struct SimMaximumSpeedCommand {
  bool enabled = false;
};

// Request/response commands
struct SimResetCommand {
  RequestId requestId = 0;
  std::optional<sim::InitialCondition> initialCondition;
};

struct ManualControlCommand {
  RequestId requestId = 0;
  control::ControlInput input;
};

struct PrimaryRollHoldConfigCommand {
  RequestId requestId = 0;
  sim::PrimaryRollHoldConfig config;
};

struct BaselineRollHoldConfigCommand {
  RequestId requestId = 0;
  sim::BaselineRollHoldConfig config;
};

struct LinearizationConfigCommand {
  RequestId requestId = 0;
  bool automaticUpdatesEnabled = false;
};

struct TrimCommand {
  RequestId requestId = 0;
  gnc::TrimRequest request;
  bool fromCurrentState = false;
};

struct ExecutionRunCommand {
  RequestId requestId = 0;
  sim::ExecutionRequest request;
};

struct TelemetryRecordingCommand {
  RequestId requestId = 0;
  bool enabled = false;
};

// State events
struct SimStatusEvent {
  sim::SimStatus status;
};

struct SimSnapshotEvent {
  sim::SimSnapshot snapshot;
};

struct TelemetryFrameEvent {
  sim::SimSlot slot = sim::SimSlot::Primary;
  telemetry::TelemetryFrame frame;
};

struct ScenarioStatusEvent {
  std::optional<sim::ScenarioExecutionStatus> status;
};

struct TelemetryRecordingStatusEvent {
  telemetry::recording::RecordingStatus status;
};

// Operation result events
struct OperationResultEvent {
  RequestId requestId = 0;
  bool succeeded = false;
  std::string error;
};

struct SimResetResultEvent {
  RequestId requestId = 0;
  bool succeeded = false;
  std::string error;
};

struct TrimResultEvent {
  RequestId requestId = 0;
  bool succeeded = false;
  std::optional<gnc::TrimResult> result;
  std::string error;
};

struct ScenarioRunResultEvent {
  RequestId requestId = 0;
  bool succeeded = false;
  std::string error;
};

struct TelemetryRecordingResultEvent {
  RequestId requestId = 0;
  bool succeeded = false;
  telemetry::recording::RecordingStatus status;
  std::string error;
};
} // namespace app::messaging
