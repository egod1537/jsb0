#include "messaging/SimMessageClient.hpp"

#include "sim/telemetry/recording/TelemetryRecordingService.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

namespace app {
namespace {
constexpr std::size_t MaximumCachedSamplesPerChannel = 4096;
constexpr std::size_t CompactedSamplesPerChannel = 2048;
std::atomic<messaging::RequestId> nextRequestId = 1;

void AppendChronologically(std::vector<telemetry::TelemetrySample> &output,
    const telemetry::TelemetrySample &first,
    const telemetry::TelemetrySample &second) {
  if (first.timeSec <= second.timeSec) {
    output.push_back(first);
    if (second.timeSec != first.timeSec) {
      output.push_back(second);
    }
  } else {
    output.push_back(second);
    output.push_back(first);
  }
}

void CompactTelemetryHistory(telemetry::TelemetrySeries &series) {
  const std::vector<telemetry::TelemetrySample> &samples = series.samples;
  if (samples.size() <= MaximumCachedSamplesPerChannel) {
    return;
  }

  std::vector<telemetry::TelemetrySample> compacted;
  compacted.reserve(CompactedSamplesPerChannel);
  // Keep the full session span while retaining each time bucket's extrema.
  // This avoids per-frame front erases and preserves spikes for overview plots.
  compacted.push_back(samples.front());

  const std::size_t interiorCount = samples.size() - 2;
  const std::size_t bucketCount = (CompactedSamplesPerChannel - 2) / 2;
  for (std::size_t bucket = 0; bucket < bucketCount; ++bucket) {
    const std::size_t begin = 1 + interiorCount * bucket / bucketCount;
    const std::size_t end = 1 + interiorCount * (bucket + 1) / bucketCount;
    if (begin >= end) {
      continue;
    }

    auto minimum = samples.begin() + static_cast<std::ptrdiff_t>(begin);
    auto maximum = minimum;
    for (auto sample = minimum + 1;
        sample != samples.begin() + static_cast<std::ptrdiff_t>(end);
        ++sample) {
      if (sample->value < minimum->value) {
        minimum = sample;
      }
      if (sample->value > maximum->value) {
        maximum = sample;
      }
    }
    AppendChronologically(compacted, *minimum, *maximum);
  }

  if (compacted.back().timeSec != samples.back().timeSec) {
    compacted.push_back(samples.back());
  }
  series.samples = std::move(compacted);
}
} // namespace

SimMessageClient::SimMessageClient(messaging::GuiToSimQueue &commands,
    messaging::SimToGuiQueue &events,
    messaging::SimToGuiTelemetryQueue &telemetry)
    : commands_(commands),
      primaryTelemetry_(std::make_shared<telemetry::TelemetrySnapshot>()),
      baselineTelemetry_(std::make_shared<telemetry::TelemetrySnapshot>()) {
  subscriptions_.push_back(
      events.Subscribe<messaging::SimStatusEvent>([this](const auto &event) {
        AssertGuiThread();
        latestStatus_ = event.status;
        latestSnapshot_.status = event.status;
      }));
  subscriptions_.push_back(
      events.Subscribe<messaging::SimSnapshotEvent>([this](const auto &event) {
        AssertGuiThread();
        latestSnapshot_ = event.snapshot;
        latestStatus_ = event.snapshot.status;
        recordingStatus_ = event.snapshot.telemetryRecording;
      }));
  subscriptions_.push_back(telemetry.Subscribe(
      [this](const auto &batch) { ReceiveTelemetryBatch(batch); }));
  subscriptions_.push_back(
      events.Subscribe<messaging::TelemetryRecordingStatusEvent>(
          [this](const auto &event) {
            AssertGuiThread();
            recordingStatus_ = event.status;
          }));
  subscriptions_.push_back(events.Subscribe<messaging::SimWorkerFatalEvent>(
      [this](const auto &event) {
        AssertGuiThread();
        lastCommandError_ = event.error;
      }));

  const auto receiveOperationResult = [this](messaging::RequestId requestId,
                                          bool succeeded,
                                          const std::string &error) {
    CompleteRequest(requestId, succeeded, error);
  };
  subscriptions_.push_back(events.Subscribe<messaging::OperationResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(events.Subscribe<messaging::SimResetResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(events.Subscribe<messaging::TrimResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(events.Subscribe<messaging::ScenarioRunResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(
      events.Subscribe<messaging::TelemetryRecordingResultEvent>(
          [this, receiveOperationResult](const auto &event) {
            AssertGuiThread();
            recordingStatus_ = event.status;
            receiveOperationResult(event.requestId,
                event.succeeded,
                event.error);
          }));
}

bool SimMessageClient::RunExecution(const sim::ExecutionRequest &request,
    CommandCompletion completion) {
  const messaging::RequestId requestId = NextRequestId(std::move(completion));
  return EnqueueRequest(messaging::ExecutionRunCommand{
      .requestId = requestId,
      .request = request,
  });
}

std::optional<ScenarioExecutionStatus>
SimMessageClient::GetScenarioExecutionStatus() const {
  AssertGuiThread();
  return latestStatus_.scenario;
}

SimExecutionState SimMessageClient::GetSimExecutionState() const {
  AssertGuiThread();
  return latestStatus_.executionState;
}

void SimMessageClient::StartSimulation() {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimStartCommand{});
}

void SimMessageClient::StopSimulation() {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimStopCommand{});
}

void SimMessageClient::PauseSimulation() {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimPauseCommand{});
}

void SimMessageClient::ResumeSimulation() {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimResumeCommand{});
}

void SimMessageClient::RequestSimTick() {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimStepCommand{});
}

bool SimMessageClient::ResetSimulation(CommandCompletion completion) {
  const messaging::RequestId requestId = NextRequestId(std::move(completion));
  return EnqueueRequest(messaging::SimResetCommand{.requestId = requestId});
}

bool SimMessageClient::ResetSimulation(
    const sim::InitialCondition &initialCondition,
    CommandCompletion completion) {
  const messaging::RequestId requestId = NextRequestId(std::move(completion));
  return EnqueueRequest(messaging::SimResetCommand{
      .requestId = requestId,
      .initialCondition = initialCondition,
  });
}

double SimMessageClient::GetAutomaticSimulationHz() const {
  AssertGuiThread();
  return latestStatus_.automaticSimulationHz;
}

void SimMessageClient::SetAutomaticSimulationHz(double hz) {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimRateCommand{.hz = hz});
}

bool SimMessageClient::IsMaximumSimulationSpeedEnabled() const {
  AssertGuiThread();
  return latestStatus_.maximumSimulationSpeedEnabled;
}

void SimMessageClient::SetMaximumSimulationSpeedEnabled(bool enabled) {
  AssertGuiThread();
  commands_.Enqueue(messaging::SimMaximumSpeedCommand{.enabled = enabled});
}

std::uint32_t SimMessageClient::GetPendingSimTickCount() const {
  AssertGuiThread();
  return latestStatus_.pendingTickCount;
}

sim::SimSnapshot SimMessageClient::GetSimSnapshot() const {
  AssertGuiThread();
  return latestSnapshot_;
}

std::shared_ptr<const telemetry::TelemetrySnapshot>
SimMessageClient::GetTelemetrySnapshot(sim::SimSlot slot) const {
  AssertGuiThread();
  const telemetry::TelemetrySnapshot &cache = slot == sim::SimSlot::Primary
                                                  ? primaryTelemetryCache_
                                                  : baselineTelemetryCache_;
  auto &published =
      slot == sim::SimSlot::Primary ? primaryTelemetry_ : baselineTelemetry_;
  if (published == nullptr || published->version != cache.version
      || published->available != cache.available) {
    published = std::make_shared<const telemetry::TelemetrySnapshot>(cache);
  }
  return published;
}

bool SimMessageClient::SetManualControl(const control::ControlInput &input) {
  const messaging::RequestId requestId = NextRequestId();
  return EnqueueRequest(messaging::ManualControlCommand{
      .requestId = requestId,
      .input = input,
  });
}

bool SimMessageClient::SetPrimaryRollHoldConfig(
    const sim::PrimaryRollHoldConfig &config) {
  const messaging::RequestId requestId = NextRequestId();
  return EnqueueRequest(messaging::PrimaryRollHoldConfigCommand{
      .requestId = requestId,
      .config = config,
  });
}

bool SimMessageClient::SetBaselineRollHoldConfig(
    const sim::BaselineRollHoldConfig &config) {
  const messaging::RequestId requestId = NextRequestId();
  return EnqueueRequest(messaging::BaselineRollHoldConfigCommand{
      .requestId = requestId,
      .config = config,
  });
}

bool SimMessageClient::RunTrim(const gnc::TrimRequest &request,
    bool fromCurrentState, CommandCompletion completion) {
  const messaging::RequestId requestId = NextRequestId(std::move(completion));
  return EnqueueRequest(messaging::TrimCommand{
      .requestId = requestId,
      .request = request,
      .fromCurrentState = fromCurrentState,
  });
}

bool SimMessageClient::SetAutomaticLinearizationEnabled(bool enabled) {
  const messaging::RequestId requestId = NextRequestId();
  return EnqueueRequest(messaging::LinearizationConfigCommand{
      .requestId = requestId,
      .automaticUpdatesEnabled = enabled,
  });
}

std::optional<std::string> SimMessageClient::GetLastCommandError() const {
  AssertGuiThread();
  return lastCommandError_;
}

bool SimMessageClient::StartTelemetryRecording() {
  const messaging::RequestId requestId = NextRequestId();
  return EnqueueRequest(messaging::TelemetryRecordingCommand{
      .requestId = requestId,
      .enabled = true,
  });
}

void SimMessageClient::StopTelemetryRecording() {
  const messaging::RequestId requestId = NextRequestId();
  EnqueueRequest(messaging::TelemetryRecordingCommand{
      .requestId = requestId,
      .enabled = false,
  });
}

telemetry::recording::RecordingStatus
SimMessageClient::GetTelemetryRecordingStatus() const {
  AssertGuiThread();
  return recordingStatus_;
}

bool SimMessageClient::OpenTelemetryRecordingsFolder() const {
  AssertGuiThread();
  const std::filesystem::path directory = telemetry::recording::
      TelemetryRecordingService::GetDefaultRecordingsDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    return false;
  }
#ifdef _WIN32
  return reinterpret_cast<std::intptr_t>(ShellExecuteW(nullptr,
             L"open",
             directory.c_str(),
             nullptr,
             nullptr,
             SW_SHOWNORMAL))
         > 32;
#else
  return false;
#endif
}

messaging::RequestId SimMessageClient::NextRequestId(
    CommandCompletion completion) {
  AssertGuiThread();
  const messaging::RequestId requestId =
      nextRequestId.fetch_add(1, std::memory_order_relaxed);
  pendingRequests_.emplace(requestId, std::move(completion));
  return requestId;
}

void SimMessageClient::CompleteRequest(messaging::RequestId requestId,
    bool succeeded, const std::string &error) {
  AssertGuiThread();
  CommandCompletion completion;
  std::string completionError;
  const auto pending = pendingRequests_.find(requestId);
  if (pending == pendingRequests_.end()) {
    return;
  }
  completion = std::move(pending->second);
  pendingRequests_.erase(pending);
  if (succeeded) {
    lastCommandError_.reset();
  } else {
    completionError = error.empty()
                          ? "Simulation command failed without error details."
                          : error;
    lastCommandError_ = completionError;
  }
  if (completion) {
    completion(succeeded, completionError);
  }
}

void SimMessageClient::ReceiveTelemetryBatch(
    const messaging::TelemetryBatch &batch) {
  AssertGuiThread();
  for (const messaging::TelemetryFrameEvent &frame : batch.frames) {
    ReceiveTelemetryFrame(frame);
  }
}

void SimMessageClient::ReceiveTelemetryFrame(
    const messaging::TelemetryFrameEvent &event) {
  telemetry::TelemetrySnapshot &updated = event.slot == sim::SimSlot::Primary
                                              ? primaryTelemetryCache_
                                              : baselineTelemetryCache_;
  if (updated.publishedTimeRange
      && event.frame.timestamp < updated.publishedTimeRange->maxSec) {
    updated.series.clear();
    updated.publishedTimeRange.reset();
  }
  updated.available = event.frame.available;
  updated.version = event.frame.sequence;
  if (!updated.publishedTimeRange) {
    updated.publishedTimeRange = telemetry::TelemetryTimeRange{
        .minSec = event.frame.timestamp,
        .maxSec = event.frame.timestamp,
    };
  } else {
    updated.publishedTimeRange->minSec =
        std::min(updated.publishedTimeRange->minSec, event.frame.timestamp);
    updated.publishedTimeRange->maxSec =
        std::max(updated.publishedTimeRange->maxSec, event.frame.timestamp);
  }

  for (const telemetry::TelemetryValue &value : event.frame.values) {
    auto position = std::lower_bound(updated.series.begin(),
        updated.series.end(),
        value.path,
        [](const telemetry::TelemetrySeries &series, const std::string &path) {
          return series.path < path;
        });
    if (position == updated.series.end() || position->path != value.path) {
      position = updated.series.insert(position,
          telemetry::TelemetrySeries{.path = value.path});
    }
    if (!position->samples.empty()
        && position->samples.back().timeSec == event.frame.timestamp) {
      position->samples.back().value = value.value;
    } else {
      position->samples.push_back(
          {.timeSec = event.frame.timestamp, .value = value.value});
      CompactTelemetryHistory(*position);
    }
  }
}

void SimMessageClient::AssertGuiThread() const {
  assert(std::this_thread::get_id() == guiThreadId_
         && "SimMessageClient must only be used on its GUI owner thread.");
}
} // namespace app
