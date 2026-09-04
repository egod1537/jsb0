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

SimMessageClient::SimMessageClient(messaging::MessageBus &bus)
    : bus_(bus),
      primaryTelemetry_(std::make_shared<telemetry::TelemetrySnapshot>()),
      baselineTelemetry_(std::make_shared<telemetry::TelemetrySnapshot>()) {
  subscriptions_.push_back(bus_.Subscribe<messaging::SimStatusEvent>(
      [this](const auto &event) {
        std::scoped_lock lock(cacheMutex_);
        latestStatus_ = event.status;
        latestSnapshot_.status = event.status;
      }));
  subscriptions_.push_back(bus_.Subscribe<messaging::SimSnapshotEvent>(
      [this](const auto &event) {
        std::scoped_lock lock(cacheMutex_);
        latestSnapshot_ = event.snapshot;
        latestStatus_ = event.snapshot.status;
        recordingStatus_ = event.snapshot.telemetryRecording;
      }));
  subscriptions_.push_back(bus_.Subscribe<messaging::TelemetryFrameEvent>(
      [this](const auto &event) { ReceiveTelemetry(event); }));
  subscriptions_.push_back(
      bus_.Subscribe<messaging::TelemetryRecordingStatusEvent>(
          [this](const auto &event) {
            std::scoped_lock lock(cacheMutex_);
            recordingStatus_ = event.status;
          }));

  const auto receiveOperationResult = [this](messaging::RequestId requestId,
                                          bool succeeded,
                                          const std::string &error) {
    std::scoped_lock lock(cacheMutex_);
    if (pendingRequests_.contains(requestId)) {
      requestResults_[requestId] = {
          .succeeded = succeeded,
          .error = error,
      };
    }
  };
  subscriptions_.push_back(bus_.Subscribe<messaging::OperationResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(
      bus_.Subscribe<messaging::SimResetResultEvent>(
          [receiveOperationResult](const auto &event) {
            receiveOperationResult(event.requestId,
                event.succeeded,
                event.error);
          }));
  subscriptions_.push_back(bus_.Subscribe<messaging::TrimResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(bus_.Subscribe<messaging::ScenarioRunResultEvent>(
      [receiveOperationResult](const auto &event) {
        receiveOperationResult(event.requestId, event.succeeded, event.error);
      }));
  subscriptions_.push_back(
      bus_.Subscribe<messaging::TelemetryRecordingResultEvent>(
          [this, receiveOperationResult](const auto &event) {
            {
              std::scoped_lock lock(cacheMutex_);
              recordingStatus_ = event.status;
            }
            receiveOperationResult(event.requestId,
                event.succeeded,
                event.error);
          }));
}

bool SimMessageClient::RunExecution(
    const sim::ExecutionRequest &request) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::ExecutionRunCommand{
      .requestId = requestId,
      .request = request,
  });
  return TakeRequestResult(requestId);
}

std::optional<ScenarioExecutionStatus>
SimMessageClient::GetScenarioExecutionStatus() const {
  std::scoped_lock lock(cacheMutex_);
  return latestStatus_.scenario;
}

SimExecutionState
SimMessageClient::GetSimExecutionState() const {
  std::scoped_lock lock(cacheMutex_);
  return latestStatus_.executionState;
}

void SimMessageClient::StartSimulation() {
  bus_.Publish(messaging::SimStartCommand{});
}

void SimMessageClient::StopSimulation() {
  bus_.Publish(messaging::SimStopCommand{});
}

void SimMessageClient::PauseSimulation() {
  bus_.Publish(messaging::SimPauseCommand{});
}

void SimMessageClient::ResumeSimulation() {
  bus_.Publish(messaging::SimResumeCommand{});
}

void SimMessageClient::RequestSimTick() {
  bus_.Publish(messaging::SimStepCommand{});
}

bool SimMessageClient::ResetSimulation() {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::SimResetCommand{.requestId = requestId});
  return TakeRequestResult(requestId);
}

bool SimMessageClient::ResetSimulation(
    const sim::InitialCondition &initialCondition) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::SimResetCommand{
      .requestId = requestId,
      .initialCondition = initialCondition,
  });
  return TakeRequestResult(requestId);
}

double SimMessageClient::GetAutomaticSimulationHz() const {
  std::scoped_lock lock(cacheMutex_);
  return latestStatus_.automaticSimulationHz;
}

void SimMessageClient::SetAutomaticSimulationHz(double hz) {
  bus_.Publish(messaging::SimRateCommand{.hz = hz});
}

bool SimMessageClient::IsMaximumSimulationSpeedEnabled() const {
  std::scoped_lock lock(cacheMutex_);
  return latestStatus_.maximumSimulationSpeedEnabled;
}

void SimMessageClient::SetMaximumSimulationSpeedEnabled(bool enabled) {
  bus_.Publish(messaging::SimMaximumSpeedCommand{.enabled = enabled});
}

std::uint32_t SimMessageClient::GetPendingSimTickCount() const {
  std::scoped_lock lock(cacheMutex_);
  return latestStatus_.pendingTickCount;
}

sim::SimSnapshot SimMessageClient::GetSimSnapshot() const {
  std::scoped_lock lock(cacheMutex_);
  return latestSnapshot_;
}

std::shared_ptr<const telemetry::TelemetrySnapshot>
SimMessageClient::GetTelemetrySnapshot(sim::SimSlot slot) const {
  std::scoped_lock lock(cacheMutex_);
  const telemetry::TelemetrySnapshot &cache =
      slot == sim::SimSlot::Primary ? primaryTelemetryCache_
                                           : baselineTelemetryCache_;
  auto &published = slot == sim::SimSlot::Primary ? primaryTelemetry_
                                                         : baselineTelemetry_;
  if (published == nullptr || published->version != cache.version
      || published->available != cache.available) {
    published = std::make_shared<const telemetry::TelemetrySnapshot>(cache);
  }
  return published;
}

bool SimMessageClient::SetManualControl(
    const control::ControlInput &input) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::ManualControlCommand{
      .requestId = requestId,
      .input = input,
  });
  return TakeRequestResult(requestId);
}

bool SimMessageClient::SetPrimaryRollHoldConfig(
    const sim::PrimaryRollHoldConfig &config) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::PrimaryRollHoldConfigCommand{
      .requestId = requestId,
      .config = config,
  });
  return TakeRequestResult(requestId);
}

bool SimMessageClient::SetBaselineRollHoldConfig(
    const sim::BaselineRollHoldConfig &config) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::BaselineRollHoldConfigCommand{
      .requestId = requestId,
      .config = config,
  });
  return TakeRequestResult(requestId);
}

bool SimMessageClient::RunTrim(const gnc::TrimRequest &request,
    bool fromCurrentState) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::TrimCommand{
      .requestId = requestId,
      .request = request,
      .fromCurrentState = fromCurrentState,
  });
  return TakeRequestResult(requestId);
}

bool SimMessageClient::SetAutomaticLinearizationEnabled(bool enabled) {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::LinearizationConfigCommand{
      .requestId = requestId,
      .automaticUpdatesEnabled = enabled,
  });
  return TakeRequestResult(requestId);
}

std::optional<std::string>
SimMessageClient::GetLastCommandError() const {
  std::scoped_lock lock(cacheMutex_);
  return lastCommandError_;
}

bool SimMessageClient::StartTelemetryRecording() {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::TelemetryRecordingCommand{
      .requestId = requestId,
      .enabled = true,
  });
  return TakeRequestResult(requestId);
}

void SimMessageClient::StopTelemetryRecording() {
  const messaging::RequestId requestId = NextRequestId();
  bus_.Publish(messaging::TelemetryRecordingCommand{
      .requestId = requestId,
      .enabled = false,
  });
  TakeRequestResult(requestId);
}

telemetry::recording::RecordingStatus
SimMessageClient::GetTelemetryRecordingStatus() const {
  std::scoped_lock lock(cacheMutex_);
  return recordingStatus_;
}

bool SimMessageClient::OpenTelemetryRecordingsFolder() const {
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

messaging::RequestId SimMessageClient::NextRequestId() {
  const messaging::RequestId requestId =
      nextRequestId.fetch_add(1, std::memory_order_relaxed);
  std::scoped_lock lock(cacheMutex_);
  pendingRequests_.insert(requestId);
  return requestId;
}

bool SimMessageClient::TakeRequestResult(
    messaging::RequestId requestId) {
  std::scoped_lock lock(cacheMutex_);
  const std::size_t removedPendingRequest = pendingRequests_.erase(requestId);
  assert(removedPendingRequest == 1
         && "TakeRequestResult requires a pending request ID.");
  if (removedPendingRequest != 1) {
    lastCommandError_ = "Message request correlation state was invalid.";
    return false;
  }

  // Publish is synchronous, so the adapter's matching result must have been
  // delivered before the command Publish call returned. Never wait here.
  const auto result = requestResults_.find(requestId);
  if (result == requestResults_.end()) {
    lastCommandError_ =
        "Synchronous message request completed without a matching result.";
    return false;
  }
  const RequestResult requestResult = std::move(result->second);
  requestResults_.erase(result);
  if (requestResult.succeeded) {
    lastCommandError_.reset();
  } else {
    lastCommandError_ = requestResult.error.empty()
                            ? "Simulation command failed without error details."
                            : requestResult.error;
  }
  return requestResult.succeeded;
}

void SimMessageClient::ReceiveTelemetry(
    const messaging::TelemetryFrameEvent &event) {
  std::scoped_lock lock(cacheMutex_);
  telemetry::TelemetrySnapshot &updated =
      event.slot == sim::SimSlot::Primary ? primaryTelemetryCache_
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
} // namespace app
