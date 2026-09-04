#include "contract/telemetry/mcap/McapRecordingReader.hpp"
#include "messaging/GuiSimBridge.hpp"
#include "messaging/MessageQueues.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/AutopilotFactory.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

namespace {
namespace messaging = app::messaging;

messaging::TelemetryBatch MakeTelemetryBatch(std::uint64_t sequence) {
  return {.frames = {
              {
                  .slot = sim::SimSlot::Primary,
                  .frame =
                      {
                          .available = true,
                          .sequence = sequence,
                          .timestamp = static_cast<double>(sequence),
                      },
              },
              {
                  .slot = sim::SimSlot::Baseline,
                  .frame =
                      {
                          .available = true,
                          .sequence = sequence,
                          .timestamp = static_cast<double>(sequence),
                      },
              },
          }};
}

void TestSnapshotHandoffOwnsLatestCoherentValue() {
  messaging::SimToGuiQueue events;
  std::vector<sim::SimSnapshot> received;
  auto subscription = events.Subscribe<messaging::SimSnapshotEvent>(
      [&received](const auto &event) { received.push_back(event.snapshot); });

  sim::SimSnapshot first;
  first.status.executionState = sim::SimExecutionState::Running;
  first.primary.aircraft.simulationTimeSec = 1.0;
  assert(events.Enqueue(messaging::SimSnapshotEvent{.snapshot = first}));
  first.status.executionState = sim::SimExecutionState::Stopped;
  first.primary.aircraft.simulationTimeSec = -1.0;

  sim::SimSnapshot latest;
  latest.status.executionState = sim::SimExecutionState::Paused;
  latest.primary.aircraft.simulationTimeSec = 2.0;
  assert(events.Enqueue(messaging::SimSnapshotEvent{.snapshot = latest}));
  latest.status.executionState = sim::SimExecutionState::Stopped;
  latest.primary.aircraft.simulationTimeSec = -2.0;

  assert(events.Drain() == 1);
  assert(received.size() == 1);
  assert(
      received.front().status.executionState == sim::SimExecutionState::Paused);
  assert(received.front().primary.aircraft.simulationTimeSec == 2.0);
}

void TestSnapshotCoalescingDoesNotDropResultEvents() {
  messaging::SimToGuiQueue events;
  std::vector<messaging::RequestId> receivedResults;
  std::size_t snapshotCount = 0;
  auto result = events.Subscribe<messaging::OperationResultEvent>(
      [&receivedResults](
          const auto &event) { receivedResults.push_back(event.requestId); });
  auto snapshot = events.Subscribe<messaging::SimSnapshotEvent>(
      [&snapshotCount](const auto &) { ++snapshotCount; });

  constexpr std::size_t ResultCount = 1'000;
  for (std::size_t index = 0; index < ResultCount; ++index) {
    assert(events.Enqueue(messaging::OperationResultEvent{
        .requestId = static_cast<messaging::RequestId>(index + 1),
        .succeeded = true,
    }));
    assert(events.Enqueue(messaging::SimSnapshotEvent{}));
  }

  events.Drain();
  assert(snapshotCount == 1);
  assert(receivedResults.size() == ResultCount);
  for (std::size_t index = 0; index < ResultCount; ++index) {
    assert(receivedResults[index] == index + 1);
  }
}

void TestBoundedTelemetryPreservesNewestBatchesAndSourceOrder() {
  constexpr std::size_t Capacity = 8;
  constexpr std::uint64_t BatchCount = 10'000;
  messaging::SimToGuiTelemetryQueue telemetry(Capacity);
  const std::thread::id guiThread = std::this_thread::get_id();
  std::thread::id callbackThread;
  std::vector<std::uint64_t> receivedSequences;
  auto subscription = telemetry.Subscribe([&](const auto &batch) {
    callbackThread = std::this_thread::get_id();
    assert(batch.frames.size() == 2);
    assert(batch.frames[0].slot == sim::SimSlot::Primary);
    assert(batch.frames[1].slot == sim::SimSlot::Baseline);
    assert(batch.frames[0].frame.sequence == batch.frames[1].frame.sequence);
    receivedSequences.push_back(batch.frames[0].frame.sequence);
  });

  std::thread producer([&] {
    for (std::uint64_t sequence = 1; sequence <= BatchCount; ++sequence) {
      assert(telemetry.Enqueue(MakeTelemetryBatch(sequence)));
    }
  });
  producer.join();

  assert(receivedSequences.empty());
  assert(telemetry.Size() == Capacity);
  const messaging::TelemetryQueueStats stats = telemetry.GetStats();
  assert(stats.droppedBatches == BatchCount - Capacity);
  assert(stats.droppedFrames == (BatchCount - Capacity) * 2);

  assert(telemetry.Drain() == Capacity);
  assert(callbackThread == guiThread);
  assert(receivedSequences.size() == Capacity);
  assert(receivedSequences.front() == BatchCount - Capacity + 1);
  assert(receivedSequences.back() == BatchCount);
  for (std::size_t index = 1; index < receivedSequences.size(); ++index) {
    assert(receivedSequences[index - 1] < receivedSequences[index]);
  }
}

void TestClientCacheUsesBoundedTelemetryOnGuiThread() {
  messaging::GuiToSimQueue commands;
  messaging::SimToGuiQueue events;
  messaging::SimToGuiTelemetryQueue telemetry(3);
  app::SimMessageClient client(commands, events, telemetry);

  static_assert(std::is_same_v<decltype(client.GetTelemetrySnapshot(
                                   sim::SimSlot::Primary)),
      std::shared_ptr<const telemetry::TelemetrySnapshot>>);

  for (std::uint64_t sequence = 1; sequence <= 10; ++sequence) {
    assert(telemetry.Enqueue(MakeTelemetryBatch(sequence)));
  }
  telemetry.Drain();

  const auto primary = client.GetTelemetrySnapshot(sim::SimSlot::Primary);
  const auto baseline = client.GetTelemetrySnapshot(sim::SimSlot::Baseline);
  assert(primary != nullptr && primary->available && primary->version == 10);
  assert(baseline != nullptr && baseline->available && baseline->version == 10);
  assert(primary != baseline);
  assert(primary->publishedTimeRange.has_value());
  assert(baseline->publishedTimeRange.has_value());
  assert(primary->publishedTimeRange->minSec == 8.0);
  assert(primary->publishedTimeRange->maxSec == 10.0);
  assert(baseline->publishedTimeRange->minSec == 8.0);
  assert(baseline->publishedTimeRange->maxSec == 10.0);
}

void TestConcurrentTelemetryProducerAndGuiDrain() {
  constexpr std::uint64_t BatchCount = 20'000;
  messaging::SimToGuiTelemetryQueue telemetry(32);
  std::atomic_bool producerDone = false;
  std::uint64_t lastSequence = 0;
  auto subscription = telemetry.Subscribe([&lastSequence](const auto &batch) {
    assert(!batch.frames.empty());
    const std::uint64_t sequence = batch.frames.front().frame.sequence;
    assert(sequence > lastSequence);
    lastSequence = sequence;
  });

  std::thread producer([&] {
    for (std::uint64_t sequence = 1; sequence <= BatchCount; ++sequence) {
      assert(telemetry.Enqueue(MakeTelemetryBatch(sequence)));
    }
    producerDone.store(true, std::memory_order_release);
  });

  while (
      !producerDone.load(std::memory_order_acquire) || telemetry.Size() != 0) {
    telemetry.Drain();
    std::this_thread::yield();
  }
  producer.join();
  telemetry.Drain();
  assert(lastSequence == BatchCount);
}

void TestSlowGuiDoesNotDropRecorderTelemetry() {
  const std::filesystem::path recordingPath =
      std::filesystem::temp_directory_path()
      / ("jsb-handoff-"
          + std::to_string(
              std::chrono::steady_clock::now().time_since_epoch().count())
          + ".mcap");

  messaging::GuiToSimQueue commands;
  messaging::SimToGuiQueue events;
  messaging::SimToGuiTelemetryQueue telemetry(2);
  sim::SimRuntime runtime(std::make_unique<sim::Simulation>(
      gnc::CreateAutopilot(gnc::AutopilotKind::Primary)));
  messaging::GuiSimBridge bridge(commands, events, telemetry, runtime);

  assert(runtime.Initialize());
  telemetry::recording::RecordingMetadata metadata;
  metadata.simulationDtSec = 1.0 / runtime.GetSnapshot().simulationHz;
  assert(runtime.StartTelemetryRecording(recordingPath, metadata));
  runtime.Start();
  constexpr std::size_t TickCount = 100;
  for (std::size_t tick = 0; tick < TickCount; ++tick) {
    assert(runtime.Tick());
    bridge.PublishState();
  }
  runtime.StopTelemetryRecording();

  assert(telemetry.Size() == telemetry.Capacity());
  assert(telemetry.GetStats().droppedBatches > 0);

  {
    telemetry::recording::McapRecordingReader reader;
    assert(reader.Open(recordingPath));
    assert(
        reader.ReadMessages("/jsb/primary/control/roll").size() == TickCount);
  }

  std::error_code error;
  std::filesystem::remove(recordingPath, error);
  assert(!error);
}
} // namespace

int main() {
  TestSnapshotHandoffOwnsLatestCoherentValue();
  TestSnapshotCoalescingDoesNotDropResultEvents();
  TestBoundedTelemetryPreservesNewestBatchesAndSourceOrder();
  TestClientCacheUsesBoundedTelemetryOnGuiThread();
  TestConcurrentTelemetryProducerAndGuiDrain();
  TestSlowGuiDoesNotDropRecorderTelemetry();
  return 0;
}
