#include "messaging/MessageQueues.hpp"

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace messaging = app::messaging;

void TestCommandFifoAcrossTypedMessages() {
  messaging::GuiToSimQueue queue;
  std::vector<std::string> received;
  auto start = queue.Subscribe<messaging::SimStartCommand>(
      [&received](const auto &) { received.emplace_back("start"); });
  auto rate = queue.Subscribe<messaging::SimRateCommand>(
      [&received](const auto &command) {
        received.push_back("rate:" + std::to_string(command.hz));
      });
  auto stop = queue.Subscribe<messaging::SimStopCommand>(
      [&received](const auto &) { received.emplace_back("stop"); });

  assert(queue.Enqueue(messaging::SimStartCommand{}));
  assert(queue.Enqueue(messaging::SimRateCommand{.hz = 120.0}));
  assert(queue.Enqueue(messaging::SimStopCommand{}));
  assert(received.empty());
  assert(queue.Drain() == 3);
  assert((received
          == std::vector<std::string>{"start", "rate:120.000000", "stop"}));
}

void TestEmptyDrain() {
  messaging::GuiToSimQueue commands;
  messaging::SimToGuiQueue events;
  messaging::SimToGuiTelemetryQueue telemetry;
  assert(commands.Drain() == 0);
  assert(events.Drain() == 0);
  assert(telemetry.Drain() == 0);
}

void TestShutdownPreservesPendingMessages() {
  messaging::GuiToSimQueue queue;
  int received = 0;
  auto subscription = queue.Subscribe<messaging::SimStepCommand>(
      [&received](const auto &) { ++received; });

  assert(queue.Enqueue(messaging::SimStepCommand{}));
  queue.Close();
  assert(queue.IsClosed());
  assert(!queue.Enqueue(messaging::SimStepCommand{}));
  assert(queue.Drain() == 1);
  assert(received == 1);
  assert(queue.Empty());
}

void TestShutdownCanDiscardPendingCommandsWithoutDispatch() {
  messaging::GuiToSimQueue queue;
  int callbackCount = 0;
  auto subscription = queue.Subscribe<messaging::SimStepCommand>(
      [&callbackCount](const auto &) { ++callbackCount; });

  assert(queue.Enqueue(messaging::SimStepCommand{}));
  assert(queue.Enqueue(messaging::SimStepCommand{}));
  queue.Close();
  assert(queue.DiscardPending() == 2);
  assert(queue.Empty());
  assert(queue.Drain() == 0);
  assert(callbackCount == 0);
  assert(!queue.Enqueue(messaging::SimStepCommand{}));
}

void TestProducerNeverRunsConsumerCallback() {
  messaging::GuiToSimQueue queue;
  const std::thread::id consumerThread = std::this_thread::get_id();
  std::thread::id callbackThread;
  int callbackCount = 0;
  auto subscription =
      queue.Subscribe<messaging::SimStartCommand>([&](const auto &) {
        callbackThread = std::this_thread::get_id();
        ++callbackCount;
      });

  std::thread producer([&] {
    assert(std::this_thread::get_id() != consumerThread);
    assert(queue.Enqueue(messaging::SimStartCommand{}));
  });
  producer.join();

  assert(callbackCount == 0);
  queue.Drain();
  assert(callbackCount == 1);
  assert(callbackThread == consumerThread);
}

void TestQueueLockIsReleasedBeforeConsumerCallback() {
  messaging::GuiToSimQueue queue;
  int stopCount = 0;
  auto start =
      queue.Subscribe<messaging::SimStartCommand>([&queue](const auto &) {
        assert(queue.Size() == 0);
        assert(queue.Enqueue(messaging::SimStopCommand{}));
      });
  auto stop = queue.Subscribe<messaging::SimStopCommand>(
      [&stopCount](const auto &) { ++stopCount; });

  assert(queue.Enqueue(messaging::SimStartCommand{}));
  assert(queue.Drain() == 1);
  assert(queue.Drain() == 1);
  assert(stopCount == 1);
}

void TestConcurrentBurstHasNoLossAndPreservesFifo() {
  messaging::GuiToSimQueue queue;
  constexpr std::size_t MessageCount = 20'000;
  std::vector<std::size_t> received;
  received.reserve(MessageCount);
  auto subscription = queue.Subscribe<messaging::SimRateCommand>(
      [&received](const auto &command) {
        received.push_back(static_cast<std::size_t>(command.hz));
      });
  std::atomic_bool producerDone = false;

  std::thread producer([&] {
    for (std::size_t index = 0; index < MessageCount; ++index) {
      assert(queue.Enqueue(
          messaging::SimRateCommand{.hz = static_cast<double>(index)}));
    }
    producerDone.store(true, std::memory_order_release);
  });

  while (!producerDone.load(std::memory_order_acquire) || !queue.Empty()) {
    queue.Drain();
    std::this_thread::yield();
  }
  producer.join();
  queue.Drain();

  assert(received.size() == MessageCount);
  for (std::size_t index = 0; index < MessageCount; ++index) {
    assert(received[index] == index);
  }
}

void TestStateCoalescingKeepsReliableEvents() {
  messaging::SimToGuiQueue queue;
  std::vector<messaging::RequestId> results;
  int statusCount = 0;
  sim::SimExecutionState latestState = sim::SimExecutionState::Stopped;
  auto status =
      queue.Subscribe<messaging::SimStatusEvent>([&](const auto &event) {
        ++statusCount;
        latestState = event.status.executionState;
      });
  auto result = queue.Subscribe<messaging::OperationResultEvent>(
      [&results](const auto &event) { results.push_back(event.requestId); });

  sim::SimStatus stopped;
  stopped.executionState = sim::SimExecutionState::Stopped;
  sim::SimStatus running;
  running.executionState = sim::SimExecutionState::Running;
  assert(queue.Enqueue(messaging::SimStatusEvent{.status = stopped}));
  assert(queue.Enqueue(messaging::OperationResultEvent{.requestId = 1}));
  assert(queue.Enqueue(messaging::SimStatusEvent{.status = running}));
  assert(queue.Enqueue(messaging::OperationResultEvent{.requestId = 2}));

  assert(queue.Drain() == 3);
  assert(statusCount == 1);
  assert(latestState == sim::SimExecutionState::Running);
  assert((results == std::vector<messaging::RequestId>{1, 2}));
}

} // namespace

int main() {
  TestCommandFifoAcrossTypedMessages();
  TestEmptyDrain();
  TestShutdownPreservesPendingMessages();
  TestShutdownCanDiscardPendingCommandsWithoutDispatch();
  TestProducerNeverRunsConsumerCallback();
  TestQueueLockIsReleasedBeforeConsumerCallback();
  TestConcurrentBurstHasNoLossAndPreservesFifo();
  TestStateCoalescingKeepsReliableEvents();
  return 0;
}
