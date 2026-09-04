#pragma once

#include "common/Options.hpp"
#include "messaging/MessageBus.hpp"
#include "messaging/SimMessages.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <type_traits>
#include <utility>
#include <variant>

namespace app::messaging {
namespace detail {
struct ReliableDeliveryPolicy {
  template <typename Message> static constexpr bool IsLatestOnly = false;
};

struct SimEventDeliveryPolicy {
  template <typename Message>
  static constexpr bool IsLatestOnly =
      std::is_same_v<Message, SimStatusEvent>
      || std::is_same_v<Message, SimSnapshotEvent>
      || std::is_same_v<Message, ScenarioStatusEvent>
      || std::is_same_v<Message, TelemetryRecordingStatusEvent>;
};

template <typename DeliveryPolicy, typename... Messages>
class ThreadSafeMessageQueue {
public:
  ThreadSafeMessageQueue() = default;

  ThreadSafeMessageQueue(const ThreadSafeMessageQueue &) = delete;
  ThreadSafeMessageQueue &operator=(const ThreadSafeMessageQueue &) = delete;

  // Producer-side delivery
  template <typename Message> bool Enqueue(Message &&message) {
    using MessageType = std::decay_t<Message>;
    static_assert((std::is_same_v<MessageType, Messages> || ...),
        "Message type is not supported by this queue.");

    {
      std::scoped_lock lock(mutex_);
      if (closed_) {
        return false;
      }
      if constexpr (DeliveryPolicy::template IsLatestOnly<MessageType>) {
        std::erase_if(messages_, [](const MessageVariant &queued) {
          return std::holds_alternative<MessageType>(queued);
        });
      }
      messages_.emplace_back(std::forward<Message>(message));
    }
    ready_.notify_one();
    return true;
  }

  void Close() {
    std::scoped_lock lock(mutex_);
    closed_ = true;
    ready_.notify_all();
  }

  bool IsClosed() const {
    std::scoped_lock lock(mutex_);
    return closed_;
  }

  std::size_t Size() const {
    std::scoped_lock lock(mutex_);
    return messages_.size();
  }

  bool Empty() const { return Size() == 0; }

  std::size_t DiscardPending() {
    std::scoped_lock lock(mutex_);
    const std::size_t discarded = messages_.size();
    messages_.clear();
    return discarded;
  }

  // Consumer-side dispatch
  template <typename Message>
  Subscription Subscribe(std::function<void(const Message &)> callback) {
    static_assert((std::is_same_v<Message, Messages> || ...),
        "Message type is not supported by this queue.");
    return consumerBus_.Subscribe<Message>(std::move(callback));
  }

  std::size_t Drain() {
    std::deque<MessageVariant> pending;
    {
      std::scoped_lock lock(mutex_);
      pending.swap(messages_);
    }

    for (const MessageVariant &message : pending) {
      std::visit([this](const auto &typed) { consumerBus_.Publish(typed); },
          message);
    }
    return pending.size();
  }

  bool DrainOne() {
    std::optional<MessageVariant> pending;
    {
      std::scoped_lock lock(mutex_);
      if (messages_.empty()) {
        return false;
      }
      pending.emplace(std::move(messages_.front()));
      messages_.pop_front();
    }
    std::visit([this](const auto &typed) { consumerBus_.Publish(typed); },
        *pending);
    return true;
  }

  void Wait(std::stop_token stopToken) {
    std::stop_callback notifyOnStop(stopToken, [this] { ready_.notify_all(); });
    std::unique_lock lock(mutex_);
    ready_.wait(lock, [this, stopToken] {
      return stopToken.stop_requested() || closed_ || !messages_.empty();
    });
  }

  template <typename Clock, typename Duration>
  void WaitUntil(std::stop_token stopToken,
      const std::chrono::time_point<Clock, Duration> &deadline) {
    std::stop_callback notifyOnStop(stopToken, [this] { ready_.notify_all(); });
    std::unique_lock lock(mutex_);
    ready_.wait_until(lock, deadline, [this, stopToken] {
      return stopToken.stop_requested() || closed_ || !messages_.empty();
    });
  }

private:
  using MessageVariant = std::variant<Messages...>;

  MessageBus consumerBus_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<MessageVariant> messages_;
  bool closed_ = false;
};
} // namespace detail

using GuiToSimQueue =
    detail::ThreadSafeMessageQueue<detail::ReliableDeliveryPolicy,
        SimStartCommand, SimStopCommand, SimPauseCommand, SimResumeCommand,
        SimStepCommand, SimRateCommand, SimMaximumSpeedCommand, SimResetCommand,
        ManualControlCommand, PrimaryRollHoldConfigCommand,
        BaselineRollHoldConfigCommand, LinearizationConfigCommand, TrimCommand,
        ExecutionRunCommand, TelemetryRecordingCommand>;

using SimToGuiQueue =
    detail::ThreadSafeMessageQueue<detail::SimEventDeliveryPolicy,
        SimStatusEvent, SimSnapshotEvent, ScenarioStatusEvent,
        TelemetryRecordingStatusEvent, SimWorkerFatalEvent,
        OperationResultEvent, SimResetResultEvent, TrimResultEvent,
        ScenarioRunResultEvent, TelemetryRecordingResultEvent>;

struct TelemetryQueueStats {
  std::uint64_t droppedBatches = 0;
  std::uint64_t droppedFrames = 0;
};

class SimToGuiTelemetryQueue final {
public:
  explicit SimToGuiTelemetryQueue(
      std::size_t capacity = opts::messaging::GuiTelemetryBatchCapacity)
      : capacity_(std::max<std::size_t>(capacity, 1)) {}

  SimToGuiTelemetryQueue(const SimToGuiTelemetryQueue &) = delete;
  SimToGuiTelemetryQueue &operator=(const SimToGuiTelemetryQueue &) = delete;

  // Simulation-thread producer
  bool Enqueue(TelemetryBatch batch) {
    {
      std::scoped_lock lock(mutex_);
      if (closed_) {
        return false;
      }
      if (batches_.size() == capacity_) {
        ++stats_.droppedBatches;
        stats_.droppedFrames += batches_.front().frames.size();
        batches_.pop_front();
      }
      batches_.push_back(std::move(batch));
    }
    return true;
  }

  // GUI-thread consumer
  Subscription Subscribe(std::function<void(const TelemetryBatch &)> callback) {
    return consumerBus_.Subscribe<TelemetryBatch>(std::move(callback));
  }

  std::size_t Drain() {
    std::deque<TelemetryBatch> pending;
    {
      std::scoped_lock lock(mutex_);
      pending.swap(batches_);
    }
    for (const TelemetryBatch &batch : pending) {
      consumerBus_.Publish(batch);
    }
    return pending.size();
  }

  void Close() {
    std::scoped_lock lock(mutex_);
    closed_ = true;
  }

  std::size_t Size() const {
    std::scoped_lock lock(mutex_);
    return batches_.size();
  }

  std::size_t Capacity() const { return capacity_; }

  TelemetryQueueStats GetStats() const {
    std::scoped_lock lock(mutex_);
    return stats_;
  }

private:
  const std::size_t capacity_;
  MessageBus consumerBus_;
  mutable std::mutex mutex_;
  std::deque<TelemetryBatch> batches_;
  TelemetryQueueStats stats_;
  bool closed_ = false;
};
} // namespace app::messaging
