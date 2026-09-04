#pragma once

#include "messaging/MessageQueues.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace sim {
class SimRuntime;
}

namespace app::messaging {
class GuiSimBridge;
}

namespace app {
class SimWorker final {
public:
  SimWorker(messaging::GuiToSimQueue &commands,
      messaging::SimToGuiQueue &events,
      messaging::SimToGuiTelemetryQueue &telemetry,
      std::unique_ptr<sim::SimRuntime> runtime);
  ~SimWorker();

  SimWorker(const SimWorker &) = delete;
  SimWorker &operator=(const SimWorker &) = delete;

  // Worker lifetime
  bool Start();
  void RequestStop();
  void Join();

  // Thread-safe worker status
  bool HasFailed() const;
  std::string GetLastError() const;

private:
  void Run(std::stop_token stopToken) noexcept;
  bool RunScheduler(std::stop_token stopToken);
  bool DrainCommandsAndSingleSteps(std::stop_token stopToken);
  bool TickOnce();
  void ShutdownRuntime() noexcept;
  void AssertSimThread() const;

  void ReportStart(bool succeeded);
  void ReportFailure(std::string error) noexcept;

  // Cross-thread transport
  messaging::GuiToSimQueue &commands_;
  messaging::SimToGuiQueue &events_;
  messaging::SimToGuiTelemetryQueue &telemetry_;

  // Simulation-thread object graph
  std::unique_ptr<sim::SimRuntime> runtime_;
  std::unique_ptr<messaging::GuiSimBridge> bridge_;

  // Worker lifetime and status
  std::jthread thread_;
  std::thread::id simThreadId_;
  std::atomic_bool failed_ = false;
  mutable std::mutex stateMutex_;
  std::condition_variable startCondition_;
  bool startRequested_ = false;
  bool startCompleted_ = false;
  bool startSucceeded_ = false;
  std::string lastError_;
};
} // namespace app
