#include "app/SimWorker.hpp"

#include "messaging/GuiSimBridge.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

namespace app {
namespace {
using Clock = std::chrono::steady_clock;

Clock::duration ToTickInterval(double hz) {
  const double clamped = std::clamp(hz,
      sim::MinimumAutomaticSimulationHz,
      sim::MaximumAutomaticSimulationHz);
  return std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / clamped));
}
} // namespace

SimWorker::SimWorker(messaging::GuiToSimQueue &commands,
    messaging::SimToGuiQueue &events,
    messaging::SimToGuiTelemetryQueue &telemetry,
    std::unique_ptr<sim::SimRuntime> runtime)
    : commands_(commands), events_(events), telemetry_(telemetry),
      runtime_(std::move(runtime)) {}

SimWorker::~SimWorker() {
  RequestStop();
  Join();
}

bool SimWorker::Start() {
  std::unique_lock lock(stateMutex_);
  if (!startRequested_) {
    startRequested_ = true;
    thread_ =
        std::jthread([this](std::stop_token stopToken) { Run(stopToken); });
  }
  startCondition_.wait(lock, [this] { return startCompleted_; });
  return startSucceeded_;
}

void SimWorker::RequestStop() {
  commands_.Close();
  commands_.DiscardPending();
  if (thread_.joinable()) {
    thread_.request_stop();
  }
}

void SimWorker::Join() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool SimWorker::HasFailed() const {
  return failed_.load(std::memory_order_acquire);
}

std::string SimWorker::GetLastError() const {
  std::scoped_lock lock(stateMutex_);
  return lastError_;
}

void SimWorker::Run(std::stop_token stopToken) noexcept {
  simThreadId_ = std::this_thread::get_id();
  AssertSimThread();
  bool startReported = false;
  try {
    if (runtime_ == nullptr) {
      ReportFailure("Simulation worker requires a runtime.");
      ReportStart(false);
      ShutdownRuntime();
      return;
    }

    bridge_ = std::make_unique<messaging::GuiSimBridge>(commands_,
        events_,
        telemetry_,
        *runtime_);
    const bool initialized = runtime_->Initialize();
    bridge_->PublishState();
    if (!initialized) {
      ReportFailure(runtime_->GetStatus().lastError);
      ReportStart(false);
      startReported = true;
    } else {
      ReportStart(true);
      startReported = true;
      RunScheduler(stopToken);
    }
  } catch (const std::exception &error) {
    ReportFailure(error.what());
    if (!startReported) {
      ReportStart(false);
      startReported = true;
    }
  } catch (...) {
    ReportFailure("Simulation worker terminated with an unknown error.");
    if (!startReported) {
      ReportStart(false);
      startReported = true;
    }
  }

  ShutdownRuntime();
}

void SimWorker::ShutdownRuntime() noexcept {
  try {
    commands_.Close();
    commands_.DiscardPending();
    if (runtime_ == nullptr) {
      return;
    }
    runtime_->Shutdown();
    if (bridge_ != nullptr) {
      bridge_->PublishState();
    }
  } catch (const std::exception &error) {
    ReportFailure(error.what());
  } catch (...) {
    ReportFailure("Simulation runtime shutdown failed with an unknown error.");
  }
}

bool SimWorker::RunScheduler(std::stop_token stopToken) {
  AssertSimThread();
  sim::SimStatus previousStatus = runtime_->GetStatus();
  double scheduledHz = previousStatus.automaticSimulationHz;
  bool scheduledMaximumSpeed = previousStatus.maximumSimulationSpeedEnabled;
  Clock::duration interval = ToTickInterval(scheduledHz);
  Clock::time_point nextTick = Clock::now();

  while (!stopToken.stop_requested()) {
    if (!DrainCommandsAndSingleSteps(stopToken)) {
      return false;
    }
    if (stopToken.stop_requested()) {
      break;
    }

    const sim::SimStatus status = runtime_->GetStatus();
    const Clock::time_point now = Clock::now();
    if (status.automaticSimulationHz != scheduledHz) {
      scheduledHz = status.automaticSimulationHz;
      interval = ToTickInterval(scheduledHz);
      nextTick = now + interval;
    }
    if (scheduledMaximumSpeed && !status.maximumSimulationSpeedEnabled) {
      nextTick = now + interval;
    }
    if (status.executionState == sim::SimExecutionState::Running
        && previousStatus.executionState != sim::SimExecutionState::Running) {
      nextTick = now;
    }
    scheduledMaximumSpeed = status.maximumSimulationSpeedEnabled;
    previousStatus = status;

    if (status.executionState == sim::SimExecutionState::Paused
        && status.pendingTickCount > 0) {
      if (!TickOnce()) {
        return false;
      }
      continue;
    }

    if (status.executionState != sim::SimExecutionState::Running) {
      commands_.Wait(stopToken);
      continue;
    }

    if (status.maximumSimulationSpeedEnabled) {
      if (!TickOnce()) {
        return false;
      }
      continue;
    }

    if (now < nextTick) {
      commands_.WaitUntil(stopToken, nextTick);
      continue;
    }

    if (!TickOnce()) {
      return false;
    }
    nextTick += interval;
  }
  return true;
}

bool SimWorker::DrainCommandsAndSingleSteps(std::stop_token stopToken) {
  AssertSimThread();
  while (!stopToken.stop_requested() && commands_.DrainOne()) {
    sim::SimStatus status = runtime_->GetStatus();
    while (!stopToken.stop_requested()
           && status.executionState == sim::SimExecutionState::Paused
           && status.pendingTickCount > 0) {
      if (!TickOnce()) {
        return false;
      }
      status = runtime_->GetStatus();
    }
  }
  return true;
}

bool SimWorker::TickOnce() {
  AssertSimThread();
  if (runtime_->Tick()) {
    bridge_->PublishState();
    return true;
  }
  bridge_->PublishState();
  ReportFailure(runtime_->GetStatus().lastError);
  return false;
}

void SimWorker::AssertSimThread() const {
  assert(std::this_thread::get_id() == simThreadId_
         && "Simulation scheduling must run on the worker thread.");
}

void SimWorker::ReportStart(bool succeeded) {
  {
    std::scoped_lock lock(stateMutex_);
    startSucceeded_ = succeeded;
    startCompleted_ = true;
  }
  startCondition_.notify_all();
}

void SimWorker::ReportFailure(std::string error) noexcept {
  try {
    std::string failure =
        error.empty() ? "Simulation worker failed." : std::move(error);
    {
      std::scoped_lock lock(stateMutex_);
      lastError_ = failure;
    }
    events_.Enqueue(
        messaging::SimWorkerFatalEvent{.error = std::move(failure)});
  } catch (...) {
  }
  failed_.store(true, std::memory_order_release);
}
} // namespace app
