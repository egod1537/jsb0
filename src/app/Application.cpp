#include "app/Application.hpp"
#include "common/Options.hpp"

#include "gui/GUI.hpp"
#include "messaging/GuiSimBridge.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

namespace {
using Clock = std::chrono::steady_clock;

Clock::duration ToClockDuration(double seconds) {
  return std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
}

Clock::duration ToSimulationInterval(double hz) {
  const double clamped = std::clamp(hz,
      sim::MinimumAutomaticSimulationHz,
      sim::MaximumAutomaticSimulationHz);
  return ToClockDuration(1.0 / clamped);
}

bool RunsAtMaximumSpeed(const sim::SimStatus &status) {
  return status.maximumSimulationSpeedEnabled
         && status.executionState == sim::SimExecutionState::Running;
}

template <typename SimulationTick>
bool RunScheduledSimulationTicks(const sim::SimStatus &status,
    const volatile std::sig_atomic_t &running, Clock::time_point &now,
    Clock::duration simulationInterval, Clock::time_point &nextSimulationTick,
    Clock::time_point nextGuiTick, SimulationTick &&tick) {
  const bool hasPendingManualTick =
      status.executionState == sim::SimExecutionState::Paused
      && status.pendingTickCount > 0;
  if (hasPendingManualTick) {
    return tick();
  }
  if (RunsAtMaximumSpeed(status)) {
    do {
      if (!tick()) {
        return false;
      }
      now = Clock::now();
    } while (running && now < nextGuiTick);
    return true;
  }
  if (status.executionState == sim::SimExecutionState::Paused) {
    nextSimulationTick = now + simulationInterval;
    return true;
  }

  while (now >= nextSimulationTick) {
    if (!tick()) {
      return false;
    }
    nextSimulationTick += simulationInterval;
    now = Clock::now();
  }
  return true;
}

template <typename GuiTick>
void RunScheduledGuiTick(Clock::time_point &now, Clock::duration guiInterval,
    Clock::time_point &nextGuiTick, GuiTick &&tick) {
  if (now < nextGuiTick) {
    return;
  }

  tick();
  now = Clock::now();
  do {
    nextGuiTick += guiInterval;
  } while (nextGuiTick <= now);
}
} // namespace

Application::Application(std::unique_ptr<gui::GUI> gui,
    std::unique_ptr<sim::SimRuntime> simRuntime)
    : simRuntime_(std::move(simRuntime)), gui_(std::move(gui)) {
  if (simRuntime_ != nullptr) {
    guiSimBridge_ = std::make_unique<app::messaging::GuiSimBridge>(messageBus_,
        *simRuntime_);
    simMessageClient_ = std::make_unique<app::SimMessageClient>(messageBus_);
  }
}

Application::Application() = default;
Application::~Application() = default;

bool Application::Run(const volatile std::sig_atomic_t &running) {
  if (!Initialize()) {
    Shutdown();
    return false;
  }

  const bool succeeded = RunTick(running);
  Shutdown();
  return succeeded;
}

bool Application::RunTick(const volatile std::sig_atomic_t &running) {
  double scheduledSimulationHz = simRuntime_->GetAutomaticSimulationHz();
  bool scheduledMaximumSimulationSpeed =
      simRuntime_->IsMaximumSimulationSpeedEnabled();
  Clock::duration simulationInterval =
      ToSimulationInterval(scheduledSimulationHz);
  const Clock::duration guiInterval =
      opts::gui::RenderDtSec > 0.0 ? ToClockDuration(opts::gui::RenderDtSec)
                                   : simulationInterval;

  auto nextSimulationTick = Clock::now();
  auto nextGuiTick = nextSimulationTick;

  while (running && !gui_->ShouldClose()) {
    auto now = Clock::now();
    const sim::SimStatus status = simRuntime_->GetStatus();

    if (status.maximumSimulationSpeedEnabled
        != scheduledMaximumSimulationSpeed) {
      scheduledMaximumSimulationSpeed = status.maximumSimulationSpeedEnabled;
      nextSimulationTick = now + simulationInterval;
    }
    if (status.automaticSimulationHz != scheduledSimulationHz) {
      scheduledSimulationHz = status.automaticSimulationHz;
      simulationInterval = ToSimulationInterval(scheduledSimulationHz);
      if (!status.maximumSimulationSpeedEnabled) {
        nextSimulationTick = now + simulationInterval;
      }
    }

    if (!RunScheduledSimulationTicks(status,
            running,
            now,
            simulationInterval,
            nextSimulationTick,
            nextGuiTick,
            [this] { return Tick(); })) {
      return false;
    }
    RunScheduledGuiTick(now, guiInterval, nextGuiTick, [this] { TickGUI(); });

    if (!RunsAtMaximumSpeed(status)) {
      std::this_thread::sleep_until(std::min(nextSimulationTick, nextGuiTick));
    }
  }

  return true;
}

bool Application::Initialize() {
  if (gui_ == nullptr || simRuntime_ == nullptr || guiSimBridge_ == nullptr
      || simMessageClient_ == nullptr) {
    std::cerr << "Application requires GUI, simulation runtime, message "
                 "adapter, and message client instances\n";
    return false;
  }
  gui_->SetSimMessageClient(simMessageClient_.get());
  if (!simRuntime_->Initialize()) {
    std::cerr << "Failed to initialize simulation runtime: "
              << simRuntime_->GetStatus().lastError << '\n';
    return false;
  }
  guiSimBridge_->PublishState();
  if (!gui_->Start()) {
    std::cerr << "Failed to start GUI\n";
    return false;
  }
  return true;
}

bool Application::Tick() {
  if (!simRuntime_->Tick()) {
    return false;
  }
  guiSimBridge_->PublishState();
  return true;
}

void Application::TickGUI() { gui_->Tick(); }

void Application::Shutdown() {
  if (gui_ != nullptr) {
    gui_->Exit();
  }
  if (simRuntime_ != nullptr) {
    simRuntime_->Shutdown();
  }
}
