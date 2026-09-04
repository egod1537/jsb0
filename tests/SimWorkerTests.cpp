#include "app/SimWorker.hpp"
#include "common/Options.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/AutopilotFactory.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

std::unique_ptr<sim::SimRuntime> MakeRuntime() {
  return std::make_unique<sim::SimRuntime>(std::make_unique<sim::Simulation>(
      gnc::CreateAutopilot(gnc::AutopilotKind::Primary)));
}

struct Harness {
  explicit Harness(std::size_t telemetryCapacity =
                       opts::messaging::GuiTelemetryBatchCapacity)
      : telemetry(telemetryCapacity), client(commands, events, telemetry),
        worker(commands, events, telemetry, MakeRuntime()) {}

  ~Harness() { Stop(); }

  bool Start() {
    const bool started = worker.Start();
    events.Drain();
    telemetry.Drain();
    return started;
  }

  void Stop() {
    worker.RequestStop();
    worker.Join();
    events.Drain();
    telemetry.Drain();
  }

  template <typename Predicate>
  bool WaitFor(Predicate predicate, Clock::duration timeout = 2s) {
    const Clock::time_point deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      events.Drain();
      telemetry.Drain();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    events.Drain();
    telemetry.Drain();
    return predicate();
  }

  void PumpFor(Clock::duration duration) {
    const Clock::time_point deadline = Clock::now() + duration;
    while (Clock::now() < deadline) {
      events.Drain();
      telemetry.Drain();
      std::this_thread::sleep_for(1ms);
    }
    events.Drain();
    telemetry.Drain();
  }

  std::size_t TickCount() const {
    const double timeSec =
        client.GetSimSnapshot().primary.aircraft.simulationTimeSec;
    return static_cast<std::size_t>(
        std::llround(timeSec / opts::simulation::DtSec));
  }

  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  app::SimMessageClient client;
  app::SimWorker worker;
};

void TestNormalHzPauseResumeAndStop() {
  Harness harness;
  assert(harness.Start());
  harness.client.SetAutomaticSimulationHz(40.0);
  harness.client.StartSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Running;
  }));

  harness.PumpFor(180ms);
  harness.client.PauseSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Paused;
  }));
  const std::size_t pausedTicks = harness.TickCount();
  assert(pausedTicks >= 4);
  assert(pausedTicks <= 20);
  harness.PumpFor(80ms);
  assert(harness.TickCount() == pausedTicks);

  harness.client.ResumeSimulation();
  assert(harness.WaitFor([&] { return harness.TickCount() > pausedTicks; }));
  harness.client.StopSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Stopped;
  }));
  const std::size_t stoppedTicks = harness.TickCount();
  harness.PumpFor(80ms);
  assert(harness.TickCount() == stoppedTicks);
}

void TestPauseStepResumeOrderingAndDeterministicCount() {
  Harness harness;
  harness.client.StartSimulation();
  harness.client.PauseSimulation();
  harness.client.RequestSimTick();
  harness.client.ResumeSimulation();
  harness.client.PauseSimulation();
  assert(harness.Start());

  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
               == sim::SimExecutionState::Paused
           && harness.client.GetPendingSimTickCount() == 0;
  }));
  assert(harness.TickCount() == 1);
  harness.PumpFor(80ms);
  assert(harness.TickCount() == 1);

  harness.client.RequestSimTick();
  assert(harness.WaitFor([&] { return harness.TickCount() == 2; }));
  harness.PumpFor(80ms);
  assert(harness.TickCount() == 2);

  harness.client.ResumeSimulation();
  assert(harness.WaitFor([&] { return harness.TickCount() > 2; }));
}

void TestBehindScheduleCatchesUp() {
  Harness harness;
  auto delay = harness.commands.Subscribe<app::messaging::ManualControlCommand>(
      [](const auto &) { std::this_thread::sleep_for(120ms); });
  bool commandCompleted = false;
  auto result = harness.events.Subscribe<app::messaging::OperationResultEvent>(
      [&commandCompleted](const auto &) { commandCompleted = true; });
  assert(harness.Start());
  harness.client.SetAutomaticSimulationHz(100.0);
  harness.client.StartSimulation();
  assert(harness.WaitFor([&] { return harness.TickCount() >= 3; }));
  const std::size_t beforeDelay = harness.TickCount();

  const Clock::time_point enqueueStarted = Clock::now();
  assert(harness.client.SetManualControl(control::ControlInput{}));
  assert(Clock::now() - enqueueStarted < 20ms);
  std::size_t responsiveGuiIterations = 0;
  const Clock::time_point guiDeadline = Clock::now() + 40ms;
  while (Clock::now() < guiDeadline) {
    harness.events.Drain();
    harness.telemetry.Drain();
    ++responsiveGuiIterations;
    std::this_thread::yield();
  }
  assert(responsiveGuiIterations > 0);
  assert(!commandCompleted);
  assert(harness.WaitFor([&] { return commandCompleted; }));
  harness.PumpFor(30ms);
  harness.client.PauseSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Paused;
  }));

  const std::size_t caughtUpTicks = harness.TickCount() - beforeDelay;
  assert(caughtUpTicks >= 10);
}

void TestMaximumSpeedAndRequestStopResponsiveness() {
  Harness harness;
  assert(harness.Start());
  harness.client.SetAutomaticSimulationHz(20.0);
  harness.client.StartSimulation();
  harness.PumpFor(100ms);
  harness.client.PauseSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Paused;
  }));
  const std::size_t normalTicks = harness.TickCount();

  harness.client.SetMaximumSimulationSpeedEnabled(true);
  harness.client.ResumeSimulation();
  harness.PumpFor(50ms);
  harness.client.PauseSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Paused;
  }));
  const std::size_t maximumSpeedTicks = harness.TickCount() - normalTicks;
  assert(maximumSpeedTicks > normalTicks);
  assert(maximumSpeedTicks >= 5);

  const std::size_t beforeResume = harness.TickCount();
  harness.client.ResumeSimulation();
  assert(harness.WaitFor([&] { return harness.TickCount() > beforeResume; }));
  const Clock::time_point stopRequested = Clock::now();
  harness.Stop();
  const Clock::duration stopLatency = Clock::now() - stopRequested;
  assert(stopLatency < 500ms);
  const std::size_t joinedTicks = harness.TickCount();
  harness.PumpFor(50ms);
  assert(harness.TickCount() == joinedTicks);
}

void TestStartIsIdempotent() {
  Harness harness;
  assert(harness.Start());
  assert(harness.worker.Start());
  assert(harness.events.Drain() == 0);
}

void TestFatalFailureIsDeliveredAsReliableEvent() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  app::SimMessageClient client(commands, events, telemetry);
  app::SimWorker worker(commands,
      events,
      telemetry,
      std::unique_ptr<sim::SimRuntime>{});

  assert(!worker.Start());
  worker.Join();
  assert(worker.HasFailed());
  assert(events.Drain() == 1);
  const auto error = client.GetLastCommandError();
  assert(error.has_value());
  assert(*error == "Simulation worker requires a runtime.");
}

void TestStopInterruptsNormalRateWait() {
  Harness harness;
  assert(harness.Start());
  harness.client.SetAutomaticSimulationHz(1.0);
  harness.client.StartSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Running;
  }));

  const Clock::time_point stopRequested = Clock::now();
  harness.Stop();
  assert(Clock::now() - stopRequested < 250ms);
}

void TestStopInterruptsPausedWait() {
  Harness harness;
  assert(harness.Start());
  harness.client.StartSimulation();
  harness.client.PauseSimulation();
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
           == sim::SimExecutionState::Paused;
  }));

  const std::size_t pausedTicks = harness.TickCount();
  const Clock::time_point stopRequested = Clock::now();
  harness.Stop();
  assert(Clock::now() - stopRequested < 250ms);
  assert(harness.TickCount() == pausedTicks);
}

void TestStopDiscardsCommandBacklogAndRejectsNewCommands() {
  Harness harness;
  std::atomic_bool activeCommandEntered = false;
  auto delay = harness.commands.Subscribe<app::messaging::ManualControlCommand>(
      [&activeCommandEntered](const auto &) {
        activeCommandEntered.store(true, std::memory_order_release);
        std::this_thread::sleep_for(120ms);
      });
  std::atomic_size_t rateCallbacks = 0;
  auto rate = harness.commands.Subscribe<app::messaging::SimRateCommand>(
      [&rateCallbacks](const auto &) {
        rateCallbacks.fetch_add(1, std::memory_order_relaxed);
      });
  assert(harness.Start());
  assert(harness.client.SetManualControl(control::ControlInput{}));
  const Clock::time_point commandDeadline = Clock::now() + 1s;
  while (!activeCommandEntered.load(std::memory_order_acquire)
         && Clock::now() < commandDeadline) {
    std::this_thread::yield();
  }
  assert(activeCommandEntered.load(std::memory_order_acquire));

  constexpr std::size_t BacklogSize = 10'000;
  for (std::size_t index = 0; index < BacklogSize; ++index) {
    assert(
        harness.commands.Enqueue(app::messaging::SimRateCommand{.hz = 500.0}));
  }
  const Clock::time_point stopRequested = Clock::now();
  harness.worker.RequestStop();
  assert(!harness.commands.Enqueue(app::messaging::SimStepCommand{}));
  harness.worker.Join();
  assert(Clock::now() - stopRequested < 500ms);
  assert(harness.commands.Empty());
  assert(rateCallbacks.load(std::memory_order_relaxed) == 0);
}

void TestTelemetrySaturationDuringHighRateAndMaximumSpeedStop() {
  constexpr std::size_t TelemetryCapacity = 4;
  Harness harness(TelemetryCapacity);
  assert(harness.Start());
  harness.client.SetAutomaticSimulationHz(500.0);
  harness.client.StartSimulation();
  std::this_thread::sleep_for(100ms);
  harness.client.SetMaximumSimulationSpeedEnabled(true);
  std::this_thread::sleep_for(50ms);

  const Clock::time_point stopRequested = Clock::now();
  harness.worker.RequestStop();
  harness.worker.Join();
  assert(Clock::now() - stopRequested < 500ms);
  assert(harness.telemetry.Size() <= TelemetryCapacity);
  assert(harness.telemetry.GetStats().droppedBatches > 0);
  assert(!harness.commands.Enqueue(app::messaging::SimStepCommand{}));
}

sim::SimSnapshot RunDeterministicCommandSequence() {
  Harness harness;
  harness.client.StartSimulation();
  harness.client.PauseSimulation();
  constexpr std::size_t StepCount = 50;
  for (std::size_t step = 0; step < StepCount; ++step) {
    harness.client.RequestSimTick();
    harness.client.ResumeSimulation();
    harness.client.PauseSimulation();
  }
  assert(harness.Start());
  assert(harness.WaitFor([&] {
    return harness.client.GetSimExecutionState()
               == sim::SimExecutionState::Paused
           && harness.client.GetPendingSimTickCount() == 0
           && harness.TickCount() == StepCount;
  }));
  return harness.client.GetSimSnapshot();
}

void TestCommandSequenceIsDeterministicAcrossWorkerLifetimes() {
  const sim::AircraftState first =
      RunDeterministicCommandSequence().primary.aircraft;
  const sim::AircraftState second =
      RunDeterministicCommandSequence().primary.aircraft;

  assert(first.simulationTimeSec == second.simulationTimeSec);
  assert(first.altitudeAslM == second.altitudeAslM);
  assert(first.calibratedAirspeedMps == second.calibratedAirspeedMps);
  assert(first.rollRad == second.rollRad);
  assert(first.pitchRad == second.pitchRad);
  assert(first.headingRad == second.headingRad);
  assert(first.pRadPerSec == second.pRadPerSec);
  assert(first.qRadPerSec == second.qRadPerSec);
  assert(first.rRadPerSec == second.rRadPerSec);
}

void TestWorkerCatchesSubscriberExceptionAndShutsDown() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  auto exception =
      commands.Subscribe<app::messaging::SimStartCommand>([](const auto &) {
        throw std::runtime_error("injected worker failure");
      });
  app::SimMessageClient client(commands, events, telemetry);
  app::SimWorker worker(commands, events, telemetry, MakeRuntime());
  assert(commands.Enqueue(app::messaging::SimStartCommand{}));
  assert(worker.Start());

  const Clock::time_point failureDeadline = Clock::now() + 2s;
  while (!worker.HasFailed() && Clock::now() < failureDeadline) {
    std::this_thread::yield();
  }
  assert(worker.HasFailed());
  worker.Join();
  events.Drain();
  const auto error = client.GetLastCommandError();
  assert(error.has_value());
  assert(*error == "injected worker failure");
}
} // namespace

int main() {
  TestNormalHzPauseResumeAndStop();
  TestPauseStepResumeOrderingAndDeterministicCount();
  TestBehindScheduleCatchesUp();
  TestMaximumSpeedAndRequestStopResponsiveness();
  TestStartIsIdempotent();
  TestFatalFailureIsDeliveredAsReliableEvent();
  TestStopInterruptsNormalRateWait();
  TestStopInterruptsPausedWait();
  TestStopDiscardsCommandBacklogAndRejectsNewCommands();
  TestTelemetrySaturationDuringHighRateAndMaximumSpeedStop();
  TestCommandSequenceIsDeterministicAcrossWorkerLifetimes();
  TestWorkerCatchesSubscriberExceptionAndShutsDown();
  return 0;
}
