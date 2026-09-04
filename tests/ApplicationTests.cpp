#include "app/Application.hpp"
#include "common/Options.hpp"
#include "gui/GUI.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/AutopilotFactory.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <cassert>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <memory>
#include <thread>

namespace {
using namespace std::chrono_literals;

std::unique_ptr<sim::SimRuntime> MakeRuntime() {
  return std::make_unique<sim::SimRuntime>(std::make_unique<sim::Simulation>(
      gnc::CreateAutopilot(gnc::AutopilotKind::Primary)));
}

struct GuiState {
  std::size_t startCount = 0;
  std::size_t pollCount = 0;
  std::size_t tickCount = 0;
  std::size_t exitCount = 0;
  std::size_t clientBindingCount = 0;
  std::size_t closeAfterPolls = 0;
  std::size_t closeAfterTicks = 0;
  std::chrono::milliseconds tickDelay = 0ms;
  bool startMaximumSpeed = false;
  bool startPaused = false;
  bool closed = false;
  bool clientBoundAtStart = false;
  bool runtimeInitializedAtStart = false;
  bool observedPaused = false;
  double latestSimulationTimeSec = 0.0;
};

class FakeGUI final : public gui::GUI {
public:
  explicit FakeGUI(std::shared_ptr<GuiState> state)
      : state_(std::move(state)) {}

  bool Initialize() override {
    ++state_->startCount;
    state_->clientBoundAtStart = client_ != nullptr;
    state_->runtimeInitializedAtStart =
        client_ != nullptr && client_->GetSimSnapshot().status.initialized;
    return true;
  }

  void PollPlatformEvents() override {
    ++state_->pollCount;
    if (state_->closeAfterPolls > 0
        && state_->pollCount >= state_->closeAfterPolls) {
      state_->closed = true;
    }
  }

  void Tick() override {
    ++state_->tickCount;
    if (state_->startMaximumSpeed && state_->tickCount == 1) {
      client_->SetMaximumSimulationSpeedEnabled(true);
      client_->StartSimulation();
    } else if (state_->startPaused && state_->tickCount == 1) {
      client_->StartSimulation();
      client_->PauseSimulation();
    }
    if (state_->tickDelay > 0ms) {
      std::this_thread::sleep_for(state_->tickDelay);
    }
    if (client_ != nullptr) {
      const sim::SimSnapshot snapshot = client_->GetSimSnapshot();
      state_->latestSimulationTimeSec =
          snapshot.primary.aircraft.simulationTimeSec;
      state_->observedPaused =
          state_->observedPaused
          || snapshot.status.executionState == sim::SimExecutionState::Paused;
    }
    if (state_->closeAfterTicks > 0
        && state_->tickCount >= state_->closeAfterTicks) {
      state_->closed = true;
    }
  }

  void Shutdown() override {
    ++state_->exitCount;
    state_->closed = true;
  }

  bool ShouldClose() const override { return state_->closed; }

  void SetSimMessageClient(app::SimMessageClient *client) override {
    ++state_->clientBindingCount;
    client_ = client;
  }

private:
  std::shared_ptr<GuiState> state_;
  app::SimMessageClient *client_ = nullptr;
};

void TestStartupFrameOrderWindowCloseAndIdempotentShutdown() {
  auto state = std::make_shared<GuiState>();
  state->closeAfterPolls = 3;
  volatile std::sig_atomic_t running = 1;
  {
    Application application(std::make_unique<FakeGUI>(state), MakeRuntime());
    assert(application.Run(running));
    assert(state->clientBindingCount == 1);
    assert(state->startCount == 1);
    assert(state->clientBoundAtStart);
    assert(state->runtimeInitializedAtStart);
    assert(state->pollCount == 3);
    assert(state->tickCount == 2);
    assert(state->exitCount == 1);
  }
  assert(state->exitCount == 1);
}

void TestMaximumSpeedRemainsResponsiveWithSlowGuiFrames() {
  auto state = std::make_shared<GuiState>();
  state->startMaximumSpeed = true;
  state->tickDelay = 100ms;
  state->closeAfterTicks = 6;
  volatile std::sig_atomic_t running = 1;

  Application application(std::make_unique<FakeGUI>(state), MakeRuntime());
  assert(application.Run(running));
  assert(state->tickCount == 6);
  assert(state->pollCount == 6);
  assert(state->latestSimulationTimeSec
         > static_cast<double>(state->tickCount) / opts::simulation::Hz);
}

void TestPausedWindowCloseUsesNormalShutdown() {
  auto state = std::make_shared<GuiState>();
  state->startPaused = true;
  state->tickDelay = 20ms;
  state->closeAfterTicks = 5;
  volatile std::sig_atomic_t running = 1;

  Application application(std::make_unique<FakeGUI>(state), MakeRuntime());
  assert(application.Run(running));
  assert(state->observedPaused);
  assert(state->exitCount == 1);
}

void TestRepeatedStartupAndShutdown() {
  volatile std::sig_atomic_t running = 1;
  for (std::size_t iteration = 0; iteration < 3; ++iteration) {
    auto state = std::make_shared<GuiState>();
    state->closeAfterPolls = 1;
    {
      Application application(std::make_unique<FakeGUI>(state), MakeRuntime());
      assert(application.Run(running));
    }
    assert(state->startCount == 1);
    assert(state->exitCount == 1);
  }
}
} // namespace

int main() {
  TestStartupFrameOrderWindowCloseAndIdempotentShutdown();
  TestMaximumSpeedRemainsResponsiveWithSlowGuiFrames();
  TestPausedWindowCloseUsesNormalShutdown();
  TestRepeatedStartupAndShutdown();
  return 0;
}
