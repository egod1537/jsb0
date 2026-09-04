#pragma once

#include "messaging/MessageQueues.hpp"

#include <csignal>
#include <cstdint>
#include <memory>
#include <thread>

namespace sim {
class SimRuntime;
}
namespace gui {
class GUI;
}
namespace app {
class SimMessageClient;
class SimWorker;
} // namespace app

class Application {
public:
  // Lifetime and main loop
  Application();
  ~Application();
  Application(std::unique_ptr<gui::GUI> gui,
      std::unique_ptr<sim::SimRuntime> simRuntime);
  bool Run(const volatile std::sig_atomic_t &running);

private:
  // Application lifecycle
  bool Initialize();

  bool RunMainLoop(const volatile std::sig_atomic_t &running);
  void DrainSimEvents();

  void Shutdown();
  void AssertGuiThread() const;

  // Cross-thread transport
  app::messaging::GuiToSimQueue guiToSimQueue_;
  app::messaging::SimToGuiQueue simToGuiQueue_;
  app::messaging::SimToGuiTelemetryQueue simToGuiTelemetryQueue_;

  // Owned services
  std::unique_ptr<app::SimWorker> simWorker_;
  std::unique_ptr<app::SimMessageClient> simMessageClient_;
  std::unique_ptr<gui::GUI> gui_;

  // Lifecycle state
  bool shutdown_ = false;

  // Thread affinity
  std::thread::id guiThreadId_ = std::this_thread::get_id();
};
