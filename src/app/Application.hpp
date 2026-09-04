#pragma once

#include "messaging/MessageBus.hpp"

#include <csignal>
#include <cstdint>
#include <memory>

namespace sim {
class SimRuntime;
}
namespace gui {
class GUI;
}
namespace app {
class SimMessageClient;
namespace messaging {
class GuiSimBridge;
}
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

  bool RunTick(const volatile std::sig_atomic_t &running);
  bool Tick();
  void TickGUI();

  void Shutdown();

  // Owned services
  app::messaging::MessageBus messageBus_;
  std::unique_ptr<sim::SimRuntime> simRuntime_;
  std::unique_ptr<app::messaging::GuiSimBridge> guiSimBridge_;
  std::unique_ptr<app::SimMessageClient> simMessageClient_;
  std::unique_ptr<gui::GUI> gui_;
};
