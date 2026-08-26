#include "application/Application.hpp"
#include "application/gui/GUI.hpp"
#include "application/gui/GUIConfig.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/gnc/autopilot/MyAutopilot.hpp"
#include "application/sim/gnc/autopilot/PX4Autopilot.hpp"

#include <csignal>
#include <memory>

namespace {
volatile std::sig_atomic_t running = 1;
}

void HandleSignal(int) { running = 0; }

int main() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  sim::SimulationConfig simConfig;
  gui::GUIConfig guiConfig;

  auto primaryAutopilot = std::make_unique<gnc::MyAutopilot>();
  std::unique_ptr<sim::Simulation> primarySimulation =
      std::make_unique<sim::Simulation>(std::move(primaryAutopilot));
  auto baselineAutopilot = std::make_unique<gnc::PX4Autopilot>();
  std::unique_ptr<sim::Simulation> baselineSimulation =
      std::make_unique<sim::Simulation>(std::move(baselineAutopilot));
  std::unique_ptr<gui::GUI> gui = std::make_unique<gui::GUI>(*primarySimulation,
      baselineSimulation.get(),
      guiConfig);

  Application app(std::move(gui),
      std::move(primarySimulation),
      simConfig,
      std::move(baselineSimulation));
  return app.Run(running) ? 0 : 1;
}
