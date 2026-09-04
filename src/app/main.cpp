#include "app/Application.hpp"
#include "gui/GUI.hpp"
#include "sim/Simulation.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <csignal>
#include <memory>

namespace {
volatile std::sig_atomic_t running = 1;
}

void OnSignal(int) { running = 0; }

int main() {
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  std::unique_ptr<sim::Simulation> primarySimulation =
      std::make_unique<sim::Simulation>(
          sim::ExecutionVariantResolver::CreateAutopilot(
              sim::ExecutionVariant::Primary));
  std::unique_ptr<sim::Simulation> baselineSimulation =
      std::make_unique<sim::Simulation>(
          sim::ExecutionVariantResolver::CreateAutopilot(
              sim::ExecutionVariant::Baseline));
  std::unique_ptr<gui::GUI> gui = std::make_unique<gui::GUI>();
  auto simRuntime =
      std::make_unique<sim::SimRuntime>(std::move(primarySimulation),
          std::move(baselineSimulation));

  Application app(std::move(gui), std::move(simRuntime));
  return app.Run(running) ? 0 : 1;
}
