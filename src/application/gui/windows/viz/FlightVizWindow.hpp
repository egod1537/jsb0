#pragma once

#include "application/gui/Window.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"

namespace sim {
class Simulation;
}

namespace gui {
enum class SimulationSlot {
  Primary,
  Baseline,
};

class FlightVizWindow final : public gui::Window {
public:
  FlightVizWindow(SimulationSlot slot, sim::Simulation *mainSimulation,
      sim::Simulation *shadowSimulation);

  SimulationSlot GetSimulationSlot() const { return slot_; }
  const sim::Simulation *GetMainSimulation() const;
  const sim::Simulation *GetShadowSimulation() const;
  viz::FlightVisualizer &GetVisualizer() { return visualizer_; }
  const viz::FlightVisualizer &GetVisualizer() const { return visualizer_; }

protected:
  ImGuiWindowFlags GetWindowFlags() const override;
  void OnRender(gui::GUI &gui) override;

private:
  SimulationSlot slot_;
  viz::FlightVisualizer visualizer_;
};
} // namespace gui
