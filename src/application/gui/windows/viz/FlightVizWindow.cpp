#include "application/gui/windows/viz/FlightVizWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/resources/EditorIcon.hpp"

namespace {
const char *GetWindowTitle(gui::SimulationSlot slot) {
  return slot == gui::SimulationSlot::Primary ? "Flight Viz · Primary"
                                              : "Flight Viz · Baseline";
}

const char *ResolveWindowId(gui::SimulationSlot slot) {
  return slot == gui::SimulationSlot::Primary ? "FlightVizPrimary"
                                              : "FlightVizBaseline";
}

const char *GetUnavailableMessage(gui::SimulationSlot slot) {
  return slot == gui::SimulationSlot::Primary
             ? "Primary simulation unavailable"
             : "Baseline simulation unavailable";
}

const char *GetShadowTooltip(gui::SimulationSlot slot) {
  return slot == gui::SimulationSlot::Primary ? "Show Baseline Shadow"
                                              : "Show Primary Shadow";
}
} // namespace

namespace gui {
FlightVizWindow::FlightVizWindow(SimulationSlot slot,
    sim::Simulation *mainSimulation, sim::Simulation *shadowSimulation)
    : Window(GetWindowTitle(slot), EditorIconAliases::FlightViz,
          ResolveWindowId(slot)),
      slot_(slot), visualizer_(mainSimulation, shadowSimulation) {}

const sim::Simulation *FlightVizWindow::GetMainSimulation() const {
  return visualizer_.GetMainSimulation();
}

const sim::Simulation *FlightVizWindow::GetShadowSimulation() const {
  return visualizer_.GetShadowSimulation();
}

ImGuiWindowFlags FlightVizWindow::GetWindowFlags() const {
  return ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
}

void FlightVizWindow::OnRender(gui::GUI &gui) {
  visualizer_.Tick();
  const EditorIconHandle shadowIcon =
      gui.GetEditorIcons().Get(EditorIconAliases::ShadowAircraft);
  const EditorIconHandle viewOptionsIcon =
      gui.GetEditorIcons().Get(EditorIconAliases::ViewOptions);
  const EditorIconHandle cameraViewIcon =
      gui.GetEditorIcons().Get(EditorIconAliases::CameraView);
  visualizer_.RenderScene(shadowIcon,
      viewOptionsIcon,
      cameraViewIcon,
      GetShadowTooltip(slot_),
      GetUnavailableMessage(slot_));
}
} // namespace gui
