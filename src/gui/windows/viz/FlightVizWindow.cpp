#include "gui/windows/viz/FlightVizWindow.hpp"

#include "gui/resources/EditorIcon.hpp"
#include "gui/resources/EditorIconRegistry.hpp"

namespace {
const char *GetWindowTitle(sim::SimSlot slot) {
  return slot == sim::SimSlot::Primary ? "Flight Viz · Primary"
                                              : "Flight Viz · Baseline";
}

const char *ResolveWindowId(sim::SimSlot slot) {
  return slot == sim::SimSlot::Primary ? "FlightVizPrimary"
                                              : "FlightVizBaseline";
}

const char *GetUnavailableMessage(sim::SimSlot slot) {
  return slot == sim::SimSlot::Primary
             ? "Primary simulation unavailable"
             : "Baseline simulation unavailable";
}

const char *GetShadowTooltip(sim::SimSlot slot) {
  return slot == sim::SimSlot::Primary ? "Show Baseline Shadow"
                                              : "Show Primary Shadow";
}
} // namespace

namespace gui {
FlightVizWindow::FlightVizWindow(sim::SimSlot slot,
    EditorIconRegistry *icons)
    : Window(GetWindowTitle(slot), editor_icon_aliases::FlightViz,
          ResolveWindowId(slot)),
      slot_(slot), icons_(icons) {}

ImGuiWindowFlags FlightVizWindow::GetWindowFlags() const {
  return ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
}

void FlightVizWindow::OnRender(const sim::SimSnapshot &snapshot) {
  const sim::SimInstanceSnapshot *primary =
      snapshot.primary.available ? &snapshot.primary : nullptr;
  const sim::SimInstanceSnapshot *baseline =
      snapshot.baseline.has_value() && snapshot.baseline->available
          ? &*snapshot.baseline
          : nullptr;
  const bool primarySlot = slot_ == sim::SimSlot::Primary;
  visualizer_.Tick(primarySlot ? primary : baseline,
      primarySlot ? baseline : primary);
  const EditorIconHandle shadowIcon =
      icons_ != nullptr ? icons_->Get(editor_icon_aliases::ShadowAircraft)
                        : EditorIconHandle{};
  const EditorIconHandle viewOptionsIcon =
      icons_ != nullptr ? icons_->Get(editor_icon_aliases::ViewOptions)
                        : EditorIconHandle{};
  const EditorIconHandle cameraViewIcon =
      icons_ != nullptr ? icons_->Get(editor_icon_aliases::CameraView)
                        : EditorIconHandle{};
  visualizer_.RenderScene(shadowIcon,
      viewOptionsIcon,
      cameraViewIcon,
      GetShadowTooltip(slot_),
      GetUnavailableMessage(slot_),
      architecture::EventSink<FlightVizShadowVisibilityChanged>{
          [this](const FlightVizShadowVisibilityChanged &event) {
            OnEvent(event);
          }},
      architecture::EventSink<FlightVizCameraViewToggleRequested>{
          [this](const FlightVizCameraViewToggleRequested &event) {
            OnEvent(event);
          }},
      architecture::EventSink<FlightVizDisplayOptionsChanged>{
          [this](
              const FlightVizDisplayOptionsChanged &event) { OnEvent(event); }},
      architecture::EventSink<FlightVizClearPathRequested>{
          [this](const FlightVizClearPathRequested &event) { OnEvent(event); }});
}

void FlightVizWindow::OnEvent(const FlightVizShadowVisibilityChanged &event) {
  visualizer_.SetShadowEnabled(event.enabled);
}

void FlightVizWindow::OnEvent(const FlightVizCameraViewToggleRequested &) {
  const viz::ViewMode next = visualizer_.GetViewMode() == viz::ViewMode::Orbit
                                 ? viz::ViewMode::ThirdPerson
                                 : viz::ViewMode::Orbit;
  visualizer_.SetViewMode(next);
}

void FlightVizWindow::OnEvent(const FlightVizDisplayOptionsChanged &event) {
  visualizer_.SetViewOptions({
      .showGroundGrid = event.showGroundGrid,
      .showTelemetry = event.showTelemetry,
      .showMinimap = event.showMinimap,
  });
}

void FlightVizWindow::OnEvent(const FlightVizClearPathRequested &) {
  visualizer_.ClearFlightPath();
}
} // namespace gui
