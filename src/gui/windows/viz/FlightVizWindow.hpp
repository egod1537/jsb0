#pragma once

#include "gui/Window.hpp"
#include "gui/features/flightviz/FlightVisualizer.hpp"
#include "sim/runtime/SimContracts.hpp"

namespace gui {
class EditorIconRegistry;

class FlightVizWindow final : public gui::Window {
public:
  explicit FlightVizWindow(sim::SimSlot slot,
      EditorIconRegistry *icons = nullptr);

  sim::SimSlot GetSimSlot() const { return slot_; }
  viz::FlightVisualizer &GetVisualizer() { return visualizer_; }
  const viz::FlightVisualizer &GetVisualizer() const { return visualizer_; }

protected:
  ImGuiWindowFlags GetWindowFlags() const override;
  void OnRender(const sim::SimSnapshot &snapshot) override;

private:
  void OnEvent(const FlightVizShadowVisibilityChanged &event);
  void OnEvent(const FlightVizCameraViewToggleRequested &event);
  void OnEvent(const FlightVizDisplayOptionsChanged &event);
  void OnEvent(const FlightVizClearPathRequested &event);

  sim::SimSlot slot_;
  EditorIconRegistry *icons_ = nullptr;
  viz::FlightVisualizer visualizer_;
};
} // namespace gui
