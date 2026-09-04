#pragma once

#include "gui/features/simulation/SimController.hpp"
#include "gui/Window.hpp"

namespace ui {
class UIElement;
}

namespace gui {
class SimWindow final : public gui::Window {
public:
  explicit SimWindow(SimController &controller);

protected:
  void OnRender(const sim::SimSnapshot &snapshot) override;

private:
  // Tab rendering
  void DrawInitialConditionTab(const sim::SimSnapshot &snapshot);
  void DrawDiagnosticsTab(const sim::SimSnapshot &snapshot);
  void DrawEnvironmentTab();
  void DrawAircraftTab(const sim::SimSnapshot &snapshot);

  // Initial-condition controls
  ui::UIElement DrawInitialConditionFields();
  ui::UIElement DrawInitialConditionActions(
      const sim::SimSnapshot &snapshot);

  // Simulation diagnostics
  ui::UIElement DrawLastError(
      const sim::SimSnapshot &snapshot) const;

  SimController &controller_;
};
} // namespace gui
