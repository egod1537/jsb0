#pragma once

#include "gui/Window.hpp"

namespace gui {
class ScenarioWindow final : public Window {
public:
  ScenarioWindow();

protected:
  // Window configuration and rendering
  void PrepareWindow() override;
  void OnRender(const sim::SimSnapshot &snapshot) override;
};
} // namespace gui
