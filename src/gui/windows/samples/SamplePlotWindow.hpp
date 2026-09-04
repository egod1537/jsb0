#pragma once

#include "gui/Window.hpp"

namespace gui {
class SamplePlotWindow final : public Window {
public:
  SamplePlotWindow();

protected:
  void OnRender(const sim::SimSnapshot &snapshot) override;
};
} // namespace gui
