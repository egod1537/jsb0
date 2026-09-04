#pragma once

#include "gui/Window.hpp"

namespace gui {
class FlightConsoleWindow final : public Window {
public:
  FlightConsoleWindow();

protected:
  void OnRender(const sim::SimSnapshot &snapshot) override;
};
} // namespace gui
