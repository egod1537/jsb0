#pragma once

#include "flightui/FlightUI.hpp"

namespace gui {
struct BaselineAutopilotPanelProps;

class TecsPanel {
public:
  static FlightUI::UIElement Build(const BaselineAutopilotPanelProps &props);
};
} // namespace gui
