#pragma once

#include "flightui/FlightUI.hpp"

namespace gui {
struct BaselineAutopilotPanelProps;

class Px4AttitudePanel {
public:
  static ui::UIElement BuildRoll(
      const BaselineAutopilotPanelProps &props);
  static ui::UIElement BuildPitch(
      const BaselineAutopilotPanelProps &props);
  static ui::UIElement BuildCourse(
      const BaselineAutopilotPanelProps &props);
  static ui::UIElement BuildYaw(const BaselineAutopilotPanelProps &props);
};
} // namespace gui
