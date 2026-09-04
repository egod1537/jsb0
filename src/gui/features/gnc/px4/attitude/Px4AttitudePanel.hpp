#pragma once

#include "flightui/FlightUI.hpp"

namespace gui {
struct BaselineAutopilotPanelProps;

class Px4AttitudePanel {
public:
  static FlightUI::UIElement BuildRoll(
      const BaselineAutopilotPanelProps &props);
  static FlightUI::UIElement BuildPitch(
      const BaselineAutopilotPanelProps &props);
  static FlightUI::UIElement BuildCourse(
      const BaselineAutopilotPanelProps &props);
  static FlightUI::UIElement BuildYaw(const BaselineAutopilotPanelProps &props);
};
} // namespace gui
