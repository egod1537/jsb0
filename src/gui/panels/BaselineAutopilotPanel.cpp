#include "gui/panels/BaselineAutopilotPanel.hpp"

#include "gui/features/gnc/px4/attitude/Px4AttitudePanel.hpp"
#include "gui/features/gnc/px4/tecs/TecsPanel.hpp"
#include "flightui/FlightUI.hpp"

namespace gui {
namespace UI = FlightUI;

void BaselineAutopilotPanel::Draw(const BaselineAutopilotPanelProps &props) {
  const UI::UIElement layout =
      UI::VerticalLayout().Spacing(8.0F) + UI::Heading("Autopilot Controls")
      + Px4AttitudePanel::BuildRoll(props) + TecsPanel::Build(props)
      + Px4AttitudePanel::BuildPitch(props)
      + Px4AttitudePanel::BuildCourse(props)
      + Px4AttitudePanel::BuildYaw(props);
  layout.Render();
}
} // namespace gui
