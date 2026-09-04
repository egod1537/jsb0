#include "gui/panels/BaselineAutopilotPanel.hpp"

#include "gui/features/gnc/px4/attitude/Px4AttitudePanel.hpp"
#include "gui/features/gnc/px4/tecs/TecsPanel.hpp"
#include "flightui/FlightUI.hpp"

namespace gui {

void BaselineAutopilotPanel::Draw(const BaselineAutopilotPanelProps &props) {
  const ui::UIElement layout =
      ui::VerticalLayout().Spacing(8.0F) + ui::Heading("Autopilot Controls")
      + Px4AttitudePanel::BuildRoll(props) + TecsPanel::Build(props)
      + Px4AttitudePanel::BuildPitch(props)
      + Px4AttitudePanel::BuildCourse(props)
      + Px4AttitudePanel::BuildYaw(props);
  layout.Render();
}
} // namespace gui
