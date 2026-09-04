#include "gui/panels/AutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

namespace gui {

namespace {
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;
constexpr float AutopilotParameterLabelWidth = 148.0F;

ui::UIElement MakeAutopilotTargetRow(const char *targetLabel,
    const char *inputId, bool enabled, double targetValue,
    architecture::EventSink<PrimaryRollHoldValueChanged> events,
    double step = 1.0, double fastStep = 10.0) {
  return ui::HorizontalLayout().Spacing(
      8.0F)[+ui::TextDisabled(targetLabel)
            + ui::InputDouble(inputId, targetValue)
                .Width(AutopilotTargetInputWidth)
                .Step(step)
                .FastStep(fastStep)
                .Format("%.2f")
                .OnChanged([events](double value) {
                  events.Emit({PrimaryRollHoldField::TargetDeg, value});
                })
            + ui::Text(enabled ? "Hold" : "Off")];
}

ui::UIElement MakeRollHoldStatusRow(const AutopilotPanelProps &props) {
  // clang-format off
  return ui::HorizontalLayout()
      .Spacing(8.0F)
      [
        +ui::ValueLabel("Current Roll", props.currentRollDeg, "%.2f deg")
        + ui::ValueLabel(
              "Roll Rate", props.currentRollRateDegPerSec, "%.2f deg/s")
        + ui::ValueLabel("Aileron", props.currentAileron, "%.3f")
        + ui::StatusBadge(props.state.rollHold ? "Not Implemented" : "Inactive",
              props.state.rollHold ? ui::StatusTone::Warning
                                   : ui::StatusTone::Neutral)
        + ui::Button("Capture")
              .Enabled(props.events.IsConnected())
              .OnAction([events = props.events,
                            value = props.currentRollDeg] {
                events.Emit({PrimaryRollHoldField::TargetDeg, value});
              })
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

ui::UIElement MakeRollHoldParametersFoldOut(AutopilotPanelState &state,
    architecture::EventSink<PrimaryRollHoldValueChanged> events) {
  ui::PropertyGridBuilder parameters =
      ui::PropertyGrid("PrimaryRollHoldParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .AlternatingRows();
  parameters
      .Add(ui::PropertyRow(
          "Roll Angle P Gain")[ui::ScalarEditor("RollAngleProportionalGain",
          state.rollAngleProportionalGain)
              .ShowSlider(false)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.2f")
              .OnChanged([events](double value) {
                events.Emit(
                    {PrimaryRollHoldField::AngleProportionalGain, value});
              })])
      .Add(ui::PropertyRow(
          "Roll Rate P Gain")[ui::ScalarEditor("RollRateProportionalGain",
          state.rollRateProportionalGain)
              .ShowSlider(false)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.2f")
              .OnChanged([events](double value) {
                events.Emit(
                    {PrimaryRollHoldField::RateProportionalGain, value});
              })]);

  return ui::FoldOut("P-P Parameters")
      .Open(state.rollHoldParametersOpen)
      .Section()
      .Id("RollHoldParameters")[parameters];
}

ui::UIElement MakeRollHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  const ui::UIElement content =
      ui::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotTargetRow("Target Roll (deg)",
          "##RollHoldTarget",
          state.rollHold,
          state.rollTargetDeg,
          props.events)
      + MakeRollHoldStatusRow(props)
      + MakeRollHoldParametersFoldOut(state, props.events);

  return ui::ToggleFoldOut("Roll Hold", state.rollHold)
      .Id("RollHoldSection")
      .DefaultOpen()
      .OnChanged([events = props.events](bool enabled) {
        events.Emit({PrimaryRollHoldField::Enabled, enabled ? 1.0 : 0.0});
      })[content];
}

} // namespace

void AutopilotPanel::Draw(const AutopilotPanelProps &props) {
  const ui::UIElement layout = ui::VerticalLayout().Spacing(8.0F)
                               + ui::Heading("Autopilot Controls")
                               + MakeRollHoldSection(props);
  layout.Render();
}
} // namespace gui
