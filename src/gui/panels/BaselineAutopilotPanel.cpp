#include "gui/panels/BaselineAutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

#include <cmath>
#include <string>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotParameterLabelWidth = 112.0F;
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;

UI::UIElement MakePx4ParameterEditor(const char *editorId, double value,
    BaselineRollHoldField field,
    architecture::EventSink<BaselineRollHoldValueChanged> events,
    double minimum, double maximum) {
  return UI::ScalarEditor(editorId, value)
      .Range(minimum, maximum)
      .Step(0.01)
      .FastStep(0.1)
      .Format("%.2f")
      .OnChanged([events, field](double newValue) {
        if (std::isfinite(newValue)) {
          events.Emit({field, newValue});
        }
      });
}

UI::UIElement MakeBaselineRollHoldTuning(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::PropertyGridBuilder parameters =
      UI::PropertyGrid("BaselinePx4RollHoldParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  parameters
      .Add(UI::PropertyRow(
          "FW_R_TC")[MakePx4ParameterEditor("Px4RollTimeConstant",
          state.px4RollTimeConstantSec,
          BaselineRollHoldField::TimeConstantSec,
          props.valueEvents,
          0.01,
          1.0)])
      .Add(UI::PropertyRow(
          "FW_R_RMAX")[MakePx4ParameterEditor("Px4RollMaximumRate",
          state.px4RollMaximumRateDegPerSec,
          BaselineRollHoldField::MaximumRateDegPerSec,
          props.valueEvents,
          10.0,
          180.0)])
      .Add(UI::PropertyRow("FW_RR_P")[MakePx4ParameterEditor("Px4RollRateP",
          state.px4RollRateProportionalGain,
          BaselineRollHoldField::RateProportionalGain,
          props.valueEvents,
          0.005,
          0.5)])
      .Add(UI::PropertyRow("FW_RR_I")[MakePx4ParameterEditor("Px4RollRateI",
          state.px4RollRateIntegralGain,
          BaselineRollHoldField::RateIntegralGain,
          props.valueEvents,
          0.005,
          0.5)])
      .Add(UI::PropertyRow("FW_RR_D")[MakePx4ParameterEditor("Px4RollRateD",
          state.px4RollRateDerivativeGain,
          BaselineRollHoldField::RateDerivativeGain,
          props.valueEvents,
          0.0,
          0.5)])
      .Add(UI::PropertyRow("FW_RR_FF")[MakePx4ParameterEditor("Px4RollRateFF",
          state.px4RollRateFeedForwardGain,
          BaselineRollHoldField::RateFeedForwardGain,
          props.valueEvents,
          0.0,
          6.0)])
      .Add(UI::PropertyRow(
          "FW_RR_IMAX")[MakePx4ParameterEditor("Px4RollIntegratorLimit",
          state.px4RollIntegratorLimit,
          BaselineRollHoldField::IntegratorLimit,
          props.valueEvents,
          0.0,
          1.0)]);

  return UI::FoldOut("PX4 Roll Hold Tuning")
      .Open(state.px4RollTuningOpen)
      .Section()
      .Id("BaselineRollHoldTuning")[UI::VerticalLayout().Spacing(
          6.0F)[+UI::TextWrapped(
                    "PX4 v1.17 Roll Hold parameters. Time constants are in "
                    "seconds; rates are in deg/s.")
                + parameters
                + UI::Button("Reset PX4 Roll Hold Tuning")
                    .OnAction(
                        [events = props.resetEvents] { events.Emit({}); })]];
}

UI::UIElement MakeBaselineRollHoldDiagnostics(
    const BaselineAutopilotPanelProps &props);

UI::UIElement MakeBaselineRollHoldSection(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::ToggleFoldOutBuilder foldOut =
      UI::ToggleFoldOut("Roll Hold", state.rollHold)
          .Id("BaselineRollHoldSection")
          .OnChanged([events = props.valueEvents](bool enabled) {
            events.Emit({BaselineRollHoldField::Enabled, enabled ? 1.0 : 0.0});
          })
          .DefaultOpen();

  // clang-format off
  return foldOut[
      UI::VerticalLayout()
          .Spacing(6.0F)
          [
            +UI::HorizontalLayout()
                 .Spacing(8.0F)
                 [
                   +UI::TextDisabled("Target Roll (deg)")
                   + UI::InputDouble("##BaselineRollHoldTarget",
                         state.rollTargetDeg)
                         .Width(AutopilotTargetInputWidth)
                         .Step(1.0)
                         .FastStep(10.0)
                         .Format("%.2f")
                         .OnChanged([events = props.valueEvents](double value) {
                           events.Emit(
                               {BaselineRollHoldField::TargetDeg, value});
                         })
                   + UI::Text(state.rollHold ? "Hold" : "Off")
                 ]
            + UI::HorizontalLayout()
                 .Spacing(8.0F)
                 [
                   +UI::ValueLabel("Current Roll",
                        props.currentRollDeg,
                        "%.2f deg")
                   + UI::ValueLabel("Roll Rate",
                         props.currentRollRateDegPerSec,
                         "%.2f deg/s")
                   + UI::ValueLabel("Aileron",
                         props.currentAileron,
                         "%.3f")
                   + UI::StatusBadge(props.rollHoldActive ? "Active" : "Inactive",
                         props.rollHoldActive ? UI::StatusTone::Success
                                              : UI::StatusTone::Neutral)
                   + UI::Button("Capture")
                         .Enabled(props.valueEvents.IsConnected())
                         .OnAction([events = props.valueEvents,
                                      value = props.currentRollDeg] {
                           events.Emit(
                               {BaselineRollHoldField::TargetDeg, value});
                         })
                         .Width(HoldCaptureButtonWidth)
                 ]
            + MakeBaselineRollHoldTuning(props)
            + MakeBaselineRollHoldDiagnostics(props)
          ]
      ];
  // clang-format on
}

UI::UIElement MakeBaselineRollHoldDiagnostics(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;

  // clang-format off
  return UI::FoldOut("Diagnostics")
      .Open(state.px4RollDiagnosticsOpen)
      .Section()
      .Id("BaselineRollHoldDiagnostics")
      [
        UI::VerticalLayout()
            .Spacing(6.0F)
            [
              +UI::TextWrapped(
                    "PX4 v1.17 fixed-wing Roll Hold state for the Baseline "
                    "simulation.")
              + UI::KeyValueGrid("BaselineRollHoldDiagnosticValues")
                    .ColumnsPerRow(2)
                    .AddDouble("PX4 Aileron",
                         props.px4RollAileronCommand,
                         "%.3f")
                    .AddDouble("PX4 Roll Error",
                         props.px4RollErrorDeg,
                         "%.2f deg")
                    .AddDouble("PX4 Rate SP",
                         props.px4RollRateSetpointDegPerSec,
                         "%.2f deg/s")
                    .AddDouble("Airspeed Scale",
                         props.px4AirspeedScaling,
                         "%.3f")
            ]
      ];
  // clang-format on
}
} // namespace

void BaselineAutopilotPanel::Draw(const BaselineAutopilotPanelProps &props) {
  const UI::UIElement layout = UI::VerticalLayout().Spacing(8.0F)
                               + UI::Heading("Autopilot Controls")
                               + MakeBaselineRollHoldSection(props);
  layout.Render();
}
} // namespace gui
