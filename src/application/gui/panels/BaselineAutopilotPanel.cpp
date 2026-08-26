#include "application/gui/panels/BaselineAutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotParameterIndent = 24.0F;
constexpr float AutopilotParameterSliderWidth = 240.0F;
constexpr float AutopilotParameterInputWidth = 88.0F;
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float ControllerHeaderCheckboxWidth = 18.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;
constexpr double DefaultPx4RollTimeConstantSec = 0.35;
constexpr double DefaultPx4RollMaximumRateDegPerSec = 70.0;
constexpr double DefaultPx4RollRateProportionalGain = 0.160;
constexpr double DefaultPx4RollRateIntegralGain = 0.080;
constexpr double DefaultPx4RollRateDerivativeGain = 0.0;
constexpr double DefaultPx4RollRateFeedForwardGain = 0.80;
constexpr double DefaultPx4RollIntegratorLimit = 0.15;

UI::UIElement MakeAutopilotParameterSlider(const char *label,
    const char *sliderId, double &value, double minimum, double maximum) {
  const std::string inputId = std::string(sliderId) + "Value";
  const auto setValue = [&value, minimum, maximum](double newValue) {
    if (std::isfinite(newValue)) {
      value = std::clamp(newValue, minimum, maximum);
    }
  };

  return UI::HorizontalLayout().Spacing(
      8.0F)[+UI::HorizontalSpace(AutopilotParameterIndent)
            + UI::TextDisabled(label)
            + UI::SliderDouble(sliderId, value, minimum, maximum)
                .Width(AutopilotParameterSliderWidth)
                .Format("%.2f")
                .OnChanged(setValue)
            + UI::InputDouble(inputId, value)
                .Width(AutopilotParameterInputWidth)
                .Step(0.01)
                .FastStep(0.1)
                .Format("%.2f")
                .OnChanged(setValue)];
}

void ResetPx4RollTuning(BaselineAutopilotPanelState &state) {
  state.px4RollTimeConstantSec = DefaultPx4RollTimeConstantSec;
  state.px4RollMaximumRateDegPerSec = DefaultPx4RollMaximumRateDegPerSec;
  state.px4RollRateProportionalGain = DefaultPx4RollRateProportionalGain;
  state.px4RollRateIntegralGain = DefaultPx4RollRateIntegralGain;
  state.px4RollRateDerivativeGain = DefaultPx4RollRateDerivativeGain;
  state.px4RollRateFeedForwardGain = DefaultPx4RollRateFeedForwardGain;
  state.px4RollIntegratorLimit = DefaultPx4RollIntegratorLimit;
}

UI::UIElement MakeBaselineRollHoldTuning(BaselineAutopilotPanelState &state) {
  return UI::FoldOut("PX4 Roll Hold Tuning")
      .Open(state.px4RollTuningOpen)
      .SpanAvailWidth()
      .Id("BaselineRollHoldTuning")[UI::VerticalLayout().Spacing(
          6.0F)[+UI::TextWrapped("PX4 v1.17 Roll Hold parameters.")
                + MakeAutopilotParameterSlider("FW_R_TC (s)",
                    "##Px4RollTimeConstant",
                    state.px4RollTimeConstantSec,
                    0.2,
                    2.0)
                + MakeAutopilotParameterSlider("FW_R_RMAX (deg/s)",
                    "##Px4RollMaximumRate",
                    state.px4RollMaximumRateDegPerSec,
                    10.0,
                    180.0)
                + MakeAutopilotParameterSlider("FW_RR_P",
                    "##Px4RollRateP",
                    state.px4RollRateProportionalGain,
                    0.0,
                    0.5)
                + MakeAutopilotParameterSlider("FW_RR_I",
                    "##Px4RollRateI",
                    state.px4RollRateIntegralGain,
                    0.0,
                    0.5)
                + MakeAutopilotParameterSlider("FW_RR_D",
                    "##Px4RollRateD",
                    state.px4RollRateDerivativeGain,
                    0.0,
                    0.5)
                + MakeAutopilotParameterSlider("FW_RR_FF",
                    "##Px4RollRateFF",
                    state.px4RollRateFeedForwardGain,
                    0.0,
                    1.5)
                + MakeAutopilotParameterSlider("FW_RR_IMAX",
                    "##Px4RollIntegratorLimit",
                    state.px4RollIntegratorLimit,
                    0.0,
                    1.0)
                + UI::Button("Reset PX4 Roll Hold Tuning").OnAction([&state] {
                    ResetPx4RollTuning(state);
                  })]];
}

UI::UIElement MakeBaselineRollHoldDiagnostics(
    const BaselineAutopilotPanelProps &props);

UI::UIElement MakeBaselineRollHoldSection(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::FoldOutBuilder foldOut =
      UI::FoldOut("Roll Hold")
          .Framed()
          .SpanAvailWidth()
          .Id("BaselineRollHoldSection")
          .HeaderLeft(UI::Toggle("##Enabled", state.rollHold)
                          .Id("BaselineRollHoldEnabled")
                          .OnChanged([&state](bool enabled) {
                            state.rollHold = enabled;
                          }),
              ControllerHeaderCheckboxWidth)
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
                         .OnChanged([&state](double value) {
                           state.rollTargetDeg = value;
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
                   + UI::Text(props.rollHoldActive ? "Active" : "Inactive")
                   + UI::Button("Capture")
                         .Enabled(static_cast<bool>(props.captureCurrentRoll))
                         .OnAction(props.captureCurrentRoll)
                         .Width(HoldCaptureButtonWidth)
                 ]
            + MakeBaselineRollHoldTuning(state)
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
      .SpanAvailWidth()
      .Id("BaselineRollHoldDiagnostics")
      [
        UI::VerticalLayout()
            .Spacing(6.0F)
            [
              +UI::TextWrapped(
                    "PX4 v1.17 fixed-wing Roll Hold state for the Baseline "
                    "simulation.")
              + UI::ValueLabel("PX4 Aileron",
                    props.px4RollAileronCommand,
                    "%.3f")
              + UI::HorizontalLayout()
                   .Spacing(8.0F)
                   [
                     +UI::ValueLabel("PX4 Roll Error",
                          props.px4RollErrorDeg,
                          "%.2f deg")
                     + UI::ValueLabel("PX4 Rate SP",
                           props.px4RollRateSetpointDegPerSec,
                           "%.2f deg/s")
                     + UI::ValueLabel("Airspeed Scale",
                           props.px4AirspeedScaling,
                           "%.3f")
                   ]
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
