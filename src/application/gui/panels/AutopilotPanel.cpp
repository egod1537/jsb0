#include "application/gui/panels/AutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AutopilotParameterIndent = 24.0F;
constexpr float AutopilotParameterSliderWidth = 240.0F;
constexpr float AutopilotParameterInputWidth = 88.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;
constexpr float ControllerHeaderCheckboxWidth = 18.0F;

struct AxisHoldSectionConfig {
  const char *holdLabel = "";
  const char *sectionId = "";
  const char *targetLabel = "";
  const char *targetInputId = "";
  bool &enabled;
  double &targetValue;
  const char *currentLabel = "";
  double currentValue = 0.0;
  const char *rateLabel = "";
  double rateValue = 0.0;
  const char *outputLabel = "";
  double outputValue = 0.0;
  bool active = false;
  bool preparing = false;
  const std::function<void()> &captureCurrent;
  const char *responseLabel = "";
  const char *responseId = "";
  const char *dampingRatioSliderId = "";
  double &dampingRatio;
  const char *naturalFrequencySliderId = "";
  double &naturalFrequencyRadPerSec;
  double minimumNaturalFrequencyRadPerSec = 0.1;
  bool &responseOpen;
  bool responseEditableWhenDisabled = false;
  bool responseVisible = true;
};

UI::UIElement MakeAutopilotTargetRow(const char *targetLabel,
    const char *inputId, bool &enabled, double &targetValue, double step = 1.0,
    double fastStep = 10.0) {
  return UI::HorizontalLayout().Spacing(
      8.0F)[+UI::TextDisabled(targetLabel)
            + UI::InputDouble(inputId, targetValue)
                .Width(AutopilotTargetInputWidth)
                .Step(step)
                .FastStep(fastStep)
                .Format("%.2f")
                .OnChanged(
                    [&targetValue](double value) { targetValue = value; })
            + UI::Text(enabled ? "Hold" : "Off")];
}

UI::FoldOutBuilder MakeControllerFoldOut(const char *label,
    const char *sectionId, bool &enabled, const char *enabledToggleId,
    bool defaultOpen = true) {
  UI::FoldOutBuilder foldOut =
      UI::FoldOut(label).Framed().SpanAvailWidth().Id(sectionId).HeaderLeft(
          UI::Toggle("##Enabled", enabled)
              .Id(enabledToggleId)
              .OnChanged([&enabled](bool value) { enabled = value; }),
          ControllerHeaderCheckboxWidth);
  if (defaultOpen) {
    foldOut.DefaultOpen();
  }
  return foldOut;
}

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

UI::UIElement MakeAxisHoldStatusRow(const AxisHoldSectionConfig &config) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(8.0F)
      [
        +UI::ValueLabel(config.currentLabel, config.currentValue, "%.2f deg")
        + UI::ValueLabel(config.rateLabel, config.rateValue, "%.2f deg/s")
        + UI::ValueLabel(config.outputLabel, config.outputValue, "%.3f")
        + UI::Text(config.active
                       ? "Active"
                       : (config.preparing ? "Preparing" : "Inactive"))
        + UI::Button("Capture")
              .Enabled(static_cast<bool>(config.captureCurrent))
              .OnAction(config.captureCurrent)
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAxisHoldResponseFoldOut(const AxisHoldSectionConfig &config) {
  return UI::FoldOut(config.responseLabel)
      .Open(config.responseOpen)
      .SpanAvailWidth()
      .Id(config.responseId)[UI::VerticalLayout().Spacing(
          6.0F)[+MakeAutopilotParameterSlider("Damping Ratio",
                    config.dampingRatioSliderId,
                    config.dampingRatio,
                    0.1,
                    2.0)
                + MakeAutopilotParameterSlider("Natural Frequency (rad/s)",
                    config.naturalFrequencySliderId,
                    config.naturalFrequencyRadPerSec,
                    config.minimumNaturalFrequencyRadPerSec,
                    10.0)]];
}

UI::UIElement MakeAxisHoldSection(const AxisHoldSectionConfig &config) {
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotTargetRow(config.targetLabel,
          config.targetInputId,
          config.enabled,
          config.targetValue);

  layout = layout + MakeAxisHoldStatusRow(config);

  if (config.responseVisible
      && (config.enabled || config.responseEditableWhenDisabled)) {
    layout = layout + MakeAxisHoldResponseFoldOut(config);
  }

  const std::string enabledToggleId = std::string(config.sectionId) + "Enabled";
  return MakeControllerFoldOut(config.holdLabel,
      config.sectionId,
      config.enabled,
      enabledToggleId.c_str())[layout];
}

UI::UIElement MakeRollHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  return MakeAxisHoldSection({
      .holdLabel = "Roll Hold",
      .sectionId = "RollHoldSection",
      .targetLabel = "Target Roll (deg)",
      .targetInputId = "##RollHoldTarget",
      .enabled = state.rollHold,
      .targetValue = state.rollTargetDeg,
      .currentLabel = "Current Roll",
      .currentValue = props.currentRollDeg,
      .rateLabel = "Roll Rate",
      .rateValue = props.currentRollRateDegPerSec,
      .outputLabel = "Aileron",
      .outputValue = props.currentAileron,
      .active = props.rollHoldActive,
      .preparing = props.rollHoldPreparing,
      .captureCurrent = props.captureCurrentRoll,
      .responseLabel = "Roll Hold Response",
      .responseId = "RollHoldResponse",
      .dampingRatioSliderId = "##RollHoldDampingRatio",
      .dampingRatio = state.rollHoldDampingRatio,
      .naturalFrequencySliderId = "##RollHoldNaturalFrequency",
      .naturalFrequencyRadPerSec = state.rollHoldNaturalFrequencyRadPerSec,
      .responseOpen = state.rollHoldResponseOpen,
      .responseEditableWhenDisabled = true,
  });
}

} // namespace

void AutopilotPanel::Draw(AutopilotPanelState &state) {
  Draw({.state = state});
}

void AutopilotPanel::Draw(const AutopilotPanelProps &props) {
  const UI::UIElement layout = UI::VerticalLayout().Spacing(8.0F)
                               + UI::Heading("Autopilot Controls")
                               + MakeRollHoldSection(props);
  layout.Render();
}
} // namespace gui
