#include "application/gui/panels/ManualControlPanel.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/control/ManualFlightControlController.hpp"
#include "flightui/FlightUI.hpp"

#include <cmath>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr double ManualInputStep = 0.05;
constexpr float ManualInputLayoutSpacing = 6.0F;
constexpr float ManualInputRowSpacing = 8.0F;
constexpr float ManualInputButtonWidth = 32.0F;
constexpr float ManualInputSliderWidth = 240.0F;
constexpr float ManualInputValueWidth = 88.0F;

bool WasShortcutPressed(UI::Key key) { return UI::IsKeyPressed(key, true); }

bool CanApplyManualInputShortcuts() {
  return UI::IsCurrentWindowFocused() && !UI::WantsTextInput();
}

bool IsManualControlAllowed(const AutopilotPanelState &autopilotState,
    control::ControlAxis axis) {
  switch (axis) {
  case control::ControlAxis::Elevator:
    return true;
  case control::ControlAxis::Aileron:
    return !autopilotState.rollHold;
  case control::ControlAxis::Rudder:
    return true;
  case control::ControlAxis::Throttle:
    return true;
  }

  return true;
}

const char *ManualControlLockTooltip(const AutopilotPanelState &autopilotState,
    control::ControlAxis axis) {
  switch (axis) {
  case control::ControlAxis::Elevator:
    break;
  case control::ControlAxis::Aileron:
    if (autopilotState.rollHold) {
      return "Roll Hold is controlling aileron.";
    }
    break;
  case control::ControlAxis::Rudder:
    break;
  case control::ControlAxis::Throttle:
    break;
  }

  return "";
}

void AdjustManualInput(control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState, control::ControlAxis axis,
    double delta) {
  if (!IsManualControlAllowed(autopilotState, axis)) {
    return;
  }

  manualController.AdjustCommandedInput(axis, delta);
}

void SetManualInput(control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState, control::ControlAxis axis,
    double value) {
  if (!IsManualControlAllowed(autopilotState, axis) || !std::isfinite(value)) {
    return;
  }

  manualController.SetCommandedInput(axis, value);
}

void ApplyManualInputShortcuts(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState) {
  if (!CanApplyManualInputShortcuts()) {
    return;
  }

  if (WasShortcutPressed(UI::Key::F)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Throttle,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::R)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Throttle,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::W)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Elevator,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::S)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Elevator,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::A)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Aileron,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::D)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Aileron,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::Q)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Rudder,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::E)) {
    AdjustManualInput(manualController,
        autopilotState,
        control::ControlAxis::Rudder,
        ManualInputStep);
  }
}

UI::UIElement MakeThrottleRow(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Throttle);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Throttle);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Throttle")
        + UI::Button("F")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##ThrottleInput", input.throttle, 0.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::InputDouble("##ThrottleInputValue", input.throttle)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.3f")
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    value);
              })
              .Width(ManualInputValueWidth)
        + UI::Button("R")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeElevatorRow(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Elevator);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Elevator);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Elevator")
        + UI::Button("W")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##ElevatorInput", input.elevator, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::InputDouble("##ElevatorInputValue", input.elevator)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.3f")
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    value);
              })
              .Width(ManualInputValueWidth)
        + UI::Button("S")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAileronRow(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Aileron);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Aileron);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Aileron")
        + UI::Button("A")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##AileronInput", input.aileron, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::InputDouble("##AileronInputValue", input.aileron)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.3f")
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    value);
              })
              .Width(ManualInputValueWidth)
        + UI::Button("D")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeRudderRow(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Rudder);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Rudder);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Rudder")
        + UI::Button("Q")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##RudderInput", input.rudder, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::InputDouble("##RudderInputValue", input.rudder)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .Step(0.01)
              .FastStep(0.1)
              .Format("%.3f")
              .OnChanged([&manualController, &autopilotState](double value) {
                SetManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    value);
              })
              .Width(ManualInputValueWidth)
        + UI::Button("E")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&manualController, &autopilotState] {
                AdjustManualInput(manualController,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeManualInputLayout(
    control::ManualFlightControlController &manualController,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input, double pitchTrim) {
  // clang-format off
  return UI::VerticalLayout()
      .Spacing(ManualInputLayoutSpacing)
      [
        +UI::Text("Control Inputs")
        + MakeThrottleRow(manualController, autopilotState, input)
        + MakeElevatorRow(manualController, autopilotState, input)
        + MakeAileronRow(manualController, autopilotState, input)
        + MakeRudderRow(manualController, autopilotState, input)
        + UI::ValueLabel("Pitch Trim", pitchTrim, "%.3f")
      ];
  // clang-format on
}
} // namespace

void ManualControlPanel::Draw(
    control::ManualFlightControlController &manualController,
    const sim::Aircraft &aircraft,
    const AutopilotPanelState &autopilotState) {
  ApplyManualInputShortcuts(manualController, autopilotState);
  MakeManualInputLayout(manualController,
      autopilotState,
      manualController.GetCommandedInput(),
      aircraft.GetControls().GetPitchTrim())
      .Render();
}
} // namespace gui
