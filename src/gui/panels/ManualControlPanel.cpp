#include "gui/panels/ManualControlPanel.hpp"

#include "flightui/FlightUI.hpp"

#include <cmath>

namespace gui {

namespace {
constexpr double ManualInputStep = 0.05;
constexpr float ManualInputLayoutSpacing = 6.0F;
constexpr float ManualInputRowSpacing = 8.0F;
constexpr float ManualInputButtonWidth = 32.0F;

bool WasShortcutPressed(ui::Key key) { return ui::IsKeyPressed(key, true); }

bool CanApplyManualInputShortcuts() {
  return ui::IsCurrentWindowFocused() && !ui::WantsTextInput();
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

void AdjustManualInput(ManualControlPanelProps &props,
    control::ControlAxis axis, double delta) {
  if (!IsManualControlAllowed(props.autopilotState, axis)) {
    return;
  }

  control::ControlInput input = props.input;
  if (control::AdjustControlAxisValue(input, axis, delta)
      && props.events.IsConnected()) {
    props.events.Emit({input});
    props.input = input;
  }
}

void SetManualInput(ManualControlPanelProps &props, control::ControlAxis axis,
    double value) {
  if (!IsManualControlAllowed(props.autopilotState, axis)
      || !std::isfinite(value)) {
    return;
  }

  control::ControlInput input = props.input;
  if (control::SetControlAxisValue(input, axis, value)
      && props.events.IsConnected()) {
    props.events.Emit({input});
    props.input = input;
  }
}

void ApplyManualInputShortcuts(ManualControlPanelProps &props) {
  if (!CanApplyManualInputShortcuts()) {
    return;
  }

  if (WasShortcutPressed(ui::Key::F)) {
    AdjustManualInput(props, control::ControlAxis::Throttle, -ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::R)) {
    AdjustManualInput(props, control::ControlAxis::Throttle, ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::W)) {
    AdjustManualInput(props, control::ControlAxis::Elevator, -ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::S)) {
    AdjustManualInput(props, control::ControlAxis::Elevator, ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::A)) {
    AdjustManualInput(props, control::ControlAxis::Aileron, -ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::D)) {
    AdjustManualInput(props, control::ControlAxis::Aileron, ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::Q)) {
    AdjustManualInput(props, control::ControlAxis::Rudder, -ManualInputStep);
  }
  if (WasShortcutPressed(ui::Key::E)) {
    AdjustManualInput(props, control::ControlAxis::Rudder, ManualInputStep);
  }
}

ui::UIElement MakeManualScalarEditor(const char *id, double value,
    double minimum, double maximum, ManualControlPanelProps &props,
    control::ControlAxis axis, bool enabled, const char *tooltip) {
  return ui::ScalarEditor(id, value)
      .Range(minimum, maximum)
      .Step(0.01)
      .FastStep(0.1)
      .Format("%.3f")
      .TrailingWidth(ManualInputButtonWidth + ManualInputRowSpacing)
      .Enabled(enabled)
      .Tooltip(tooltip)
      .OnChanged([&props, axis](
                     double changed) { SetManualInput(props, axis, changed); });
}

ui::UIElement MakeThrottleRow(ManualControlPanelProps &props) {
  const bool enabled = IsManualControlAllowed(props.autopilotState,
      control::ControlAxis::Throttle);
  const char *tooltip = ManualControlLockTooltip(props.autopilotState,
      control::ControlAxis::Throttle);

  // clang-format off
  return ui::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +ui::Text("Throttle")
        + ui::Button("F")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Throttle, -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + MakeManualScalarEditor("ThrottleInput",
              props.input.throttle,
              0.0,
              1.0,
              props,
              control::ControlAxis::Throttle,
              enabled,
              tooltip)
        + ui::Button("R")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Throttle, ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

ui::UIElement MakeElevatorRow(ManualControlPanelProps &props) {
  const bool enabled = IsManualControlAllowed(props.autopilotState,
      control::ControlAxis::Elevator);
  const char *tooltip = ManualControlLockTooltip(props.autopilotState,
      control::ControlAxis::Elevator);

  // clang-format off
  return ui::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +ui::Text("Elevator")
        + ui::Button("W")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Elevator, -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + MakeManualScalarEditor("ElevatorInput",
              props.input.elevator,
              -1.0,
              1.0,
              props,
              control::ControlAxis::Elevator,
              enabled,
              tooltip)
        + ui::Button("S")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Elevator, ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

ui::UIElement MakeAileronRow(ManualControlPanelProps &props) {
  const bool enabled = IsManualControlAllowed(props.autopilotState,
      control::ControlAxis::Aileron);
  const char *tooltip = ManualControlLockTooltip(props.autopilotState,
      control::ControlAxis::Aileron);

  // clang-format off
  return ui::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +ui::Text("Aileron")
        + ui::Button("A")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Aileron, -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + MakeManualScalarEditor("AileronInput",
              props.input.aileron,
              -1.0,
              1.0,
              props,
              control::ControlAxis::Aileron,
              enabled,
              tooltip)
        + ui::Button("D")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Aileron, ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

ui::UIElement MakeRudderRow(ManualControlPanelProps &props) {
  const bool enabled = IsManualControlAllowed(props.autopilotState,
      control::ControlAxis::Rudder);
  const char *tooltip = ManualControlLockTooltip(props.autopilotState,
      control::ControlAxis::Rudder);

  // clang-format off
  return ui::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +ui::Text("Rudder")
        + ui::Button("Q")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Rudder, -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + MakeManualScalarEditor("RudderInput",
              props.input.rudder,
              -1.0,
              1.0,
              props,
              control::ControlAxis::Rudder,
              enabled,
              tooltip)
        + ui::Button("E")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&props] {
                AdjustManualInput(
                    props, control::ControlAxis::Rudder, ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

ui::UIElement MakeManualInputLayout(ManualControlPanelProps &props) {
  // clang-format off
  return ui::VerticalLayout()
      .Spacing(ManualInputLayoutSpacing)
      [
        +ui::Text("Control Inputs")
        + MakeThrottleRow(props)
        + MakeElevatorRow(props)
        + MakeAileronRow(props)
        + MakeRudderRow(props)
        + ui::ValueLabel("Pitch Trim", props.pitchTrim, "%.3f")
      ];
  // clang-format on
}
} // namespace

void ManualControlPanel::Draw(const ManualControlPanelProps &props) {
  ManualControlPanelProps editableProps = props;
  ApplyManualInputShortcuts(editableProps);
  MakeManualInputLayout(editableProps).Render();
}
} // namespace gui
