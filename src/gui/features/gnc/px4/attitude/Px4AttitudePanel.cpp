#include "gui/features/gnc/px4/attitude/Px4AttitudePanel.hpp"

#include "gui/panels/BaselineAutopilotPanel.hpp"

#include "gui/features/gnc/components/ParameterEditor.hpp"
#include "flightui/FlightUI.hpp"

#include <algorithm>
#include <string>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotParameterLabelWidth = 112.0F;
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AdaptivePropertyInputWidth = 0.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;

UI::PropertyRowBuilder RenderPx4ParameterRow(
    const BaselinePx4RollHoldParameterBinding &binding,
    const BaselineAutopilotPanelState &state,
    architecture::EventSink<BaselineRollHoldValueChanged> events) {
  const auto &metadata =
      gnc::GetPx4RollHoldParameterMetadata(binding.parameter);
  return RenderGncParameterRow(metadata,
      "Editor",
      state.*(binding.value),
      [events, field = binding.field](
          double value) { events.Emit({field, value}); });
}

UI::PropertyRowBuilder RenderPx4CourseParameterRow(
    const BaselinePx4CourseHoldParameterBinding &binding,
    const BaselineAutopilotPanelState &state,
    architecture::EventSink<BaselineRollHoldValueChanged> events) {
  const auto &metadata =
      gnc::GetPx4CourseHoldParameterMetadata(binding.parameter);
  return RenderGncParameterRow(metadata,
      "CourseEditor",
      state.*(binding.value),
      [events, field = binding.field](
          double value) { events.Emit({field, value}); });
}

UI::PropertyRowBuilder RenderPx4PitchParameterRow(
    const BaselinePx4PitchHoldParameterBinding &binding,
    const BaselineAutopilotPanelState &state,
    architecture::EventSink<BaselineRollHoldValueChanged> events) {
  const auto &metadata =
      gnc::GetPx4PitchHoldParameterMetadata(binding.parameter);
  return RenderGncParameterRow(metadata,
      "PitchEditor",
      state.*(binding.value),
      [events, field = binding.field](
          double value) { events.Emit({field, value}); });
}

UI::PropertyRowBuilder RenderYawParameterRow(
    const BaselinePx4YawRateParameterBinding &binding,
    const BaselineAutopilotPanelState &state,
    architecture::EventSink<BaselineRollHoldValueChanged> events) {
  const auto &metadata = gnc::GetPx4YawRateParameterMetadata(binding.parameter);
  return RenderGncParameterRow(metadata,
      "YawEditor",
      state.*(binding.value),
      [events, field = binding.field](
          double value) { events.Emit({field, value}); });
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
  for (const BaselinePx4RollHoldParameterBinding &binding :
      BaselinePx4RollHoldParameterBindings) {
    parameters.Add(RenderPx4ParameterRow(binding, state, props.valueEvents));
  }

  const auto &maximumRateMetadata = gnc::GetPx4RollHoldParameterMetadata(
      gnc::Px4RollHoldParameter::MaximumRollRate);
  UI::PropertyGridBuilder directRateInput =
      UI::PropertyGrid("BaselineDirectRollRateTestParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  directRateInput.Add(UI::PropertyRow("Roll Rate Command")
          .Tooltip("Direct body roll-rate command in deg/s. The active "
                   "FW_R_RMAX limit is still applied.")[UI::ScalarEditor(
              "DirectRollRateCommand",
              state.directRollRateCommandDegPerSec)
                  .Range(-maximumRateMetadata.maximum,
                      maximumRateMetadata.maximum)
                  .ShowSlider(false)
                  .ShowStepper()
                  .Step(0.5)
                  .FastStep(5.0)
                  .Format("%.2f deg/s")
                  .InputWidth(AdaptivePropertyInputWidth)
                  .OnChanged([events = props.valueEvents](double value) {
                    events.Emit(
                        {BaselineRollHoldField::DirectRollRateCommandDegPerSec,
                            value});
                  })]);

  return UI::FoldOut("PX4 Roll Hold Tuning")
      .Open(state.px4RollTuningOpen)
      .Section()
      .Id("BaselineRollHoldTuning")[UI::VerticalLayout().Spacing(
          6.0F)[+UI::TextWrapped(
                    "PX4 v1.17 Roll Hold parameters. Time constants are in "
                    "seconds; rates are in deg/s.")
                + parameters
                + UI::Button("Reset PX4 Roll Hold Tuning")
                    .OnAction([events = props.resetEvents] { events.Emit({}); })
                + UI::Separator()
                + UI::TextWrapped(
                    "DEBUG / TUNING ONLY: bypasses the roll-angle outer "
                    "loop while preserving the PX4 rate controller and "
                    "rate limit.")
                + UI::Toggle("Direct Roll Rate Test",
                    state.directRollRateTestEnabled)
                    .Tooltip("Use the direct roll-rate command instead of "
                             "the roll-angle controller output.")
                    .OnChanged([events = props.valueEvents](bool enabled) {
                      events.Emit(
                          {BaselineRollHoldField::DirectRollRateTestEnabled,
                              enabled ? 1.0 : 0.0});
                    })
                + directRateInput]];
}

UI::UIElement MakeBaselineYawControlSection(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::PropertyGridBuilder yawParameters =
      UI::PropertyGrid("BaselinePx4YawRateParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  for (const BaselinePx4YawRateParameterBinding &binding :
      BaselinePx4YawRateParameterBindings) {
    yawParameters.Add(RenderYawParameterRow(binding, state, props.valueEvents));
  }

  UI::ToggleFoldOutBuilder foldOut =
      UI::ToggleFoldOut("Yaw Coordination / Damper",
          state.yawRateControlEnabled)
          .Id("BaselineYawControlSection")
          .OnChanged([events = props.valueEvents](bool enabled) {
            events.Emit({BaselineRollHoldField::YawRateControlEnabled,
                enabled ? 1.0 : 0.0});
          });

  return foldOut[UI::VerticalLayout().Spacing(
      6.0F)[+UI::TextWrapped(
                "EXPERIMENTAL: independent PX4 yaw-rate control with JSB0 "
                "sideslip feedback. Validated candidate: P=0.8, K_beta=8, "
                "I/D/FF=0. Disabled by default.")
            + UI::Toggle("Coordinated Turn Setpoint",
                state.coordinatedTurnEnabled)
                .Tooltip("ON: add g/V*sin(phi)*cos(theta). OFF: damping test "
                         "with a zero base yaw-rate setpoint.")
                .OnChanged([events = props.valueEvents](bool enabled) {
                  events.Emit({BaselineRollHoldField::CoordinatedTurnEnabled,
                      enabled ? 1.0 : 0.0});
                })
            + yawParameters]];
}

UI::UIElement MakeBaselineRollHoldDiagnostics(
    const BaselineAutopilotPanelProps &props);

UI::UIElement MakeBaselinePitchHoldTuning(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::PropertyGridBuilder parameters =
      UI::PropertyGrid("BaselinePx4PitchHoldParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  for (const BaselinePx4PitchHoldParameterBinding &binding :
      BaselinePx4PitchHoldParameterBindings) {
    parameters.Add(
        RenderPx4PitchParameterRow(binding, state, props.valueEvents));
  }

  return UI::FoldOut("PX4 Pitch Hold Tuning")
      .Open(state.px4PitchTuningOpen)
      .Section()
      .Id("BaselinePitchHoldTuning")[UI::VerticalLayout().Spacing(
          6.0F)[+UI::TextWrapped(
                    "PX4 fixed-wing pitch attitude/rate cascade. Positive "
                    "and negative pitch-rate limits are independent.")
                + parameters
                + UI::Button("Reset PX4 Pitch Hold Tuning")
                    .OnAction([events = props.pitchResetEvents] {
                      events.Emit({});
                    })]];
}

UI::UIElement MakeBaselinePitchHoldDiagnostics(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  return UI::FoldOut("Diagnostics")
      .Open(state.px4PitchDiagnosticsOpen)
      .Section()
      .Id("BaselinePitchHoldDiagnostics")[UI::VerticalLayout().Spacing(
          6.0F)[+UI::TextWrapped(
                    "PX4 fixed-wing Pitch Hold state for the Baseline "
                    "simulation.")
                + UI::KeyValueGrid("BaselinePitchHoldDiagnosticValues")
                    .ColumnsPerRow(2)
                    .AddDouble("PX4 Elevator",
                        props.px4PitchElevatorCommand,
                        "%.3f")
                    .AddDouble("PX4 Pitch Error",
                        props.px4PitchErrorDeg,
                        "%.2f deg")
                    .AddDouble("PX4 Rate SP",
                        props.px4PitchRateSetpointDegPerSec,
                        "%.2f deg/s")
                    .AddDouble("Airspeed Scale",
                        props.px4PitchAirspeedScaling,
                        "%.3f")]];
}

UI::UIElement MakeBaselinePitchHoldSection(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::ToggleFoldOutBuilder foldOut =
      UI::ToggleFoldOut("Pitch Hold", state.pitchHold)
          .Id("BaselinePitchHoldSection")
          .OnChanged([events = props.valueEvents](bool enabled) {
            events.Emit(
                {BaselineRollHoldField::PitchHoldEnabled, enabled ? 1.0 : 0.0});
          });

  // clang-format off
  return foldOut[
      UI::VerticalLayout()
          .Spacing(6.0F)
          [
            +UI::HorizontalLayout()
                 .Spacing(8.0F)
                 [
                   +UI::TextDisabled("Target Pitch (deg)")
                   + UI::InputDouble("##BaselinePitchHoldTarget",
                         state.pitchTargetDeg)
                         .Width(AutopilotTargetInputWidth)
                         .Step(0.5)
                         .FastStep(5.0)
                         .Format("%.2f")
                         .OnChanged([events = props.valueEvents](double value) {
                           events.Emit(
                               {BaselineRollHoldField::TargetPitchDeg, value});
                         })
                   + UI::Text(state.pitchHold ? "Hold" : "Off")
                 ]
            + UI::HorizontalLayout()
                 .Spacing(8.0F)
                 [
                   +UI::ValueLabel("Current Pitch",
                        props.currentPitchDeg,
                        "%.2f deg")
                   + UI::ValueLabel("Pitch Rate",
                         props.currentPitchRateDegPerSec,
                         "%.2f deg/s")
                   + UI::ValueLabel("Elevator",
                         props.currentElevator,
                         "%.3f")
                   + UI::StatusBadge(props.pitchHoldActive ? "Active" : "Inactive",
                         props.pitchHoldActive ? UI::StatusTone::Success
                                               : UI::StatusTone::Neutral)
                   + UI::Button("Capture")
                         .Enabled(props.valueEvents.IsConnected())
                         .OnAction([events = props.valueEvents,
                                      value = props.currentPitchDeg] {
                           events.Emit(
                               {BaselineRollHoldField::TargetPitchDeg, value});
                         })
                         .Width(HoldCaptureButtonWidth)
                 ]
            + MakeBaselinePitchHoldTuning(props)
            + MakeBaselinePitchHoldDiagnostics(props)
          ]
      ];
  // clang-format on
}

UI::UIElement MakeBaselineCourseHoldSection(
    const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::PropertyGridBuilder parameters =
      UI::PropertyGrid("BaselinePx4CourseHoldParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  for (const BaselinePx4CourseHoldParameterBinding &binding :
      BaselinePx4CourseHoldParameterBindings) {
    parameters.Add(
        RenderPx4CourseParameterRow(binding, state, props.valueEvents));
  }

  UI::ToggleFoldOutBuilder foldOut =
      UI::ToggleFoldOut("Course Hold", state.courseHold)
          .Id("BaselineCourseHoldSection")
          .OnChanged([events = props.valueEvents](bool enabled) {
            events.Emit({BaselineRollHoldField::CourseHoldEnabled,
                enabled ? 1.0 : 0.0});
          });

  return foldOut[UI::VerticalLayout().Spacing(
      6.0F)[+UI::TextWrapped(
                "PX4 mainline directional-guidance subset. Produces only a "
                "roll setpoint; Roll Hold and yaw augmentation remain "
                "separate.")
            + UI::PropertyGrid("BaselineCourseTarget")
                .LabelWidth(AutopilotParameterLabelWidth)
                .Add(UI::PropertyRow(
                    "Target Course")[UI::ScalarEditor("TargetCourseDeg",
                    state.targetCourseDeg)
                        .Range(-180.0, 180.0)
                        .ShowSlider(false)
                        .ShowStepper()
                        .Step(1.0)
                        .FastStep(10.0)
                        .Format("%.2f deg")
                        .InputWidth(AdaptivePropertyInputWidth)
                        .OnChanged([events = props.valueEvents](double value) {
                          events.Emit(
                              {BaselineRollHoldField::TargetCourseDeg, value});
                        })])
            + parameters
            + UI::KeyValueGrid("BaselineCourseHoldDiagnostics")
                .ColumnsPerRow(2)
                .AddDouble("Current Course", props.currentCourseDeg, "%.2f deg")
                .AddDouble("Course Error", props.courseErrorDeg, "%.2f deg")
                .AddDouble("Raw Roll SP",
                    props.courseRawRollSetpointDeg,
                    "%.2f deg")
                .AddDouble("Limited Roll SP",
                    props.courseLimitedRollSetpointDeg,
                    "%.2f deg")
            + UI::StatusBadge(props.courseHoldActive ? "Active" : "Inactive",
                props.courseHoldActive ? UI::StatusTone::Success
                                       : UI::StatusTone::Neutral)]];
}

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
                         .Enabled(!state.courseHold)
                         .Width(AutopilotTargetInputWidth)
                         .Step(1.0)
                         .FastStep(10.0)
                         .Format("%.2f")
                         .OnChanged([events = props.valueEvents](double value) {
                           events.Emit(
                               {BaselineRollHoldField::TargetDeg, value});
                         })
                   + UI::Text(state.rollHold ? "Hold" : "Off")
                   + UI::TextDisabled(state.courseHold
                         ? "Overridden by Course Hold"
                         : "")
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

UI::UIElement Px4AttitudePanel::BuildRoll(
    const BaselineAutopilotPanelProps &props) {
  return MakeBaselineRollHoldSection(props);
}

UI::UIElement Px4AttitudePanel::BuildPitch(
    const BaselineAutopilotPanelProps &props) {
  return MakeBaselinePitchHoldSection(props);
}

UI::UIElement Px4AttitudePanel::BuildCourse(
    const BaselineAutopilotPanelProps &props) {
  return MakeBaselineCourseHoldSection(props);
}

UI::UIElement Px4AttitudePanel::BuildYaw(
    const BaselineAutopilotPanelProps &props) {
  return MakeBaselineYawControlSection(props);
}
} // namespace gui
