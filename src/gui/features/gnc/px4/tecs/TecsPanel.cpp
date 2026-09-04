#include "gui/features/gnc/px4/tecs/TecsPanel.hpp"

#include "gui/features/gnc/components/ParameterEditor.hpp"
#include "gui/panels/BaselineAutopilotPanel.hpp"

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotParameterLabelWidth = 112.0F;
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;

UI::PropertyRowBuilder RenderTecsParameterRow(gnc::Px4TecsParameter parameter,
    const BaselineAutopilotPanelState &state,
    architecture::EventSink<BaselineTecsParameterChanged> events) {
  const auto &metadata = gnc::GetPx4TecsParameterMetadata(parameter);
  return RenderGncParameterRow(metadata,
      "TecsEditor",
      gnc::GetPx4TecsParameterValue(state.tecsSettings, parameter),
      [events, parameter](double value) { events.Emit({parameter, value}); });
}
} // namespace

UI::UIElement TecsPanel::Build(const BaselineAutopilotPanelProps &props) {
  BaselineAutopilotPanelState &state = props.state;
  UI::PropertyGridBuilder parameters =
      UI::PropertyGrid("BaselinePx4TecsParameters")
          .LabelWidth(AutopilotParameterLabelWidth)
          .ColumnSpacing(4.0F)
          .RowPadding(2.0F)
          .AlternatingRows();
  for (std::size_t index = 0;
      index < static_cast<std::size_t>(gnc::Px4TecsParameter::Count);
      ++index) {
    parameters.Add(
        RenderTecsParameterRow(static_cast<gnc::Px4TecsParameter>(index),
            state,
            props.tecsParameterEvents));
  }

  UI::ToggleFoldOutBuilder foldOut =
      UI::ToggleFoldOut("TECS", state.tecs)
          .Id("BaselineTecsSection")
          .OnChanged([events = props.tecsValueEvents](bool enabled) {
            events.Emit({BaselineTecsField::Enabled, enabled ? 1.0 : 0.0});
          });

  // clang-format off
  return foldOut[
      UI::VerticalLayout().Spacing(6.0F)
          [
            +UI::TextWrapped(
                "PX4-style total-energy outer loop. TECS owns the pitch "
                "setpoint and throttle; the existing Pitch Hold continues "
                "to own elevator control.")
            + UI::HorizontalLayout().Spacing(8.0F)
                  [
                    +UI::TextDisabled("Altitude SP (m AGL)")
                    + UI::InputDouble("##BaselineTecsAltitudeTarget",
                          state.tecsTargetAltitudeM)
                          .Width(AutopilotTargetInputWidth)
                          .Step(5.0)
                          .FastStep(50.0)
                          .Format("%.1f")
                          .OnChanged([events = props.tecsValueEvents](double value) {
                            events.Emit({BaselineTecsField::TargetAltitudeM, value});
                          })
                    + UI::Button("Capture##TecsAltitude")
                          .OnAction([events = props.tecsAltitudeCaptureEvents,
                                       value = props.currentAltitudeAglM] {
                            events.Emit({value});
                          })
                          .Width(HoldCaptureButtonWidth)
                  ]
            + UI::HorizontalLayout().Spacing(8.0F)
                  [
                    +UI::TextDisabled("Airspeed SP (m/s CAS)")
                    + UI::InputDouble("##BaselineTecsAirspeedTarget",
                          state.tecsTargetAirspeedMps)
                          .Width(AutopilotTargetInputWidth)
                          .Step(1.0)
                          .FastStep(5.0)
                          .Format("%.2f")
                          .OnChanged([events = props.tecsValueEvents](double value) {
                            events.Emit({BaselineTecsField::TargetAirspeedMps, value});
                          })
                    + UI::Button("Capture##TecsAirspeed")
                          .OnAction([events = props.tecsAirspeedCaptureEvents,
                                       value = props.currentCalibratedAirspeedMps] {
                            events.Emit({value});
                          })
                          .Width(HoldCaptureButtonWidth)
                  ]
            + UI::KeyValueGrid("BaselineTecsDiagnosticValues")
                  .ColumnsPerRow(2)
                  .AddDouble("Internal Altitude SP",
                      props.tecsInternalAltitudeSetpointM,
                      "%.1f m")
                  .AddDouble("Pitch SP", props.tecsTargetPitchDeg, "%.2f deg")
                  .AddDouble("Throttle SP", props.tecsTargetThrottle, "%.3f")
                  .AddDouble("Total Energy Error",
                      props.tecsTotalEnergyError,
                      "%.2f m^2/s^2")
                  .AddDouble("Balance Error",
                      props.tecsEnergyBalanceError,
                      "%.2f m^2/s^2")
            + UI::HorizontalLayout().Spacing(8.0F)
                  [
                    +UI::StatusBadge(props.tecsActive ? "Active" : "Inactive",
                        props.tecsActive ? UI::StatusTone::Success
                                         : UI::StatusTone::Neutral)
                    + UI::StatusBadge(
                          props.tecsUnderspeedProtectionActive
                              ? "Underspeed protection"
                              : "Airspeed safe",
                          props.tecsUnderspeedProtectionActive
                              ? UI::StatusTone::Warning
                              : UI::StatusTone::Neutral)
                    + UI::StatusBadge(
                          props.tecsOverspeedProtectionActive
                              ? "Overspeed protection"
                              : "Below overspeed",
                          props.tecsOverspeedProtectionActive
                              ? UI::StatusTone::Warning
                              : UI::StatusTone::Neutral)
                  ]
            + UI::FoldOut("PX4 TECS Tuning").Section()
                  [
                    UI::VerticalLayout().Spacing(6.0F)
                        [
                          +parameters
                          + UI::Button("Reset C172x TECS Tuning")
                                .OnAction([events = props.tecsResetEvents] {
                                  events.Emit({});
                                })
                        ]
                  ]
          ]
      ];
  // clang-format on
}
} // namespace gui
