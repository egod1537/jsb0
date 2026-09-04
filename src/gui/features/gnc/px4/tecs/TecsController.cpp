#include "gui/features/gnc/px4/tecs/TecsController.hpp"

#include "gui/panels/BaselineAutopilotPanel.hpp"
#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/runtime/SimContracts.hpp"

#include <cmath>

namespace gui {
TecsController::TecsController(BaselineAutopilotPanelState &state)
    : state_(state) {}

void TecsController::Synchronize(const sim::BaselineRollHoldConfig &config) {
  state_.tecs = config.tecsEnabled;
  state_.tecsTargetAltitudeM = config.targetAltitudeM;
  state_.tecsTargetAirspeedMps = config.targetAirspeedMps;
  state_.tecsSettings = config.tecsSettings;
}

void TecsController::OnEvent(const BaselineTecsValueChanged &event) {
  if (!std::isfinite(event.value)) {
    return;
  }
  switch (event.field) {
  case BaselineTecsField::Enabled:
    state_.tecs = event.value != 0.0;
    break;
  case BaselineTecsField::TargetAltitudeM:
    state_.tecsTargetAltitudeM = event.value;
    break;
  case BaselineTecsField::TargetAirspeedMps:
    if (event.value > 0.0) {
      state_.tecsTargetAirspeedMps = event.value;
    }
    break;
  }
}

void TecsController::OnEvent(const BaselineTecsParameterChanged &event) {
  gnc::SetPx4TecsParameterValue(state_.tecsSettings,
      event.parameter,
      event.value);
}

void TecsController::OnEvent(const BaselineTecsTuningResetRequested &) {
  const double synchronizedTrimThrottle = state_.tecsSettings.trimThrottle;
  state_.tecsSettings = gnc::GetC172xPx4ControlProfile().tecs;
  state_.tecsSettings.trimThrottle = synchronizedTrimThrottle;
}

void TecsController::OnEvent(const BaselineTecsAltitudeCaptureRequested &event) {
  OnEvent(BaselineTecsValueChanged{BaselineTecsField::TargetAltitudeM,
      event.currentAltitudeAglM});
}

void TecsController::OnEvent(const BaselineTecsAirspeedCaptureRequested &event) {
  OnEvent(BaselineTecsValueChanged{BaselineTecsField::TargetAirspeedMps,
      event.currentCalibratedAirspeedMps});
}
} // namespace gui
