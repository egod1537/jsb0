#include "gui/features/gnc/experimental/ExperimentalController.hpp"

#include "common/math/Math.hpp"
#include "gui/panels/AutopilotPanel.hpp"

#include <cmath>

namespace gui {
ExperimentalController::ExperimentalController(AutopilotPanelState &state)
    : state_(state) {}

void ExperimentalController::Synchronize(
    const sim::PrimaryRollHoldConfig &config) {
  state_.rollHold = config.enabled;
  state_.rollTargetDeg = math::RadToDeg(config.targetRollRad);
  state_.rollAngleProportionalGain = config.rollAngleProportionalGain;
  state_.rollRateProportionalGain = config.rollRateProportionalGain;
}

void ExperimentalController::OnEvent(const PrimaryRollHoldValueChanged &event) {
  if (!std::isfinite(event.value)) {
    return;
  }
  switch (event.field) {
  case PrimaryRollHoldField::Enabled:
    state_.rollHold = event.value != 0.0;
    break;
  case PrimaryRollHoldField::TargetDeg:
    state_.rollTargetDeg = event.value;
    break;
  case PrimaryRollHoldField::AngleProportionalGain:
    state_.rollAngleProportionalGain = event.value;
    break;
  case PrimaryRollHoldField::RateProportionalGain:
    state_.rollRateProportionalGain = event.value;
    break;
  }
}

void ExperimentalController::OnEvent(const ExperimentalViewStateChanged &event) {
  state_.rollHoldParametersOpen = event.primaryParametersOpen;
}
} // namespace gui
