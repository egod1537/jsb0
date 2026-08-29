#include "gui/features/gnc/GNCController.hpp"

#include "messaging/SimulationMessageClient.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <cmath>

namespace gui {
GNCController::GNCController(application::SimulationMessageClient &client)
    : client_(client) {}

void GNCController::Synchronize(const sim::SimulationSnapshot &snapshot) {
  if (model_.autopilotStateLoaded) {
    return;
  }

  const sim::PrimaryRollHoldConfig &primary =
      snapshot.primaryAutopilot.primaryRollHold;
  model_.primaryAutopilot.rollHold = primary.enabled;
  model_.primaryAutopilot.rollTargetDeg = math::RadToDeg(primary.targetRollRad);
  model_.primaryAutopilot.rollAngleProportionalGain =
      primary.rollAngleProportionalGain;
  model_.primaryAutopilot.rollRateProportionalGain =
      primary.rollRateProportionalGain;

  if (snapshot.baselineAutopilot) {
    const sim::BaselineRollHoldConfig &baseline =
        snapshot.baselineAutopilot->baselineRollHold;
    model_.baselineAutopilot.rollHold = baseline.enabled;
    model_.baselineAutopilot.rollTargetDeg =
        math::RadToDeg(baseline.targetRollRad);
    model_.baselineAutopilot.px4RollTimeConstantSec = baseline.timeConstantSec;
    model_.baselineAutopilot.px4RollMaximumRateDegPerSec =
        math::RadToDeg(baseline.maximumRollRateRadPerSec);
    model_.baselineAutopilot.px4RollRateProportionalGain =
        baseline.rateProportionalGain;
    model_.baselineAutopilot.px4RollRateIntegralGain =
        baseline.rateIntegralGain;
    model_.baselineAutopilot.px4RollRateDerivativeGain =
        baseline.rateDerivativeGain;
    model_.baselineAutopilot.px4RollRateFeedForwardGain =
        baseline.rateFeedForwardGain;
    model_.baselineAutopilot.px4RollIntegratorLimit = baseline.integratorLimit;
  }
  model_.autopilotStateLoaded = true;
}

void GNCController::PublishConfiguration(
    const sim::SimulationSnapshot &snapshot) {
  if (snapshot.status.scenario.has_value()) {
    return;
  }
  Handle(PrimaryRollHoldConfigChanged{{
      .enabled = model_.primaryAutopilot.rollHold,
      .targetRollRad = math::DegToRad(model_.primaryAutopilot.rollTargetDeg),
      .rollAngleProportionalGain =
          model_.primaryAutopilot.rollAngleProportionalGain,
      .rollRateProportionalGain =
          model_.primaryAutopilot.rollRateProportionalGain,
  }});

  if (snapshot.baseline.has_value() && snapshot.baselineAutopilot.has_value()
      && snapshot.baselineAutopilot->available) {
    Handle(BaselineRollHoldConfigChanged{{
        .enabled = model_.baselineAutopilot.rollHold,
        .targetRollRad = math::DegToRad(model_.baselineAutopilot.rollTargetDeg),
        .timeConstantSec = model_.baselineAutopilot.px4RollTimeConstantSec,
        .maximumRollRateRadPerSec = math::DegToRad(
            model_.baselineAutopilot.px4RollMaximumRateDegPerSec),
        .rateProportionalGain =
            model_.baselineAutopilot.px4RollRateProportionalGain,
        .rateIntegralGain = model_.baselineAutopilot.px4RollRateIntegralGain,
        .rateDerivativeGain =
            model_.baselineAutopilot.px4RollRateDerivativeGain,
        .rateFeedForwardGain =
            model_.baselineAutopilot.px4RollRateFeedForwardGain,
        .integratorLimit = model_.baselineAutopilot.px4RollIntegratorLimit,
    }});
  }
}

void GNCController::Handle(const TrimRequested &event) {
  client_.RunTrim(event.request, event.fromCurrentState);
}

void GNCController::Handle(const ManualControlChanged &event) {
  client_.SetManualControl(event.input);
}

void GNCController::Handle(const PrimaryRollHoldConfigChanged &event) {
  client_.SetPrimaryRollHoldConfig(event.config);
}

void GNCController::Handle(const BaselineRollHoldConfigChanged &event) {
  client_.SetBaselineRollHoldConfig(event.config);
}

void GNCController::Handle(const PrimaryRollHoldValueChanged &event) {
  switch (event.field) {
  case PrimaryRollHoldField::Enabled:
    model_.primaryAutopilot.rollHold = event.value != 0.0;
    break;
  case PrimaryRollHoldField::TargetDeg:
    model_.primaryAutopilot.rollTargetDeg = event.value;
    break;
  case PrimaryRollHoldField::AngleProportionalGain:
    model_.primaryAutopilot.rollAngleProportionalGain = event.value;
    break;
  case PrimaryRollHoldField::RateProportionalGain:
    model_.primaryAutopilot.rollRateProportionalGain = event.value;
    break;
  }
}

void GNCController::Handle(const BaselineRollHoldValueChanged &event) {
  double value = event.value;
  switch (event.field) {
  case BaselineRollHoldField::Enabled:
    model_.baselineAutopilot.rollHold = value != 0.0;
    return;
  case BaselineRollHoldField::TargetDeg:
    model_.baselineAutopilot.rollTargetDeg = value;
    return;
  case BaselineRollHoldField::TimeConstantSec:
    value = std::clamp(value, 0.01, 1.0);
    model_.baselineAutopilot.px4RollTimeConstantSec = value;
    return;
  case BaselineRollHoldField::MaximumRateDegPerSec:
    value = std::clamp(value, 10.0, 180.0);
    model_.baselineAutopilot.px4RollMaximumRateDegPerSec = value;
    return;
  case BaselineRollHoldField::RateProportionalGain:
    value = std::clamp(value, 0.005, 0.5);
    model_.baselineAutopilot.px4RollRateProportionalGain = value;
    return;
  case BaselineRollHoldField::RateIntegralGain:
    value = std::clamp(value, 0.005, 0.5);
    model_.baselineAutopilot.px4RollRateIntegralGain = value;
    return;
  case BaselineRollHoldField::RateDerivativeGain:
    value = std::clamp(value, 0.0, 0.5);
    model_.baselineAutopilot.px4RollRateDerivativeGain = value;
    return;
  case BaselineRollHoldField::RateFeedForwardGain:
    value = std::clamp(value, 0.0, 6.0);
    model_.baselineAutopilot.px4RollRateFeedForwardGain = value;
    return;
  case BaselineRollHoldField::IntegratorLimit:
    value = std::clamp(value, 0.0, 1.0);
    model_.baselineAutopilot.px4RollIntegratorLimit = value;
    return;
  }
}

void GNCController::Handle(const BaselineRollHoldTuningResetRequested &) {
  model_.baselineAutopilot.px4RollTimeConstantSec = 0.35;
  model_.baselineAutopilot.px4RollMaximumRateDegPerSec = 70.0;
  model_.baselineAutopilot.px4RollRateProportionalGain = 0.160;
  model_.baselineAutopilot.px4RollRateIntegralGain = 0.080;
  model_.baselineAutopilot.px4RollRateDerivativeGain = 0.0;
  model_.baselineAutopilot.px4RollRateFeedForwardGain = 0.80;
  model_.baselineAutopilot.px4RollIntegratorLimit = 0.15;
}

void GNCController::Handle(const TrimRequestValueChanged &event) {
  switch (event.field) {
  case TrimRequestField::Mode:
    model_.trimRequest.mode = static_cast<gnc::TrimMode>(
        std::clamp(static_cast<int>(event.value), 0, 2));
    break;
  case TrimRequestField::AirspeedKts:
    model_.trimRequest.airspeedKts = event.value;
    break;
  case TrimRequestField::AltitudeFt:
    model_.trimRequest.altitudeFt = event.value;
    break;
  case TrimRequestField::FlightPathAngleDeg:
    model_.trimRequest.flightPathAngleDeg = event.value;
    break;
  }
}

void GNCController::Handle(const TrimExecutionRequested &event) {
  if (model_.trimInProgress) {
    return;
  }
  model_.trimInProgress = true;
  Handle(TrimRequested{model_.trimRequest, event.fromCurrentState});
  model_.trimResultOpen = true;
  model_.trimResidualOpen = true;
  model_.trimInProgress = false;
}

void GNCController::Handle(const GNCViewStateChanged &event) {
  model_.primaryAutopilot.rollHoldParametersOpen = event.primaryParametersOpen;
  model_.baselineAutopilot.px4RollTuningOpen = event.baselineTuningOpen;
  model_.baselineAutopilot.px4RollDiagnosticsOpen =
      event.baselineDiagnosticsOpen;
  model_.trimResultOpen = event.trimResultOpen;
  model_.trimResidualOpen = event.trimResidualOpen;
}
} // namespace gui
