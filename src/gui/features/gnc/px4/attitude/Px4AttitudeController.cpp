#include "gui/features/gnc/px4/attitude/Px4AttitudeController.hpp"

#include "common/math/Math.hpp"

#include <algorithm>
#include <cmath>

namespace gui {
Px4AttitudeController::Px4AttitudeController(
    BaselineAutopilotPanelState &baseline)
    : baseline_(baseline) {}

void Px4AttitudeController::SynchronizeBaseline(
    const sim::BaselineRollHoldConfig &config) {
  baseline_.rollHold = config.enabled;
  baseline_.rollTargetDeg = math::RadToDeg(config.targetRollRad);
  baseline_.courseHold = config.courseHoldEnabled;
  baseline_.targetCourseDeg = math::RadToDeg(config.targetCourseRad);
  baseline_.courseGuidancePeriodSec = config.courseGuidancePeriodSec;
  baseline_.courseGuidanceDampingRatio = config.courseGuidanceDampingRatio;
  baseline_.courseMaximumRollDeg = math::RadToDeg(config.courseMaxRollRad);
  baseline_.courseMaximumRollSetpointRateDegPerSec =
      math::RadToDeg(config.courseMaxRollSetpointRateRadPerSec);
  baseline_.px4RollTimeConstantSec = config.timeConstantSec;
  baseline_.px4RollMaximumRateDegPerSec =
      math::RadToDeg(config.maximumRollRateRadPerSec);
  baseline_.px4RollRateProportionalGain = config.rateProportionalGain;
  baseline_.px4RollRateIntegralGain = config.rateIntegralGain;
  baseline_.px4RollRateDerivativeGain = config.rateDerivativeGain;
  baseline_.px4RollRateFeedForwardGain = config.rateFeedForwardGain;
  baseline_.px4RollIntegratorLimit = config.integratorLimit;
  baseline_.pitchHold = config.pitchHoldEnabled;
  baseline_.pitchTargetDeg = math::RadToDeg(config.targetPitchRad);
  baseline_.px4PitchTimeConstantSec = config.pitchTimeConstantSec;
  baseline_.px4PitchMaximumPositiveRateDegPerSec =
      math::RadToDeg(config.maximumPositivePitchRateRadPerSec);
  baseline_.px4PitchMaximumNegativeRateDegPerSec =
      math::RadToDeg(config.maximumNegativePitchRateRadPerSec);
  baseline_.px4PitchRateProportionalGain = config.pitchRateProportionalGain;
  baseline_.px4PitchRateIntegralGain = config.pitchRateIntegralGain;
  baseline_.px4PitchRateDerivativeGain = config.pitchRateDerivativeGain;
  baseline_.px4PitchRateFeedForwardGain = config.pitchRateFeedForwardGain;
  baseline_.px4PitchIntegratorLimit = config.pitchIntegratorLimit;
  baseline_.directRollRateTestEnabled = config.directRollRateTestEnabled;
  baseline_.directRollRateCommandDegPerSec =
      math::RadToDeg(config.directRollRateCommandRadPerSec);
  baseline_.yawRateControlEnabled = config.yawRateControlEnabled;
  baseline_.coordinatedTurnEnabled = config.coordinatedTurnEnabled;
  baseline_.px4YawMaximumRateDegPerSec =
      math::RadToDeg(config.maximumYawRateRadPerSec);
  baseline_.px4YawRateProportionalGain = config.yawRateProportionalGain;
  baseline_.px4YawRateIntegralGain = config.yawRateIntegralGain;
  baseline_.px4YawRateDerivativeGain = config.yawRateDerivativeGain;
  baseline_.px4YawRateFeedForwardGain = config.yawRateFeedForwardGain;
  baseline_.px4YawIntegratorLimit = config.yawIntegratorLimit;
  baseline_.sideslipToYawRateGain = config.sideslipToYawRateGain;
  baseline_.yawRateWashoutTimeConstantSec =
      config.yawRateWashoutTimeConstantSec;
  baseline_.rollToYawFeedForwardGain = config.rollToYawFeedForwardGain;
}

void Px4AttitudeController::Handle(const BaselineRollHoldValueChanged &event) {
  if (SetBaselinePx4PitchHoldParameter(baseline_, event.field, event.value)
      || SetBaselinePx4CourseHoldParameter(baseline_, event.field, event.value)
      || SetBaselinePx4RollHoldParameter(baseline_, event.field, event.value)
      || SetBaselinePx4YawRateParameter(baseline_, event.field, event.value)) {
    return;
  }
  if (!std::isfinite(event.value)) {
    return;
  }

  switch (event.field) {
  case BaselineRollHoldField::Enabled:
    baseline_.rollHold = event.value != 0.0;
    return;
  case BaselineRollHoldField::TargetDeg:
    baseline_.rollTargetDeg = event.value;
    return;
  case BaselineRollHoldField::PitchHoldEnabled:
    baseline_.pitchHold = event.value != 0.0;
    return;
  case BaselineRollHoldField::TargetPitchDeg:
    baseline_.pitchTargetDeg = std::clamp(event.value, -90.0, 90.0);
    return;
  case BaselineRollHoldField::CourseHoldEnabled:
    baseline_.courseHold = event.value != 0.0;
    if (baseline_.courseHold) {
      baseline_.rollHold = true;
    }
    return;
  case BaselineRollHoldField::TargetCourseDeg:
    baseline_.targetCourseDeg = event.value;
    return;
  case BaselineRollHoldField::DirectRollRateTestEnabled:
    baseline_.directRollRateTestEnabled = event.value != 0.0;
    return;
  case BaselineRollHoldField::DirectRollRateCommandDegPerSec:
    baseline_.directRollRateCommandDegPerSec = event.value;
    return;
  case BaselineRollHoldField::YawRateControlEnabled:
    baseline_.yawRateControlEnabled = event.value != 0.0;
    return;
  case BaselineRollHoldField::CoordinatedTurnEnabled:
    baseline_.coordinatedTurnEnabled = event.value != 0.0;
    return;
  default:
    return;
  }
}

void Px4AttitudeController::Handle(
    const BaselineRollHoldTuningResetRequested &) {
  ResetBaselinePx4RollHoldTuning(baseline_);
}

void Px4AttitudeController::Handle(
    const BaselinePitchHoldTuningResetRequested &) {
  ResetBaselinePx4PitchHoldTuning(baseline_);
}

void Px4AttitudeController::Handle(const Px4AttitudeViewStateChanged &event) {
  baseline_.px4RollTuningOpen = event.rollTuningOpen;
  baseline_.px4RollDiagnosticsOpen = event.rollDiagnosticsOpen;
  baseline_.px4PitchTuningOpen = event.pitchTuningOpen;
  baseline_.px4PitchDiagnosticsOpen = event.pitchDiagnosticsOpen;
}
} // namespace gui
