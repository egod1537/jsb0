#include "gui/features/gnc/GNCController.hpp"

#include "common/math/Math.hpp"
#include "messaging/SimulationMessageClient.hpp"

namespace gui {
GNCController::GNCController(application::SimulationMessageClient &client)
    : client_(client), experimentalController_(model_.primaryAutopilot),
      px4AttitudeController_(model_.baselineAutopilot),
      tecsController_(model_.baselineAutopilot),
      trimController_(client_, model_.trimRequest, model_.trimResultOpen,
          model_.trimResidualOpen, model_.trimInProgress) {}

void GNCController::Synchronize(const sim::SimulationSnapshot &snapshot) {
  if (model_.autopilotStateLoaded) {
    return;
  }

  experimentalController_.Synchronize(
      snapshot.primaryAutopilot.primaryRollHold);
  if (snapshot.baselineAutopilot) {
    const sim::BaselineRollHoldConfig &baseline =
        snapshot.baselineAutopilot->baselineRollHold;
    px4AttitudeController_.SynchronizeBaseline(baseline);
    tecsController_.Synchronize(baseline);
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

  if (!snapshot.baseline.has_value() || !snapshot.baselineAutopilot.has_value()
      || !snapshot.baselineAutopilot->available) {
    return;
  }

  const BaselineAutopilotPanelState &state = model_.baselineAutopilot;
  Handle(BaselineRollHoldConfigChanged{{
      .enabled = state.rollHold,
      .targetRollRad = math::DegToRad(state.rollTargetDeg),
      .timeConstantSec = state.px4RollTimeConstantSec,
      .maximumRollRateRadPerSec =
          math::DegToRad(state.px4RollMaximumRateDegPerSec),
      .rateProportionalGain = state.px4RollRateProportionalGain,
      .rateIntegralGain = state.px4RollRateIntegralGain,
      .rateDerivativeGain = state.px4RollRateDerivativeGain,
      .rateFeedForwardGain = state.px4RollRateFeedForwardGain,
      .integratorLimit = state.px4RollIntegratorLimit,
      .pitchHoldEnabled = state.pitchHold,
      .targetPitchRad = math::DegToRad(state.pitchTargetDeg),
      .pitchTimeConstantSec = state.px4PitchTimeConstantSec,
      .maximumPositivePitchRateRadPerSec =
          math::DegToRad(state.px4PitchMaximumPositiveRateDegPerSec),
      .maximumNegativePitchRateRadPerSec =
          math::DegToRad(state.px4PitchMaximumNegativeRateDegPerSec),
      .pitchRateProportionalGain = state.px4PitchRateProportionalGain,
      .pitchRateIntegralGain = state.px4PitchRateIntegralGain,
      .pitchRateDerivativeGain = state.px4PitchRateDerivativeGain,
      .pitchRateFeedForwardGain = state.px4PitchRateFeedForwardGain,
      .pitchIntegratorLimit = state.px4PitchIntegratorLimit,
      .tecsEnabled = state.tecs,
      .targetAltitudeM = state.tecsTargetAltitudeM,
      .targetAirspeedMps = state.tecsTargetAirspeedMps,
      .tecsSettings = state.tecsSettings,
      .directRollRateTestEnabled = state.directRollRateTestEnabled,
      .directRollRateCommandRadPerSec =
          math::DegToRad(state.directRollRateCommandDegPerSec),
      .courseHoldEnabled = state.courseHold,
      .targetCourseRad = math::DegToRad(state.targetCourseDeg),
      .courseGuidancePeriodSec = state.courseGuidancePeriodSec,
      .courseGuidanceDampingRatio = state.courseGuidanceDampingRatio,
      .courseMaxRollRad = math::DegToRad(state.courseMaximumRollDeg),
      .courseMaxRollSetpointRateRadPerSec =
          math::DegToRad(state.courseMaximumRollSetpointRateDegPerSec),
      .yawRateControlEnabled = state.yawRateControlEnabled,
      .coordinatedTurnEnabled = state.coordinatedTurnEnabled,
      .maximumYawRateRadPerSec =
          math::DegToRad(state.px4YawMaximumRateDegPerSec),
      .yawRateProportionalGain = state.px4YawRateProportionalGain,
      .yawRateIntegralGain = state.px4YawRateIntegralGain,
      .yawRateDerivativeGain = state.px4YawRateDerivativeGain,
      .yawRateFeedForwardGain = state.px4YawRateFeedForwardGain,
      .yawIntegratorLimit = state.px4YawIntegratorLimit,
      .sideslipToYawRateGain = state.sideslipToYawRateGain,
      .yawRateWashoutTimeConstantSec = state.yawRateWashoutTimeConstantSec,
      .rollToYawFeedForwardGain = state.rollToYawFeedForwardGain,
  }});
}

void GNCController::Handle(const TrimRequested &event) {
  trimController_.Handle(event);
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
  experimentalController_.Handle(event);
}

void GNCController::Handle(const BaselineRollHoldValueChanged &event) {
  px4AttitudeController_.Handle(event);
}

void GNCController::Handle(const BaselineRollHoldTuningResetRequested &event) {
  px4AttitudeController_.Handle(event);
}

void GNCController::Handle(const BaselinePitchHoldTuningResetRequested &event) {
  px4AttitudeController_.Handle(event);
}

void GNCController::Handle(const BaselineTecsValueChanged &event) {
  tecsController_.Handle(event);
}

void GNCController::Handle(const BaselineTecsParameterChanged &event) {
  tecsController_.Handle(event);
}

void GNCController::Handle(const BaselineTecsTuningResetRequested &event) {
  tecsController_.Handle(event);
}

void GNCController::Handle(const BaselineTecsAltitudeCaptureRequested &event) {
  tecsController_.Handle(event);
}

void GNCController::Handle(const BaselineTecsAirspeedCaptureRequested &event) {
  tecsController_.Handle(event);
}

void GNCController::Handle(const TrimRequestValueChanged &event) {
  trimController_.Handle(event);
}

void GNCController::Handle(const TrimExecutionRequested &event) {
  trimController_.Handle(event);
}

void GNCController::Handle(const ExperimentalViewStateChanged &event) {
  experimentalController_.Handle(event);
}

void GNCController::Handle(const Px4AttitudeViewStateChanged &event) {
  px4AttitudeController_.Handle(event);
}

void GNCController::Handle(const TrimViewStateChanged &event) {
  trimController_.Handle(event);
}
} // namespace gui
