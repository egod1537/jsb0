#include "sim/runtime/SimulationSnapshotBuilder.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Simulation.hpp"
#include "sim/analysis/LinearizationService.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/autopilot/MyAutopilot.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"

namespace sim {
SimulationInstanceSnapshot SimulationSnapshotBuilder::CaptureInstance(
    const Simulation &simulation) {
  const Aircraft &aircraft = simulation.GetAircraft();
  return SimulationInstanceSnapshot{
      .aircraft = aircraft.GetAircraftState(),
      .aircraftDerivative = aircraft.GetAircraftStateDerivative(),
      .fdmState = aircraft.ExtractFDMState(FDMStateFlags::All),
      .controlInput = aircraft.GetControls().GetInput(),
      .engines = aircraft.GetEngines().GetEngineStates(),
      .currentCondition = simulation.GetCurrentCondition(),
      .pitchTrim = aircraft.GetControls().GetPitchTrim(),
      .available = true,
  };
}

AutopilotSnapshot SimulationSnapshotBuilder::CaptureAutopilot(
    const Simulation &simulation) {
  AutopilotSnapshot snapshot;
  const auto *manager =
      simulation.GetComponent<control::FlightControlManager>();
  if (manager == nullptr) {
    return snapshot;
  }

  snapshot.available = true;
  snapshot.mode = manager->GetMode();
  snapshot.manualControl = manager->GetManualController().GetCommandedInput();
  const gnc::IAutopilot &strategy = manager->GetAutopilot();
  if (const auto *autopilot =
          dynamic_cast<const gnc::MyAutopilot *>(&strategy)) {
    snapshot.strategyName = "MyAutopilot";
    const gnc::RollHoldSettings &settings = autopilot->GetRollHoldSettings();
    snapshot.primaryRollHold = {
        .enabled = autopilot->IsRollHoldEnabled(),
        .targetRollRad = settings.targetRollRad,
        .rollAngleProportionalGain = settings.attitudeLoop.proportionalGain,
        .rollRateProportionalGain = settings.rateLoop.proportionalGain,
    };
  } else if (const auto *autopilot =
                 dynamic_cast<const gnc::PX4Autopilot *>(&strategy)) {
    snapshot.strategyName = "PX4Autopilot";
    const gnc::Px4RollHoldReferenceSettings &settings =
        autopilot->GetRollHoldSettings();
    const gnc::Px4AutopilotDiagnostics autopilotDiagnostics =
        autopilot->GetDiagnostics();
    const gnc::Px4RollHoldReferenceDiagnostics &diagnostics =
        autopilotDiagnostics.roll;
    const gnc::Px4CourseHoldSettings &courseSettings =
        autopilot->GetCourseHoldSettings();
    const gnc::Px4CourseHoldDiagnostics &courseDiagnostics =
        autopilotDiagnostics.course;
    const gnc::Px4PitchHoldSettings &pitchSettings =
        autopilot->GetPitchHoldSettings();
    const gnc::Px4PitchHoldDiagnostics &pitchDiagnostics =
        autopilotDiagnostics.pitch;
    const gnc::Px4TecsDiagnostics &tecsDiagnostics = autopilotDiagnostics.tecs;
    snapshot.baselineRollHold = {
        .enabled = autopilot->IsRollHoldEnabled(),
        .targetRollRad = autopilot->GetTargetRollRad(),
        .timeConstantSec = settings.timeConstantSec,
        .maximumRollRateRadPerSec = settings.maximumRollRateRadPerSec,
        .rateProportionalGain = settings.rateProportionalGain,
        .rateIntegralGain = settings.rateIntegralGain,
        .rateDerivativeGain = settings.rateDerivativeGain,
        .rateFeedForwardGain = settings.rateFeedForwardGain,
        .integratorLimit = settings.integratorLimit,
        .pitchHoldEnabled = autopilot->IsPitchHoldEnabled(),
        .targetPitchRad = autopilot->GetTargetPitchRad(),
        .pitchTimeConstantSec = pitchSettings.timeConstantSec,
        .maximumPositivePitchRateRadPerSec =
            pitchSettings.maximumPositivePitchRateRadPerSec,
        .maximumNegativePitchRateRadPerSec =
            pitchSettings.maximumNegativePitchRateRadPerSec,
        .pitchRateProportionalGain = pitchSettings.rateProportionalGain,
        .pitchRateIntegralGain = pitchSettings.rateIntegralGain,
        .pitchRateDerivativeGain = pitchSettings.rateDerivativeGain,
        .pitchRateFeedForwardGain = pitchSettings.rateFeedForwardGain,
        .pitchIntegratorLimit = pitchSettings.integratorLimit,
        .tecsEnabled = autopilot->IsTecsEnabled(),
        .targetAltitudeM = autopilot->GetTargetAltitudeM(),
        .targetAirspeedMps = autopilot->GetTargetAirspeedMps(),
        .tecsSettings = autopilot->GetTecsSettings(),
        .directRollRateTestEnabled = settings.directRollRateTestEnabled,
        .directRollRateCommandRadPerSec =
            settings.directRollRateCommandRadPerSec,
        .courseHoldEnabled = autopilot->IsCourseHoldEnabled(),
        .targetCourseRad = autopilot->GetTargetCourseRad(),
        .courseGuidancePeriodSec = courseSettings.guidancePeriodSec,
        .courseGuidanceDampingRatio = courseSettings.guidanceDampingRatio,
        .courseMaxRollRad = courseSettings.maxRollRad,
        .courseMaxRollSetpointRateRadPerSec =
            courseSettings.maxRollSetpointRateRadPerSec,
        .yawRateControlEnabled = autopilot->IsYawRateControlEnabled(),
        .coordinatedTurnEnabled =
            autopilot->GetYawRateSettings().setpointMode
            == gnc::Px4YawRateSetpointMode::CoordinatedTurn,
        .maximumYawRateRadPerSec =
            autopilot->GetYawRateSettings().maximumYawRateRadPerSec,
        .yawRateProportionalGain =
            autopilot->GetYawRateSettings().rateProportionalGain,
        .yawRateIntegralGain = autopilot->GetYawRateSettings().rateIntegralGain,
        .yawRateDerivativeGain =
            autopilot->GetYawRateSettings().rateDerivativeGain,
        .yawRateFeedForwardGain =
            autopilot->GetYawRateSettings().rateFeedForwardGain,
        .yawIntegratorLimit = autopilot->GetYawRateSettings().integratorLimit,
        .sideslipToYawRateGain =
            autopilot->GetYawRateSettings().sideslipToYawRateGain,
        .yawRateWashoutTimeConstantSec =
            autopilot->GetYawRateSettings().yawRateWashoutTimeConstantSec,
        .rollToYawFeedForwardGain =
            autopilot->GetYawRateSettings().rollToYawFeedForwardGain,
    };
    snapshot.baselineDiagnostics = {
        .aileronCommand = diagnostics.aileronCommand,
        .bodyRateSetpointRadPerSec = diagnostics.bodyRateSetpointRadPerSec,
        .rollErrorRad = diagnostics.rollErrorRad,
        .airspeedScaling = diagnostics.airspeedScaling,
    };
    snapshot.baselinePitchDiagnostics = {
        .outputValid = pitchDiagnostics.controlOutputValid,
        .elevatorCommand = pitchDiagnostics.elevatorCommand,
        .bodyRateSetpointRadPerSec = pitchDiagnostics.bodyRateSetpointRadPerSec,
        .pitchErrorRad = pitchDiagnostics.pitchErrorRad,
        .airspeedScaling = pitchDiagnostics.airspeedScaling,
    };
    snapshot.baselineTecsDiagnostics = {
        .outputValid = tecsDiagnostics.controlOutputValid,
        .internalAltitudeSetpointM = tecsDiagnostics.internalAltitudeSetpointM,
        .targetPitchRad = tecsDiagnostics.targetPitchRad,
        .targetThrottle = tecsDiagnostics.targetThrottle,
        .totalEnergyError = tecsDiagnostics.totalEnergyError,
        .energyBalanceError = tecsDiagnostics.energyBalanceError,
        .underspeedProtectionActive =
            tecsDiagnostics.underspeedProtectionActive,
        .overspeedProtectionActive = tecsDiagnostics.overspeedProtectionActive,
    };
    snapshot.baselineCourseDiagnostics = {
        .outputValid = courseDiagnostics.controlOutputValid,
        .groundSpeedValid = courseDiagnostics.groundSpeedValid,
        .targetCourseRad = courseDiagnostics.targetCourseRad,
        .currentCourseRad = courseDiagnostics.currentCourseRad,
        .courseErrorRad = courseDiagnostics.courseErrorRad,
        .groundSpeedMps = courseDiagnostics.groundSpeedMps,
        .rawRollSetpointRad = courseDiagnostics.rawRollSetpointRad,
        .limitedRollSetpointRad = courseDiagnostics.limitedRollSetpointRad,
        .rollLimited = courseDiagnostics.rollLimited,
        .rollSetpointRateLimited = courseDiagnostics.rollSetpointRateLimited,
    };
  } else {
    snapshot.strategyName = "Autopilot Strategy";
  }
  return snapshot;
}

LinearizationSnapshot SimulationSnapshotBuilder::CaptureLinearization(
    const Simulation &simulation) {
  const LinearizationServiceState state =
      LinearizationService{}.Capture(simulation);
  return LinearizationSnapshot{
      .available = state.available,
      .automaticUpdatesEnabled = state.automaticUpdatesEnabled,
      .updateInProgress = state.updateInProgress,
      .errorMessage = state.errorMessage,
      .result = state.result,
      .dynamicModeHistory = state.dynamicModeHistory,
  };
}
} // namespace sim
