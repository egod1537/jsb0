#include "sim/telemetry/SimulationTelemetryPublisher.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Tick.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/autopilot/IControllerInspectable.hpp"
#include "sim/gnc/autopilot/IRollHoldAutopilot.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/gnc/control/legacy/RollHoldController.hpp"
#include "sim/telemetry/AircraftTelemetry.hpp"
#include "sim/telemetry/AutopilotTelemetry.hpp"
#include "sim/telemetry/TelemetryRegistry.hpp"
#include "common/math/Math.hpp"

#include <cmath>
#include <optional>
#include <string_view>

namespace {
struct RollHoldTelemetrySnapshot {
  double commandedRollRad = 0.0;
  double rollRad = 0.0;
  double rollErrorRad = 0.0;
  std::optional<double> commandedRollRateRadPerSec;
  double rollRateRadPerSec = 0.0;
  std::optional<double> rollRateErrorRadPerSec;
};
} // namespace

namespace telemetry {
void SimulationTelemetryPublisher::Publish(const sim::Aircraft &aircraft,
    const control::FlightControlManager &flightControls, const sim::Tick &tick,
    TelemetryRegistry &registry) {
  PublishAutopilot(aircraft, flightControls, tick, registry);
  PublishAircraft(aircraft, tick, registry);
}

void SimulationTelemetryPublisher::PublishAutopilot(
    const sim::Aircraft &aircraft,
    const control::FlightControlManager &flightControls, const sim::Tick &tick,
    TelemetryRegistry &registry) {
  const auto publish = [&registry, &tick](std::string_view path, double value) {
    registry.Publish(path, tick.simTimeSec, value);
  };

  const gnc::IAutopilot &autopilot = flightControls.GetAutopilot();
  const auto *rollHoldCapability =
      dynamic_cast<const gnc::IRollHoldAutopilot *>(&autopilot);
  const auto *controllers =
      dynamic_cast<const gnc::IControllerInspectable *>(&autopilot);
  const auto *px4Autopilot =
      dynamic_cast<const gnc::PX4Autopilot *>(&autopilot);

  if (px4Autopilot != nullptr) {
    publish(paths::AutopilotTecsEnabled,
        px4Autopilot->IsTecsEnabled() ? 1.0 : 0.0);
  }
  if (px4Autopilot != nullptr) {
    const auto autopilotDiagnostics = px4Autopilot->GetDiagnostics();
    const gnc::Px4TecsDiagnostics &diagnostics = autopilotDiagnostics.tecs;
    if (diagnostics.controlOutputValid) {
      publish(paths::AutopilotTecsAltitude, diagnostics.altitudeM);
      publish(paths::AutopilotTecsTargetAltitude, diagnostics.targetAltitudeM);
      publish(paths::AutopilotTecsInternalAltitudeSetpoint,
          diagnostics.internalAltitudeSetpointM);
      publish(paths::AutopilotTecsAirspeed, diagnostics.airspeedMps);
      publish(paths::AutopilotTecsTargetAirspeed,
          diagnostics.targetAirspeedMps);
      publish(paths::AutopilotTecsVerticalSpeed, diagnostics.verticalSpeedMps);
      publish(paths::AutopilotTecsAirspeedRate, diagnostics.airspeedRateMps2);
      publish(paths::AutopilotTecsTargetVerticalSpeed,
          diagnostics.targetVerticalSpeedMps);
      publish(paths::AutopilotTecsTargetAirspeedRate,
          diagnostics.targetAirspeedRateMps2);
      publish(paths::AutopilotTecsPotentialEnergy, diagnostics.potentialEnergy);
      publish(paths::AutopilotTecsPotentialEnergySetpoint,
          diagnostics.potentialEnergySetpoint);
      publish(paths::AutopilotTecsPotentialEnergyError,
          diagnostics.potentialEnergyError);
      publish(paths::AutopilotTecsKineticEnergy, diagnostics.kineticEnergy);
      publish(paths::AutopilotTecsKineticEnergySetpoint,
          diagnostics.kineticEnergySetpoint);
      publish(paths::AutopilotTecsKineticEnergyError,
          diagnostics.kineticEnergyError);
      publish(paths::AutopilotTecsTotalEnergy, diagnostics.totalEnergy);
      publish(paths::AutopilotTecsTotalEnergySetpoint,
          diagnostics.totalEnergySetpoint);
      publish(paths::AutopilotTecsTotalEnergyError,
          diagnostics.totalEnergyError);
      publish(paths::AutopilotTecsEnergyBalance, diagnostics.energyBalance);
      publish(paths::AutopilotTecsEnergyBalanceSetpoint,
          diagnostics.energyBalanceSetpoint);
      publish(paths::AutopilotTecsEnergyBalanceError,
          diagnostics.energyBalanceError);
      publish(paths::AutopilotTecsTotalEnergyRate, diagnostics.totalEnergyRate);
      publish(paths::AutopilotTecsTotalEnergyRateSetpoint,
          diagnostics.totalEnergyRateSetpoint);
      publish(paths::AutopilotTecsTotalEnergyRateError,
          diagnostics.totalEnergyRateError);
      publish(paths::AutopilotTecsEnergyBalanceRate,
          diagnostics.energyBalanceRate);
      publish(paths::AutopilotTecsEnergyBalanceRateSetpoint,
          diagnostics.energyBalanceRateSetpoint);
      publish(paths::AutopilotTecsEnergyBalanceRateError,
          diagnostics.energyBalanceRateError);
      publish(paths::AutopilotTecsTargetPitch, diagnostics.targetPitchRad);
      publish(paths::AutopilotTecsTargetThrottle, diagnostics.targetThrottle);
      publish(paths::AutopilotTecsUnclampedPitch,
          diagnostics.unclampedPitchRad);
      publish(paths::AutopilotTecsUnclampedThrottle,
          diagnostics.unclampedThrottle);
      publish(paths::AutopilotTecsThrottleFeedForwardTerm,
          diagnostics.throttleFeedForwardTerm);
      publish(paths::AutopilotTecsThrottleProportionalTerm,
          diagnostics.throttleProportionalTerm);
      publish(paths::AutopilotTecsThrottleIntegralTerm,
          diagnostics.throttleIntegralTerm);
      publish(paths::AutopilotTecsThrottleRateTerm,
          diagnostics.throttleRateTerm);
      publish(paths::AutopilotTecsPitchProportionalTerm,
          diagnostics.pitchProportionalTerm);
      publish(paths::AutopilotTecsPitchIntegralTerm,
          diagnostics.pitchIntegralTerm);
      publish(paths::AutopilotTecsPitchRateTerm, diagnostics.pitchRateTerm);
      publish(paths::AutopilotTecsPitchUpperLimited,
          diagnostics.pitchUpperLimited ? 1.0 : 0.0);
      publish(paths::AutopilotTecsPitchLowerLimited,
          diagnostics.pitchLowerLimited ? 1.0 : 0.0);
      publish(paths::AutopilotTecsPitchRateLimited,
          diagnostics.pitchRateLimited ? 1.0 : 0.0);
      publish(paths::AutopilotTecsThrottleUpperSaturated,
          diagnostics.throttleUpperSaturated ? 1.0 : 0.0);
      publish(paths::AutopilotTecsThrottleLowerSaturated,
          diagnostics.throttleLowerSaturated ? 1.0 : 0.0);
      publish(paths::AutopilotTecsThrottleRateLimited,
          diagnostics.throttleRateLimited ? 1.0 : 0.0);
      publish(paths::AutopilotTecsUnderspeedProtection,
          diagnostics.underspeedProtectionActive ? 1.0 : 0.0);
      publish(paths::AutopilotTecsOverspeedProtection,
          diagnostics.overspeedProtectionActive ? 1.0 : 0.0);
      publish(paths::AutopilotTecsThrottleIntegratorLimited,
          diagnostics.throttleIntegratorLimited ? 1.0 : 0.0);
      publish(paths::AutopilotTecsPitchIntegratorLimited,
          diagnostics.pitchIntegratorLimited ? 1.0 : 0.0);
    }
  }

  if (px4Autopilot != nullptr) {
    const auto autopilotDiagnostics = px4Autopilot->GetDiagnostics();
    const gnc::Px4PitchHoldDiagnostics &diagnostics =
        autopilotDiagnostics.pitch;
    if (diagnostics.controlOutputValid) {
      const double integratorLimit =
          std::abs(px4Autopilot->GetPitchHoldSettings().integratorLimit);
      publish(paths::AutopilotPitchHoldCommandedPitch,
          diagnostics.targetPitchRad);
      publish(paths::AutopilotPitchHoldPitch,
          aircraft.GetProperties().Pitch().Rad());
      publish(paths::AutopilotPitchHoldPitchError, diagnostics.pitchErrorRad);
      publish(paths::AutopilotPitchHoldCommandedPitchRate,
          diagnostics.bodyRateSetpointRadPerSec);
      publish(paths::AutopilotPitchHoldPitchRate,
          diagnostics.bodyRateSetpointRadPerSec
              - diagnostics.bodyRateErrorRadPerSec);
      publish(paths::AutopilotPitchHoldPitchRateError,
          diagnostics.bodyRateErrorRadPerSec);
      publish(paths::AutopilotPitchHoldElevatorCommand,
          diagnostics.elevatorCommand);
      publish(paths::AutopilotPitchHoldRateProportionalTerm,
          diagnostics.rateProportionalTerm);
      publish(paths::AutopilotPitchHoldRateIntegralTerm,
          diagnostics.rateIntegralTerm);
      publish(paths::AutopilotPitchHoldRateDerivativeTerm,
          diagnostics.rateDerivativeTerm);
      publish(paths::AutopilotPitchHoldRateFeedForwardTerm,
          diagnostics.rateFeedForwardTerm);
      publish(paths::AutopilotPitchHoldUnscaledTorqueCommand,
          diagnostics.unscaledTorqueCommand);
      publish(paths::AutopilotPitchHoldRawTorqueCommand,
          diagnostics.rawTorqueCommand);
      publish(paths::AutopilotPitchHoldPitchTorqueCommand,
          diagnostics.pitchTorqueCommand);
      publish(paths::AutopilotPitchHoldAirspeedScaling,
          diagnostics.airspeedScaling);
      publish(paths::AutopilotPitchHoldPositiveSaturation,
          diagnostics.positiveSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotPitchHoldNegativeSaturation,
          diagnostics.negativeSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotPitchHoldIntegratorLimited,
          diagnostics.integratorLimited ? 1.0 : 0.0);
      publish(paths::AutopilotPitchHoldTrimElevatorCommand,
          diagnostics.trimElevatorCommand);
      publish(paths::AutopilotPitchHoldIntegratorPositiveLimit,
          integratorLimit);
      publish(paths::AutopilotPitchHoldIntegratorNegativeLimit,
          -integratorLimit);
    }
  }

  if (px4Autopilot != nullptr) {
    const auto autopilotDiagnostics = px4Autopilot->GetDiagnostics();
    const gnc::Px4CourseHoldDiagnostics &diagnostics =
        autopilotDiagnostics.course;
    if (diagnostics.controlOutputValid) {
      const double currentCourseRad = aircraft.GetProperties().Course().Rad();
      const double groundSpeedMps =
          std::hypot(aircraft.GetProperties().NorthVelocity().Mps(),
              aircraft.GetProperties().EastVelocity().Mps());
      publish(paths::AutopilotCourseHoldCommandedCourse,
          diagnostics.targetCourseRad);
      publish(paths::AutopilotCourseHoldCourse, currentCourseRad);
      publish(paths::AutopilotCourseHoldCourseError,
          math::DeltaAngleRad(currentCourseRad, diagnostics.targetCourseRad));
      publish(paths::AutopilotCourseHoldGroundSpeed, groundSpeedMps);
      publish(paths::AutopilotCourseHoldRawRollSetpoint,
          diagnostics.rawRollSetpointRad);
      publish(paths::AutopilotCourseHoldLimitedRollSetpoint,
          diagnostics.limitedRollSetpointRad);
      publish(paths::AutopilotCourseHoldRollLimited,
          diagnostics.rollLimited ? 1.0 : 0.0);
      publish(paths::AutopilotCourseHoldRollSetpointRateLimited,
          diagnostics.rollSetpointRateLimited ? 1.0 : 0.0);
    }
  }

  double aileronCommand = 0.0;
  std::optional<RollHoldTelemetrySnapshot> snapshot;
  if (const auto *rollHold =
          controllers != nullptr
              ? controllers->GetController<gnc::RollHoldController>()
              : nullptr) {
    const gnc::RollHoldDiagnostics &diagnostics = rollHold->GetDiagnostics();
    aileronCommand = diagnostics.aileronCommand;
    if (diagnostics.controlOutputValid) {
      snapshot = RollHoldTelemetrySnapshot{
          .commandedRollRad = diagnostics.commandedRollRad,
          .rollRad = diagnostics.rollRad,
          .rollErrorRad = diagnostics.rollErrorRad,
          .commandedRollRateRadPerSec =
              diagnostics.commandedRollRateValid
                  ? std::optional<double>(
                        diagnostics.commandedRollRateRadPerSec)
                  : std::nullopt,
          .rollRateRadPerSec = diagnostics.rollRateRadPerSec,
          .rollRateErrorRadPerSec =
              diagnostics.commandedRollRateValid
                  ? std::optional<double>(diagnostics.rollRateErrorRadPerSec)
                  : std::nullopt,
      };
    }
  }
  if (px4Autopilot != nullptr) {
    const auto autopilotDiagnostics = px4Autopilot->GetDiagnostics();
    const gnc::Px4RollHoldReferenceDiagnostics &diagnostics =
        autopilotDiagnostics.roll;
    aileronCommand = diagnostics.aileronCommand;
    if (diagnostics.controlOutputValid) {
      const double integratorLimit =
          std::abs(px4Autopilot->GetRollHoldSettings().integratorLimit);
      snapshot = RollHoldTelemetrySnapshot{
          .commandedRollRad = diagnostics.targetRollRad,
          .rollRad = diagnostics.targetRollRad - diagnostics.rollErrorRad,
          .rollErrorRad = diagnostics.rollErrorRad,
          .commandedRollRateRadPerSec = diagnostics.bodyRateSetpointRadPerSec,
          .rollRateRadPerSec = diagnostics.bodyRateSetpointRadPerSec
                               - diagnostics.bodyRateErrorRadPerSec,
          .rollRateErrorRadPerSec = diagnostics.bodyRateErrorRadPerSec,
      };
      publish(paths::AutopilotRollHoldRateProportionalTerm,
          diagnostics.rateProportionalTerm);
      publish(paths::AutopilotRollHoldRateIntegralTerm,
          diagnostics.rateIntegralTerm);
      publish(paths::AutopilotRollHoldRateDerivativeTerm,
          diagnostics.rateDerivativeTerm);
      publish(paths::AutopilotRollHoldRateFeedForwardTerm,
          diagnostics.rateFeedForwardTerm);
      publish(paths::AutopilotRollHoldUnscaledTorqueCommand,
          diagnostics.unscaledTorqueCommand);
      publish(paths::AutopilotRollHoldRawTorqueCommand,
          diagnostics.rawTorqueCommand);
      publish(paths::AutopilotRollHoldRollTorqueCommand,
          diagnostics.rollTorqueCommand);
      publish(paths::AutopilotRollHoldAirspeedScaling,
          diagnostics.airspeedScaling);
      publish(paths::AutopilotRollHoldPositiveSaturation,
          diagnostics.positiveSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotRollHoldNegativeSaturation,
          diagnostics.negativeSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotRollHoldIntegratorLimited,
          diagnostics.integratorLimited ? 1.0 : 0.0);
      publish(paths::AutopilotRollHoldTrimRollCommand,
          diagnostics.trimRollCommand);
      publish(paths::AutopilotRollHoldRateIntegratorPositiveLimit,
          integratorLimit);
      publish(paths::AutopilotRollHoldRateIntegratorNegativeLimit,
          -integratorLimit);
    }
  }
  const double commandedRollRad =
      snapshot ? snapshot->commandedRollRad
               : (rollHoldCapability != nullptr
                         ? rollHoldCapability->GetTargetRollRad()
                         : 0.0);
  publish(paths::AutopilotRollHoldCommandedRoll, commandedRollRad);

  if (px4Autopilot != nullptr) {
    const auto autopilotDiagnostics = px4Autopilot->GetDiagnostics();
    const gnc::Px4YawRateDiagnostics &diagnostics = autopilotDiagnostics.yaw;
    if (diagnostics.controlOutputValid) {
      const double integratorLimit =
          std::abs(px4Autopilot->GetYawRateSettings().integratorLimit);
      publish(paths::AutopilotYawRateCommandedYawRate,
          diagnostics.bodyRateSetpointRadPerSec);
      publish(paths::AutopilotYawRateCoordinatedYawRate,
          diagnostics.coordinatedRateSetpointRadPerSec);
      publish(paths::AutopilotYawRateSideslip, diagnostics.sideslipRad);
      publish(paths::AutopilotYawRateSideslipCorrection,
          diagnostics.sideslipRateCorrectionRadPerSec);
      publish(paths::AutopilotYawRateYawRate, diagnostics.bodyRateRadPerSec);
      publish(paths::AutopilotYawRateFeedbackYawRate,
          diagnostics.feedbackBodyRateRadPerSec);
      publish(paths::AutopilotYawRateError, diagnostics.bodyRateErrorRadPerSec);
      publish(paths::AutopilotYawRateProportionalTerm,
          diagnostics.rateProportionalTerm);
      publish(paths::AutopilotYawRateIntegralTerm,
          diagnostics.rateIntegralTerm);
      publish(paths::AutopilotYawRateDerivativeTerm,
          diagnostics.rateDerivativeTerm);
      publish(paths::AutopilotYawRateFeedForwardTerm,
          diagnostics.rateFeedForwardTerm);
      publish(paths::AutopilotYawRateRollToYawFeedForwardTerm,
          diagnostics.rollToYawFeedForwardTerm);
      publish(paths::AutopilotYawRateIntegrator, diagnostics.rateIntegrator);
      publish(paths::AutopilotYawRateUnscaledTorqueCommand,
          diagnostics.unscaledTorqueCommand);
      publish(paths::AutopilotYawRateRawTorqueCommand,
          diagnostics.rawTorqueCommand);
      publish(paths::AutopilotYawRateYawTorqueCommand,
          diagnostics.yawTorqueCommand);
      publish(paths::AutopilotYawRateRawRudderCommand,
          diagnostics.rawRudderCommand);
      publish(paths::AutopilotYawRateRudderCommand, diagnostics.rudderCommand);
      publish(paths::AutopilotYawRateAirspeedScaling,
          diagnostics.airspeedScaling);
      publish(paths::AutopilotYawRateTrimRudderCommand,
          diagnostics.trimRudderCommand);
      publish(paths::AutopilotYawRatePositiveSaturation,
          diagnostics.positiveSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotYawRateNegativeSaturation,
          diagnostics.negativeSaturation ? 1.0 : 0.0);
      publish(paths::AutopilotYawRateIntegratorLimited,
          diagnostics.integratorLimited ? 1.0 : 0.0);
      publish(paths::AutopilotYawRateIntegratorPositiveLimit, integratorLimit);
      publish(paths::AutopilotYawRateIntegratorNegativeLimit, -integratorLimit);
    }
  }

  if (!snapshot) {
    return;
  }

  publish(paths::AutopilotRollHoldAileronCommand, aileronCommand);
  publish(paths::AutopilotRollHoldRoll, snapshot->rollRad);
  publish(paths::AutopilotRollHoldRollError, snapshot->rollErrorRad);
  publish(paths::AutopilotRollHoldRollRate, snapshot->rollRateRadPerSec);
  if (snapshot->commandedRollRateRadPerSec) {
    publish(paths::AutopilotRollHoldCommandedRollRate,
        *snapshot->commandedRollRateRadPerSec);
  }
  if (snapshot->rollRateErrorRadPerSec) {
    publish(paths::AutopilotRollHoldRollRateError,
        *snapshot->rollRateErrorRadPerSec);
  }
}

void SimulationTelemetryPublisher::PublishAircraft(
    const sim::Aircraft &aircraft, const sim::Tick &tick,
    TelemetryRegistry &registry) {
  const sim::AircraftState state = aircraft.GetAircraftState();
  const sim::AircraftStateDerivative derivative =
      aircraft.GetAircraftStateDerivative();
  const auto publish = [&registry, &tick](std::string_view path, double value) {
    registry.Publish(path, tick.simTimeSec, value);
  };

  publish(paths::AircraftAeroAlpha, state.alphaRad);
  publish(paths::AircraftAeroBeta, state.betaRad);
  publish(paths::AircraftAttitudeRoll, state.rollRad);
  publish(paths::AircraftAttitudePitch, state.pitchRad);
  publish(paths::AircraftAttitudeHeading, state.headingRad);
  publish(paths::AircraftNavigationCourse, state.courseRad);
  publish(paths::AircraftBodyVelocityU, state.uMps);
  publish(paths::AircraftBodyVelocityV, state.vMps);
  publish(paths::AircraftBodyVelocityW, state.wMps);
  publish(paths::AircraftRateP, state.pRadPerSec);
  publish(paths::AircraftRateQ, state.qRadPerSec);
  publish(paths::AircraftRateR, state.rRadPerSec);
  publish(paths::AircraftCalibratedAirspeed, state.calibratedAirspeedMps);
  publish(paths::AircraftTrueAirspeed, state.trueAirspeedMps);
  publish(paths::AircraftAltitudeAgl, state.altitudeAglM);
  publish(paths::AircraftBodyAccelerationU, derivative.uDotMps2);
  publish(paths::AircraftBodyAccelerationV, derivative.vDotMps2);
  publish(paths::AircraftBodyAccelerationW, derivative.wDotMps2);
  publish(paths::AircraftAngularAccelerationP, derivative.pDotRadPerSec2);
  publish(paths::AircraftAngularAccelerationQ, derivative.qDotRadPerSec2);
  publish(paths::AircraftAngularAccelerationR, derivative.rDotRadPerSec2);
  publish(paths::AircraftControlAileron,
      aircraft.GetControls().GetInput().aileron);
  publish(paths::AircraftControlElevator,
      aircraft.GetControls().GetInput().elevator);
  publish(paths::AircraftControlRudder,
      aircraft.GetControls().GetInput().rudder);
  publish(paths::AircraftControlThrottle,
      aircraft.GetControls().GetInput().throttle);
}
} // namespace telemetry
