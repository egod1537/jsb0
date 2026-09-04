#include "sim/runtime/AutopilotConfigurationService.hpp"

#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/execution/ExecutionRequest.hpp"
#include "sim/gnc/autopilot/MyAutopilot.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/gnc/control/attitude/Px4RollParameterMetadata.hpp"
#include "sim/runtime/SimContracts.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
bool AutopilotConfigurationService::ApplyPrimary(Simulation &simulation,
    const PrimaryRollHoldConfig &config, bool &tuningChanged) {
  tuningChanged = false;
  auto &manager = simulation.GetFlightControlManager();
  auto *autopilot = dynamic_cast<gnc::MyAutopilot *>(&manager.GetAutopilot());
  if (autopilot == nullptr) {
    return false;
  }

  const gnc::RollHoldSettings previous = autopilot->GetRollHoldSettings();
  gnc::RollHoldSettings settings = previous;
  settings.targetRollRad = config.targetRollRad;
  settings.attitudeLoop.proportionalGain = config.rollAngleProportionalGain;
  settings.rateLoop.proportionalGain = config.rollRateProportionalGain;
  autopilot->SetRollHoldSettings(settings);
  autopilot->SetRollHoldEnabled(config.enabled);
  manager.SetMode(config.enabled ? control::FlightControlMode::Autopilot
                                 : control::FlightControlMode::Manual);

  tuningChanged = previous.attitudeLoop.proportionalGain
                      != settings.attitudeLoop.proportionalGain
                  || previous.rateLoop.proportionalGain
                         != settings.rateLoop.proportionalGain;
  return true;
}

bool AutopilotConfigurationService::ApplyBaseline(Simulation &simulation,
    const BaselineRollHoldConfig &config, bool &tuningChanged) {
  tuningChanged = false;
  auto &manager = simulation.GetFlightControlManager();
  auto *autopilot = dynamic_cast<gnc::PX4Autopilot *>(&manager.GetAutopilot());
  if (autopilot == nullptr) {
    return false;
  }

  const gnc::Px4RollHoldReferenceSettings previous =
      autopilot->GetRollHoldSettings();
  gnc::Px4RollHoldReferenceSettings settings = previous;
  settings.timeConstantSec = config.timeConstantSec;
  settings.maximumRollRateRadPerSec = config.maximumRollRateRadPerSec;
  settings.rateProportionalGain = config.rateProportionalGain;
  settings.rateIntegralGain = config.rateIntegralGain;
  settings.rateDerivativeGain = config.rateDerivativeGain;
  settings.rateFeedForwardGain = config.rateFeedForwardGain;
  settings.integratorLimit = config.integratorLimit;
  settings.directRollRateTestEnabled = config.directRollRateTestEnabled;
  settings.directRollRateCommandRadPerSec =
      config.directRollRateCommandRadPerSec;
  autopilot->SetRollHoldSettings(settings);

  gnc::Px4PitchHoldSettings pitchSettings = autopilot->GetPitchHoldSettings();
  pitchSettings.timeConstantSec = config.pitchTimeConstantSec;
  pitchSettings.maximumPositivePitchRateRadPerSec =
      config.maximumPositivePitchRateRadPerSec;
  pitchSettings.maximumNegativePitchRateRadPerSec =
      config.maximumNegativePitchRateRadPerSec;
  pitchSettings.rateProportionalGain = config.pitchRateProportionalGain;
  pitchSettings.rateIntegralGain = config.pitchRateIntegralGain;
  pitchSettings.rateDerivativeGain = config.pitchRateDerivativeGain;
  pitchSettings.rateFeedForwardGain = config.pitchRateFeedForwardGain;
  pitchSettings.integratorLimit = config.pitchIntegratorLimit;
  autopilot->SetPitchHoldSettings(pitchSettings);
  autopilot->SetTargetPitchRad(config.targetPitchRad);

  autopilot->SetTecsSettings(config.tecsSettings);
  autopilot->SetTargetAltitudeM(config.targetAltitudeM);
  autopilot->SetTargetAirspeedMps(config.targetAirspeedMps);

  gnc::Px4CourseHoldSettings courseSettings =
      autopilot->GetCourseHoldSettings();
  courseSettings.guidancePeriodSec = config.courseGuidancePeriodSec;
  courseSettings.guidanceDampingRatio = config.courseGuidanceDampingRatio;
  courseSettings.maxRollRad = config.courseMaxRollRad;
  courseSettings.maxRollSetpointRateRadPerSec =
      config.courseMaxRollSetpointRateRadPerSec;
  autopilot->SetCourseHoldSettings(courseSettings);
  autopilot->SetTargetCourseRad(config.targetCourseRad);

  gnc::Px4YawRateSettings yawSettings = autopilot->GetYawRateSettings();
  yawSettings.setpointMode = config.coordinatedTurnEnabled
                                 ? gnc::Px4YawRateSetpointMode::CoordinatedTurn
                                 : gnc::Px4YawRateSetpointMode::DampingOnly;
  yawSettings.maximumYawRateRadPerSec = config.maximumYawRateRadPerSec;
  yawSettings.rateProportionalGain = config.yawRateProportionalGain;
  yawSettings.rateIntegralGain = config.yawRateIntegralGain;
  yawSettings.rateDerivativeGain = config.yawRateDerivativeGain;
  yawSettings.rateFeedForwardGain = config.yawRateFeedForwardGain;
  yawSettings.integratorLimit = config.yawIntegratorLimit;
  yawSettings.sideslipToYawRateGain = config.sideslipToYawRateGain;
  yawSettings.yawRateWashoutTimeConstantSec =
      config.yawRateWashoutTimeConstantSec;
  yawSettings.rollToYawFeedForwardGain = config.rollToYawFeedForwardGain;
  autopilot->SetYawRateSettings(yawSettings);
  autopilot->SetYawRateControlEnabled(config.yawRateControlEnabled);
  autopilot->SetTargetRollRad(config.targetRollRad);
  autopilot->SetRollHoldEnabled(config.enabled);
  autopilot->SetPitchHoldEnabled(config.pitchHoldEnabled);
  autopilot->SetTecsEnabled(config.tecsEnabled);
  autopilot->SetCourseHoldEnabled(config.courseHoldEnabled);
  manager.SetMode(config.enabled || config.pitchHoldEnabled
                          || config.tecsEnabled || config.courseHoldEnabled
                          || config.yawRateControlEnabled
                      ? control::FlightControlMode::Autopilot
                      : control::FlightControlMode::Manual);

  tuningChanged =
      previous.timeConstantSec != settings.timeConstantSec
      || previous.maximumRollRateRadPerSec != settings.maximumRollRateRadPerSec
      || previous.rateProportionalGain != settings.rateProportionalGain
      || previous.rateIntegralGain != settings.rateIntegralGain
      || previous.rateDerivativeGain != settings.rateDerivativeGain
      || previous.rateFeedForwardGain != settings.rateFeedForwardGain
      || previous.integratorLimit != settings.integratorLimit;
  return true;
}

bool AutopilotConfigurationService::ApplyExecutionParameters(
    Simulation &simulation, const ResolvedExecutionSpec &execution,
    std::string &error) {
  if (execution.parameters.empty()) {
    return true;
  }
  if (execution.variant != ExecutionVariant::Baseline) {
    error = "controller parameters are not supported by execution variant '"
            + std::string(ToString(execution.variant)) + "'";
    return false;
  }
  auto &manager = simulation.GetFlightControlManager();
  auto *autopilot = dynamic_cast<gnc::PX4Autopilot *>(&manager.GetAutopilot());
  if (autopilot == nullptr) {
    error = "baseline controller parameters require PX4Autopilot";
    return false;
  }

  gnc::Px4RollHoldReferenceSettings settings = autopilot->GetRollHoldSettings();
  for (const auto &[id, value] : execution.parameters) {
    const auto declared =
        std::find(execution.scenario.controllerParameters.begin(),
            execution.scenario.controllerParameters.end(),
            id);
    if (declared == execution.scenario.controllerParameters.end()) {
      error =
          "controller parameter '" + id + "' is not declared by the Scenario";
      return false;
    }
    const auto metadata = std::find_if(gnc::Px4RollHoldParameters.begin(),
        gnc::Px4RollHoldParameters.end(),
        [&](const gnc::Px4RollHoldParameterMetadata &item) {
          return item.id == id;
        });
    if (metadata == gnc::Px4RollHoldParameters.end()) {
      error = "unsupported baseline controller parameter '" + id + "'";
      return false;
    }
    if (!std::isfinite(value) || value < metadata->minimum
        || value > metadata->maximum) {
      error = "controller parameter '" + id + "' is outside ["
              + std::to_string(metadata->minimum) + ", "
              + std::to_string(metadata->maximum) + "]";
      return false;
    }
    gnc::SetPx4RollHoldParameterValue(settings, metadata->parameter, value);
  }
  autopilot->SetRollHoldSettings(settings);
  return true;
}
} // namespace sim
