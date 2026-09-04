#pragma once

#include "common/Options.hpp"
#include "contract/telemetry/RecordingTypes.hpp"
#include "sim/AircraftState.hpp"
#include "sim/EngineState.hpp"
#include "sim/FDMState.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/control/ControlInput.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/execution/ExecutionRequest.hpp"
#include "sim/gnc/TrimTypes.hpp"
#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/linearization/DynamicModeHistory.hpp"
#include "sim/linearization/LinearizationResult.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sim {
inline constexpr double MinimumAutomaticSimulationHz = 1.0;
inline constexpr double MaximumAutomaticSimulationHz = 1000.0;

enum class SimExecutionState {
  Running,
  Paused,
  Stopped,
};

enum class SimSlot {
  Primary,
  Baseline,
};

enum class SimCommandType {
  Start,
  Stop,
  Pause,
  Resume,
  Reset,
  TickOnce,
  RunExecution,
  SetAutomaticRate,
  SetMaximumSpeed,
  SetManualControl,
  SetControllerConfig,
};

struct PrimaryRollHoldConfig {
  bool enabled = false;
  double targetRollRad = 0.0;
  double rollAngleProportionalGain = 0.0;
  double rollRateProportionalGain = 0.0;
};

struct BaselineRollHoldConfig {
  // PX4 Roll Hold
  bool enabled = false;
  double targetRollRad = 0.0;
  double timeConstantSec =
      gnc::GetC172xPx4ControlProfile().roll.timeConstantSec;
  double maximumRollRateRadPerSec =
      gnc::GetC172xPx4ControlProfile().roll.maximumRollRateRadPerSec;
  double rateProportionalGain =
      gnc::GetC172xPx4ControlProfile().roll.rateProportionalGain;
  double rateIntegralGain =
      gnc::GetC172xPx4ControlProfile().roll.rateIntegralGain;
  double rateDerivativeGain =
      gnc::GetC172xPx4ControlProfile().roll.rateDerivativeGain;
  double rateFeedForwardGain =
      gnc::GetC172xPx4ControlProfile().roll.rateFeedForwardGain;
  double integratorLimit =
      gnc::GetC172xPx4ControlProfile().roll.integratorLimit;

  // PX4 Pitch Hold
  bool pitchHoldEnabled = false;
  double targetPitchRad = 0.0;
  double pitchTimeConstantSec =
      gnc::GetC172xPx4ControlProfile().pitch.timeConstantSec;
  double maximumPositivePitchRateRadPerSec =
      gnc::GetC172xPx4ControlProfile().pitch.maximumPositivePitchRateRadPerSec;
  double maximumNegativePitchRateRadPerSec =
      gnc::GetC172xPx4ControlProfile().pitch.maximumNegativePitchRateRadPerSec;
  double pitchRateProportionalGain =
      gnc::GetC172xPx4ControlProfile().pitch.rateProportionalGain;
  double pitchRateIntegralGain =
      gnc::GetC172xPx4ControlProfile().pitch.rateIntegralGain;
  double pitchRateDerivativeGain =
      gnc::GetC172xPx4ControlProfile().pitch.rateDerivativeGain;
  double pitchRateFeedForwardGain =
      gnc::GetC172xPx4ControlProfile().pitch.rateFeedForwardGain;
  double pitchIntegratorLimit =
      gnc::GetC172xPx4ControlProfile().pitch.integratorLimit;

  // PX4 total-energy longitudinal outer loop
  bool tecsEnabled = false;
  double targetAltitudeM = 304.8;
  double targetAirspeedMps = 41.1556;
  gnc::Px4TecsSettings tecsSettings = gnc::GetC172xPx4ControlProfile().tecs;

  // Temporary direct-rate tuning bypass
  bool directRollRateTestEnabled = false;
  double directRollRateCommandRadPerSec = 0.0;

  // PX4 course/lateral outer loop
  bool courseHoldEnabled = false;
  double targetCourseRad = 0.0;
  double courseGuidancePeriodSec =
      gnc::GetC172xPx4ControlProfile().course.guidancePeriodSec;
  double courseGuidanceDampingRatio =
      gnc::GetC172xPx4ControlProfile().course.guidanceDampingRatio;
  double courseMaxRollRad = gnc::GetC172xPx4ControlProfile().course.maxRollRad;
  double courseMaxRollSetpointRateRadPerSec =
      gnc::GetC172xPx4ControlProfile().course.maxRollSetpointRateRadPerSec;

  // Experimental PX4 yaw-rate and sideslip augmentation
  bool yawRateControlEnabled = false;
  bool coordinatedTurnEnabled = true;
  double maximumYawRateRadPerSec =
      gnc::GetC172xPx4ControlProfile().yaw.maximumYawRateRadPerSec;
  double yawRateProportionalGain =
      gnc::GetC172xPx4ControlProfile().yaw.rateProportionalGain;
  double yawRateIntegralGain =
      gnc::GetC172xPx4ControlProfile().yaw.rateIntegralGain;
  double yawRateDerivativeGain =
      gnc::GetC172xPx4ControlProfile().yaw.rateDerivativeGain;
  double yawRateFeedForwardGain =
      gnc::GetC172xPx4ControlProfile().yaw.rateFeedForwardGain;
  double yawIntegratorLimit =
      gnc::GetC172xPx4ControlProfile().yaw.integratorLimit;
  double sideslipToYawRateGain =
      gnc::GetC172xPx4ControlProfile().yaw.sideslipToYawRateGain;
  double yawRateWashoutTimeConstantSec =
      gnc::GetC172xPx4ControlProfile().yaw.yawRateWashoutTimeConstantSec;
  double rollToYawFeedForwardGain =
      gnc::GetC172xPx4ControlProfile().yaw.rollToYawFeedForwardGain;
};

struct ControllerConfig {
  PrimaryRollHoldConfig primary;
  BaselineRollHoldConfig baseline;
};

struct SimCommand {
  SimCommandType type = SimCommandType::TickOnce;
  SimSlot slot = SimSlot::Primary;
  double value = 0.0;
  bool enabled = false;
  control::ControlInput manualControl;
  ControllerConfig controller;
};

struct BaselineRollHoldDiagnostics {
  double aileronCommand = 0.0;
  double bodyRateSetpointRadPerSec = 0.0;
  double rollErrorRad = 0.0;
  double airspeedScaling = 1.0;
};

struct BaselineCourseHoldDiagnostics {
  bool outputValid = false;
  bool groundSpeedValid = false;
  double targetCourseRad = 0.0;
  double currentCourseRad = 0.0;
  double courseErrorRad = 0.0;
  double groundSpeedMps = 0.0;
  double rawRollSetpointRad = 0.0;
  double limitedRollSetpointRad = 0.0;
  bool rollLimited = false;
  bool rollSetpointRateLimited = false;
};

struct BaselinePitchHoldDiagnostics {
  bool outputValid = false;
  double elevatorCommand = 0.0;
  double bodyRateSetpointRadPerSec = 0.0;
  double pitchErrorRad = 0.0;
  double airspeedScaling = 1.0;
};

struct BaselineTecsDiagnostics {
  bool outputValid = false;
  double internalAltitudeSetpointM = 0.0;
  double targetPitchRad = 0.0;
  double targetThrottle = 0.0;
  double totalEnergyError = 0.0;
  double energyBalanceError = 0.0;
  bool underspeedProtectionActive = false;
  bool overspeedProtectionActive = false;
};

struct AutopilotSnapshot {
  bool available = false;
  std::string strategyName;
  control::FlightControlMode mode = control::FlightControlMode::Manual;
  control::ControlInput manualControl;
  PrimaryRollHoldConfig primaryRollHold;
  BaselineRollHoldConfig baselineRollHold;
  BaselineRollHoldDiagnostics baselineDiagnostics;
  BaselinePitchHoldDiagnostics baselinePitchDiagnostics;
  BaselineTecsDiagnostics baselineTecsDiagnostics;
  BaselineCourseHoldDiagnostics baselineCourseDiagnostics;
};

struct TrimSnapshot {
  std::optional<gnc::TrimResult> result;
};

struct LinearizationSnapshot {
  bool available = false;
  bool automaticUpdatesEnabled = false;
  bool updateInProgress = false;
  std::string errorMessage;
  std::optional<gnc::LinearizationResult> result;
  gnc::DynamicModeHistory dynamicModeHistory;
};

struct ScenarioExecutionStatus {
  std::string name;
  double elapsedSec = 0.0;
  double durationSec = 0.0;
};

struct SimStatus {
  SimExecutionState executionState = SimExecutionState::Stopped;
  std::optional<ScenarioExecutionStatus> scenario;
  double automaticSimulationHz = opts::simulation::Hz;
  bool maximumSimulationSpeedEnabled = false;
  std::uint32_t pendingTickCount = 0;
  bool initialized = false;
  bool baselineAvailable = false;
  std::string lastError;
};

struct SimInstanceSnapshot {
  AircraftState aircraft;
  AircraftStateDerivative aircraftDerivative;
  FDMState fdmState;
  control::ControlInput controlInput;
  std::vector<EngineState> engines;
  InitialCondition currentCondition;
  double pitchTrim = 0.0;
  bool available = false;
};

struct SimSnapshot {
  SimStatus status;
  std::string aircraftName = std::string(opts::simulation::AircraftName);
  double simulationHz = opts::simulation::Hz;
  std::optional<ResolvedExecutionSpec> appliedExecution;
  InitialCondition defaultInitialCondition;
  SimInstanceSnapshot primary;
  std::optional<SimInstanceSnapshot> baseline;
  AutopilotSnapshot primaryAutopilot;
  std::optional<AutopilotSnapshot> baselineAutopilot;
  TrimSnapshot trim;
  LinearizationSnapshot linearization;
  telemetry::recording::RecordingStatus telemetryRecording;
};

inline const char *ToString(SimExecutionState state) {
  switch (state) {
  case SimExecutionState::Running:
    return "Running";
  case SimExecutionState::Paused:
    return "Paused";
  case SimExecutionState::Stopped:
    return "Stopped";
  }

  return "Unknown";
}
} // namespace sim
