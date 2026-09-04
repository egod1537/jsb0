#include "gui/features/simulation/SimulationController.hpp"

#include "common/math/Math.hpp"
#include "messaging/SimulationMessageClient.hpp"

namespace gui {
SimulationController::SimulationController(
    application::SimulationMessageClient &client)
    : client_(client) {}

SimulationTransportProps SimulationController::GetTransportProps() const {
  return {
      .executionState = client_.GetSimulationExecutionState(),
      .scenarioStatus = client_.GetScenarioExecutionStatus(),
      .recordingStatus = client_.GetTelemetryRecordingStatus(),
      .automaticHz = client_.GetAutomaticSimulationHz(),
      .pendingTickCount = client_.GetPendingSimulationTickCount(),
      .maximumSpeed = client_.IsMaximumSimulationSpeedEnabled(),
  };
}

void SimulationController::Synchronize(
    const sim::SimulationSnapshot &snapshot) {
  if (initialCondition_.initialized) {
    return;
  }
  initialCondition_.pending = snapshot.defaultInitialCondition;
  initialCondition_.initialized = true;
}

void SimulationController::Handle(const SimulationStartRequested &) {
  client_.StartSimulation();
}

void SimulationController::Handle(const SimulationStopRequested &) {
  client_.StopSimulation();
}

void SimulationController::Handle(const SimulationPlaybackToggled &) {
  if (client_.GetSimulationExecutionState()
      == sim::SimulationExecutionState::Stopped) {
    client_.StartSimulation();
  } else {
    client_.StopSimulation();
  }
}

void SimulationController::Handle(const SimulationPauseRequested &) {
  client_.PauseSimulation();
}

void SimulationController::Handle(const SimulationResumeRequested &) {
  client_.ResumeSimulation();
}

void SimulationController::Handle(const SimulationResetRequested &) {
  ResetSimulation(nullptr);
}

void SimulationController::Handle(const SimulationStepRequested &) {
  client_.RequestSimulationTick();
}

void SimulationController::Handle(const SimulationRateChanged &event) {
  client_.SetAutomaticSimulationHz(event.hz);
}

void SimulationController::Handle(const MaximumSimulationSpeedChanged &event) {
  client_.SetMaximumSimulationSpeedEnabled(event.enabled);
}

void SimulationController::Handle(const TelemetryRecordingToggled &) {
  if (client_.GetTelemetryRecordingStatus().state
      == telemetry::recording::RecordingState::Recording) {
    client_.StopTelemetryRecording();
  } else {
    client_.StartTelemetryRecording();
  }
}

void SimulationController::Handle(const OpenTelemetryFolderRequested &) {
  client_.OpenTelemetryRecordingsFolder();
}

bool SimulationController::Handle(const ScenarioLaunchRequested &event) {
  return client_.RunExecution(event.request);
}

std::optional<std::string> SimulationController::GetLastCommandError() const {
  return client_.GetLastCommandError();
}

void SimulationController::Handle(const InitialConditionFieldChanged &event) {
  double *field = nullptr;
  switch (event.field) {
  case InitialConditionField::LatitudeDeg:
    initialCondition_.pending.latitudeRad = math::DegToRad(event.value);
    return;
  case InitialConditionField::LongitudeDeg:
    initialCondition_.pending.longitudeRad = math::DegToRad(event.value);
    return;
  case InitialConditionField::AltitudeAslM:
    field = &initialCondition_.pending.altitudeAslM;
    break;
  case InitialConditionField::RollDeg:
    initialCondition_.pending.rollRad = math::DegToRad(event.value);
    return;
  case InitialConditionField::PitchDeg:
    initialCondition_.pending.pitchRad = math::DegToRad(event.value);
    return;
  case InitialConditionField::HeadingDeg:
    initialCondition_.pending.headingRad = math::DegToRad(event.value);
    return;
  case InitialConditionField::CalibratedAirspeedMps:
    field = &initialCondition_.pending.calibratedAirspeedMps;
    break;
  }
  *field = event.value;
}

void SimulationController::Handle(
    const UseCurrentInitialConditionRequested &event) {
  initialCondition_.pending = event.current;
}

void SimulationController::Handle(
    const RestoreDefaultInitialConditionRequested &event) {
  initialCondition_.pending = event.defaults;
}

void SimulationController::Handle(const ResetWithInitialConditionRequested &) {
  ResetSimulation(&initialCondition_.pending);
}

void SimulationController::ResetSimulation(
    const sim::InitialCondition *initialCondition) {
  const bool resumeAfterReset = client_.GetSimulationExecutionState()
                                == sim::SimulationExecutionState::Running;
  client_.PauseSimulation();
  const bool reset = initialCondition == nullptr
                         ? client_.ResetSimulation()
                         : client_.ResetSimulation(*initialCondition);
  if (reset && resumeAfterReset) {
    client_.ResumeSimulation();
  }
}
} // namespace gui
