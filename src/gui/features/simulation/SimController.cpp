#include "gui/features/simulation/SimController.hpp"

#include "common/math/Math.hpp"
#include "messaging/SimMessageClient.hpp"

#include <utility>

namespace gui {
SimController::SimController(app::SimMessageClient &client) : client_(client) {}

SimTransportProps SimController::GetTransportProps() const {
  return {
      .executionState = client_.GetSimExecutionState(),
      .scenarioStatus = client_.GetScenarioExecutionStatus(),
      .recordingStatus = client_.GetTelemetryRecordingStatus(),
      .automaticHz = client_.GetAutomaticSimulationHz(),
      .pendingTickCount = client_.GetPendingSimTickCount(),
      .maximumSpeed = client_.IsMaximumSimulationSpeedEnabled(),
  };
}

void SimController::Synchronize(const sim::SimSnapshot &snapshot) {
  if (initialCondition_.initialized) {
    return;
  }
  initialCondition_.pending = snapshot.defaultInitialCondition;
  initialCondition_.initialized = true;
}

void SimController::OnEvent(const SimStartRequested &) {
  client_.StartSimulation();
}

void SimController::OnEvent(const SimStopRequested &) {
  client_.StopSimulation();
}

void SimController::OnEvent(const SimPlaybackToggled &) {
  if (client_.GetSimExecutionState() == sim::SimExecutionState::Stopped) {
    client_.StartSimulation();
  } else {
    client_.StopSimulation();
  }
}

void SimController::OnEvent(const SimPauseRequested &) {
  client_.PauseSimulation();
}

void SimController::OnEvent(const SimResumeRequested &) {
  client_.ResumeSimulation();
}

void SimController::OnEvent(const SimResetRequested &) {
  ResetSimulation(nullptr);
}

void SimController::OnEvent(const SimStepRequested &) {
  client_.RequestSimTick();
}

void SimController::OnEvent(const SimRateChanged &event) {
  client_.SetAutomaticSimulationHz(event.hz);
}

void SimController::OnEvent(const MaximumSimulationSpeedChanged &event) {
  client_.SetMaximumSimulationSpeedEnabled(event.enabled);
}

void SimController::OnEvent(const TelemetryRecordingToggled &) {
  if (client_.GetTelemetryRecordingStatus().state
      == telemetry::recording::RecordingState::Recording) {
    client_.StopTelemetryRecording();
  } else {
    client_.StartTelemetryRecording();
  }
}

void SimController::OnEvent(const OpenTelemetryFolderRequested &) {
  client_.OpenTelemetryRecordingsFolder();
}

bool SimController::OnEvent(const ScenarioLaunchRequested &event,
    std::function<void(bool, const std::string &)> completion) {
  return client_.RunExecution(event.request, std::move(completion));
}

std::optional<std::string> SimController::GetLastCommandError() const {
  return client_.GetLastCommandError();
}

void SimController::OnEvent(const InitialConditionFieldChanged &event) {
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

void SimController::OnEvent(const UseCurrentInitialConditionRequested &event) {
  initialCondition_.pending = event.current;
}

void SimController::OnEvent(
    const RestoreDefaultInitialConditionRequested &event) {
  initialCondition_.pending = event.defaults;
}

void SimController::OnEvent(const ResetWithInitialConditionRequested &) {
  ResetSimulation(&initialCondition_.pending);
}

void SimController::ResetSimulation(
    const sim::InitialCondition *initialCondition) {
  const bool resumeAfterReset =
      client_.GetSimExecutionState() == sim::SimExecutionState::Running;
  client_.PauseSimulation();
  auto resume = [this, resumeAfterReset](bool succeeded, const std::string &) {
    if (succeeded && resumeAfterReset) {
      client_.ResumeSimulation();
    }
  };
  if (initialCondition == nullptr) {
    client_.ResetSimulation(std::move(resume));
  } else {
    client_.ResetSimulation(*initialCondition, std::move(resume));
  }
}
} // namespace gui
