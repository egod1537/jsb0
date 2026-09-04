#pragma once

#include "gui/features/simulation/SimEvents.hpp"
#include "gui/features/simulation/SimModel.hpp"
#include "sim/runtime/SimContracts.hpp"
#include "contract/telemetry/RecordingTypes.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace app {
class SimMessageClient;
}

namespace gui {
struct SimTransportProps {
  sim::SimExecutionState executionState =
      sim::SimExecutionState::Stopped;
  std::optional<sim::ScenarioExecutionStatus> scenarioStatus;
  telemetry::recording::RecordingStatus recordingStatus;
  double automaticHz = 0.0;
  std::uint32_t pendingTickCount = 0;
  bool maximumSpeed = false;
};

class SimController {
public:
  explicit SimController(app::SimMessageClient &client);

  // Immutable state for views
  SimTransportProps GetTransportProps() const;
  const InitialConditionModel &GetInitialConditionModel() const {
    return initialCondition_;
  }
  void Synchronize(const sim::SimSnapshot &snapshot);

  // Transport events
  void OnEvent(const SimStartRequested &event);
  void OnEvent(const SimStopRequested &event);
  void OnEvent(const SimPlaybackToggled &event);
  void OnEvent(const SimPauseRequested &event);
  void OnEvent(const SimResumeRequested &event);
  void OnEvent(const SimResetRequested &event);
  void OnEvent(const SimStepRequested &event);
  void OnEvent(const SimRateChanged &event);
  void OnEvent(const MaximumSimulationSpeedChanged &event);
  void OnEvent(const TelemetryRecordingToggled &event);
  void OnEvent(const OpenTelemetryFolderRequested &event);
  bool OnEvent(const ScenarioLaunchRequested &event);
  std::optional<std::string> GetLastCommandError() const;

  // Initial-condition child events
  void OnEvent(const InitialConditionFieldChanged &event);
  void OnEvent(const UseCurrentInitialConditionRequested &event);
  void OnEvent(const RestoreDefaultInitialConditionRequested &event);
  void OnEvent(const ResetWithInitialConditionRequested &event);

private:
  void ResetSimulation(const sim::InitialCondition *initialCondition);

  // Dependencies
  app::SimMessageClient &client_;

  // Child feature state
  InitialConditionModel initialCondition_;
};
} // namespace gui
