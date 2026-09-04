#pragma once

#include "common/Options.hpp"
#include "sim/runtime/SimContracts.hpp"
#include "sim/execution/ExecutionRequest.hpp"
#include "sim/telemetry/TelemetryContracts.hpp"
#include "contract/telemetry/RecordingTypes.hpp"
#include "sim/telemetry/recording/TelemetryRecordingService.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim {
class Simulation;
class ScenarioExecutor;

class SimRuntime {
public:
  SimRuntime(std::unique_ptr<Simulation> primarySimulation,
      std::unique_ptr<Simulation> baselineSimulation = nullptr);
  ~SimRuntime();

  SimRuntime(const SimRuntime &) = delete;
  SimRuntime &operator=(const SimRuntime &) = delete;

  static std::unique_ptr<SimRuntime> CreateForExecution(
      const ResolvedExecutionSpec &execution, std::string &error);

  // Lifetime and stepping
  bool Initialize(
      std::string_view aircraftName = opts::simulation::AircraftName,
      double simulationHz = opts::simulation::Hz);
  void Shutdown();
  void Start();
  void Stop();
  void Pause();
  void Resume();
  bool Reset();
  bool Reset(const InitialCondition &initialCondition);
  void RequestTick();
  bool Tick();

  // Resolved execution
  bool RunExecution(const ResolvedExecutionSpec &execution);
  std::optional<ScenarioExecutionStatus> GetScenarioStatus() const;

  // Scheduling
  void SetAutomaticSimulationHz(double hz);
  double GetAutomaticSimulationHz() const;
  void SetMaximumSimulationSpeedEnabled(bool enabled);
  bool IsMaximumSimulationSpeedEnabled() const;

  // Data contracts
  SimStatus GetStatus() const;
  SimSnapshot GetSnapshot() const;
  std::uint64_t GetTelemetryVersion(SimSlot slot) const;
  telemetry::TelemetryFrame GetLatestTelemetryFrame(SimSlot slot) const;
  telemetry::TelemetrySnapshot GetTelemetrySnapshot(SimSlot slot) const;
  double GetSimulationTimeSec() const;
  std::optional<telemetry::recording::TelemetrySourceFrame>
  CaptureRecordingSource() const;
  std::vector<telemetry::recording::ScenarioEvent> TakeScenarioEvents();

  // External command boundary
  bool SetManualControl(const control::ControlInput &input);
  bool SetPrimaryRollHoldConfig(const PrimaryRollHoldConfig &config);
  bool SetBaselineRollHoldConfig(const BaselineRollHoldConfig &config);
  bool RunTrim(const gnc::TrimRequest &request, bool fromCurrentState);
  bool SetAutomaticLinearizationEnabled(bool enabled);

  // Telemetry recording
  bool StartTelemetryRecording();
  bool StartTelemetryRecording(const std::filesystem::path &path,
      const telemetry::recording::RecordingMetadata &metadata);
  void StopTelemetryRecording();
  telemetry::recording::RecordingStatus GetTelemetryRecordingStatus() const;

private:
  // Simulation coordination
  void FinishScenario();
  void RecordPendingScenarioCommandEvent();
  bool SelectExecutionVariant(ExecutionVariant variant);
  bool ReinitializeForScenario(const SimScenario &scenario);
  void RestoreInteractiveSimulationOrder();
  Simulation *GetSimulation(SimSlot slot);
  const Simulation *GetSimulation(SimSlot slot) const;

  // Owned simulations and services
  std::unique_ptr<Simulation> primarySimulation_;
  std::unique_ptr<Simulation> baselineSimulation_;
  std::unique_ptr<ScenarioExecutor> scenarioExecutor_;
  std::optional<ResolvedExecutionSpec> resolvedExecution_;
  bool scenarioSimulationSwapped_ = false;
  telemetry::recording::TelemetryRecordingService telemetryRecording_;
  std::vector<telemetry::recording::ScenarioEvent> pendingScenarioEvents_;

  // Configuration and execution state
  std::string aircraftName_;
  double simulationHz_ = opts::simulation::Hz;
  SimExecutionState executionState_ = SimExecutionState::Stopped;
  double automaticSimulationHz_ = opts::simulation::Hz;
  bool maximumSimulationSpeedEnabled_ = false;
  std::uint32_t pendingTicks_ = 0;
  bool initialized_ = false;
  std::string lastError_;
};
} // namespace sim
