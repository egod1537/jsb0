#pragma once

#include "RunnerOptions.hpp"
#include "contract/telemetry/RecordingTypes.hpp"

#include <csignal>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace runner {
enum class RunnerExitCode : int {
  Success = 0,
  GeneralFailure = 1,
  InvalidArguments = 2,
  ScenarioLoadFailure = 3,
  SimInitializationFailure = 4,
  SimExecutionFailure = 5,
  OutputFailure = 6,
};

struct RunnerResult {
  RunnerExitCode exitCode = RunnerExitCode::GeneralFailure;
  std::string status = "failed";
  std::string endedAt;
  double simulationTimeSec = 0.0;
  double wallTimeSec = 0.0;
  double realtimeFactor = 0.0;
  std::uint64_t steps = 0;
  std::string error;
  std::string baselineStatus;
  std::string baselineError;
  std::string primaryStatus;
  std::string primaryError;
};

struct SimRunInfo {
  std::string scenarioName;
  std::string scenarioFile;
  std::string scenarioDigest;
  std::uint32_t scenarioSchemaVersion = 1;
  std::string scenarioType = "roll_hold";
  std::string aircraft = "c172x";
  std::string parameterFile;
  std::string parameterDigest;
  std::string startedAt;
  std::filesystem::path outputDirectory;
  double dtSec = 0.0;
  double durationSec = 0.0;
};

struct SimRunObservation {
  telemetry::recording::TelemetryFrame telemetry;
  std::vector<telemetry::recording::ScenarioEvent> scenarioEvents;
};

class ISimRunObserver {
public:
  virtual ~ISimRunObserver() = default;

  // Run lifecycle
  virtual bool OnRunStarted(const SimRunInfo &info,
      const SimRunObservation &observation, std::string &error) = 0;
  virtual bool OnSimStep(const SimRunInfo &info,
      const SimRunObservation &observation, std::string &error) = 0;
  virtual bool OnRunFinished(const SimRunInfo &info,
      const RunnerResult &result, std::string &error) = 0;
};

class SimRunner {
public:
  void AddObserver(ISimRunObserver &observer);

  RunnerResult Run(const RunnerOptions &options,
      const volatile std::sig_atomic_t *running = nullptr);

private:
  std::vector<ISimRunObserver *> observers_;
};
} // namespace runner
