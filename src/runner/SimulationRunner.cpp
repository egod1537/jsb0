#include "SimulationRunner.hpp"

#include "common/crypto/Sha256.hpp"
#include "sim/runtime/SimulationRuntime.hpp"
#include "sim/scenario/SimulationScenarioSerializer.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <system_error>

namespace runner {
namespace {
using Clock = std::chrono::steady_clock;

std::string GetWallClockTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string JsonEscape(std::string_view value) {
  std::string escaped;
  for (const char character : value) {
    switch (character) {
    case '\\':
      escaped += "\\\\";
      break;
    case '"':
      escaped += "\\\"";
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    default:
      escaped.push_back(character);
      break;
    }
  }
  return escaped;
}

bool PrepareOutputDirectory(const std::filesystem::path &directory,
    std::string &error) {
  if (directory.empty()) {
    error = "output directory is empty";
    return false;
  }
  std::error_code filesystemError;
  std::filesystem::create_directories(directory, filesystemError);
  if (filesystemError || !std::filesystem::is_directory(directory)) {
    error =
        "output directory is not writable: "
        + (filesystemError ? filesystemError.message() : directory.string());
    return false;
  }
  const std::filesystem::path probe = directory / ".jsb-runner-write-test";
  std::ofstream output(probe, std::ios::binary | std::ios::trunc);
  if (!output) {
    error = "output directory is not writable: " + directory.string();
    return false;
  }
  output << "ok\n";
  output.close();
  std::filesystem::remove(probe, filesystemError);
  return true;
}

bool WriteManifest(const std::filesystem::path &directory,
    const SimulationRunInfo &info, const RunnerResult &result,
    std::string &error) {
  std::ofstream output(directory / "run.json",
      std::ios::binary | std::ios::trunc);
  if (!output) {
    error = "could not open run.json for writing";
    return false;
  }
  output << std::setprecision(17)
         << "{\n"
            "  \"contract_version\": \""
         << JSB_CONTRACT_VERSION
         << "\",\n"
            "  \"runtime\": {\n"
            "    \"branch\": \""
         << JsonEscape(JSB_RUNTIME_BRANCH)
         << "\",\n"
            "    \"commit\": \""
         << JsonEscape(JSB_GIT_COMMIT)
         << "\",\n"
            "    \"application_version\": \""
         << JsonEscape(JSB_APPLICATION_VERSION)
         << "\"\n"
            "  },\n"
            "  \"scenario\": {\n"
            "    \"name\": \""
         << JsonEscape(info.scenarioName)
         << "\",\n"
            "    \"file\": \""
         << JsonEscape(info.scenarioFile)
         << "\",\n"
            "    \"digest_sha256\": \""
         << JsonEscape(info.scenarioDigest)
         << "\",\n"
            "    \"schema_version\": "
         << info.scenarioSchemaVersion
         << ",\n"
            "    \"scenario_type\": \""
         << JsonEscape(info.scenarioType)
         << "\"\n"
            "  },\n"
            "  \"telemetry_schema_version\": "
         << JSB_TELEMETRY_SCHEMA_VERSION
         << ",\n"
            "  \"aircraft\": \""
         << JsonEscape(info.aircraft)
         << "\",\n"
            "  \"autopilot\": \""
         << gnc::ToString(info.autopilot)
         << "\",\n"
            "  \"started_at\": \""
         << JsonEscape(info.startedAt)
         << "\",\n"
            "  \"ended_at\": \""
         << JsonEscape(result.endedAt)
         << "\",\n"
            "  \"duration_s\": "
         << info.durationSec
         << ",\n"
            "  \"status\": \""
         << JsonEscape(result.status)
         << "\",\n"
            "  \"simulation_dt_s\": "
         << info.dtSec
         << ",\n"
            "  \"simulation_time_s\": "
         << result.simulationTimeSec
         << ",\n"
            "  \"wall_time_s\": "
         << result.wallTimeSec
         << ",\n"
            "  \"realtime_factor\": "
         << result.realtimeFactor
         << ",\n"
            "  \"steps\": "
         << result.steps;
  if (!result.error.empty()) {
    output << ",\n"
              "  \"error\": \""
           << JsonEscape(result.error) << '"';
  }
  output << "\n}\n";
  if (!output) {
    error = "could not write run.json";
    return false;
  }
  return true;
}

void SetOutputFailure(RunnerResult &result, std::string error) {
  result.status = "failed";
  result.exitCode = RunnerExitCode::OutputFailure;
  if (result.error.empty()) {
    result.error = std::move(error);
  } else if (!error.empty() && result.error != error) {
    result.error += "; ";
    result.error += error;
  }
}

void WriteFinalManifest(const SimulationRunInfo &info, RunnerResult &result) {
  if (result.endedAt.empty()) {
    result.endedAt = GetWallClockTimestamp();
  }
  std::string error;
  if (!WriteManifest(info.outputDirectory, info, result, error)) {
    SetOutputFailure(result, std::move(error));
  }
}
} // namespace

void SimulationRunner::AddObserver(ISimulationRunObserver &observer) {
  observers_.push_back(&observer);
}

RunnerResult SimulationRunner::Run(const RunnerOptions &options,
    const volatile std::sig_atomic_t *running) {
  RunnerResult result;
  std::string error;
  if (!PrepareOutputDirectory(options.outputDirectory, error)) {
    result.exitCode = RunnerExitCode::OutputFailure;
    result.error = std::move(error);
    return result;
  }

  std::error_code pathError;
  const std::filesystem::path absoluteScenarioPath =
      std::filesystem::absolute(options.scenarioPath, pathError);
  SimulationRunInfo info{
      .scenarioName = options.scenarioPath.stem().string(),
      .scenarioFile =
          (pathError ? options.scenarioPath : absoluteScenarioPath).string(),
      .startedAt = GetWallClockTimestamp(),
      .outputDirectory = options.outputDirectory,
  };

  {
    std::ifstream scenarioInput(options.scenarioPath, std::ios::binary);
    if (scenarioInput) {
      std::ostringstream bytes;
      bytes << scenarioInput.rdbuf();
      if (!scenarioInput.bad()) {
        info.scenarioDigest = common::crypto::Sha256Hex(bytes.str());
      }
    }
  }

  sim::SimulationScenario scenario;
  if (!sim::SimulationScenarioSerializer::Load(options.scenarioPath,
          scenario,
          error)) {
    result.exitCode = RunnerExitCode::ScenarioLoadFailure;
    result.error = std::move(error);
    WriteFinalManifest(info, result);
    return result;
  }
  scenario.sourceFile = info.scenarioFile;
  scenario.sourceDigestSha256 = info.scenarioDigest;
  if (!sim::ValidateSimulationScenario(scenario, &error)) {
    result.exitCode = RunnerExitCode::ScenarioLoadFailure;
    result.error = std::move(error);
    info.scenarioName = scenario.name;
    info.durationSec = scenario.durationSec;
    WriteFinalManifest(info, result);
    return result;
  }

  info.scenarioName = scenario.name;
  info.scenarioSchemaVersion =
      static_cast<std::uint32_t>(scenario.schemaVersion);
  info.scenarioType = scenario.scenarioType;
  info.aircraft = scenario.aircraft;
  info.autopilot = scenario.autopilot;
  info.dtSec = scenario.dtSec;
  info.durationSec = scenario.durationSec;
  std::unique_ptr<sim::SimulationRuntime> runtimeOwner =
      sim::SimulationRuntime::CreateForScenario(scenario, error);
  if (runtimeOwner == nullptr) {
    result.exitCode = RunnerExitCode::SimulationInitializationFailure;
    result.error = std::move(error);
    WriteFinalManifest(info, result);
    return result;
  }
  sim::SimulationRuntime &runtime = *runtimeOwner;

  if (!runtime.RunScenario(scenario)) {
    result.exitCode = RunnerExitCode::SimulationInitializationFailure;
    result.error = runtime.GetStatus().lastError;
    runtime.Shutdown();
    WriteFinalManifest(info, result);
    return result;
  }

  std::vector<ISimulationRunObserver *> startedObservers;
  for (ISimulationRunObserver *observer : observers_) {
    std::string observerError;
    if (!observer->OnRunStarted(info, runtime, observerError)) {
      result.exitCode = RunnerExitCode::OutputFailure;
      result.error = observerError.empty() ? "run observer failed to start"
                                           : std::move(observerError);
      runtime.Stop();
      result.simulationTimeSec = 0.0;
      for (ISimulationRunObserver *startedObserver : startedObservers) {
        observerError.clear();
        if (!startedObserver->OnRunFinished(info,
                runtime,
                result,
                observerError)) {
          SetOutputFailure(result,
              observerError.empty() ? "run observer failed to finalize"
                                    : std::move(observerError));
        }
      }
      runtime.Shutdown();
      WriteFinalManifest(info, result);
      return result;
    }
    startedObservers.push_back(observer);
  }

  std::cout << "[runner] scenario: " << scenario.name << '\n'
            << "[runner] autopilot: " << gnc::ToString(info.autopilot) << '\n'
            << "[runner] dt: " << std::fixed << std::setprecision(6)
            << scenario.dtSec << '\n'
            << "[runner] duration: " << std::setprecision(3)
            << scenario.durationSec << '\n'
            << "[runner] starting\n";
  const Clock::time_point start = Clock::now();
  while (runtime.GetScenarioStatus().has_value()) {
    if (running != nullptr && *running == 0) {
      result.status = "interrupted";
      result.exitCode = RunnerExitCode::GeneralFailure;
      result.error = "interrupted by signal";
      runtime.Stop();
      break;
    }
    if (!runtime.Tick()) {
      result.status = "failed";
      result.exitCode = RunnerExitCode::SimulationExecutionFailure;
      result.error = runtime.GetStatus().lastError;
      runtime.Stop();
      break;
    }
    ++result.steps;
    for (ISimulationRunObserver *observer : startedObservers) {
      std::string observerError;
      if (!observer->OnSimulationStep(info, runtime, observerError)) {
        result.status = "failed";
        result.exitCode = RunnerExitCode::OutputFailure;
        result.error = observerError.empty()
                           ? "run observer failed to consume simulation step"
                           : std::move(observerError);
        runtime.Stop();
        break;
      }
    }
    if (result.exitCode == RunnerExitCode::OutputFailure) {
      break;
    }
  }
  const std::chrono::duration<double> wallDuration = Clock::now() - start;
  result.wallTimeSec = wallDuration.count();
  result.simulationTimeSec = std::min(scenario.durationSec,
      static_cast<double>(result.steps) * scenario.dtSec);
  result.realtimeFactor = result.wallTimeSec > 0.0
                              ? result.simulationTimeSec / result.wallTimeSec
                              : 0.0;
  if (result.error.empty()) {
    result.status = "completed";
    result.exitCode = RunnerExitCode::Success;
  }
  for (ISimulationRunObserver *observer : startedObservers) {
    std::string observerError;
    if (!observer->OnRunFinished(info, runtime, result, observerError)) {
      SetOutputFailure(result,
          observerError.empty() ? "run observer failed to finalize"
                                : std::move(observerError));
    }
  }
  runtime.Shutdown();

  WriteFinalManifest(info, result);
  std::cout << "[runner] " << result.status << '\n'
            << "[runner] sim time: " << std::fixed << std::setprecision(2)
            << result.simulationTimeSec << " s\n"
            << "[runner] wall time: " << std::setprecision(2)
            << result.wallTimeSec << " s\n"
            << "[runner] speed: " << std::setprecision(2)
            << result.realtimeFactor << "x realtime\n";
  return result;
}
} // namespace runner
