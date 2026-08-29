#include "runner/McapRunObserver.hpp"
#include "runner/SimulationRunner.hpp"

#include "contract/telemetry/mcap/McapRecordingReader.hpp"
#include "contract/telemetry/mcap/McapTelemetrySchema.hpp"
#include "sim/scenario/SimulationScenarioSerializer.hpp"
#include "telemetry/simulation.pb.h"

#include <google/protobuf/descriptor.pb.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using runner::McapRunObserver;
using runner::RunnerExitCode;
using runner::RunnerOptions;
using runner::RunnerResult;
using runner::SimulationRunner;
using telemetry::recording::McapRecordingReader;
using telemetry::recording::RecordedSample;

void Require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    const auto suffix =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path()
            / ("jsb-runner-mcap-test-" + std::to_string(suffix));
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path &GetPath() const { return path_; }

private:
  std::filesystem::path path_;
};

std::string ReadTextFile(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  Require(input.is_open(), "failed to open " + path.string());
  return std::string(std::istreambuf_iterator<char>(input),
      std::istreambuf_iterator<char>());
}

RunnerOptions MakeOptions(const std::filesystem::path &output) {
  RunnerOptions options;
  options.scenarioPath = JSB_TEST_HEADLESS_SCENARIO_PATH;
  options.outputDirectory = output;
  return options;
}

RunnerResult RunWithMcap(const RunnerOptions &options,
    const volatile std::sig_atomic_t *running = nullptr) {
  McapRunObserver observer;
  SimulationRunner runner;
  runner.AddObserver(observer);
  return runner.Run(options, running);
}

void RequireMonotonicSimulationTimestamps(
    const std::vector<RecordedSample> &samples, std::string_view topic) {
  Require(!samples.empty(), std::string(topic) + " has no samples");
  for (std::size_t index = 0; index < samples.size(); ++index) {
    Require(samples[index].logTimeNanoseconds
                == samples[index].publishTimeNanoseconds,
        std::string(topic) + " log/publish timestamps differ");
    if (index > 0) {
      Require(samples[index - 1].logTimeNanoseconds
                  < samples[index].logTimeNanoseconds,
          std::string(topic) + " timestamps are not strictly monotonic");
    }
  }
}

void RequireSameSamples(const std::vector<RecordedSample> &first,
    const std::vector<RecordedSample> &second, std::string_view topic) {
  Require(first.size() == second.size(),
      std::string(topic) + " sample count is not deterministic");
  for (std::size_t index = 0; index < first.size(); ++index) {
    Require(first[index].logTimeNanoseconds == second[index].logTimeNanoseconds
                && first[index].publishTimeNanoseconds
                       == second[index].publishTimeNanoseconds
                && first[index].payload == second[index].payload,
        std::string(topic) + " samples are not deterministic");
  }
}

void TestSuccessfulRunArtifactsAndSignals() {
  TemporaryDirectory temporary;
  const std::filesystem::path output = temporary.GetPath() / "completed";
  const RunnerResult result = RunWithMcap(MakeOptions(output));
  Require(result.exitCode == RunnerExitCode::Success,
      "runner failed: " + result.error);
  Require(result.status == "completed", "runner status is not completed");
  Require(result.steps == 10, "runner produced an unexpected step count");

  const std::filesystem::path manifestPath = output / "run.json";
  const std::filesystem::path mcapPath = output / "telemetry.mcap";
  Require(std::filesystem::is_regular_file(manifestPath),
      "run.json was not produced");
  Require(std::filesystem::is_regular_file(mcapPath),
      "telemetry.mcap was not produced");

  const std::string manifest = ReadTextFile(manifestPath);
  Require(manifest.find("\"status\": \"completed\"") != std::string::npos,
      "run.json status is not completed");
  Require(manifest.find("\"autopilot\": \"primary\"") != std::string::npos,
      "run.json autopilot is missing");

  McapRecordingReader reader;
  Require(reader.Open(mcapPath),
      "failed to open runner MCAP: " + reader.GetLastError());
  Require(reader.GetRunInfo().scenarioName == "Headless Smoke",
      "runner MCAP scenario metadata is incorrect");
  Require(reader.GetRunInfo().simulationDtSec == 0.01,
      "runner MCAP timestep metadata is incorrect");
  Require(reader.GetRunInfo().contractVersion == "1.0.0",
      "runner MCAP contract version is incorrect");
  Require(reader.GetRunInfo().telemetrySchemaVersion == 1,
      "runner MCAP telemetry schema version is incorrect");
  Require(reader.GetRunInfo().gitCommit.size() == 40,
      "runner MCAP does not contain the immutable full commit SHA");
  Require(reader.GetRunInfo().scenarioDigest.size() == 64,
      "runner MCAP does not contain the scenario SHA-256 digest");

  const std::vector<RecordedSample> diagnostics =
      reader.ReadMessages("/jsb/primary/control/roll");
  const std::vector<RecordedSample> aircraft =
      reader.ReadMessages("/jsb/primary/aircraft/state");
  const std::vector<RecordedSample> simulationEvents =
      reader.ReadMessages("/jsb/simulation/event");
  RequireMonotonicSimulationTimestamps(diagnostics,
      "/jsb/primary/control/roll");
  RequireMonotonicSimulationTimestamps(aircraft, "/jsb/primary/aircraft/state");
  Require(!simulationEvents.empty(), "/jsb/simulation/event has no samples");
  bool foundScheduledCommand = false;
  for (const RecordedSample &sample : simulationEvents) {
    jsb::telemetry::v1::SimulationEvent event;
    Require(event.ParseFromString(sample.payload),
        "runner simulation event payload is invalid");
    if (event.type() == jsb::telemetry::v1::SIMULATION_EVENT_COMMAND_APPLIED) {
      foundScheduledCommand = true;
      Require(event.sim_time_ns() == 20'000'000,
          "scenario command did not execute at its declared simulation time");
      Require(event.has_target_roll_rad(),
          "scenario command event omitted the resolved roll target");
    }
  }
  Require(foundScheduledCommand,
      "runner MCAP did not record the scheduled scenario command");
  Require(diagnostics.front().logTimeNanoseconds == 10'000'000,
      "runner MCAP does not start at the first simulation step");
  Require(diagnostics.back().logTimeNanoseconds == 100'000'000,
      "runner MCAP simulation time range is incorrect");

  Require(telemetry::recording::mcap_schema::DeserializeRollHoldDiagnostics(
              diagnostics.front().payload)
              .has_value(),
      "runner roll-hold payload is invalid");
  for (const auto &channel : reader.GetChannels()) {
    if (channel.topic == "/jsb/primary/control/roll"
        || channel.topic == "/jsb/primary/aircraft/state"
        || channel.topic == "/jsb/simulation/event") {
      Require(channel.messageEncoding == "protobuf",
          "contract channel message encoding is not protobuf");
      Require(channel.schemaEncoding == "protobuf",
          "contract channel schema encoding is not protobuf");
      Require(channel.schemaDataSize > 0,
          "contract channel does not embed a FileDescriptorSet");
      google::protobuf::FileDescriptorSet descriptorSet;
      Require(descriptorSet.ParseFromString(channel.schemaData),
          "contract channel schema is not a valid FileDescriptorSet");
      Require(descriptorSet.file_size() > 0,
          "contract channel FileDescriptorSet is empty");
    }
  }

  const std::filesystem::path repeatedOutput = temporary.GetPath() / "repeated";
  const RunnerResult repeatedResult = RunWithMcap(MakeOptions(repeatedOutput));
  Require(repeatedResult.exitCode == RunnerExitCode::Success,
      "repeated deterministic run failed: " + repeatedResult.error);
  McapRecordingReader repeatedReader;
  Require(repeatedReader.Open(repeatedOutput / "telemetry.mcap"),
      "failed to open repeated runner MCAP: " + repeatedReader.GetLastError());
  RequireSameSamples(diagnostics,
      repeatedReader.ReadMessages("/jsb/primary/control/roll"),
      "/jsb/primary/control/roll");
  RequireSameSamples(aircraft,
      repeatedReader.ReadMessages("/jsb/primary/aircraft/state"),
      "/jsb/primary/aircraft/state");
  RequireSameSamples(simulationEvents,
      repeatedReader.ReadMessages("/jsb/simulation/event"),
      "/jsb/simulation/event");
}

void TestFailureManifestAndMcapOpenFailure() {
  TemporaryDirectory temporary;

  RunnerOptions invalidScenario = MakeOptions(temporary.GetPath() / "invalid");
  invalidScenario.scenarioPath = temporary.GetPath() / "missing.yaml";
  const RunnerResult scenarioResult = RunWithMcap(invalidScenario);
  Require(scenarioResult.exitCode == RunnerExitCode::ScenarioLoadFailure,
      "missing scenario returned an unexpected exit code");
  Require(std::filesystem::is_regular_file(
              invalidScenario.outputDirectory / "run.json"),
      "missing scenario did not produce run.json");

  const std::filesystem::path blockedOutput = temporary.GetPath() / "blocked";
  std::filesystem::create_directories(blockedOutput / "telemetry.mcap");
  const RunnerResult recorderResult = RunWithMcap(MakeOptions(blockedOutput));
  Require(recorderResult.exitCode == RunnerExitCode::OutputFailure,
      "MCAP open failure returned an unexpected exit code");
  Require(recorderResult.error.find("failed to initialize MCAP recorder")
              != std::string::npos,
      "MCAP open failure did not propagate through RunnerResult.error");
  const std::string manifest = ReadTextFile(blockedOutput / "run.json");
  Require(manifest.find("\"status\": \"failed\"") != std::string::npos,
      "MCAP open failure manifest is not failed");
}

void TestInterruptedRunFinalizesMcap() {
  TemporaryDirectory temporary;
  const std::filesystem::path output = temporary.GetPath() / "interrupted";
  volatile std::sig_atomic_t running = 0;
  const RunnerResult result = RunWithMcap(MakeOptions(output), &running);
  Require(result.exitCode != RunnerExitCode::Success,
      "interrupted runner unexpectedly succeeded");
  Require(result.status == "interrupted", "runner did not report interruption");

  McapRecordingReader reader;
  Require(reader.Open(output / "telemetry.mcap"),
      "interrupted MCAP was not finalized: " + reader.GetLastError());
  Require(std::filesystem::is_regular_file(output / "run.json"),
      "interrupted run did not produce run.json");
}

void TestSameRunnerExecutesScenarioSelectedAutopilots() {
  TemporaryDirectory temporary;
  sim::SimulationScenario baseline;
  std::string error;
  Require(
      sim::SimulationScenarioSerializer::Load(JSB_TEST_HEADLESS_SCENARIO_PATH,
          baseline,
          error),
      "failed to load primary fixture: " + error);
  baseline.name = "Headless Baseline";
  baseline.autopilot = gnc::AutopilotKind::Baseline;
  const std::filesystem::path baselinePath =
      temporary.GetPath() / "baseline.yaml";
  Require(
      sim::SimulationScenarioSerializer::Save(baselinePath, baseline, error),
      "failed to save baseline fixture: " + error);

  RunnerOptions options = MakeOptions(temporary.GetPath() / "baseline-run");
  options.scenarioPath = baselinePath;
  const RunnerResult result = RunWithMcap(options);
  Require(result.exitCode == RunnerExitCode::Success,
      "baseline scenario failed in the same runner: " + result.error);
  const std::string manifest =
      ReadTextFile(options.outputDirectory / "run.json");
  Require(manifest.find("\"autopilot\": \"baseline\"") != std::string::npos,
      "metadata did not preserve scenario-selected baseline autopilot");
  Require(manifest.find("\"digest_sha256\": \"") != std::string::npos,
      "metadata did not include scenario digest");
  McapRecordingReader reader;
  Require(reader.Open(options.outputDirectory / "telemetry.mcap"),
      "failed to read baseline MCAP");
  Require(reader.GetRunInfo().resolvedAutopilot == "baseline",
      "MCAP metadata did not identify the resolved autopilot");
  Require(reader.GetRunInfo().scenarioDigest.size() == 64,
      "MCAP metadata did not include the SHA-256 scenario digest");
}

void TestSemanticCliOverridesAreRejected() {
  const std::vector<std::vector<std::string_view>> overrides = {
      {"--autopilot", "baseline"},
      {"--aircraft", "c172x"},
      {"--duration", "5"},
      {"--dt", "0.01"},
      {"--no-trim"},
  };
  for (const auto &override : overrides) {
    std::vector<std::string_view> arguments = {"--scenario",
        "primary.yaml",
        "--output",
        "out"};
    arguments.insert(arguments.end(), override.begin(), override.end());
    const runner::RunnerParseResult parsed =
        runner::ParseRunnerOptions(arguments);
    Require(!parsed.options.has_value(),
        "semantic CLI override was unexpectedly accepted");
    Require(parsed.error.find("defined by the scenario") != std::string::npos,
        "semantic CLI override rejection is not actionable");
  }
}

void TestInvalidScenarioFailsBeforeSimulationStarts() {
  TemporaryDirectory temporary;
  std::string yaml = ReadTextFile(JSB_TEST_HEADLESS_SCENARIO_PATH);
  const std::string validDuration = "duration_sec: 0.1";
  const std::size_t position = yaml.find(validDuration);
  Require(position != std::string::npos,
      "headless fixture duration was not found");
  yaml.replace(position, validDuration.size(), "duration_sec: 0");
  const std::filesystem::path scenarioPath =
      temporary.GetPath() / "invalid-duration.yaml";
  {
    std::ofstream output(scenarioPath, std::ios::binary);
    output << yaml;
  }
  RunnerOptions options = MakeOptions(temporary.GetPath() / "invalid-run");
  options.scenarioPath = scenarioPath;
  const RunnerResult result = RunWithMcap(options);
  Require(result.exitCode == RunnerExitCode::ScenarioLoadFailure,
      "invalid scenario returned an unexpected exit code");
  Require(result.steps == 0,
      "invalid scenario advanced the simulation before failing");
  Require(!std::filesystem::exists(options.outputDirectory / "telemetry.mcap"),
      "invalid scenario started telemetry recording");
}
} // namespace

int main() {
  TestSuccessfulRunArtifactsAndSignals();
  TestFailureManifestAndMcapOpenFailure();
  TestInterruptedRunFinalizesMcap();
  TestSameRunnerExecutesScenarioSelectedAutopilots();
  TestSemanticCliOverridesAreRejected();
  TestInvalidScenarioFailsBeforeSimulationStarts();
  return 0;
}
