#include "application/gui/windows/ScenarioWindow.hpp"
#include "application/sim/scenario/SimulationScenario.hpp"
#include "application/sim/scenario/SimulationScenarioSerializer.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
constexpr double Tolerance = 1.0e-12;

void RequireNear(double actual, double expected) {
  assert(std::abs(actual - expected) <= Tolerance);
}

void RequireDefaultScenario(const sim::SimulationScenario &scenario) {
  assert(scenario.name == "C172 Roll Hold 5deg");
  RequireNear(scenario.altitudeFt, 3000.0);
  RequireNear(scenario.airspeedKts, 100.0);
  RequireNear(scenario.initialRollDeg, 0.0);
  RequireNear(scenario.initialPitchDeg, 0.0);
  RequireNear(scenario.initialHeadingDeg, 0.0);
  assert(!scenario.windEnabled);
  assert(scenario.runTrim);
  assert(scenario.trimMode == gnc::TrimMode::Full);
  RequireNear(scenario.durationSec, 30.0);
  RequireNear(scenario.commandStartSec, 5.0);
  RequireNear(scenario.commandedRollDeg, 5.0);
  RequireNear(scenario.settlingBandDeg, 0.5);
  RequireNear(scenario.settlingTimeLimitSec, 10.0);
  RequireNear(scenario.overshootLimitDeg, 1.0);
  RequireNear(scenario.maxOscillationCycles, 2.0);
}

void TestScenarioValidation() {
  std::string error;
  assert(sim::ValidateSimulationScenario(sim::SimulationScenario{}, &error));
  assert(error.empty());

  const auto requireInvalid = [](sim::SimulationScenario scenario,
                                  const std::string &expectedField) {
    std::string validationError;
    assert(!sim::ValidateSimulationScenario(scenario, &validationError));
    assert(validationError.find(expectedField) != std::string::npos);
  };

  sim::SimulationScenario scenario;
  scenario.airspeedKts = -1.0;
  requireInvalid(scenario, "initial_condition.airspeed_kts");

  scenario = {};
  scenario.trimMode = static_cast<gnc::TrimMode>(-1);
  requireInvalid(scenario, "trim.mode");

  scenario = {};
  scenario.commandStartSec = scenario.durationSec + 1.0;
  requireInvalid(scenario, "command.start_sec");

  scenario = {};
  scenario.overshootLimitDeg = -1.0;
  requireInvalid(scenario, "acceptance.overshoot_limit_deg");
}

std::filesystem::path MakeTemporaryScenarioDirectory() {
  const auto uniqueSuffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path()
      / ("jsb-scenario-tests-" + std::to_string(uniqueSuffix));
  assert(std::filesystem::create_directory(directory));
  return directory;
}

void TestYamlRoundTrip() {
  sim::SimulationScenario source;
  source.name = "Edited YAML Scenario";
  source.altitudeFt = 4250.5;
  source.airspeedKts = 87.25;
  source.initialRollDeg = -3.5;
  source.initialPitchDeg = 2.25;
  source.initialHeadingDeg = 271.0;
  source.windEnabled = true;
  source.runTrim = false;
  source.trimMode = gnc::TrimMode::Ground;
  source.durationSec = 45.0;
  source.commandStartSec = 7.5;
  source.commandedRollDeg = -12.0;
  source.settlingBandDeg = 0.25;
  source.settlingTimeLimitSec = 8.0;
  source.overshootLimitDeg = 0.75;
  source.maxOscillationCycles = 3.0;

  const std::string yaml = sim::SimulationScenarioSerializer::Serialize(source);
  assert(yaml.find("initial_condition:") != std::string::npos);
  assert(yaml.find("mode: Ground") != std::string::npos);

  sim::SimulationScenario parsed;
  std::string error;
  assert(sim::SimulationScenarioSerializer::Deserialize(yaml, parsed, error));
  assert(error.empty());
  assert(parsed == source);
}

void TestInvalidYamlIsTransactional() {
  const sim::SimulationScenario original;
  sim::SimulationScenario destination = original;
  destination.name = "Keep Me";
  const sim::SimulationScenario before = destination;
  std::string error;

  const std::string missingFieldYaml = R"(
name: Missing Acceptance
initial_condition:
  altitude_ft: 3000
  airspeed_kts: 100
  roll_deg: 0
  pitch_deg: 0
  heading_deg: 0
environment:
  wind_enabled: false
trim:
  enabled: true
  mode: Full
command:
  start_sec: 5
  roll_deg: 5
simulation:
  duration_sec: 30
)";
  assert(!sim::SimulationScenarioSerializer::Deserialize(missingFieldYaml,
      destination,
      error));
  assert(error.find("acceptance") != std::string::npos);
  assert(destination == before);

  std::string invalidTypeYaml =
      sim::SimulationScenarioSerializer::Serialize(original);
  const std::size_t airspeedPosition =
      invalidTypeYaml.find("airspeed_kts: 100");
  assert(airspeedPosition != std::string::npos);
  invalidTypeYaml.replace(airspeedPosition,
      std::string("airspeed_kts: 100").size(),
      "airspeed_kts: fast");
  assert(!sim::SimulationScenarioSerializer::Deserialize(invalidTypeYaml,
      destination,
      error));
  assert(error.find("initial_condition.airspeed_kts") != std::string::npos);
  assert(destination == before);

  assert(!sim::SimulationScenarioSerializer::Deserialize("name: [unterminated",
      destination,
      error));
  assert(error.find("YAML parse error") != std::string::npos);
  assert(destination == before);
}

void TestScenarioWindowFileLifecycle() {
  const std::filesystem::path directory = MakeTemporaryScenarioDirectory();
  const std::filesystem::path validFile = directory / "edited.yaml";
  const std::filesystem::path invalidFile = directory / "invalid.yaml";

  gui::ScenarioWindow window(directory);
  assert(window.GetScenarioDirectory() == directory);
  assert(window.GetCurrentFilePath().empty());
  assert(!window.IsDirty());

  window.GetScenario().name = "Saved Scenario";
  assert(window.IsDirty());
  assert(!window.SaveScenarioFile());
  assert(window.GetFileStatusMessage().find("Save As") != std::string::npos);

  assert(window.SaveScenarioFileAs("edited.yaml"));
  assert(window.GetCurrentFilePath() == validFile);
  assert(std::filesystem::is_regular_file(validFile));
  assert(!window.IsDirty());

  window.GetScenario().commandedRollDeg = 9.0;
  assert(window.IsDirty());
  assert(window.SaveScenarioFile());
  assert(!window.IsDirty());

  {
    std::ofstream invalidOutput(invalidFile, std::ios::binary);
    invalidOutput << "name: [invalid";
  }
  const sim::SimulationScenario beforeInvalidLoad = window.GetScenario();
  const std::filesystem::path connectedBeforeInvalidLoad =
      window.GetCurrentFilePath();
  assert(!window.LoadScenarioFile(invalidFile));
  assert(window.GetScenario() == beforeInvalidLoad);
  assert(window.GetCurrentFilePath() == connectedBeforeInvalidLoad);

  window.NewScenario();
  RequireDefaultScenario(window.GetScenario());
  assert(window.GetCurrentFilePath().empty());
  assert(!window.IsDirty());

  assert(window.LoadScenarioFile(validFile));
  assert(window.GetScenario().name == "Saved Scenario");
  RequireNear(window.GetScenario().commandedRollDeg, 9.0);
  assert(!window.IsDirty());

  assert(std::filesystem::remove(invalidFile));
  assert(std::filesystem::remove(validFile));
  assert(std::filesystem::remove(directory));
}

void TestRepositoryScenarioAsset() {
  gui::ScenarioWindow window;
  assert(window.LoadScenarioFile("c172_roll_hold_5deg.yaml"));
  RequireDefaultScenario(window.GetScenario());
  assert(!window.IsDirty());
}
} // namespace

int main() {
  RequireDefaultScenario(sim::SimulationScenario{});
  TestScenarioValidation();

  gui::ScenarioWindow window;
  RequireDefaultScenario(window.GetScenario());

  sim::SimulationScenario &edited = window.GetScenario();
  edited.name = "Edited Scenario";
  edited.altitudeFt = 1200.0;
  edited.windEnabled = true;
  edited.runTrim = false;
  edited.trimMode = gnc::TrimMode::Ground;
  edited.commandedRollDeg = -12.0;
  edited.maxOscillationCycles = 8.0;

  window.ResetDefaults();
  RequireDefaultScenario(window.GetScenario());

  TestYamlRoundTrip();
  TestInvalidYamlIsTransactional();
  TestScenarioWindowFileLifecycle();
  TestRepositoryScenarioAsset();
  return 0;
}
