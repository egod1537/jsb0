#include "gui/features/simulation/ScenarioController.hpp"
#include "gui/features/simulation/ScenarioSetupPopup.hpp"
#include "gui/windows/ScenarioWindow.hpp"
#include "sim/scenario/SimScenario.hpp"
#include "sim/scenario/SimScenarioSerializer.hpp"
#include "common/math/Math.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
constexpr double Tolerance = 1.0e-12;

template <typename T>
concept HasEditableScenario = requires(T &window) { window.GetScenario(); };

static_assert(!HasEditableScenario<gui::ScenarioWindow>);

void RequireNear(double actual, double expected) {
  assert(std::abs(actual - expected) <= Tolerance);
}

void RequireDefaultScenario(const sim::SimScenario &scenario) {
  assert(scenario.name == "Roll Hold 5deg 30s");
  RequireNear(scenario.initialCondition.altitudeAslM,
      math::FeetToMeters(3000.0));
  RequireNear(scenario.initialCondition.calibratedAirspeedMps,
      math::KnotsToMetersPerSecond(100.0));
  RequireNear(scenario.initialCondition.rollRad, 0.0);
  RequireNear(scenario.initialCondition.pitchRad, 0.0);
  RequireNear(scenario.initialCondition.headingRad, 0.0);
  assert(!scenario.windEnabled);
  assert(scenario.runTrim);
  assert(scenario.trimMode == gnc::TrimMode::Full);
  RequireNear(scenario.durationSec, 30.0);
  RequireNear(scenario.dtSec, opts::simulation::DtSec);
  RequireNear(scenario.events.front().timeSec, 5.0);
  RequireNear(scenario.events.front().command.rollRad, math::DegToRad(5.0));
  RequireNear(scenario.settlingBandRad, math::DegToRad(0.5));
  RequireNear(scenario.settlingTimeLimitSec, 10.0);
  RequireNear(scenario.overshootLimitRad, math::DegToRad(1.0));
  RequireNear(scenario.maxOscillationCycles, 2.0);
}

void TestScenarioValidation() {
  std::string error;
  assert(sim::ValidateSimScenario(sim::SimScenario{}, &error));
  assert(error.empty());

  const auto requireInvalid = [](sim::SimScenario scenario,
                                  const std::string &expectedField) {
    std::string validationError;
    assert(!sim::ValidateSimScenario(scenario, &validationError));
    assert(validationError.find(expectedField) != std::string::npos);
  };

  sim::SimScenario scenario;
  scenario.initialCondition.calibratedAirspeedMps = -1.0;
  requireInvalid(scenario, "initial_condition.airspeed_kts");

  scenario = {};
  scenario.trimMode = static_cast<gnc::TrimMode>(-1);
  requireInvalid(scenario, "trim.mode");

  scenario = {};
  scenario.events.front().timeSec = scenario.durationSec + 1.0;
  requireInvalid(scenario, "events[0].time_sec");

  scenario = {};
  scenario.overshootLimitRad = -1.0;
  requireInvalid(scenario, "acceptance.overshoot_limit_deg");

  scenario = {};
  scenario.controllerParameters = {"FW_RR_P", "FW_RR_P"};
  requireInvalid(scenario, "controller_parameters[1]");

  scenario = {};
  scenario.controllerParameters = {"roll-rate-p"};
  requireInvalid(scenario, "controller_parameters[0]");
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
  sim::SimScenario source;
  source.name = "Edited YAML Scenario";
  source.controllerParameters = {"FW_RR_P", "FW_RR_I"};
  source.initialCondition.altitudeAslM = math::FeetToMeters(4250.5);
  source.initialCondition.calibratedAirspeedMps =
      math::KnotsToMetersPerSecond(87.25);
  source.initialCondition.rollRad = math::DegToRad(-3.5);
  source.initialCondition.pitchRad = math::DegToRad(2.25);
  source.initialCondition.headingRad = math::DegToRad(271.0);
  source.windEnabled = false;
  source.runTrim = false;
  source.trimMode = gnc::TrimMode::Ground;
  source.durationSec = 45.0;
  source.events.front().timeSec = 7.5;
  source.events.front().command.rollRad = math::DegToRad(-12.0);
  source.settlingBandRad = math::DegToRad(0.25);
  source.settlingTimeLimitSec = 8.0;
  source.overshootLimitRad = math::DegToRad(0.75);
  source.maxOscillationCycles = 3.0;

  const std::string yaml = sim::SimScenarioSerializer::Serialize(source);
  assert(yaml.find("initial_condition:") != std::string::npos);
  assert(yaml.find("controller_parameters:") != std::string::npos);
  assert(yaml.find("mode: Ground") != std::string::npos);

  sim::SimScenario parsed;
  std::string error;
  assert(sim::SimScenarioSerializer::Deserialize(yaml, parsed, error));
  assert(error.empty());
  assert(parsed == source);
}

void TestInvalidYamlIsTransactional() {
  const sim::SimScenario original;
  sim::SimScenario destination = original;
  destination.name = "Keep Me";
  const sim::SimScenario before = destination;
  std::string error;

  const std::string missingFieldYaml = R"(
schema_version: 1
scenario_type: roll_hold
name: Missing Acceptance
aircraft: c172x
initial_condition:
  latitude_deg: 0
  longitude_deg: 0
  altitude_ft: 3000
  airspeed_kts: 100
  roll_deg: 0
  pitch_deg: 0
  heading_deg: 0
  p_rad_s: 0
  q_rad_s: 0
  r_rad_s: 0
environment:
  wind_enabled: false
trim:
  enabled: true
  mode: Full
simulation:
  duration_sec: 30
  dt_sec: 0.03333333333333333
events:
  - time_sec: 5
    command:
      type: roll_hold
      roll_deg: 5
)";
  assert(!sim::SimScenarioSerializer::Deserialize(missingFieldYaml,
      destination,
      error));
  assert(error.find("acceptance") != std::string::npos);
  assert(destination == before);

  std::string invalidTypeYaml = sim::SimScenarioSerializer::Serialize(original);
  const std::size_t airspeedPosition =
      invalidTypeYaml.find("airspeed_kts: 100");
  assert(airspeedPosition != std::string::npos);
  invalidTypeYaml.replace(airspeedPosition,
      std::string("airspeed_kts: 100").size(),
      "airspeed_kts: fast");
  assert(!sim::SimScenarioSerializer::Deserialize(invalidTypeYaml,
      destination,
      error));
  assert(error.find("initial_condition.airspeed_kts") != std::string::npos);
  assert(destination == before);

  assert(!sim::SimScenarioSerializer::Deserialize("name: [unterminated",
      destination,
      error));
  assert(error.find("YAML parse error") != std::string::npos);
  assert(destination == before);
}

void TestScenarioControllerFileLifecycle() {
  const std::filesystem::path directory = MakeTemporaryScenarioDirectory();
  const std::filesystem::path validFile = directory / "edited.yaml";
  const std::filesystem::path invalidFile = directory / "invalid.yaml";

  gui::ScenarioController controller(directory);
  assert(controller.GetModel().directory == directory);
  assert(controller.GetModel().currentFilePath.empty());
  assert(!controller.IsDirty());

  controller.EditDraftForCompatibility().name = "Saved Scenario";
  assert(controller.IsDirty());
  assert(!controller.Save());
  assert(
      controller.GetModel().statusMessage.find("Save As") != std::string::npos);

  assert(controller.SaveAs("edited.yaml"));
  assert(controller.GetModel().currentFilePath == validFile);
  assert(std::filesystem::is_regular_file(validFile));
  assert(!controller.IsDirty());

  controller.EditDraftForCompatibility().events.front().command.rollRad =
      math::DegToRad(9.0);
  assert(controller.IsDirty());
  assert(controller.Save());
  assert(!controller.IsDirty());

  {
    std::ofstream invalidOutput(invalidFile, std::ios::binary);
    invalidOutput << "name: [invalid";
  }
  const sim::SimScenario beforeInvalidLoad = controller.GetModel().draft;
  const std::filesystem::path connectedBeforeInvalidLoad =
      controller.GetModel().currentFilePath;
  assert(!controller.Load(invalidFile));
  assert(controller.GetModel().draft == beforeInvalidLoad);
  assert(controller.GetModel().currentFilePath == connectedBeforeInvalidLoad);

  controller.NewScenario();
  RequireDefaultScenario(controller.GetModel().draft);
  assert(controller.GetModel().currentFilePath.empty());
  assert(!controller.IsDirty());

  assert(controller.Load(validFile));
  assert(controller.GetModel().draft.name == "Saved Scenario");
  RequireNear(controller.GetModel().draft.events.front().command.rollRad,
      math::DegToRad(9.0));
  assert(!controller.IsDirty());

  assert(std::filesystem::remove(invalidFile));
  assert(std::filesystem::remove(validFile));
  assert(std::filesystem::remove(directory));
}

void TestRepositoryScenarioAsset() {
  gui::ScenarioController controller;
  assert(controller.Load("roll_hold_5deg_30s.yaml"));
  RequireDefaultScenario(controller.GetModel().draft);
  assert(controller.GetModel().draft.controllerParameters
         == std::vector<std::string>({"FW_R_TC",
             "FW_RR_P",
             "FW_RR_I",
             "FW_RR_D",
             "FW_RR_FF",
             "FW_RR_IMAX"}));
  assert(!controller.IsDirty());
}

void TestScenarioApplyAndPopupLifecycle() {
  int launchCount = 0;
  gui::ScenarioController *controllerPtr = nullptr;
  gui::ScenarioController controller({},
      gui::architecture::EventSink<gui::ScenarioLaunchRequested>{
          [&launchCount, &controllerPtr](const gui::ScenarioLaunchRequested &) {
            ++launchCount;
            controllerPtr->OnEvent(gui::ScenarioApplyCompleted{
                .succeeded = true,
            });
          }});
  controllerPtr = &controller;

  gui::ScenarioSetupPopup popup(controller);
  popup.RequestOpen();
  assert(popup.IsOpenRequested());
  popup.Cancel();
  assert(!popup.IsOpenRequested());
  assert(launchCount == 0);

  assert(controller.Apply());
  assert(launchCount == 1);
  assert(controller.GetModel().lastApplySucceeded);

  sim::SimScenario invalid = controller.GetModel().draft;
  invalid.durationSec = 0.0;
  controller.OnEvent(gui::ScenarioDraftChanged{invalid});
  assert(!controller.Apply());
  assert(launchCount == 1);
  assert(controller.GetModel().statusIsError);
}

void TestScenarioApplyRemainsPendingUntilAsyncResult() {
  int launchCount = 0;
  gui::ScenarioController controller({},
      gui::architecture::EventSink<gui::ScenarioLaunchRequested>{
          [&launchCount](
              const gui::ScenarioLaunchRequested &) { ++launchCount; }});

  assert(controller.Apply());
  assert(launchCount == 1);
  assert(controller.GetModel().applyPending);
  assert(!controller.GetModel().lastApplySucceeded);

  controller.OnEvent(gui::ScenarioApplyCompleted{
      .succeeded = false,
      .error = "Runtime rejected scenario.",
  });
  assert(!controller.GetModel().applyPending);
  assert(!controller.GetModel().lastApplySucceeded);
  assert(controller.GetModel().statusIsError);
  assert(controller.GetModel().statusMessage == "Runtime rejected scenario.");
}
} // namespace

int main() {
  RequireDefaultScenario(sim::SimScenario{});
  TestScenarioValidation();

  gui::ScenarioController controller;
  RequireDefaultScenario(controller.GetModel().draft);

  sim::SimScenario &edited = controller.EditDraftForCompatibility();
  edited.name = "Edited Scenario";
  edited.initialCondition.altitudeAslM = 1200.0;
  edited.windEnabled = true;
  edited.runTrim = false;
  edited.trimMode = gnc::TrimMode::Ground;
  edited.events.front().command.rollRad = math::DegToRad(-12.0);
  edited.maxOscillationCycles = 8.0;

  controller.ResetDefaults();
  RequireDefaultScenario(controller.GetModel().draft);

  TestYamlRoundTrip();
  TestInvalidYamlIsTransactional();
  TestScenarioControllerFileLifecycle();
  TestRepositoryScenarioAsset();
  TestScenarioApplyAndPopupLifecycle();
  TestScenarioApplyRemainsPendingUntilAsyncResult();
  return 0;
}
