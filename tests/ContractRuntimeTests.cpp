#include "common/crypto/Sha256.hpp"
#include "sim/scenario/SimulationScenarioSerializer.hpp"
#include "telemetry/aircraft_state.pb.h"
#include "telemetry/control.pb.h"

#include <google/protobuf/descriptor.h>

#include <cassert>
#include <limits>
#include <string>
#include <string_view>

#ifndef JSB_TEST_CONTRACT_SCENARIO_PATH
#define JSB_TEST_CONTRACT_SCENARIO_PATH                                        \
  "contract/examples/scenario/roll_hold.yaml"
#endif

namespace {
void TestCanonicalScenarioIsExecutableInput() {
  sim::SimulationScenario scenario;
  std::string error;
  assert(
      sim::SimulationScenarioSerializer::Load(JSB_TEST_CONTRACT_SCENARIO_PATH,
          scenario,
          error));
  assert(error.empty());
  assert(scenario.schemaVersion == 1);
  assert(scenario.scenarioType == "roll_hold");
  assert(scenario.aircraft == "c172x");
  assert(scenario.autopilot == gnc::AutopilotKind::Baseline);
}

void TestUnsupportedScenarioVersionIsRejected() {
  sim::SimulationScenario scenario;
  scenario.schemaVersion = 2;
  std::string error;
  assert(!sim::ValidateSimulationScenario(scenario, &error));
  assert(error.find("schema_version") != std::string::npos);
}

void TestAuthoritativeScenarioValidation() {
  sim::SimulationScenario valid;
  std::string error;
  const auto requireInvalid = [&error](sim::SimulationScenario scenario,
                                  std::string_view path) {
    assert(!sim::ValidateSimulationScenario(scenario, &error));
    assert(error.find(path) != std::string::npos);
  };

  sim::SimulationScenario invalid = valid;
  invalid.aircraft = "unknown";
  requireInvalid(invalid, "aircraft");
  invalid = valid;
  invalid.autopilot = static_cast<gnc::AutopilotKind>(99);
  requireInvalid(invalid, "autopilot.type");
  invalid = valid;
  invalid.durationSec = 0.0;
  requireInvalid(invalid, "simulation.duration_sec");
  invalid = valid;
  invalid.initialCondition.airspeedKts =
      std::numeric_limits<double>::quiet_NaN();
  requireInvalid(invalid, "initial_condition.airspeed_kts");
  invalid = valid;
  invalid.events.front().timeSec = valid.durationSec + 1.0;
  requireInvalid(invalid, "events[0].time_sec");
  invalid = valid;
  invalid.events.front().command.type =
      static_cast<sim::ScenarioCommandType>(99);
  requireInvalid(invalid, "events[0].command.type");

  const std::string serialized =
      sim::SimulationScenarioSerializer::Serialize(valid);
  const std::string autopilotBlock = "autopilot:\n  type: primary\n";
  const std::size_t position = serialized.find(autopilotBlock);
  assert(position != std::string::npos);
  std::string missingAutopilot = serialized;
  missingAutopilot.erase(position, autopilotBlock.size());
  sim::SimulationScenario parsed;
  assert(!sim::SimulationScenarioSerializer::Deserialize(missingAutopilot,
      parsed,
      error));
  assert(error.find("autopilot") != std::string::npos);

  std::string invalidAutopilot = serialized;
  invalidAutopilot.replace(position,
      autopilotBlock.size(),
      "autopilot:\n  type: unsupported\n");
  assert(!sim::SimulationScenarioSerializer::Deserialize(invalidAutopilot,
      parsed,
      error));
  assert(error.find("autopilot.type") != std::string::npos);
}

void TestRequiredProtobufSignalsExist() {
  const google::protobuf::Descriptor *aircraft =
      jsb::telemetry::v1::AircraftState::descriptor();
  assert(aircraft->FindFieldByName("sim_time_ns") != nullptr);
  assert(aircraft->FindFieldByName("roll_rad") != nullptr);
  assert(aircraft->FindFieldByName("roll_rate_rad_s") != nullptr);

  const google::protobuf::Descriptor *control =
      jsb::telemetry::v1::RollControlState::descriptor();
  assert(control->FindFieldByName("commanded_roll_rad") != nullptr);
  assert(control->FindFieldByName("commanded_roll_rate_rad_s") != nullptr);
  assert(control->FindFieldByName("aileron_command") != nullptr);
}

void TestScenarioDigestUsesSha256() {
  assert(common::crypto::Sha256Hex("abc")
         == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}
} // namespace

int main() {
  TestCanonicalScenarioIsExecutableInput();
  TestUnsupportedScenarioVersionIsRejected();
  TestAuthoritativeScenarioValidation();
  TestRequiredProtobufSignalsExist();
  TestScenarioDigestUsesSha256();
  return 0;
}
