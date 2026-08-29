#include "sim/scenario/SimulationScenarioSerializer.hpp"
#include "telemetry/aircraft_state.pb.h"
#include "telemetry/control.pb.h"

#include <google/protobuf/descriptor.h>

#include <cassert>
#include <string>

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
  assert(scenario.autopilot == "baseline");
}

void TestUnsupportedScenarioVersionIsRejected() {
  sim::SimulationScenario scenario;
  scenario.schemaVersion = 2;
  std::string error;
  assert(!sim::ValidateSimulationScenario(scenario, &error));
  assert(error.find("schema_version") != std::string::npos);
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
} // namespace

int main() {
  TestCanonicalScenarioIsExecutableInput();
  TestUnsupportedScenarioVersionIsRejected();
  TestRequiredProtobufSignalsExist();
  return 0;
}
