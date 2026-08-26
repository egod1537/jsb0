#include "application/Application.hpp"
#include "application/SimulationExecutionControl.hpp"
#include "application/gui/GUI.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlManager.hpp"
#include "application/sim/gnc/autopilot/IRollHoldAutopilot.hpp"
#include "application/sim/gnc/autopilot/MyAutopilot.hpp"
#include "application/sim/gnc/autopilot/PX4Autopilot.hpp"
#include "application/sim/scenario/SimulationScenario.hpp"
#include "common/math/Math.hpp"

#include <cassert>
#include <cmath>
#include <memory>

namespace {
bool NearlyEqual(double left, double right, double tolerance = 1.0e-6) {
  return std::abs(left - right) <= tolerance;
}

void SetNorthWind(sim::Simulation &simulation, double northFps) {
  sim::FDMState state =
      simulation.GetAircraft().ExtractFDMState(sim::FDMStateFlags::Environment);
  state.environment.windNedFps[0] = northFps;
  simulation.GetAircraft().ApplyFDMState(state);
}

sim::FDMEnvironmentState GetEnvironment(const sim::Simulation &simulation) {
  return simulation.GetAircraft()
      .ExtractFDMState(sim::FDMStateFlags::Environment)
      .environment;
}

void TestInteractiveExecutionHasNoScenarioState() {
  Application application;
  application::SimulationExecutionControl &control = application;

  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());

  sim::SimulationScenario scenario;
  assert(!control.RunScenario(scenario));

  control.StartSimulation();
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Running);
  assert(!control.GetScenarioExecutionStatus().has_value());
  control.StopSimulation();
}

void TestScenarioAppliesSharedExperimentToBothSimulations() {
  sim::SimulationConfig config;
  auto primary =
      std::make_unique<sim::Simulation>(std::make_unique<gnc::MyAutopilot>());
  auto baseline =
      std::make_unique<sim::Simulation>(std::make_unique<gnc::PX4Autopilot>());
  assert(primary->Initialize(config));
  assert(baseline->Initialize(config));

  sim::Simulation *primarySource = primary.get();
  sim::Simulation *baselineSource = baseline.get();
  auto gui = std::make_unique<gui::GUI>(*primarySource,
      baselineSource,
      gui::GUIConfig{});
  Application application(std::move(gui),
      std::move(primary),
      config,
      std::move(baseline));
  application::SimulationExecutionControl &control = application;

  sim::SimulationScenario scenario;
  scenario.name = "Dual Roll Hold";
  scenario.altitudeFt = 4200.0;
  scenario.airspeedKts = 105.0;
  scenario.initialRollDeg = 2.0;
  scenario.initialPitchDeg = 1.0;
  scenario.initialHeadingDeg = 35.0;
  scenario.runTrim = false;
  scenario.commandStartSec = 0.0;
  scenario.commandedRollDeg = 8.0;
  scenario.durationSec = 12.0;

  SetNorthWind(*primarySource, 17.0);
  SetNorthWind(*baselineSource, -4.0);

  sim::SimulationScenario invalidScenario = scenario;
  invalidScenario.settlingBandDeg = -1.0;
  assert(!control.RunScenario(invalidScenario));
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());

  assert(control.RunScenario(scenario));
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Running);
  const auto status = control.GetScenarioExecutionStatus();
  assert(status.has_value());
  assert(status->name == scenario.name);
  assert(status->elapsedSec == 0.0);
  assert(status->durationSec == scenario.durationSec);

  const sim::InitialCondition primaryCondition =
      primarySource->GetCurrentCondition();
  const sim::InitialCondition baselineCondition =
      baselineSource->GetCurrentCondition();
  assert(NearlyEqual(primaryCondition.altitudeFt, scenario.altitudeFt));
  assert(NearlyEqual(primaryCondition.airspeedKts, scenario.airspeedKts));
  assert(NearlyEqual(primaryCondition.rollDeg, scenario.initialRollDeg));
  assert(NearlyEqual(primaryCondition.pitchDeg, scenario.initialPitchDeg));
  assert(NearlyEqual(primaryCondition.headingDeg, scenario.initialHeadingDeg));
  assert(
      NearlyEqual(primaryCondition.altitudeFt, baselineCondition.altitudeFt));
  assert(
      NearlyEqual(primaryCondition.airspeedKts, baselineCondition.airspeedKts));
  assert(NearlyEqual(primaryCondition.rollDeg, baselineCondition.rollDeg));
  assert(NearlyEqual(primaryCondition.pitchDeg, baselineCondition.pitchDeg));
  assert(
      NearlyEqual(primaryCondition.headingDeg, baselineCondition.headingDeg));
  assert(primarySource->GetTickSizeSec() == baselineSource->GetTickSizeSec());
  assert(&primarySource->GetAircraft() != &baselineSource->GetAircraft());
  assert(&primarySource->GetTelemetryRegistry()
         != &baselineSource->GetTelemetryRegistry());
  const sim::FDMEnvironmentState primaryEnvironment =
      GetEnvironment(*primarySource);
  const sim::FDMEnvironmentState baselineEnvironment =
      GetEnvironment(*baselineSource);
  assert(primaryEnvironment.windNedFps[0] == 0.0);
  assert(primaryEnvironment.windNedFps == baselineEnvironment.windNedFps);
  assert(primaryEnvironment.gustNedFps == baselineEnvironment.gustNedFps);
  assert(primaryEnvironment.turbulenceNedFps
         == baselineEnvironment.turbulenceNedFps);

  auto *primaryManager =
      primarySource->GetComponent<control::FlightControlManager>();
  auto *baselineManager =
      baselineSource->GetComponent<control::FlightControlManager>();
  assert(primaryManager != nullptr);
  assert(baselineManager != nullptr);
  auto *primaryRollHold =
      dynamic_cast<gnc::IRollHoldAutopilot *>(&primaryManager->GetAutopilot());
  auto *baselineRollHold =
      dynamic_cast<gnc::IRollHoldAutopilot *>(&baselineManager->GetAutopilot());
  assert(primaryRollHold != nullptr);
  assert(baselineRollHold != nullptr);
  assert(primaryRollHold->IsRollHoldEnabled());
  assert(baselineRollHold->IsRollHoldEnabled());
  assert(std::abs(primaryRollHold->GetTargetRollRad()
                  - math::DegToRad(scenario.commandedRollDeg))
         < 1.0e-12);
  assert(primaryRollHold->GetTargetRollRad()
         == baselineRollHold->GetTargetRollRad());

  control.StopSimulation();
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());
  assert(!primaryRollHold->IsRollHoldEnabled());
  assert(!baselineRollHold->IsRollHoldEnabled());

  SetNorthWind(*primarySource, 12.0);
  SetNorthWind(*baselineSource, -9.0);
  sim::SimulationScenario windScenario = scenario;
  windScenario.name = "Shared Wind";
  windScenario.windEnabled = true;
  assert(control.RunScenario(windScenario));
  const sim::FDMEnvironmentState sharedPrimaryEnvironment =
      GetEnvironment(*primarySource);
  const sim::FDMEnvironmentState sharedBaselineEnvironment =
      GetEnvironment(*baselineSource);
  assert(NearlyEqual(sharedPrimaryEnvironment.windNedFps[0], 12.0));
  assert(sharedPrimaryEnvironment.windNedFps
         == sharedBaselineEnvironment.windNedFps);
  control.StopSimulation();
}
} // namespace

int main() {
  TestInteractiveExecutionHasNoScenarioState();
  TestScenarioAppliesSharedExperimentToBothSimulations();
  return 0;
}
