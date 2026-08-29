#include "messaging/MessageBus.hpp"
#include "messaging/SimulationMessageAdapter.hpp"
#include "messaging/SimulationMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/MyAutopilot.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/runtime/SimulationRuntime.hpp"
#include "sim/scenario/SimulationScenario.hpp"
#include "common/math/Math.hpp"

#include <cassert>
#include <cmath>
#include <memory>

namespace {
bool NearlyEqual(double left, double right, double tolerance = 1.0e-6) {
  return std::abs(left - right) <= tolerance;
}

std::unique_ptr<sim::Simulation> MakePrimarySimulation() {
  return std::make_unique<sim::Simulation>(
      std::make_unique<gnc::MyAutopilot>());
}

std::unique_ptr<sim::Simulation> MakeBaselineSimulation() {
  return std::make_unique<sim::Simulation>(
      std::make_unique<gnc::PX4Autopilot>());
}

void TestInteractiveRuntimeExecution() {
  application::messaging::MessageBus bus;
  sim::SimulationRuntime runtime(MakePrimarySimulation());
  application::messaging::SimulationMessageAdapter adapter(bus, runtime);
  application::SimulationMessageClient control(bus);

  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());
  assert(!control.RunScenario(sim::SimulationScenario{}));
  assert(runtime.Initialize(sim::SimulationConfig{}));
  adapter.PublishState();

  control.StartSimulation();
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Running);
  assert(runtime.Tick());
  adapter.PublishState();
  const double runningTime =
      control.GetSimulationSnapshot().primary.aircraft.simulationTimeSec;
  assert(runningTime > 0.0);

  control.PauseSimulation();
  control.RequestSimulationTick();
  assert(control.GetPendingSimulationTickCount() == 1);
  assert(runtime.Tick());
  adapter.PublishState();
  assert(control.GetPendingSimulationTickCount() == 0);
  assert(control.GetSimulationSnapshot().primary.aircraft.simulationTimeSec
         > runningTime);

  assert(control.ResetSimulation());
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Paused);
  control.ResumeSimulation();
  control.StopSimulation();
  assert(control.GetSimulationExecutionState()
         == application::SimulationExecutionState::Stopped);
}

void TestScenarioAppliesSharedExperimentToBothSimulations() {
  application::messaging::MessageBus bus;
  sim::SimulationRuntime runtime(MakePrimarySimulation(),
      MakeBaselineSimulation());
  assert(runtime.Initialize(sim::SimulationConfig{}));
  application::messaging::SimulationMessageAdapter adapter(bus, runtime);
  application::SimulationMessageClient control(bus);
  adapter.PublishState();

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

  const sim::SimulationSnapshot snapshot = control.GetSimulationSnapshot();
  assert(snapshot.baseline.has_value());
  assert(snapshot.baselineAutopilot.has_value());
  const sim::InitialCondition &primaryCondition =
      snapshot.primary.currentCondition;
  const sim::InitialCondition &baselineCondition =
      snapshot.baseline->currentCondition;
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
  assert(snapshot.primary.fdmState.environment.windNedFps
         == snapshot.baseline->fdmState.environment.windNedFps);
  assert(snapshot.primary.fdmState.environment.gustNedFps
         == snapshot.baseline->fdmState.environment.gustNedFps);
  assert(snapshot.primaryAutopilot.primaryRollHold.enabled);
  assert(snapshot.baselineAutopilot->baselineRollHold.enabled);
  assert(std::abs(snapshot.primaryAutopilot.primaryRollHold.targetRollRad
                  - math::DegToRad(scenario.commandedRollDeg))
         < 1.0e-12);
  assert(snapshot.primaryAutopilot.primaryRollHold.targetRollRad
         == snapshot.baselineAutopilot->baselineRollHold.targetRollRad);
  const auto primaryTelemetry =
      control.GetTelemetrySnapshot(sim::SimulationSlot::Primary);
  const auto baselineTelemetry =
      control.GetTelemetrySnapshot(sim::SimulationSlot::Baseline);
  assert(primaryTelemetry != nullptr && primaryTelemetry->available);
  assert(baselineTelemetry != nullptr && baselineTelemetry->available);
  assert(primaryTelemetry != baselineTelemetry);

  control.StopSimulation();
  const sim::SimulationSnapshot stopped = control.GetSimulationSnapshot();
  assert(
      stopped.status.executionState == sim::SimulationExecutionState::Stopped);
  assert(!stopped.status.scenario.has_value());
  assert(!stopped.primaryAutopilot.primaryRollHold.enabled);
  assert(stopped.baselineAutopilot.has_value());
  assert(!stopped.baselineAutopilot->baselineRollHold.enabled);
}
} // namespace

int main() {
  TestInteractiveRuntimeExecution();
  TestScenarioAppliesSharedExperimentToBothSimulations();
  return 0;
}
