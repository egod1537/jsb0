#include "messaging/GuiSimBridge.hpp"
#include "messaging/MessageQueues.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/MyAutopilot.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/runtime/SimRuntime.hpp"
#include "sim/execution/ExecutionVariantResolver.hpp"
#include "sim/scenario/SimScenario.hpp"
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

sim::ExecutionRequest MakeRequest(const sim::SimScenario &scenario,
    sim::ExecutionVariant variant = sim::ExecutionVariant::Primary) {
  return {.scenario = scenario, .variant = variant};
}

sim::ResolvedExecutionSpec Resolve(const sim::SimScenario &scenario,
    sim::ExecutionVariant variant) {
  sim::ResolvedExecutionSpec resolved;
  std::string error;
  assert(sim::ExecutionVariantResolver::Resolve(MakeRequest(scenario, variant),
      resolved,
      error));
  return resolved;
}

void TestInteractiveRuntimeExecution() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  sim::SimRuntime runtime(MakePrimarySimulation());
  app::messaging::GuiSimBridge bridge(commands, events, telemetry, runtime);
  app::SimMessageClient control(commands, events, telemetry);
  const auto dispatch = [&] {
    commands.Drain();
    events.Drain();
    telemetry.Drain();
  };

  assert(control.GetSimExecutionState() == app::SimExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());
  assert(!control.GetSimSnapshot().appliedExecution.has_value());
  assert(control.RunExecution(MakeRequest(sim::SimScenario{})));
  dispatch();
  assert(control.GetLastCommandError().has_value());
  assert(runtime.Initialize());
  bridge.PublishState();
  events.Drain();

  control.StartSimulation();
  dispatch();
  assert(control.GetSimExecutionState() == app::SimExecutionState::Running);
  assert(runtime.Tick());
  bridge.PublishState();
  events.Drain();
  const double runningTime =
      control.GetSimSnapshot().primary.aircraft.simulationTimeSec;
  assert(runningTime > 0.0);

  control.PauseSimulation();
  control.RequestSimTick();
  dispatch();
  assert(control.GetPendingSimTickCount() == 1);
  assert(runtime.Tick());
  bridge.PublishState();
  events.Drain();
  assert(control.GetPendingSimTickCount() == 0);
  assert(control.GetSimSnapshot().primary.aircraft.simulationTimeSec
         > runningTime);

  assert(control.ResetSimulation());
  dispatch();
  assert(control.GetSimExecutionState() == app::SimExecutionState::Paused);
  control.ResumeSimulation();
  control.StopSimulation();
  dispatch();
  assert(control.GetSimExecutionState() == app::SimExecutionState::Stopped);
}

void TestScenarioExecutesOnlyScenarioSelectedAutopilot() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  sim::SimRuntime runtime(MakePrimarySimulation(), MakeBaselineSimulation());
  assert(runtime.Initialize());
  app::messaging::GuiSimBridge bridge(commands, events, telemetry, runtime);
  app::SimMessageClient control(commands, events, telemetry);
  const auto dispatch = [&] {
    commands.Drain();
    events.Drain();
    telemetry.Drain();
  };
  bridge.PublishState();
  events.Drain();

  sim::SimScenario scenario;
  scenario.name = "Dual Roll Hold";
  scenario.initialCondition.altitudeAslM = math::FeetToMeters(4200.0);
  scenario.initialCondition.calibratedAirspeedMps =
      math::KnotsToMetersPerSecond(105.0);
  scenario.initialCondition.rollRad = math::DegToRad(2.0);
  scenario.initialCondition.pitchRad = math::DegToRad(1.0);
  scenario.initialCondition.headingRad = math::DegToRad(35.0);
  scenario.runTrim = false;
  scenario.events.front().timeSec = 0.0;
  scenario.events.front().command.rollRad = math::DegToRad(8.0);
  scenario.durationSec = 12.0;

  sim::SimScenario invalidScenario = scenario;
  invalidScenario.settlingBandRad = -1.0;
  assert(control.RunExecution(MakeRequest(invalidScenario)));
  dispatch();
  assert(control.GetLastCommandError().has_value());
  assert(control.GetSimExecutionState() == app::SimExecutionState::Stopped);
  assert(!control.GetScenarioExecutionStatus().has_value());

  assert(control.RunExecution(MakeRequest(scenario)));
  dispatch();
  assert(control.GetSimExecutionState() == app::SimExecutionState::Running);
  const auto status = control.GetScenarioExecutionStatus();
  assert(status.has_value());
  assert(status->name == scenario.name);
  assert(status->elapsedSec == 0.0);
  assert(status->durationSec == scenario.durationSec);

  const sim::SimSnapshot snapshot = control.GetSimSnapshot();
  assert(snapshot.appliedExecution.has_value());
  assert(snapshot.appliedExecution->scenario == scenario);
  assert(snapshot.appliedExecution->variant == sim::ExecutionVariant::Primary);
  assert(snapshot.baseline.has_value());
  assert(snapshot.baselineAutopilot.has_value());
  const sim::InitialCondition &primaryCondition =
      snapshot.primary.currentCondition;
  assert(NearlyEqual(primaryCondition.altitudeAslM,
      scenario.initialCondition.altitudeAslM));
  assert(NearlyEqual(primaryCondition.calibratedAirspeedMps,
      scenario.initialCondition.calibratedAirspeedMps));
  assert(
      NearlyEqual(primaryCondition.rollRad, scenario.initialCondition.rollRad));
  assert(NearlyEqual(primaryCondition.pitchRad,
      scenario.initialCondition.pitchRad));
  assert(NearlyEqual(primaryCondition.headingRad,
      scenario.initialCondition.headingRad));
  assert(snapshot.primaryAutopilot.primaryRollHold.enabled);
  assert(!snapshot.baselineAutopilot->baselineRollHold.enabled);
  assert(std::abs(snapshot.primaryAutopilot.primaryRollHold.targetRollRad
                  - scenario.events.front().command.rollRad)
         < 1.0e-12);
  const auto primaryTelemetry =
      control.GetTelemetrySnapshot(sim::SimSlot::Primary);
  const auto baselineTelemetry =
      control.GetTelemetrySnapshot(sim::SimSlot::Baseline);
  assert(primaryTelemetry != nullptr && primaryTelemetry->available);
  assert(baselineTelemetry != nullptr && baselineTelemetry->available);
  assert(primaryTelemetry != baselineTelemetry);

  control.StopSimulation();
  dispatch();
  const sim::SimSnapshot stopped = control.GetSimSnapshot();
  assert(stopped.status.executionState == sim::SimExecutionState::Stopped);
  assert(!stopped.status.scenario.has_value());
  assert(stopped.appliedExecution.has_value());
  assert(stopped.appliedExecution->scenario == scenario);
  assert(!stopped.primaryAutopilot.primaryRollHold.enabled);
  assert(stopped.baselineAutopilot.has_value());
  assert(!stopped.baselineAutopilot->baselineRollHold.enabled);

  assert(control.RunExecution(MakeRequest(invalidScenario)));
  dispatch();
  assert(control.GetLastCommandError().has_value());
  const sim::SimSnapshot afterRejectedApply = control.GetSimSnapshot();
  assert(afterRejectedApply.appliedExecution.has_value());
  assert(afterRejectedApply.appliedExecution->scenario == scenario);
}

void TestBaselineScenarioSelectsBaselineWithoutRebuild() {
  sim::SimRuntime runtime(MakePrimarySimulation(), MakeBaselineSimulation());
  assert(runtime.Initialize());
  sim::SimScenario scenario;
  scenario.runTrim = false;
  scenario.events.front().timeSec = 0.0;
  scenario.durationSec = 0.1;
  assert(
      runtime.RunExecution(Resolve(scenario, sim::ExecutionVariant::Baseline)));
  const sim::SimSnapshot running = runtime.GetSnapshot();
  assert(running.primaryAutopilot.strategyName == "PX4Autopilot");
  assert(running.primaryAutopilot.baselineRollHold.enabled);
  assert(running.baselineAutopilot.has_value());
  assert(running.baselineAutopilot->strategyName == "MyAutopilot");
  runtime.Stop();
  const sim::SimSnapshot restored = runtime.GetSnapshot();
  assert(restored.primaryAutopilot.strategyName == "MyAutopilot");
  assert(restored.baselineAutopilot->strategyName == "PX4Autopilot");
}
} // namespace

int main() {
  TestInteractiveRuntimeExecution();
  TestScenarioExecutesOnlyScenarioSelectedAutopilot();
  TestBaselineScenarioSelectsBaselineWithoutRebuild();
  return 0;
}
