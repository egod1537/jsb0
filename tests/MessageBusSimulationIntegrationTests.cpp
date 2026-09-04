#include "messaging/GuiSimBridge.hpp"
#include "messaging/MessageQueues.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/Simulation.hpp"
#include "sim/gnc/autopilot/AutopilotFactory.hpp"
#include "sim/runtime/SimRuntime.hpp"
#include "sim/scenario/SimScenario.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {
struct Harness {
  Harness()
      : runtime(std::make_unique<sim::Simulation>(
                    gnc::CreateAutopilot(gnc::AutopilotKind::Primary)),
            std::make_unique<sim::Simulation>(
                gnc::CreateAutopilot(gnc::AutopilotKind::Baseline))),
        bridge(commands, events, telemetry, runtime),
        client(commands, events, telemetry) {}

  bool Initialize() {
    const bool initialized = runtime.Initialize();
    bridge.PublishState();
    events.Drain();
    telemetry.Drain();
    return initialized;
  }

  bool Tick() {
    commands.Drain();
    const bool succeeded = runtime.Tick();
    bridge.PublishState();
    events.Drain();
    telemetry.Drain();
    return succeeded;
  }

  void Dispatch() {
    commands.Drain();
    events.Drain();
    telemetry.Drain();
  }

  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  sim::SimRuntime runtime;
  app::messaging::GuiSimBridge bridge;
  app::SimMessageClient client;
};

void TestCommandsSnapshotsAndTelemetry() {
  Harness harness;
  assert(harness.Initialize());
  assert(harness.client.GetSimSnapshot().baseline.has_value());

  harness.client.StartSimulation();
  harness.Dispatch();
  assert(
      harness.client.GetSimExecutionState() == sim::SimExecutionState::Running);
  assert(harness.Tick());
  const sim::SimSnapshot runningSnapshot = harness.client.GetSimSnapshot();
  const double runningTime = runningSnapshot.primary.aircraft.simulationTimeSec;
  assert(runningTime > 0.0);
  assert(runningSnapshot.baseline.has_value());
  assert(std::abs(
             runningSnapshot.baseline->aircraft.simulationTimeSec - runningTime)
         < 1.0e-12);

  harness.client.PauseSimulation();
  harness.Dispatch();
  assert(
      harness.client.GetSimExecutionState() == sim::SimExecutionState::Paused);
  harness.client.RequestSimTick();
  harness.Dispatch();
  assert(harness.client.GetPendingSimTickCount() == 1);
  assert(harness.Tick());
  assert(harness.client.GetPendingSimTickCount() == 0);
  assert(harness.client.GetSimSnapshot().primary.aircraft.simulationTimeSec
         > runningTime);

  control::ControlInput input;
  input.aileron = 0.25;
  input.throttle = 0.7;
  assert(harness.client.SetManualControl(input));
  harness.Dispatch();
  assert(!harness.client.GetLastCommandError().has_value());
  assert(
      std::abs(
          harness.client.GetSimSnapshot().primaryAutopilot.manualControl.aileron
          - input.aileron)
      < 1.0e-12);
  harness.client.RequestSimTick();
  assert(harness.Tick());
  const sim::SimSnapshot synchronizedSnapshot = harness.client.GetSimSnapshot();
  assert(synchronizedSnapshot.baseline.has_value());
  assert(synchronizedSnapshot.baselineAutopilot.has_value());
  assert(std::abs(synchronizedSnapshot.baseline->aircraft.simulationTimeSec
                  - synchronizedSnapshot.primary.aircraft.simulationTimeSec)
         < 1.0e-12);
  assert(std::abs(synchronizedSnapshot.baselineAutopilot->manualControl.aileron
                  - input.aileron)
         < 1.0e-12);

  assert(harness.client.ResetSimulation());
  harness.Dispatch();
  const sim::SimSnapshot resetSnapshot = harness.client.GetSimSnapshot();
  assert(resetSnapshot.baseline.has_value());
  assert(std::abs(resetSnapshot.primary.aircraft.simulationTimeSec) < 1.0e-12);
  assert(
      std::abs(resetSnapshot.baseline->aircraft.simulationTimeSec) < 1.0e-12);
  const auto primaryTelemetry =
      harness.client.GetTelemetrySnapshot(sim::SimSlot::Primary);
  const auto baselineTelemetry =
      harness.client.GetTelemetrySnapshot(sim::SimSlot::Baseline);
  assert(primaryTelemetry != nullptr && primaryTelemetry->available);
  assert(baselineTelemetry != nullptr && baselineTelemetry->available);

  harness.client.ResumeSimulation();
  harness.Dispatch();
  assert(
      harness.client.GetSimExecutionState() == sim::SimExecutionState::Running);
  harness.client.StopSimulation();
  harness.Dispatch();
  assert(
      harness.client.GetSimExecutionState() == sim::SimExecutionState::Stopped);
}

void TestFailedRequestPreservesErrorAndSuccessClearsIt() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  sim::SimRuntime runtime(std::make_unique<sim::Simulation>(
      gnc::CreateAutopilot(gnc::AutopilotKind::Primary)));
  app::messaging::GuiSimBridge bridge(commands, events, telemetry, runtime);
  app::SimMessageClient client(commands, events, telemetry);

  assert(client.ResetSimulation());
  commands.Drain();
  events.Drain();
  const std::optional<std::string> failure = client.GetLastCommandError();
  assert(failure.has_value());
  assert(*failure == "Simulation reset failed.");

  assert(runtime.Initialize());
  bridge.PublishState();
  events.Drain();
  assert(client.ResetSimulation());
  commands.Drain();
  events.Drain();
  assert(!client.GetLastCommandError().has_value());
}

void TestBaselineAutopilotConfigurationFlowsThroughGuiBoundary() {
  Harness harness;
  assert(harness.Initialize());

  sim::BaselineRollHoldConfig config;
  config.enabled = true;
  config.pitchHoldEnabled = true;
  config.targetPitchRad = math::DegToRad(4.0);
  config.pitchTimeConstantSec = 0.65;
  config.maximumPositivePitchRateRadPerSec = math::DegToRad(45.0);
  config.maximumNegativePitchRateRadPerSec = math::DegToRad(35.0);
  config.pitchRateProportionalGain = 0.12;
  config.pitchRateIntegralGain = 0.09;
  config.pitchRateDerivativeGain = 0.01;
  config.pitchRateFeedForwardGain = 0.6;
  config.pitchIntegratorLimit = 0.3;
  config.yawRateControlEnabled = true;
  config.coordinatedTurnEnabled = true;
  config.yawRateProportionalGain = 0.8;
  config.yawRateIntegralGain = 0.0;
  config.yawRateDerivativeGain = 0.0;
  config.yawRateFeedForwardGain = 0.0;
  config.sideslipToYawRateGain = 8.0;
  config.yawRateWashoutTimeConstantSec = 0.0;
  config.rollToYawFeedForwardGain = 0.0;
  assert(harness.client.SetBaselineRollHoldConfig(config));
  harness.Dispatch();

  const sim::SimSnapshot configured = harness.client.GetSimSnapshot();
  assert(configured.baselineAutopilot.has_value());
  const sim::BaselineRollHoldConfig &actual =
      configured.baselineAutopilot->baselineRollHold;
  assert(actual.yawRateControlEnabled);
  assert(actual.coordinatedTurnEnabled);
  assert(actual.yawRateProportionalGain == 0.8);
  assert(actual.sideslipToYawRateGain == 8.0);
  assert(actual.pitchHoldEnabled);
  assert(actual.targetPitchRad == math::DegToRad(4.0));
  assert(actual.pitchTimeConstantSec == 0.65);
  assert(actual.maximumPositivePitchRateRadPerSec == math::DegToRad(45.0));
  assert(actual.maximumNegativePitchRateRadPerSec == math::DegToRad(35.0));
  assert(actual.pitchRateProportionalGain == 0.12);
  assert(actual.pitchRateIntegralGain == 0.09);
  assert(actual.pitchRateDerivativeGain == 0.01);
  assert(actual.pitchRateFeedForwardGain == 0.6);
  assert(actual.pitchIntegratorLimit == 0.3);

  harness.client.StartSimulation();
  harness.Dispatch();
  assert(harness.Tick());
  const auto telemetry =
      harness.client.GetTelemetrySnapshot(sim::SimSlot::Baseline);
  assert(telemetry != nullptr);
  assert(telemetry->Find("autopilot/yaw_rate/yaw_rate") != nullptr);
  assert(telemetry->Find("autopilot/yaw_rate/sideslip_rate_correction")
         != nullptr);
  assert(telemetry->Find("autopilot/pitch_hold/commanded_pitch") != nullptr);
  assert(telemetry->Find("autopilot/pitch_hold/elevator_command") != nullptr);
}

void TestClosedCommandQueueFailsCleanly() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  app::messaging::SimToGuiTelemetryQueue telemetry;
  app::SimMessageClient client(commands, events, telemetry);
  commands.Close();

  assert(!client.SetManualControl(control::ControlInput{}));
  const std::optional<std::string> failure = client.GetLastCommandError();
  assert(failure.has_value());
  assert(*failure == "Simulation command queue is closed.");
}

void TestRequestCompletionRunsOnlyDuringGuiEventDrain() {
  Harness harness;
  assert(harness.Initialize());
  const std::thread::id guiThread = std::this_thread::get_id();
  std::thread::id completionThread;
  bool completed = false;

  assert(harness.client.ResetSimulation(
      [&](bool succeeded, const std::string &error) {
        assert(succeeded);
        assert(error.empty());
        completed = true;
        completionThread = std::this_thread::get_id();
      }));
  harness.commands.Drain();
  assert(!completed);
  harness.events.Drain();
  assert(completed);
  assert(completionThread == guiThread);
}

void TestTrimAndScenarioRequestResults() {
  Harness harness;
  assert(harness.Initialize());

  assert(harness.client.SetAutomaticLinearizationEnabled(false));
  harness.Dispatch();
  const sim::SimSnapshot linearizationDisabled =
      harness.client.GetSimSnapshot();
  assert(linearizationDisabled.linearization.available);
  assert(!linearizationDisabled.linearization.automaticUpdatesEnabled);
  assert(harness.client.SetAutomaticLinearizationEnabled(true));
  harness.Dispatch();

  gnc::TrimRequest trimRequest;
  assert(harness.client.RunTrim(trimRequest, false));
  harness.Dispatch();
  const sim::SimSnapshot trimmed = harness.client.GetSimSnapshot();
  assert(trimmed.trim.result.has_value());
  assert(trimmed.trim.result->success);

  sim::SimScenario scenario;
  scenario.name = "Message bus integration";
  scenario.runTrim = false;
  scenario.durationSec = 0.2;
  scenario.events.front().timeSec = 0.1;
  assert(harness.client.RunExecution({
      .scenario = scenario,
      .variant = sim::ExecutionVariant::Primary,
  }));
  harness.Dispatch();
  assert(harness.client.GetScenarioExecutionStatus().has_value());
  while (harness.client.GetSimExecutionState()
         == sim::SimExecutionState::Running) {
    assert(harness.Tick());
  }

  const sim::SimSnapshot completed = harness.client.GetSimSnapshot();
  assert(completed.status.executionState == sim::SimExecutionState::Stopped);
  assert(!completed.status.scenario.has_value());
  assert(completed.appliedExecution.has_value());
  assert(completed.appliedExecution->scenario == scenario);
  assert(completed.appliedExecution->variant == sim::ExecutionVariant::Primary);
  assert(completed.primary.available);
  assert(completed.baseline.has_value() && completed.baseline->available);
  assert(harness.client.GetTelemetrySnapshot(sim::SimSlot::Primary)->available);
  assert(
      harness.client.GetTelemetrySnapshot(sim::SimSlot::Baseline)->available);
}

void TestTelemetryCacheRetainsFullSessionRangeEfficiently() {
  app::messaging::GuiToSimQueue commands;
  app::messaging::SimToGuiQueue events;
  constexpr std::size_t SampleCount = 10'000;
  app::messaging::SimToGuiTelemetryQueue telemetry(SampleCount);
  app::SimMessageClient client(commands, events, telemetry);
  for (std::size_t index = 0; index < SampleCount; ++index) {
    telemetry.Enqueue(app::messaging::TelemetryBatch{
        .frames = {{
            .slot = sim::SimSlot::Primary,
            .frame =
                {
                    .available = true,
                    .sequence = index + 1,
                    .timestamp = static_cast<double>(index),
                    .values = {{
                        .path = "test/long_history",
                        .value = index % 2 == 0 ? -1000.0 : 1000.0,
                    }},
                },
        }}});
  }
  telemetry.Drain();

  const auto snapshot = client.GetTelemetrySnapshot(sim::SimSlot::Primary);
  assert(snapshot != nullptr && snapshot->publishedTimeRange.has_value());
  assert(snapshot->publishedTimeRange->minSec == 0.0);
  assert(snapshot->publishedTimeRange->maxSec
         == static_cast<double>(SampleCount - 1));
  const telemetry::TelemetrySeries *series =
      snapshot->Find("test/long_history");
  assert(series != nullptr && !series->samples.empty());
  assert(series->samples.size() <= 4096);
  assert(series->samples.front().timeSec == 0.0);
  assert(
      series->samples.back().timeSec == static_cast<double>(SampleCount - 1));
  for (std::size_t index = 1; index < series->samples.size(); ++index) {
    assert(series->samples[index - 1].timeSec < series->samples[index].timeSec);
  }
  const bool retainedMinimum = std::any_of(series->samples.begin(),
      series->samples.end(),
      [](const telemetry::TelemetrySample &sample) {
        return sample.value == -1000.0;
      });
  const bool retainedMaximum = std::any_of(series->samples.begin(),
      series->samples.end(),
      [](const telemetry::TelemetrySample &sample) {
        return sample.value == 1000.0;
      });
  assert(retainedMinimum && retainedMaximum);
}
} // namespace

int main() {
  TestCommandsSnapshotsAndTelemetry();
  TestBaselineAutopilotConfigurationFlowsThroughGuiBoundary();
  TestFailedRequestPreservesErrorAndSuccessClearsIt();
  TestClosedCommandQueueFailsCleanly();
  TestRequestCompletionRunsOnlyDuringGuiEventDrain();
  TestTrimAndScenarioRequestResults();
  TestTelemetryCacheRetainsFullSessionRangeEfficiently();
  return 0;
}
