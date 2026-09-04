# Simulation Core Responsibility Map

This refactor preserves the existing runtime, scenario, telemetry, GUI, and
recording contracts. It changes ownership boundaries, not flight dynamics or
controller behavior.

## Before

| Type/file | Responsibilities before the refactor |
| --- | --- |
| `Simulation` | One Aircraft/JSBSim instance, component/control tick, reset and initial trim sequencing, trim state, every controller/aircraft telemetry mapping, telemetry storage, errors |
| `SimulationRuntime` | Primary/baseline ownership and lifecycle, pair stepping and input synchronization, scenario coordination, command configuration, trim sequencing, linearization interface probing, snapshot DTO mapping, recording coordination, runtime state/errors |
| `Aircraft` | JSBSim model ownership, raw property boundary, controls, state extraction/application |
| `ScenarioExecutor` | Deterministic scenario reset, command schedule, step count, stop/result state |
| `SimulationScenarioSerializer` | Scenario YAML parse, validation boundary, load/save |
| `TrimService` / `TrimSolver` | Trim result lifetime and numerical solve/apply implementation |
| Linearization classes | Async scheduling, numerical perturbation, state-space result and dynamic-mode analysis |
| `TelemetryRegistry` | Signal registry, latest frame, history snapshot |

The two concentration points were telemetry/controller knowledge in
`Simulation.cpp` and domain-to-contract/analysis/paired-instance details in
`SimulationRuntime.cpp`.

## After

| Owner | Responsibility |
| --- | --- |
| `Simulation` | Own and execute one Aircraft/JSBSim instance; apply its component/control pipeline; advance one tick; reset/reinitialize it; expose stable state, telemetry storage, trim result, and domain error |
| `SimulationInstanceSet` | Apply identical initialize/reset/step/shutdown paths to the interactive primary and optional baseline; synchronize their manual command before a paired tick |
| `SimulationRuntime` | Application lifecycle state machine, primary/baseline policy, scenario/service coordination, public commands, recording coordination, status publication |
| `AutopilotConfigurationService` | Validate and apply primary/baseline controller settings and resolved execution parameters without exposing concrete autopilot details to Runtime |
| `ScenarioExecutor` | Scenario definition execution, deterministic command schedule, duration/stop state, scenario-local failure |
| `SimulationScenarioSerializer` | Scenario loader/saver role and unchanged YAML contract |
| `SimulationTelemetryPublisher` | Convert one Simulation's aircraft/controller diagnostics into the existing telemetry signal paths and SI values |
| `TelemetryRegistry` | Store signals and build immutable telemetry frames/snapshots |
| `TelemetryRecordingService` | Recording policy implementation and MCAP source adaptation |
| `SimulationSnapshotBuilder` | Convert one Simulation and analysis state into existing Runtime snapshot contracts |
| `TrimWorkflow` | Shared trim request conversion, compute/apply sequence, control synchronization, optional clock reset |
| `TrimService` / `TrimSolver` | Per-instance trim result and numerical trim algorithm |
| `LinearizationService` | Runtime-facing analysis capability access and snapshot state |
| `AsyncAircraftLinearizer` and analysis classes | Numerical linearization scheduling/calculation and dynamic-mode analysis |

Primary and baseline remain distinct instances because their autopilot
strategies differ, but both use the same `Simulation` primitive and
`SimulationInstanceSet` lifecycle path. Scenario runs intentionally drive only
the selected instance through `ScenarioExecutor`; variant selection remains a
Runtime policy.

## Dependency and error rules

The dependency direction remains `jsb_sim_runtime -> jsb_sim_core -> common`
and `sim` has no dependency on GUI, FlightUI, messaging, ImGui, or GLFW.

- `Simulation` creates errors for invalid per-instance input, component
  failures, and JSBSim failures.
- `ScenarioExecutor` creates schedule, validation, and scenario-step errors.
- `SimulationRuntime` creates application/orchestration errors and surfaces
  lower-layer errors without rebuilding GUI-specific text.
- Messaging and GUI layers only transport or present stable Runtime contracts.

No public Runtime/Simulation API, scenario schema, telemetry path/value, trim
behavior, or controller tuning was changed by this decomposition.
