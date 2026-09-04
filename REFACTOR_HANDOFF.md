# Simulation/GUI Message Bus Refactor Note

작성일: 2026-08-29  
브랜치: `backend`

## Architecture

The interactive application composes a pure C++ synchronous message path:

```text
GUI
  <-> SimMessageClient
  <-> MessageBus
  <-> GuiSimBridge
  <-> SimRuntime
  <-> Simulation Core
```

The headless runner continues to use `SimRuntime` directly.

## Boundaries

- `SimRuntime` owns application lifecycle and coordinates scenarios,
  primary/baseline instances, trim, linearization, snapshots, and telemetry
  recording. The corresponding algorithms and data conversion live in focused
  sim services rather than Runtime itself.
- `GuiSimBridge` translates typed commands into runtime calls and publishes
  plain-data status, snapshot, telemetry, and result events. It does not depend
  on a concrete GUI implementation; it bridges GUI-side messaging and the
  simulation runtime.
- `SimMessageClient` publishes GUI commands and maintains local event
  caches. GUI rendering only reads those caches.
- `TelemetryRegistry` remains internal. GUI plotting consumes immutable
  `TelemetrySnapshot` data assembled from `TelemetryFrame` events.
- `Simulation` is the one-aircraft execution primitive. `SimInstanceSet`
  gives primary and baseline the same initialize/reset/step/shutdown path.
- `SimTelemetryPublisher`, `SimSnapshotBuilder`,
  `AutopilotConfigurationService`, `TrimWorkflow`, and `LinearizationService`
  own telemetry mapping, contract snapshots, autopilot-specific settings
  application, trim sequencing, and analysis access respectively.
- GNC setpoints are grouped in `FixedWingSetpoint`; applied trim state crosses
  the autopilot boundary as `AircraftTrimReference`; and C172x PX4 defaults are
  injected as `Px4ControlProfile`.
- PX4 and experimental autopilots have separate composition factories. New
  experimental LQR/MPC work does not require edits to the PX4 coordinator.
- `MessageBus` dispatch is synchronous and in-process. Subscription lifetime is
  explicit and RAII-based; removing a subscription during publication prevents
  later invocation from the current callback snapshot.

## Build targets

```text
jsb_sim_core
  <- jsb_sim_runtime
  <- jsb_messaging -> jsb_message_bus
  <- jsb_editor

jsb_sim_runtime <- jsb_runner
```

The application has no external messaging middleware or generated-interface
build step.

## Validation

Run:

```powershell
cmake --build build -j 8
ctest --test-dir build --output-on-failure
```

Result: the complete Debug build succeeded and all 22 tests passed, including
message-bus unit/integration coverage, scenario execution, telemetry, MCAP,
and the headless smoke test.

For the GUI-free path:

```powershell
cmake -S . -B build-headless-check -G Ninja -DJSB_BUILD_EDITOR=OFF `
  -DBUILD_DOCS=OFF
cmake --build build-headless-check --target jsb_sim_runtime jsb-sim-runner
ctest --test-dir build-headless-check --output-on-failure
```

Result: the editor-free runtime/runner build succeeded and all three headless
runner checks passed.

## Parameter/settings consolidation

The C172x PX4 course, roll, pitch, yaw, and TECS families now share
`ParameterMetadata`, `ParameterBinding`, unit/display conversion, finite-value
validation, clamping, and default-reset helpers. Settings structs remain the
algorithm source of truth and retain named members. The previous controller and
scenario switch mappings and duplicated GUI parameter rows were replaced by
typed descriptor iteration; telemetry metadata remains separate.

`GetC172xPx4ControlProfile()` is the central aircraft-profile snapshot used by
PX4 composition, Runtime contract defaults, and GUI aircraft-tuning reset.
Algorithm-default reset, aircraft-profile reset, runtime controller reset, and
trim synchronization are explicitly separate. External contract fields,
scenario keys, PX4 IDs, telemetry paths, and tuned numeric values are retained.
See `docs/PARAMETER_SYSTEM.md` and `parameter_tests`.

## Monitor decomposition

`MonitorView` is now page composition rather than a combined catalog, dialog,
layout, and ImPlot implementation. Headless model/controller code is paired
with declarative preset and signal catalogs; UI files separately own toolbar,
preset panel, grid/card, Add/Edit modal, timeline, and dynamic-mode rendering.
Plot axes/series/legend/tooltip and descriptor-driven acceptance bands live in
the plotting package.

Telemetry snapshots now include display name, symbol, unit, and description
metadata from `jsb_telemetry_contracts`. Monitor retains only grouping and
preset associations, while Primary/Baseline/Compare and timeline state remain
single values in `MonitorState`. Existing signal paths, presets, plot visuals,
layout semantics, and GUI events are unchanged.

## GUI GNC feature decomposition

The previous `GNCController` event switch was separated into experimental,
PX4 attitude, TECS, and trim feature controllers. Their events now live beside
each feature;
the root event header is a small composition include. TECS capture is a typed
feature action carrying meters AGL or meters per second CAS, and numeric
feature edits consistently reject non-finite values.

`BaselineAutopilotPanel` is now a visual-order composition wrapper. PX4
attitude and TECS rendering are separate components and share the generic,
descriptor-driven `ParameterEditor`. Runtime access still crosses only
`SimMessageClient`, and the complete primary/baseline configuration
commands are unchanged. See `docs/GUI_GNC_ARCHITECTURE.md`.
