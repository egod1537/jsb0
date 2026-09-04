# JSB0 Runtime Contract

JSB0 Runtime owns the executable scenario, telemetry, and run-metadata
contracts in this directory. JSB1 consumes these files for UI generation,
validation, MCAP decoding, visualization, analysis, and notebooks. JSB1 must
not redefine the meaning, unit, frame, sign, topic, or field of a JSB0 signal.

MCAP is the timestamped container for contract-defined Protobuf data; it is not
the contract itself. Every contract Protobuf channel embeds a binary
`FileDescriptorSet`, so a standalone MCAP file remains dynamically decodable.

## Version

`VERSION` is semantic version `2.0.0`; consumers must compare its major
component. Removing or renaming a topic, changing field semantics, units,
frames or sign conventions, changing a Protobuf field number, or adding a
required scenario/metadata field requires a major version increment.

`index.json` is the discovery entry point for a standalone consumer. Every
path in it is relative to the exported contract root; contracts never contain
repository or machine absolute paths.

Compatible changes include a new Protobuf field with a new field number and a
new optional JSON Schema property. Protobuf field numbers are never reused;
removed fields must be declared `reserved`. Adding a required JSON property or
removing an enum value is breaking.

## Telemetry topics

| Topic | Protobuf message |
|---|---|
| `/jsb/primary/aircraft/state` | `jsb.telemetry.v1.AircraftState` |
| `/jsb/primary/control/roll` | `jsb.telemetry.v1.RollControlState` |
| `/jsb/baseline/aircraft/state` | `jsb.telemetry.v1.AircraftState` |
| `/jsb/baseline/control/roll` | `jsb.telemetry.v1.RollControlState` |
| `/jsb/simulation/event` | `jsb.telemetry.v1.SimulationEvent` |

Topics are absolute, lower-case, and hierarchical. The source instance appears
immediately after `/jsb` for instance-specific data. Aliases for the same
semantic channel are not created. Channel `message_encoding` and Schema
`encoding` are both `protobuf`; Schema data is an imports-complete binary
`FileDescriptorSet`.

The initial v1 aircraft message intentionally contains only the roll state
needed by the supported roll-hold analysis. Future state is added only when the
Runtime can publish it with verified semantics.

## Scenario and metadata

`scenario/scenario.schema.json` describes the experiment-condition YAML
accepted by the Runtime. Unit suffixes are authoritative and are used
consistently for numeric Scenario fields. Scenario v1 exposes the supported
`roll_hold` capability, C172x model, and the three Runtime trim modes. Its
optional `controller_parameters` array is a whitelist of canonical Runtime
parameter IDs that an operator may tune for that Scenario; parameter values
and metadata are not duplicated into Scenario YAML.

Because this revision supports exactly one Scenario discriminator and one
aircraft, their schema `const`/`enum` values are the capability discovery
source. Separate scenario-type and aircraft catalogs would duplicate those
semantics and are deferred until multiple entries require display metadata.

`execution/capabilities.json` is the machine-readable JSB0 headless capability
source for JSB1. It declares the `compare` mode and its fixed `baseline` and
`primary` pair, the minimal runner invocation, parameter transport, Scenario
and telemetry versions, and the artifact-manifest location. The narrower
`execution/variants.json` remains the Execution Variant contract. JSB1 must
read these JSB0-owned artifacts rather than maintaining an authoritative list.

## Execution parameters

`execution/parameters.json` is the authoritative external catalog of Runtime
parameters that can actually be changed through headless execution. It exposes
the stable ID, label and description, controller/module, numeric type, display
unit, bounds, increment, supported variant and aircraft, algorithm default,
and aircraft-profile value. `default_value` is the effective default for the
declared C172x profile; `algorithm_default` preserves the distinction from the
controller's algorithm default even when the two currently match.

The catalog is generated deterministically from `ParameterMetadata`, the
controller binding, and `Px4ControlProfile`. The same exporter generates
`execution/parameter-set.schema.json`, including its ID whitelist and numeric
bounds. The build compares both committed files byte-for-byte with fresh C++
output, so numeric metadata is not maintained independently in C++ and JSON.
Only the seven baseline PX4 roll parameters are published because those are
the parameters the current runner accepts; metadata for other controllers is
not an execution capability.

A Scenario owns aircraft, complete initial condition, fixed timestep,
duration, trim/environment choice, ordered typed events, and common acceptance
criteria. An Execution Variant owns the runtime/control implementation. Every
headless invocation is one dual-variant Run and accepts no CLI override for
Scenario semantics or variant selection:

```text
jsb-sim-runner --scenario scenarios/roll_hold_5deg_30s.yaml --output out/comparison
```

When controller tuning is requested, the caller prepares
`out/comparison/parameters.yaml` before launch. The file contains one
`controller_parameters` mapping. The runner reads it by convention without
adding a CLI option, rejects IDs not declared by the Scenario whitelist, and
applies the values only to their supported variant. Unknown root fields,
unknown IDs, unsupported variants, non-numeric or non-finite values, and values
outside the published range are rejected. The file remains separate from the
exact Scenario snapshot and is retained as Run provenance. Consumers generate
an editor by intersecting the Scenario `controller_parameters` whitelist with
`execution/parameters.json`; values never belong in Scenario YAML.

It creates independent baseline and primary JSBSim/controller runtimes from
the same parsed Scenario and steps them sequentially on one authoritative
integer step clock. Duration, `dt`, trim request, initial condition,
environment, and event schedule are common. Event times are resolved to step
indices before execution, and divergence in simulation time or emitted command
events fails the whole comparison. A fatal failure in either variant stops the
pair; internal threading is not used.

`execution/artifacts.json` is the machine-readable output layout. Comparison
output is one `run.json`, exact `scenario.yaml` snapshot, and one
`telemetry.mcap`, plus the optional caller-supplied `parameters.yaml`. The MCAP contains both `/jsb/baseline/...` and
`/jsb/primary/...` channels using the same Protobuf schemas. Messages produced
at the same shared step use the same simulation-time log/publish timestamp.
`run.json` records `mode: compare`, the canonical variant pair, and per-variant
results. MCAP Metadata records `execution_mode=compare` and
`execution_variants=baseline,primary`.

The runtime loader can parse a legacy Scenario `autopilot` field for desktop
migration, but headless execution rejects it because one value cannot describe
the required pair. Saving the Scenario produces canonical YAML without that
legacy field.

Baseline and primary channels use identical Protobuf schemas, units, and
timing semantics and can be overlaid directly. Desktop `SimSlot`
namespaces remain unchanged and are mapped explicitly to Execution Variants.

`metadata/run.schema.json` describes `run.json`. Its contract/runtime/scenario
identifiers are also copied into MCAP Metadata named `jsb0.run`. When a
parameter set is present, `run.json` records its relative path, SHA-256 digest,
and affected variant. Failed and interrupted runs retain the legacy `error`
text and additionally emit a structured `failure` with a stable code,
category, message, and failing simulation time. Existing status values remain
`completed`, `failed`, and `interrupted`. Artifact references such as
`scenario.file` are output-root-relative and never expose host absolute paths.

## Signal semantics

`catalog/signals.yaml` maps logical names to one exact topic and field and adds
the unit, frame, axis, sign convention, presentation-neutral semantic group,
required status, and range. Protobuf is the wire shape; this catalog is the
semantic layer. Colors, plot layouts, and other consumer UI policy are not
part of this contract.

The v1 recorder mapping is explicit:

| Runtime registry path/value | Registry unit | Contract field | Wire unit |
|---|---|---|---|
| `autopilot/roll_hold/commanded_roll` | rad | `RollControlState.commanded_roll_rad` | rad |
| `autopilot/roll_hold/roll` | rad | `RollControlState.roll_rad` | rad |
| `autopilot/roll_hold/commanded_roll_rate` | rad/s | `RollControlState.commanded_roll_rate_rad_s` | rad/s |
| `autopilot/roll_hold/roll_rate` | rad/s | `RollControlState.roll_rate_rad_s` | rad/s |
| `autopilot/roll_hold/aileron_command` | normalized | `RollControlState.aileron_command` | normalized |
| `aircraft/attitude/roll` | rad | `AircraftState.roll_rad` | rad |
| `aircraft/rates/p` | rad/s | `AircraftState.roll_rate_rad_s` | rad/s |
| `Tick.simTimeSec` | s | every message `sim_time_ns` | ns |

`Simulation.cpp` owns the registry publications and
`TelemetryRecordingService.cpp` copies the SI registry values into the matching
SI wire fields without another unit conversion.

## Generation, validation, and export

After configuring CMake:

```text
cmake --build build --target contract_generate_python
cmake --build build --target contract_generate_parameters
cmake --build build --target contract_validate
cmake --build build --target contract_export
```

The normal C++ build deterministically generates C++ Protobuf types under the
build tree. Python output is generated under `build/generated/python`.
`contract_validate` verifies schemas, cross-file references, parameter/runtime
consistency, Scenario whitelists, catalog topic/field/unit mappings, variants,
and semantic version syntax. `contract_export` includes an imports-complete
`jsb_telemetry_v1.pb`; its consumer smoke test reads only the exported tree.
JSB1 must consume that exact-revision export rather than copying and editing
schemas.

## JSB1 consumer policy

JSB1 should read the scenario schema for form generation and validation, use
the embedded or exported Protobuf descriptors for MCAP decoding, use the signal
catalog for labels/units/conventions, and reject an unsupported contract major
version. It may add presentation metadata but not redefine Runtime semantics.

The `main` branch is the latest stable consumer contract. Development branches
such as `impl` may carry an unreleased development contract and must not be
treated as stable. An actual run is always governed by the contract exported
from its exact immutable JSB0 commit; Scenario schema, parameters,
capabilities, telemetry descriptors, and runner implementation must come from
that same revision.

## Foxglove and MAVLink

The first contract contains scalar aircraft/control analysis fields, so no
Foxglove `Vector3`, `Quaternion`, `Pose`, or transform schema is forced into it.
Those schemas should be reconsidered when JSB0 publishes a verified 3D pose or
frame transform. Aircraft/control-specific messages remain JSB-owned. MAVLink's
definition/generation model is informative, but JSB0 does not introduce a
MAVLink dialect or force simulation diagnostics into MAVLink.
