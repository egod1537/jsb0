# JSB0 Runtime Contract

JSB0 Runtime owns the executable scenario, telemetry, and run-metadata
contracts in this directory. JSB1 consumes these files for UI generation,
validation, MCAP decoding, visualization, analysis, and notebooks. JSB1 must
not redefine the meaning, unit, frame, sign, topic, or field of a JSB0 signal.

MCAP is the timestamped container for contract-defined Protobuf data; it is not
the contract itself. Every contract Protobuf channel embeds a binary
`FileDescriptorSet`, so a standalone MCAP file remains dynamically decodable.

## Version

`VERSION` is semantic version `1.0.0`; consumers must compare its major
component. Removing or renaming a topic, changing field semantics, units,
frames or sign conventions, changing a Protobuf field number, or adding a
required scenario/metadata field requires a major version increment.

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

`scenario/scenario.schema.json` describes the YAML accepted by the Runtime.
Unit suffixes are authoritative and are used consistently for numeric scenario
fields. v1 exposes the actually supported `roll_hold` capability, C172x model,
`primary|baseline` autopilots, and the three Runtime trim modes.

The Scenario is the authoritative definition of one reproducible execution.
It explicitly owns aircraft, `autopilot.type`, complete initial condition,
fixed timestep, duration, trim/environment choice, and the ordered typed event
sequence. Runtime implementation defaults that are not exposed remain part of
the JSB0 commit identity. A normal headless run accepts no CLI override for
aircraft, autopilot, timestep, duration, or trim:

```text
jsb0-runner --scenario scenarios/roll_hold_primary.yaml --output run
```

The same binary executes `roll_hold_primary.yaml` and
`roll_hold_baseline.yaml`; autopilot is run configuration, never build
identity. `run.json` and MCAP metadata record the full runtime commit SHA, the
SHA-256 digest of the original scenario bytes, and the resolved autopilot.

`metadata/run.schema.json` describes `run.json`. Its contract/runtime/scenario
identifiers are also copied into MCAP Metadata named `jsb0.run`.

## Signal semantics

`catalog/signals.yaml` maps logical names to one exact topic and field and adds
the unit, frame, axis, sign convention, required status, and range. Protobuf is
the wire shape; this catalog is the semantic layer.

The v1 recorder mapping is explicit:

| Runtime registry path/value | Registry unit | Contract field | Wire unit |
|---|---|---|---|
| `autopilot/roll_hold/commanded_roll` | deg | `RollControlState.commanded_roll_rad` | rad |
| `autopilot/roll_hold/roll` | deg | `RollControlState.roll_rad` | rad |
| `autopilot/roll_hold/commanded_roll_rate` | deg/s | `RollControlState.commanded_roll_rate_rad_s` | rad/s |
| `autopilot/roll_hold/roll_rate` | deg/s | `RollControlState.roll_rate_rad_s` | rad/s |
| `autopilot/roll_hold/aileron_command` | normalized | `RollControlState.aileron_command` | normalized |
| `aircraft/attitude/roll` | deg | `AircraftState.roll_rad` | rad |
| `aircraft/rates/p` | deg/s | `AircraftState.roll_rate_rad_s` | rad/s |
| `Tick.simTimeSec` | s | every message `sim_time_ns` | ns |

`Simulation.cpp` owns the registry publications and
`TelemetryRecordingService.cpp` owns the verified degree-to-radian conversion
at the recorder boundary.

## Generation, validation, and export

After configuring CMake:

```text
cmake --build build --target contract_generate_python
cmake --build build --target contract_validate
cmake --build build --target contract_export
```

The normal C++ build deterministically generates C++ Protobuf types under the
build tree. Python output is generated under `build/generated/python`; JSB1
must consume the exported contract or generated package rather than copying and
editing schemas.

## JSB1 consumer policy

JSB1 should read the scenario schema for form generation and validation, use
the embedded or exported Protobuf descriptors for MCAP decoding, use the signal
catalog for labels/units/conventions, and reject an unsupported contract major
version. It may add presentation metadata but not redefine Runtime semantics.

## Foxglove and MAVLink

The first contract contains scalar aircraft/control analysis fields, so no
Foxglove `Vector3`, `Quaternion`, `Pose`, or transform schema is forced into it.
Those schemas should be reconsidered when JSB0 publishes a verified 3D pose or
frame transform. Aircraft/control-specific messages remain JSB-owned. MAVLink's
definition/generation model is informative, but JSB0 does not introduce a
MAVLink dialect or force simulation diagnostics into MAVLink.
