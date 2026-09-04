# GNC Architecture

This refactor preserves every existing control equation, gain, limit,
enable/reset transition, and telemetry signal. It establishes composition,
setpoint, trim, and aircraft-profile boundaries for future guidance and
experimental controllers.

## Composition graph

```text
FlightControlManager
  -> IAutopilot
       -> MyAutopilot
            -> RollHoldController
            -> AsyncAircraftLinearizer
       -> PX4Autopilot
            -> Px4CourseController
            -> Px4RollController
            -> Px4TecsController
            -> Px4PitchController
            -> Px4YawRateController

TrimResult + Aircraft
  -> ITrimReferenceConsumer
       -> each concrete autopilot
```

Controller source ownership follows implementation responsibility:

```text
control/
  attitude/   PX4 roll and pitch attitude/rate cascades
  lateral/    PX4 course-to-roll control
  yaw/        PX4 yaw-rate control
  legacy/     experimental/unused pre-PX4 controllers and dynamics DTOs
```

User-facing hold mode APIs, settings/diagnostics type names, telemetry paths,
scenario keys, and PX4 parameter IDs retain their established names. Only
internal controller class names and source ownership reflect the control
algorithm responsibility.

## Controller responsibility map

| Module | Input | Output | Runtime state | Settings/diagnostics | Trim dependency |
| --- | --- | --- | --- | --- | --- |
| `Px4CourseController` | Aircraft course, ground velocity, roll; course setpoint; tick | Roll setpoint | Roll-setpoint slew state | Course settings and course/lateral diagnostics | None |
| `Px4RollController` | Roll/rate/airspeed; roll setpoint; tick | Normalized aileron | Rate integrator | PX4 roll settings and contribution/saturation diagnostics | Trim airspeed and aileron |
| `Px4PitchController` | Pitch/rate/airspeed; pitch setpoint; tick | Normalized elevator | Rate integrator | PX4 pitch settings and contribution/saturation diagnostics | Trim airspeed and elevator |
| `Px4TecsController` | AGL altitude, vertical speed, CAS, pitch, throttle, altitude/CAS targets, gravity, dt | Pitch setpoint and normalized throttle | Shaped references, filtered rates, pitch/throttle integrators | TECS settings, energy/protection/limit diagnostics | Trim throttle; synchronized current state on first update |
| `Px4YawRateController` | Roll/pitch/yaw rate, sideslip, CAS, roll command, tick | Normalized rudder | Rate integrator and washout state | PX4 yaw settings and contribution/saturation diagnostics | Trim airspeed and rudder |
| `RollHoldController` (legacy/experimental) | Aircraft roll/rate and `ControlContext` | Aileron | No dynamic controller state | Primary settings and roll diagnostics | Trim aileron |
| `PitchHoldController` (legacy/unused) | Aircraft pitch/rate and `ControlContext` | Elevator | No dynamic controller state | Primary settings | Trim elevator |
| `AirspeedHoldController` (legacy/unused) | Aircraft CAS and tick | Throttle | No dynamic controller state | Airspeed settings | Trim throttle |
| `AltitudeHoldController` (legacy/unused) | Configuration only; no active output path | None | No dynamic controller state | Legacy altitude settings | Trim elevator |
| `YawDamperController` (legacy/unused) | Aircraft yaw rate, tick, `ControlContext` | Rudder | Washout filter | Yaw diagnostics | Trim rudder |
| `MyAutopilot` | Aircraft/tick/manual pass-through | `ControlInput` | Controller registry and async analysis state | Controller inspection and linearization diagnostics | Shared aircraft trim reference |
| `PX4Autopilot` | Aircraft/tick/manual pass-through plus fixed-wing setpoints | `ControlInput` | Mode ownership and owned controller state | Aggregated PX4 diagnostics plus module settings | Shared aircraft trim reference |

## Current layer graph

```text
navigation/                    (future state/mission source)
      |
      v
guidance/FixedWingSetpoint     course, roll, altitude AGL, CAS, pitch
      |
      +---------------------> guidance/lateral concept
      |                         Px4CourseController -> roll SP
      v
energy control concept
  Px4TecsController ------------------------------+
      | pitch SP + throttle SP                     |
      v                                            |
attitude/rate control                              |
  Px4RollController    -> aileron                  |
  Px4PitchController   -> elevator                 |
  Px4YawRateController           -> rudder         |
      |                                            |
      +---------------- actuator command <---------+

autopilot/px4 composition          autopilot/experimental composition
  -> PX4Autopilot                     -> MyAutopilot today
  -> Px4ControlProfile                -> LQR/MPC/custom composition later
```

The former mixed-responsibility source directory has been removed. Legacy
controllers remain buildable for experiments and tests, but PX4 composition
does not register them.

## PX4Autopilot ownership

`PX4Autopilot` is the fixed-wing coordinator. It owns modules, routes
`FixedWingSetpoint` values, coordinates TECS-versus-direct-pitch ownership,
resets modules, applies one shared trim reference, and exposes aggregate
diagnostics. Course, energy, attitude/rate, protection, and actuator control
equations remain in their existing controller classes.

The public scalar getters/setters remain for contract compatibility, but their
storage is now:

```text
FixedWingSetpoint
  lateral.courseRad
  lateral.rollRad
  longitudinal.altitudeAglM
  longitudinal.calibratedAirspeedMps
  longitudinal.pitchRad
```

This is the handoff point for future NPFG/L1/waypoint/loiter guidance. Guidance
can populate setpoints without changing an attitude or rate controller.

## Trim and aircraft configuration

`FlightControlManager` converts `Aircraft + TrimResult` once into
`AircraftTrimReference`. `IAutopilot` provides a lifecycle trim hook with a
default no-op, eliminating the one-method `ITrimReferenceConsumer` capability.
Concrete autopilots read only the reference values they need.

```text
TrimService result + normalized Aircraft state
  -> AircraftTrimReference
       altitudeAglM, calibratedAirspeedMps,
       throttle, elevator, aileron, rudder
  -> active autopilot
  -> owned controller settings
```

Synchronization occurs after a trim is applied during initialization, reset,
or an explicit trim request. Controller enable transitions reset dynamic
controller state but retain the most recently synchronized trim settings.

`Px4ControlProfile` aggregates course, roll, pitch, yaw, and TECS settings.
The PX4 composition factory injects `MakeC172xPx4ControlProfile()`. The public
default constructor retains algorithm defaults for compatibility with direct
controller tests and embeddings; application composition does not depend on
that implicit choice. A future aircraft can construct `PX4Autopilot` with
another profile without modifying controller algorithms.

## Interfaces and extension points

- `IAutopilot`: lifecycle, actuator update, and common trim lifecycle hook.
- `IRollHoldAutopilot`: scenario-specific roll command capability.
- `IAutopilotAnalysis`: optional linearization/analysis capability.
- `IControllerInspectable`: optional controller registry inspection used by
  diagnostics and experiments.

The dedicated trim interface was removed instead of adding another
capability. The remaining optional interfaces represent distinct consumers
and are not merged into one broad autopilot interface.

The root `AutopilotFactory` selects only a composition family. PX4 creation is
under `autopilot/px4`; experimental creation is under
`autopilot/experimental`. Adding an LQR/MPC implementation changes the
experimental composition, not `PX4Autopilot` or its controllers.

## Enforced dependency rules

`scripts/check_architecture.py` now rejects these internal reverse edges:

- control -> navigation, guidance, energy, or autopilot;
- energy -> navigation, guidance, or autopilot;
- the existing TECS energy controller -> navigation, guidance, or autopilot;
- navigation -> downstream guidance, energy, control, or autopilot;
- PX4 composition <-> experimental composition.

GUI and Monitor continue to consume stable Runtime snapshots and telemetry;
they do not include controller implementation details.

## Parameter and profile flow

Course, roll, pitch, yaw, and TECS expose strongly typed settings backed by a
common compile-time descriptor/binding pattern. The GUI iterates the same
descriptors used for bounds and defaults; PX4 control equations only access
named settings members. The central C172x `Px4ControlProfile` supplies the
aircraft-tuned snapshot, while trim references enter through
`AircraftTrimReference` and runtime state is reset by each controller. These
three sources are not interchangeable. Full validation and reset semantics are
documented in `docs/PARAMETER_SYSTEM.md`.
