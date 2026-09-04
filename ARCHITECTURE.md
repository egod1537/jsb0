# JSB0 Runtime Architecture

The source tree exposes architectural ownership directly. JSB0 Runtime owns
simulation execution and the external JSB0↔JSB1 contract; desktop and headless
entry points compose those modules without moving domain behavior into the
executable layer.

## Modules

- `src/app` is the desktop composition root. It constructs Runtime, messaging,
  and GUI and owns startup, scheduling, and shutdown.
- `src/sim` owns simulation state and execution, JSBSim access, control and
  autopilot behavior, scenarios, linearization, and runtime telemetry
  production. It must not depend on GUI or rendering libraries.
- `src/messaging` is the typed process/module boundary between Runtime and GUI.
  It owns the in-process bus, command/event contracts, client cache, and Runtime
  adapter.
- `src/gui` owns the ImGui editor, controller/view features, layouts, windows,
  platform services, and presentation-local state. It sends commands through
  messaging and may consume stable plain simulation contracts.
- `src/flightui` owns dumb ImGui/ImPlot controls plus passive 3D scene, camera,
  and drawing primitives. GUI-specific typed events remain in `src/gui`.
- `src/contract` owns C++ integration for the declarative root `contract/`
  source of truth: generated Protobuf types, recording DTOs, and MCAP adapters.
- `src/runner` is the JSB1-facing headless execution boundary. It loads a
  scenario, composes Runtime, and writes contract artifacts and exit status.
- `src/common` contains only subsystem-independent math, containers, and small
  utilities.

The repository-root `contract/` contains the machine-readable index, execution
capabilities, generated tunable-parameter catalog and parameter-set schema,
artifact layout, Protobuf, JSON Schema, signal catalog, version, and examples.
The parameter export is derived from the Runtime's typed controller metadata
and aircraft profile and checked byte-for-byte. `src/contract/` contains only
the C++ adapters that implement that specification.

## Dependency direction

```text
               app
             ┌──┴────┐
             v       v
            gui  messaging
             │       │
             v       v
          flightui  sim
                     │
                     v
                  contract
                     │
                     v
                   common

runner ────────────> sim + contract
```

`sim` must not include GUI, FlightUI, messaging, or ImGui/GLFW. `common` must
not include any higher-level subsystem. GUI code must not include concrete
`SimRuntime`, JSBSim wrappers, or autopilot implementations. FlightUI
may consume stable render-state structs but must not know GUI controllers,
messaging, Runtime, or external contract adapters.

The desktop messaging path is `GUI -> SimMessageClient -> MessageBus ->
GuiSimBridge -> SimRuntime`. `GuiSimBridge` belongs to the messaging
boundary and does not depend on the concrete GUI implementation.

## CMake targets

```text
jsb_common
jsb_contract_proto -> jsb_contract
jsb_sim_core -> jsb_sim_runtime
jsb_message_bus -> jsb_messaging
jsb_gui_architecture + jsb_monitor + flightui -> jsb_editor
jsb_editor -> jsb_app -> jsb-flight-console
jsb_sim_runtime + jsb_contract -> jsb_runner -> jsb-sim-runner
```

`jsb_monitor` remains separate because it has useful headless controller tests.
Other GUI features stay together in `jsb_editor` to avoid target fragmentation.

## Boundary enforcement

`scripts/check_architecture.py` scans source includes for prohibited reverse
dependencies. CTest runs it as `architecture_boundaries`. It is intentionally a
small guardrail rather than a new dependency-analysis framework.

Namespaces retain their established names in this filesystem/CMake refactor.
Future namespace normalization can introduce `jsb::...` incrementally without
combining that source-level churn with ownership moves.

## Lifecycle and event handler naming

- `OnXxx` identifies a callback invoked when an external lifecycle transition
  or event occurs, such as `OnTick()`, `OnEvent(...)`, or `OnSignal(...)`.
- `HandleXxx` identifies command or message dispatch handling, such as
  `HandleResetCommand(...)`.
- A plain imperative verb identifies an operation the caller asks an object to
  perform, such as `Initialize()`, `Reset()`, `Step()`, `Update()`, or
  `PublishState()`.

Do not add `On` merely because an operation resembles a lifecycle verb. The
distinction is whether the method receives notification of something that has
happened or actively performs the requested operation.

## Simulation internals

`src/sim` has two CMake layers with a one-way dependency:

```text
jsb_sim_runtime
  SimRuntime
    -> SimInstanceSet
    -> AutopilotConfigurationService
    -> ScenarioExecutor
    -> SimSnapshotBuilder
    -> LinearizationService
    -> TelemetryRecordingService
          |
          v
jsb_sim_core
  Simulation (one aircraft/model instance)
    -> Aircraft / JSBSim boundary
    -> FlightControlManager
    -> TrimWorkflow / TrimService
    -> SimTelemetryPublisher -> TelemetryRegistry
```

`Simulation` owns and advances exactly one aircraft/model instance. It directly
owns `FlightControlManager`, applies one instance's controls, advances one
JSBSim tick, and exposes that instance's state. It does not choose primary
versus baseline, run a scenario schedule, publish application status, or
implement recording policy.

`SimRuntime` is the application-facing orchestrator. Primary and
baseline are both ordinary `Simulation` objects; `SimInstanceSet`
applies the same initialize, reset, shutdown, and step path to them and owns
only pair coordination such as manual-control synchronization. Scenario YAML
loading remains in `SimScenarioSerializer`, while deterministic command
scheduling and pass/fail state remain in `ScenarioExecutor`.

Telemetry signal mapping is owned by `SimTelemetryPublisher`; storage
and immutable frame/snapshot capture stay in `TelemetryRegistry`; recording is
implemented by `TelemetryRecordingService` and merely coordinated by Runtime.
Signal display name/symbol/unit metadata is resolved in the lightweight
`jsb_telemetry_contracts` layer and is included in immutable snapshots.
Monitor consumes this metadata rather than duplicating a signal-name/unit
table.
`SimSnapshotBuilder` translates domain state into stable Runtime
contracts without putting GUI naming or view state in the core.
Autopilot-specific settings validation and application are isolated in
`AutopilotConfigurationService`; Runtime only coordinates the request and any
recording event it produces.

Trim algorithms remain in `TrimSolver`/`TrimService`, and shared compute/apply/
control-synchronization sequencing is in `TrimWorkflow`. Linearization access
is exposed through `LinearizationService`; numeric linearization and mode
analysis remain under `src/sim/linearization`.

Errors stay at their source: `Simulation::ErrorTracker` reports one-instance
input and JSBSim failures; `ScenarioExecutor` reports scenario failures, and
`SimRuntime::lastError` reports orchestration failures. Messaging and GUI
boundaries may present those strings but do not create simulation-domain
errors.

## Monitor internals

Monitor follows a one-way Model/View/Controller boundary. `MonitorController`
owns persistent `MonitorState`; `MonitorView` only composes the page and emits
typed events. Preset and signal catalogs are headless data modules. Grid/card,
toolbar, preset panel, Add/Edit dialog, timeline, dynamic-mode view, plot
renderer, series legend/tooltip, and generic acceptance-band rendering are
separate responsibilities under `src/gui/features/monitor`.

Primary/Baseline/Compare remains one `MonitorState::displayMode` value passed
down to the renderer. Plot components do not cache source mode or telemetry.
All plots share the same timeline ranges and cursor, and temporary dialog
buffers remain view-local rather than entering persistent Monitor state.

## GNC internals

GNC data flow is navigation -> guidance -> energy/high-level control ->
attitude/rate control -> actuator command. The current PX4 coordinator routes
typed `FixedWingSetpoint` values through course guidance and TECS into the
existing roll/pitch/yaw controllers. Control equations remain in controller
classes; aircraft defaults enter through `Px4ControlProfile`, and trim enters
through one `AircraftTrimReference` boundary.

PX4 and experimental construction are separate composition packages below
`src/sim/gnc/autopilot`. The root factory selects a package but does not build
its controller graph. Internal reverse dependencies and PX4/experimental
cross-includes are checked by `architecture_boundaries`. See
`docs/GNC_ARCHITECTURE.md` for the controller responsibility map and extension
points.

PX4 controller tuning uses typed descriptor and pointer-to-member binding
tables from `src/sim/gnc/parameters`. Controller algorithms continue to consume
strongly typed settings; the common layer supplies metadata, unit-aware display
conversion, validation, default reset, and generic UI iteration without a
string-valued source of truth. C172x values are composed once by
`GetC172xPx4ControlProfile()`. Runtime-state reset, algorithm-default reset,
aircraft-profile reset, and trim synchronization remain separate operations.
See `docs/PARAMETER_SYSTEM.md` for the policy and compatibility boundary.

PX4 attitude, lateral, and yaw controllers live under
`src/sim/gnc/control`; pre-PX4 experimental controllers and their dynamics
DTOs are isolated under `control/legacy`. The user-facing hold APIs,
telemetry paths, scenario keys, and parameter IDs remain unchanged.

GUI GNC composition mirrors these feature boundaries. `GNCController` owns
snapshot/configuration coordination while PX4 attitude, TECS, and trim each
own their typed events, editing controller, validation, reset, and view state.
`BaselineAutopilotPanel` preserves the existing section order as a thin
composition wrapper over `Px4AttitudePanel` and `TecsPanel`. All PX4 tuning
rows reuse the descriptor-driven GNC parameter editor. See
`docs/GUI_GNC_ARCHITECTURE.md` for state and messaging ownership.

## Units and physical quantities

Simulation-domain state, controller/configuration state, and raw telemetry use
SI units. JSBSim-native and versioned external scenario units are converted
once at their boundary through the shared math conversion helpers. Altitude
references (AGL versus ASL/MSL) and airspeed semantics (CAS versus TAS) are
encoded in field names and metadata rather than inferred. Display layers may
render angles in degrees and the Trim panel may use aviation units, but their
models and messages remain SI. See `docs/UNITS.md` for the complete boundary
and display policy.
