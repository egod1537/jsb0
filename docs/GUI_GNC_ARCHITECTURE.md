# GUI GNC feature architecture

The GNC editor keeps simulation/controller state behind the existing typed
messaging boundary. GUI code never reaches into `SimulationRuntime` or a
concrete autopilot.

## Ownership

```text
GNCWindow (tabs and page composition)
  -> GNCController (shared snapshot/configuration coordination)
       -> ExperimentalController (current primary/MyAutopilot editing state)
       -> Px4AttitudeController (roll, pitch, course, yaw edit/reset/view state)
       -> TecsController (enable, SI setpoints, capture, tuning/reset)
       -> TrimController (request fields, execution, result view state)
  -> BaselineAutopilotPanel (visual-order composition only)
       -> Px4AttitudePanel
       -> TecsPanel
```

`GNCModel` is persistent editor state. Simulation snapshots are read-only
inputs, controller settings remain the runtime source of truth, and temporary
widget state stays in the view. The autopilot selector is therefore not copied
into feature controller state.

Feature events live with their feature:

- `trim/TrimEvents.hpp`
- `experimental/ExperimentalEvents.hpp`
- `px4/attitude/Px4AttitudeEvents.hpp`
- `px4/tecs/TecsEvents.hpp`

`GNCEvents.hpp` is only the compatibility/composition include plus the shared
manual-control event. Adding guidance or an experimental controller should add
a sibling feature and have the top-level window/controller compose it; it
should not extend a common giant event enum.

## Parameter editing and validation

`components/ParameterEditor.hpp` renders any typed GNC parameter descriptor.
It owns display-unit conversion and editor bounds; feature controllers own
finite-value validation and application to strongly typed settings. Special
controls such as enable toggles and TECS setpoint capture remain feature-local.

TECS altitude capture carries meters AGL and airspeed capture carries meters
per second CAS. No unit conversion occurs in `GNCController`.

## Messaging flow

```text
panel event -> feature controller -> GNC composition controller
            -> SimulationMessageClient -> typed command -> runtime

runtime snapshot -> SimulationMessageClient cache -> GNCWindow
                 -> feature synchronization/model
```

The existing complete primary/baseline configuration command remains the
public messaging contract. `GNCController` aggregates feature state only at
that command boundary so no runtime or scenario contract changed.

## Extension points

- PX4 guidance: `features/gnc/px4/guidance/`
- Experimental UI such as LQR: `features/gnc/experimental/`

Those features may reuse `ParameterEditor` and typed messaging commands but do
not need changes to the TECS, attitude, or trim controllers.
