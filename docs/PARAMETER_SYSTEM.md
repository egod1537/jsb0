# Controller Parameter System

## Scope and previous pattern

The shared parameter infrastructure covers the C172x PX4 baseline controller
families: course, roll, pitch, yaw, and TECS. The legacy experimental
`MyAutopilot` hold settings and trim requests remain strongly typed special
configuration because they do not expose the PX4 parameter catalog or the
generic baseline tuning UI.

Previously, each controller declared a different metadata structure and then
repeated the same mapping in getters, setters, controller validation, GUI
widgets, and reset handlers. In particular, TECS had 19-case getter and setter
switches, scenario roll configuration had another seven-case switch, and the
GUI had three near-identical hold renderers, a separate TECS renderer, and
hard-coded yaw rows. C172x values were repeated between metadata, Runtime
contract defaults, and GUI reset code.

## Ownership

`sim/gnc/parameters/Parameter.hpp` owns the reusable primitives:

- typed `ParameterMetadata<Enum>` descriptors;
- a closed `UnitId` set and display-boundary conversion;
- typed pointer-to-member `ParameterBinding<Enum, Settings>` entries;
- finite-value rejection, bounds clamping, normalization, and default reset;
- compile-time schema validation for enum order, count, bindings, bounds, and
  defaults.

Each controller still owns its parameter enum, descriptor array, binding array,
and strongly typed settings struct. Algorithms read named settings members and
never use strings or a runtime parameter map. The descriptor's stable `id`
preserves existing PX4/scenario keys; `displayName`, unit, bounds, default, and
edit increment are the single UI-facing source of truth.

Telemetry descriptors are deliberately separate. Runtime signals and mutable
configuration have different identity and lifetime even when they share a unit
symbol.

## Validation policy

Finite values are clamped to the descriptor range. Interactive setters reject
NaN and infinity without changing the current value. Applying a complete
settings snapshot replaces non-finite bound members with their descriptor
defaults and clamps finite out-of-range members. Controller-specific relational
constraints, such as TECS minimum/maximum pairs and trim throttle, remain in
the controller after generic member validation.

Angle and angular-rate settings remain radians and radians per second inside
controllers. A descriptor can select degrees for editing; conversion occurs in
the generic GUI renderer at the presentation boundary.

## Reset meanings

The following operations are intentionally distinct:

- `Controller::Reset()` clears runtime state such as integrators, filters, and
  synchronization state. It does not select tuning.
- `ResetPx4*ParametersToDefaults()` restores descriptor/algorithm defaults for
  bound parameters only. It preserves trim references, modes, and test state.
- `GetC172xPx4ControlProfile()` supplies the centralized aircraft tuning
  snapshot used by construction, Runtime contract defaults, and GUI tuning
  reset actions.
- trim synchronization updates aircraft operating-point references after
  initialization, reset, or a new trim result; it is not a parameter reset.

The TECS GUI reset retains its synchronized trim-throttle reference while
restoring the remaining C172x tuning, preserving the existing bumpless-transfer
policy.

## Compatibility

Public scenario/contract field names, PX4 parameter IDs, settings member names,
telemetry paths, and telemetry semantics are unchanged. UI rows are now emitted
from descriptors, with specialized mode/setpoint controls retained where a
numeric parameter row is not appropriate.
