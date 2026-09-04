# Passive Monitor feature

Monitor is a one-way visualization consumer. The GUI root reads the latest
published snapshots from `SimMessageClient`'s cache and constructs a
`MonitorInput`. Monitor never queries runtime or simulation objects.

```text
SimRuntime -> GuiSimBridge -> bounded SimToGuiTelemetryQueue
                                      |
                                      v
                             SimMessageClient cache
                                      |
                                      v
                            GUI root input adapter
                                      |
                                      v
                                 MonitorInput
                                      |
                              MonitorController
                               /            \
                       MonitorState       MonitorView
                                            |
                       page composition / child views
```

## Module ownership

- `MonitorView` composes the Plots and Dynamic Modes pages and owns only
  per-frame interaction wiring.
- `model/MonitorModel.cpp` owns plot-slot and manual-axis invariants;
  `MonitorController` applies typed events and timeline transitions.
- `catalog/MonitorPlotPresetCatalog` owns declarative preset-to-template
  associations. Adding a preset does not require editing plot rendering.
- `view/MonitorToolbar`, `MonitorPresetPanel`, `MonitorGrid`,
  `MonitorPlotConfigDialog`, `MonitorTimeline`, and `MonitorDynamicModes` own
  their respective ImGui interactions. `MonitorPlotDialogModel` contains the
  temporary Add/Edit buffers, separate from persistent `MonitorState`.
- `plotting/MonitorPlotRenderer` owns axes, source composition, sample
  limiting, manual Y ranges, and overlay hooks. `MonitorPlotSeries` owns the
  legend, tooltip, and per-source series visibility. Course, roll, and pitch
  acceptance bands use one descriptor-driven underlay path.

## Dependency audit

Visualization inputs (category A):

- immutable primary and baseline `TelemetrySnapshot` instances;
- an immutable dynamic-mode history view and update-status values supplied in
  `MonitorDynamicModeInput`;
- telemetry path constants used to select snapshot series.

The GUI drains bounded `TelemetryBatch` transport at frame start. A slow GUI
may skip old live-Monitor batches, but Primary/Baseline source identity and
ordering within each retained batch are preserved. MCAP recording is a
separate lossless simulation-thread consumer and is unaffected by this queue.

Visualization-local state (category B):

- live/frozen state, total/view/visible ranges, cursor, selection and ticks;
- preset-backed plot definitions, active presets, series visibility and layout;
- pane sizes, timeline drag state and selected dynamic mode.

Commands/dependencies removed from Monitor (category C):

- `SimMessageClient` and its per-frame getters;
- the automatic-linearization command call;
- simulation/runtime/controller objects and mutable telemetry registries.

The automatic-linearization checkbox now emits
`MonitorAutomaticLinearizationChanged` to the GUI parent. The parent decides
which application controller handles that intent.

## State and interaction rules

`MonitorController` is the single owner of `MonitorState`. `MonitorView`
receives immutable state, edits a per-frame presentation copy, and emits typed
`MonitorEvent` values upward. Every plot receives the same timeline object;
plots never own independent live, viewport, visible-range, or cursor state.
Preset plot visibility is composed only from the active Monitor presets. The
preset pane is the single place where preset plot groups are enabled or
disabled.

The `PX4 Pitch Hold Diagnostics` preset follows the longitudinal control path
from pitch-attitude and pitch-rate tracking through PID/FF terms, torque and
elevator output, integrator limits, airspeed scaling, angle of attack, pitch
acceleration, and saturation state. Its pitch-tracking plot renders the
configured command-relative tolerance band.

The `PX4 TECS` preset uses that same plot workspace for altitude (target,
rate-limited internal setpoint, actual), airspeed (target and actual), specific
total/balance energy error, and the pitch/throttle outputs. All other
`autopilot/tecs/*` diagnostics remain searchable through Add Plot and flow
through the normal in-memory telemetry snapshot path. The external version-1
MCAP contract remains scoped to the existing Roll Hold experiment.
Altitude plots use metres AGL, airspeed plots use metres per second CAS, and
angle/rate plots consume the raw radian SI channels without display-side data
conversion.

Custom plots use the same `MonitorPlotState` and rendering path as preset
plots. Their deterministic slot assignments, signal selections, Y-axis mode,
and legend preference live in `MonitorState`; only modal-open state, search,
and edit buffers live in `MonitorPlotDialogModel`. Changing to a smaller
layout preserves custom plots in inactive slots, and resetting the preset
workspace removes them with the rest of the workspace state.

Each `TelemetrySnapshot` carries channel metadata resolved by the telemetry
layer. The Monitor signal catalog reuses its display name, symbol, and unit;
only category/subgroup derivation and preset/template association remain
Monitor-specific. Preset-only channels use the same telemetry resolver, so
the Add Plot dialog does not maintain a second raw-signal metadata table.

Telemetry and dynamic-mode snapshots are authoritative read-only inputs. They
are not copied into `MonitorState`. Dynamic-mode identification and eigenmode
calculation remain outside the GUI; Monitor only renders the supplied history.

The model, controller, catalogs, and dialog model are built as the headless
`jsb_monitor` target so timeline, layout, preset, filtering, and Add/Edit state
can be tested without ImGui, `SimRuntime`, or `TelemetryRegistry`.
