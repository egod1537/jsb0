# Thread Ownership Contract

This document defines the ownership contract separating desktop GUI work from
simulation execution. `Application` runs GUI work on the main thread and owns
a `SimWorker` backed by `std::jthread`; the worker exclusively drains simulation
commands and operates the Runtime object graph.

## Ownership

| Owner | Objects and responsibilities |
| --- | --- |
| Main / GUI thread | GLFW event handling and windows, ImGui, ImPlot, OpenGL context and backends, `GUI`, GUI controllers and views, `SimMessageClient`, and all GUI-side caches |
| Simulation worker | `GuiSimBridge`, `SimRuntime`, `Simulation`, aircraft and JSBSim access, flight controllers and autopilots, scenario execution, simulation telemetry production, and recording coordination |
| Shared boundary | Immutable value DTOs and snapshots, request and result identifiers, thread-safe command and event queues, and stop/join synchronization primitives |

The headless runner has no GUI thread. Its runner thread is the sole simulation
owner, so it may drive `SimRuntime` directly.

## Cross-thread boundary

```text
Main / GUI thread
  GUI + SimMessageClient + GUI-side caches
                    |
                    v
          GUI -> Sim command queue
                    |
                    v
Simulation worker
  GuiSimBridge -> SimRuntime -> Simulation/controllers/scenario
          |                         |
          v                         v
 reliable event/snapshot queue   bounded telemetry-batch queue
          |                         |
          +------------+------------+
                       v
Main / GUI thread
```

The queues are the only cross-thread application boundary:

- The GUI thread must not call `GuiSimBridge`, `SimRuntime`, `Simulation`, or a
  simulation controller directly.
- The simulation worker must not call GUI, GLFW, ImGui, ImPlot, OpenGL, or a GUI
  controller directly.
- A publisher must not execute a callback owned by the other thread. Each
  consumer drains its inbound queue and dispatches work on its own thread.
- Mutable references, mutable pointers, and mutable shared object graphs must
  not cross the boundary. Messages transfer values or immutable snapshots.
- A mutex inside an owned object does not make that object shared. In
  particular, `SimMessageClient` and its cache remain GUI-thread-owned.
- Queue payloads must not contain borrowed storage whose lifetime or mutation
  is controlled by the producing thread. Large immutable data may use explicit
  immutable ownership such as `std::shared_ptr<const T>`.

`MessageBus::Publish()` invokes subscribers synchronously on the publishing
thread. It is therefore used only as the consumer-side dispatch primitive:
`GuiToSimQueue::Drain()` dispatches commands to `GuiSimBridge`, and
`SimToGuiQueue::Drain()` dispatches events to `SimMessageClient`, and
`SimToGuiTelemetryQueue::Drain()` dispatches monitor telemetry batches to the
same GUI-owned client. Producers only enqueue typed values and never invoke
callbacks. Code must not publish across threads as a substitute for these
queues.

All commands and operation results/errors use reliable FIFO delivery. Pending
high-frequency state/status/snapshot messages may be coalesced to the latest
value of each type. `GuiSimBridge` captures one `SimSnapshot` value per logical
publication, and the snapshot contains no Runtime, controller, Aircraft, or
other mutable borrowed object. `SimMessageClient` replaces its latest snapshot
cache only while the GUI thread drains events, and rendering reads that cache.

Live Monitor telemetry uses `TelemetryBatch` values on a separately bounded
queue. A batch keeps Primary before Baseline when both sources are present and
preserves each source's version and frame ordering. On overflow the oldest GUI
batch is discarded so the newest monitor data remains available; this policy
does not apply to lifecycle results, errors, scenario/trim results, or other
reliable events. The simulation thread feeds `TelemetryRecordingService`
directly before GUI publication, so MCAP recording is lossless and cannot be
throttled or dropped by a slow GUI.

`SimWorker` drains commands before autonomous ticks and processes commands while
stepping is paused. It dispatches commands one at a time so a paused single-step
is executed before a later FIFO command such as Resume. `Application` drains
both GUI inbound queues at the beginning of each GUI frame, before rendering.

## Simulation scheduling

`SimWorker` owns all steady-clock scheduling state. Normal execution uses the
configured automatic simulation frequency. A late worker advances the deadline
by one interval per deterministic step and catches up without changing the
simulation dt. Paused execution performs no autonomous steps; each queued step
request performs exactly one step. Maximum-speed execution advances as quickly
as possible while checking commands and the stop token before every step.

`SimRuntime::Tick()` remains one deterministic domain step. It contains no
wall-clock sleeps, deadlines, or catch-up policy.

## Desktop event loop

`Application` owns only the main-thread composition loop. Each iteration calls
`GUI::PollPlatformEvents()`, drains the event/snapshot and telemetry queues,
checks the worker failure state, and calls `GUI::Tick()`. There is no GUI
cadence helper, simulation deadline, simulation tick, or scheduler state in
`Application`. Buffer swapping/vsync belongs to GUI policy and is independent
of configured simulation Hz.

Startup proceeds in this order:

1. Validate the GUI, worker, and messaging client dependencies.
2. Bind `SimMessageClient` to the GUI.
3. Start `SimWorker`; it initializes Runtime on the simulation thread.
4. Drain the initial snapshot/events on the GUI thread.
5. Initialize platform and GUI resources.

A fatal worker error is enqueued as a reliable `SimWorkerFatalEvent` before the
worker failure flag is published. `Application` performs a final drain before
reporting failure, so the GUI-side error cache observes the same failure. A
window-close event exits the loop and enters the normal stop/join sequence.
The detailed pending-command, exception, locking, and stress-test policies are
defined in [`CONCURRENCY_VALIDATION.md`](CONCURRENCY_VALIDATION.md).

## Lifetime and shutdown

`Application` owns the complete desktop lifetime: GUI resources, messaging
endpoints, queue endpoints, stop state, and the simulation worker. Simulation
objects form a worker-owned object graph even though `Application` controls its
lifetime.

The required shutdown order is:

1. Stop accepting new GUI commands and request simulation shutdown.
2. Signal the worker through the command/stop boundary.
3. Let the worker finish or cancel simulation work, run simulation shutdown,
   and return without retaining access to the simulation object graph.
4. Join the worker.
5. Only after the join, destroy `GuiSimBridge`, `SimRuntime`, `Simulation`,
   controllers, scenario state, queue storage, and messaging endpoints.
6. Destroy GUI and platform resources on the main / GUI thread.

No simulation-owned object may outlive the worker, and `SimRuntime` destruction
must never race with worker execution. Initialization and shutdown operations
for GUI-owned objects remain on the GUI thread; initialization and shutdown
operations for the simulation object graph occur on the simulation worker.
Object destruction follows the join and is therefore outside concurrent worker
execution.

## Enforcement

`scripts/check_architecture.py` provides a static ownership guard. GUI-owned
source cannot include `GuiSimBridge`, concrete runtime/core simulation types,
or simulation implementation layers. `SimMessageClient` cannot include the
simulation-side bridge or runtime. Conversely, `GuiSimBridge` cannot include
`SimMessageClient` or GUI implementation headers. The composition root remains
the intentional place that knows both sides so it can own and connect their
lifetimes.

Debug affinity checks record the `Application` GUI owner and `SimWorker`
simulation owner. GUI lifecycle/frame entry points assert the GUI thread, while
worker scheduling, command consumption, and tick entry points assert the
simulation thread.
