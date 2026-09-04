# In-Process Simulation Message Bus

The interactive application exchanges commands and state through a small,
type-safe C++ message bus. Message types, rather than string topic names,
identify channels.

```text
GUI
  ↓
SimMessageClient
  ↓
GuiToSimQueue
  ↓ consumer-side MessageBus dispatch
GuiSimBridge
  ↓
SimRuntime

SimRuntime
  ↓
GuiSimBridge
  ├─→ SimToGuiQueue (reliable events + latest snapshot)
  └─→ SimToGuiTelemetryQueue (bounded TelemetryBatch)
            ↓ consumer-side MessageBus dispatch
SimMessageClient
  ↓
GUI cache
```

`GuiSimBridge` does not depend on a concrete GUI implementation. It bridges
GUI-side typed messaging and `SimRuntime`; the GUI reaches the runtime only
through `SimMessageClient` and `GuiToSimQueue`.

`MessageBus::Publish` dispatches synchronously on the publishing thread. The
bus does not own threads, queues, clocks, or tick timing. The simulation-side
bus is dispatched by `SimWorker`; the GUI-side bus is dispatched by the main
thread. It must not be used to invoke a subscriber owned by another thread.

`GuiToSimQueue`, `SimToGuiQueue`, and `SimToGuiTelemetryQueue` use
mutex-protected storage. Each side consumes its inbound queues and performs
local bus dispatch on its owning thread. Making `MessageBus` independently
thread-safe would not remove this ownership requirement. See
[`THREADING.md`](THREADING.md) for the complete contract.

`Subscription` is move-only and automatically unsubscribes when destroyed.
Publishing takes a callback snapshot before invocation. A callback may destroy
its own subscription or another subscription safely; a removed callback that
has not yet run is skipped during the current publication.

Commands and events are ordinary C++ structs in
`messaging/SimMessages.hpp`. Request/response operations
carry request IDs. Continuous state is delivered as owned snapshot/status
values and telemetry batches. `SimMessageClient` turns those messages into
GUI-thread-local caches for rendering; it never reads Runtime-owned mutable
state.

Commands and operation results/errors are reliable FIFO messages. Repeated
state, scenario-status, recording-status, and simulation-snapshot events are
latest-only while pending. The live Monitor telemetry queue is bounded and
drops its oldest pending batch on overflow. Primary/Baseline frames from one
logical update remain together and ordered within a batch. Recorder/MCAP
telemetry does not traverse this queue and remains lossless on the simulation
thread. No coalescing or bounded policy applies to results or errors.
Worker fatal errors use the same reliable event queue and are drained before
`Application` returns failure.

Request methods on `SimMessageClient` return whether the command queue accepted
the request; they do not wait for Runtime. Optional completions are correlated
by request ID and run only when the GUI thread drains the matching result event.

The headless runner does not require the bus and continues to drive
`SimRuntime` directly.
