# In-Process Simulation Message Bus

The interactive application exchanges commands and state through a small,
type-safe C++ message bus. Message types, rather than string topic names,
identify channels.

```text
GUI
  ↓
SimMessageClient
  ↓
MessageBus
  ↓
GuiSimBridge
  ↓
SimRuntime

GUI <- cached events          <- MessageBus <- GuiSimBridge
```

`GuiSimBridge` does not depend on a concrete GUI implementation. It bridges
GUI-side typed messaging and `SimRuntime`; the GUI reaches the runtime
only through `SimMessageClient` and `MessageBus`.

`MessageBus::Publish` dispatches synchronously on the publishing thread. The
application therefore retains its existing single-threaded GUI and simulation
scheduling; the bus does not own threads, queues, clocks, or tick timing.
Cross-thread calls must be externally serialized.

`Subscription` is move-only and automatically unsubscribes when destroyed.
Publishing takes a callback snapshot before invocation. A callback may destroy
its own subscription or another subscription safely; a removed callback that
has not yet run is skipped during the current publication.

Commands and events are ordinary C++ structs in
`messaging/SimMessages.hpp`. Request/response operations
carry request IDs. Continuous state is delivered as snapshot, status, and
telemetry frame events. `SimMessageClient` turns those events into local
immutable caches for GUI rendering.

The headless runner does not require the bus and continues to drive
`SimRuntime` directly.
