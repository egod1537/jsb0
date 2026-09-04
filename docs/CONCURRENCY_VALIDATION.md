# Shutdown and Concurrency Validation

This document fixes the desktop shutdown policy and records the concurrency
invariants exercised by the stress tests.

## Shutdown contract

Desktop shutdown is ordered as follows:

1. A signal or window-close request ends the GUI loop.
2. `SimWorker::RequestStop()` closes `GuiToSimQueue`, rejecting new commands,
   and requests the worker stop token.
3. A command callback already executing on the simulation thread may finish.
   Commands still waiting in the queue are discarded without dispatch or
   completion callbacks. GUI request completions are irrelevant after desktop
   shutdown has begun.
4. The worker closes and clears its command queue, shuts down Runtime, and
   publishes its final state.
5. `Application` joins the worker and destroys `SimWorker`; consequently
   `GuiSimBridge` and `SimRuntime` are destroyed before GUI/platform teardown.
6. The GUI thread drains final reliable events and retained bounded telemetry,
   then shuts down GUI/platform resources.

Event/result/error messages already published remain reliable and are drained
after join. Monitor telemetry retains at most the configured bounded number of
batches. Recorder telemetry is consumed losslessly on the simulation thread
and is finalized during Runtime shutdown.

## Exception boundary

The `std::jthread` entry point catches standard and unknown exceptions.
Failures become reliable `SimWorkerFatalEvent` values and set the worker's
atomic failure state. Runtime shutdown is protected by its own exception
boundary. No Runtime, bridge, command callback, or telemetry exception is
allowed to escape the worker entry point into `std::terminate`.

## Locking rules

- Queue mutexes protect only queue storage, close state, and telemetry drop
  counters. Drain swaps or removes storage while locked, releases the lock,
  and only then invokes consumer-side dispatch.
- `MessageBus` copies callback entries while holding its state mutex and
  releases the mutex before invoking external callbacks.
- `SimWorker::stateMutex_` protects startup status and error text. It is not
  held while calling Runtime, waiting for a command, publishing an event, or
  invoking callbacks.
- Queue and worker-state locks are never nested, so there is no cross-object
  lock acquisition order.
- `SimRuntime` and telemetry producers are simulation-thread-owned.
  `SimMessageClient` caches are GUI-thread-owned and require no cache mutex.
- Stop-token callbacks only wake queue condition variables; they do not run
  Runtime or GUI work.

Code must never hold a queue, bus, or worker-state lock while invoking an
external callback.

## Validation matrix

Automated tests cover:

- stop while waiting at normal 1 Hz, while paused, and at maximum speed;
- rejection after stop and cancellation of a 10,000-command backlog;
- configured 500 Hz and maximum-speed operation with a saturated bounded
  telemetry queue;
- a 10 FPS GUI while simulation runs independently at maximum speed;
- a deliberately slow simulation-side command while the GUI-side drain stays
  non-blocking;
- repeated pause/step/resume sequences with an exact deterministic step count;
- identical state after the same dt and command sequence across independent
  worker lifetimes;
- Primary/Baseline identity and ordering under telemetry pressure;
- worker startup failure and an exception thrown by a simulation-thread
  subscriber;
- window close, paused stop, maximum-speed stop, repeated startup/shutdown, and
  idempotent desktop teardown.

The sanity thresholds are intentionally broad enough for CI scheduling jitter:
condition-variable stop wakeups must complete within 250 ms, active-command
shutdown and maximum-speed shutdown within 500 ms, queue memory remains bounded
by capacity, and maximum-speed progress must exceed GUI frame progress.

The current Windows MinGW toolchain has no usable ThreadSanitizer runtime.
Concurrent producer/drain stress tests are always run; TSAN should additionally
be enabled without code changes when the project is built on a supported Linux
Clang/GCC environment.
