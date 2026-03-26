# 01. Architecture Rules

This document contains the non-negotiable architectural laws.
Rationale and examples belong in `docs/references/`.

## Layering Rules

- `common/` is portable and must not import concrete `embassy-rp` peripheral types.
- `device/` owns peripheral construction, pins, interrupts, DMA, executors, clocks, and board resource ownership.
- Simulation and HIL backends must preserve the same contracts consumed by `common/`.
- Board-specific constants must enter portable logic only through explicit configuration or typed messages.
- Protocol code owns bytes, framing, parsing, and checksums; it does not own transport hardware.
- Policies, estimators, and controllers do not touch hardware directly unless there is an explicit, reviewed exception.

## Execution-Class Rules

- ISR code stays minimal: latch, timestamp, acknowledge, and notify.
- Tier 0 code owns edge I/O and remains bounded.
- Tier 1 code is pure transform logic and must not touch hardware.
- Tier 2 code owns stateful domain and service behavior and should consume typed data or explicit interfaces.
- Mixed hardware access plus complex domain logic must be split unless there is a documented timing reason not to.

## Scheduling Rules

- Tasks are scheduling boundaries, not architecture layers.
- A new task requires a real need for timing isolation, latency isolation, ownership isolation, cadence separation, or fan-out decoupling.
- The fast estimator/control path must not be fragmented into extra queues and wakeups without strong justification.
- Slow or variable-latency work must not share timing fate with the deterministic control path by default.

## Timing and Core Rules

- No blocking or variable-latency peripheral transaction may sit directly in the critical estimator/control path without written justification.
- Portable estimation, control, and mission logic derive `dt` from typed sample timestamps, not local wall-clock reads.
- Core 0 owns deterministic, flight-critical, and watchdog-feed-critical work on flight-critical targets.
- Core 1 owns slower or more variable-latency work unless a target-specific reason says otherwise.
- Logging that is part of the fast-path observability contract stays on the publisher core; slow sinks must still be buffered and non-blocking.

## Dataflow and Bus Rules

- Typed messages define ownership and cadence boundaries.
- Same-core pub-sub is the default fan-out mechanism.
- Cross-core visibility requires an explicit target-owned bridge in `device/.../channels.rs`.
- Telemetry and logging are subscribers; they must not backpressure the producer.
- Every shared bus has an explicit owner or tightly controlled service domain.
- Portable drivers decode and publish typed data upward; board-level bus coordination remains in `device/`.

## Watchdog Rules

- Only one architectural owner may feed the hardware watchdog.
- Feed permission is based on validated forward progress of required critical functions.
- Core 1 may report liveness, but final feed authority remains under Core 0 for flight-critical targets.
- Non-critical services must not mask critical failures by continuing to feed.
- Reset cause and watchdog evidence must be surfaced at boot and into health/logging paths.
- SITL and HIL must preserve watchdog and reset semantics as architecture, not as an afterthought.

## Change Rule

If a change adds or alters a task boundary, core boundary, shared-bus owner, message contract, interface seam, watchdog contract, or simulation parity rule, update the canonical docs in this set as part of the change.
