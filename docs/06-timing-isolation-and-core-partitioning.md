# 06. Timing Isolation and Core Partitioning

## Purpose

Timing isolation is one of the primary architectural constraints of MARV.

The architecture should be shaped around the fact that some work is:

- deterministic and critical
- moderately time-sensitive
- slow and variable-latency
- non-critical

The system should make these distinctions visible in both code structure and runtime placement.

---

## Timing classes

### Fast deterministic loop

This is the most timing-sensitive part of the firmware.

Examples:

- IMU acquisition
- estimator propagation/update
- control law execution
- actuator output update
- fast liveness/deadline accounting
- watchdog-critical health checks

This path must have:

- minimal jitter
- bounded execution
- minimal blocking
- minimal scheduling fragmentation

---

### Medium-rate service loop

This class supports the fast loop but usually tolerates more latency.

Examples:

- magnetometer acquisition
- barometer acquisition
- health aggregation
- command processing
- status updates

These tasks must still be well-behaved, but their jitter tolerance is higher than the fast loop.

---

### Slow/background loop

This class is the most likely to contain variable-latency work.

Examples:

- GPS parsing
- telemetry framing and transmission
- SD card logging
- radio bridge work
- SBC bridge work
- indicators and diagnostics
- debugging shell

These must be isolated so they do not casually affect flight-critical timing.

---

## Timing rule

> No blocking or variable-latency peripheral transaction may directly sit inside the critical estimator/control path unless intentionally justified.

This is one of the most important rules in the architecture.

That means the preferred pattern is:

- sensor task publishes typed samples
- estimator consumes typed samples
- control consumes estimates
- logging and telemetry subscribe to results
- slow I/O stays away from the deterministic loop

---

## Core partitioning guidance

A recommended direction is:

### Core 0
Owns deterministic and flight-critical work.

Examples:

- fast acquisition
- estimator updates
- control updates
- actuator outputs
- critical liveness tracking
- watchdog supervisor
- final watchdog feed authority

### Core 1
Owns variable-latency and less critical work.

Examples:

- telemetry
- logging
- GPS
- bridge services
- diagnostics
- indicators
- optional non-critical processing

This split is not absolute law, but it is a strong architectural guard rail.

---

## Why Core 0 should own watchdog authority

Core 0 should own the final watchdog decision because:

- it has the most authoritative view of flight-critical progress
- if Core 0 stalls, the watchdog should naturally stop being fed
- Core 1 should not be able to mask Core 0 failure
- critical truth should live with the critical core

Core 1 may publish liveness status, but should not directly feed the watchdog.

---

## Timing islands

The firmware should be thought of in terms of timing islands.

A timing island is a set of operations that are intentionally grouped because separating them would introduce more timing risk than benefit.

A typical fast timing island may be:

```text
IMU read
    -> sample conversion
    -> estimator step
    -> control step
    -> actuator command publish/write
````

This is often preferable to fragmenting the whole chain across multiple unrelated tasks.

---

## Deadline thinking

The architecture should make deadlines visible conceptually even before implementation details are finalized.

For each critical execution path, the team should be able to answer:

* what triggers this work?
* how often must it run?
* what is the maximum acceptable age of its inputs?
* what is the maximum acceptable completion time?
* what happens if it misses deadline?
* does deadline failure affect watchdog feed eligibility?

This is a design concern, not just a runtime optimization concern.

---

## Data freshness matters as much as execution frequency

A task waking up “on time” is not sufficient if it operates on stale data.

The architecture should therefore separate:

* task cadence
* input freshness
* output freshness
* end-to-end loop age

This is especially important for:

* estimator inputs
* control outputs
* watchdog liveness decisions

---

## Guidance for fast-path decomposition

The fast path should be split only when there is strong justification.

Good reasons to split include:

* bus ownership necessity
* unavoidable isolation requirement
* measurable benefit to determinism
* architectural need for strict producer/consumer separation

Bad reasons to split include:

* aesthetic modularity
* convenience
* “everything should be a task”
* generic async enthusiasm

---

## Jitter sources to suspect early

The architecture should assume the following are dangerous until proven otherwise:

* shared bus contention
* SD card writes
* radio transmission paths
* UART parsing bursts
* dynamic retry behavior
* multi-queue critical-path chains
* slow logging sinks
* debug output
* large telemetry serialization

These should be isolated away from the critical timing path.

---

## Timing-aware channel usage

Channels are useful, but they are not free.

Each channel boundary may add:

* queueing delay
* backpressure
* stale message risk
* more complex recovery semantics

Use channels where they provide:

* decoupling
* ownership clarity
* acceptable timing behavior

Avoid them where they only add delay inside a critical path.

---

## Recommended timing posture for MARV

### Core 0

* fast sensor domain
* estimation
* control
* actuator path
* critical health state
* watchdog supervision

### Core 1

* telemetry
* logging
* GPS
* radio/SBC bridge
* diagnostics
* indicators
* optional slow services

### Medium-rate tasks

May land on either core depending on target constraints, but should not be allowed to erode Core 0 determinism casually.

---

## Final position

Timing isolation should be treated as an architectural property, not as an optimization to worry about later.

The firmware should be designed so that:

* critical work is structurally protected
* slow work is structurally isolated
* core placement reflects criticality
* watchdog decisions reflect actual forward progress
* fast-loop composition is deliberate
