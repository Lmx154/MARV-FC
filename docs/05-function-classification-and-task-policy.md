# 05. Function Classification and Task Policy

## Purpose

The firmware should not classify code only by folder or module.  
It should also classify code by:

- side effects
- timing behavior
- state ownership
- scheduling implications

This is especially important for a latency-sensitive embedded system.

The goal is to keep:

- hardware edge behavior narrow
- pure logic testable
- orchestration explicit
- task count deliberate

---

## Execution classes

The architecture should use four conceptual execution classes:

1. ISR class
2. Tier 0 — Edge / I/O functions
3. Tier 1 — Pure transform functions
4. Tier 2 — Stateful domain/service functions

Tasks are **not** one of the tiers.  
A task is a scheduling container that may execute Tier 0, Tier 1, and Tier 2 code depending on what it owns.

---

## ISR class

Interrupt handlers are their own execution class.

They should be minimal and highly constrained.

ISR responsibilities may include:

- latching status
- timestamping
- recording minimal event flags
- acknowledging interrupts
- noting DMA completion
- waking a task or signaling work availability
- capturing pre-timeout watchdog indication if supported

ISR responsibilities should **not** include:

- estimator math
- control law evaluation
- policy state transitions
- heavy parsing
- complex logging
- broad system behavior

The ISR class exists to keep interrupt behavior bounded and predictable.

---

## Tier 0 — Edge / I/O functions

These functions directly interact with hardware or hardware-like endpoints.

Examples:

- sensor reads
- GPIO toggles
- UART byte movement
- SPI transactions
- I2C transactions
- DMA-backed hardware submission/completion handling
- PWM output writes
- watchdog feed calls
- reset cause reads

Characteristics:

- side-effectful
- timing-sensitive
- bounded
- narrow in scope
- platform edge oriented

Examples of conceptual functions:

- `read_bmi088_accel_raw()`
- `poll_gps_uart()`
- `set_servo_pwm()`
- `watchdog_feed()`
- `read_reset_reason()`

Tier 0 functions should not contain:

- high-level policy
- estimator logic
- broad orchestration
- mixed-domain decision making

They should be small and boring.

---

## Tier 1 — Pure transform functions

These functions take values in and return values out.

They do not:

- touch hardware
- block on I/O
- mutate random platform state
- perform peripheral construction
- depend on a specific board

Examples:

- raw-to-SI conversion
- calibration transforms
- filtering steps
- coordinate transforms
- quaternion normalization
- CRC logic
- packet parsing from available bytes
- deadline arithmetic
- freshness evaluation helpers

Examples of conceptual functions:

- `accel_counts_to_mps2(raw)`
- `gyro_counts_to_rad_s(raw)`
- `apply_mag_calibration(sample, cal)`
- `parse_ubx_payload(bytes)`
- `is_fresh(last_seen, now, max_age)`

Tier 1 functions are the most portable and most testable class in the system.

---

## Tier 2 — Stateful domain/service functions

These functions or modules own state and orchestrate workflows.

They may:

- consume typed inputs
- update service state
- call Tier 1 helpers
- emit messages
- make mode/fault/watchdog decisions
- coordinate pipeline progress

Examples:

- estimator step
- control update
- telemetry packaging step
- policy update step
- health aggregation
- watchdog supervision logic

Examples of conceptual functions:

- `ekf_step(state, imu_sample, gps_sample, dt)`
- `control_step(estimate, setpoint, mode)`
- `arming_policy_step(inputs)`
- `watchdog_supervisor_step(statuses, now)`

Tier 2 logic should usually consume typed data rather than directly reaching into hardware.

---

## Important correction to complexity-based thinking

The architecture should **not** say:

> high-tier functions are allowed to directly do hardware access and complex logic together

That leads to bad boundaries.

Instead, the architecture should say:

> complex domain logic may consume data that originated from hardware, but it should not directly poke hardware unless there is a deliberate and justified reason

This separation protects:

- testability
- timing analysis
- SITL portability
- code review clarity
- fault containment

---

## Function classification rules

### Rule 1
Edge functions may touch hardware but must stay narrow and bounded.

### Rule 2
Pure transform functions may not touch hardware or random global mutable device state.

### Rule 3
Stateful domain/service functions should consume typed data and explicit interfaces rather than directly manipulating hardware wherever possible.

### Rule 4
Complex logic that mixes hardware access with domain decisions should be split unless there is a strong timing justification.

### Rule 5
ISRs must remain minimal and should usually hand work to tasks or service logic instead of doing the work themselves.

---

## Tasks are scheduling boundaries

A task is not merely an organizational unit.  
It is a scheduling boundary and therefore a timing decision.

Each new task may introduce:

- wakeup latency
- queue latency
- synchronization overhead
- ordering complexity
- stale-data hazards
- more difficult timing analysis

For that reason, task creation must be deliberate.

---

## Task creation policy

A task should exist only when there is a real need for:

- independent scheduling
- latency isolation
- ownership isolation
- criticality separation
- cadence separation
- fan-out decoupling

A task should **not** be created simply because:

- a module exists
- a sensor exists
- a folder exists
- async makes it easy
- a function is long
- a service “feels separate”

---

## Valid reasons to create a task

### 1. Different timing rate
Examples:

- IMU acquisition at a high rate
- GPS parsing at a low rate
- barometer at medium rate
- telemetry output at slower or bursty rate

### 2. Different latency behavior
Examples:

- SD card logging
- radio bridging
- UART parsing
- telemetry transmission

### 3. Ownership isolation
Examples:

- one task owns a shared SPI bus
- one task owns a parser state machine
- one task owns a watchdog supervisor

### 4. Criticality separation
Examples:

- estimator/control path is critical
- LED/status/debug work is non-critical

### 5. Fan-out decoupling
Examples:

- one producer feeds multiple consumers through bounded channels

---

## When not to create a task

A function or module may remain within an existing deterministic task when:

- it always runs at the same cadence as the caller
- it is compute-only
- it is bounded
- it does not wait on I/O
- making it a separate task would only add queue and scheduling overhead

---

## Fast-path policy

The fast estimator/control path should not be over-fragmented into many small tasks unless there is a very strong reason.

A preferred pattern is often:

```text
fast acquisition step
    -> estimator step
    -> control step
    -> actuator output step
````

within one tightly budgeted timing island

rather than:

```text
imu task
    -> estimator task
    -> control task
    -> actuator task
```

with multiple queues and wakeups in between.

The more boundaries in the critical path, the more opportunities for:

* jitter
* stale data
* sequencing errors
* debugging pain

---

## Shared bus warning

Shared buses are a common source of accidental timing degradation.

Bad pattern:

* many tasks casually lock the same shared SPI or I2C bus

Better pattern:

* one owner task or tightly controlled timing domain owns the bus
* it performs reads on a known schedule
* it publishes typed samples downstream

This makes timing far easier to reason about.

---

## Recommended task classes

### Fast deterministic loop

Examples:

* IMU acquisition
* estimator update
* control update
* actuator output
* deadline-critical safety accounting

### Medium-rate service loop

Examples:

* magnetometer
* barometer
* health/status aggregation
* command ingestion

### Slow/background loop

Examples:

* GPS parsing
* telemetry framing
* SD logging
* radio or SBC bridge
* diagnostics
* LED patterns

---

## Relationship to architecture folders

### `device/`

Mostly where Tier 0 ownership is assembled and where scheduling is concretely assigned.

### `common/drivers/`

Often mixes portable Tier 0 bus-driven behavior with Tier 1 decoding.

### `common/localization/`, `common/control/`, `common/protocol/`

Mostly Tier 1 and Tier 2.

### `common/services/`

Mostly Tier 2.

### `common/tasks/`

Portable task bodies, but not task placement.

---

## Final position

The architecture should classify code by:

* ISR behavior
* hardware edge behavior
* pure transform behavior
* stateful service/domain behavior

and should treat tasks as scheduling and ownership decisions, not as a default structuring mechanism.

This is the right approach for protecting a latency-sensitive firmware architecture.