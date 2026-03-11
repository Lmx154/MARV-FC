# 07. Communication, Bus, and Dataflow

## Purpose

The communication architecture should make:

- ownership explicit
- dataflow visible
- bus contention understandable
- message boundaries intentional
- replay/SITL parity easier to preserve

This applies to both inter-task communication and physical bus ownership.

---

## Dataflow principle

Dataflow should be explicit and typed.

Recommended primitives conceptually include:

- bounded channels for streams
- signals or watch-style latest-value mechanisms for snapshots
- request/response only where truly needed

The architecture should avoid hidden shared mutable state as the default design pattern.

---

## Preferred dataflow shape

```text
sensor task -> sensor message -> estimator service -> state message
state message -> control service -> actuator command
state/control/fault -> telemetry service
state/sensor/fault -> logging service
policy inputs -> policy state -> mode/arming/failsafe decisions
health/liveness -> watchdog supervisor -> feed or withhold feed
````

Benefits:

* easier tracing of ownership
* easier SITL parity
* easier replay
* easier latency reasoning
* easier debugging

---

## Messages should represent meaningful boundaries

A message boundary should exist when it reflects:

* a real producer/consumer relationship
* cadence separation
* ownership separation
* fan-out need
* fault containment need

A message boundary should not exist solely because a subsystem sounds architecturally clean on paper.

---

## Transport layering

The communication stack should be thought of in layers:

### Protocol layer

Responsible for:

* framing
* parsing
* serialization
* CRC/checksum
* packet type definitions

### Comms/session layer

Responsible for:

* retry behavior
* routing
* bridging
* link state
* session state
* heartbeats
* endpoint coordination

This separation should remain strict.

---

## Shared bus ownership principle

Shared buses must have explicit ownership.

The architecture should not allow many unrelated tasks to casually contend for the same bus without a clearly designed ownership model.

This applies to:

* SPI
* I2C
* UART ownership where parser and producer boundaries matter
* shared transport links

---

## SPI guidance

For MARV-FC-RL-RP2354B, the architecture should recognize at least the following SPI domains:

### SPI1 shared sensor cluster

* BMI088 accel
* BMI088 gyro
* LSM6DSV32X

This is likely a fast sensing domain and should be treated accordingly.

### SPI0 storage domain

* microSD

This should be isolated from the flight-critical timing path as much as practical.

---

## I2C guidance

### I2C0 environmental domain

* BMP581

### I2C1 auxiliary navigation domain

* BMM350
* u-blox NEO M9N GPS-related interface if applicable to this bus plan

Shared I2C coordination belongs in `device/`.

Portable drivers in `common/` should not own board-level bus coordination policy.

---

## UART guidance

### UART0 link domain

* FC ↔ Radio

### UART1 companion domain

* FC ↔ SBC

These are not just devices. They are communication domains with different latency and criticality characteristics.

UART parsing, buffering, bridging, and routing should respect those differences.

---

## Bus policy guidance

A bus with multiple devices should usually have one of these ownership models:

### Model A — single owner task

One task owns the bus and performs all related transactions on a known schedule.

### Model B — tightly coordinated service domain

A tightly controlled service owns bus access, often within a timing island.

### Model C — carefully wrapped shared access

Only when justified, with explicit arbitration and timing understanding.

The default should not be “everyone lock the bus when needed.”

---

## Why bus ownership matters

Poor bus ownership causes:

* timing unpredictability
* priority inversion
* awkward lock behavior
* stale sensor data
* hard-to-debug jitter
* fragile scaling as the system grows

Good bus ownership makes the system more analyzable.

---

## Telemetry and logging subscriptions

Telemetry and logging should generally be subscribers, not owners of the critical path.

Good pattern:

* fast path produces estimates/control/fault outputs
* telemetry consumes copies or snapshots
* logger consumes copies or snapshots

Bad pattern:

* fast path waits for telemetry/logging serialization or sink behavior

---

## Data freshness and bus cadence

Bus acquisition cadence should line up with system needs, not just device maximum rates.

Questions to answer per bus domain:

* what data rate is actually needed?
* what devices share this bus?
* which device is timing-critical?
* who owns the polling cadence?
* does this bus feed the watchdog-critical path?
* what is the consequence of temporary bus delay?

---

## Communication topology for watchdog relevance

Not all communication failures are watchdog failures.

Suggested categories:

### Feed-critical

* communication or dataflow required for estimator/control forward progress

### Degrade-critical

* communication loss causes degraded mode, but not immediate watchdog denial

### Non-critical

* communication failure is logged/reported, but not watchdog-relevant

This prevents non-essential communication issues from causing unnecessary resets.

---

## Final position

The communication and bus architecture should prioritize:

* explicit ownership
* typed message flow
* clear protocol/session separation
* strong fast-path protection
* minimal accidental contention
* careful use of channels and subscriptions

These rules are essential for both embedded determinism and SITL parity.
