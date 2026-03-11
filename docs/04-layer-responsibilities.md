# 04. Layer Responsibilities

## Overview

The system is layered so that:

- hardware-specific concerns remain at the platform edge
- reusable logic remains in `common/`
- system behavior is built on typed messages and services
- policies remain separate from hardware access
- SITL can swap the backend while preserving system logic

---

## 4.1 `common/interfaces/`

This is the most important seam in the architecture.

It defines the abstract capabilities required by the system, such as:

- sensor interfaces
- comms transport interfaces
- storage/log sink interfaces
- actuator output interfaces
- timing interfaces
- health and liveness reporting interfaces
- system-level abstractions

This layer should define **what the system needs**, not how a board provides it.

Examples of conceptual interfaces:

- `Imu`
- `Barometer`
- `Magnetometer`
- `Gps`
- `TelemetryTransport`
- `RadioLink`
- `LogSink`
- `ActuatorBus`
- `StatusLed`
- `MonotonicClock`
- `WatchdogFeeder` or equivalent platform-facing abstraction where appropriate

These interfaces enable:

- embedded target implementations
- simulation implementations
- fake/replay/testing implementations

This layer should be stable, minimal, and explicit.

---

## 4.2 `common/messages/`

This layer defines typed data exchanged between modules and tasks.

Message categories may include:

- raw sensor samples
- converted sensor samples
- estimator state
- control outputs
- telemetry records
- log records
- health reports
- fault reports
- watchdog status
- reset cause information

These types should be:

- explicit
- bounded
- cheap to move or copy
- suitable for no-heap operation
- versionable when necessary

This layer prevents every subsystem from inventing ad hoc payloads.

---

## 4.3 `common/drivers/`

Portable drivers and adapters belong here.

These are drivers written against abstract interfaces or generic traits, such as:

- generic SPI
- generic I2C
- generic UART/serial
- generic read/write traits

Examples:

- BMI088 driver over generic SPI
- BMP581 driver over generic I2C
- SX1262 driver over generic SPI
- UBX transport adapter over generic serial

This layer should not own:

- RP-specific peripheral construction
- RP-specific interrupts
- RP-specific DMA configuration
- board pin ownership

Portable drivers may perform:

- register transactions via abstract buses
- decoding of raw register bytes
- typed sample construction
- validation and device-state management

---

## 4.4 `common/protocol/`

This layer handles packet formats and parser/serializer logic.

Examples:

- MAVLink framing/parsing
- UBX parsing
- CRC logic
- custom framing
- packet types
- log record framing

This layer should know how bytes become structured packets and back.

It should not know:

- which UART is used
- which task owns transport
- which board is active
- which core a parser runs on

---

## 4.5 `common/comms/`

This layer is above protocol.

It owns link/session behavior such as:

- routing
- retries
- bridging
- heartbeats
- session state
- link-state tracking
- connection-level behavior
- FC/Radio/GS/SBC bridging logic

Mental model:

- `protocol/` = byte format
- `comms/` = link behavior

---

## 4.6 `common/policies/`

This layer holds decision rules and state machines.

Examples:

- arming rules
- mode transitions
- failsafe decisions
- fault escalation rules
- mission logic
- reset policy
- watchdog escalation policy

Policies should consume clean inputs and produce decisions.

Policies should not directly touch hardware.

---

## 4.7 `common/localization/`

This is the estimator domain.

Examples:

- attitude estimation
- navigation estimation
- calibration
- sensor fusion
- state definition
- bias estimation
- propagation/update logic

It consumes time-stamped measurements and produces estimates.

This layer should remain portable so it can run in:

- embedded targets
- SITL
- replay tools
- offline analysis tools

---

## 4.8 `common/control/`

This layer contains control-specific logic.

Examples:

- rate control
- attitude control
- guidance
- mixing
- output shaping
- setpoint conditioning

Inputs:

- estimated state
- policy/mode decisions
- setpoints or mission data

Outputs:

- abstract control commands
- actuator setpoints
- output intents

This layer must remain hardware-independent.

---

## 4.9 `common/services/`

Services orchestrate reusable workflows and stateful pipelines.

Examples:

- acquisition pipeline
- estimator service
- control service
- telemetry packaging
- logging service
- health aggregation
- liveness tracking
- watchdog supervision
- status reporting

A service is bigger than a driver and smaller than platform assembly.

This is where reusable operational workflows should live.

---

## 4.10 `common/tasks/`

This directory may hold task bodies that remain HAL-agnostic.

These tasks should operate over:

- interfaces
- messages
- services
- channels
- signals
- clocks

Task bodies may be portable.

Task spawning and placement are not portable and belong in:

- `device/`
- `sim/`

---

## 4.11 `common/utilities/`

Shared reusable helpers belong here.

Examples:

- math utilities
- filters
- unit conversions
- time helpers
- bounded buffers
- identifiers

This folder should remain disciplined and not become a dumping ground.

---

## 4.12 `device/<target>/`

Each target directory is the embedded platform assembly layer.

It owns:

- Embassy RP initialization
- peripheral construction
- DMA ownership
- interrupt bindings
- pin map
- bus construction
- resource ownership
- executor startup
- core placement
- watchdog hardware ownership
- startup sequence

### File responsibilities

#### `resources.rs`
Owns concrete hardware resources and wrappers.

#### `pinmap.rs`
Board pin assignments and aliases.

#### `interrupts.rs`
Interrupt binding definitions.

#### `config.rs`
Target-specific runtime configuration.

#### `clocks.rs`
Clock/timebase configuration.

#### `buses.rs`
Construction and ownership of shared buses and related wrappers.

#### `channels.rs`
Inter-task communication topology.

#### `watchdog.rs`
Concrete watchdog peripheral ownership, feed API, reset cause access, and related low-level behavior.

#### `core0.rs` / `core1.rs`
Per-core executor composition and task placement.

#### `main.rs`
Assembly root that wires the whole target together.

---

## 4.13 `sim/shared/`

Common simulation resources and services, such as:

- time control
- world state access
- channel topology
- shared transports
- scenario support

---

## 4.14 `sim/backends/`

Virtual implementations of interfaces used by `common/`.

Examples:

- virtual IMU
- virtual barometer
- virtual magnetometer
- virtual GPS
- actuator feedback
- loopback UART/radio
- simulated log sinks
- deterministic time scheduler
- simulated watchdog behavior

These backends should preserve architectural contracts even if their internals differ from embedded hardware.

---

## 4.15 `sim/<target>/`

Each SITL target mirrors the corresponding embedded target shape.

These per-target folders should own:

- scenario selection
- target config
- core partitioning model
- main assembly
- simulation runtime composition

This symmetry is intentional and important.

---

## Final position

A module should live in the layer that matches its responsibility, not the layer that feels convenient during implementation.

That discipline is what preserves architecture over time.