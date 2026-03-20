# HIL Framework Plan

Status: draft architecture plan

This document defines the target architecture for Hardware-in-the-Loop (`HIL`) as a reusable framework capability in `common/`, not as a one-off implementation for a single board.

The current FC MAVLink-over-USB CDC path is useful as a spike and a source of requirements, but it is not the desired long-term shape. The framework described here is the architecture future device implementations should converge toward.

## 1. Goals

Primary goals:

- make HIL reusable across multiple embedded targets and host-side runtimes
- keep mission, estimation, and control logic unaware of whether inputs came from hardware, HIL, or replay
- make simulation time the authoritative measurement time when HIL is enabled
- keep target crates thin by limiting them to peripheral bring-up, executor wiring, and transport ownership
- allow multiple transports and protocols without redefining the HIL architecture each time

Secondary goals:

- preserve the same typed sample contracts across real, HIL, and replay backends
- support deterministic pause, step, and replay semantics
- keep outbound actuator and mission/event reporting portable

## 2. Non-Goals

This plan does not require:

- a MAVLink-only HIL architecture
- a USB CDC-only device path
- mission logic with explicit HIL branches
- estimator or controller logic that reads local wall-clock time for `dt`
- per-device HIL business logic living in `device/<target>/`

## 3. Architectural Position

HIL should be treated as a reusable framework layer with three boundaries:

1. transport
2. protocol
3. semantic bridge into portable sample and output contracts

The intended flow is:

```text
device transport adapter
  -> common HIL runtime
  -> canonical typed sample channels / control-output channels
  -> estimator / policy / control
```

And in the reverse direction:

```text
portable control / mission outputs
  -> common HIL egress runtime
  -> protocol adapter
  -> device transport adapter
```

## 4. Ownership Boundaries

### 4.1 `common/services/hil/`

This module should become the framework home for HIL.

It should own:

- HIL runtime state and progression rules
- HIL ingress orchestration
- HIL egress orchestration
- HIL semantic message model
- framework-level timeout and liveness rules
- reusable async loops that process inbound and outbound HIL traffic
- translation between HIL semantics and canonical internal message contracts

Suggested shape:

```text
common/src/services/hil/
  mod.rs
  model.rs
  backend.rs
  ingress.rs
  egress.rs
  runtime.rs
  routing.rs
  tasks.rs
```

### 4.2 `common/protocol/`

Protocol modules remain protocol-only.

They should own:

- frame encoding and decoding
- checksums and framing rules
- protocol-specific packet structs
- protocol-specific compatibility quirks

They should not own:

- channel publication
- replay tick semantics
- mission or control assumptions
- transport ownership

### 4.3 `device/<target>/`

Target crates should stay thin.

They should own:

- peripheral construction
- concrete transport setup such as USB CDC, UART, UDP, CAN, or radio links
- executor startup and task spawning
- core placement
- static allocation of target-owned channels and resources
- choosing whether a target boots in real, HIL, or replay-backed acquisition mode

They should not own:

- protocol parsing policy
- HIL tick semantics
- sensor-to-mission translation rules
- control-to-HIL egress rules

## 5. Canonical Contracts

HIL is only reusable if it converges onto the same portable message contracts used by real hardware producers.

Ingress must publish canonical typed samples such as:

- `TimeSample`
- `ImuSampleStamped`
- `BarometerSampleStamped`
- `GpsFixSampleStamped`
- future `MagnetometerSampleStamped`
- future device-specific typed samples where appropriate

Egress must consume canonical portable outputs such as:

- actuator bus commands
- servo intents
- pyro intents
- mission events
- mode/state changes where needed

The seam is at the producer/consumer boundary.

Consumers above that boundary must not know whether a sample originated from:

- SPI/I2C/UART hardware
- HIL transport input
- replay logs

## 6. HIL Semantic Model

`common/services/hil/model.rs` should define protocol-neutral HIL semantics.

Examples:

- `HilTick`
- `HilImuSample`
- `HilBarometerSample`
- `HilGpsSample`
- `HilMagSample`
- `HilActuatorCommand`
- `HilMissionEvent`

Protocol adapters such as MAVLink should translate between wire messages and this semantic model.

That keeps:

- MAVLink as an adapter
- HIL as a framework

not the other way around.

## 7. Time Authority Model

When HIL is enabled, simulation time must become the authoritative measurement time for the portable flight stack.

This means:

- estimator time progression comes from HIL-provided sample timestamps
- controller `dt` comes from HIL-provided sample timestamps
- mission timing decisions come from HIL-provided timestamps or state derived from them
- replay stepping is driven by accepted HIL ticks, not by embedded wall-clock cadence

The required distinction is:

- local wall-clock time remains valid for platform-owned mechanics such as USB polling, queue waits, retry backoff, and watchdog implementation details
- simulation time owns the meaning of measurement progression for acquisition, estimation, control, and mission behavior

### 7.1 Rules for complete HIL time takeover

When HIL is active:

1. real sensor producer tasks that generate authoritative measurement timestamps must be disabled or fully excluded from the active backend
2. portable consumers must derive `dt` only from sample timestamps carried by HIL-backed messages
3. logging that represents flight-state progression should timestamp snapshots from HIL tick time or accepted measurement time, not local uptime
4. no portable estimator/controller/policy code may call a local monotonic clock as its source of truth for flight progression
5. if the simulator pauses, portable flight progression pauses
6. if the simulator steps once, portable flight progression advances exactly once according to the injected timestamps

### 7.2 Accepted uses of local time under HIL

Local embedded time is still allowed for:

- transport read/write timeouts
- host disconnect detection
- watchdog servicing mechanics
- cooperative task delays used to prevent busy looping
- non-authoritative diagnostics

Local embedded time is not allowed to define:

- measurement timestamps
- estimator `dt`
- control `dt`
- mission elapsed time
- replay tick progression

## 8. Runtime Behavior

The HIL runtime should be step-driven, not schedule-driven.

Preferred runtime model:

1. ingest bytes from a target-owned transport
2. decode protocol frames
3. translate frames into HIL semantic messages
4. accept or reject tick progression according to runtime rules
5. publish canonical typed samples with simulation-provided timestamps
6. allow downstream estimator, mission, and control logic to react only to those published messages
7. translate portable outputs back into HIL semantic outputs
8. encode and transmit outbound protocol frames

This preserves the same mental model across:

- embedded HIL
- host SITL
- replay

## 9. Reusable Task Model

Task wrappers are target-specific. Task bodies should be reusable.

Target-owned wrappers should look conceptually like:

```rust
#[embassy_executor::task]
async fn hil_ingress_task(transport: TargetHilTransport) -> ! {
    common::services::hil::tasks::run_hil_ingress_loop(
        transport.reader(),
        protocol_adapter,
        runtime,
        channels,
    ).await
}
```

Portable HIL task bodies should live in `common/services/hil/tasks.rs`.

That keeps:

- executor annotations
- concrete peripheral types
- target ownership

inside `device/<target>/`, while keeping the behavior reusable.

## 10. Backend Selection

Backend selection should be a first-class assembly concern.

Example conceptual model:

```rust
pub enum SensorBackend {
    Real,
    Hil,
    Replay,
}
```

The important property is not the enum itself. The important property is that each backend feeds the same canonical downstream contracts.

No mission, estimator, or controller code should branch on `Hil` versus `Real` for core behavior.

## 11. Migration Guidance From Current Code

The current repository already contains useful pieces:

- shared MAVLink framing and parsing
- a shared MAVLink stream pump
- a narrow MAVLink HIL bridge
- a device-specific USB CDC ingest path

Those pieces should be treated as inputs to the framework design, not as the final architecture.

Recommended migration direction:

1. promote HIL from `common/services/acquisition/hil/` into `common/services/hil/`
2. introduce protocol-neutral HIL semantic types
3. keep MAVLink as the first protocol adapter beneath that framework
4. generalize the current bridge into protocol adapter plus ingress runtime
5. unify downstream consumers onto canonical shared channels rather than separate HIL-only consumer paths
6. add egress mapping from portable outputs into HIL semantic outputs
7. keep target crates limited to transport ownership and spawn wrappers

## 12. Current Time Leaks That Must Be Removed for Full HIL Takeover

The current codebase already supports stamped samples, but it does not yet give HIL complete time authority.

Examples of current local-time ownership:

- embedded acquisition producers stamp samples from a local `MonotonicClock`
- the FC SPI sensor cluster schedules work from local monotonic time
- current Core 0 sensor snapshot logging uses a caller-provided time source that is currently backed by local uptime on the RP2354B target

Under full HIL takeover:

- HIL-backed acquisition becomes the active measurement authority
- real sensor producers are disabled for that backend
- estimator and control consume only HIL-backed sample timestamps
- logging snapshots that represent simulated progression are emitted against HIL tick time

## 13. Acceptance Criteria

This plan is satisfied when all of the following are true:

- HIL can be enabled on more than one target without moving HIL business logic into each target crate
- the same portable mission, estimation, and control code runs against real, HIL, and replay backends
- protocol choice can vary without redefining the HIL architecture
- when HIL is active, portable flight progression follows simulation time rather than embedded wall-clock time
- pausing or stepping the simulator pauses or steps the portable flight stack deterministically
- targets remain responsible only for transport setup, resource ownership, and task spawning

## 14. Immediate Next Steps

1. create `common/services/hil/` and define protocol-neutral HIL semantic types
2. define reusable ingress and egress task bodies in `common`
3. define the canonical runtime rules for tick acceptance and timestamp authority
4. refactor the current FC MAVLink HIL path to use the new common framework
5. add one additional target or host runtime path to prove the design is truly reusable
