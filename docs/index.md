# MARV Firmware Architecture Design Documents

This document set defines the architectural guard rails for the MARV firmware stack across:

- Rocket launcher stack (`RL`)
- Senior project drone stack (`SP`)
- Flight controller (`FC`)
- Radio
- Ground station (`GS`)
- SITL backends

Primary goals:

- Separate HAL-dependent and HAL-agnostic logic
- Preserve portability across RP2354 targets and SITL
- Protect timing-critical execution paths
- Keep task creation deliberate
- Make watchdog supervision a first-class architectural concern
- Maintain explicit ownership of buses, channels, cores, and responsibilities

## Repository Architecture Tree

This is the target repository shape the docs define. It is the architecture we are
aligning the workspace toward, even where legacy crates or legacy module names
still exist during migration.

```text
MARV-FC/
├── common/
│   ├── Cargo.toml
│   └── src/
│       ├── interfaces/
│       │   ├── sensors/
│       │   ├── comms/
│       │   ├── storage/
│       │   ├── actuators/
│       │   ├── timing/
│       │   ├── health/
│       │   └── system/
│       ├── messages/
│       │   ├── sensor/
│       │   ├── estimate/
│       │   ├── control/
│       │   ├── telemetry/
│       │   ├── logging/
│       │   └── fault/
│       ├── drivers/
│       │   ├── sensors/
│       │   ├── storage/
│       │   ├── radio/
│       │   └── leds/
│       ├── protocol/
│       │   ├── mavlink/
│       │   ├── ubx/
│       │   ├── framing/
│       │   ├── crc/
│       │   └── packet_types/
│       ├── comms/
│       │   ├── links/
│       │   ├── routing/
│       │   ├── session/
│       │   ├── retry/
│       │   └── bridge/
│       ├── policies/
│       │   ├── arming/
│       │   ├── modes/
│       │   ├── faults/
│       │   ├── failsafe/
│       │   └── mission/
│       ├── localization/
│       │   ├── attitude/
│       │   ├── navigation/
│       │   ├── sensor_fusion/
│       │   ├── calibration/
│       │   └── state/
│       ├── control/
│       │   ├── rate/
│       │   ├── attitude/
│       │   ├── guidance/
│       │   ├── mixing/
│       │   └── outputs/
│       ├── services/
│       │   ├── acquisition/
│       │   ├── estimation/
│       │   ├── control/
│       │   ├── telemetry/
│       │   ├── logging/
│       │   ├── health/
│       │   └── status/
│       ├── tasks/
│       │   ├── fast_loop/
│       │   ├── medium_loop/
│       │   ├── slow_loop/
│       │   └── background/
│       ├── utilities/
│       │   ├── math/
│       │   ├── filters/
│       │   ├── buffers/
│       │   ├── units/
│       │   ├── time/
│       │   └── ids/
│       └── prelude/
├── device/
│   ├── MARV-FC-RL-RP2354B/
│   │   └── src/
│   │       ├── resources.rs
│   │       ├── pinmap.rs
│   │       ├── interrupts.rs
│   │       ├── config.rs
│   │       ├── clocks.rs
│   │       ├── buses.rs
│   │       ├── channels.rs
│   │       ├── watchdog.rs
│   │       ├── core0.rs
│   │       ├── core1.rs
│   │       └── main.rs
│   ├── MARV-RADIO-*/
│   ├── MARV-GS-*/
│   └── MARV-*-SITL/
├── sim/
│   ├── shared/
│   ├── backends/
│   └── MARV-*-SITL/
└── docs/
```

Current migration note:

- The preserved LoRa implementation remains parked under
  `common/src/coms/transport/lora/` and should be moved into
  `common/src/comms/links/lora/` only as a deliberate follow-up migration.

## Document Index

1. [`01-overview-and-scope.md`](./01-overview-and-scope.md)
2. [`02-core-principles-and-guard-rails.md`](./02-core-principles-and-guard-rails.md)
3. [`03-repository-structure.md`](./03-repository-structure.md)
4. [`04-layer-responsibilities.md`](./04-layer-responsibilities.md)
5. [`05-function-classification-and-task-policy.md`](./05-function-classification-and-task-policy.md)
6. [`06-timing-isolation-and-core-partitioning.md`](./06-timing-isolation-and-core-partitioning.md)
7. [`07-communication-bus-and-dataflow.md`](./07-communication-bus-and-dataflow.md)
8. [`08-watchdog-architecture-and-liveness-supervision.md`](./08-watchdog-architecture-and-liveness-supervision.md)
9. [`09-target-guidance-and-milestones.md`](./09-target-guidance-and-milestones.md)
10. [`10-architecture-and-abstractions-visualization.md`](./10-architecture-and-abstractions-visualization.md)

## Reading Order

Recommended order:

1. Overview and scope
2. Core principles and guard rails
3. Repository structure
4. Layer responsibilities
5. Function classification and task policy
6. Timing isolation and core partitioning
7. Communication, buses, and dataflow
8. Watchdog architecture and liveness supervision
9. Target-specific guidance and milestones
10. Architecture and abstractions visualization

## Intended Use

These documents are intended to serve as:

- architectural guard rails
- onboarding material
- design review reference
- implementation boundary reference
- SITL parity reference
- long-term maintainability constraints

This document set is intentionally detailed at the architectural level, but it does not define implementation specifics beyond module boundaries, ownership, and system behavior constraints.
