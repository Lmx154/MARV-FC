# 02. Interface Contracts

This document defines the stable seams of the architecture.

## Capability Contracts

- `common/interfaces/` defines what the system needs from sensors, comms, storage, actuators, timing, health, and system services.
- Interfaces stay minimal, typed, and portable.
- Interfaces must not expose board pin choices, DMA channels, interrupt bindings, or concrete RP peripheral types.

## Message Contracts

- `common/messages/` defines the typed data that crosses subsystem, task, and core boundaries.
- Messages must be explicit, bounded, no-heap-friendly, and cheap to move or copy.
- Sensor and runtime messages carry the timestamps and freshness information needed by downstream portable logic.

## Producer and Consumer Contracts

- A producer owns the acquisition boundary for its data source.
- Producers stamp measurement time at acquisition and publish typed samples.
- Portable consumers compute `dt` and freshness from those typed samples, not from local scheduler cadence.
- Request/response should be exceptional; stream, pub-sub, and snapshot flows are the default patterns.

## Layer Contracts

- `common/protocol/` translates bytes to structured packets and back.
- `common/comms/` owns routing, retry, session, heartbeats, bridging, and link behavior above protocol.
- `common/services/` orchestrates reusable stateful workflows over interfaces, messages, and policies.
- `common/tasks/` may contain portable task bodies, but spawning, executor assignment, and core placement belong to target assembly.

## Target Assembly Contracts

Per-target files have stable responsibilities:

- `resources.rs`: concrete peripheral and resource ownership
- `pinmap.rs`: board pin assignments and aliases
- `interrupts.rs`: interrupt bindings when the target needs them
- `config.rs`: target-owned runtime configuration
- `clocks.rs`: clocks and timebase setup
- `buses.rs`: shared-bus ownership and arbitration wrappers
- `channels.rs`: same-core topology, cross-core bridges, and channel policy
- `watchdog.rs`: concrete watchdog hardware ownership, feed API, and reset-cause access
- `core0.rs` and `core1.rs`: executor composition and task placement
- `main.rs`: assembly root and boot wiring

## Cross-Core and Observability Contracts

- Fast-path streams remain core-local by default.
- Cross-core mirrors or bridges must be explicit, non-incidental, and owned by the target.
- Telemetry and logging consume copies, snapshots, or buffered records.
- Slow sinks and serializers never become the ownership boundary for the fast path.

## Simulation and HIL Contracts

- Simulation, replay, and HIL ingress must publish the same canonical typed samples that embedded producers publish.
- Simulation, replay, and HIL egress must consume the same portable output contracts that embedded targets consume.
- The current repo uses `simulator/` as the transitional host runtime for those contracts until the long-term `sim/` layout is adopted.
