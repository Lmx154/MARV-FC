# 00. System Definition Spine

This is the top-level control document for the MARV firmware architecture.
It defines the mission, the hard constraints, and the canonical map of the system.
Detailed rationale, examples, diagrams, and target notes live in `docs/references/`.

## Mission

Build a portable, timing-aware, safety-conscious firmware stack that can run on embedded RP235x targets and contract-preserving simulated backends without rewriting the core system logic.

## Scope

This architecture governs:

- `common/` portable firmware logic
- `device/` embedded target assembly
- host-side simulation and HIL backends, including the current transitional `simulator/` crate

It applies first to the flight-controller stack and its supporting targets, and it is intended to scale to additional MARV device families without changing the architectural laws.

## Goals

- Keep HAL-dependent and HAL-agnostic code separate.
- Preserve SITL, HIL, replay, and offline-analysis reuse.
- Protect deterministic control and estimation paths from variable-latency work.
- Make ownership of buses, channels, cores, and watchdog authority explicit.
- Keep task creation deliberate and reviewable.

## Derived Objectives

- Make `common/` the home of reusable logic, typed contracts, and portable services.
- Make `device/<target>/` the only place that constructs concrete peripherals, pins, interrupts, DMA, executors, and core placement.
- Make simulation and HIL inject the same typed samples and consume the same typed outputs as embedded targets.
- Make measurement timestamps, not scheduler cadence, drive portable estimation and control time progression.

## Hard Constraints

- `common/` must not depend on concrete `embassy-rp` peripheral types.
- Tasks are scheduling boundaries, not architecture layers.
- ISR, Tier 0, Tier 1, and Tier 2 responsibilities must stay distinct.
- Core 0 owns deterministic and feed-critical work on flight-critical targets.
- Cross-core exchange is explicit and target-owned; same-core fan-out is the default.
- Telemetry and logging are subscribers and must not backpressure the fast path.
- The hardware watchdog has one feed owner, and feed permission depends on validated forward progress.

## Capability Map

- Acquisition: sensors, buses, transport ingress, typed sample publication
- Estimation and localization: timestamp-driven state estimation and fusion
- Control and actuation: control-law execution, actuator intents, output shaping
- Policy and mission: arming, mode, failsafe, mission, reset and fault policy
- Communication: protocol parsing/framing, routing, retry, session, bridge behavior
- Health and watchdog: liveness, deadlines, fault aggregation, watchdog supervision
- Observability: telemetry, logging, reset-cause reporting, replay-compatible records
- Simulation and HIL: step-driven sample injection, watchdog parity, canonical output egress

## Layer Map

- `common/interfaces/` defines capabilities.
- `common/messages/` defines typed payloads at stable seams.
- `common/drivers/`, `protocol/`, `comms/`, `policies/`, `localization/`, `control/`, and `services/` hold portable logic.
- `common/tasks/` may hold portable task bodies, but not placement.
- `device/<target>/` owns hardware assembly, channels, buses, watchdog hardware, and per-core runtime composition.
- Simulation and HIL backends mirror the same contracts. The current repo uses `simulator/` as a transitional host runtime instead of the long-term `sim/` layout.

## Interface Map

- Capability seam: `common/interfaces/`
- Message seam: `common/messages/`
- Transport/protocol seam: `common/protocol/` below `common/comms/`
- Platform seam: per-target `resources.rs`, `buses.rs`, `channels.rs`, `watchdog.rs`, `core0.rs`, and `core1.rs`
- Simulation/HIL seam: canonical typed sample ingress and canonical output egress

## Verification Stack

1. Tier 1 unit tests for math, transforms, parsing, and deadline helpers
2. Boundary integration tests for drivers, protocols, messages, services, and channels
3. Host/SITL/replay validation for timestamp semantics, step-driven progression, and contract parity
4. HIL and target validation for watchdog, reset, liveness, and degraded-mode behavior
5. Timing and resource evidence for jitter, freshness, RAM/flash deltas, bus occupancy, and log volume

## Canonical Reading Order

1. `00-system-definition-spine.md`
2. `01-architecture-rules.md`
3. `02-interface-contracts.md`
4. `03-verification-strategy.md`
5. `04-review-checklist.md`
6. `05-current-deviations-and-migration.md`

References are supporting material. If a reference conflicts with a canonical doc, the canonical doc wins.
