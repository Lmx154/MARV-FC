# MARV Repo Instructions

Start with the canonical docs in `docs/00-system-definition-spine.md` through `docs/05-current-deviations-and-migration.md`.
Load one targeted file from `docs/references/` only when the change touches that area.
Use `docs/templates/ai-change-spec.md` for non-trivial changes.

## Architecture Split

- `common/` is portable logic.
- `device/` is embedded target assembly.
- The current host-side simulation runtime lives in `simulator/` as a transitional stand-in for the long-term `sim/` layout.

## Code Classification

- ISR: latch, timestamp, acknowledge, notify.
- Tier 0: edge I/O and hardware ownership.
- Tier 1: pure transforms and parsing.
- Tier 2: stateful services, policy, estimation, control, health, and watchdog supervision.
- Tasks are scheduling boundaries, not architecture layers.

## Timing Rules

- No variable-latency work in the fast control path without explicit justification.
- Portable estimation, control, and mission logic derive `dt` from typed sample timestamps.
- Core 0 owns deterministic and feed-critical work.
- Core 1 owns slower or more variable-latency work by default.

## Message and Dataflow Rules

- Prefer typed messages at ownership and cadence boundaries.
- Same-core pub-sub is the default fan-out model.
- Cross-core exchange requires an explicit target-owned bridge in `channels.rs`.
- Telemetry and logging are subscribers and must not backpressure feed-critical producers.
- Shared buses need explicit ownership.

## Watchdog Rules

- Only one architectural owner may feed the hardware watchdog.
- Feed is allowed only from validated forward progress.
- Core 0 owns final watchdog feed authority on flight-critical targets.

## Validation Rules

- Every non-trivial change must name exact touched files, tests, validation commands, and required measurements.
- If a change alters architecture, update the canonical docs first.
- If the repo still differs from the target architecture after the change, update `docs/05-current-deviations-and-migration.md`.

## Current Migration Notes

- Preserved LoRa code remains under `common/src/coms/transport/lora/` until a deliberate migration moves it to `common/src/comms/links/lora/`.
- `common/src/services/acquisition/hil/` is a compatibility shim while HIL converges on `common/src/services/hil/`.
