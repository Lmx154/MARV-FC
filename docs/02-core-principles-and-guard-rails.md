# 02. Core Principles and Guard Rails

## Core principle

The firmware shall be split into two primary implementation domains and one simulation domain:

- `common/` → portable, HAL-agnostic logic
- `device/` → hardware-specific runtime assembly
- `sim/` → simulation-specific runtime assembly and interface backends

The most important rule is:

> `common/` may depend on portable async and trait-based abstractions, but it must not depend on concrete `embassy-rp` peripheral types.

---

## Allowed in `common/`

The following belong in `common/`:

- domain logic
- protocols
- parsers/serializers
- estimator logic
- control logic
- policy state machines
- health supervision
- watchdog supervisory logic
- services and orchestration logic
- portable drivers written against generic traits
- message types
- channel-facing task bodies that remain platform-independent
- pure transforms and math helpers
- time abstractions
- portable async abstractions

Typical dependencies allowed in `common/`:

- `embassy-sync`
- `embassy-time`
- `heapless`
- `embedded-hal`
- `embedded-hal-async`
- `embedded-io`
- `embedded-io-async`

---

## Not allowed in `common/`

The following shall not appear in `common/`:

- `embassy_rp::spi::Spi<...>`
- `embassy_rp::uart::Uart<...>`
- `embassy_rp::i2c::I2c<...>`
- RP-specific pin types
- RP-specific DMA channel types
- RP-specific interrupt bindings
- raw peripheral singleton construction
- pin maps
- reset/watchdog register access
- concrete board resource ownership
- any assumption that a specific board is active

---

## Why this matters

This separation protects:

- SITL reuse
- future portability to new MCUs
- testability of domain logic
- clarity of platform boundaries
- timing reasoning
- code review discipline

This keeps the firmware architecture from collapsing into platform-specific glue everywhere.

---

## Architectural guard rails

### Rule 1
`common/` never imports concrete RP peripheral types.

### Rule 2
`device/` owns all pin, interrupt, DMA, and peripheral construction.

### Rule 3
`sim/` implements the same abstract interfaces consumed by `common/`.

### Rule 4
No board-specific constants shall leak into generic estimator, control, policy, protocol, or service logic unless intentionally passed as configuration.

### Rule 5
Task bodies may be portable, but task spawning, executor assignment, and core placement are platform-owned.

### Rule 6
Drivers should expose typed data upward, not raw register semantics.

### Rule 7
Protocols do not own transport hardware.

### Rule 8
Policies do not touch hardware directly.

### Rule 9
Fast-loop logic should consume typed messages or stable snapshots, not reach across the codebase for arbitrary device state.

### Rule 10
Slow or variable-latency I/O must not sit directly in the flight-critical path unless intentionally justified.
Time-sensitive logging capture may live on Core 0 beside the publishers, but any slow logging sink behavior must remain buffered and non-blocking.

### Rule 11
Only one architectural owner may feed the hardware watchdog.

### Rule 12
Watchdog feeding must be based on validated liveness, not random task activity.

---

## Architectural questions every new module must answer

Every new module should be classifiable by answering these questions:

1. Is it portable?
2. Is it hardware assembly?
3. Is it a simulation backend?
4. Is it protocol, policy, service, or driver?
5. Is it on the timing-critical path?
6. Does it own hardware?
7. Does it own state?
8. Is it allowed to block?
9. Is it allowed to feed the watchdog?
10. Does it belong on Core 0 or Core 1?

If those answers are unclear, the module boundary is probably wrong.

---

## Architectural anti-patterns

### Do not let `common/drivers/` become a second HAL
A portable driver may use generic traits, but if it fundamentally depends on RP-specific interrupt or DMA knowledge, that portion belongs in `device/`.

### Do not make `tasks/` the architecture
Tasks are scheduling containers, not the architecture itself.

### Do not turn `utilities/` into a junk drawer
If something has a real role, it deserves a real home.

### Do not fuse protocol and transport
Byte format and link behavior are not the same thing.

### Do not allow non-critical work to share timing fate with critical work by default
Telemetry, debug, and indicators must not casually introduce jitter into control and estimation paths.
Time-sensitive logging is allowed to remain on Core 0 when it is part of the same measurement-observability boundary, but buffered sinks must still be prevented from stalling the fast path.

### Do not let SITL become a side project with a different mental model
It should mirror production architecture as closely as practical.

### Do not allow arbitrary tasks to feed the watchdog
That can hide critical failures and defeat the whole purpose of the watchdog.

---

## Naming guidance

Use target names consistently across:

- code
- modules
- docs
- logs
- build targets
- CI
- scenarios

Recommended naming pattern:

- `MARV-[ROLE]-[PROGRAM]-[TARGET]`

Examples:

- `MARV-FC-RL-RP2354B`
- `MARV-RADIO-SP-RP2354A`
- `MARV-FC-RL-SITL`

This naming scheme is preferable to generic names like `board/`, `platform/`, or `rp2354/` because it preserves role clarity and avoids ambiguity.

---

## Final position

These guard rails are intentionally strict.

The cost of slightly more structure now is much lower than the cost of trying to untangle:

- cross-layer leakage
- hardware assumptions in portable logic
- too many tasks
- unclear ownership
- weak timing boundaries
- unreliable watchdog behavior

later.
