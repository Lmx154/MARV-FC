# 09. Target Guidance and Architectural Milestones

## Target-specific guidance

This section captures guidance that should shape early architectural thinking for MARV targets.

---

## MARV-FC-RL-RP2354B

This target should treat the following as first-class hardware and timing domains.

### SPI1 shared sensor cluster
- BMI088 accel
- BMI088 gyro
- LSM6DSV32X

This is likely part of the fast sensing domain and should be treated as timing-sensitive.

### SPI0 storage domain
- microSD

This should be isolated from critical control/estimation timing as much as possible.

### I2C0 environmental domain
- BMP581

### I2C1 auxiliary navigation domain
- BMM350
- u-blox NEO M9N

### UART0 link domain
- FC ↔ Radio

### UART1 companion domain
- FC ↔ SBC

### PWM / actuator domain
- actuator outputs

### LED / status domain
- SK6805 status output

This grouping matters because architecture is not only code organization.  
It is also timing and ownership organization.

---

## MARV-RADIO targets

Radio targets should likely emphasize:

- link/session stability
- transport reliability
- bridge behavior
- telemetry/command routing
- health and watchdog relevance of link-state machinery
- clean separation between protocol and transport ownership

These targets may have different criticality posture than FC targets, but should still respect the same architectural layering.

---

## MARV-GS targets

Ground station targets should preserve the same architecture where practical:

- protocol separation
- session/link separation
- message discipline
- watchdog awareness if embedded runtime reliability matters
- SITL parity where applicable

Ground station timing concerns are often different from FC timing concerns, but ownership and layering discipline still matter.

---

## SITL targets

SITL targets should mirror the embedded targets conceptually.

The goal is not to create a loose mock environment.  
The goal is to preserve architectural contracts so that:

- estimator logic remains the same
- control logic remains the same
- policy logic remains the same
- health/watchdog logic remains structurally meaningful
- message flow remains comparable
- scenarios can exercise real failure paths

---

## Recommended architectural milestones

These are architectural milestones, not code milestones.

### Milestone 1 — Freeze folder structure and guard rails
The architecture must have stable top-level boundaries before implementation spreads.

### Milestone 2 — Define `common/interfaces/`
This is the most important seam in the system.

### Milestone 3 — Define `common/messages/`
Typed message flow should be established before large services emerge.

### Milestone 4 — Define function classification rules
The team should agree on:
- ISR constraints
- Tier 0 behavior
- Tier 1 behavior
- Tier 2 behavior
- task creation policy

### Milestone 5 — Define per-target `device/.../resources.rs` responsibilities
Hardware assembly boundaries must be explicit early.

### Milestone 6 — Define per-target channel topology
This should happen before ad hoc channels spread everywhere.

### Milestone 7 — Define per-target core ownership
Core 0 vs Core 1 responsibilities should be a design decision, not an afterthought.

### Milestone 8 — Define watchdog liveness contract
Before implementation proceeds too far, the team should agree on:
- who can feed
- what counts as progress
- which failures are feed-critical
- what reset/reporting behavior is expected

### Milestone 9 — Define SITL symmetry
SITL should be shaped alongside the embedded design, not retrofitted later.

### Milestone 10 — Begin vertical bring-up of one narrow sensor path
Only after boundaries are stable should the team begin vertical bring-up such as:
- one bus
- one sensor
- one typed sample
- one service path
- one health/liveness path

This helps validate the architecture without prematurely expanding complexity.

---

## Review checklist for new work

Before approving a new module or service, ask:

1. Does it belong in `common/`, `device/`, or `sim/`?
2. Is it ISR, Tier 0, Tier 1, or Tier 2?
3. Does it introduce a new task?
4. If so, what timing/ownership need justifies that task?
5. Does it touch a shared bus?
6. Who owns that bus?
7. Is it on the watchdog-critical path?
8. Can it affect Core 0 determinism?
9. Can it run in SITL without rewrite?
10. Does it introduce new cross-layer leakage?

If these answers are vague, the design likely needs refinement.

---

## Final position

The first goal is not to code quickly.  
The first goal is to preserve architecture while the system is still small enough to protect.

A strong architecture early will make later work in:

- estimators
- comms
- radio links
- control loops
- SITL
- health supervision
- watchdog behavior

much easier to scale safely.