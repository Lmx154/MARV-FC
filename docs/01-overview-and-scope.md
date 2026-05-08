# 01. Overview and Scope

## Purpose

This design document defines the architectural shape of the MARV firmware stack and establishes guard rails for separating HAL-dependent and HAL-agnostic logic across:

- flight controllers
- radios
- ground stations
- SITL backends

The architecture is designed for a reusable, timing-aware, safety-conscious firmware stack that can support both embedded deployment and simulation without rewriting core system logic.

---

## Scope

This architecture applies to the following target families:

### Rocket launcher stack (`RL`)
- flight controller
- radio
- ground station
- SITL backends

### Senior project drone stack (`SP`)
- flight controller
- radio
- ground station
- SITL backends

---

## Primary targets

- **MCU / HAL:** RP2354A / RP2354B
- **Language:** Rust
- **Runtime ecosystem:** Embassy, Embassy-RP, Embassy sync/time/executor patterns

---

## Architectural goals

The firmware architecture shall prioritize:

- timing isolation
- explicit module boundaries
- clear separation of hardware-specific and portable logic
- portability of flight logic into SITL
- low ambiguity in ownership and scheduling
- latency-conscious execution
- deliberate task creation
- watchdog-aware safety design
- long-term maintainability

---

## Non-goals

This document does **not** attempt to define:

- complete implementation details
- exact trait signatures
- exact register maps
- exact channel buffer sizes
- exact loop rates for every target
- full control/estimator math
- exact build system structure

Those belong in follow-on implementation specifications and target-specific bring-up documents.

---

## Design philosophy

The architecture should not be organized around what is easiest to code first.

It should instead be organized around:

- portability
- timing
- ownership
- system criticality
- testability
- simulation parity
- safety

This is especially important for a firmware stack that may eventually support:

- safety-critical control loops
- state estimation
- sensor fusion
- communications bridging
- telemetry and logging
- fault escalation
- watchdog supervision

---

## Key idea

The single most important architectural decision is that the firmware stack is split into three major domains:

- `common/` → HAL-agnostic portable logic
- `device/` → embedded platform assembly and HAL ownership
- `sim/` → simulation platform assembly and virtual backends

This ensures that the reusable intelligence of the system lives outside platform-specific hardware code.

---

## High-level architectural view

```text
+--------------------------------------------------------------+
|                          APPLICATION                          |
|  mission logic | arming logic | failsafe logic | mode logic  |
+--------------------------------------------------------------+
|                       DOMAIN / SERVICES                       |
| acquisition | estimation | control | telemetry | health      |
+--------------------------------------------------------------+
|                   PROTOCOL / COMMUNICATION                    |
| mavlink | ubx | routing | framing | retries | sessions       |
+--------------------------------------------------------------+
|                   DRIVERS / DEVICE ADAPTERS                   |
| imu | baro | gps | mag | sd | radio | actuators | leds       |
+--------------------------------------------------------------+
|                 HAL BOUNDARY / PLATFORM LAYER                 |
| resources | pins | interrupts | dma | executors | buses      |
+--------------------------------------------------------------+
|                    HARDWARE OR SIM BACKEND                    |
| RP2354A/B boards            OR          SITL models          |
+--------------------------------------------------------------+
````

---

## Portability target

The architecture shall allow core logic to be reused across:

* `MARV-FC-RL-RP2354B`
* `MARV-RADIO-RL-RP2354A`
* `MARV-GS-RL-RP2354A`
* `MARV-FC-SP-RP2354B`
* `MARV-RADIO-SP-RP2354A`
* `MARV-GS-SP-RP2354`
* all corresponding SITL targets

This means the logic for:

* protocols
* estimation
* control
* policy
* message flow
* health supervision
* watchdog supervision rules

should not be tightly coupled to a particular RP peripheral implementation.

---

## Safety and timing position

This architecture assumes that:

* jitter matters
* blocking behavior matters
* bus ownership matters
* task count matters
* core placement matters
* watchdog behavior matters

For that reason, the architecture is intentionally opinionated about:

* what belongs in `common/`
* what belongs in `device/`
* what belongs in `sim/`
* what should or should not become a task
* how watchdog feeding authority should be controlled

---

## Final position

This architecture exists to make the system easier to reason about before implementation complexity grows.

It should be treated as a design constraint document, not just a folder layout suggestion.

The goal is not merely clean code.
The goal is a firmware architecture that is:

* portable
* inspectable
* timing-aware
* simulation-friendly
* safety-conscious
* scalable across MARV targets