# 08. Watchdog Architecture and Liveness Supervision

## Purpose

The watchdog is not just a hardware timer.  
In a firmware architecture like MARV, it should be treated as a liveness contract.

The contract is:

> The system may only feed the hardware watchdog if the required critical functions have made valid forward progress recently.

This is the correct architectural view of a watchdog in a timing-sensitive and safety-conscious system.

---

## Architectural placement

The watchdog exists in three architectural places.

### 1. `device/` — hardware watchdog ownership
This is where the real watchdog peripheral belongs.

Responsibilities:

- watchdog configuration
- watchdog start/enable
- watchdog feed
- pre-timeout interrupt hookup if supported
- reset cause access
- low-level watchdog timing setup

This is **Tier 0 edge functionality**.

Examples of conceptual responsibilities:

- `watchdog_init()`
- `watchdog_feed()`
- `watchdog_read_reset_reason()`

These should be narrow and boring.

---

### 2. `common/services/health/` — watchdog supervisor logic
This is where the decision to feed belongs.

This is **Tier 2 stateful service logic**.

Responsibilities:

- consume task/service liveness inputs
- evaluate deadline/freshness status
- decide whether the system is healthy enough to feed
- classify degraded vs fatal conditions
- publish watchdog status or denial reason
- support policy-controlled escalation

This is the architectural brain of the watchdog.

---

### 3. `sim/` — simulated watchdog backend
SITL should not ignore watchdog behavior.

Simulation should model:

- liveness loss
- feed denial
- reset behavior
- reboot/restart events
- reset cause or fault capture
- watchdog-related scenario testing

This preserves architectural parity and helps validate fault behavior before flight.

---

## The most important rule

> No random task may feed the hardware watchdog directly.

This is a critical guard rail.

Bad pattern:

- telemetry task feeds watchdog
- logger feeds watchdog
- IMU task feeds watchdog
- control task feeds watchdog

That can hide real failure.

Example:

- estimator is deadlocked
- telemetry is still active
- watchdog still gets fed
- unsafe condition persists

This defeats the entire purpose of the watchdog.

The correct pattern is:

- one supervisor evaluates system liveness
- one owner performs the actual feed

---

## Feed authority

Only one architectural owner should feed the hardware watchdog.

Recommended owner:

- a watchdog supervisor task or service running under Core 0 authority

Core 1 may publish liveness and status, but should not have direct feed authority.

---

## ISR role

If the MCU supports watchdog pre-timeout or early-warning interrupt behavior, that belongs in the ISR class.

ISR work should remain minimal:

- latch an early-warning flag
- record minimal metadata
- wake minimal follow-up handling if needed

Do not perform large-scale recovery logic in the ISR.

---

## Tier mapping

### ISR class
- early-warning or pre-timeout capture

### Tier 0
- hardware watchdog init/feed/reset-cause access

### Tier 1
- freshness checks
- timeout math
- heartbeat mask evaluation
- deadline helper functions

### Tier 2
- watchdog supervision
- liveness aggregation
- feed/no-feed decision
- degradation classification
- escalation coordination

---

## Valid forward progress

Watchdog feeding should be based on meaningful forward progress, not merely task wakeups.

Useful liveness criteria include:

- fast loop executed within deadline
- IMU sample freshness valid
- estimator updated recently
- control output updated recently
- actuator path still alive
- critical bus owner not stuck beyond threshold
- essential services reporting recent progress
- required core-to-core heartbeat still valid

This should be explicit in design, not implicit in implementation.

---

## Liveness classes

Not every subsystem should be equally important to watchdog feeding.

### Class A — feed-critical
If these fail, feeding should stop.

Examples:

- fast sensor path
- estimator
- control loop
- actuator output path
- watchdog supervisor itself

### Class B — degrade-critical
These may cause degraded mode before feed denial.

Examples:

- radio link
- telemetry bridge
- GPS
- companion link

### Class C — non-critical
These should not block feeding unless intentionally configured to do so.

Examples:

- LED/status patterns
- best-effort logging
- debug console
- non-essential diagnostics

This prevents useless resets due to non-critical failures.

---

## Recommended structure

### Level 1 — software liveness monitors
Critical services publish health/liveness evidence.

Examples:

- fast loop heartbeat
- estimator freshness
- control freshness
- acquisition freshness
- optional Core 1 service heartbeats

### Level 2 — watchdog supervisor
Consumes the liveness inputs and decides:

- `FeedAllowed`
- `DegradedButFeedable`
- `DoNotFeed`
- denial reason / escalation context

### Level 3 — hardware watchdog
Concrete platform-owned feed call.

---

## Dedicated watchdog task

A dedicated watchdog task is usually a good idea, provided it stays small and deliberate.

Good responsibilities:

- run at a fixed cadence
- evaluate liveness contract
- feed or withhold feed
- publish watchdog state
- avoid becoming a giant generic health manager

This is one of the clearer justifications for a dedicated task because it has:

- explicit ownership
- explicit cadence
- direct safety relevance

---

## Windowed watchdogs

If the hardware supports a windowed watchdog, it should be preferred.

Why:

- classic watchdog catches “too late”
- windowed watchdog also catches “too early”

This helps detect:
- runaway loops
- broken “feed all the time” logic
- invalid feed cadence

If no hardware windowed watchdog exists, a software-enforced min/max feed window may still be valuable.

---

## External watchdog consideration

For higher-assurance hardware revisions, consider an external watchdog/supervisor IC.

Benefits:

- independent timing source
- independent reset path
- potential resilience against more internal failure modes

Architecturally, this still fits the same model:

- `common/` decides whether the system deserves life
- `device/` owns the concrete feed mechanism

---

## Boot and reset integration

The watchdog affects startup as well as runtime.

On boot, the system should inspect and surface:

- reset cause
- watchdog reset indicator if available
- last retained fault cause if available
- prior boot abnormality marker if available

This information should feed into:

- health reports
- telemetry startup status
- logs
- post-flight analysis
- SITL fault/restart testing

---

## Suggested file placement

### In `common/`
```text
common/
├── messages/
│   └── fault/
│       ├── reset_reason.rs
│       ├── watchdog_status.rs
│       └── health_report.rs
│
├── services/
│   └── health/
│       ├── watchdog_supervisor.rs
│       ├── liveness.rs
│       ├── deadlines.rs
│       └── fault_aggregation.rs
│
├── policies/
│   └── faults/
│       ├── escalation.rs
│       └── reset_policy.rs
````

### In `device/<target>/`

```text
device/MARV-FC-RL-RP2354B/
├── watchdog.rs
```

### In `sim/`

```text
sim/
├── backends/
│   └── timing/
│       ├── watchdog.rs
│       └── scheduler.rs
```

---

## Watchdog design rules

### Rule 1

Only one module may feed the hardware watchdog.

### Rule 2

Feed permission must be based on validated liveness of required critical functions.

### Rule 3

Non-critical services must not be allowed to mask critical failures.

### Rule 4

Feeding must not be tied merely to task wakeups; it must be tied to meaningful progress.

### Rule 5

Watchdog reset cause should be captured and surfaced during boot and health reporting.

### Rule 6

SITL should model watchdog expiration and restart behavior.

### Rule 7

Core 0 should own final watchdog feed authority for flight-critical targets unless there is a very strong reason otherwise.

---

## Final position

The watchdog belongs:

* in `device/` as concrete hardware ownership
* in `common/services/health/` as supervisory liveness logic
* in `sim/` as a modeled safety mechanism

The hardware watchdog is not the architecture.
The supervisory contract around it is.
