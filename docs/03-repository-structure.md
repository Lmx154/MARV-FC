# 03. Repository Structure

## Repository intent

The repository layout should encode the architecture itself.

The structure should make it difficult to accidentally mix:

- hardware assembly
- portable logic
- simulation backends
- protocols
- policies
- timing-critical runtime decisions

The repo tree is therefore part of the architecture, not just a convenience.

---

## Refined repository tree

```text
firmware/
├── common/
│   ├── interfaces/
│   │   ├── sensors/
│   │   ├── comms/
│   │   ├── storage/
│   │   ├── actuators/
│   │   ├── timing/
│   │   ├── health/
│   │   └── system/
│   │
│   ├── messages/
│   │   ├── sensor/
│   │   ├── estimate/
│   │   ├── control/
│   │   ├── telemetry/
│   │   ├── logging/
│   │   └── fault/
│   │
│   ├── drivers/
│   │   ├── sensors/
│   │   │   ├── bmi088/
│   │   │   ├── lsm6dsv32x/
│   │   │   ├── bmp581/
│   │   │   ├── bmm350/
│   │   │   └── neo_m9n/
│   │   ├── storage/
│   │   │   └── microsd/
│   │   ├── radio/
│   │   │   └── sx1262/
│   │   └── leds/
│   │
│   ├── protocol/
│   │   ├── mavlink/
│   │   ├── ubx/
│   │   ├── framing/
│   │   ├── crc/
│   │   └── packet_types/
│   │
│   ├── comms/
│   │   ├── links/
│   │   ├── routing/
│   │   ├── session/
│   │   ├── retry/
│   │   └── bridge/
│   │
│   ├── policies/
│   │   ├── arming/
│   │   ├── modes/
│   │   ├── faults/
│   │   ├── failsafe/
│   │   └── mission/
│   │
│   ├── localization/
│   │   ├── attitude/
│   │   ├── navigation/
│   │   ├── sensor_fusion/
│   │   ├── calibration/
│   │   └── state/
│   │
│   ├── control/
│   │   ├── rate/
│   │   ├── attitude/
│   │   ├── guidance/
│   │   ├── mixing/
│   │   └── outputs/
│   │
│   ├── services/
│   │   ├── acquisition/
│   │   ├── estimation/
│   │   ├── control/
│   │   ├── telemetry/
│   │   ├── logging/
│   │   ├── health/
│   │   └── status/
│   │
│   ├── tasks/
│   │   ├── fast_loop/
│   │   ├── medium_loop/
│   │   ├── slow_loop/
│   │   └── background/
│   │
│   ├── utilities/
│   │   ├── math/
│   │   ├── filters/
│   │   ├── buffers/
│   │   ├── units/
│   │   ├── time/
│   │   └── ids/
│   │
│   └── prelude/
│
├── device/
│   ├── MARV-FC-RL-RP2354B/
│   │   ├── resources.rs
│   │   ├── pinmap.rs
│   │   ├── interrupts.rs
│   │   ├── config.rs
│   │   ├── clocks.rs
│   │   ├── buses.rs
│   │   ├── channels.rs
│   │   ├── watchdog.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   └── main.rs
│   │
│   ├── MARV-RADIO-RL-RP2354A/
│   │   ├── resources.rs
│   │   ├── pinmap.rs
│   │   ├── interrupts.rs
│   │   ├── config.rs
│   │   ├── clocks.rs
│   │   ├── buses.rs
│   │   ├── channels.rs
│   │   ├── watchdog.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   └── main.rs
│   │
│   ├── MARV-GS-RL-RP2354A/
│   │   ├── resources.rs
│   │   ├── pinmap.rs
│   │   ├── interrupts.rs
│   │   ├── config.rs
│   │   ├── clocks.rs
│   │   ├── buses.rs
│   │   ├── channels.rs
│   │   ├── watchdog.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   └── main.rs
│   │
│   ├── MARV-FC-SP-RP2354B/
│   │   ├── resources.rs
│   │   ├── pinmap.rs
│   │   ├── interrupts.rs
│   │   ├── config.rs
│   │   ├── clocks.rs
│   │   ├── buses.rs
│   │   ├── channels.rs
│   │   ├── watchdog.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   └── main.rs
│   │
│   ├── MARV-RADIO-SP-RP2354A/
│   │   ├── resources.rs
│   │   ├── pinmap.rs
│   │   ├── interrupts.rs
│   │   ├── config.rs
│   │   ├── clocks.rs
│   │   ├── buses.rs
│   │   ├── channels.rs
│   │   ├── watchdog.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   └── main.rs
│   │
│   └── MARV-GS-SP-RP2354/
│       ├── resources.rs
│       ├── pinmap.rs
│       ├── interrupts.rs
│       ├── config.rs
│       ├── clocks.rs
│       ├── buses.rs
│       ├── channels.rs
│       ├── watchdog.rs
│       ├── core0.rs
│       ├── core1.rs
│       └── main.rs
│
├── sim/
│   ├── shared/
│   │   ├── resources.rs
│   │   ├── clocks.rs
│   │   ├── channels.rs
│   │   ├── transports.rs
│   │   └── world.rs
│   │
│   ├── backends/
│   │   ├── sensors/
│   │   │   ├── imu.rs
│   │   │   ├── baro.rs
│   │   │   ├── mag.rs
│   │   │   ├── gps.rs
│   │   │   └── actuator_feedback.rs
│   │   ├── comms/
│   │   │   ├── uart.rs
│   │   │   ├── radio.rs
│   │   │   └── loopback.rs
│   │   ├── storage/
│   │   │   └── log_sink.rs
│   │   └── timing/
│   │       ├── scheduler.rs
│   │       └── watchdog.rs
│   │
│   ├── MARV-FC-RL-SITL/
│   │   ├── config.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   ├── scenario.rs
│   │   └── main.rs
│   │
│   ├── MARV-RADIO-RL-SITL/
│   │   ├── config.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   ├── scenario.rs
│   │   └── main.rs
│   │
│   ├── MARV-GS-RL-SITL/
│   │   ├── config.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   ├── scenario.rs
│   │   └── main.rs
│   │
│   ├── MARV-FC-SP-SITL/
│   │   ├── config.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   ├── scenario.rs
│   │   └── main.rs
│   │
│   ├── MARV-RADIO-SP-SITL/
│   │   ├── config.rs
│   │   ├── core0.rs
│   │   ├── core1.rs
│   │   ├── scenario.rs
│   │   └── main.rs
│   │
│   └── MARV-GS-SP-SITL/
│       ├── config.rs
│       ├── core0.rs
│       ├── core1.rs
│       ├── scenario.rs
│       └── main.rs
│
├── cerberus/
│   ├── src/
│   │   ├── backend/
│   │   ├── bridge/
│   │   ├── frontend/
│   │   ├── app.rs
│   │   └── main.rs
│   │
│   └── gazebo_bridge/
│       ├── include/
│       ├── src/
│       └── CMakeLists.txt
│
└── docs/
    ├── architecture/
    ├── timing/
    ├── interfaces/
    ├── pinmaps/
    └── scenarios/
````

---

## Structural meaning

### `common/`

Holds reusable intelligence and portable logic.

### `device/`

Holds embedded runtime assembly and concrete ownership of hardware resources.

### `sim/`

Holds simulation-specific platform assembly and interface-compatible virtual backends.

### `cerberus/`

Holds the host-side visualization and HIL control UI.

Cerberus is part of the repo because it consumes the same firmware contracts it visualizes:

* `src/backend/` owns host services and parsers
* `src/bridge/` maps backend state into UI state
* `src/frontend/` owns the egui presentation
* `gazebo_bridge/` owns the C++ Gazebo transport bridge used by the UI backend

### `docs/`

Holds supporting documentation, diagrams, timing notes, pin maps, and scenarios.

---

## Why separate `device/` and `sim/`

It is tempting to treat SITL as a test harness glued onto the side. That should be avoided.

SITL should be treated as another platform target.

That means:

* `device/` assembles concrete RP hardware
* `sim/` assembles virtual hardware
* `common/` remains the shared logic center

This symmetry helps preserve:

* mental model consistency
* interface discipline
* long-term maintainability
* replay and analysis potential

---

## Why `watchdog.rs` is explicit

The watchdog should not disappear into `resources.rs` or `main.rs` as a detail.

Making it explicit reinforces that it is:

* a hardware resource
* a safety mechanism
* a first-class architectural concern

This also supports symmetry with:

* `common/services/health/`
* `sim/backends/timing/watchdog.rs`

---

## Why `channels.rs` is explicit

The channel topology of a target is not an implementation detail. It encodes:

* data ownership
* latency boundaries
* scheduling boundaries
* failure propagation paths
* core locality and any explicit cross-core bridges

That is important enough to deserve an explicit home.

---

## Why `core0.rs` and `core1.rs` are explicit

Core placement is a design decision, not just a runtime detail.

Separate files make it easier to reason about:

* timing isolation
* criticality separation
* core-specific responsibilities
* watchdog liveness dependencies

---

## Final position

This repository structure is intentionally opinionated.
It exists to make architectural mistakes harder to commit and easier to detect during review.
