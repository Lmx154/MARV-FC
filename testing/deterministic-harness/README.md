# MARV Deterministic Harness

This crate is the host-side home for deterministic MARV flight-pipeline tests.
Phase 0 freezes the commands and conventions before the harness grows a
lockstep clock, fixtures, plant models, and MARV pipeline adapters.

## Goals

- Run on the host with normal `cargo test` commands.
- Exercise portable code from `common` before involving device crates, UART,
  sockets, or Gazebo.
- Make every later test replayable from deterministic inputs, explicit time,
  and visible seeds.
- Keep Gazebo as a late-stage integration dependency, not a prerequisite for
  basic estimator, controller, mixer, or actuator checks.

## Non-goals

- No wall-clock sleeps, network sockets, serial ports, or Gazebo dependency in
  the deterministic harness.
- No embedded executor fidelity in Phase 0.
- No randomized noise unless a later test case records its seed.
- No filesystem writes except explicit temporary reports added by later phases.

## Host Commands

Run the host-friendly workspace members:

```sh
cargo test --workspace --exclude marv-fc-rl-rp2354b --exclude marv-fc-sp-rp2354b --exclude marv-radio-rp2354a --exclude payload-controller-rp2350 --exclude recovery-altimeter-rp2350 --exclude rp235x-base --exclude rp235x-test --no-run
```

Run the shared portable library:

```sh
cargo test -p common --no-run
cargo test -p common
```

Run the Tauri backend crate:

```sh
cargo test -p telemetry-app --no-run
cargo test -p telemetry-app
```

Run this deterministic harness crate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml --no-run
cargo test --manifest-path testing/deterministic-harness/Cargo.toml
```

## Workspace Host Compatibility

Host-friendly members:

- `common`: portable no-std library with host unit tests enabled under
  `#[cfg(test)]`.
- `telemetry-app/src-tauri`: host Tauri backend and bridge logic.
- `testing/deterministic-harness`: this crate.

Embedded-only members:

- `device/MARV-FC-RL-RP2354B`
- `device/MARV-FC-SP-RP2354B`
- `device/MARV-RADIO-RP2354A`
- `device/PAYLOAD-CONTROLLER-RP2350`
- `device/RECOVERY-ALTIMETER-RP2350`
- `device/RP235X-BASE`
- `device/RP235X-TEST`

Those device crates target RP235x firmware builds and should be built with
their explicit embedded target/tooling rather than as part of host harness
checks.

## Frozen Conventions

The harness assumes the same conventions used by `common`.

- Navigation frame: local NED. Position arrays are `[north_m, east_m, down_m]`;
  velocity arrays are `[vn, ve, vd]`. Increasing MSL altitude maps to negative
  down.
- Body rates and gyroscope samples: `[roll_rps, pitch_rps, yaw_rps]` in radians
  per second.
- Attitude quaternion order: `[w, x, y, z]`. The identity/level attitude is
  `[1.0, 0.0, 0.0, 0.0]`.
- Rotation matrix convention: estimator quaternion helpers produce
  body-to-inertial rotation matrices.
- Attitude error convention: attitude control computes
  `target * inverse(current)` and flips the error quaternion to the shortest
  hemisphere when `w < 0`.
- Euler setpoint convention: `from_euler_rad(roll, pitch, yaw)` returns a
  `[w, x, y, z]` quaternion using roll, pitch, then yaw half-angle terms.
- Altitude convention: setpoint and estimate use NED down. If the vehicle is
  below the down setpoint, throttle increases; if it is above, throttle
  decreases.
- Position convention: positive north error commands nose-down pitch; positive
  east error commands right roll at zero yaw.
- Mixer geometry: `QuadXMixer` is logical quad-X with motor commands:
  - motor 1: `throttle + roll + pitch - yaw`
  - motor 2: `throttle - roll + pitch + yaw`
  - motor 3: `throttle - roll - pitch - yaw`
  - motor 4: `throttle + roll - pitch + yaw`
- Motor order convention: `MotorOrder::from_one_based([..])` maps logical
  motor 1 through 4 to one-based output channels. `[1, 2, 3, 4]` is identity.
- Motor output convention: normalized `[0.0, 1.0]` actuator commands. Mixer
  outputs report `clamped = true` when any command hits configured limits.
- Motor spin direction: not yet represented in `common`; Phase 5 must add an
  explicit `MotorGeometry` that names each motor position, output index, spin
  direction, thrust coefficient, reaction torque coefficient, and hover
  throttle before plant-response tests rely on yaw torque.

## Phase 0 Gate

Phase 0 is complete when these commands compile:

```sh
cargo test -p common --no-run
cargo test --manifest-path testing/deterministic-harness/Cargo.toml --no-run
```

## Phase 2 API Surface

The initial deterministic runner pieces are available from the crate root:

- `LockstepClock`: explicit `tick`, `sim_time_us`, and last-step `dt_s`.
- `TestCase`: case metadata with visible `random_seed` when a later test uses
  seeded randomness.
- `StepTrace`: per-step trace records with tick, simulated time, `dt_s`, label,
  and optional motor outputs.
- `HarnessReport`: in-memory report that carries the final failure tick and
  last trace without writing files.
- `Fixture`: deterministic CSV fixture loader whose first column must be
  `timestamp_us` and whose timestamps must be strictly monotonic.

Phase 2 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p00_clock
cargo test --manifest-path testing/deterministic-harness/Cargo.toml fixture
cargo test --manifest-path testing/deterministic-harness/Cargo.toml report_contains_failure_tick_and_last_trace
```

## Phase 3 Estimator Replay

Phase 3 adds deterministic sensor-frame replay into `common`'s
`LayeredNavigationStack` without invoking control, mixer, plant, UART, sockets,
or Gazebo.

Fixture columns are fixed for now:

```text
timestamp_us,
accel_x,accel_y,accel_z,
gyro_x,gyro_y,gyro_z,
mag_valid,mag_x,mag_y,mag_z,
gps_valid,gps_n,gps_e,gps_d,gps_vn,gps_ve,gps_vd,
baro_valid,baro_down
```

The first sample establishes time and is reported with `estimate_valid = false`;
subsequent strictly monotonic samples compute `dt_s` and update the estimator.
GPS and magnetometer groups are optional via their valid flags. Barometric
spikes can be bounded with `EstimatorReplayConfig::max_baro_step_m`; rejected
spikes are counted and skipped.

Phase 3 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p02_estimator_replay
cargo test -p common --test estimation_replay
```

## Phase 4 Pure Control Pipeline

Phase 4 adds `ControlPipeline`, a host-side adapter that feeds a truth
`EstimateSnapshot` directly into the position, altitude, attitude, rate, and
quad-X mixer controllers from `common`. It intentionally bypasses estimator
replay and plant physics.

`ControlPipelineTrace` records the required debug surface:

- input estimate: position, velocity, quaternion, and valid flag
- IMU accelerometer and gyro used by control
- attitude setpoint
- body-rate setpoint
- throttle and torque command
- normalized motor outputs and clamped flag

Phase 4 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p03_control_pipeline
```

## Phase 5 Open-Loop Plant

Phase 5 adds an explicit `MotorGeometry` and a minimal `OpenLoopPlant`.
This plant does not integrate attitude or position yet; it only exposes the
immediate response needed to validate motor order, thrust scaling, and torque
signs.

`MotorGeometry` names each motor's:

- one-based logical motor number
- zero-based output channel
- body-frame position
- spin direction
- thrust coefficient
- reaction torque coefficient

The default quad-X geometry matches the documented mixer basis:

- motor 1: front-right, clockwise
- motor 2: front-left, counter-clockwise
- motor 3: rear-left, clockwise
- motor 4: rear-right, counter-clockwise

Phase 5 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p04_motor_truth_table
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p04_open_loop_plant
```

## Phase 6 Closed-Loop Truth

Phase 6 adds a truth-state closed-loop runner. `ClosedLoopRunner` feeds the
plant's truth state directly into `ControlPipeline`, applies the resulting motor
outputs to `TruthPlant`, and records a per-tick `ClosedLoopTrace`.

The Phase 6 plant intentionally remains simple:

- vertical NED position and velocity are integrated from total thrust
- roll, pitch, and yaw rates are integrated from motor torques
- Euler angles are integrated from those body rates
- estimates are generated directly from truth state, bypassing the estimator

Phase 6 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p05_closed_loop_truth
```

## Phase 7 Closed-Loop Estimator

Phase 7 adds `ClosedLoopEstimatorRunner`, which feeds truth-derived simulated
sensor frames into the real estimator, converts the estimator output into the
pure `ControlPipeline`, applies motor outputs to `TruthPlant`, and records a
per-tick `ClosedLoopEstimatorTrace`.

The Phase 7 runner records:

- truth state after each plant step
- simulated sensor frame used by the estimator
- estimator trace, including validity, GPS use, baro use, and baro rejection
- control trace and normalized motor outputs

`ClosedLoopEstimatorReport::trace_csv()` emits the stable truth-vs-estimate CSV
surface required for Phase 7 evidence.

Phase 7 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p06_closed_loop_estimator
```

## Phase 8 HIL Semantics

Phase 8 adds a deterministic HIL semantic adapter. `HilSemanticAdapter` converts
harness `SensorFrame` values into MARV `HilSensorFrame` wire-domain structs,
drives `common::services::hil::HilSensorFrameAdapter`, records raw frames and
responses in memory, and builds `HilResponseFrame` values from estimator/control
traces.

The Phase 8 adapter verifies the firmware boundary used by `RP235X-TEST`
without creating sockets, serial ports, Gazebo dependencies, or a new device
crate.

Phase 8 checks cover:

- duplicate tick rejection
- out-of-order timestamp rejection
- partial valid flags publishing only asserted sensor groups
- non-finite asserted sample rejection
- response correlation with the latest accepted simulator stamp
- stale estimate failsafe with valid zero motor output

Phase 8 gate:

```sh
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p07_hil_semantics
```

## Phase 9 Gazebo Bridge Contracts

Phase 9 adds deterministic Gazebo bridge contract checks before any Gazebo
runtime tests are enabled. The harness parses known newline `SENSOR` frames,
maps them to MARV `HilSensorFrame` values, parses the checked-in Gazebo bridge
config, and validates actuator scaling from normalized motor commands to Gazebo
rotor speeds.

The Tauri backend also exposes pure `gazebo_bridge_client` parser/formatter
helpers so the Rust client protocol can be tested without sockets or a running
bridge process.

Phase 9 checks cover:

- known `SENSOR` line fields map exactly to MARV HIL sensor fields
- HIL actuator commands format to the bridge `ACTUATOR` protocol
- normalized motor outputs map to configured Gazebo rad/s speeds
- zero, hover, max, and clamped motor-speed limits
- motor direction is explicitly delegated to the Gazebo model
- checked-in topic config matches the MARV `marv_field` / `marv_f450` map

Phase 9 gate:

```sh
cargo test -p telemetry-app gazebo_bridge_client
cargo test --manifest-path testing/deterministic-harness/Cargo.toml p08_gazebo_contract
```
