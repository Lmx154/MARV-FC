# MARV Deterministic Harness

This crate is the host-side home for MARV GNC validation work.

Active GNC testing work is now controlled by
[`gnc-testing-reset-plan.html`](./gnc-testing-reset-plan.html). That plan is the
single source of truth for cleanup, consolidation, and the rebuilt proof gates.

The previous phase-numbered harness growth narrative is no longer active
guidance. Phase 2 removed the old `p##_*.rs` tests, synthetic plant/replay
helpers, and canned CSV fixtures. Phase 3 rebuilt the harvested contract suite
and added the first shared scenario/report support surface. Phase 4 restored
the minimal G0/G1/G2 gate files as ignored manual runtime gates with concrete
airframe, control, HIL, and report-shape assertions. Phase 4 now also drives
the live Gazebo bridge for G0 actuator truth, including reset/play commands,
actuator commands, `SENSOR` collection, and measured gyro scoring against the
hardware airframe profile. Phase 5 added the G2.5 motion primitive gate as
direct local-trajectory primitives before navigation.
The harvested assertions and rebuilt gate intent are recorded in
[`VALIDATION_MATRIX.md`](./VALIDATION_MATRIX.md).

## Current Cleanup State

- G0/G1/G2/G2.5 runtime gate files now exist as ignored manual gates.
- G0 actuator truth now has a live bridge path that connects to
  `MARV_GAZEBO_ENDPOINT` or `127.0.0.1:9000`, resets/plays Gazebo, sends
  actuator cases, and scores measured `SENSOR` gyro evidence.
- Treat `config/airframes/f450_xing2_2809_1045_4s_v0.cfg` as the hardware
  source of truth. If live G0 disagrees with it, update Gazebo model, motor
  ordering, spin direction, or bridge mapping until the simulator matches the
  profile.
- G1/G2/G2.5 still need live runner wiring beyond their production
  control/HIL/spec assertions.
- Do not add placeholder tests or future empty gate files.
- Do not keep archive, legacy, disabled, old, or temporary copies of deleted
  tests.
- Do not loosen thresholds to push a later gate green while a lower gate is red.
- Do not add or enable G3 navigation acceptance until G2.5 motion primitives
  prove movement, braking, yaw turns, zig-zags, recovery, and estimator
  agreement in the runtime path.
- Treat `VALIDATION_MATRIX.md` as the harvest record for the removed phase files.
- Keep shared runtime scaffolding in `tests/support/mod.rs` until reuse across
  multiple active gates justifies moving it into `src/`.

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
- Motor spin direction: represented by harness `MotorGeometry` only where a
  current cleanup input still needs it. Future acceptance gates should keep
  geometry tied to the active Gazebo airframe config instead of synthetic plant
  assumptions.

## Phase 0 Gate

Phase 0 is complete when:

- `gnc-testing-reset-plan.html` is present in this directory.
- This README points to that plan and no longer presents historical
  phase-numbered files as the active workflow.
- `VALIDATION_MATRIX.md` gives every current test file a planned fate.
- No new GNC tests were added.

Compile check:

```sh
cargo test -p common --no-run
cargo test --manifest-path testing/deterministic-harness/Cargo.toml --no-run
```

## Phase 2 Gate

Phase 2 is complete when:

- No `p##_*.rs` files remain under `testing/deterministic-harness/tests/`.
- No obsolete test archive directory exists.
- The old CSV fixture files and synthetic plant/replay helpers are gone.
- The harness source tree exports only helpers used by surviving or immediately
  rebuilt tests.

Current active source surface:

```text
src/
  lib.rs
  gazebo_contract.rs
  hil_semantics.rs
```

## Phase 3 Gate

Phase 3 is complete when:

- `tests/contracts.rs` contains only harvested C0 contracts from the validation
  matrix.
- `tests/support/mod.rs` contains the shared gate, schedule, threshold, and
  report shape used by the contracts and future runtime gates.
- No G0/G1/G2/G2.5/G3 runtime gate files exist yet.
- The harness has one parser/config seam, one HIL validity seam, and one report
  shape for later gates.

Active test surface after Phase 4:

```text
tests/
  contracts.rs
  support/
    mod.rs
```

## Phase 5 Gate

Phase 5 is complete when:

- `tests/gazebo_g25_motion_primitives.rs` exists and covers M01 through M08:
  cardinal motion, speed profile, brake-to-hover, slow yaw, fast yaw, moving
  heading changes, zig-zag, and abrupt reversal recovery.
- G2.5 uses direct estimator-local trajectory primitives, not mission waypoint
  or navigation state-machine acceptance.
- Motion primitive assertions preserve world-frame NED velocity/acceleration
  intent across yaw changes, verify roll/pitch/yaw signs, keep altitude and
  motor authority visible, and classify failures through the shared report
  shape.
- G3 navigation acceptance remains absent and blocked.

Current active test surface:

```text
tests/
  contracts.rs
  gazebo_g0_actuator_truth.rs
  gazebo_g1_control_primitives.rs
  gazebo_g2_takeoff_hover_land.rs
  gazebo_g25_motion_primitives.rs
  support/
    mod.rs
```

## Phase 4 Gate

Phase 4 is complete when:

- `tests/contracts.rs` remains fast, deterministic, and default-on.
- `tests/gazebo_g0_actuator_truth.rs` covers reset/hover motor truth case
  generation, actuator formatting, runtime airframe motor signs, the shared
  report shape, and the live Gazebo TCP bridge actuator-truth loop.
- `tests/gazebo_g1_control_primitives.rs` covers estimator-bypass hover,
  roll/pitch/yaw recovery, vertical steps, runtime motor geometry, and the
  shared report shape.
- `tests/gazebo_g2_takeoff_hover_land.rs` covers the HIL sensor handoff,
  estimator/control response flags, disarm-to-zero behavior, NED altitude sign,
  and the shared report shape.
- G0/G1/G2 tests are `#[ignore]` manual runtime gates; G2.5 and G3 files do not
  exist yet.

Current active test surface:

```text
tests/
  contracts.rs
  gazebo_g0_actuator_truth.rs
  gazebo_g1_control_primitives.rs
  gazebo_g2_takeoff_hover_land.rs
  support/
    mod.rs
```
