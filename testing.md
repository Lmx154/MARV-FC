# MARV-FC Software Test Plan

This plan is for building out tests that can run without flight hardware first, then adding SITL coverage where simulated sensor input and actuator output can validate larger behavior. The current repository already has a good start in `common` unit tests and Tauri backend tests, but the workspace needs a small cleanup before the full test suite can run consistently.

## Current State

- `testing.md` was empty before this plan.
- Host-friendly workspace members are intended to be:
  - `common`
  - `telemetry-app/src-tauri`
- Existing Rust unit tests are already present in:
  - `common/src/protocol/*`
  - `common/src/control/*`
  - `common/src/policies/*`
  - `common/src/services/*`
  - `common/src/drivers/*` where logic is pure enough to test on host
  - `telemetry-app/src-tauri/src/bridge/*`
  - `telemetry-app/src-tauri/src/backend/hilink_parser_service.rs`
- `common/src/lib.rs` supports host tests by using `#![cfg_attr(not(test), no_std)]` plus a minimal test logger for `defmt`.
- `docs/simulator.md` documents a `simulator` crate and `docs/FC_SITL_ICD.md`, but neither exists in the current checkout.
- `Cargo.toml` references `device/RP2354B-CROSSCORE-TESTS`, but that directory has no `Cargo.toml`.
- Because of the missing workspace member, `cargo test --workspace --no-run` currently fails before test compilation.

## Test Goals

1. Make host-side tests easy to run with one command.
2. Protect protocol, estimation, control, policy, logging, and telemetry behavior with deterministic software tests.
3. Separate pure logic tests from hardware-bound device tests.
4. Add a SITL path that replays simulated sensor frames and verifies accepted samples, logs, state transitions, and actuator outputs.
5. Keep test fixtures small, versioned, and readable enough to debug flight issues later.

## Phase 0: Restore the Testable Workspace

These tasks unblock every later phase.

- Decide what to do with missing workspace entries:
  - Either restore `device/RP2354B-CROSSCORE-TESTS/Cargo.toml`, or remove it from the workspace until it exists.
  - Either restore the documented `simulator` crate, or update `docs/simulator.md` to match the current Gazebo/Tauri bridge architecture.
- Add explicit workspace commands to the README or this file:
  - `cargo test -p common`
  - `cargo test -p telemetry-app`
  - `cargo test --workspace --exclude <device-only-crates>` once the workspace is clean.
- Confirm all host tests compile:
  - `cargo test -p common --no-run`
  - `cargo test -p telemetry-app --no-run`
- Add CI later only after the local commands are stable.

Acceptance criteria:

- `cargo test -p common` runs on the host.
- `cargo test -p telemetry-app` runs on the host.
- A workspace-level host test command exists and is documented.

## Phase 1: Expand Pure Software Unit Tests

Start with modules that are deterministic and already mostly hardware-independent.

### Protocol Tests

Targets:

- `common/src/protocol/crc`
- `common/src/protocol/framing`
- `common/src/protocol/mavlink`
- `common/src/protocol/ubx`
- `common/src/protocol/hilink`
- `common/src/comms/links/lora`

Tests to add:

- Encode/decode round trips for every packet type used by firmware and telemetry.
- CRC known-answer tests from fixed byte fixtures.
- Truncated frame handling.
- Corrupt checksum rejection.
- Unknown packet type behavior.
- Boundary payload lengths.
- Duplicate, stale, and out-of-order frame handling where applicable.

Preferred style:

- Keep binary fixtures in small byte arrays inside test modules first.
- Move larger fixtures to `common/tests/fixtures/` once they become hard to read inline.

### Control Tests

Targets:

- `common/src/control/rate`
- `common/src/control/attitude`
- `common/src/control/altitude`
- `common/src/control/position`
- `common/src/control/mixing`
- `common/src/control/outputs`
- `common/src/control/guidance`

Tests to add:

- Controller output for zero error.
- Saturation and clamping behavior.
- Mixer output normalization.
- Failsafe output behavior.
- NaN or invalid input handling if the module accepts floats from external sources.
- Regression tests for representative flight modes.

### Localization and Estimation Tests

Targets:

- `common/src/localization/estimation/*`
- `common/src/localization/attitude`
- `common/src/localization/navigation`
- `common/src/localization/sensor_fusion`
- `common/src/localization/calibration`

Tests to add:

- Quaternion normalization and rotation invariants.
- Static IMU attitude convergence using fixed synthetic samples.
- Barometer altitude conversion known-answer tests.
- GPS position and velocity measurement gating.
- EKF or ESKF predict/update sanity checks:
  - covariance remains finite
  - state does not drift under zero-motion input
  - bad measurements are rejected by gating policy
- Golden replay tests from short synthetic CSV logs.

### Policy and Health Tests

Targets:

- `common/src/policies/failsafe`
- `common/src/policies/arming`
- `common/src/policies/mission`
- `common/src/policies/modes`
- `common/src/services/health`

Tests to add:

- Arming allowed and denied cases.
- Mission mode transition table tests.
- Fault escalation thresholds.
- Watchdog supervisor behavior when tasks miss deadlines.
- Liveness reporting for healthy, stale, and missing tasks.
- Reset policy behavior for repeated failures.

### Logging Tests

Targets:

- `common/src/services/logging`
- `common/src/drivers/storage/microsd`
- `common/src/interfaces/storage`

Tests to add:

- Flight log filename incrementing.
- CSV schema stability.
- Sensor snapshot row formatting.
- Missing sensor sample fields.
- Queue overflow behavior.
- Storage error propagation.

## Phase 2: Add Host Integration Tests

Host integration tests should exercise several `common` modules together without requiring hardware.

Suggested structure:

- `common/tests/protocol_roundtrip.rs`
- `common/tests/control_pipeline.rs`
- `common/tests/estimation_replay.rs`
- `common/tests/health_faults.rs`
- `common/tests/logging_snapshot.rs`

Integration test scenarios:

- MAVLink or HiLink bytes -> parser -> typed message -> telemetry event.
- Synthetic sensor samples -> acquisition service -> logger row.
- IMU/baro/GPS sequence -> estimator update -> bounded state output.
- Fault events -> aggregation -> failsafe decision.
- Control command -> mixer -> actuator output limits.

Acceptance criteria:

- Tests use deterministic fixtures and do not depend on wall-clock timing.
- Any async code uses a controlled executor, not sleeps.
- Tests document the system behavior they protect.

## Phase 3: Telemetry App Software Tests

The Tauri backend already has Rust tests. The React frontend currently has build tooling but no dedicated test runner.

Backend targets:

- `telemetry-app/src-tauri/src/backend/hilink_parser_service.rs`
- `telemetry-app/src-tauri/src/bridge/app_bridge.rs`
- `telemetry-app/src-tauri/src/bridge/gazebo_bridge_client.rs`
- `telemetry-app/src-tauri/src/commands/*`

Backend tests to add:

- Parser fixtures for all supported telemetry frame types.
- Bridge reconnect behavior.
- Malformed line handling from the Gazebo bridge.
- Command validation for serial, UART, Gazebo, and HiLink commands.

Frontend targets:

- `telemetry-app/src/utils/format.ts`
- `telemetry-app/src/utils/telemetry.ts`
- key stateful views such as `Dashboard`, `HilView`, and `DiagnosticsView`

Frontend test setup:

- Add Vitest and React Testing Library when ready.
- Add scripts:
  - `pnpm --dir telemetry-app test`
  - `pnpm --dir telemetry-app test:run`

Frontend tests to add:

- Formatting helpers.
- Telemetry mapping helpers.
- Rendering of disconnected, stale, and live states.
- Command history behavior.
- HIL controls state changes.

Acceptance criteria:

- `pnpm --dir telemetry-app build` remains the minimum frontend gate.
- Once test tooling is added, `pnpm --dir telemetry-app test:run` becomes the frontend software test gate.

## Phase 4: SITL Test Harness

SITL tests should validate the same code paths used by the host-side runtime or bridge, without requiring the physical flight controller.

Current repo note:

- `docs/simulator.md` describes a Rust `simulator` crate that consumes MAVLink `SYSTEM_TIME`, `HIL_SENSOR`, and `HIL_GPS`.
- The current checkout does not contain that crate.
- `telemetry-app/gazebo_bridge` is present and provides a Gazebo-to-backend bridge over newline-delimited text frames.

Decision needed:

- Restore the Rust `simulator` crate and test against the documented MAVLink SITL interface, or
- Treat the Tauri backend plus `telemetry-app/gazebo_bridge` as the active SITL path and update simulator docs accordingly.

Recommended path:

1. Restore or create a small host-side SITL harness crate.
2. Keep it separate from device crates.
3. Reuse `common` parsers, acquisition channels, logging, estimator, control, and actuator encoding.
4. Make the harness step-driven so tests control simulated time.

SITL test scenarios:

- Boot with no simulator input and report not-ready or stale state.
- Accept monotonic `SYSTEM_TIME` ticks.
- Ignore duplicate or stale ticks.
- Accept `HIL_SENSOR` without GPS.
- Accept `HIL_GPS` only when fix quality is valid.
- Write a sensor snapshot row for each accepted tick.
- Run estimator over a short hover/static dataset.
- Run controller and mixer over a simple command profile.
- Emit actuator commands once output support is wired.

Suggested fixtures:

- `tests/fixtures/sitl/static_on_pad.csv`
- `tests/fixtures/sitl/vertical_ascent.csv`
- `tests/fixtures/sitl/gps_dropout.csv`
- `tests/fixtures/sitl/baro_spike.csv`
- `tests/fixtures/sitl/imu_saturation.csv`

Acceptance criteria:

- SITL tests run without Gazebo for quick CI.
- Gazebo bridge tests are optional and can be marked ignored or placed behind a feature flag.
- SITL logs are written to a temporary directory and removed after each test.
- The harness verifies outputs, not just lack of panics.

## Phase 5: Hardware-Adjacent Tests

These tests should be kept out of the normal host test command unless they are fully simulated.

Targets:

- `device/RP235X-TEST`
- `device/RP2354B-CROSSCORE-TESTS` if restored
- board-specific SPI, UART, radio, storage, LED, and watchdog modules

Tests to add:

- Compile-only checks for each device crate.
- Board pinmap consistency checks where possible.
- Fake bus tests for device drivers that can accept embedded-hal mocks.
- Probe-run or hardware-in-loop smoke tests later, documented separately.

Acceptance criteria:

- Host tests never require attached hardware.
- Hardware-required tests have explicit commands and prerequisites.
- Device compile checks are separated from behavioral tests.

## Phase 6: CI Gates

Start with gates that are deterministic on a clean host.

Recommended initial gates:

```bash
cargo fmt --all -- --check
cargo test -p common
cargo test -p telemetry-app
pnpm --dir telemetry-app build
cmake -S telemetry-app/gazebo_bridge -B telemetry-app/gazebo_bridge/build
cmake --build telemetry-app/gazebo_bridge/build
```

Later gates:

```bash
cargo test --workspace --exclude <hardware-only-crates>
pnpm --dir telemetry-app test:run
cargo test -p sitl-harness
```

Device compile checks can be added once target/toolchain expectations are documented.

## Test Priorities

Immediate:

- Fix missing workspace members so host tests can run.
- Run and stabilize `cargo test -p common`.
- Run and stabilize `cargo test -p telemetry-app`.
- Add integration tests around protocol, logging, health, and control behavior.

Next:

- Restore or replace the documented SITL crate.
- Add deterministic SITL replay fixtures.
- Connect estimator/control/mixer into the SITL path as those runtime pieces become available.

Later:

- Add frontend unit/component tests.
- Add Gazebo bridge integration tests.
- Add hardware-in-loop smoke tests.

## Definition of Done for a New Test Area

A test area is ready when:

- It has a documented command.
- It runs without hardware unless explicitly marked hardware-required.
- It has at least one happy-path test and one failure-path test.
- It checks actual outputs or state transitions.
- It avoids sleeps, wall-clock dependence, and external network access.
- It can be run repeatedly without leaving generated files in the repo.
