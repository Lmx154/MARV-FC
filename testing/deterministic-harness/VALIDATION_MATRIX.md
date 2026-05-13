# GNC Validation Matrix

Active plan: [gnc-testing-reset-plan.html](./gnc-testing-reset-plan.html)

This matrix records the planned fate of current deterministic-harness tests
during the cleanup-first GNC reset. It is not a license to keep old phase files
wholesale. Harvested assertions should move only into compact contract tests or
rebuilt gate tests named by the active plan.

## Phase 1 Harvest Shortlist

Carry these assertions forward. Everything else in the old phase files is either
deleted, rebuilt under a runtime gate, or treated as diagnostic scaffolding.

| contract id | source | replacement gate | exact assertion to keep |
| --- | --- | --- | --- |
| `C0-FRAME-NED` | `tests/p00_phase0.rs` | `tests/contracts.rs` | Positive latitude maps to north, positive longitude maps to east, and higher MSL altitude maps to negative NED down through `LocalNedFrame`. |
| `C0-QUAT-IDENTITY` | `tests/p00_phase0.rs` | `tests/contracts.rs` | Level attitude quaternion order is `[w, x, y, z]` and `AttitudeSetpoint::LEVEL.quaternion == [1.0, 0.0, 0.0, 0.0]`. |
| `C0-MIXER-BASIS` | `tests/p00_phase0.rs`, `tests/p04_motor_truth_table.rs` | `tests/contracts.rs` | Identity motor order and quad-X basis map `TorqueCommand::new(0.1, 0.2, 0.05, 0.5)` to `[0.75, 0.65, 0.15, 0.45]`; physical allocator matches the production mixer for roll, pitch, and yaw basis commands. |
| `C0-FAILSAFE-ZERO` | `tests/p09_portable_control.rs`, `tests/p03_control_pipeline.rs`, `tests/p14_g2_demand_limits.rs`, `tests/p17_navigation_steps.rs` | `tests/contracts.rs` | Invalid estimate, missing IMU, non-finite setpoint/input, and disarmed commands produce zero motors, invalid control, no attitude/rate output, and no clamp flag unless the command is armed-but-invalid. |
| `C0-NED-ALTITUDE-SIGN` | `tests/p03_control_pipeline.rs` | `tests/contracts.rs` | Below a negative-down altitude setpoint increases throttle; being above the setpoint decreases throttle. |
| `C0-CONTROL-OPPOSES-RATES` | `tests/p03_control_pipeline.rs`, `tests/p12_gazebo_control_truth.rs` | `tests/contracts.rs` or `tests/gazebo_g1_control_primitives.rs` | Positive roll and pitch attitude errors command opposing rate/torque; positive measured yaw rate commands negative yaw torque; runtime motor geometry closes those signs in the measured Gazebo motor basis. |
| `C0-LIMIT-VISIBILITY` | `tests/p03_control_pipeline.rs`, `tests/p04_motor_truth_table.rs`, `tests/p14_g2_demand_limits.rs` | `tests/contracts.rs` | Saturation sets `clamped`, reports mixer roll/pitch and yaw limit flags, preserves desaturation priority, exposes rate-axis limit flags before mixing, and keeps clamped motors inside configured limits. |
| `C0-DEMAND-LIMITS` | `tests/p14_g2_demand_limits.rs` | `tests/contracts.rs` | Extreme position/altitude demands stay control-valid while respecting max tilt, horizontal acceleration, throttle correction, axis command, and normalized motor bounds. |
| `C0-TILT-COLLECTIVE` | `tests/p14_g2_demand_limits.rs` | `tests/contracts.rs` | Lateral acceleration/tilt demand increases collective above hover, records tilt compensation, and preserves positive vertical throttle margin. |
| `C0-TRAJECTORY-TAGGING` | `tests/p03_control_pipeline.rs`, `tests/p17_navigation_steps.rs` | `tests/contracts.rs` | Local trajectory setpoints preserve bounded velocity and acceleration feed-forward, command correct north/east roll/pitch signs, and keep setpoint source tagged as estimator-local trajectory rather than truth evidence. |
| `C0-HIL-SEMANTICS` | `tests/p07_hil_semantics.rs` | `tests/contracts.rs` | HIL duplicate ticks, out-of-order ticks, non-finite asserted groups, partial valid flags, response correlation, and stale-estimate failsafe response semantics remain explicit. |
| `C0-GAZEBO-BRIDGE` | `tests/p08_gazebo_contract.rs`, `tests/p11_gazebo_actuator_truth_table.rs` | `tests/contracts.rs` | Gazebo `SENSOR`, `ACTUATOR`, and `SIM_CONTROL` parsing/formatting, valid flag bits, checked-in topic config, airframe profile constants, actuator scaling, and G0 oracle case generation match the checked-in bridge and airframe files. |
| `G0-ACTUATOR-TRUTH` | `tests/p11_gazebo_actuator_truth_table.rs` | `tests/gazebo_g0_actuator_truth.rs` | Live reset-clean actuator cases must prove zero, hover, single-motor bumps, and roll/pitch/yaw basis signs against Gazebo-clocked sensor evidence. |
| `G1-CONTROL-PRIMITIVES` | `tests/p12_gazebo_control_truth.rs` | `tests/gazebo_g1_control_primitives.rs` | Estimator-bypass origin hold and vertical step must bound attitude, altitude error, clamp ratio, motor spread, and final vertical improvement through the shared runner/report path. |
| `G2-TAKEOFF-HOVER-LAND` | `tests/p13_gazebo_estimator_loop.rs`, `tests/p15_takeoff_hover_land.rs` | `tests/gazebo_g2_takeoff_hover_land.rs` | Estimator-in-loop warmup, origin hold, vertical step, staged takeoff, hover, landing contact, post-contact spooldown, and final disarmed zero-output remain required G2 evidence. |
| `G25-MOTION-PRIMITIVES` | `tests/p13_gazebo_estimator_loop.rs`, `tests/p14_g2_control_hardening.rs`, `tests/p14_g2_demand_limits.rs` | `tests/gazebo_g25_motion_primitives.rs` | Yaw disturbance, lateral pulse, bounded return-to-origin, demand-limit pulse, altitude ladder, and estimator/truth agreement become G2.5 motion-primitive cases only after Phase 3 shared reporting exists. |
| `G3-NAV-BLOCKED` | `tests/p17_navigation_steps.rs`, `tests/p13_gazebo_estimator_loop.rs` | `tests/gazebo_g3_navigation_acceptance.rs` | Navigation north/east/diagonal/yawed/square acceptance intent is retained, but G3 remains blocked until G2.5 proves movement, braking, turning, recovery, and estimator agreement. |

## Test File Disposition

| file | requirement | production seam | replacement gate | action | harvested assertion |
| --- | --- | --- | --- | --- | --- |
| `tests/p00_clock.rs` | Deterministic runner time is monotonic. | Harness clock only. | None, unless Phase 3 shared runner needs a small lockstep-time contract. | Delete. | No Phase 1 harvest. |
| `tests/p00_fixture.rs` | CSV fixtures reject malformed or non-monotonic input. | Fixture loader only. | None. | Delete. | No Phase 1 harvest. |
| `tests/p00_phase0.rs` | Frozen frame, quaternion, sign, altitude, position, mixer, and motor-order conventions. | `common` frame and mixer conventions. | `tests/contracts.rs`. | Harvest then delete. | `C0-FRAME-NED`, `C0-QUAT-IDENTITY`, `C0-MIXER-BASIS`. |
| `tests/p00_report.rs` | Harness reports carry failure tick and trace context. | Harness report object only. | Phase 3 shared report path only if reused by live gates. | Delete. | No Phase 1 harvest. |
| `tests/p02_estimator_replay.rs` | Canned sensor frames replay through estimator without non-finite output. | Estimator replay adapter and fixture path. | Future real-log regression only. | Delete. | No Phase 1 harvest; broad replay finiteness is not actionable enough. |
| `tests/p03_control_pipeline.rs` | Control pipeline maps estimates, setpoints, IMU, and mixer output consistently. | `common` controller/mixer flow through harness adapter. | `tests/contracts.rs` and later G1/G2 gates. | Harvest then delete. | `C0-FAILSAFE-ZERO`, `C0-NED-ALTITUDE-SIGN`, `C0-CONTROL-OPPOSES-RATES`, `C0-LIMIT-VISIBILITY`, `C0-TRAJECTORY-TAGGING`. |
| `tests/p04_motor_truth_table.rs` | Motor geometry, order, spin direction, thrust, and torque signs are explicit. | Allocator geometry and actuator mapping. | `tests/contracts.rs`; Gazebo G0 for physical truth. | Harvest then delete. | `C0-MIXER-BASIS`, `C0-LIMIT-VISIBILITY`; do not keep `OpenLoopPlant` response assertions. |
| `tests/p04_open_loop_plant.rs` | Synthetic open-loop plant responds to motor commands. | Harness plant model only. | None. | Delete. | No Phase 1 harvest. |
| `tests/p05_closed_loop_truth.rs` | Synthetic truth loop can take off and respond to setpoints. | Harness closed-loop plant bypassing estimator. | Gazebo G1/G2/G2.5. | Delete. | No Phase 1 harvest. |
| `tests/p06_closed_loop_estimator.rs` | Synthetic plant and estimator loop stay finite. | Harness estimator/control surrogate. | Gazebo G2/G2.5. | Delete. | No Phase 1 harvest. |
| `tests/p07_hil_semantics.rs` | HIL duplicate, out-of-order, validity, non-finite, and response correlation behavior. | `common::services::hil` semantic boundary. | `tests/contracts.rs`. | Harvest then delete. | `C0-HIL-SEMANTICS`; rebuild without closed-loop synthetic runner setup. |
| `tests/p08_gazebo_contract.rs` | Gazebo bridge parsing, formatting, config, actuator scaling, and topic map. | Gazebo bridge protocol/config seam. | `tests/contracts.rs`; keep trimmed source helper. | Harvest then delete. | `C0-GAZEBO-BRIDGE`, including truth/estimator adapter conversion and gravity removal contracts. |
| `tests/p09_portable_control.rs` | Portable control fails safe for invalid, stale, missing, non-finite, or disarmed inputs. | Production control failsafe path. | `tests/contracts.rs`. | Harvest then delete. | `C0-FAILSAFE-ZERO`. |
| `tests/p10_control_envelope.rs` | Synthetic control envelope remains within configured bounds. | Harness envelope sweep. | G1/G2/G2.5 scenario thresholds. | Delete. | No Phase 1 harvest. |
| `tests/p10_gain_sweep.rs` | Gain sweeps reveal control margin changes. | Diagnostic sweep only. | None. | Delete. | No Phase 1 harvest. |
| `tests/p11_gazebo_actuator_truth_table.rs` | Live Gazebo motor commands produce expected physical actuator truth. | Gazebo bridge, model, and actuator physics. | `tests/gazebo_g0_actuator_truth.rs`. | Rebuild as G0. | `C0-GAZEBO-BRIDGE`, `G0-ACTUATOR-TRUTH`; keep selected-case aliases only if the replay helper survives. |
| `tests/p12_gazebo_control_truth.rs` | Estimator-bypass control can command controlled plant behavior in Gazebo. | Production controller/mixer/bridge with truth estimate. | `tests/gazebo_g1_control_primitives.rs`. | Rebuild as G1. | `C0-CONTROL-OPPOSES-RATES`, `G1-CONTROL-PRIMITIVES`; socket/reset/report code must move to shared support. |
| `tests/p13_gazebo_estimator_loop.rs` | Estimator warmup and estimator-in-loop behavior stay bounded before navigation. | Live Gazebo estimator/control path. | `tests/gazebo_g2_takeoff_hover_land.rs`, `tests/gazebo_g25_motion_primitives.rs`, and later G3. | Harvest then delete or rebuild as gate support. | `G2-TAKEOFF-HOVER-LAND`, `G25-MOTION-PRIMITIVES`, `G3-NAV-BLOCKED`; split the mixed G2/P14/P15/P17 bundle into gate-specific files. |
| `tests/p14_g2_control_hardening.rs` | G2 hardening cases cover yaw, disturbance, margins, saturation, and failsafe behavior. | Mixed host synthetic and live hardening paths. | G1/G2/G2.5 cases only where tied to a production seam. | Delete. | `G25-MOTION-PRIMITIVES` intent only; no wholesale host synthetic sweep survives. |
| `tests/p14_g2_demand_limits.rs` | Demand limiting keeps setpoints and actuator demands bounded. | Control demand-limit contract. | `tests/contracts.rs` or G2.5 thresholds. | Harvest then delete. | `C0-DEMAND-LIMITS`, `C0-TILT-COLLECTIVE`, `C0-LIMIT-VISIBILITY`, `C0-FAILSAFE-ZERO`. |
| `tests/p15_takeoff_hover_land.rs` | Takeoff, hover, and landing behavior are bounded. | Required estimator-in-loop flight gate. | `tests/gazebo_g2_takeoff_hover_land.rs`. | Rebuild as G2. | `G2-TAKEOFF-HOVER-LAND`; synthetic host plant version does not survive. |
| `tests/p17_navigation_steps.rs` | Navigation progresses through waypoints. | Full navigation acceptance. | `tests/gazebo_g3_navigation_acceptance.rs` after G2.5. | Block then rebuild. | `C0-TRAJECTORY-TAGGING`, `C0-FAILSAFE-ZERO`, `G3-NAV-BLOCKED`; host synthetic navigation dynamics do not survive as acceptance evidence. |

## Harness Source Cleanup Notes

| file | requirement | production seam | replacement gate | action | harvested assertion |
| --- | --- | --- | --- | --- | --- |
| `src/closed_loop.rs` | Synthetic closed-loop truth plant. | Harness-only plant. | None. | Deleted in Phase 2. | No Phase 1 harvest. |
| `src/closed_loop_estimator.rs` | Synthetic estimator/control loop. | Harness-only surrogate. | None. | Deleted in Phase 2. | No Phase 1 harvest. |
| `src/plant.rs` | Open-loop plant approximation. | Harness-only plant. | None. | Deleted in Phase 2. | No Phase 1 harvest. |
| `src/fixtures.rs` | CSV fixture loading. | Harness fixture path. | Future real-log regression only. | Deleted in Phase 2. | No Phase 1 harvest. |
| `src/estimator_replay.rs` | Deterministic estimator replay adapter. | Harness replay path. | Future real-log regression only. | Deleted in Phase 2. | No Phase 1 harvest by default; estimator sensor conversion contracts are harvested from Gazebo bridge tests instead. |
| `src/cases.rs` | Historical scenario case tables. | Harness scenario scaffolding. | New gate-local case tables or shared support only after reuse is proven. | Deleted in Phase 2. | No Phase 1 harvest. |
| `src/clock.rs` | Lockstep clock. | Harness runner timing. | Phase 3 shared runner if needed. | Deleted in Phase 2. | No Phase 1 harvest. Recreate a small contract only if Phase 3 proves it is required. |
| `src/control_pipeline.rs` | Harness adapter around common control. | Production control/mixer path via adapter. | Prefer direct `common` imports plus thin non-drifting glue. | Deleted in Phase 2. | Future contracts should import production `common` control types directly where possible. |
| `src/gazebo_contract.rs` | Gazebo parser, formatter, config, and scaling helpers. | Gazebo bridge protocol/config seam. | `tests/contracts.rs` and live gate support. | Keep and trim. | Keep helpers needed by `C0-GAZEBO-BRIDGE`, `G0-ACTUATOR-TRUTH`, and shared Gazebo support. |
| `src/hil_semantics.rs` | HIL semantic adapter helpers. | HIL validity/rejection/response seam. | `tests/contracts.rs`. | Keep and trim. | Keep helpers needed by `C0-HIL-SEMANTICS`; remove synthetic closed-loop dependencies where possible. |
| `src/lib.rs` | Harness public exports. | Active test helper surface. | All active gates. | Keep minimal. | Export only helpers used by the shortlist above or rebuilt gates. |
| `src/bin/gazebo_replay.rs` | Replay selected Gazebo gates. | Developer runtime helper. | Phase 3 shared runner/report workflow. | Deleted in Phase 2. | Recreate only after active gate names and shared reset/report rules exist. |
