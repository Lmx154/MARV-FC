//! Guidance state helpers for estimator-local takeoff and navigation handoff.

use crate::control::pipeline::{EstimatorLocalState, LocalTrajectorySetpoint, MissionWaypoint};

const DEFAULT_TRAJECTORY_SPEED_MPS: f32 = 0.45;
const DEFAULT_TRAJECTORY_ACCEL_MPS2: f32 = 0.75;
const DEFAULT_TRAJECTORY_JERK_MPS3: f32 = 3.0;
const DEFAULT_ACCEPTANCE_RADIUS_M: f32 = 0.08;
const DEFAULT_YAW_RATE_RAD_S: f32 = 45.0 * core::f32::consts::PI / 180.0;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum GuidancePhase {
    #[default]
    Idle,
    Takeoff,
    Hold,
    NavigateLeg,
    Loiter,
    Land,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GuidanceCommand {
    Idle,
    Takeoff(LocalTrajectorySetpoint),
    Hold(LocalTrajectorySetpoint),
    NavigateLeg(MissionWaypoint),
    Loiter(MissionWaypoint),
    Land(MissionWaypoint),
}

impl GuidanceCommand {
    pub const fn phase(self) -> GuidancePhase {
        match self {
            Self::Idle => GuidancePhase::Idle,
            Self::Takeoff(_) => GuidancePhase::Takeoff,
            Self::Hold(_) => GuidancePhase::Hold,
            Self::NavigateLeg(_) => GuidancePhase::NavigateLeg,
            Self::Loiter(_) => GuidancePhase::Loiter,
            Self::Land(_) => GuidancePhase::Land,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrajectoryLimits {
    pub max_velocity_mps: f32,
    pub max_accel_mps2: f32,
    pub max_jerk_mps3: f32,
    pub acceptance_radius_m: f32,
    pub max_yaw_rate_rad_s: f32,
}

impl Default for TrajectoryLimits {
    fn default() -> Self {
        Self {
            max_velocity_mps: DEFAULT_TRAJECTORY_SPEED_MPS,
            max_accel_mps2: DEFAULT_TRAJECTORY_ACCEL_MPS2,
            max_jerk_mps3: DEFAULT_TRAJECTORY_JERK_MPS3,
            acceptance_radius_m: DEFAULT_ACCEPTANCE_RADIUS_M,
            max_yaw_rate_rad_s: DEFAULT_YAW_RATE_RAD_S,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuidanceOutput {
    pub phase: GuidancePhase,
    pub trajectory: LocalTrajectorySetpoint,
    pub target_reached: bool,
    pub distance_to_target_m: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrajectoryGenerator {
    limits: TrajectoryLimits,
    initialized: bool,
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    acceleration_ned_mps2: [f32; 3],
    yaw_rad: f32,
}

impl TrajectoryGenerator {
    pub const fn new(limits: TrajectoryLimits) -> Self {
        Self {
            limits,
            initialized: false,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            acceleration_ned_mps2: [0.0; 3],
            yaw_rad: 0.0,
        }
    }

    pub fn reset_to_estimate(&mut self, state: EstimatorLocalState) -> bool {
        if !state.valid
            || !finite_f32x3(state.position_ned_m)
            || !finite_f32x3(state.velocity_ned_mps)
            || !state.yaw_rad.is_finite()
        {
            self.initialized = false;
            return false;
        }

        self.initialized = true;
        self.position_ned_m = state.position_ned_m;
        self.velocity_ned_mps = state.velocity_ned_mps;
        self.acceleration_ned_mps2 = [0.0; 3];
        self.yaw_rad = state.yaw_rad;
        true
    }

    pub const fn limits(&self) -> TrajectoryLimits {
        self.limits
    }

    pub fn update_toward(
        &mut self,
        state: EstimatorLocalState,
        waypoint: MissionWaypoint,
        dt_s: f32,
    ) -> Option<GuidanceOutput> {
        if !valid_dt(dt_s)
            || !waypoint
                .position_ned_m
                .iter()
                .all(|value| value.is_finite())
            || !waypoint.yaw_rad.is_finite()
            || !state.valid
            || !finite_f32x3(state.position_ned_m)
            || !state.yaw_rad.is_finite()
        {
            self.initialized = false;
            return None;
        }

        if !self.initialized && !self.reset_to_estimate(state) {
            return None;
        }

        let max_velocity_mps = positive_finite_or_zero(self.limits.max_velocity_mps);
        let max_accel_mps2 = positive_finite_or_zero(self.limits.max_accel_mps2);
        let max_jerk_mps3 = positive_finite_or_zero(self.limits.max_jerk_mps3);
        let acceptance_radius_m = positive_finite_or_zero(self.limits.acceptance_radius_m);

        let target_delta = sub3(waypoint.position_ned_m, self.position_ned_m);
        let distance_to_target_m = norm3(target_delta);
        let speed_mps = norm3(self.velocity_ned_mps);
        let reached =
            distance_to_target_m <= acceptance_radius_m && speed_mps <= max_velocity_mps * 0.10;

        if reached || max_velocity_mps <= f32::EPSILON || max_accel_mps2 <= f32::EPSILON {
            self.settle_at_target(waypoint, dt_s, max_velocity_mps, max_jerk_mps3);
        } else {
            let direction = scale3(target_delta, 1.0 / distance_to_target_m.max(f32::EPSILON));
            let stopping_speed_mps =
                micromath::F32Ext::sqrt(2.0 * max_accel_mps2 * distance_to_target_m);
            let desired_speed_mps = max_velocity_mps.min(stopping_speed_mps);
            let desired_velocity = scale3(direction, desired_speed_mps);
            let accel_request = scale3(sub3(desired_velocity, self.velocity_ned_mps), 1.0 / dt_s);
            let mut next_accel = clamp_norm3(accel_request, max_accel_mps2);

            if max_jerk_mps3 > f32::EPSILON {
                let max_accel_delta = max_jerk_mps3 * dt_s;
                let accel_delta = clamp_norm3(
                    sub3(next_accel, self.acceleration_ned_mps2),
                    max_accel_delta,
                );
                next_accel = add3(self.acceleration_ned_mps2, accel_delta);
            }

            let next_velocity = clamp_norm3(
                add3(self.velocity_ned_mps, scale3(next_accel, dt_s)),
                max_velocity_mps,
            );
            let next_position = add3(self.position_ned_m, scale3(next_velocity, dt_s));

            if dot3(sub3(waypoint.position_ned_m, next_position), target_delta) <= 0.0 {
                self.settle_at_target(waypoint, dt_s, max_velocity_mps, max_jerk_mps3);
            } else {
                self.position_ned_m = next_position;
                self.velocity_ned_mps = next_velocity;
                self.acceleration_ned_mps2 = next_accel;
            }
        }

        let yaw_error_rad = wrap_pi(waypoint.yaw_rad - self.yaw_rad);
        let yaw_rate_rad_s = clamp_symmetric(
            yaw_error_rad / dt_s,
            positive_finite_or_zero(self.limits.max_yaw_rate_rad_s),
        );
        self.yaw_rad = wrap_pi(self.yaw_rad + yaw_rate_rad_s * dt_s);
        if yaw_error_rad.abs() <= yaw_rate_rad_s.abs() * dt_s + 0.000_01 {
            self.yaw_rad = waypoint.yaw_rad;
        }

        let trajectory = LocalTrajectorySetpoint {
            position_ned_m: self.position_ned_m,
            velocity_ned_mps: self.velocity_ned_mps,
            acceleration_ned_mps2: self.acceleration_ned_mps2,
            yaw_rad: self.yaw_rad,
            yaw_rate_rad_s,
        };

        Some(GuidanceOutput {
            phase: GuidancePhase::NavigateLeg,
            trajectory,
            target_reached: norm3(sub3(waypoint.position_ned_m, self.position_ned_m))
                <= acceptance_radius_m,
            distance_to_target_m: norm3(sub3(waypoint.position_ned_m, self.position_ned_m)),
        })
    }

    fn settle_at_target(
        &mut self,
        waypoint: MissionWaypoint,
        dt_s: f32,
        max_velocity_mps: f32,
        max_jerk_mps3: f32,
    ) {
        let next_accel = if max_jerk_mps3 > f32::EPSILON {
            let max_accel_delta = max_jerk_mps3 * dt_s;
            add3(
                self.acceleration_ned_mps2,
                clamp_norm3(scale3(self.acceleration_ned_mps2, -1.0), max_accel_delta),
            )
        } else {
            [0.0; 3]
        };
        let next_velocity = clamp_norm3(
            add3(self.velocity_ned_mps, scale3(next_accel, dt_s)),
            max_velocity_mps,
        );

        self.position_ned_m = waypoint.position_ned_m;
        self.velocity_ned_mps = next_velocity;
        self.acceleration_ned_mps2 = next_accel;
    }
}

impl Default for TrajectoryGenerator {
    fn default() -> Self {
        Self::new(TrajectoryLimits::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuidanceStateMachine {
    trajectory: TrajectoryGenerator,
    phase: GuidancePhase,
}

impl GuidanceStateMachine {
    pub const fn new(limits: TrajectoryLimits) -> Self {
        Self {
            trajectory: TrajectoryGenerator::new(limits),
            phase: GuidancePhase::Idle,
        }
    }

    pub const fn phase(&self) -> GuidancePhase {
        self.phase
    }

    pub fn reset(&mut self) {
        self.trajectory.initialized = false;
        self.phase = GuidancePhase::Idle;
    }

    pub fn update(
        &mut self,
        state: EstimatorLocalState,
        command: GuidanceCommand,
        dt_s: f32,
    ) -> Option<GuidanceOutput> {
        self.phase = command.phase();
        match command {
            GuidanceCommand::Idle => {
                self.trajectory.initialized = false;
                None
            }
            GuidanceCommand::Takeoff(setpoint) | GuidanceCommand::Hold(setpoint) => {
                if !setpoint.finite() {
                    self.trajectory.initialized = false;
                    return None;
                }
                self.trajectory.initialized = true;
                self.trajectory.position_ned_m = setpoint.position_ned_m;
                self.trajectory.velocity_ned_mps = [0.0; 3];
                self.trajectory.acceleration_ned_mps2 = [0.0; 3];
                self.trajectory.yaw_rad = setpoint.yaw_rad;
                Some(GuidanceOutput {
                    phase: self.phase,
                    trajectory: setpoint,
                    target_reached: true,
                    distance_to_target_m: 0.0,
                })
            }
            GuidanceCommand::NavigateLeg(waypoint)
            | GuidanceCommand::Loiter(waypoint)
            | GuidanceCommand::Land(waypoint) => {
                let mut output = self.trajectory.update_toward(state, waypoint, dt_s)?;
                output.phase = self.phase;
                Some(output)
            }
        }
    }
}

impl Default for GuidanceStateMachine {
    fn default() -> Self {
        Self::new(TrajectoryLimits::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffNavGateConfig {
    pub takeoff_down_m: f32,
    pub max_takeoff_down_error_m: f32,
    pub max_vertical_speed_mps: f32,
    pub max_horizontal_speed_mps: f32,
    pub max_estimate_truth_down_error_m: f32,
    pub max_yaw_error_rad: f32,
    pub max_clamp_ratio: f32,
    pub min_stable_frames: usize,
}

impl Default for TakeoffNavGateConfig {
    fn default() -> Self {
        Self {
            takeoff_down_m: -1.0,
            max_takeoff_down_error_m: 0.25,
            max_vertical_speed_mps: 0.25,
            max_horizontal_speed_mps: 0.20,
            max_estimate_truth_down_error_m: 0.25,
            max_yaw_error_rad: 5.0_f32.to_radians(),
            max_clamp_ratio: 0.02,
            min_stable_frames: 10,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffNavGateSample {
    pub estimate_position_ned_m: [f32; 3],
    pub estimate_velocity_ned_mps: [f32; 3],
    pub truth_position_ned_m: Option<[f32; 3]>,
    pub truth_velocity_ned_mps: Option<[f32; 3]>,
    pub yaw_error_rad: Option<f32>,
    pub clamp_ratio: f32,
    pub estimate_valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffNavDecision {
    pub hold_setpoint: LocalTrajectorySetpoint,
    pub latched_takeoff_origin_ned_m: [f32; 3],
    pub stable_frames: usize,
    pub nav_ready: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TakeoffNavGate {
    config: TakeoffNavGateConfig,
    latched_takeoff_origin_ned_m: Option<[f32; 3]>,
    stable_frames: usize,
    nav_ready: bool,
}

impl TakeoffNavGate {
    pub const fn new(config: TakeoffNavGateConfig) -> Self {
        Self {
            config,
            latched_takeoff_origin_ned_m: None,
            stable_frames: 0,
            nav_ready: false,
        }
    }

    pub fn reset(&mut self) {
        self.latched_takeoff_origin_ned_m = None;
        self.stable_frames = 0;
        self.nav_ready = false;
    }

    pub fn update(&mut self, sample: TakeoffNavGateSample, yaw_rad: f32) -> TakeoffNavDecision {
        let origin = *self
            .latched_takeoff_origin_ned_m
            .get_or_insert(sample.estimate_position_ned_m);
        let hold_position_ned_m = [origin[0], origin[1], self.config.takeoff_down_m];

        if self.sample_is_stable(sample) {
            self.stable_frames = self.stable_frames.saturating_add(1);
        } else {
            self.stable_frames = 0;
        }
        self.nav_ready = self.nav_ready || self.stable_frames >= self.config.min_stable_frames;

        TakeoffNavDecision {
            hold_setpoint: LocalTrajectorySetpoint::position_hold(hold_position_ned_m, yaw_rad),
            latched_takeoff_origin_ned_m: origin,
            stable_frames: self.stable_frames,
            nav_ready: self.nav_ready,
        }
    }

    fn sample_is_stable(&self, sample: TakeoffNavGateSample) -> bool {
        if !sample.estimate_valid
            || !finite_f32x3(sample.estimate_position_ned_m)
            || !finite_f32x3(sample.estimate_velocity_ned_mps)
            || !sample.clamp_ratio.is_finite()
            || sample.clamp_ratio > self.config.max_clamp_ratio
        {
            return false;
        }

        let estimate_down_error_m =
            (sample.estimate_position_ned_m[2] - self.config.takeoff_down_m).abs();
        let estimate_vertical_speed_mps = sample.estimate_velocity_ned_mps[2].abs();
        let estimate_horizontal_speed_mps = horizontal_norm([
            sample.estimate_velocity_ned_mps[0],
            sample.estimate_velocity_ned_mps[1],
        ]);
        if estimate_down_error_m > self.config.max_takeoff_down_error_m
            || estimate_vertical_speed_mps > self.config.max_vertical_speed_mps
            || estimate_horizontal_speed_mps > self.config.max_horizontal_speed_mps
        {
            return false;
        }

        if let Some(yaw_error_rad) = sample.yaw_error_rad {
            if !yaw_error_rad.is_finite() || yaw_error_rad.abs() > self.config.max_yaw_error_rad {
                return false;
            }
        }

        if let Some(truth_position_ned_m) = sample.truth_position_ned_m {
            if !finite_f32x3(truth_position_ned_m)
                || (sample.estimate_position_ned_m[2] - truth_position_ned_m[2]).abs()
                    > self.config.max_estimate_truth_down_error_m
            {
                return false;
            }
        }

        if let Some(truth_velocity_ned_mps) = sample.truth_velocity_ned_mps {
            if !finite_f32x3(truth_velocity_ned_mps)
                || truth_velocity_ned_mps[2].abs() > self.config.max_vertical_speed_mps
                || horizontal_norm([truth_velocity_ned_mps[0], truth_velocity_ned_mps[1]])
                    > self.config.max_horizontal_speed_mps
            {
                return false;
            }
        }

        true
    }
}

impl Default for TakeoffNavGate {
    fn default() -> Self {
        Self::new(TakeoffNavGateConfig::default())
    }
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn valid_dt(dt_s: f32) -> bool {
    dt_s.is_finite() && dt_s > 0.0
}

fn horizontal_norm(values: [f32; 2]) -> f32 {
    micromath::F32Ext::sqrt(values[0] * values[0] + values[1] * values[1])
}

fn norm3(values: [f32; 3]) -> f32 {
    micromath::F32Ext::sqrt(dot3(values, values))
}

fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale3(values: [f32; 3], scale: f32) -> [f32; 3] {
    [values[0] * scale, values[1] * scale, values[2] * scale]
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn clamp_norm3(values: [f32; 3], limit: f32) -> [f32; 3] {
    let limit = positive_finite_or_zero(limit);
    let magnitude = norm3(values);
    if magnitude <= limit || magnitude <= f32::EPSILON {
        values
    } else {
        scale3(values, limit / magnitude)
    }
}

fn positive_finite_or_zero(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    let limit = positive_finite_or_zero(limit);
    if limit <= f32::EPSILON {
        0.0
    } else {
        value.clamp(-limit, limit)
    }
}

fn wrap_pi(mut angle_rad: f32) -> f32 {
    while angle_rad > core::f32::consts::PI {
        angle_rad -= 2.0 * core::f32::consts::PI;
    }
    while angle_rad < -core::f32::consts::PI {
        angle_rad += 2.0 * core::f32::consts::PI;
    }
    angle_rad
}

#[cfg(test)]
mod tests {
    use super::{
        GuidanceCommand, GuidancePhase, GuidanceStateMachine, TakeoffNavGate, TakeoffNavGateConfig,
        TakeoffNavGateSample, TrajectoryGenerator, TrajectoryLimits,
    };
    use crate::control::pipeline::{ControlSetpoint, EstimatorLocalState, MissionWaypoint};

    #[test]
    fn takeoff_hold_latches_estimator_xy_offset() {
        let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
            takeoff_down_m: -2.0,
            ..TakeoffNavGateConfig::default()
        });

        let decision = gate.update(
            TakeoffNavGateSample {
                estimate_position_ned_m: [12.0, -7.0, 0.0],
                estimate_velocity_ned_mps: [0.0, 0.0, 0.0],
                truth_position_ned_m: None,
                truth_velocity_ned_mps: None,
                yaw_error_rad: Some(0.0),
                clamp_ratio: 0.0,
                estimate_valid: true,
            },
            0.0,
        );

        assert_eq!(decision.latched_takeoff_origin_ned_m, [12.0, -7.0, 0.0]);
        assert_eq!(decision.hold_setpoint.position_ned_m, [12.0, -7.0, -2.0]);
        assert!(!decision.nav_ready);
    }

    #[test]
    fn nav_waits_until_vertical_horizontal_yaw_and_clamp_are_stable() {
        let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
            takeoff_down_m: -2.0,
            min_stable_frames: 2,
            ..TakeoffNavGateConfig::default()
        });

        let moving = stable_sample_with_velocity([0.0, 0.0, -0.8]);
        assert!(!gate.update(moving, 0.0).nav_ready);

        let stable = stable_sample_with_velocity([0.05, 0.03, 0.02]);
        assert!(!gate.update(stable, 0.0).nav_ready);
        assert!(gate.update(stable, 0.0).nav_ready);
    }

    #[test]
    fn clamp_ratio_blocks_nav_start() {
        let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
            takeoff_down_m: -2.0,
            min_stable_frames: 1,
            max_clamp_ratio: 0.05,
            ..TakeoffNavGateConfig::default()
        });
        let mut sample = stable_sample_with_velocity([0.0, 0.0, 0.0]);
        sample.clamp_ratio = 0.25;

        assert!(!gate.update(sample, 0.0).nav_ready);
    }

    #[test]
    fn trajectory_generator_shapes_waypoint_target_with_limits() {
        let limits = TrajectoryLimits {
            max_velocity_mps: 0.5,
            max_accel_mps2: 0.8,
            max_jerk_mps3: 4.0,
            acceptance_radius_m: 0.05,
            max_yaw_rate_rad_s: 0.75,
        };
        let mut generator = TrajectoryGenerator::new(limits);
        let state = EstimatorLocalState::new([0.0, 0.0, -1.0], [0.0; 3], 0.0, true);
        let waypoint = MissionWaypoint::new([2.0, 0.0, -1.0], 1.0);

        let mut previous_velocity = [0.0; 3];
        let mut previous_accel = [0.0; 3];
        let mut final_output = None;
        for step in 0..600 {
            let output = generator
                .update_toward(state, waypoint, 0.01)
                .expect("finite trajectory update should succeed");
            let velocity_delta = super::sub3(output.trajectory.velocity_ned_mps, previous_velocity);
            let accel_delta = super::sub3(output.trajectory.acceleration_ned_mps2, previous_accel);

            assert!(super::norm3(output.trajectory.velocity_ned_mps) <= 0.5 + 0.001);
            assert!(super::norm3(velocity_delta) / 0.01 <= 0.8 + 0.05);
            let jerk_mps3 = super::norm3(accel_delta) / 0.01;
            assert!(
                jerk_mps3 <= 4.0 + 0.20,
                "step={step} jerk_mps3={jerk_mps3} accel={:?} previous={:?}",
                output.trajectory.acceleration_ned_mps2,
                previous_accel
            );
            assert!(output.trajectory.yaw_rate_rad_s.abs() <= 0.75 + 0.001);

            previous_velocity = output.trajectory.velocity_ned_mps;
            previous_accel = output.trajectory.acceleration_ned_mps2;
            final_output = Some(output);
        }

        let final_output = final_output.unwrap();
        assert!(final_output.trajectory.position_ned_m[0] > 1.7);
        assert!(final_output.trajectory.position_ned_m[1].abs() < 0.001);
    }

    #[test]
    fn guidance_state_machine_emits_explicit_navigation_phase() {
        let mut guidance = GuidanceStateMachine::default();
        let state = EstimatorLocalState::new([0.0, 0.0, -1.0], [0.0; 3], 0.0, true);
        let output = guidance
            .update(
                state,
                GuidanceCommand::NavigateLeg(MissionWaypoint::new([1.0, 0.0, -1.0], 0.0)),
                0.02,
            )
            .expect("navigation guidance should produce trajectory");

        assert_eq!(guidance.phase(), GuidancePhase::NavigateLeg);
        assert_eq!(output.phase, GuidancePhase::NavigateLeg);
        let setpoint = ControlSetpoint::from_local_trajectory(output.trajectory, true);
        assert!(setpoint.armed());
    }

    fn stable_sample_with_velocity(velocity_ned_mps: [f32; 3]) -> TakeoffNavGateSample {
        TakeoffNavGateSample {
            estimate_position_ned_m: [0.5, -0.25, -2.05],
            estimate_velocity_ned_mps: velocity_ned_mps,
            truth_position_ned_m: Some([0.52, -0.20, -2.0]),
            truth_velocity_ned_mps: Some(velocity_ned_mps),
            yaw_error_rad: Some(0.01),
            clamp_ratio: 0.0,
            estimate_valid: true,
        }
    }
}
