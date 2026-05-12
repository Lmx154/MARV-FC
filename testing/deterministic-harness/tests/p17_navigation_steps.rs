use deterministic_harness::{
    ControlPipeline, ControlPipelineTrace, ControlSetpoint, ControlSetpointSource,
    EstimateSnapshot, EstimatorLocalState, GuidanceCommand, GuidancePhase, GuidanceStateMachine,
    ImuControlInput, MissionWaypoint, TrajectoryLimits,
};

const DT_S: f32 = 0.01;
const STEP_TICKS: usize = 1_200;
const LEG_TICKS: usize = 900;
const GUIDED_LEG_TICKS: usize = 1_800;
const HOLD_DOWN_M: f32 = -1.0;
const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;
const MAX_TILT_RAD: f32 = 10.0_f32.to_radians();
const MAX_HORIZONTAL_ACCEL_MPS2: f32 = 1.5;
const MAX_HORIZONTAL_SPEED_MPS: f32 = 1.0;
const MAX_TRAJECTORY_SPEED_MPS: f32 = 0.45;
const MAX_TRAJECTORY_ACCEL_MPS2: f32 = 0.75;
const MAX_TRAJECTORY_JERK_MPS3: f32 = 3.0;
const MIN_STEP_PROGRESS_M: f32 = 0.45;
const MAX_CROSS_AXIS_DRIFT_M: f32 = 0.03;
const MAX_SQUARE_FINAL_ERROR_M: f32 = 0.65;

#[derive(Clone, Copy, Debug, PartialEq)]
struct NavigationState {
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    yaw_rad: f32,
}

impl NavigationState {
    const fn new(yaw_rad: f32) -> Self {
        Self {
            position_ned_m: [0.0, 0.0, HOLD_DOWN_M],
            velocity_ned_mps: [0.0, 0.0, 0.0],
            yaw_rad,
        }
    }

    fn estimate(self) -> EstimateSnapshot {
        EstimateSnapshot {
            position_ned_m: self.position_ned_m,
            velocity_ned_mps: self.velocity_ned_mps,
            quaternion: yaw_quaternion(self.yaw_rad),
            valid: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct NavigationSample {
    tick: u64,
    target_ned_m: [f32; 3],
    target_velocity_ned_mps: [f32; 3],
    target_acceleration_ned_mps2: [f32; 3],
    setpoint_source: ControlSetpointSource,
    state: NavigationState,
    accel_ned_mps2: [f32; 2],
    roll_rad: f32,
    pitch_rad: f32,
    control: ControlPipelineTrace,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct NavigationSummary {
    initial_target_error_m: f32,
    final_target_error_m: f32,
    max_tilt_rad: f32,
    max_horizontal_accel_mps2: f32,
    max_horizontal_speed_mps: f32,
    max_target_velocity_mps: f32,
    max_target_accel_mps2: f32,
    max_target_jerk_mps3: f32,
    clamp_ratio: f32,
    final_position_ned_m: [f32; 3],
    final_velocity_ned_mps: [f32; 3],
}

#[test]
fn p17_navigation_steps_move_in_world_frame_axes() {
    let north = run_fixed_step(
        ControlSetpoint::local_position_ned([1.0, 0.0, HOLD_DOWN_M], 0.0, true),
        0.0,
    );
    let north_summary = summarize(&north);
    assert_navigation_envelope("north step", north_summary);
    assert!(
        north_summary.final_position_ned_m[0] >= MIN_STEP_PROGRESS_M,
        "north step did not move north enough: {north_summary:?}"
    );
    assert!(
        north_summary.final_position_ned_m[1].abs() <= MAX_CROSS_AXIS_DRIFT_M,
        "north step drifted on east axis: {north_summary:?}"
    );
    assert!(
        initial_pitch(&north) < 0.0 && initial_roll(&north).abs() <= 0.001,
        "north step did not command nose-down world-frame acceleration"
    );

    let east = run_fixed_step(
        ControlSetpoint::local_position_ned([0.0, 1.0, HOLD_DOWN_M], 0.0, true),
        0.0,
    );
    let east_summary = summarize(&east);
    assert_navigation_envelope("east step", east_summary);
    assert!(
        east_summary.final_position_ned_m[1] >= MIN_STEP_PROGRESS_M,
        "east step did not move east enough: {east_summary:?}"
    );
    assert!(
        east_summary.final_position_ned_m[0].abs() <= MAX_CROSS_AXIS_DRIFT_M,
        "east step drifted on north axis: {east_summary:?}"
    );
    assert!(
        initial_roll(&east) > 0.0 && initial_pitch(&east).abs() <= 0.001,
        "east step did not command right-roll world-frame acceleration"
    );
}

#[test]
fn p17_navigation_steps_diagonal_and_return_origin_are_bounded() {
    let diagonal = run_fixed_step(
        ControlSetpoint::local_position_ned([1.0, 1.0, HOLD_DOWN_M], 0.0, true),
        0.0,
    );
    let diagonal_summary = summarize(&diagonal);
    assert_navigation_envelope("diagonal step", diagonal_summary);
    assert!(
        diagonal_summary.final_position_ned_m[0] >= MIN_STEP_PROGRESS_M
            && diagonal_summary.final_position_ned_m[1] >= MIN_STEP_PROGRESS_M,
        "diagonal step did not move both axes with the correct signs: {diagonal_summary:?}"
    );
    assert!(
        initial_roll(&diagonal) > 0.0 && initial_pitch(&diagonal) < 0.0,
        "diagonal step did not demand matching roll/pitch signs"
    );

    let square = run_square();
    let square_summary = summarize(&square);
    assert_navigation_envelope("waypoint square", square_summary);
    assert!(
        square_summary.final_target_error_m <= MAX_SQUARE_FINAL_ERROR_M,
        "small waypoint square did not return close enough to origin: {square_summary:?}"
    );
    assert_square_leg_progress(&square);
}

#[test]
fn p17_guided_waypoint_square_uses_bounded_local_trajectory() {
    let square = run_guided_square();
    let square_summary = summarize(&square);
    assert_navigation_dynamics("guided waypoint square", square_summary);
    assert!(
        square_summary.max_target_velocity_mps <= MAX_TRAJECTORY_SPEED_MPS + 0.001,
        "guided square exceeded trajectory velocity limit: {square_summary:?}"
    );
    assert!(
        square_summary.max_target_accel_mps2 <= MAX_TRAJECTORY_ACCEL_MPS2 + 0.02,
        "guided square exceeded trajectory acceleration limit: {square_summary:?}"
    );
    assert!(
        square_summary.max_target_jerk_mps3 <= MAX_TRAJECTORY_JERK_MPS3 + 0.20,
        "guided square exceeded trajectory jerk limit: {square_summary:?}"
    );
    assert!(
        horizontal_error(square_summary.final_position_ned_m, [0.0, 0.0, HOLD_DOWN_M])
            <= MAX_SQUARE_FINAL_ERROR_M,
        "guided waypoint square did not return close enough to the mission origin: {square_summary:?}"
    );
    assert!(
        square.iter().all(|sample| {
            sample.setpoint_source == ControlSetpointSource::EstimatorLocalTrajectory
        }),
        "guided square must keep truth evidence out of the control setpoint path"
    );
}

#[test]
fn p17_navigation_steps_yawed_navigation_remains_world_frame_correct() {
    let yaw_rad = 90.0_f32.to_radians();
    let north = run_fixed_step(
        ControlSetpoint::local_position_ned([1.0, 0.0, HOLD_DOWN_M], yaw_rad, true),
        yaw_rad,
    );
    let north_summary = summarize(&north);
    assert_navigation_envelope("yawed north step", north_summary);
    assert!(
        north_summary.final_position_ned_m[0] >= MIN_STEP_PROGRESS_M,
        "yawed north step did not move north in world frame: {north_summary:?}"
    );
    assert!(
        north_summary.final_position_ned_m[1].abs() <= MAX_CROSS_AXIS_DRIFT_M,
        "yawed north step coupled into east motion: {north_summary:?}"
    );
    assert!(
        initial_roll(&north) < 0.0 && initial_pitch(&north).abs() <= 0.001,
        "yawed north step should map world north acceleration into body-left roll"
    );

    let east = run_fixed_step(
        ControlSetpoint::local_position_ned([0.0, 1.0, HOLD_DOWN_M], yaw_rad, true),
        yaw_rad,
    );
    let east_summary = summarize(&east);
    assert_navigation_envelope("yawed east step", east_summary);
    assert!(
        east_summary.final_position_ned_m[1] >= MIN_STEP_PROGRESS_M,
        "yawed east step did not move east in world frame: {east_summary:?}"
    );
    assert!(
        east_summary.final_position_ned_m[0].abs() <= MAX_CROSS_AXIS_DRIFT_M,
        "yawed east step coupled into north motion: {east_summary:?}"
    );
    assert!(
        initial_pitch(&east) < 0.0 && initial_roll(&east).abs() <= 0.001,
        "yawed east step should map world east acceleration into nose-down pitch"
    );
}

#[test]
fn p17_navigation_steps_reject_invalid_or_disarmed_navigation_demands() {
    let invalid = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([1.0, f32::NAN, HOLD_DOWN_M], 0.0, true),
    );
    assert!(invalid.armed);
    assert!(!invalid.control_valid);
    assert_eq!(invalid.motors, [0.0; 4]);
    assert!(invalid.attitude_setpoint.is_none());

    let disarmed = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([1.0, 1.0, HOLD_DOWN_M], 0.0, false),
    );
    assert!(!disarmed.armed);
    assert!(!disarmed.control_valid);
    assert_eq!(disarmed.motors, [0.0; 4]);
}

fn run_fixed_step(setpoint: ControlSetpoint, yaw_rad: f32) -> Vec<NavigationSample> {
    run_navigation(yaw_rad, STEP_TICKS, move |_, _| {
        (setpoint, [0.0; 3], [0.0; 3])
    })
}

fn run_square() -> Vec<NavigationSample> {
    run_navigation(0.0, LEG_TICKS * 4, |tick, _| {
        let leg = tick / LEG_TICKS;
        let position = match leg {
            0 => [1.0, 0.0, HOLD_DOWN_M],
            1 => [1.0, 1.0, HOLD_DOWN_M],
            2 => [0.0, 1.0, HOLD_DOWN_M],
            _ => [0.0, 0.0, HOLD_DOWN_M],
        };
        (
            ControlSetpoint::local_position_ned(position, 0.0, true),
            [0.0; 3],
            [0.0; 3],
        )
    })
}

fn run_guided_square() -> Vec<NavigationSample> {
    let mut guidance = GuidanceStateMachine::new(TrajectoryLimits {
        max_velocity_mps: MAX_TRAJECTORY_SPEED_MPS,
        max_accel_mps2: MAX_TRAJECTORY_ACCEL_MPS2,
        max_jerk_mps3: MAX_TRAJECTORY_JERK_MPS3,
        acceptance_radius_m: 0.08,
        max_yaw_rate_rad_s: 1.0,
    });

    run_navigation(0.0, GUIDED_LEG_TICKS * 4, |tick, state| {
        let leg = tick / GUIDED_LEG_TICKS;
        let position = match leg {
            0 => [1.0, 0.0, HOLD_DOWN_M],
            1 => [1.0, 1.0, HOLD_DOWN_M],
            2 => [0.0, 1.0, HOLD_DOWN_M],
            _ => [0.0, 0.0, HOLD_DOWN_M],
        };
        let output = guidance
            .update(
                state.estimator_local(),
                GuidanceCommand::NavigateLeg(MissionWaypoint::new(position, 0.0)),
                DT_S,
            )
            .expect("guided waypoint square should produce a trajectory setpoint");
        assert_eq!(output.phase, GuidancePhase::NavigateLeg);
        let setpoint = ControlSetpoint::from_local_trajectory(output.trajectory, true);
        assert_eq!(
            setpoint.source(),
            ControlSetpointSource::EstimatorLocalTrajectory
        );
        (
            setpoint,
            output.trajectory.velocity_ned_mps,
            output.trajectory.acceleration_ned_mps2,
        )
    })
}

fn run_navigation(
    yaw_rad: f32,
    ticks: usize,
    mut setpoint_for_tick: impl FnMut(usize, NavigationState) -> (ControlSetpoint, [f32; 3], [f32; 3]),
) -> Vec<NavigationSample> {
    let pipeline = ControlPipeline::default();
    let mut state = NavigationState::new(yaw_rad);
    let mut samples = Vec::with_capacity(ticks);

    for index in 0..ticks {
        let (setpoint, target_velocity_ned_mps, target_acceleration_ned_mps2) =
            setpoint_for_tick(index, state);
        state.yaw_rad = setpoint.yaw_rad();
        let control = pipeline.step(state.estimate(), state.imu(), setpoint);
        assert!(
            control.control_valid,
            "navigation control should stay valid at tick {}: {control:?}",
            index + 1
        );
        let attitude = control
            .attitude_setpoint
            .expect("valid navigation control should produce an attitude setpoint");
        let (roll_rad, pitch_rad, yaw_rad) = euler_from_quaternion(attitude.quaternion);
        let accel_ned_mps2 = horizontal_accel_from_tilt(roll_rad, pitch_rad, yaw_rad);

        state.velocity_ned_mps[0] += accel_ned_mps2[0] * DT_S;
        state.velocity_ned_mps[1] += accel_ned_mps2[1] * DT_S;
        state.position_ned_m[0] += state.velocity_ned_mps[0] * DT_S;
        state.position_ned_m[1] += state.velocity_ned_mps[1] * DT_S;

        samples.push(NavigationSample {
            tick: (index + 1) as u64,
            target_ned_m: setpoint.position_ned_m(),
            target_velocity_ned_mps,
            target_acceleration_ned_mps2,
            setpoint_source: setpoint.source(),
            state,
            accel_ned_mps2,
            roll_rad,
            pitch_rad,
            control,
        });
    }

    samples
}

impl NavigationState {
    fn imu(self) -> ImuControlInput {
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: [0.0, 0.0, 0.0],
        }
    }

    fn estimator_local(self) -> EstimatorLocalState {
        EstimatorLocalState::new(
            self.position_ned_m,
            self.velocity_ned_mps,
            self.yaw_rad,
            true,
        )
    }
}

fn summarize(samples: &[NavigationSample]) -> NavigationSummary {
    assert!(!samples.is_empty());
    let first = samples.first().unwrap();
    let last = samples.last().unwrap();
    let clamped = samples
        .iter()
        .filter(|sample| sample.control.clamped)
        .count();
    let max_tilt_rad = samples
        .iter()
        .map(|sample| sample.roll_rad.abs().max(sample.pitch_rad.abs()))
        .fold(0.0, f32::max);
    let max_horizontal_accel_mps2 = samples
        .iter()
        .map(|sample| horizontal_norm(sample.accel_ned_mps2[0], sample.accel_ned_mps2[1]))
        .fold(0.0, f32::max);
    let max_horizontal_speed_mps = samples
        .iter()
        .map(|sample| {
            horizontal_norm(
                sample.state.velocity_ned_mps[0],
                sample.state.velocity_ned_mps[1],
            )
        })
        .fold(0.0, f32::max);
    let max_target_velocity_mps = samples
        .iter()
        .map(|sample| horizontal_norm3(sample.target_velocity_ned_mps))
        .fold(0.0, f32::max);
    let max_target_accel_mps2 = samples
        .iter()
        .map(|sample| horizontal_norm3(sample.target_acceleration_ned_mps2))
        .fold(0.0, f32::max);
    let max_target_jerk_mps3 = samples
        .windows(2)
        .map(|pair| {
            horizontal_norm3([
                pair[1].target_acceleration_ned_mps2[0] - pair[0].target_acceleration_ned_mps2[0],
                pair[1].target_acceleration_ned_mps2[1] - pair[0].target_acceleration_ned_mps2[1],
                pair[1].target_acceleration_ned_mps2[2] - pair[0].target_acceleration_ned_mps2[2],
            ]) / DT_S
        })
        .fold(0.0, f32::max);

    NavigationSummary {
        initial_target_error_m: horizontal_error(first.state.position_ned_m, first.target_ned_m),
        final_target_error_m: horizontal_error(last.state.position_ned_m, last.target_ned_m),
        max_tilt_rad,
        max_horizontal_accel_mps2,
        max_horizontal_speed_mps,
        max_target_velocity_mps,
        max_target_accel_mps2,
        max_target_jerk_mps3,
        clamp_ratio: clamped as f32 / samples.len() as f32,
        final_position_ned_m: last.state.position_ned_m,
        final_velocity_ned_mps: last.state.velocity_ned_mps,
    }
}

fn assert_navigation_envelope(label: &str, summary: NavigationSummary) {
    assert!(
        summary.final_target_error_m <= summary.initial_target_error_m - MIN_STEP_PROGRESS_M,
        "{label} did not improve enough toward the target: {summary:?}"
    );
    assert_navigation_dynamics(label, summary);
}

fn assert_navigation_dynamics(label: &str, summary: NavigationSummary) {
    assert!(
        summary.max_tilt_rad <= MAX_TILT_RAD + 0.001,
        "{label} exceeded tilt limit: {summary:?}"
    );
    assert!(
        summary.max_horizontal_accel_mps2 <= MAX_HORIZONTAL_ACCEL_MPS2 + 0.02,
        "{label} exceeded horizontal acceleration limit: {summary:?}"
    );
    assert!(
        summary.max_horizontal_speed_mps <= MAX_HORIZONTAL_SPEED_MPS,
        "{label} exceeded horizontal speed envelope: {summary:?}"
    );
    assert_eq!(
        summary.clamp_ratio, 0.0,
        "{label} should not need mixer clamping for small navigation steps: {summary:?}"
    );
}

fn assert_square_leg_progress(samples: &[NavigationSample]) {
    for leg in 0..4 {
        let start_index = leg * LEG_TICKS;
        let end_index = ((leg + 1) * LEG_TICKS).saturating_sub(1);
        let start = samples[start_index];
        let end = samples[end_index];
        let initial_error = horizontal_error(start.state.position_ned_m, start.target_ned_m);
        let final_error = horizontal_error(end.state.position_ned_m, end.target_ned_m);
        assert!(
            final_error < initial_error,
            "square leg {leg} did not improve: initial_error={initial_error} final_error={final_error}"
        );
    }
}

fn initial_roll(samples: &[NavigationSample]) -> f32 {
    samples.first().unwrap().roll_rad
}

fn initial_pitch(samples: &[NavigationSample]) -> f32 {
    samples.first().unwrap().pitch_rad
}

fn horizontal_error(position_ned_m: [f32; 3], target_ned_m: [f32; 3]) -> f32 {
    horizontal_norm(
        target_ned_m[0] - position_ned_m[0],
        target_ned_m[1] - position_ned_m[1],
    )
}

fn horizontal_norm(north_m: f32, east_m: f32) -> f32 {
    (north_m * north_m + east_m * east_m).sqrt()
}

fn horizontal_norm3(values: [f32; 3]) -> f32 {
    (values[0] * values[0] + values[1] * values[1] + values[2] * values[2]).sqrt()
}

fn horizontal_accel_from_tilt(roll_rad: f32, pitch_rad: f32, yaw_rad: f32) -> [f32; 2] {
    let yaw_sin = yaw_rad.sin();
    let yaw_cos = yaw_rad.cos();
    [
        -STANDARD_GRAVITY_MPS2 * (yaw_sin * roll_rad + yaw_cos * pitch_rad),
        STANDARD_GRAVITY_MPS2 * (yaw_cos * roll_rad - yaw_sin * pitch_rad),
    ]
}

fn yaw_quaternion(yaw_rad: f32) -> [f32; 4] {
    let half_yaw = 0.5 * yaw_rad;
    [half_yaw.cos(), 0.0, 0.0, half_yaw.sin()]
}

fn euler_from_quaternion(q: [f32; 4]) -> (f32, f32, f32) {
    let [w, x, y, z] = q;
    let roll = f32::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = f32::asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    let yaw = f32::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    (roll, pitch, yaw)
}
