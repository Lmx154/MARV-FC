use deterministic_harness::{ClosedLoopTruthState, ControlPipeline, ControlSetpoint, TruthPlant};

const DT_S: f32 = 0.01;
const TICKS: usize = 1_500;
const TAKEOFF_1_END_TICK: u64 = 250;
const TAKEOFF_2_END_TICK: u64 = 550;
const HOVER_END_TICK: u64 = 850;
const MIN_SPOOLDOWN_TICKS: u64 = 100;
const GROUND_DOWN_M: f32 = 0.0;
const DESCENT_SETPOINT_DOWN_M: f32 = 0.05;
const CONTACT_EPSILON_M: f32 = 0.01;
const ZERO_MOTOR_EPSILON: f32 = 0.001;
const MAX_VERTICAL_SPEED_MPS: f32 = 3.0;
const MAX_IMPACT_SPEED_MPS: f32 = 1.75;
const MAX_HOVER_ERROR_M: f32 = 0.55;
const MAX_CLAMP_RATIO: f32 = 0.02;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MissionStage {
    TakeoffOne,
    TakeoffTwo,
    Hover,
    Descent,
    Spooldown,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MissionTrace {
    tick: u64,
    stage: MissionStage,
    truth: ClosedLoopTruthState,
    motors: [f32; 4],
    armed: bool,
    control_valid: bool,
    clamped: bool,
    contact: bool,
    impact_speed_down_mps: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MissionSummary {
    first_contact_tick: Option<u64>,
    min_down_m: f32,
    max_hover_error_m: f32,
    max_vertical_speed_mps: f32,
    max_impact_speed_down_mps: f32,
    clamp_ratio: f32,
    pre_contact_zero_motor_frames: usize,
    post_contact_max_motor: f32,
    final_down_m: f32,
    final_vertical_speed_mps: f32,
    final_armed: bool,
    final_control_valid: bool,
}

#[test]
fn p15_takeoff_hover_land_stages_contact_and_spooldown() {
    let traces = run_takeoff_hover_land();
    let summary = summarize(&traces);
    let first_contact_tick = summary
        .first_contact_tick
        .expect("mission should reach ground contact");

    assert!(
        summary.min_down_m <= -1.55,
        "staged takeoff did not climb high enough: {summary:?}"
    );
    assert!(
        summary.max_hover_error_m <= MAX_HOVER_ERROR_M,
        "hover hold wandered outside altitude band: {summary:?}"
    );
    assert!(
        summary.max_vertical_speed_mps <= MAX_VERTICAL_SPEED_MPS,
        "vertical profile had an unsafe speed: {summary:?}"
    );
    assert!(
        summary.max_impact_speed_down_mps <= MAX_IMPACT_SPEED_MPS,
        "landing impact was too hard: {summary:?}"
    );
    assert!(
        summary.clamp_ratio <= MAX_CLAMP_RATIO,
        "mission saturated too often: {summary:?}"
    );
    assert_eq!(
        summary.pre_contact_zero_motor_frames, 0,
        "motors went idle before contact criteria were met: {summary:?}"
    );
    assert!(
        first_contact_tick + MIN_SPOOLDOWN_TICKS <= TICKS as u64,
        "mission did not leave enough post-contact spooldown evidence: {summary:?}"
    );
    assert!(
        summary.post_contact_max_motor <= ZERO_MOTOR_EPSILON,
        "motors did not spool down after contact: {summary:?}"
    );
    assert!(
        summary.final_down_m.abs() <= CONTACT_EPSILON_M,
        "landed state did not remain on the pad: {summary:?}"
    );
    assert!(
        summary.final_vertical_speed_mps.abs() <= f32::EPSILON,
        "landed state relaunched or fell through the pad: {summary:?}"
    );
    assert!(
        !summary.final_armed && !summary.final_control_valid,
        "final state should be disarmed zero-output: {summary:?}"
    );
}

fn run_takeoff_hover_land() -> Vec<MissionTrace> {
    let pipeline = ControlPipeline::default();
    let mut plant = TruthPlant::default();
    let mut traces = Vec::with_capacity(TICKS);
    let mut contact_latched = false;

    for index in 0..TICKS {
        let tick = (index + 1) as u64;
        let stage = mission_stage(tick, contact_latched);
        let setpoint = setpoint_for_stage(stage);
        let control = pipeline.step(plant.state.estimate(), plant.state.imu(), setpoint);
        let mut truth = plant.step(control.motors, DT_S);
        let mut contact = contact_latched;
        let mut impact_speed_down_mps = 0.0;

        if truth.position_ned_m[2] >= GROUND_DOWN_M && stage == MissionStage::Descent {
            contact = true;
            contact_latched = true;
            impact_speed_down_mps = truth.velocity_ned_mps[2].max(0.0);
            truth.position_ned_m[2] = GROUND_DOWN_M;
            truth.velocity_ned_mps[2] = 0.0;
            plant.state = truth;
        } else if contact_latched {
            contact = true;
            truth.position_ned_m[2] = GROUND_DOWN_M;
            truth.velocity_ned_mps[2] = 0.0;
            plant.state = truth;
        }

        traces.push(MissionTrace {
            tick,
            stage,
            truth,
            motors: control.motors,
            armed: control.armed,
            control_valid: control.control_valid,
            clamped: control.clamped,
            contact,
            impact_speed_down_mps,
        });
    }

    traces
}

fn mission_stage(tick: u64, contact_latched: bool) -> MissionStage {
    if contact_latched {
        MissionStage::Spooldown
    } else if tick <= TAKEOFF_1_END_TICK {
        MissionStage::TakeoffOne
    } else if tick <= TAKEOFF_2_END_TICK {
        MissionStage::TakeoffTwo
    } else if tick <= HOVER_END_TICK {
        MissionStage::Hover
    } else {
        MissionStage::Descent
    }
}

fn setpoint_for_stage(stage: MissionStage) -> ControlSetpoint {
    match stage {
        MissionStage::TakeoffOne => ControlSetpoint::new([0.0, 0.0, -1.0], 0.0, true),
        MissionStage::TakeoffTwo | MissionStage::Hover => {
            ControlSetpoint::new([0.0, 0.0, -2.0], 0.0, true)
        }
        MissionStage::Descent => {
            ControlSetpoint::new([0.0, 0.0, DESCENT_SETPOINT_DOWN_M], 0.0, true)
        }
        MissionStage::Spooldown => ControlSetpoint::new([0.0, 0.0, GROUND_DOWN_M], 0.0, false),
    }
}

fn summarize(traces: &[MissionTrace]) -> MissionSummary {
    assert!(!traces.is_empty());
    let first_contact_tick = traces
        .iter()
        .find(|trace| trace.contact)
        .map(|trace| trace.tick);
    let min_down_m = traces
        .iter()
        .map(|trace| trace.truth.position_ned_m[2])
        .fold(f32::INFINITY, f32::min);
    let max_hover_error_m = traces
        .iter()
        .filter(|trace| trace.stage == MissionStage::Hover)
        .map(|trace| (trace.truth.position_ned_m[2] + 2.0).abs())
        .fold(0.0, f32::max);
    let max_vertical_speed_mps = traces
        .iter()
        .map(|trace| trace.truth.velocity_ned_mps[2].abs())
        .fold(0.0, f32::max);
    let max_impact_speed_down_mps = traces
        .iter()
        .map(|trace| trace.impact_speed_down_mps)
        .fold(0.0, f32::max);
    let clamped_frames = traces.iter().filter(|trace| trace.clamped).count();
    let pre_contact_zero_motor_frames = traces
        .iter()
        .take_while(|trace| !trace.contact)
        .filter(|trace| max_motor(trace.motors) <= ZERO_MOTOR_EPSILON)
        .count();
    let post_contact_max_motor = first_contact_tick
        .map(|contact_tick| {
            traces
                .iter()
                .filter(|trace| trace.tick > contact_tick)
                .map(|trace| max_motor(trace.motors))
                .fold(0.0, f32::max)
        })
        .unwrap_or(1.0);
    let final_trace = traces.last().unwrap();

    MissionSummary {
        first_contact_tick,
        min_down_m,
        max_hover_error_m,
        max_vertical_speed_mps,
        max_impact_speed_down_mps,
        clamp_ratio: clamped_frames as f32 / traces.len() as f32,
        pre_contact_zero_motor_frames,
        post_contact_max_motor,
        final_down_m: final_trace.truth.position_ned_m[2],
        final_vertical_speed_mps: final_trace.truth.velocity_ned_mps[2],
        final_armed: final_trace.armed,
        final_control_valid: final_trace.control_valid,
    }
}

fn max_motor(motors: [f32; 4]) -> f32 {
    motors.into_iter().fold(0.0, f32::max)
}
