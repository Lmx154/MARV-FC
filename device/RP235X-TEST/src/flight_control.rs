use core::sync::atomic::{AtomicU32, Ordering};

use common::control::attitude::{AttitudeSetpoint, BodyRateSetpoint};
use common::control::mixing::{MotorOrder, MotorOutputs, TorqueCommand};
use common::control::pipeline::{
    ControlDebug, ControlInput, ControlSetpoint, EstimateSnapshot, FlightControlConfig,
    FlightControlPipeline, ImuControlInput,
};
use common::control::position::PositionControllerSetpoint;
use common::localization::navigation::{GeodeticPosition, LocalNedFrame};
use common::messages::control::{ActuatorOutputSample, ActuatorOutputStamped};
use common::messages::estimate::StateEstimateStamped;
use common::messages::sensor::{GpsFixSampleStamped, ImuSampleStamped};
use common::protocol::hilink::{GlobalWaypointPayload, MixerMotorOrderPayload};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::WaitResult;

use crate::channels::{
    ACTUATOR_OUTPUT_CHANNEL, GPS_CHANNEL, IMU_CHANNEL, STATE_ESTIMATE_CHANNEL, TestGpsSubscriber,
    TestImuSubscriber, TestStateEstimateSubscriber,
};
use crate::control_config::HIL_CONTROL_CONFIG;
use crate::protocol;

const CONTROL_COMMAND_DEPTH: usize = 4;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
const MIN_GPS_FIX_TYPE: u8 = 3;
const MIN_GPS_SATS: u8 = 4;

static CONTROL_COMMANDS: Channel<CriticalSectionRawMutex, ControlCommand, CONTROL_COMMAND_DEPTH> =
    Channel::new();
static MIXER_MOTOR_ORDER: AtomicU32 = AtomicU32::new(pack_motor_order([1, 2, 3, 4]));

#[derive(Clone, Copy, Debug)]
enum ControlCommand {
    Waypoint {
        position: GeodeticPosition,
        yaw_rad: f32,
    },
}

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub struct ControlStepOutput {
    pub actuator: ActuatorOutputStamped,
    pub attitude_setpoint: Option<AttitudeSetpoint>,
    pub rate_setpoint: Option<BodyRateSetpoint>,
    pub throttle: Option<f32>,
    pub torque: Option<TorqueCommand>,
    pub motor_outputs: Option<MotorOutputs<4>>,
}

pub fn spawn(spawner: &Spawner) {
    spawner
        .spawn(control_task(
            STATE_ESTIMATE_CHANNEL.subscriber().unwrap(),
            IMU_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
        ))
        .unwrap();
}

pub fn submit_control_waypoint(payload: GlobalWaypointPayload) -> bool {
    if !payload.lat_deg.is_finite()
        || !payload.lon_deg.is_finite()
        || !payload.alt_msl_m.is_finite()
        || !payload.yaw_deg.is_finite()
    {
        return false;
    }

    let queued = CONTROL_COMMANDS
        .try_send(ControlCommand::Waypoint {
            position: GeodeticPosition::new(payload.lat_deg, payload.lon_deg, payload.alt_msl_m),
            yaw_rad: payload.yaw_deg * DEG_TO_RAD,
        })
        .is_ok();

    if queued {
        info!(
            "rp235x-test queued control waypoint lat={=f64} lon={=f64} alt_msl={=f32} yaw_deg={=f32}",
            payload.lat_deg, payload.lon_deg, payload.alt_msl_m, payload.yaw_deg
        );
    }

    queued
}

pub fn submit_mixer_motor_order(payload: MixerMotorOrderPayload) -> bool {
    if MotorOrder::from_one_based(payload.output_for_motor).is_none() {
        return false;
    }

    MIXER_MOTOR_ORDER.store(
        pack_motor_order(payload.output_for_motor),
        Ordering::Relaxed,
    );
    info!(
        "rp235x-test mixer motor order set m1->out{=u8} m2->out{=u8} m3->out{=u8} m4->out{=u8}",
        payload.output_for_motor[0],
        payload.output_for_motor[1],
        payload.output_for_motor[2],
        payload.output_for_motor[3]
    );
    true
}

pub fn mixer_motor_order_payload() -> [u8; 4] {
    unpack_motor_order(MIXER_MOTOR_ORDER.load(Ordering::Relaxed))
}

#[embassy_executor::task]
async fn control_task(
    mut state_subscriber: TestStateEstimateSubscriber,
    mut imu_subscriber: TestImuSubscriber,
    mut gps_subscriber: TestGpsSubscriber,
) -> ! {
    let config = HIL_CONTROL_CONFIG;
    let mut pipeline = FlightControlPipeline::new(FlightControlConfig {
        loop_config: config,
    });
    let mut setpoint = PositionControllerSetpoint::ORIGIN_HOLD;
    let mut pending_waypoint = None;
    let mut local_frame = None;
    let mut latest_imu = None;

    loop {
        let estimate = match state_subscriber.next_message().await {
            WaitResult::Message(sample) => sample,
            WaitResult::Lagged(skipped) => {
                warn!(
                    "rp235x-test control skipped {=u64} state estimates",
                    skipped
                );
                latest_imu = None;
                continue;
            }
        };

        drain_latest_imu(&mut imu_subscriber, &mut latest_imu);
        drain_latest_gps(&mut gps_subscriber, &mut local_frame);
        drain_control_commands(&mut pending_waypoint);
        apply_pending_setpoint(&mut setpoint, &mut pending_waypoint, local_frame);
        pipeline.set_motor_order(current_motor_order());

        let output = actuator_output_for_estimate(estimate, latest_imu, setpoint, &pipeline);
        ACTUATOR_OUTPUT_CHANNEL
            .immediate_publisher()
            .publish_immediate(output);
    }
}

fn drain_control_commands(pending_waypoint: &mut Option<ControlCommand>) {
    while let Ok(command) = CONTROL_COMMANDS.try_receive() {
        *pending_waypoint = Some(command);
    }
}

fn apply_pending_setpoint(
    setpoint: &mut PositionControllerSetpoint,
    pending_waypoint: &mut Option<ControlCommand>,
    local_frame: Option<LocalNedFrame>,
) {
    let Some(command) = *pending_waypoint else {
        return;
    };
    let Some(local_frame) = local_frame else {
        return;
    };

    let ControlCommand::Waypoint { position, yaw_rad } = command;
    let Some(position_ned_m) = local_frame.position_ned_m(position) else {
        return;
    };
    *setpoint = PositionControllerSetpoint::new(position_ned_m, yaw_rad);
    *pending_waypoint = None;
    info!(
        "rp235x-test applied control waypoint ned_n={=f32} ned_e={=f32} ned_d={=f32} yaw_rad={=f32}",
        position_ned_m[0], position_ned_m[1], position_ned_m[2], yaw_rad
    );
}

fn drain_latest_imu(subscriber: &mut TestImuSubscriber, latest_imu: &mut Option<ImuSampleStamped>) {
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            *latest_imu = Some(sample);
        }
    }
}

fn drain_latest_gps(subscriber: &mut TestGpsSubscriber, local_frame: &mut Option<LocalNedFrame>) {
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            update_local_frame(sample, local_frame);
        }
    }
}

fn update_local_frame(sample: GpsFixSampleStamped, local_frame: &mut Option<LocalNedFrame>) {
    if sample.sample.fix_type < MIN_GPS_FIX_TYPE
        || sample.sample.sats < MIN_GPS_SATS
        || !finite_f32x3(sample.sample.vel_ned_mps)
    {
        return;
    }

    let position = GeodeticPosition::new(
        sample.sample.lat_deg,
        sample.sample.lon_deg,
        sample.sample.alt_m,
    );
    let _ = LocalNedFrame::from_first_valid(local_frame, position);
}

fn actuator_output_for_estimate(
    estimate: StateEstimateStamped,
    latest_imu: Option<ImuSampleStamped>,
    setpoint: PositionControllerSetpoint,
    pipeline: &FlightControlPipeline,
) -> ActuatorOutputStamped {
    control_step(
        protocol::is_armed(),
        estimate,
        latest_imu,
        setpoint,
        pipeline,
    )
    .actuator
}

pub fn control_step(
    armed: bool,
    estimate: StateEstimateStamped,
    latest_imu: Option<ImuSampleStamped>,
    setpoint: PositionControllerSetpoint,
    pipeline: &FlightControlPipeline,
) -> ControlStepOutput {
    let output = pipeline.step_input(ControlInput {
        estimate: EstimateSnapshot {
            position_ned_m: estimate.sample.position_ned_m,
            velocity_ned_mps: estimate.sample.velocity_ned_mps,
            quaternion: estimate.sample.attitude_quat,
            valid: estimate.sample.valid,
        },
        imu: latest_imu.map(|imu| ImuControlInput {
            accel_mps2: imu.sample.accel_mps2,
            gyro_rps: imu.sample.gyro_rad_s,
        }),
        setpoint: ControlSetpoint::from_position_setpoint(setpoint, armed),
    });

    control_step_output(
        estimate,
        output.motors,
        !armed || output.control_valid,
        output.clamped,
        output.debug,
    )
}

fn control_step_output(
    estimate: StateEstimateStamped,
    motor_command_normalized: [f32; 4],
    valid: bool,
    clamped: bool,
    debug: ControlDebug,
) -> ControlStepOutput {
    ControlStepOutput {
        actuator: actuator_output(estimate, motor_command_normalized, valid, clamped),
        attitude_setpoint: debug.attitude_setpoint,
        rate_setpoint: debug.rate_setpoint,
        throttle: debug.throttle,
        torque: debug.torque_command,
        motor_outputs: debug.motor_outputs,
    }
}

fn actuator_output(
    estimate: StateEstimateStamped,
    motor_command_normalized: [f32; 4],
    valid: bool,
    clamped: bool,
) -> ActuatorOutputStamped {
    ActuatorOutputStamped {
        timestamp: estimate.timestamp,
        sample: ActuatorOutputSample::new(motor_command_normalized, valid, clamped),
    }
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

const fn pack_motor_order(order: [u8; 4]) -> u32 {
    order[0] as u32
        | ((order[1] as u32) << 8)
        | ((order[2] as u32) << 16)
        | ((order[3] as u32) << 24)
}

fn unpack_motor_order(packed: u32) -> [u8; 4] {
    [
        (packed & 0xff) as u8,
        ((packed >> 8) & 0xff) as u8,
        ((packed >> 16) & 0xff) as u8,
        ((packed >> 24) & 0xff) as u8,
    ]
}

fn current_motor_order() -> MotorOrder {
    MotorOrder::from_one_based(mixer_motor_order_payload()).unwrap_or_default()
}
