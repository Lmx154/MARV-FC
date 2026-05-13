mod support;

use common::{
    control::{
        altitude::AltitudeControllerConfig,
        attitude::AttitudeSetpoint,
        config::ControlLoopConfig,
        mixing::{
            DesaturationPriority, MixerLimits, MotorOrder, PhysicalControlAllocator, QuadXMixer,
            TorqueCommand,
        },
        pipeline::{
            ControlInput, ControlOutput, ControlSetpoint, ControlSetpointSource, EstimateSnapshot,
            FlightControlConfig, FlightControlPipeline, ImuControlInput, LocalTrajectorySetpoint,
        },
        position::PositionControllerConfig,
        rate::RateControllerConfig,
    },
    localization::navigation::{GeodeticPosition, LocalNedFrame},
    messages::{control::ActuatorOutputSample, estimate::StateEstimateSample},
    protocol::hilink::{SimStamp, response_flags, valid},
    services::hil::HilSensorFrameRejection,
};
use deterministic_harness::{
    GazeboActuatorExpectation, GazeboAirframeConfig, GazeboBridgeConfig, GazeboSensorLine,
    GazeboSimControlAction, GazeboTruthOrigin, HilSemanticAdapter, HilSemanticFrameConfig,
    SensorFrame, format_gazebo_actuator_line, format_gazebo_sim_control_line,
    gazebo_g0_actuator_truth_cases, hil_response_from_samples, response_has_flag,
    response_is_failsafe_zero_output, sensor_frame_to_hil_frame, stale_hil_response_from_samples,
};
use support::{
    AcceptanceThresholds, CommandKind, CommandSchedule, FailureLayer, Gate, ScenarioMetrics,
    ScenarioReport, ScenarioSpec, ScheduledCommand,
};

const BRIDGE_CONFIG: &str =
    include_str!("../../../telemetry-app/gazebo_bridge/config/bridge_config");
const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");

#[test]
fn shared_scenario_report_shape_classifies_runtime_failures() {
    let gates = [
        Gate::C0,
        Gate::G0,
        Gate::G1,
        Gate::G2,
        Gate::G2Point5,
        Gate::G3,
    ];
    assert_eq!(gates.len(), 6);
    let command_kinds = [
        CommandKind::Hold,
        CommandKind::LocalPosition,
        CommandKind::LocalTrajectory,
        CommandKind::ActuatorMotors,
        CommandKind::SimReset,
        CommandKind::SimPlay,
    ];
    assert_eq!(command_kinds.len(), 6);

    let spec = ScenarioSpec {
        id: "g25_lateral_brake",
        gate: Gate::G2Point5,
        description: "move, brake, and return using shared runtime metrics",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 10,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: 50,
                kind: CommandKind::LocalPosition,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.45,
            max_altitude_error_m: 0.75,
            max_roll_pitch_rad: 0.35,
            max_speed_error_mps: 0.30,
            max_clamp_ratio: 0.10,
        },
    };

    let report = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.10,
            max_altitude_error_m: 0.20,
            max_roll_pitch_rad: 0.10,
            max_speed_error_mps: 0.20,
            clamp_ratio: 0.15,
            first_clamp_source: Some("motor_output_limit".to_string()),
        },
    );

    assert!(!report.pass);
    assert_eq!(report.id, spec.id);
    assert_eq!(report.failure_layer, Some(FailureLayer::Actuator));
    assert_eq!(
        report.first_clamp_source.as_deref(),
        Some("motor_output_limit")
    );
}

#[test]
fn c0_frame_quaternion_and_mixer_basis_are_frozen() {
    let frame = LocalNedFrame::new(GeodeticPosition::new(30.0, -97.0, 120.0));
    let nearby = frame
        .position_ned_m(GeodeticPosition::new(30.000_01, -96.999_99, 125.0))
        .expect("finite position maps into local NED");

    assert!(nearby[0] > 0.0, "positive latitude offset is north");
    assert!(nearby[1] > 0.0, "positive longitude offset is east");
    assert!(nearby[2] < 0.0, "higher altitude is negative down");
    assert_eq!(AttitudeSetpoint::LEVEL.quaternion, [1.0, 0.0, 0.0, 0.0]);

    let mixer = QuadXMixer::with_motor_order(
        Default::default(),
        MotorOrder::from_one_based([1, 2, 3, 4]).expect("identity order is valid"),
    );
    let outputs = mixer.mix(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));
    assert_commands_near(outputs.commands, [0.75, 0.65, 0.15, 0.45], 0.000_001);

    let allocator = PhysicalControlAllocator::default();
    for command in [
        TorqueCommand::new(0.1, 0.0, 0.0, 0.5),
        TorqueCommand::new(0.0, 0.1, 0.0, 0.5),
        TorqueCommand::new(0.0, 0.0, 0.1, 0.5),
    ] {
        assert_eq!(
            allocator.allocate(command).commands,
            QuadXMixer::default().mix(command).commands
        );
    }
}

#[test]
fn c0_failsafe_zero_outputs_cover_invalid_disarmed_and_missing_inputs() {
    let invalid_estimate = FlightControlPipeline::default().step_input(ControlInput::new(
        EstimateSnapshot {
            valid: false,
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    ));
    assert_failsafe_zero(invalid_estimate, true);

    let missing_imu = FlightControlPipeline::default().step_input(ControlInput::without_imu(
        EstimateSnapshot::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    ));
    assert_failsafe_zero(missing_imu, true);

    let non_finite_setpoint = FlightControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, f32::NAN, 0.0], 0.0, true),
    );
    assert_failsafe_zero(non_finite_setpoint, true);

    let disarmed = FlightControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false),
    );
    assert_failsafe_zero(disarmed, false);
}

#[test]
fn c0_control_signs_altitude_direction_and_trajectory_tags_are_preserved() {
    let roll = origin_step(
        estimate_with_euler_deg(10.0, 0.0, 0.0),
        ImuControlInput::default(),
    );
    assert!(roll.rate_setpoint.expect("rate setpoint").roll_rps < 0.0);
    assert!(roll.torque_command.roll < 0.0);

    let pitch = origin_step(
        estimate_with_euler_deg(0.0, 10.0, 0.0),
        ImuControlInput::default(),
    );
    assert!(pitch.rate_setpoint.expect("rate setpoint").pitch_rps < 0.0);
    assert!(pitch.torque_command.pitch < 0.0);

    let yaw = origin_step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0; 3],
            gyro_rps: [0.0, 0.0, 1.0],
        },
    );
    assert!(yaw.torque_command.yaw < 0.0);

    let pipeline = FlightControlPipeline::default();
    let below_setpoint = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, -2.0], 0.0, true),
    );
    let above_setpoint = pipeline.step(
        EstimateSnapshot {
            position_ned_m: [0.0, 0.0, -2.0],
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(below_setpoint.throttle > 0.5);
    assert!(above_setpoint.throttle < 0.5);

    let trajectory = LocalTrajectorySetpoint {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [1.0, 0.0, 0.0],
        acceleration_ned_mps2: [0.2, 0.0, 0.0],
        yaw_rad: 0.0,
        yaw_rate_rad_s: 0.0,
    };
    let setpoint = ControlSetpoint::from_local_trajectory(trajectory, true);
    let trace = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        setpoint,
    );
    assert_eq!(
        setpoint.source(),
        ControlSetpointSource::EstimatorLocalTrajectory
    );
    assert_eq!(trace.debug.velocity_setpoint_ned_mps, Some([0.5, 0.0]));
    assert_eq!(trace.debug.feed_forward_accel_ned_mps2, Some([0.2, 0.0]));
    assert!(
        trace
            .attitude_setpoint
            .expect("attitude setpoint")
            .quaternion[2]
            < 0.0
    );
}

#[test]
fn c0_limit_visibility_demand_limits_and_tilt_compensation_are_preserved() {
    let mut saturated_config = ControlLoopConfig::default();
    saturated_config.rate = RateControllerConfig {
        roll_gain: 1.0,
        pitch_gain: 1.0,
        yaw_gain: 1.0,
        max_axis_command: 0.1,
        measured_rate_deadband_rps: 0.0,
    };
    saturated_config.mixer_limits = MixerLimits::new(0.45, 0.55);
    let saturated =
        FlightControlPipeline::new(FlightControlConfig::with_loop_config(saturated_config)).step(
            EstimateSnapshot::LEVEL_ORIGIN,
            ImuControlInput {
                accel_mps2: [0.0; 3],
                gyro_rps: [-1.0, 1.0, -1.0],
            },
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        );

    assert!(saturated.control_valid);
    assert!(saturated.clamped);
    assert!(saturated.debug.rate_limit_flags.roll);
    assert!(saturated.debug.rate_limit_flags.pitch);
    assert!(saturated.debug.rate_limit_flags.yaw);
    assert!(saturated.debug.mixer_limit_flags.any());
    assert_eq!(
        saturated.debug.desaturation.priority,
        DesaturationPriority::PreserveRollPitchOverYaw
    );
    assert!(saturated.motors.iter().all(|motor| {
        (saturated_config.mixer_limits.min..=saturated_config.mixer_limits.max).contains(motor)
    }));

    let config = demand_limit_config();
    let output = FlightControlPipeline::new(FlightControlConfig::with_loop_config(config)).step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned(
            [1_000.0, -1_000.0, -100.0],
            90.0_f32.to_radians(),
            true,
        ),
    );
    assert!(output.control_valid);
    assert!(
        output
            .motors
            .iter()
            .all(|motor| (0.0..=1.0).contains(motor))
    );
    assert!(
        output
            .debug
            .applied_accel_ned_mps2
            .expect("applied horizontal accel")[0]
            .abs()
            <= config.position.max_horizontal_accel_mps2 + 0.001
    );
    assert!(
        (output.throttle - config.altitude.hover_throttle).abs()
            <= config.altitude.max_throttle_correction + 0.000_001
    );
    let axis_command = output
        .torque_command
        .roll
        .abs()
        .max(output.torque_command.pitch.abs())
        .max(output.torque_command.yaw.abs());
    assert!(axis_command <= config.rate.max_axis_command + 0.000_001);

    let tilted = FlightControlPipeline::new(FlightControlConfig::with_loop_config(config)).step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            LocalTrajectorySetpoint {
                position_ned_m: [0.0, 0.0, 0.0],
                velocity_ned_mps: [0.0, 0.0, 0.0],
                acceleration_ned_mps2: [config.position.max_horizontal_accel_mps2, 0.0, 0.0],
                yaw_rad: 0.0,
                yaw_rate_rad_s: 0.0,
            },
            true,
        ),
    );
    assert!(tilted.debug.tilt_compensation.expect("tilt compensation") > 1.0);
    assert!(
        tilted
            .debug
            .requested_collective_throttle
            .expect("requested collective")
            > config.altitude.hover_throttle
    );
    assert!(
        tilted
            .debug
            .vertical_throttle_margin
            .expect("vertical margin")
            > 0.0
    );
}

#[test]
fn c0_hil_semantics_keep_rejection_validity_and_response_contracts() {
    let mut adapter = HilSemanticAdapter::active();
    let first = sensor_frame_to_hil_frame(2, &sensor_at(20_000), HilSemanticFrameConfig::default());
    let duplicate =
        sensor_frame_to_hil_frame(2, &sensor_at(30_000), HilSemanticFrameConfig::default());

    assert!(adapter.accept_frame(first).accepted);
    let duplicate_dispatch = adapter.accept_frame(duplicate);
    assert!(!duplicate_dispatch.accepted);
    assert_eq!(
        duplicate_dispatch.rejection,
        Some(HilSensorFrameRejection::DuplicateTick)
    );

    let mut partial = sensor_at(40_000);
    partial.mag_body_ut = None;
    partial.gps_position_ned_m = None;
    partial.gps_velocity_ned_mps = None;
    partial.baro_down_m = None;
    let mut partial_adapter = HilSemanticAdapter::active();
    let partial_dispatch =
        partial_adapter.accept_sensor_frame(3, &partial, HilSemanticFrameConfig::default());
    assert!(partial_dispatch.accepted);
    assert_eq!(partial_adapter.published_groups().imu, 1);
    assert_eq!(partial_adapter.published_groups().gps, 0);
    assert_eq!(partial_adapter.published_groups().magnetometer, 0);

    let mut invalid =
        sensor_frame_to_hil_frame(4, &sensor_at(50_000), HilSemanticFrameConfig::default());
    invalid.gyro_rps[0] = f32::NAN;
    let invalid_dispatch = HilSemanticAdapter::active().accept_frame(invalid);
    assert!(!invalid_dispatch.accepted);
    assert_eq!(
        invalid_dispatch.rejection,
        Some(HilSensorFrameRejection::InvalidSample)
    );

    let stamp = SimStamp {
        sim_tick: 5,
        sim_time_us: 60_000,
    };
    let estimate = StateEstimateSample {
        position_ned_m: [0.0, 0.0, -1.0],
        velocity_ned_mps: [0.0; 3],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        valid: true,
    };
    let actuator = ActuatorOutputSample::new([0.25, 0.25, 0.25, 0.25], true, false);
    let response = hil_response_from_samples(stamp, true, true, Some(estimate), Some(actuator));
    assert!(response_has_flag(
        response,
        response_flags::SENSOR_INPUT_VALID
    ));
    assert!(response_has_flag(response, response_flags::ESTIMATOR_VALID));
    assert!(response_has_flag(response, response_flags::CONTROL_VALID));
    assert!(response_has_flag(response, response_flags::MOTORS_VALID));
    assert!(!response_has_flag(response, response_flags::FAILSAFE));

    let stale =
        stale_hil_response_from_samples(stamp, 1, true, true, Some(estimate), Some(actuator));
    assert!(response_has_flag(stale, response_flags::FAILSAFE));
    assert!(response_is_failsafe_zero_output(stale));
}

#[test]
fn c0_gazebo_bridge_config_sensor_and_actuator_contracts_are_preserved() {
    let line = "SENSOR seq=42 sim_time_us=840000 clock=gazebo valid=31 ax=1.0 ay=-2.0 az=3.5 gx=0.1 gy=-0.2 gz=0.3 qw=0.996 qx=0.087 qy=0.0 qz=0.0 roll=0.1745329 pitch=0.0 yaw=0.0 pn=1.25 pe=-2.5 pd=0.75 mx=20.5 my=-3.5 mz=44.25 pressure_pa=100123.25 baro_alt_m=101.5 temp_c=18.75 lat_deg=26.310942 lon_deg=-98.174728 alt_msl_m=99.5 vn=4.5 ve=-5.5 vd=6.5 sats=13 fix=3 battery_v=15.2 rssi_dbm=-42 snr_db_x100=725 loss_pct_x100=12";
    let gazebo = GazeboSensorLine::parse(line).expect("sensor line should parse");
    let hil = gazebo.to_hil_sensor_frame();

    assert_eq!(hil.stamp.sim_tick, 42);
    assert_eq!(hil.stamp.sim_time_us, 840_000);
    assert_eq!(
        hil.valid_flags,
        valid::ACCEL | valid::GYRO | valid::MAG | valid::BARO | valid::GPS
    );
    assert_eq!(hil.accel_mps2, [1.0, -2.0, 3.5]);
    assert_eq!(hil.gyro_rps, [0.1, -0.2, 0.3]);
    assert_eq!(gazebo.position_ned_m, Some([1.25, -2.5, 0.75]));

    let origin = GazeboTruthOrigin::from_frame(&gazebo).expect("origin should be finite");
    let next = GazeboSensorLine::parse(
        "SENSOR seq=43 sim_time_us=860000 valid=31 pn=2.25 pe=-4.5 pd=-0.25 lat_deg=26.310952 lon_deg=-98.174718 alt_msl_m=100.5 baro_alt_m=102.0 vn=1 ve=-2 vd=0.5 ax=0 ay=-1.7029068 az=-9.657665 gx=0.1 gy=-0.2 gz=0.3 qw=0.9961947 qx=0.0871557 qy=0 qz=0 mx=21 my=1 mz=41",
    )
    .expect("next line should parse");
    let truth = next
        .to_truth_estimate(origin)
        .expect("truth estimate should convert");
    assert_eq!(truth.position_ned_m, [1.0, -2.0, -1.0]);
    let sensor = next
        .to_estimator_sensor_frame(origin)
        .expect("estimator frame should convert");
    assert_near(sensor.accel_mps2[0] as f32, 0.0, 0.000_1);
    assert_near(sensor.accel_mps2[1] as f32, 0.0, 0.000_1);
    assert_near(sensor.accel_mps2[2] as f32, 0.0, 0.000_1);

    let bridge = GazeboBridgeConfig::parse(BRIDGE_CONFIG).expect("bridge config parses");
    let airframe = airframe_config();
    assert_eq!(bridge.clock_topic, "/world/marv_field/clock");
    assert_eq!(bridge.motor_direction_mode, "gazebo_model");
    assert_eq!(bridge.motor_gazebo_indices, [1, 2, 3, 0]);
    assert_near(
        bridge.motor_speed_rad_s(0, airframe.hover_motor_command()) as f32,
        630.0,
        0.01,
    );
    assert_eq!(
        bridge.gazebo_motor_speeds_rad_s([1.0, 0.0, 0.0, 0.0]),
        [0.0, 1100.0, 0.0, 0.0]
    );
    assert_eq!(
        format_gazebo_actuator_line(42, Some(840_000), [0.0, 1.0, 1.0, 0.0]),
        "ACTUATOR seq=42 sim_time_us=840000 m0=0.000 m1=1.000 m2=1.000 m3=0.000\n"
    );
    assert_eq!(
        format_gazebo_sim_control_line(7, GazeboSimControlAction::Reset),
        "SIM_CONTROL seq=7 action=reset\n"
    );

    let cases = gazebo_g0_actuator_truth_cases(&airframe);
    assert_eq!(cases.len(), 12);
    assert_eq!(cases[1].name, "hover");
    assert_commands_near(
        cases[1].motors,
        [0.572_727, 0.572_727, 0.572_727, 0.572_727],
        0.000_001,
    );
    assert_eq!(
        cases[2].expectation,
        GazeboActuatorExpectation::MotorBump {
            motor: 1,
            roll_positive: false,
            pitch_positive: true,
            yaw_positive: true,
        }
    );
}

fn origin_step(estimate: EstimateSnapshot, imu: ImuControlInput) -> ControlOutput {
    FlightControlPipeline::default().step(estimate, imu, ControlSetpoint::ORIGIN_HOLD_ARMED)
}

fn estimate_with_euler_deg(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> EstimateSnapshot {
    let to_rad = core::f32::consts::PI / 180.0;
    EstimateSnapshot {
        quaternion: AttitudeSetpoint::from_euler_rad(
            roll_deg * to_rad,
            pitch_deg * to_rad,
            yaw_deg * to_rad,
        )
        .expect("finite euler angles should produce quaternion")
        .quaternion,
        ..EstimateSnapshot::LEVEL_ORIGIN
    }
}

fn demand_limit_config() -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.position = PositionControllerConfig {
        position_gain: 8.0,
        velocity_gain: 0.0,
        max_horizontal_velocity_mps: 100.0,
        horizontal_integral_gain: 0.0,
        horizontal_integral_leak: 0.0,
        max_horizontal_integral_accel_mps2: 0.0,
        max_horizontal_accel_mps2: 0.8,
        max_horizontal_accel_slew_mps3: 0.0,
        max_tilt_rad: 7.0_f32.to_radians(),
    };
    config.altitude = AltitudeControllerConfig {
        hover_throttle: 0.5,
        altitude_gain: 4.0,
        vertical_velocity_gain: 0.0,
        max_altitude_error_m: 100.0,
        max_vertical_velocity_mps: 100.0,
        max_throttle_correction: 0.10,
    };
    config.rate = RateControllerConfig {
        roll_gain: 1.0,
        pitch_gain: 1.0,
        yaw_gain: 1.0,
        max_axis_command: 0.12,
        measured_rate_deadband_rps: 0.0,
    };
    config
}

fn sensor_at(timestamp_us: u64) -> SensorFrame {
    SensorFrame {
        timestamp_us,
        accel_mps2: [0.0, 0.0, 0.0],
        gyro_rps: [0.0, 0.0, 0.0],
        gravity_body_mps2: None,
        mag_body_ut: Some([20.0, 0.0, 40.0]),
        gps_position_ned_m: Some([0.0, 0.0, 0.0]),
        gps_velocity_ned_mps: Some([0.0, 0.0, 0.0]),
        baro_down_m: Some(0.0),
    }
}

fn airframe_config() -> GazeboAirframeConfig {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config is physically self-consistent");
    airframe
}

fn assert_failsafe_zero(output: ControlOutput, armed: bool) {
    assert_eq!(output.armed, armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
    assert!(!output.clamped);
    assert!(output.attitude_setpoint.is_none());
    assert!(output.rate_setpoint.is_none());
}

fn assert_commands_near(actual: [f32; 4], expected: [f32; 4], tolerance: f32) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_near(actual, expected, tolerance);
    }
}

fn assert_near(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {actual} to be within {tolerance} of {expected}"
    );
}
