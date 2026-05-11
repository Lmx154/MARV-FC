//! Failsafe policy rules.

use crate::messages::control::{ActuatorOutputSample, ActuatorOutputStamped};
use crate::messages::estimate::{StateEstimateSample, StateEstimateStamped};
use crate::protocol::hilink::{HilResponseFrame, SimStamp, response_flags};
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HilResponseHealth {
    pub flags: u32,
    pub motor_command_normalized: [f32; 4],
}

pub fn evaluate_hil_response_health(
    armed: bool,
    estimator_valid: bool,
    actuator: Option<ActuatorOutputSample>,
) -> HilResponseHealth {
    let actuator_valid = actuator.is_some_and(|sample| sample.valid);
    let failsafe = armed && (!estimator_valid || !actuator_valid);
    let mut flags = 0;

    if armed {
        flags |= response_flags::ARMED;
    }
    if estimator_valid {
        flags |= response_flags::ESTIMATOR_VALID;
    }
    if let Some(actuator) = actuator {
        if actuator.valid {
            flags |= response_flags::CONTROL_VALID;
            if actuator.clamped {
                flags |= response_flags::CONTROL_CLAMPED;
            }
        }
    }
    if failsafe {
        flags |= response_flags::FAILSAFE | response_flags::MOTORS_VALID;
        return HilResponseHealth {
            flags,
            motor_command_normalized: [0.0; 4],
        };
    }

    if let Some(actuator) = actuator {
        if actuator.valid {
            flags |= response_flags::MOTORS_VALID;
            return HilResponseHealth {
                flags,
                motor_command_normalized: actuator.motor_command_normalized,
            };
        }
    }

    if !armed {
        flags |= response_flags::MOTORS_VALID;
    }

    HilResponseHealth {
        flags,
        motor_command_normalized: [0.0; 4],
    }
}

pub fn build_hil_response_frame(
    stamp: SimStamp,
    system_state: u8,
    armed: bool,
    sensor_input_valid: bool,
    latest_estimate: Option<StateEstimateStamped>,
    latest_actuator: Option<ActuatorOutputStamped>,
) -> HilResponseFrame {
    let response_timestamp = MeasurementTimestamp::from_micros(stamp.sim_time_us);
    let estimate = latest_estimate
        .filter(|estimate| estimate.timestamp >= response_timestamp)
        .map(|estimate| estimate.sample)
        .unwrap_or(StateEstimateSample::NEUTRAL_INVALID);
    let actuator = latest_actuator
        .filter(|actuator| actuator.timestamp >= response_timestamp)
        .map(|actuator| actuator.sample);
    let health = evaluate_hil_response_health(armed, estimate.valid, actuator);
    let mut flags = health.flags;
    if sensor_input_valid {
        flags |= response_flags::SENSOR_INPUT_VALID;
    }

    HilResponseFrame {
        stamp,
        system_state,
        reserved0: [0; 3],
        flags,
        position_ned_m: estimate.position_ned_m,
        velocity_ned_mps: estimate.velocity_ned_mps,
        attitude_quat: estimate.attitude_quat,
        motor_cmd: normalized_motor_commands(health.motor_command_normalized),
    }
}

fn normalized_motor_commands(motor_command_normalized: [f32; 4]) -> [u16; 4] {
    [
        normalized_f32_to_u16(motor_command_normalized[0]),
        normalized_f32_to_u16(motor_command_normalized[1]),
        normalized_f32_to_u16(motor_command_normalized[2]),
        normalized_f32_to_u16(motor_command_normalized[3]),
    ]
}

fn normalized_f32_to_u16(value: f32) -> u16 {
    if !value.is_finite() || value <= 0.0 {
        return 0;
    }
    if value >= 1.0 {
        return u16::MAX;
    }

    (value * f32::from(u16::MAX)) as u16
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    use super::{build_hil_response_frame, evaluate_hil_response_health};
    use crate::messages::control::{ActuatorOutputSample, ActuatorOutputStamped};
    use crate::messages::estimate::{StateEstimateSample, StateEstimateStamped};
    use crate::protocol::hilink::{HilSensorFrame, SimStamp, response_flags, valid};
    use crate::services::acquisition::{ImuSampleChannel, TimeSampleChannel};
    use crate::services::hil::routing::HilIngressRoutes;
    use crate::services::hil::{HilSensorFrameAdapter, HilSensorFrameRejection};
    use crate::utilities::time::MeasurementTimestamp;

    const TEST_STATE_ARMED: u8 = 2;

    fn stamp(time_us: u64) -> SimStamp {
        SimStamp {
            sim_tick: time_us / 1_000,
            sim_time_us: time_us,
        }
    }

    fn timestamp(time_us: u64) -> MeasurementTimestamp {
        MeasurementTimestamp::from_micros(time_us)
    }

    fn valid_estimate(time_us: u64) -> StateEstimateStamped {
        StateEstimateStamped {
            timestamp: timestamp(time_us),
            sample: StateEstimateSample {
                position_ned_m: [1.0, 2.0, 3.0],
                velocity_ned_mps: [0.1, 0.2, 0.3],
                attitude_quat: [1.0, 0.0, 0.0, 0.0],
                valid: true,
            },
        }
    }

    fn valid_actuator(time_us: u64, clamped: bool) -> ActuatorOutputStamped {
        ActuatorOutputStamped {
            timestamp: timestamp(time_us),
            sample: ActuatorOutputSample::new([0.25, 0.5, 0.75, 1.0], true, clamped),
        }
    }

    fn valid_sensor_frame(time_us: u64) -> HilSensorFrame {
        HilSensorFrame {
            stamp: stamp(time_us),
            valid_flags: valid::ACCEL | valid::GYRO,
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rps: [0.1, 0.2, 0.3],
            ..HilSensorFrame::default()
        }
    }

    #[test]
    fn healthy_armed_response_passes_motor_outputs() {
        let health = evaluate_hil_response_health(
            true,
            true,
            Some(ActuatorOutputSample::new([0.1, 0.2, 0.3, 0.4], true, false)),
        );

        assert_eq!(
            health.flags,
            response_flags::ARMED
                | response_flags::ESTIMATOR_VALID
                | response_flags::MOTORS_VALID
                | response_flags::CONTROL_VALID
        );
        assert_eq!(health.motor_command_normalized, [0.1, 0.2, 0.3, 0.4]);
    }

    #[test]
    fn armed_invalid_estimator_enters_valid_zero_motor_failsafe() {
        let health = evaluate_hil_response_health(
            true,
            false,
            Some(ActuatorOutputSample::new([0.1, 0.2, 0.3, 0.4], true, false)),
        );

        assert_eq!(
            health.flags,
            response_flags::ARMED
                | response_flags::FAILSAFE
                | response_flags::MOTORS_VALID
                | response_flags::CONTROL_VALID
        );
        assert_eq!(health.motor_command_normalized, [0.0; 4]);
    }

    #[test]
    fn armed_invalid_actuator_enters_valid_zero_motor_failsafe() {
        let health = evaluate_hil_response_health(
            true,
            true,
            Some(ActuatorOutputSample::new([0.0; 4], false, false)),
        );

        assert_eq!(
            health.flags,
            response_flags::ARMED
                | response_flags::FAILSAFE
                | response_flags::ESTIMATOR_VALID
                | response_flags::MOTORS_VALID
        );
        assert_eq!(health.motor_command_normalized, [0.0; 4]);
    }

    #[test]
    fn clamped_control_output_is_flagged() {
        let health = evaluate_hil_response_health(
            true,
            true,
            Some(ActuatorOutputSample::new([1.0, 0.0, 0.5, 0.5], true, true)),
        );

        assert_eq!(
            health.flags,
            response_flags::ARMED
                | response_flags::ESTIMATOR_VALID
                | response_flags::MOTORS_VALID
                | response_flags::CONTROL_VALID
                | response_flags::CONTROL_CLAMPED
        );
        assert_eq!(health.motor_command_normalized, [1.0, 0.0, 0.5, 0.5]);
    }

    #[test]
    fn disarmed_missing_actuator_is_valid_zero_output_without_failsafe() {
        let health = evaluate_hil_response_health(false, false, None);

        assert_eq!(health.flags, response_flags::MOTORS_VALID);
        assert_eq!(health.motor_command_normalized, [0.0; 4]);
    }

    #[test]
    fn stale_estimator_sample_does_not_satisfy_response() {
        let response = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            true,
            Some(valid_estimate(1_000)),
            Some(valid_actuator(2_000, false)),
        );

        assert_eq!(
            response.position_ned_m,
            StateEstimateSample::NEUTRAL_INVALID.position_ned_m
        );
        assert_eq!(
            response.velocity_ned_mps,
            StateEstimateSample::NEUTRAL_INVALID.velocity_ned_mps
        );
        assert_eq!(
            response.attitude_quat,
            StateEstimateSample::NEUTRAL_INVALID.attitude_quat
        );
        assert_eq!(response.motor_cmd, [0; 4]);
        assert_eq!(
            response.flags,
            response_flags::ARMED
                | response_flags::FAILSAFE
                | response_flags::MOTORS_VALID
                | response_flags::CONTROL_VALID
                | response_flags::SENSOR_INPUT_VALID
        );
    }

    #[test]
    fn stale_actuator_sample_does_not_satisfy_response() {
        let response = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            true,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(1_000, false)),
        );

        assert_eq!(response.motor_cmd, [0; 4]);
        assert_eq!(
            response.flags,
            response_flags::ARMED
                | response_flags::FAILSAFE
                | response_flags::ESTIMATOR_VALID
                | response_flags::MOTORS_VALID
                | response_flags::SENSOR_INPUT_VALID
        );
    }

    #[test]
    fn accepted_hil_frame_sets_sensor_input_valid() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();
        let sensors = valid_sensor_frame(2_000);

        let dispatch = adapter.accept_frame(sensors, &routes);
        let response = build_hil_response_frame(
            sensors.stamp,
            TEST_STATE_ARMED,
            true,
            dispatch.accepted && dispatch.dispatch.imu_published,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, false)),
        );

        assert!(dispatch.accepted);
        assert!(dispatch.dispatch.imu_published);
        assert_ne!(response.flags & response_flags::SENSOR_INPUT_VALID, 0);
    }

    #[test]
    fn rejected_hil_frame_clears_sensor_input_valid() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::new();
        let sensors = valid_sensor_frame(2_000);

        let dispatch = adapter.accept_frame(sensors, &routes);
        let response = build_hil_response_frame(
            sensors.stamp,
            TEST_STATE_ARMED,
            true,
            dispatch.accepted && dispatch.dispatch.imu_published,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, false)),
        );

        assert!(!dispatch.accepted);
        assert_eq!(response.flags & response_flags::SENSOR_INPUT_VALID, 0);
    }

    #[test]
    fn clamped_actuator_output_sets_control_clamped() {
        let response = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            true,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, true)),
        );

        assert_ne!(response.flags & response_flags::CONTROL_CLAMPED, 0);
        assert_eq!(response.motor_cmd, [16_383, 32_767, 49_151, u16::MAX]);
    }

    #[test]
    fn normalized_motor_commands_scale_to_u16_with_truncation() {
        let response = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            false,
            Some(valid_estimate(2_000)),
            Some(ActuatorOutputStamped {
                timestamp: timestamp(2_000),
                sample: ActuatorOutputSample::new([0.0, 0.5, 0.999_99, 1.0], true, false),
            }),
        );

        assert_eq!(response.motor_cmd, [0, 32_767, 65_534, u16::MAX]);
    }

    #[test]
    fn non_finite_negative_and_zero_motor_commands_encode_as_zero() {
        let response = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            false,
            Some(valid_estimate(2_000)),
            Some(ActuatorOutputStamped {
                timestamp: timestamp(2_000),
                sample: ActuatorOutputSample::new([f32::NAN, -0.1, 0.0, 1.2], true, false),
            }),
        );

        assert_eq!(response.motor_cmd, [0, 0, 0, u16::MAX]);
    }

    #[test]
    fn stale_estimate_and_actuator_timestamp_truth_table() {
        let cases = [
            (
                "fresh estimate and fresh actuator",
                Some(valid_estimate(2_000)),
                Some(valid_actuator(2_000, false)),
                response_flags::ARMED
                    | response_flags::ESTIMATOR_VALID
                    | response_flags::CONTROL_VALID
                    | response_flags::MOTORS_VALID,
                [16_383, 32_767, 49_151, u16::MAX],
            ),
            (
                "stale estimate and fresh actuator",
                Some(valid_estimate(1_999)),
                Some(valid_actuator(2_000, false)),
                response_flags::ARMED
                    | response_flags::CONTROL_VALID
                    | response_flags::FAILSAFE
                    | response_flags::MOTORS_VALID,
                [0; 4],
            ),
            (
                "fresh estimate and stale actuator",
                Some(valid_estimate(2_000)),
                Some(valid_actuator(1_999, false)),
                response_flags::ARMED
                    | response_flags::ESTIMATOR_VALID
                    | response_flags::FAILSAFE
                    | response_flags::MOTORS_VALID,
                [0; 4],
            ),
            (
                "stale estimate and stale actuator",
                Some(valid_estimate(1_999)),
                Some(valid_actuator(1_999, false)),
                response_flags::ARMED | response_flags::FAILSAFE | response_flags::MOTORS_VALID,
                [0; 4],
            ),
        ];

        for (name, estimate, actuator, expected_flags, expected_motors) in cases {
            let response = build_hil_response_frame(
                stamp(2_000),
                TEST_STATE_ARMED,
                true,
                false,
                estimate,
                actuator,
            );

            assert_eq!(response.flags, expected_flags, "{name}");
            assert_eq!(response.motor_cmd, expected_motors, "{name}");
        }
    }

    #[test]
    fn sensor_input_valid_flag_follows_response_argument() {
        let without_sensor = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            false,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, false)),
        );
        let with_sensor = build_hil_response_frame(
            stamp(2_000),
            TEST_STATE_ARMED,
            true,
            true,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, false)),
        );

        assert_eq!(without_sensor.flags & response_flags::SENSOR_INPUT_VALID, 0);
        assert_ne!(with_sensor.flags & response_flags::SENSOR_INPUT_VALID, 0);
    }

    #[test]
    fn invalid_clamped_actuator_does_not_set_control_clamped() {
        let health = evaluate_hil_response_health(
            true,
            true,
            Some(ActuatorOutputSample::new([0.0; 4], false, true)),
        );

        assert_eq!(health.flags & response_flags::CONTROL_CLAMPED, 0);
        assert_ne!(health.flags & response_flags::FAILSAFE, 0);
    }

    #[test]
    fn invalid_asserted_sensor_fields_do_not_set_sensor_input_valid() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();
        let mut sensors = valid_sensor_frame(2_000);
        sensors.gyro_rps[0] = f32::NAN;

        let dispatch = adapter.accept_frame(sensors, &routes);
        let response = build_hil_response_frame(
            sensors.stamp,
            TEST_STATE_ARMED,
            true,
            dispatch.accepted && dispatch.dispatch.imu_published,
            Some(valid_estimate(2_000)),
            Some(valid_actuator(2_000, false)),
        );

        assert_eq!(
            dispatch.rejection,
            Some(HilSensorFrameRejection::InvalidSample)
        );
        assert_eq!(response.flags & response_flags::SENSOR_INPUT_VALID, 0);
    }
}
