use common::localization::estimation::core::Vec3;
use common::localization::estimation::math::quaternion_to_rotation_matrix;
use common::localization::estimation::stacks::LayeredNavigationStack;
use common::localization::navigation::{GeodeticPosition, LocalNedFrame};
use common::messages::estimate::{StateEstimateSample, StateEstimateStamped};
use common::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
};
use common::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_m};
use defmt::warn;
use embassy_executor::Spawner;
use embassy_sync::pubsub::WaitResult;

use crate::channels::{
    BAROMETER_CHANNEL, GPS_CHANNEL, IMU_CHANNEL, MAGNETOMETER_CHANNEL, STATE_ESTIMATE_CHANNEL,
    TestBarometerSubscriber, TestGpsSubscriber, TestImuSubscriber, TestMagnetometerSubscriber,
};

const MIN_ESTIMATOR_DT_S: f32 = 1.0e-5;
const MAX_ESTIMATOR_DT_S: f32 = 0.1;
const MIN_GPS_FIX_TYPE: u8 = 3;
const MIN_GPS_SATS: u8 = 4;

pub fn spawn(spawner: &Spawner) {
    spawner
        .spawn(estimator_task(
            IMU_CHANNEL.subscriber().unwrap(),
            BAROMETER_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
            MAGNETOMETER_CHANNEL.subscriber().unwrap(),
        ))
        .unwrap();
}

#[embassy_executor::task]
async fn estimator_task(
    mut imu_subscriber: TestImuSubscriber,
    mut barometer_subscriber: TestBarometerSubscriber,
    mut gps_subscriber: TestGpsSubscriber,
    mut magnetometer_subscriber: TestMagnetometerSubscriber,
) -> ! {
    let mut estimator = LayeredNavigationStack::<f32>::default();
    let mut previous_imu = None;
    let mut local_frame = None;
    let mut baro_origin_altitude_m = None;
    let mut magnetic_reference_inertial_ut = None;

    loop {
        let imu = match imu_subscriber.next_message().await {
            WaitResult::Message(sample) => sample,
            WaitResult::Lagged(skipped) => {
                warn!("rp235x-test estimator skipped {=u64} IMU samples", skipped);
                previous_imu = None;
                continue;
            }
        };

        if !finite_f32x3(imu.sample.accel_mps2) || !finite_f32x3(imu.sample.gyro_rad_s) {
            publish_invalid(imu);
            previous_imu = None;
            continue;
        }

        let Some(previous) = previous_imu.replace(imu) else {
            publish_invalid(imu);
            continue;
        };

        let Some(dt) = estimator_dt_s(previous, imu) else {
            publish_invalid(imu);
            continue;
        };

        let mut valid = estimator
            .predict(
                vec3(imu.sample.accel_mps2),
                vec3(imu.sample.gyro_rad_s),
                dt,
                Some(imu.timestamp.as_micros() as f32 * 1.0e-6),
            )
            .is_ok();

        if valid {
            valid &= estimator
                .update_gravity_alignment(vec3(imu.sample.accel_mps2), None)
                .is_ok();
        }

        if valid {
            if let Some(magnetometer) = drain_latest_magnetometer(&mut magnetometer_subscriber) {
                if let Some(reference) = magnetic_reference_inertial_ut
                    .or_else(|| magnetic_reference_inertial_ut_for(&estimator, magnetometer))
                {
                    magnetic_reference_inertial_ut = Some(reference);
                    valid &= estimator
                        .update_magnetic_field(vec3(magnetometer.sample.field_ut), reference, None)
                        .is_ok();
                }
            }
        }

        if let Some(barometer) = drain_latest_baro(&mut barometer_subscriber) {
            if let Some(altitude_m) =
                pressure_altitude_m(barometer.sample.pressure_pa, STANDARD_SEA_LEVEL_PRESSURE_PA)
            {
                let origin_altitude_m = *baro_origin_altitude_m.get_or_insert(altitude_m);
                let down_m = origin_altitude_m - altitude_m;
                valid &= estimator.update_barometric_altitude(down_m, None).is_ok();
            }
        }

        if let Some(gps) = drain_latest_gps(&mut gps_subscriber) {
            if let Some(position_ned_m) = gps_position_ned_m(gps, &mut local_frame) {
                valid &= estimator
                    .update_position(vec3(position_ned_m), None)
                    .is_ok();
                valid &= estimator
                    .update_velocity(vec3(gps.sample.vel_ned_mps), None)
                    .is_ok();
            }
        }

        let sample = state_estimate_sample(&estimator, valid);
        STATE_ESTIMATE_CHANNEL
            .immediate_publisher()
            .publish_immediate(StateEstimateStamped {
                timestamp: imu.timestamp,
                sample,
            });
    }
}

fn estimator_dt_s(previous: ImuSampleStamped, current: ImuSampleStamped) -> Option<f32> {
    let dt = current.dt_since(previous)?.as_seconds_f32();
    if (MIN_ESTIMATOR_DT_S..=MAX_ESTIMATOR_DT_S).contains(&dt) {
        Some(dt)
    } else {
        None
    }
}

fn publish_invalid(imu: ImuSampleStamped) {
    STATE_ESTIMATE_CHANNEL
        .immediate_publisher()
        .publish_immediate(StateEstimateStamped {
            timestamp: imu.timestamp,
            sample: StateEstimateSample::NEUTRAL_INVALID,
        });
}

fn drain_latest_baro(subscriber: &mut TestBarometerSubscriber) -> Option<BarometerSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
}

fn drain_latest_gps(subscriber: &mut TestGpsSubscriber) -> Option<GpsFixSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
}

fn drain_latest_magnetometer(
    subscriber: &mut TestMagnetometerSubscriber,
) -> Option<MagnetometerSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
}

fn magnetic_reference_inertial_ut_for(
    estimator: &LayeredNavigationStack<f32>,
    magnetometer: MagnetometerSampleStamped,
) -> Option<Vec3<f32>> {
    if !finite_f32x3(magnetometer.sample.field_ut) {
        return None;
    }

    let field_body_ut = vec3(magnetometer.sample.field_ut);
    if field_body_ut.norm() <= f32::EPSILON {
        return None;
    }

    let state = estimator.state();
    Some(quaternion_to_rotation_matrix(&state.quaternion) * field_body_ut)
}

fn gps_position_ned_m(
    gps: GpsFixSampleStamped,
    local_frame: &mut Option<LocalNedFrame>,
) -> Option<[f32; 3]> {
    if gps.sample.fix_type < MIN_GPS_FIX_TYPE
        || gps.sample.sats < MIN_GPS_SATS
        || !gps.sample.lat_deg.is_finite()
        || !gps.sample.lon_deg.is_finite()
        || !gps.sample.alt_m.is_finite()
        || !finite_f32x3(gps.sample.vel_ned_mps)
    {
        return None;
    }

    let position = GeodeticPosition::new(gps.sample.lat_deg, gps.sample.lon_deg, gps.sample.alt_m);
    LocalNedFrame::from_first_valid(local_frame, position)?.position_ned_m(position)
}

fn state_estimate_sample(
    estimator: &LayeredNavigationStack<f32>,
    valid: bool,
) -> StateEstimateSample {
    let state = estimator.state();
    let sample = StateEstimateSample {
        position_ned_m: [
            state.position_m[0],
            state.position_m[1],
            state.position_m[2],
        ],
        velocity_ned_mps: [
            state.velocity_mps[0],
            state.velocity_mps[1],
            state.velocity_mps[2],
        ],
        attitude_quat: [
            state.quaternion[0],
            state.quaternion[1],
            state.quaternion[2],
            state.quaternion[3],
        ],
        valid,
    };

    if finite_f32x3(sample.position_ned_m)
        && finite_f32x3(sample.velocity_ned_mps)
        && finite_f32x4(sample.attitude_quat)
    {
        sample
    } else {
        StateEstimateSample::NEUTRAL_INVALID
    }
}

fn vec3(values: [f32; 3]) -> Vec3<f32> {
    Vec3::<f32>::new(values[0], values[1], values[2])
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn finite_f32x4(values: [f32; 4]) -> bool {
    values.iter().all(|value| value.is_finite())
}
