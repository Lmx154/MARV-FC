use core::cell::RefCell;

use common::{
    messages::{
        control::{ActuatorOutputSample, ActuatorOutputStamped},
        estimate::{StateEstimateSample, StateEstimateStamped},
        sensor::{
            BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped,
            MagnetometerSampleStamped, TimeSample,
        },
    },
    policies::failsafe::build_hil_response_frame,
    protocol::hilink::{HilResponseFrame, HilSensorFrame, SimStamp, response_flags, valid},
    services::hil::{
        HilBarometerRoute, HilCorrelationState, HilGpsRoute, HilImuRoute, HilIngressRoutes,
        HilMagnetometerRoute, HilSensorFrameAdapter, HilSensorFrameDispatch, HilTimeRoute,
    },
    utilities::time::MeasurementTimestamp,
};

use crate::{ControlPipelineTrace, EstimatorReplayTrace, SensorFrame};

const STANDARD_PRESSURE_PA: f32 = 101_325.0;
const STANDARD_TEMPERATURE_C: f32 = 15.0;
const TEST_SYSTEM_STATE_ARMED: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HilSemanticFrameConfig {
    pub origin_lat_deg: f64,
    pub origin_lon_deg: f64,
    pub origin_alt_msl_m: f32,
    pub default_sats: u8,
    pub default_fix_type: u8,
    pub battery_voltage_v: f32,
}

impl Default for HilSemanticFrameConfig {
    fn default() -> Self {
        Self {
            origin_lat_deg: 26.0,
            origin_lon_deg: -98.0,
            origin_alt_msl_m: 120.0,
            default_sats: 10,
            default_fix_type: 3,
            battery_voltage_v: 16.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilPublishedGroups {
    pub time: usize,
    pub imu: usize,
    pub barometer: usize,
    pub gps: usize,
    pub magnetometer: usize,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct HilSemanticArchive {
    pub raw_frames: Vec<HilSensorFrame>,
    pub responses: Vec<HilResponseFrame>,
    pub dispatches: Vec<HilSensorFrameDispatch>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HilSemanticTrace {
    pub frame: HilSensorFrame,
    pub dispatch: HilSensorFrameDispatch,
    pub response: HilResponseFrame,
    pub published_groups: HilPublishedGroups,
}

#[derive(Debug, Default)]
pub struct HilRouteRecorder {
    time: RefCell<Vec<TimeSample>>,
    imu: RefCell<Vec<ImuSampleStamped>>,
    barometer: RefCell<Vec<BarometerSampleStamped>>,
    gps: RefCell<Vec<GpsFixSampleStamped>>,
    magnetometer: RefCell<Vec<MagnetometerSampleStamped>>,
}

impl HilRouteRecorder {
    pub fn published_groups(&self) -> HilPublishedGroups {
        HilPublishedGroups {
            time: self.time.borrow().len(),
            imu: self.imu.borrow().len(),
            barometer: self.barometer.borrow().len(),
            gps: self.gps.borrow().len(),
            magnetometer: self.magnetometer.borrow().len(),
        }
    }
}

impl HilTimeRoute for HilRouteRecorder {
    fn publish_time(&self, sample: TimeSample) {
        self.time.borrow_mut().push(sample);
    }
}

impl HilImuRoute for HilRouteRecorder {
    fn publish_imu(&self, sample: ImuSampleStamped) {
        self.imu.borrow_mut().push(sample);
    }
}

impl HilBarometerRoute for HilRouteRecorder {
    fn publish_barometer(&self, sample: BarometerSampleStamped) {
        self.barometer.borrow_mut().push(sample);
    }
}

impl HilGpsRoute for HilRouteRecorder {
    fn publish_gps(&self, sample: GpsFixSampleStamped) {
        self.gps.borrow_mut().push(sample);
    }
}

impl HilMagnetometerRoute for HilRouteRecorder {
    fn publish_magnetometer(&self, sample: MagnetometerSampleStamped) {
        self.magnetometer.borrow_mut().push(sample);
    }
}

#[derive(Debug, Default)]
pub struct HilSemanticAdapter {
    adapter: HilSensorFrameAdapter,
    routes: HilRouteRecorder,
    archive: HilSemanticArchive,
}

impl HilSemanticAdapter {
    pub fn active() -> Self {
        Self {
            adapter: HilSensorFrameAdapter::active(),
            routes: HilRouteRecorder::default(),
            archive: HilSemanticArchive::default(),
        }
    }

    pub fn accept_frame(&mut self, frame: HilSensorFrame) -> HilSensorFrameDispatch {
        let routes = HilIngressRoutes::new(
            &self.routes,
            &self.routes,
            &self.routes,
            &self.routes,
            &self.routes,
            &(),
        );
        let dispatch = self.adapter.accept_frame(frame, &routes);
        self.archive.raw_frames.push(frame);
        self.archive.dispatches.push(dispatch);
        dispatch
    }

    pub fn accept_sensor_frame(
        &mut self,
        tick: u64,
        sensor: &SensorFrame,
        config: HilSemanticFrameConfig,
    ) -> HilSensorFrameDispatch {
        self.accept_frame(sensor_frame_to_hil_frame(tick, sensor, config))
    }

    pub fn push_response(&mut self, response: HilResponseFrame) {
        self.adapter.mark_response_sent(response.stamp);
        self.archive.responses.push(response);
    }

    pub fn mark_missed_response(&mut self) {
        self.adapter.mark_missed_response();
    }

    pub fn published_groups(&self) -> HilPublishedGroups {
        self.routes.published_groups()
    }

    pub fn correlation(&self) -> HilCorrelationState {
        self.adapter.correlation()
    }

    pub fn archive(&self) -> &HilSemanticArchive {
        &self.archive
    }
}

pub fn sensor_frame_to_hil_frame(
    tick: u64,
    sensor: &SensorFrame,
    config: HilSemanticFrameConfig,
) -> HilSensorFrame {
    let gps_position = sensor.gps_position_ned_m;
    let gps_velocity = sensor.gps_velocity_ned_mps;
    let gps_valid = gps_position.is_some() && gps_velocity.is_some();
    let baro_altitude_m = sensor
        .baro_down_m
        .map(|down| config.origin_alt_msl_m - down as f32)
        .unwrap_or(config.origin_alt_msl_m);

    HilSensorFrame {
        stamp: SimStamp {
            sim_tick: tick,
            sim_time_us: sensor.timestamp_us,
        },
        valid_flags: hil_valid_flags(sensor),
        accel_mps2: sensor.accel_mps2.map(|value| value as f32),
        gyro_rps: sensor.gyro_rps.map(|value| value as f32),
        mag_ut: sensor
            .mag_body_ut
            .map(|mag| mag.map(|value| value as f32))
            .unwrap_or([0.0; 3]),
        pressure_pa: pressure_for_altitude_m(baro_altitude_m),
        baro_altitude_m,
        temperature_c: STANDARD_TEMPERATURE_C,
        lat_deg: gps_position
            .map(|position| config.origin_lat_deg + position[0] / 111_320.0)
            .unwrap_or(config.origin_lat_deg),
        lon_deg: gps_position
            .map(|position| {
                let meters_per_degree_lon = 111_320.0 * config.origin_lat_deg.to_radians().cos();
                config.origin_lon_deg + position[1] / meters_per_degree_lon
            })
            .unwrap_or(config.origin_lon_deg),
        alt_msl_m: gps_position
            .map(|position| config.origin_alt_msl_m - position[2] as f32)
            .unwrap_or(config.origin_alt_msl_m),
        vel_ned_mps: gps_velocity
            .map(|velocity| velocity.map(|value| value as f32))
            .unwrap_or([0.0; 3]),
        sats: if gps_valid { config.default_sats } else { 0 },
        fix_type: if gps_valid {
            config.default_fix_type
        } else {
            0
        },
        reserved0: [0; 3],
        battery_voltage_v: config.battery_voltage_v,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
    }
}

pub fn hil_valid_flags(sensor: &SensorFrame) -> u32 {
    let mut flags = valid::ACCEL | valid::GYRO;
    if sensor.mag_body_ut.is_some() {
        flags |= valid::MAG;
    }
    if sensor.baro_down_m.is_some() {
        flags |= valid::BARO;
    }
    if sensor.gps_position_ned_m.is_some() && sensor.gps_velocity_ned_mps.is_some() {
        flags |= valid::GPS;
    }
    flags
}

pub fn hil_response_from_pipeline(
    stamp: SimStamp,
    sensor_input_valid: bool,
    estimate: &EstimatorReplayTrace,
    control: &ControlPipelineTrace,
) -> HilResponseFrame {
    build_hil_response_frame(
        stamp,
        TEST_SYSTEM_STATE_ARMED,
        control.armed,
        sensor_input_valid,
        Some(state_estimate_for_stamp(stamp, estimate)),
        Some(actuator_for_stamp(stamp, control)),
    )
}

pub fn stale_hil_response_from_pipeline(
    stamp: SimStamp,
    stale_by_us: u64,
    sensor_input_valid: bool,
    estimate: &EstimatorReplayTrace,
    control: &ControlPipelineTrace,
) -> HilResponseFrame {
    let stale_stamp = SimStamp {
        sim_tick: stamp.sim_tick.saturating_sub(1),
        sim_time_us: stamp.sim_time_us.saturating_sub(stale_by_us),
    };
    build_hil_response_frame(
        stamp,
        TEST_SYSTEM_STATE_ARMED,
        control.armed,
        sensor_input_valid,
        Some(state_estimate_for_stamp(stale_stamp, estimate)),
        Some(actuator_for_stamp(stamp, control)),
    )
}

pub fn response_has_flag(response: HilResponseFrame, flag: u32) -> bool {
    response.flags & flag != 0
}

pub fn response_is_failsafe_zero_output(response: HilResponseFrame) -> bool {
    response_has_flag(response, response_flags::FAILSAFE) && response.motor_cmd == [0; 4]
}

fn state_estimate_for_stamp(
    stamp: SimStamp,
    estimate: &EstimatorReplayTrace,
) -> StateEstimateStamped {
    StateEstimateStamped {
        timestamp: MeasurementTimestamp::from_micros(stamp.sim_time_us),
        sample: StateEstimateSample {
            position_ned_m: estimate.position_ned_m.map(|value| value as f32),
            velocity_ned_mps: estimate.velocity_ned_mps.map(|value| value as f32),
            attitude_quat: estimate.quaternion.map(|value| value as f32),
            valid: estimate.estimate_valid,
        },
    }
}

fn actuator_for_stamp(stamp: SimStamp, control: &ControlPipelineTrace) -> ActuatorOutputStamped {
    ActuatorOutputStamped {
        timestamp: MeasurementTimestamp::from_micros(stamp.sim_time_us),
        sample: ActuatorOutputSample::new(control.motors, control.control_valid, control.clamped),
    }
}

fn pressure_for_altitude_m(altitude_m: f32) -> f32 {
    if !altitude_m.is_finite() {
        return f32::NAN;
    }
    STANDARD_PRESSURE_PA * (-altitude_m / 8_434.5).exp()
}
