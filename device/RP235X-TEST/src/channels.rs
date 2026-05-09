use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleChannel, GpsFixSampleSubscriber,
    ImuSampleChannel, ImuSampleSubscriber, MagnetometerSampleChannel, MagnetometerSampleSubscriber,
    TimeSampleChannel,
};
use common::services::control::{ActuatorOutputChannel, ActuatorOutputSubscriber};
use common::services::estimation::{StateEstimateChannel, StateEstimateSubscriber};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImuInitReport {
    pub imu_ready: bool,
    pub aux_imu_ready: bool,
}

pub const IMU_CHANNEL_DEPTH: usize = 16;
pub const IMU_CHANNEL_SUBS: usize = 3;
pub const IMU_CHANNEL_PUBS: usize = 1;
pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 3;
pub const SENSOR_CHANNEL_PUBS: usize = 1;
pub const GPS_CHANNEL_DEPTH: usize = 16;
pub const GPS_CHANNEL_SUBS: usize = 4;
pub const GPS_CHANNEL_PUBS: usize = 1;
pub const TIME_CHANNEL_DEPTH: usize = 16;
pub const TIME_CHANNEL_SUBS: usize = 2;
pub const TIME_CHANNEL_PUBS: usize = 1;
pub const STATE_ESTIMATE_CHANNEL_DEPTH: usize = 4;
pub const STATE_ESTIMATE_CHANNEL_SUBS: usize = 2;
pub const STATE_ESTIMATE_CHANNEL_PUBS: usize = 1;
pub const ACTUATOR_OUTPUT_CHANNEL_DEPTH: usize = 16;
pub const ACTUATOR_OUTPUT_CHANNEL_SUBS: usize = 2;
pub const ACTUATOR_OUTPUT_CHANNEL_PUBS: usize = 1;

pub type TestImuChannel = ImuSampleChannel<
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type TestBarometerChannel = BarometerSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type TestMagnetometerChannel = MagnetometerSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type TestGpsChannel = GpsFixSampleChannel<
    CriticalSectionRawMutex,
    GPS_CHANNEL_DEPTH,
    GPS_CHANNEL_SUBS,
    GPS_CHANNEL_PUBS,
>;
pub type TestTimeChannel = TimeSampleChannel<
    CriticalSectionRawMutex,
    TIME_CHANNEL_DEPTH,
    TIME_CHANNEL_SUBS,
    TIME_CHANNEL_PUBS,
>;
pub type TestStateEstimateChannel = StateEstimateChannel<
    CriticalSectionRawMutex,
    STATE_ESTIMATE_CHANNEL_DEPTH,
    STATE_ESTIMATE_CHANNEL_SUBS,
    STATE_ESTIMATE_CHANNEL_PUBS,
>;
pub type TestActuatorOutputChannel = ActuatorOutputChannel<
    CriticalSectionRawMutex,
    ACTUATOR_OUTPUT_CHANNEL_DEPTH,
    ACTUATOR_OUTPUT_CHANNEL_SUBS,
    ACTUATOR_OUTPUT_CHANNEL_PUBS,
>;
pub type TestImuSubscriber = ImuSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type TestBarometerSubscriber = BarometerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type TestMagnetometerSubscriber = MagnetometerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type TestGpsSubscriber = GpsFixSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    GPS_CHANNEL_DEPTH,
    GPS_CHANNEL_SUBS,
    GPS_CHANNEL_PUBS,
>;
pub type TestStateEstimateSubscriber = StateEstimateSubscriber<
    'static,
    CriticalSectionRawMutex,
    STATE_ESTIMATE_CHANNEL_DEPTH,
    STATE_ESTIMATE_CHANNEL_SUBS,
    STATE_ESTIMATE_CHANNEL_PUBS,
>;
pub type TestActuatorOutputSubscriber = ActuatorOutputSubscriber<
    'static,
    CriticalSectionRawMutex,
    ACTUATOR_OUTPUT_CHANNEL_DEPTH,
    ACTUATOR_OUTPUT_CHANNEL_SUBS,
    ACTUATOR_OUTPUT_CHANNEL_PUBS,
>;

pub static TIME_CHANNEL: TestTimeChannel = TestTimeChannel::new();
pub static IMU_CHANNEL: TestImuChannel = TestImuChannel::new();
pub static AUX_IMU_CHANNEL: TestImuChannel = TestImuChannel::new();
pub static BAROMETER_CHANNEL: TestBarometerChannel = TestBarometerChannel::new();
pub static MAGNETOMETER_CHANNEL: TestMagnetometerChannel = TestMagnetometerChannel::new();
pub static GPS_CHANNEL: TestGpsChannel = TestGpsChannel::new();
pub static STATE_ESTIMATE_CHANNEL: TestStateEstimateChannel = TestStateEstimateChannel::new();
pub static ACTUATOR_OUTPUT_CHANNEL: TestActuatorOutputChannel = TestActuatorOutputChannel::new();
pub static IMU_INIT_SIGNAL: Signal<CriticalSectionRawMutex, ImuInitReport> = Signal::new();
