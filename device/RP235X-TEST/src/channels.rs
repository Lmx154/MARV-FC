use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, ImuSampleChannel, ImuSampleSubscriber,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImuInitReport {
    pub imu_ready: bool,
    pub aux_imu_ready: bool,
}

pub const IMU_CHANNEL_DEPTH: usize = 16;
pub const IMU_CHANNEL_SUBS: usize = 2;
pub const IMU_CHANNEL_PUBS: usize = 1;
pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 2;
pub const SENSOR_CHANNEL_PUBS: usize = 1;

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

pub static IMU_CHANNEL: TestImuChannel = TestImuChannel::new();
pub static AUX_IMU_CHANNEL: TestImuChannel = TestImuChannel::new();
pub static BAROMETER_CHANNEL: TestBarometerChannel = TestBarometerChannel::new();
pub static IMU_INIT_SIGNAL: Signal<CriticalSectionRawMutex, ImuInitReport> = Signal::new();
