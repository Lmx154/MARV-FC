use common::messages::logging::LogSinkState;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, ImuSampleChannel, ImuSampleSubscriber,
};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;

pub const BAROMETER_CHANNEL_DEPTH: usize = 16;
pub const BAROMETER_CHANNEL_SUBS: usize = 2;
pub const BAROMETER_CHANNEL_PUBS: usize = 1;
pub const IMU_CHANNEL_DEPTH: usize = 16;
pub const IMU_CHANNEL_SUBS: usize = 2;
pub const IMU_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;

pub type RecoveryBarometerChannel = BarometerSampleChannel<
    CriticalSectionRawMutex,
    BAROMETER_CHANNEL_DEPTH,
    BAROMETER_CHANNEL_SUBS,
    BAROMETER_CHANNEL_PUBS,
>;
pub type RecoveryBarometerSubscriber = BarometerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    BAROMETER_CHANNEL_DEPTH,
    BAROMETER_CHANNEL_SUBS,
    BAROMETER_CHANNEL_PUBS,
>;
pub type RecoveryImuChannel = ImuSampleChannel<
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type RecoveryImuSubscriber = ImuSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type RecoveryLogSinkStateReceiver =
    Receiver<'static, CriticalSectionRawMutex, LogSinkState, LOG_SINK_STATE_DEPTH>;

pub static BAROMETER_CHANNEL: RecoveryBarometerChannel = RecoveryBarometerChannel::new();
pub static IMU_CHANNEL: RecoveryImuChannel = RecoveryImuChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
