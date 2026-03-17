use common::messages::logging::{LogSinkState, LoggedSensor};
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleChannel, GpsFixSampleSubscriber,
    ImuSampleChannel, ImuSampleSubscriber, MagnetometerSampleSubscriber, TimeSampleChannel,
};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::signal::Signal;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelId {
    FastSensorSample,
    EnvironmentalSample,
    AuxiliaryNavigationSample,
    LoggingRecord,
    SensorFault,
    FcRadioTraffic,
    CompanionTraffic,
    WatchdogStatus,
}

pub struct ChannelTopology {
    pub core0_local: &'static [ChannelId],
    pub core1_local: &'static [ChannelId],
    pub cross_core_bridges: &'static [ChannelId],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImuInitReport {
    pub imu_ready: bool,
    pub aux_imu_ready: bool,
}

pub const IMU_CHANNEL_DEPTH: usize = 16;
pub const IMU_CHANNEL_SUBS: usize = 2;
pub const IMU_CHANNEL_PUBS: usize = 1;
pub const HIL_CHANNEL_DEPTH: usize = 16;
pub const HIL_CHANNEL_SUBS: usize = 2;
pub const HIL_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;
pub const SENSOR_FAULT_DEPTH: usize = 8;

pub type FcImuChannel = ImuSampleChannel<
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type FcImuSubscriber = ImuSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
pub type FcLogSinkStateReceiver =
    Receiver<'static, CriticalSectionRawMutex, LogSinkState, LOG_SINK_STATE_DEPTH>;
pub type FcSensorFaultReceiver =
    Receiver<'static, CriticalSectionRawMutex, LoggedSensor, SENSOR_FAULT_DEPTH>;
pub type FcHilTimeChannel = TimeSampleChannel<
    CriticalSectionRawMutex,
    HIL_CHANNEL_DEPTH,
    HIL_CHANNEL_SUBS,
    HIL_CHANNEL_PUBS,
>;
pub type FcHilImuChannel = ImuSampleChannel<
    CriticalSectionRawMutex,
    HIL_CHANNEL_DEPTH,
    HIL_CHANNEL_SUBS,
    HIL_CHANNEL_PUBS,
>;
pub type FcHilBarometerChannel = BarometerSampleChannel<
    CriticalSectionRawMutex,
    HIL_CHANNEL_DEPTH,
    HIL_CHANNEL_SUBS,
    HIL_CHANNEL_PUBS,
>;
pub type FcHilGpsChannel = GpsFixSampleChannel<
    CriticalSectionRawMutex,
    HIL_CHANNEL_DEPTH,
    HIL_CHANNEL_SUBS,
    HIL_CHANNEL_PUBS,
>;
pub type DisabledBarometerSubscriber =
    BarometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type DisabledMagnetometerSubscriber =
    MagnetometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type DisabledGpsSubscriber = GpsFixSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;

pub static IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
pub static AUX_IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
pub static HIL_TIME_CHANNEL: FcHilTimeChannel = FcHilTimeChannel::new();
pub static HIL_IMU_CHANNEL: FcHilImuChannel = FcHilImuChannel::new();
pub static HIL_BAROMETER_CHANNEL: FcHilBarometerChannel = FcHilBarometerChannel::new();
pub static HIL_GPS_CHANNEL: FcHilGpsChannel = FcHilGpsChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
pub static SENSOR_FAULT_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    LoggedSensor,
    SENSOR_FAULT_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static IMU_INIT_SIGNAL: Signal<CriticalSectionRawMutex, ImuInitReport> = Signal::new();

const CORE0_LOCAL: &[ChannelId] = &[
    ChannelId::FastSensorSample,
    ChannelId::EnvironmentalSample,
    ChannelId::AuxiliaryNavigationSample,
    ChannelId::LoggingRecord,
    ChannelId::SensorFault,
    ChannelId::WatchdogStatus,
];

const CORE1_LOCAL: &[ChannelId] = &[ChannelId::FcRadioTraffic, ChannelId::CompanionTraffic];

const CROSS_CORE_BRIDGES: &[ChannelId] = &[];

pub const TOPOLOGY: ChannelTopology = ChannelTopology {
    core0_local: CORE0_LOCAL,
    core1_local: CORE1_LOCAL,
    cross_core_bridges: CROSS_CORE_BRIDGES,
};
