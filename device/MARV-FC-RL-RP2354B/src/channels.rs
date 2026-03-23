use common::messages::control::RgbLedCommand;
use common::messages::logging::{LogSinkState, LoggedSensor};
use common::messages::runtime::FlightPhase;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleChannel, GpsFixSampleSubscriber,
    ImuSampleChannel, ImuSampleSubscriber, MagnetometerSampleSubscriber,
    PressureTransducerSampleSubscriber, TimeSampleChannel, TimeSampleSubscriber,
};
use common::services::health::LivenessUpdate;
use common::services::hil::{HilControlCommand, HilEgressMessage};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_sync::signal::Signal;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelId {
    FastSensorSample,
    EnvironmentalSample,
    AuxiliaryNavigationSample,
    LoggingRecord,
    HilControlCommand,
    MissionCommand,
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
pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 4;
pub const SENSOR_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;
pub const SENSOR_FAULT_DEPTH: usize = 8;
pub const RGB_LED_COMMAND_DEPTH: usize = 4;
pub const HIL_CONTROL_COMMAND_DEPTH: usize = 4;
pub const HIL_EGRESS_DEPTH: usize = 8;
pub const WATCHDOG_LIVENESS_DEPTH: usize = 16;
pub const FLIGHT_PHASE_DEPTH: usize = 4;
pub const FLIGHT_PHASE_SUBS: usize = 4;
pub const FLIGHT_PHASE_PUBS: usize = 1;

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
pub type FcRgbLedCommandReceiver =
    Receiver<'static, CriticalSectionRawMutex, RgbLedCommand, RGB_LED_COMMAND_DEPTH>;
pub type FcRgbLedCommandSender =
    Sender<'static, CriticalSectionRawMutex, RgbLedCommand, RGB_LED_COMMAND_DEPTH>;
pub type FcHilControlCommandReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilControlCommand, HIL_CONTROL_COMMAND_DEPTH>;
pub type FcHilEgressReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilEgressMessage, HIL_EGRESS_DEPTH>;
pub type FcWatchdogLivenessReceiver =
    Receiver<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type FcWatchdogLivenessSender =
    Sender<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type FcTimeChannel = TimeSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type FcTimeSubscriber = TimeSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type FcBarometerChannel = BarometerSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type FcBarometerSubscriber = BarometerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type FcGpsChannel = GpsFixSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type FcGpsSubscriber = GpsFixSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type DisabledPressureTransducerSubscriber =
    PressureTransducerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type DisabledMagnetometerSubscriber =
    MagnetometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type FcFlightPhaseChannel = PubSubChannel<
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;
pub type FcFlightPhaseSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;

pub static IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
pub static AUX_IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
pub static TIME_CHANNEL: FcTimeChannel = FcTimeChannel::new();
pub static BAROMETER_CHANNEL: FcBarometerChannel = FcBarometerChannel::new();
pub static GPS_CHANNEL: FcGpsChannel = FcGpsChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
pub static RGB_LED_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    RgbLedCommand,
    RGB_LED_COMMAND_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static SENSOR_FAULT_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    LoggedSensor,
    SENSOR_FAULT_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static HIL_CONTROL_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    HilControlCommand,
    HIL_CONTROL_COMMAND_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static HIL_EGRESS_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    HilEgressMessage,
    HIL_EGRESS_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static WATCHDOG_LIVENESS_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    LivenessUpdate,
    WATCHDOG_LIVENESS_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static FLIGHT_PHASE_CHANNEL: FcFlightPhaseChannel = FcFlightPhaseChannel::new();
pub static HIL_BOOT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static IMU_INIT_SIGNAL: Signal<CriticalSectionRawMutex, ImuInitReport> = Signal::new();

const CORE0_LOCAL: &[ChannelId] = &[
    ChannelId::FastSensorSample,
    ChannelId::EnvironmentalSample,
    ChannelId::AuxiliaryNavigationSample,
    ChannelId::LoggingRecord,
    ChannelId::HilControlCommand,
    ChannelId::MissionCommand,
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
