use common::messages::control::StaticLedCommand;
use common::messages::logging::LogSinkState;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleSubscriber, ImuSampleSubscriber,
    MagnetometerSampleSubscriber, PressureTransducerSampleChannel,
    PressureTransducerSampleSubscriber, TimeSampleChannel, TimeSampleSubscriber,
};
use common::services::hil::HilMissionEvent;
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 2;
pub const SENSOR_CHANNEL_PUBS: usize = 1;
pub const BAROMETER_CHANNEL_DEPTH: usize = 16;
pub const BAROMETER_CHANNEL_SUBS: usize = 2;
pub const BAROMETER_CHANNEL_PUBS: usize = 1;
pub const PRESSURE_TRANSDUCER_CHANNEL_DEPTH: usize = 16;
pub const PRESSURE_TRANSDUCER_CHANNEL_SUBS: usize = 1;
pub const PRESSURE_TRANSDUCER_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;
pub const STATUS_LED_COMMAND_DEPTH: usize = 4;
pub const HIL_MISSION_EVENT_DEPTH: usize = 4;

pub type PayloadBarometerChannel = BarometerSampleChannel<
    CriticalSectionRawMutex,
    BAROMETER_CHANNEL_DEPTH,
    BAROMETER_CHANNEL_SUBS,
    BAROMETER_CHANNEL_PUBS,
>;
pub type PayloadBarometerSubscriber = BarometerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    BAROMETER_CHANNEL_DEPTH,
    BAROMETER_CHANNEL_SUBS,
    BAROMETER_CHANNEL_PUBS,
>;
pub type PayloadTimeChannel = TimeSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type PayloadTimeSubscriber = TimeSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type PayloadPressureTransducerChannel = PressureTransducerSampleChannel<
    CriticalSectionRawMutex,
    PRESSURE_TRANSDUCER_CHANNEL_DEPTH,
    PRESSURE_TRANSDUCER_CHANNEL_SUBS,
    PRESSURE_TRANSDUCER_CHANNEL_PUBS,
>;
pub type PayloadPressureTransducerSubscriber = PressureTransducerSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    PRESSURE_TRANSDUCER_CHANNEL_DEPTH,
    PRESSURE_TRANSDUCER_CHANNEL_SUBS,
    PRESSURE_TRANSDUCER_CHANNEL_PUBS,
>;
pub type PayloadLogSinkStateReceiver =
    Receiver<'static, CriticalSectionRawMutex, LogSinkState, LOG_SINK_STATE_DEPTH>;
pub type PayloadStatusLedCommandReceiver =
    Receiver<'static, CriticalSectionRawMutex, StaticLedCommand, STATUS_LED_COMMAND_DEPTH>;
pub type PayloadStatusLedCommandSender =
    Sender<'static, CriticalSectionRawMutex, StaticLedCommand, STATUS_LED_COMMAND_DEPTH>;
pub type PayloadHilMissionEventReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilMissionEvent, HIL_MISSION_EVENT_DEPTH>;
pub type PayloadHilMissionEventSender =
    Sender<'static, CriticalSectionRawMutex, HilMissionEvent, HIL_MISSION_EVENT_DEPTH>;
pub type DisabledImuSubscriber = ImuSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type DisabledGpsSubscriber = GpsFixSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
pub type DisabledMagnetometerSubscriber =
    MagnetometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;

pub static TIME_CHANNEL: PayloadTimeChannel = PayloadTimeChannel::new();
pub static BAROMETER_CHANNEL: PayloadBarometerChannel = PayloadBarometerChannel::new();
pub static PRESSURE_TRANSDUCER_CHANNEL: PayloadPressureTransducerChannel =
    PayloadPressureTransducerChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
pub static STATUS_LED_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    StaticLedCommand,
    STATUS_LED_COMMAND_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static HIL_MISSION_EVENT_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    HilMissionEvent,
    HIL_MISSION_EVENT_DEPTH,
> = embassy_sync::channel::Channel::new();
