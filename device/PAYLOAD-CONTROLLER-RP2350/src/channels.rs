use common::messages::control::StaticLedCommand;
use common::messages::logging::LogSinkState;
use common::messages::runtime::FlightPhase;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleSubscriber, ImuSampleSubscriber,
    MagnetometerSampleSubscriber, PressureTransducerSampleChannel,
    PressureTransducerSampleSubscriber, TimeSampleChannel, TimeSampleSubscriber,
};
use common::services::health::LivenessUpdate;
use common::services::hil::{HilControlCommand, HilEgressMessage, HilSessionState};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_sync::signal::Signal;

pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 2;
pub const SENSOR_CHANNEL_PUBS: usize = 1;
pub const BAROMETER_CHANNEL_DEPTH: usize = 16;
pub const BAROMETER_CHANNEL_SUBS: usize = 4;
pub const BAROMETER_CHANNEL_PUBS: usize = 1;
pub const PRESSURE_TRANSDUCER_CHANNEL_DEPTH: usize = 16;
pub const PRESSURE_TRANSDUCER_CHANNEL_SUBS: usize = 2;
pub const PRESSURE_TRANSDUCER_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;
pub const STATUS_LED_COMMAND_DEPTH: usize = 4;
pub const HIL_CONTROL_COMMAND_DEPTH: usize = 4;
pub const HIL_EGRESS_DEPTH: usize = 8;
pub const HIL_SESSION_STATE_DEPTH: usize = 4;
pub const HIL_SESSION_STATE_SUBS: usize = 2;
pub const HIL_SESSION_STATE_PUBS: usize = 1;
pub const WATCHDOG_LIVENESS_DEPTH: usize = 16;
pub const FLIGHT_PHASE_DEPTH: usize = 4;
pub const FLIGHT_PHASE_SUBS: usize = 4;
pub const FLIGHT_PHASE_PUBS: usize = 1;

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
pub type PayloadHilControlCommandReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilControlCommand, HIL_CONTROL_COMMAND_DEPTH>;
pub type PayloadHilControlCommandSender =
    Sender<'static, CriticalSectionRawMutex, HilControlCommand, HIL_CONTROL_COMMAND_DEPTH>;
pub type PayloadHilEgressReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilEgressMessage, HIL_EGRESS_DEPTH>;
pub type PayloadHilEgressSender =
    Sender<'static, CriticalSectionRawMutex, HilEgressMessage, HIL_EGRESS_DEPTH>;
pub type PayloadHilSessionStateChannel = PubSubChannel<
    CriticalSectionRawMutex,
    HilSessionState,
    HIL_SESSION_STATE_DEPTH,
    HIL_SESSION_STATE_SUBS,
    HIL_SESSION_STATE_PUBS,
>;
pub type PayloadHilSessionStateSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    HilSessionState,
    HIL_SESSION_STATE_DEPTH,
    HIL_SESSION_STATE_SUBS,
    HIL_SESSION_STATE_PUBS,
>;
pub type PayloadWatchdogLivenessReceiver =
    Receiver<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type PayloadWatchdogLivenessSender =
    Sender<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type PayloadFlightPhaseChannel = PubSubChannel<
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;
pub type PayloadFlightPhaseSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;
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
pub static HIL_SESSION_STATE_CHANNEL: PayloadHilSessionStateChannel =
    PayloadHilSessionStateChannel::new();
pub static WATCHDOG_LIVENESS_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    LivenessUpdate,
    WATCHDOG_LIVENESS_DEPTH,
> = embassy_sync::channel::Channel::new();
pub static FLIGHT_PHASE_CHANNEL: PayloadFlightPhaseChannel = PayloadFlightPhaseChannel::new();
pub static HIL_BOOT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
