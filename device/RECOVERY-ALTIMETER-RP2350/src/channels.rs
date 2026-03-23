use common::messages::logging::LogSinkState;
use common::messages::runtime::FlightPhase;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, ImuSampleChannel, ImuSampleSubscriber,
    TimeSampleChannel, TimeSampleSubscriber,
};
use common::services::health::LivenessUpdate;
use common::services::hil::{HilControlCommand, HilEgressMessage};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_sync::signal::Signal;

pub const SENSOR_CHANNEL_DEPTH: usize = 16;
pub const SENSOR_CHANNEL_SUBS: usize = 2;
pub const SENSOR_CHANNEL_PUBS: usize = 1;
pub const BAROMETER_CHANNEL_DEPTH: usize = 16;
pub const BAROMETER_CHANNEL_SUBS: usize = 2;
pub const BAROMETER_CHANNEL_PUBS: usize = 1;
pub const IMU_CHANNEL_DEPTH: usize = 16;
pub const IMU_CHANNEL_SUBS: usize = 2;
pub const IMU_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;
pub const HIL_CONTROL_COMMAND_DEPTH: usize = 4;
pub const HIL_EGRESS_DEPTH: usize = 8;
pub const WATCHDOG_LIVENESS_DEPTH: usize = 16;
pub const FLIGHT_PHASE_DEPTH: usize = 4;
pub const FLIGHT_PHASE_SUBS: usize = 4;
pub const FLIGHT_PHASE_PUBS: usize = 1;

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
pub type RecoveryTimeChannel = TimeSampleChannel<
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
>;
pub type RecoveryTimeSubscriber = TimeSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    SENSOR_CHANNEL_DEPTH,
    SENSOR_CHANNEL_SUBS,
    SENSOR_CHANNEL_PUBS,
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
pub type RecoveryHilControlCommandReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilControlCommand, HIL_CONTROL_COMMAND_DEPTH>;
pub type RecoveryHilEgressReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilEgressMessage, HIL_EGRESS_DEPTH>;
pub type RecoveryHilEgressSender =
    Sender<'static, CriticalSectionRawMutex, HilEgressMessage, HIL_EGRESS_DEPTH>;
pub type RecoveryWatchdogLivenessReceiver =
    Receiver<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type RecoveryWatchdogLivenessSender =
    Sender<'static, CriticalSectionRawMutex, LivenessUpdate, WATCHDOG_LIVENESS_DEPTH>;
pub type RecoveryFlightPhaseChannel = PubSubChannel<
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;
pub type RecoveryFlightPhaseSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    FlightPhase,
    FLIGHT_PHASE_DEPTH,
    FLIGHT_PHASE_SUBS,
    FLIGHT_PHASE_PUBS,
>;

pub static TIME_CHANNEL: RecoveryTimeChannel = RecoveryTimeChannel::new();
pub static BAROMETER_CHANNEL: RecoveryBarometerChannel = RecoveryBarometerChannel::new();
pub static IMU_CHANNEL: RecoveryImuChannel = RecoveryImuChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
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
pub static FLIGHT_PHASE_CHANNEL: RecoveryFlightPhaseChannel = RecoveryFlightPhaseChannel::new();
pub static HIL_BOOT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
