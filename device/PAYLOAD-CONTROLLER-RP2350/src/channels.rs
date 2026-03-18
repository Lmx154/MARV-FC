use common::messages::logging::LogSinkState;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, PressureTransducerSampleChannel,
    PressureTransducerSampleSubscriber,
};
use common::services::logging::{LogChannel, LogSinkStateChannel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;

pub const BAROMETER_CHANNEL_DEPTH: usize = 16;
pub const BAROMETER_CHANNEL_SUBS: usize = 2;
pub const BAROMETER_CHANNEL_PUBS: usize = 1;
pub const PRESSURE_TRANSDUCER_CHANNEL_DEPTH: usize = 16;
pub const PRESSURE_TRANSDUCER_CHANNEL_SUBS: usize = 1;
pub const PRESSURE_TRANSDUCER_CHANNEL_PUBS: usize = 1;
pub const LOG_CHANNEL_DEPTH: usize = 32;
pub const LOG_SINK_STATE_DEPTH: usize = 4;

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

pub static BAROMETER_CHANNEL: PayloadBarometerChannel = PayloadBarometerChannel::new();
pub static PRESSURE_TRANSDUCER_CHANNEL: PayloadPressureTransducerChannel =
    PayloadPressureTransducerChannel::new();
pub static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
pub static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<
    CriticalSectionRawMutex,
    LOG_SINK_STATE_DEPTH,
> = LogSinkStateChannel::new();
