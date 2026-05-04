use common::comms::links::lora::state::LoraLinkState;
use common::messages::control::RgbLedCommand;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum StatusIndicatorEvent {
    LinkState(LoraLinkState),
    Ping,
    Pong,
}

pub type StatusLedCommandChannel = Channel<CriticalSectionRawMutex, RgbLedCommand, 4>;
pub type StatusLedCommandReceiver = Receiver<'static, CriticalSectionRawMutex, RgbLedCommand, 4>;
pub type StatusIndicatorChannel = Channel<CriticalSectionRawMutex, StatusIndicatorEvent, 16>;
pub type StatusIndicatorSender = Sender<'static, CriticalSectionRawMutex, StatusIndicatorEvent, 16>;
pub type StatusIndicatorReceiver =
    Receiver<'static, CriticalSectionRawMutex, StatusIndicatorEvent, 16>;

pub static STATUS_LED_COMMAND_CHANNEL: StatusLedCommandChannel = Channel::new();
pub static STATUS_INDICATOR_CHANNEL: StatusIndicatorChannel = Channel::new();
