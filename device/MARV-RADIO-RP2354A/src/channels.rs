use common::comms::links::lora::frame::MAX_FRAME_PAYLOAD_LEN;
use common::comms::links::lora::state::LoraLinkState;
use common::messages::control::RgbLedCommand;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};

pub const HILINK_BRIDGE_FRAME_BYTES: usize = MAX_FRAME_PAYLOAD_LEN;
pub const HILINK_BRIDGE_CHANNEL_DEPTH: usize = 8;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum HilinkBridgePriority {
    P0Critical,
    P1Command,
    P2Event,
    P3Snapshot,
    P4Background,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilinkBridgeFrame {
    pub len: usize,
    pub bytes: [u8; HILINK_BRIDGE_FRAME_BYTES],
}

impl HilinkBridgeFrame {
    pub const fn new() -> Self {
        Self {
            len: 0,
            bytes: [0; HILINK_BRIDGE_FRAME_BYTES],
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn push(&mut self, byte: u8) -> bool {
        if self.len >= self.bytes.len() {
            return false;
        }

        self.bytes[self.len] = byte;
        self.len += 1;
        true
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum StatusIndicatorEvent {
    LinkState(LoraLinkState),
}

pub type HilinkBridgeChannel =
    Channel<CriticalSectionRawMutex, HilinkBridgeFrame, HILINK_BRIDGE_CHANNEL_DEPTH>;
pub type HilinkBridgeSender =
    Sender<'static, CriticalSectionRawMutex, HilinkBridgeFrame, HILINK_BRIDGE_CHANNEL_DEPTH>;
pub type HilinkBridgeReceiver =
    Receiver<'static, CriticalSectionRawMutex, HilinkBridgeFrame, HILINK_BRIDGE_CHANNEL_DEPTH>;
pub type StatusLedCommandChannel = Channel<CriticalSectionRawMutex, RgbLedCommand, 4>;
pub type StatusLedCommandReceiver = Receiver<'static, CriticalSectionRawMutex, RgbLedCommand, 4>;
pub type StatusIndicatorChannel = Channel<CriticalSectionRawMutex, StatusIndicatorEvent, 16>;
pub type StatusIndicatorSender = Sender<'static, CriticalSectionRawMutex, StatusIndicatorEvent, 16>;
pub type StatusIndicatorReceiver =
    Receiver<'static, CriticalSectionRawMutex, StatusIndicatorEvent, 16>;

pub static HOST_TO_LORA_P0_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static HOST_TO_LORA_P1_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static HOST_TO_LORA_P2_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static HOST_TO_LORA_P3_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static HOST_TO_LORA_P4_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static LORA_TO_HOST_CHANNEL: HilinkBridgeChannel = Channel::new();
pub static STATUS_LED_COMMAND_CHANNEL: StatusLedCommandChannel = Channel::new();
pub static STATUS_INDICATOR_CHANNEL: StatusIndicatorChannel = Channel::new();

pub fn host_to_lora_sender(priority: HilinkBridgePriority) -> HilinkBridgeSender {
    match priority {
        HilinkBridgePriority::P0Critical => HOST_TO_LORA_P0_CHANNEL.sender(),
        HilinkBridgePriority::P1Command => HOST_TO_LORA_P1_CHANNEL.sender(),
        HilinkBridgePriority::P2Event => HOST_TO_LORA_P2_CHANNEL.sender(),
        HilinkBridgePriority::P3Snapshot => HOST_TO_LORA_P3_CHANNEL.sender(),
        HilinkBridgePriority::P4Background => HOST_TO_LORA_P4_CHANNEL.sender(),
    }
}
