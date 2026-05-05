use crate::channels::{
    HILINK_BRIDGE_FRAME_BYTES, HOST_TO_LORA_P0_CHANNEL, HOST_TO_LORA_P1_CHANNEL,
    HOST_TO_LORA_P2_CHANNEL, HOST_TO_LORA_P3_CHANNEL, HOST_TO_LORA_P4_CHANNEL, HilinkBridgeFrame,
};
use crate::radio_dialect::{normal, policy};

pub fn classify_host_frame(frame: &HilinkBridgeFrame) -> policy::RadioPriority {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let Ok(packet) = normal::decode_frame(frame.as_slice(), &mut raw) else {
        return policy::RadioPriority::P3Snapshot;
    };

    policy::classify_normal_msg(&packet)
}

pub fn receive_next_host_frame() -> Option<HilinkBridgeFrame> {
    if let Ok(frame) = HOST_TO_LORA_P0_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P1_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P2_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P3_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P4_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else {
        None
    }
}
