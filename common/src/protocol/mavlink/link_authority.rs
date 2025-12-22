//! Explicit Link Authority State (LAS) message helpers.

use mavio::protocol::V2;
use mavio::Frame;

use crate::protocol::mavlink::encode::MavEndpointConfig;

/// MAVLink message id for `MARV_LINK_AUTH` (custom dialect).
pub const MAVLINK_MSG_ID_MARV_LINK_AUTH: u32 = 42_002;
/// CRC extra for `MARV_LINK_AUTH` (per MAVLink mavparse).
pub const MARV_LINK_AUTH_CRC_EXTRA: u8 = 21;
pub const MARV_LINK_AUTH_PAYLOAD_LEN: usize = 5;

pub const LAS_STATE_UNKNOWN: u8 = 0;
pub const LAS_STATE_DISARMED: u8 = 1;
pub const LAS_STATE_ARMED: u8 = 2;

#[derive(Clone, Copy, Debug)]
pub struct LinkAuthority {
    pub state: u8,
    pub max_age_ms: u16,
    pub target_system: u8,
    pub target_component: u8,
}

/// Build a `MARV_LINK_AUTH` frame (manual payload/CRC).
pub fn build_link_authority_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    target_system: u8,
    target_component: u8,
    state: u8,
    max_age_ms: u16,
) -> Frame<V2> {
    // Wire order follows MAVLink field ordering (u16 first, then u8 fields).
    let mut payload = [0u8; MARV_LINK_AUTH_PAYLOAD_LEN];
    payload[0..2].copy_from_slice(&max_age_ms.to_le_bytes());
    payload[2] = target_system;
    payload[3] = target_component;
    payload[4] = state;

    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id)
        .component_id(cfg.comp_id)
        .sequence(seq)
        .message_id(MAVLINK_MSG_ID_MARV_LINK_AUTH)
        .payload(&payload)
        .crc_extra(MARV_LINK_AUTH_CRC_EXTRA)
        .build()
}

/// Parse a `MARV_LINK_AUTH` frame and filter by target system/component.
pub fn parse_link_authority_frame(
    frame: &Frame<V2>,
    local_system: u8,
    local_component: u8,
) -> Option<LinkAuthority> {
    if frame.message_id() != MAVLINK_MSG_ID_MARV_LINK_AUTH {
        return None;
    }

    let payload = frame.payload().bytes();
    if payload.len() < MARV_LINK_AUTH_PAYLOAD_LEN {
        return None;
    }

    let max_age_ms = u16::from_le_bytes([payload[0], payload[1]]);
    let target_system = payload[2];
    let target_component = payload[3];
    let state = payload[4];

    if target_system != 0 && target_system != local_system {
        return None;
    }
    if target_component != 0 && target_component != local_component {
        return None;
    }

    Some(LinkAuthority {
        state,
        max_age_ms,
        target_system,
        target_component,
    })
}
