//! Link MAC configuration command helpers.
//!
//! Goal: allow driving tick/slot/FAST budget at runtime without reflashing,
//! while keeping a single configuration struct (`LinkMacConfig`) as the contract.

use mavio::dialects::common::messages;
use mavio::dialects::common::enums::MavCmd;
use mavio::protocol::V2;
use mavio::Frame;

use crate::coms::transport::lora::mac::{LinkMacConfig, SlotMode, INNER_MTU};
use crate::protocol::mavlink::encode::{build_frame_from_msg, MavEndpointConfig};

/// MAVLink common message id for `COMMAND_LONG`.
pub const MAVLINK_MSG_ID_COMMAND_LONG: u32 = 76;

/// MARV-FC private command id for setting LoRa tick MAC configuration.
///
/// Chosen from the MAV_CMD_USER_* range.
pub const MARV_CMD_LINK_MAC_CONFIG: u16 = 31_000;

/// Discriminator to avoid accidentally parsing unrelated `COMMAND_LONG` frames.
///
/// This is carried in `COMMAND_LONG.param4`.
///
/// Note: keep this an integer representable exactly in `f32` (<= 2^24).
pub const LINK_MAC_CONFIG_MAGIC_F32: f32 = 424_242.0;

/// Version for the link-mac-config payload encoding.
///
/// This is carried in `COMMAND_LONG.param5`.
pub const LINK_MAC_CONFIG_VERSION_F32: f32 = 1.0;

/// Result of inspecting a MAVLink2 frame for a link-mac-config `COMMAND_LONG`.
#[derive(Clone, Copy, Debug)]
pub enum LinkMacConfigCmdParse {
    /// Not a link-mac-config command.
    NotLinkMacConfig,
    /// Is link-mac-config, but addressed to a different sys/comp.
    NotForUs,
    /// Is link-mac-config and addressed to us, but failed magic/version validation.
    Rejected {
        sender_system: u8,
        sender_component: u8,
    },
    /// Is link-mac-config, addressed to us, and decoded successfully.
    Accepted {
        cfg: LinkMacConfig,
        sender_system: u8,
        sender_component: u8,
    },
}

/// Build a MAVLink2 `COMMAND_LONG` frame that carries `LinkMacConfig`.
///
/// Encoding (params):
/// - param1: tick_hz (float, but intended as integer Hz)
/// - param2: slot_mode (0=UuuD, 1=UdUd, 2=Uddd)
/// - param3: fast_max_bytes
/// - param4: magic discriminator (`LINK_MAC_CONFIG_MAGIC_F32`)
/// - param5: encoding version (`LINK_MAC_CONFIG_VERSION_F32`)
pub fn build_link_mac_config_command_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    target_system: u8,
    target_component: u8,
    link_cfg: LinkMacConfig,
) -> Frame<V2> {
    // Use the standard USER_1 command id (31000) as our carrier.
    // This avoids requiring a custom dialect while still giving us a stable command id.
    let command = MavCmd::User1;

    let slot_mode_code: f32 = match link_cfg.slot_mode {
        SlotMode::UuuD => 0.0,
        SlotMode::UdUd => 1.0,
        SlotMode::Uddd => 2.0,
    };

    let msg = messages::CommandLong {
        target_system,
        target_component,
        command,
        confirmation: 0,
        param1: link_cfg.tick_hz as f32,
        param2: slot_mode_code,
        param3: link_cfg.fast_max_bytes as f32,
        param4: LINK_MAC_CONFIG_MAGIC_F32,
        param5: LINK_MAC_CONFIG_VERSION_F32,
        param6: 0.0,
        param7: 0.0,
    };

    build_frame_from_msg(cfg, seq, &msg)
}

/// Try to parse a `LinkMacConfig` update from a raw MAVLink2 frame.
///
/// This is a lightweight parser intended for `no_std` firmware loops that mostly
/// forward bytes. It does not validate CRC/signature.
///
/// Returns `(cfg, sender_system, sender_component)`.
pub fn try_parse_link_mac_config_cmd(
    frame_bytes: &[u8],
    local_system: u8,
    local_component: u8,
) -> Option<(LinkMacConfig, u8, u8)> {
    match parse_link_mac_config_cmd(frame_bytes, local_system, local_component) {
        LinkMacConfigCmdParse::Accepted {
            cfg,
            sender_system,
            sender_component,
        } => Some((cfg, sender_system, sender_component)),
        _ => None,
    }
}

/// Parse a link-mac-config `COMMAND_LONG` and classify it for ACK handling.
pub fn parse_link_mac_config_cmd(
    frame_bytes: &[u8],
    local_system: u8,
    local_component: u8,
) -> LinkMacConfigCmdParse {
    const MAV2_MAGIC: u8 = 0xFD;
    const MAV2_HEADER_LEN: usize = 10;

    if frame_bytes.len() < MAV2_HEADER_LEN {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }
    if frame_bytes[0] != MAV2_MAGIC {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }

    let payload_len = frame_bytes[1] as usize;
    if frame_bytes.len() < MAV2_HEADER_LEN + payload_len {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }

    let sender_system = frame_bytes[5];
    let sender_component = frame_bytes[6];

    let msg_id = (frame_bytes[7] as u32)
        | ((frame_bytes[8] as u32) << 8)
        | ((frame_bytes[9] as u32) << 16);
    if msg_id != MAVLINK_MSG_ID_COMMAND_LONG {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }

    // COMMAND_LONG payload layout (33 bytes):
    // param1..7: f32 LE (28 bytes)
    // command: u16 LE (2 bytes)
    // target_system: u8
    // target_component: u8
    // confirmation: u8
    if payload_len < 33 {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }

    let p = &frame_bytes[MAV2_HEADER_LEN..MAV2_HEADER_LEN + payload_len];
    let command_id = u16::from_le_bytes([p[28], p[29]]);
    if command_id != MARV_CMD_LINK_MAC_CONFIG {
        return LinkMacConfigCmdParse::NotLinkMacConfig;
    }

    // Validate target system/component.
    let target_system = p[30];
    if target_system != 0 && target_system != local_system {
        return LinkMacConfigCmdParse::NotForUs;
    }
    let target_component = p[31];
    if target_component != 0 && target_component != local_component {
        return LinkMacConfigCmdParse::NotForUs;
    }

    // Validate discriminator + version to avoid false positives.
    let magic = f32::from_le_bytes([p[12], p[13], p[14], p[15]]);
    if magic != LINK_MAC_CONFIG_MAGIC_F32 {
        return LinkMacConfigCmdParse::Rejected {
            sender_system,
            sender_component,
        };
    }
    let version = f32::from_le_bytes([p[16], p[17], p[18], p[19]]);
    if version != LINK_MAC_CONFIG_VERSION_F32 {
        return LinkMacConfigCmdParse::Rejected {
            sender_system,
            sender_component,
        };
    }

    let tick_hz_f = f32::from_le_bytes([p[0], p[1], p[2], p[3]]);
    let slot_mode_f = f32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    let fast_max_bytes_f = f32::from_le_bytes([p[8], p[9], p[10], p[11]]);

    // Interpret as integers with clamping.
    let mut tick_hz = tick_hz_f as i32;
    if tick_hz < 1 {
        tick_hz = 1;
    }
    if tick_hz > u16::MAX as i32 {
        tick_hz = u16::MAX as i32;
    }

    let slot_mode = match slot_mode_f as i32 {
        0 => SlotMode::UuuD,
        1 => SlotMode::UdUd,
        2 => SlotMode::Uddd,
        _ => SlotMode::UdUd,
    };

    let mut fast_max_bytes = fast_max_bytes_f as i32;
    if fast_max_bytes < 0 {
        fast_max_bytes = 0;
    }
    let fast_max_bytes = (fast_max_bytes as u32)
        .min(INNER_MTU as u32)
        .min(u16::MAX as u32) as u16;

    LinkMacConfigCmdParse::Accepted {
        cfg: LinkMacConfig {
            tick_hz: tick_hz as u16,
            slot_mode,
            fast_max_bytes,
        },
        sender_system,
        sender_component,
    }
}
