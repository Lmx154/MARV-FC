//! Staged RF reconfiguration command helpers.
//!
//! This implements the link-spec "transactional radio reconfiguration" primitive
//! using MAVLink2 `COMMAND_LONG` as a carrier (no custom dialect).
//!
//! Encoding uses only integers representable exactly in `f32` (<= 2^24).

use mavio::dialects::common::messages;
use mavio::dialects::common::enums::MavCmd;
use mavio::protocol::V2;
use mavio::Frame;

use crate::coms::transport::lora::lora_config::LoRaConfig;
use crate::protocol::mavlink::encode::{build_frame_from_msg, MavEndpointConfig};

/// MAVLink common message id for `COMMAND_LONG`.
pub const MAVLINK_MSG_ID_COMMAND_LONG: u32 = 76;

/// MARV-FC private command id for staged RF reconfiguration.
///
/// Chosen from the MAV_CMD_USER_* range.
pub const MARV_CMD_RF_RECONFIG: u16 = 31_001;

/// Discriminator to avoid accidentally parsing unrelated `COMMAND_LONG` frames.
///
/// This is carried in `COMMAND_LONG.param6`.
///
/// Note: keep this an integer representable exactly in `f32` (<= 2^24).
pub const RF_RECONFIG_MAGIC_F32: f32 = 515_151.0;

/// Version for the rf-reconfig payload encoding.
///
/// This is carried in `COMMAND_LONG.param7`.
pub const RF_RECONFIG_VERSION_F32: f32 = 1.0;

/// RF reconfiguration operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RfReconfigOp {
    Propose,
    Commit,
    Abort,
}

/// Minimal radio settings that are safe to change only via staged apply.
///
/// These fields are chosen to match the existing `LoRaConfig` knobs while keeping
/// a compact, float-safe encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfReconfigSettings {
    /// Frequency in kHz (not Hz) to keep the integer <= 2^24.
    pub freq_khz: u32,

    /// Spreading factor 5..12.
    pub sf: u8,

    /// Bandwidth code:
    /// - 0 = 125 kHz
    /// - 1 = 250 kHz
    /// - 2 = 500 kHz
    pub bw_code: u8,

    /// Coding-rate code:
    /// - 0 = 4/5
    /// - 1 = 4/6
    /// - 2 = 4/7
    /// - 3 = 4/8
    pub cr_code: u8,

    /// LoRa sync word.
    pub sync_word: u16,
}

impl RfReconfigSettings {
    pub fn apply_to_base(self, mut base: LoRaConfig) -> LoRaConfig {
        base.freq_hz = self.freq_khz.saturating_mul(1_000);
        base.sf = self.sf;
        base.bw = match self.bw_code {
            0 => 0x04, // 125 kHz (Semtech encoding)
            1 => 0x05, // 250 kHz
            2 => 0x06, // 500 kHz
            _ => base.bw,
        };
        base.cr = match self.cr_code {
            0 => 0x01, // 4/5
            1 => 0x02, // 4/6
            2 => 0x03, // 4/7
            3 => 0x04, // 4/8
            _ => base.cr,
        };
        base.sync_word = self.sync_word;
        base
    }
}

/// Parsed staged-RF-reconfig command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfReconfigCommand {
    pub op: RfReconfigOp,
    pub txn_id: u16,

    /// Only meaningful for `Propose`.
    pub apply_after_ms: u32,

    /// Only meaningful for `Propose`.
    pub settings: Option<RfReconfigSettings>,
}

/// Result of inspecting a MAVLink2 frame for an rf-reconfig `COMMAND_LONG`.
#[derive(Clone, Copy, Debug)]
pub enum RfReconfigCmdParse {
    NotRfReconfig,
    NotForUs,
    Rejected {
        sender_system: u8,
        sender_component: u8,
    },
    Accepted {
        cmd: RfReconfigCommand,
        sender_system: u8,
        sender_component: u8,
    },
}

/// Build a MAVLink2 `COMMAND_LONG` frame that carries an RF reconfiguration operation.
///
/// Encoding (params):
/// - param1: op code (1=PROPOSE, 2=COMMIT, 3=ABORT)
/// - param2: txn_id (integer)
/// - param3: apply_after_ms (integer; PROPOSE only, otherwise 0)
/// - param4: packed0 (u32 encoded in f32): freq_khz (bits 0..19) + sf (bits 20..23)
/// - param5: packed1 (u32 encoded in f32): bw_code (0..3) + cr_code (bits 4..7) + sync_word (bits 8..23)
/// - param6: magic discriminator (`RF_RECONFIG_MAGIC_F32`)
/// - param7: encoding version (`RF_RECONFIG_VERSION_F32`)
pub fn build_rf_reconfig_command_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    target_system: u8,
    target_component: u8,
    cmd: RfReconfigCommand,
) -> Frame<V2> {
    let command = MavCmd::User2;

    let op_code = match cmd.op {
        RfReconfigOp::Propose => 1.0,
        RfReconfigOp::Commit => 2.0,
        RfReconfigOp::Abort => 3.0,
    };

    let (packed0_f, packed1_f) = if let (RfReconfigOp::Propose, Some(s)) = (cmd.op, cmd.settings) {
        let freq_khz = s.freq_khz.min((1 << 20) - 1);
        let sf = (s.sf.min(15) as u32) & 0x0F;
        let packed0 = freq_khz | (sf << 20);

        let bw = (s.bw_code as u32) & 0x0F;
        let cr = (s.cr_code as u32) & 0x0F;
        let sw = (s.sync_word as u32) & 0xFFFF;
        let packed1 = bw | (cr << 4) | (sw << 8);

        (packed0 as f32, packed1 as f32)
    } else {
        (0.0, 0.0)
    };

    let msg = messages::CommandLong {
        target_system,
        target_component,
        command,
        confirmation: 0,
        param1: op_code,
        param2: cmd.txn_id as f32,
        param3: cmd.apply_after_ms as f32,
        param4: packed0_f,
        param5: packed1_f,
        param6: RF_RECONFIG_MAGIC_F32,
        param7: RF_RECONFIG_VERSION_F32,
    };

    build_frame_from_msg(cfg, seq, &msg)
}

/// Parse an rf-reconfig `COMMAND_LONG` and classify it for ACK handling.
pub fn parse_rf_reconfig_cmd(
    frame_bytes: &[u8],
    local_system: u8,
    local_component: u8,
) -> RfReconfigCmdParse {
    const MAV2_MAGIC: u8 = 0xFD;
    const MAV2_HEADER_LEN: usize = 10;

    if frame_bytes.len() < MAV2_HEADER_LEN {
        return RfReconfigCmdParse::NotRfReconfig;
    }
    if frame_bytes[0] != MAV2_MAGIC {
        return RfReconfigCmdParse::NotRfReconfig;
    }

    let payload_len = frame_bytes[1] as usize;
    if frame_bytes.len() < MAV2_HEADER_LEN + payload_len {
        return RfReconfigCmdParse::NotRfReconfig;
    }

    let sender_system = frame_bytes[5];
    let sender_component = frame_bytes[6];

    let msg_id = (frame_bytes[7] as u32)
        | ((frame_bytes[8] as u32) << 8)
        | ((frame_bytes[9] as u32) << 16);
    if msg_id != MAVLINK_MSG_ID_COMMAND_LONG {
        return RfReconfigCmdParse::NotRfReconfig;
    }

    // COMMAND_LONG payload layout (33 bytes):
    // param1..7: f32 LE (28 bytes)
    // command: u16 LE (2 bytes)
    // target_system: u8
    // target_component: u8
    // confirmation: u8
    if payload_len < 33 {
        return RfReconfigCmdParse::NotRfReconfig;
    }

    let p = &frame_bytes[MAV2_HEADER_LEN..MAV2_HEADER_LEN + payload_len];
    let command_id = u16::from_le_bytes([p[28], p[29]]);
    if command_id != MARV_CMD_RF_RECONFIG {
        return RfReconfigCmdParse::NotRfReconfig;
    }

    let target_system = p[30];
    if target_system != 0 && target_system != local_system {
        return RfReconfigCmdParse::NotForUs;
    }
    let target_component = p[31];
    if target_component != 0 && target_component != local_component {
        return RfReconfigCmdParse::NotForUs;
    }

    let magic = f32::from_le_bytes([p[20], p[21], p[22], p[23]]);
    if magic != RF_RECONFIG_MAGIC_F32 {
        return RfReconfigCmdParse::Rejected {
            sender_system,
            sender_component,
        };
    }
    let version = f32::from_le_bytes([p[24], p[25], p[26], p[27]]);
    if version != RF_RECONFIG_VERSION_F32 {
        return RfReconfigCmdParse::Rejected {
            sender_system,
            sender_component,
        };
    }

    let op_f = f32::from_le_bytes([p[0], p[1], p[2], p[3]]);
    let txn_id_f = f32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    let apply_after_ms_f = f32::from_le_bytes([p[8], p[9], p[10], p[11]]);
    let packed0_f = f32::from_le_bytes([p[12], p[13], p[14], p[15]]);
    let packed1_f = f32::from_le_bytes([p[16], p[17], p[18], p[19]]);

    let op = match op_f as i32 {
        1 => RfReconfigOp::Propose,
        2 => RfReconfigOp::Commit,
        3 => RfReconfigOp::Abort,
        _ => {
            return RfReconfigCmdParse::Rejected {
                sender_system,
                sender_component,
            }
        }
    };

    let txn_id_i = txn_id_f as i32;
    if txn_id_i < 0 || txn_id_i > u16::MAX as i32 {
        return RfReconfigCmdParse::Rejected {
            sender_system,
            sender_component,
        };
    }

    let apply_after_i = apply_after_ms_f as i32;
    let apply_after_ms = if apply_after_i < 0 { 0 } else { apply_after_i as u32 };

    let (settings, apply_after_ms) = if op == RfReconfigOp::Propose {
        // Packed ints must be <= 2^24-1 to be representable exactly.
        let packed0 = packed0_f as i32;
        let packed1 = packed1_f as i32;
        if packed0 < 0 || packed0 > 0x00FF_FFFF || packed1 < 0 || packed1 > 0x00FF_FFFF {
            return RfReconfigCmdParse::Rejected {
                sender_system,
                sender_component,
            };
        }

        let packed0 = packed0 as u32;
        let packed1 = packed1 as u32;

        let freq_khz = packed0 & ((1 << 20) - 1);
        let sf = ((packed0 >> 20) & 0x0F) as u8;

        let bw_code = (packed1 & 0x0F) as u8;
        let cr_code = ((packed1 >> 4) & 0x0F) as u8;
        let sync_word = ((packed1 >> 8) & 0xFFFF) as u16;

        // Basic sanity.
        if !(5..=12).contains(&sf) {
            return RfReconfigCmdParse::Rejected {
                sender_system,
                sender_component,
            };
        }
        if bw_code > 2 || cr_code > 3 {
            return RfReconfigCmdParse::Rejected {
                sender_system,
                sender_component,
            };
        }

        (
            Some(RfReconfigSettings {
                freq_khz,
                sf,
                bw_code,
                cr_code,
                sync_word,
            }),
            apply_after_ms,
        )
    } else {
        (None, 0)
    };

    RfReconfigCmdParse::Accepted {
        cmd: RfReconfigCommand {
            op,
            txn_id: txn_id_i as u16,
            apply_after_ms,
            settings,
        },
        sender_system,
        sender_component,
    }
}
