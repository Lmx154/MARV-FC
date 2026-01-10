//! MAVLink inbound message handling.

use heapless::Vec;
use mavio::dialects::ardupilotmega::Ardupilotmega;
use mavio::dialects::common::enums as common_enums;
use mavio::dialects::common::messages as common_messages;
use mavio::protocol::V2;
use mavio::Frame;

use crate::params::ParamRegistry;
use crate::policies::fc_state::{CommandOutcome, FcCustomMode, FcState, FcStatePolicy};
use crate::protocol::mavlink::encode::{
    build_frame_from_msg, build_param_value_frame, build_statustext,
    build_statustext_frame, find_visible_param_by_name, parse_param_value,
    raw_index_from_visible, MavEndpointConfig,
};

/// Simplified DEVICE_OP_READ payload for device-side handlers.
#[derive(Clone, Debug)]
pub struct DeviceOpReadRequest {
    pub request_id: u32,
    pub bustype: u8,
    pub bus: u8,
    pub address: u8,
    pub regstart: u8,
    pub count: u8,
}

/// Simplified DEVICE_OP_WRITE payload for device-side handlers.
#[derive(Clone, Debug)]
pub struct DeviceOpWriteRequest {
    pub request_id: u32,
    pub bustype: u8,
    pub bus: u8,
    pub address: u8,
    pub regstart: u8,
    pub count: u8,
    pub data: [u8; 128],
}

/// Handler output decision.
pub enum MessageHandlerResult {
    SendFrame(Frame<V2>),
    SendFrames(Vec<Frame<V2>, 2>),
    SendAllParams,
    DeviceOpRead(DeviceOpReadRequest),
    DeviceOpWrite(DeviceOpWriteRequest),
    NoResponse,
    Unhandled,
}

/// Decode a MAVLink frame and apply command/parameter handlers.
pub fn dispatch_mavlink_message(
    frame: Frame<V2>,
    cfg: MavEndpointConfig,
    seq: u8,
    params: &mut ParamRegistry,
    fc_state: &mut FcState,
    policy: FcStatePolicy,
    statustext_en: bool,
) -> (MessageHandlerResult, bool) {
    let msg = match frame.decode::<Ardupilotmega>() {
        Ok(m) => m,
        Err(_) => return (MessageHandlerResult::Unhandled, false),
    };

    let mut params_changed = false;

    match msg {
        Ardupilotmega::ParamRequestList(req) => {
            if !is_target(req.target_system, req.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }
            (MessageHandlerResult::SendAllParams, false)
        }
        Ardupilotmega::ParamRequestRead(req) => {
            if !is_target(req.target_system, req.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }
            let raw_idx = if req.param_index >= 0 {
                raw_index_from_visible(req.param_index as u16)
            } else {
                let name = match param_id_to_str(&req.param_id) {
                    Some(n) => n,
                    None => return (MessageHandlerResult::NoResponse, false),
                };
                find_visible_param_by_name(name)
            };

            if let Some(idx) = raw_idx {
                if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                    return (MessageHandlerResult::SendFrame(reply), false);
                }
            }
            (MessageHandlerResult::NoResponse, false)
        }
        Ardupilotmega::ParamSet(req) => {
            if !is_target(req.target_system, req.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }
            let name = match param_id_to_str(&req.param_id) {
                Some(n) => n,
                None => return (MessageHandlerResult::NoResponse, false),
            };
            let raw_idx = match find_visible_param_by_name(name) {
                Some(idx) => idx,
                None => return (MessageHandlerResult::NoResponse, false),
            };
            let def = match params.def_by_index(raw_idx) {
                Some(def) => def,
                None => return (MessageHandlerResult::NoResponse, false),
            };
            let new_val = parse_param_value(def, req.param_value);
            let old_val = params.get_by_index(raw_idx);
            if old_val != Some(new_val) {
                if params.set_by_index(raw_idx, new_val).is_ok() {
                    params_changed = true;
                }
            }
            if let Some(reply) = build_param_value_frame(cfg, seq, params, raw_idx) {
                (MessageHandlerResult::SendFrame(reply), params_changed)
            } else {
                (MessageHandlerResult::NoResponse, params_changed)
            }
        }
        Ardupilotmega::CommandLong(cmd) => {
            if !is_target(cmd.target_system, cmd.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }

            let command = match common_enums::MavCmd::try_from(cmd.command as u16) {
                Ok(command) => command,
                Err(_) => return (MessageHandlerResult::Unhandled, false),
            };

            match command {
                common_enums::MavCmd::ComponentArmDisarm => {
                    let arm = cmd.param1 >= 0.5;
                    let outcome = fc_state.request_arm(arm, policy);
                    let ack = build_command_ack_frame(
                        cfg,
                        seq,
                        command,
                        outcome.mav_result,
                        cmd.target_system,
                        cmd.target_component,
                    );
                    if statustext_en {
                        if let Some(text) = outcome.statustext {
                            let mut frames: Vec<Frame<V2>, 2> = Vec::new();
                            let _ = frames.push(ack);
                            let status = build_statustext(text);
                            let status_frame =
                                build_statustext_frame(cfg, seq.wrapping_add(1), status);
                            let _ = frames.push(status_frame);
                            return (MessageHandlerResult::SendFrames(frames), false);
                        }
                    }
                    (MessageHandlerResult::SendFrame(ack), false)
                }
                common_enums::MavCmd::DoSetMode => {
                    let requested = FcCustomMode::from_raw(cmd.param2 as u32);
                    let outcome = match requested {
                        Some(mode) => fc_state.request_mode(mode, policy),
                        None => CommandOutcome::unsupported(fc_state.snapshot()),
                    };
                    let ack = build_command_ack_frame(
                        cfg,
                        seq,
                        command,
                        outcome.mav_result,
                        cmd.target_system,
                        cmd.target_component,
                    );
                    if statustext_en {
                        if let Some(text) = outcome.statustext {
                            let mut frames: Vec<Frame<V2>, 2> = Vec::new();
                            let _ = frames.push(ack);
                            let status = build_statustext(text);
                            let status_frame =
                                build_statustext_frame(cfg, seq.wrapping_add(1), status);
                            let _ = frames.push(status_frame);
                            return (MessageHandlerResult::SendFrames(frames), false);
                        }
                    }
                    (MessageHandlerResult::SendFrame(ack), false)
                }
                _ => (MessageHandlerResult::Unhandled, false),
            }
        }
        Ardupilotmega::DeviceOpRead(req) => {
            if !is_target(req.target_system, req.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }
            let out = DeviceOpReadRequest {
                request_id: req.request_id,
                bustype: req.bustype as u8,
                bus: req.bus,
                address: req.address,
                regstart: req.regstart,
                count: req.count,
            };
            (MessageHandlerResult::DeviceOpRead(out), false)
        }
        Ardupilotmega::DeviceOpWrite(req) => {
            if !is_target(req.target_system, req.target_component, cfg) {
                return (MessageHandlerResult::Unhandled, false);
            }
            let out = DeviceOpWriteRequest {
                request_id: req.request_id,
                bustype: req.bustype as u8,
                bus: req.bus,
                address: req.address,
                regstart: req.regstart,
                count: req.count,
                data: req.data,
            };
            (MessageHandlerResult::DeviceOpWrite(out), false)
        }
        _ => (MessageHandlerResult::Unhandled, false),
    }
}

fn is_target(target_system: u8, target_component: u8, cfg: MavEndpointConfig) -> bool {
    let sys_ok = target_system == 0 || target_system == cfg.sys_id;
    let comp_ok = target_component == 0 || target_component == cfg.comp_id;
    sys_ok && comp_ok
}

fn param_id_to_str(id: &[u8; 16]) -> Option<&str> {
    let len = id.iter().position(|&b| b == 0).unwrap_or(id.len());
    core::str::from_utf8(&id[..len]).ok()
}

fn build_command_ack_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    command: common_enums::MavCmd,
    result: common_enums::MavResult,
    target_system: u8,
    target_component: u8,
) -> Frame<V2> {
    let msg = common_messages::CommandAck {
        command,
        result,
        progress: 0,
        result_param2: 0,
        target_system,
        target_component,
    };
    build_frame_from_msg(cfg, seq, &msg)
}
