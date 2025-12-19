//! Parameter protocol handler - integrates ParamRegistry with MAVLink frames.
//!
//! This module provides a ready-to-use handler for MAVLink parameter protocol
//! that you can plug into your UART or LoRa message dispatch loop.

#![allow(dead_code)]

use mavio::dialects::common::Common;
use mavio::Frame;
use mavio::protocol::V2;

use crate::params::ParamRegistry;
use crate::protocol::mavlink::encode::{
    build_param_value_frame, handle_param_request_read, handle_param_set, MavEndpointConfig,
};

/// Result of processing a parameter-related message.
pub enum ParamHandlerResult {
    /// No parameter message detected or not for this component.
    NotHandled,
    /// Parameter message handled, reply frame(s) ready to send.
    Reply(Frame<V2>),
    /// Multiple reply frames needed (e.g., PARAM_REQUEST_LIST).
    /// Returns the number of parameters to stream.
    StreamAll(u16),
}

/// Process an incoming MAVLink frame for parameter protocol messages.
pub fn handle_param_message(
    frame: &Frame<V2>,
    registry: &mut ParamRegistry,
    cfg: MavEndpointConfig,
    seq: u8,
) -> ParamHandlerResult {
    let Ok(msg) = frame.decode::<Common>() else {
        return ParamHandlerResult::NotHandled;
    };

    match msg {
        Common::ParamRequestList(req) => {
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }

            ParamHandlerResult::StreamAll(registry.count())
        }

        Common::ParamRequestRead(req) => {
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }

            if let Some(idx) = handle_param_request_read(registry, &req) {
                if let Some(reply) = build_param_value_frame(cfg, seq, registry, idx) {
                    return ParamHandlerResult::Reply(reply);
                }
            }

            ParamHandlerResult::NotHandled
        }

        Common::ParamSet(req) => {
            if req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }

            if let Some(idx) = handle_param_set(registry, &req) {
                if let Some(reply) = build_param_value_frame(cfg, seq, registry, idx) {
                    return ParamHandlerResult::Reply(reply);
                }
            }

            ParamHandlerResult::NotHandled
        }

        _ => ParamHandlerResult::NotHandled,
    }
}
