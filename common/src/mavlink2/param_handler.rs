//! Parameter protocol handler - integrates ParamRegistry with MAVLink transport.
//!
//! This module provides a ready-to-use handler for MAVLink parameter protocol
//! that you can plug into your UART or LoRa message dispatch loop.

#![allow(dead_code)]

use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::Common;

use crate::params::ParamRegistry;
use crate::mavlink2::msg::{
    MavEndpointConfig, 
    handle_param_request_read, 
    handle_param_set,
    build_param_value_frame,
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
/// Returns frames to send back, or None if not a parameter message.
///
/// ## Usage
/// ```ignore
/// let result = handle_param_message(&frame, &mut params, cfg, seq);
/// match result {
///     ParamHandlerResult::Reply(reply_frame) => {
///         send_frame(&reply_frame).await;
///         seq = seq.wrapping_add(1);
///     }
///     ParamHandlerResult::StreamAll(count) => {
///         for idx in 0..count {
///             if let Some(reply) = build_param_value_frame(cfg, seq, &params, idx) {
///                 send_frame(&reply).await;
///                 seq = seq.wrapping_add(1);
///             }
///         }
///     }
///     ParamHandlerResult::NotHandled => {
///         // Handle other message types
///     }
/// }
/// ```
pub fn handle_param_message(
    frame: &Frame<V2>,
    registry: &mut ParamRegistry,
    cfg: MavEndpointConfig,
    seq: u8,
) -> ParamHandlerResult {
    // Decode the frame
    let Ok(msg) = frame.decode::<Common>() else {
        return ParamHandlerResult::NotHandled;
    };

    match msg {
        // PARAM_REQUEST_LIST: send all parameters
        Common::ParamRequestList(req) => {
            // Only respond if request is for our component (or broadcast)
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }
            
            ParamHandlerResult::StreamAll(registry.count())
        }

        // PARAM_REQUEST_READ: send one parameter
        Common::ParamRequestRead(req) => {
            // Check target
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }

            // Find the parameter
            if let Some(idx) = handle_param_request_read(registry, &req) {
                if let Some(reply) = build_param_value_frame(cfg, seq, registry, idx) {
                    return ParamHandlerResult::Reply(reply);
                }
            }
            
            ParamHandlerResult::NotHandled
        }

        // PARAM_SET: update parameter and ACK with PARAM_VALUE
        Common::ParamSet(req) => {
            // Check target
            if req.target_system != cfg.sys_id {
                return ParamHandlerResult::NotHandled;
            }
            if req.target_component != cfg.comp_id {
                return ParamHandlerResult::NotHandled;
            }

            // Attempt to set the parameter
            if let Some(idx) = handle_param_set(registry, &req) {
                // ACK by sending back the updated value
                if let Some(reply) = build_param_value_frame(cfg, seq, registry, idx) {
                    return ParamHandlerResult::Reply(reply);
                }
            }
            
            ParamHandlerResult::NotHandled
        }

        _ => ParamHandlerResult::NotHandled,
    }
}
