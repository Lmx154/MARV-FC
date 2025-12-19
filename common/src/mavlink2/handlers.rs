//! MAVLink message handlers
//! 
//! Pure protocol handling - no hardware or runtime dependencies

use crate::mavlink2::msg::{
    build_param_value_frame, handle_param_request_read, handle_param_set,
};
use crate::mavlink2::prelude::{dialects::common::Common, Frame, V2};
use crate::params::ParamRegistry;
use crate::tasks::coms::MavEndpointConfig;

/// Result of handling a MAVLink message
pub enum MessageHandlerResult {
    /// Send a response frame
    SendFrame(Frame<V2>),
    /// Send all parameters (PARAM_REQUEST_LIST)
    SendAllParams,
    /// No response needed
    NoResponse,
    /// Message not recognized
    Unhandled,
}

/// Handle incoming PARAM_REQUEST_LIST message
pub fn handle_param_request_list(
    msg: &Common,
    cfg: MavEndpointConfig,
    _params: &ParamRegistry,
) -> MessageHandlerResult {
    if let Common::ParamRequestList(req) = msg {
        // Validate target system/component
        if req.target_system != 0 && req.target_system != cfg.sys_id {
            return MessageHandlerResult::NoResponse;
        }
        if req.target_component != 0 && req.target_component != cfg.comp_id {
            return MessageHandlerResult::NoResponse;
        }

        MessageHandlerResult::SendAllParams
    } else {
        MessageHandlerResult::Unhandled
    }
}

/// Handle incoming PARAM_REQUEST_READ message
pub fn handle_param_request_read_msg(
    msg: &Common,
    cfg: MavEndpointConfig,
    seq: u8,
    params: &ParamRegistry,
) -> MessageHandlerResult {
    if let Common::ParamRequestRead(req) = msg {
        // Validate target system/component
        if req.target_system != 0 && req.target_system != cfg.sys_id {
            return MessageHandlerResult::NoResponse;
        }
        if req.target_component != 0 && req.target_component != cfg.comp_id {
            return MessageHandlerResult::NoResponse;
        }

        if let Some(idx) = handle_param_request_read(params, req) {
            if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                return MessageHandlerResult::SendFrame(reply);
            }
        }
    }
    MessageHandlerResult::NoResponse
}

/// Handle incoming PARAM_SET message
/// Returns (result, dirty_flag) where dirty_flag indicates if parameter was modified
pub fn handle_param_set_msg(
    msg: &Common,
    cfg: MavEndpointConfig,
    seq: u8,
    params: &mut ParamRegistry,
) -> (MessageHandlerResult, bool) {
    if let Common::ParamSet(req) = msg {
        // Validate target system/component
        if req.target_system != cfg.sys_id {
            return (MessageHandlerResult::NoResponse, false);
        }
        if req.target_component != cfg.comp_id {
            return (MessageHandlerResult::NoResponse, false);
        }

        if let Some(idx) = handle_param_set(params, req) {
            // ACK by sending back the updated value
            if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                return (MessageHandlerResult::SendFrame(reply), true); // Parameter was modified
            }
        }
    }
    (MessageHandlerResult::NoResponse, false)
}

/// Handle incoming HEARTBEAT message
pub fn handle_heartbeat_msg(
    _frame: &Frame<V2>,
) -> MessageHandlerResult {
    MessageHandlerResult::NoResponse
}

/// Handle incoming STATUSTEXT message
pub fn handle_statustext_msg(
    _msg: &Common,
) -> MessageHandlerResult {
    MessageHandlerResult::NoResponse
}

/// Dispatch incoming MAVLink message to appropriate handler
pub fn dispatch_mavlink_message(
    frame: Frame<V2>,
    cfg: MavEndpointConfig,
    seq: u8,
    params: &mut ParamRegistry,
) -> (MessageHandlerResult, bool) {
    match frame.decode::<Common>() {
        Ok(msg @ Common::ParamRequestList(_)) => {
            (handle_param_request_list(&msg, cfg, params), false)
        }
        Ok(msg @ Common::ParamRequestRead(_)) => {
            (handle_param_request_read_msg(&msg, cfg, seq, params), false)
        }
        Ok(msg @ Common::ParamSet(_)) => {
            handle_param_set_msg(&msg, cfg, seq, params)
        }
        Ok(Common::Heartbeat(_)) => (handle_heartbeat_msg(&frame), false),
        Ok(msg @ Common::Statustext(_)) => (handle_statustext_msg(&msg), false),
        Ok(_) => {
            (MessageHandlerResult::Unhandled, false)
        }
        Err(_) => {
            (MessageHandlerResult::Unhandled, false)
        }
    }
}
