//! MAVLink message handlers
//!
//! Pure protocol handling - no hardware or runtime dependencies.

use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::Common;
use mavio::dialects::ardupilotmega::Ardupilotmega;

use crate::params::ParamRegistry;

use crate::protocol::mavlink::encode::{
    build_param_value_frame, handle_param_request_read, handle_param_set, MavEndpointConfig,
};

/// Result of handling a MAVLink message.
pub enum MessageHandlerResult {
    /// Send a response frame.
    SendFrame(Frame<V2>),
    /// Send all parameters (PARAM_REQUEST_LIST).
    SendAllParams,
    /// No response needed.
    NoResponse,
    /// Message not recognized.
    Unhandled,

    /// Perform a device operation (I2C/SPI/etc) and reply.
    DeviceOpRead(DeviceOpReadRequest),

    /// Perform a device write operation (I2C/SPI/etc) and reply.
    DeviceOpWrite(DeviceOpWriteRequest),
}

#[derive(Clone, Copy)]
pub struct DeviceOpReadRequest {
    pub request_id: u32,
    pub bustype: u8,
    pub bus: u8,
    pub busname: [u8; 40],
    pub address: u8,
    pub regstart: u8,
    pub count: u8,
}

#[derive(Clone, Copy)]
pub struct DeviceOpWriteRequest {
    pub request_id: u32,
    pub bustype: u8,
    pub bus: u8,
    pub busname: [u8; 40],
    pub address: u8,
    pub regstart: u8,
    pub count: u8,
    pub data: [u8; 128],
}

fn handle_device_op(
    msg: &Ardupilotmega,
    cfg: MavEndpointConfig,
) -> MessageHandlerResult {
    match msg {
        Ardupilotmega::DeviceOpRead(req) => {
            // Validate target system/component (0 allows broadcast)
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return MessageHandlerResult::NoResponse;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return MessageHandlerResult::NoResponse;
            }

            MessageHandlerResult::DeviceOpRead(DeviceOpReadRequest {
                request_id: req.request_id,
                bustype: req.bustype as u8,
                bus: req.bus,
                busname: req.busname,
                address: req.address,
                regstart: req.regstart,
                count: req.count,
            })
        }
        Ardupilotmega::DeviceOpWrite(req) => {
            // Validate target system/component (0 allows broadcast)
            if req.target_system != 0 && req.target_system != cfg.sys_id {
                return MessageHandlerResult::NoResponse;
            }
            if req.target_component != 0 && req.target_component != cfg.comp_id {
                return MessageHandlerResult::NoResponse;
            }

            MessageHandlerResult::DeviceOpWrite(DeviceOpWriteRequest {
                request_id: req.request_id,
                bustype: req.bustype as u8,
                bus: req.bus,
                busname: req.busname,
                address: req.address,
                regstart: req.regstart,
                count: req.count,
                data: req.data,
            })
        }
        _ => MessageHandlerResult::Unhandled,
    }
}

/// Handle incoming PARAM_REQUEST_LIST message.
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

/// Handle incoming PARAM_REQUEST_READ message.
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

/// Handle incoming PARAM_SET message.
/// Returns (result, dirty_flag) where dirty_flag indicates if parameter was modified.
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
                return (MessageHandlerResult::SendFrame(reply), true);
            }
        }
    }
    (MessageHandlerResult::NoResponse, false)
}

/// Handle incoming HEARTBEAT message.
pub fn handle_heartbeat_msg(_frame: &Frame<V2>) -> MessageHandlerResult {
    MessageHandlerResult::NoResponse
}

/// Handle incoming STATUSTEXT message.
pub fn handle_statustext_msg(_msg: &Common) -> MessageHandlerResult {
    MessageHandlerResult::NoResponse
}

/// Dispatch incoming MAVLink message to appropriate handler.
pub fn dispatch_mavlink_message(
    frame: Frame<V2>,
    cfg: MavEndpointConfig,
    seq: u8,
    params: &mut ParamRegistry,
) -> (MessageHandlerResult, bool) {
    // First, try ArduPilotMega dialect for extra GCS tooling support (e.g. Mission Planner DEVICE_OP_*).
    if let Ok(msg) = frame.decode::<Ardupilotmega>() {
        let res = handle_device_op(&msg, cfg);
        if !matches!(res, MessageHandlerResult::Unhandled) {
            return (res, false);
        }
    }

    match frame.decode::<Common>() {
        Ok(msg @ Common::ParamRequestList(_)) => (handle_param_request_list(&msg, cfg, params), false),
        Ok(msg @ Common::ParamRequestRead(_)) => (handle_param_request_read_msg(&msg, cfg, seq, params), false),
        Ok(msg @ Common::ParamSet(_)) => handle_param_set_msg(&msg, cfg, seq, params),
        Ok(Common::Heartbeat(_)) => (handle_heartbeat_msg(&frame), false),
        Ok(msg @ Common::Statustext(_)) => (handle_statustext_msg(&msg), false),
        Ok(_) => (MessageHandlerResult::Unhandled, false),
        Err(_) => (MessageHandlerResult::Unhandled, false),
    }
}
