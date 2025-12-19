//! Complete MAVLink + Parameter Integration Example
//!
//! This file demonstrates how to wire the ParamRegistry into your
//! existing MAVLink message loop (UART or LoRa).
//!
//! Copy this pattern into your device firmware's communication task.

#![allow(dead_code)]
#![allow(unused_imports)]

use defmt::{info, warn};
use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::Common;

use common::params::{ParamRegistry, ParamId};
use common::mavlink2::{
    MavError,
    recv_frame_over_uart,
    send_frame_over_uart,
};
use common::mavlink2::msg::{
    MavEndpointConfig,
    build_param_value_frame,
    handle_param_request_read,
    handle_param_set,
    build_statustext,
    build_statustext_frame,
};
use common::coms::uart_coms::{AsyncUartBus, MAVLINK_MAX_FRAME};
use common::utils::delay::DelayMs;

/// Example: MAVLink message handler with integrated parameter protocol.
///
/// This function shows the complete pattern for handling incoming MAVLink
/// frames and responding to parameter requests.
pub async fn mavlink_message_loop<U>(
    uart: &mut U,
    params: &mut ParamRegistry,
    cfg: MavEndpointConfig,
) where
    U: AsyncUartBus,
    U::Error: defmt::Format,
{
    info!("MAVLink: starting message loop");
    
    let mut rx_buf = [0u8; MAVLINK_MAX_FRAME];
    let mut tx_buf = [0u8; MAVLINK_MAX_FRAME];
    let mut seq: u8 = 0;

    // Send initial STATUSTEXT to announce we're alive
    let status = build_statustext("MARV-FC ready");
    let frame = build_statustext_frame(cfg, seq, status);
    seq = seq.wrapping_add(1);
    let _ = send_frame_over_uart(uart, &frame, &mut tx_buf).await;

    loop {
        // Receive a frame
        let frame: Frame<V2> = match recv_frame_over_uart(uart, &mut rx_buf).await {
            Ok(f) => f,
            Err(MavError::Serial) => {
                warn!("MAVLink: UART transport error");
                continue;
            }
            Err(e) => {
                warn!("MAVLink: recv error {:?}", e);
                continue;
            }
        };

        // Decode the message
        let msg = match frame.decode::<Common>() {
            Ok(m) => m,
            Err(_) => {
                warn!("MAVLink: decode error");
                continue;
            }
        };

        // Handle the message
        match msg {
            // ==================== PARAMETER PROTOCOL ====================
            
            Common::ParamRequestList(req) => {
                // Only respond if request is for our component (or broadcast)
                if req.target_system != 0 && req.target_system != cfg.sys_id {
                    continue;
                }
                if req.target_component != 0 && req.target_component != cfg.comp_id {
                    continue;
                }

                info!("MAVLink: PARAM_REQUEST_LIST - sending {} params", params.count());

                // Send all parameters
                for idx in 0..params.count() {
                    if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                        seq = seq.wrapping_add(1);
                        if let Err(e) = send_frame_over_uart(uart, &reply, &mut tx_buf).await {
                            warn!("MAVLink: failed to send param {} - {:?}", idx, e);
                            break;
                        }
                    }
                }
                
                info!("MAVLink: PARAM_REQUEST_LIST complete");
            }

            Common::ParamRequestRead(req) => {
                // Check target
                if req.target_system != 0 && req.target_system != cfg.sys_id {
                    continue;
                }
                if req.target_component != 0 && req.target_component != cfg.comp_id {
                    continue;
                }

                // Find and send the requested parameter
                if let Some(idx) = handle_param_request_read(params, &req) {
                    if let Some(def) = params.def_by_index(idx) {
                        info!("MAVLink: PARAM_REQUEST_READ #{} ({})", idx, def.name);
                    }

                    if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                        seq = seq.wrapping_add(1);
                        let _ = send_frame_over_uart(uart, &reply, &mut tx_buf).await;
                    }
                } else {
                    warn!("MAVLink: PARAM_REQUEST_READ - param not found");
                }
            }

            Common::ParamSet(req) => {
                // Check target
                if req.target_system != cfg.sys_id {
                    continue;
                }
                if req.target_component != cfg.comp_id {
                    continue;
                }

                // Attempt to set the parameter
                if let Some(idx) = handle_param_set(params, &req) {
                    if let Some(def) = params.def_by_index(idx) {
                        if let Some(val) = params.get_by_index(idx) {
                            info!("MAVLink: PARAM_SET {} = {:?}", def.name, val);
                        }
                    }

                    // ACK by sending back the updated value
                    if let Some(reply) = build_param_value_frame(cfg, seq, params, idx) {
                        seq = seq.wrapping_add(1);
                        let _ = send_frame_over_uart(uart, &reply, &mut tx_buf).await;
                    }
                } else {
                    warn!("MAVLink: PARAM_SET failed - param not found or type mismatch");
                }
            }

            // ==================== OTHER MESSAGE TYPES ====================
            
            Common::Heartbeat(hb) => {
                info!("MAVLink: HEARTBEAT from sys={} comp={}", 
                      frame.system_id(), frame.component_id());
                
                // Optionally send heartbeat reply
                // ...
            }

            Common::Statustext(msg) => {
                // Extract text for logging
                let end = msg.text.iter().position(|&b| b == 0).unwrap_or(msg.text.len());
                let text = core::str::from_utf8(&msg.text[..end]).unwrap_or("<non-utf8>");
                info!("MAVLink: STATUSTEXT: {}", text);
            }

            Common::CommandLong(cmd) => {
                info!("MAVLink: COMMAND_LONG cmd={}", cmd.command);
                // Handle commands here
                // Example: MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_COMPONENT_ARM_DISARM, etc.
            }

            _ => {
                // Unhandled message type
                // Can log at trace level or ignore
            }
        }
    }
}

/// Example: How to use parameters in your sensor tasks
pub fn example_using_params(params: &ParamRegistry) {
    // Check if a sensor is enabled
    if params.bool(ParamId::Bmi088En) {
        let rate = params.u32(ParamId::Bmi088RateHz);
        info!("BMI088 enabled at {} Hz", rate);
        
        // Configure sensor based on params
        // ...
    }

    // Apply calibration offsets
    let acc_bias_x = params.f32(ParamId::AccBiasX);
    let acc_bias_y = params.f32(ParamId::AccBiasY);
    let acc_bias_z = params.f32(ParamId::AccBiasZ);
    
    info!("Accel bias: [{}, {}, {}]", acc_bias_x, acc_bias_y, acc_bias_z);

    // Apply filter settings
    let lpf_hz = params.f32(ParamId::ImuAccLpfHz);
    if lpf_hz > 0.0 {
        info!("Applying {} Hz LPF to accelerometer", lpf_hz);
        // Configure filter
        // ...
    }

    // Use system parameters
    let sys_id = params.u32(ParamId::SysId);
    let comp_id = params.u32(ParamId::CompId);
    let cfg = MavEndpointConfig {
        sys_id: sys_id as u8,
        comp_id: comp_id as u8,
    };
    
    info!("MAVLink endpoint: sys={} comp={}", cfg.sys_id, cfg.comp_id);
}

/// Example: Initialize and configure the parameter registry at startup
pub fn example_param_setup() -> ParamRegistry {
    // Create registry with defaults
    let mut params = ParamRegistry::new();
    
    // Optionally override some defaults programmatically
    // (Later these will come from persistent storage / SD card)
    use common::params::{ParamValue, ParamId};
    
    // Example: set system ID from hardware config
    let _ = params.set_by_index(ParamId::SysId as u16, ParamValue::U32(1));
    let _ = params.set_by_index(ParamId::CompId as u16, ParamValue::U32(1));
    
    // Example: enable specific sensors
    let _ = params.set_by_index(ParamId::Bmi088En as u16, ParamValue::Bool(true));
    let _ = params.set_by_index(ParamId::Bmm350En as u16, ParamValue::Bool(true));
    
    params
}
