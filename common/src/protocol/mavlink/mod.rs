#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use mavio::error::FrameError;

pub mod encode;
pub mod handlers;
pub mod link_authority;
pub mod link_mac_config;
pub mod link_metrics;
pub mod rf_reconfig;
pub mod param_handler;
pub mod telemetry;
pub mod transport_lora;
pub mod transport_uart;

pub use transport_lora::{
    recv_frame_over_lora, recv_frame_over_lora_mac, send_frame_over_lora, send_frame_over_lora_mac,
};
pub use transport_uart::{recv_frame_over_uart, send_frame_over_uart};

/// Public re-exports so higher layers can use mavio types directly.
pub mod prelude {
    pub use mavio::dialects;
    pub use mavio::protocol::V2;
    pub use mavio::Frame;

    pub use crate::protocol::mavlink::{
        recv_frame_over_lora, recv_frame_over_lora_mac, recv_frame_over_uart, send_frame_over_lora,
        send_frame_over_lora_mac, send_frame_over_uart,
        MavError,
    };

    pub use crate::protocol::mavlink::encode::*;
}

/// Error type for MAVLink2 framing over different transports.
#[derive(Debug, defmt::Format)]
pub enum MavError {
    /// Underlying LoRaLink error (reliability / radio).
    Link(crate::coms::transport::lora::link::LinkError),
    /// UART/serial transport error.
    Serial,
    /// MAVLink frame is too large for LoRaLink MTU or UART scratch.
    EncodeTooBig,
    /// MAVLink frame (de)serialization error.
    Frame,
}

impl From<crate::coms::transport::lora::link::LinkError> for MavError {
    fn from(e: crate::coms::transport::lora::link::LinkError) -> Self {
        MavError::Link(e)
    }
}

impl From<FrameError> for MavError {
    fn from(_e: FrameError) -> Self {
        MavError::Frame
    }
}
