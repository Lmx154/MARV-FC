//! MAVLink-common helpers shared across transports (LoRa, UART, etc).
//! This module now re-exports builders from `mavlink2::msg` so framing is centralized.

pub use crate::mavlink2::msg::{
    build_statustext, build_statustext_frame, build_telemetry_frame, statustext_to_str,
    MavEndpointConfig, TelemetrySample, TelemetrySource, STATUSTEXT_CAP,
};
