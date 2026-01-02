//! MAVLink-common helpers shared across transports (LoRa, UART, etc).
//! This module re-exports builders from `protocol::mavlink::encode` so framing is centralized.

pub use crate::protocol::mavlink::encode::{
    build_statustext, build_statustext_frame, build_telemetry_frame, statustext_to_str,
    MavEndpointConfig, TelemetrySample, TelemetrySource, STATUSTEXT_CAP,
};
