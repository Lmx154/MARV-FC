//! Telemetry producers.
//!
//! For now we keep the existing compact `TelemetrySample` format (carried via
//! MAVLink `EncapsulatedData`) to avoid schema churn. Later we can evolve this
//! into multiple logical frames (attitude, GPS, health, ...) and map those to
//! MAVLink in the protocol layer.

pub use crate::protocol::mavlink::encode::{TelemetrySample, TelemetrySource};
