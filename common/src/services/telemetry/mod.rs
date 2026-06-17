//! Telemetry packaging services belong here.

pub mod mavlink_stream;
pub mod radio_link;

pub use mavlink_stream::MavlinkStreamPump;
pub use radio_link::{
    RadioEmitError, RadioReply, RadioTelemetryConfig, run_radio_telemetry_emitter,
};
