// common/src/mavlink2/mod.rs
// Compatibility facade: the real MAVLink protocol implementation lives under
// `crate::protocol::mavlink`. Keep `crate::mavlink2::*` stable for device crates.

#![allow(dead_code)]
#![allow(async_fn_in_trait)]

pub use crate::protocol::mavlink::{
    MavError, recv_frame_over_lora, recv_frame_over_uart, send_frame_over_lora,
    send_frame_over_uart,
};

/// Public re-exports so higher layers can use mavio types directly.
pub mod prelude {
    pub use crate::protocol::mavlink::prelude::*;
}

/// Backwards-compatible module name for MAVLink frame/payload builders.
pub mod msg {
    pub use crate::protocol::mavlink::encode::*;
}

/// Backwards-compatible module name for protocol handlers.
pub mod handlers {
    pub use crate::protocol::mavlink::handlers::*;
}

/// Backwards-compatible module name for the param handler helper.
pub mod param_handler {
    pub use crate::protocol::mavlink::param_handler::*;
}
