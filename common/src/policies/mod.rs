//! Project-wide policy modules (HAL-agnostic).

#[cfg(feature = "mavlink")]
pub mod fc_state;

#[cfg(feature = "mavlink")]
pub use fc_state::*;
