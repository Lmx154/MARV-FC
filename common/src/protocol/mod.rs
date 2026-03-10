//! Protocol-level modules (encoding/decoding, protocol handlers).
//!
//! This layer is intentionally transport-agnostic. Transport adapters live either
//! under `crate::coms::transport` or in small protocol/transport bridge modules.

pub mod packet;

