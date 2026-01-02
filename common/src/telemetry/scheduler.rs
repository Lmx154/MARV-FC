//! Backwards-compatible re-export.
//!
//! The scheduler is a link/comms concern (per-link pacing/budget), so its
//! canonical home is now `crate::coms::scheduler`.

pub use crate::coms::scheduler::*;
