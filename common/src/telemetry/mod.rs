//! Telemetry layer: producers + scheduling glue.
//!
//! This module is intentionally transport-agnostic.
//!
//! Step 3 goal (see docs): introduce a reusable producer/scheduler layer so
//! multiple links (USB, radio/LoRa) can share the same producer(s) but run at
//! different rates.

pub mod producer;
pub mod scheduler;
