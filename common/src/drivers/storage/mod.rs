//! Storage drivers and adapters.

pub mod microsd;

pub use microsd::{DEFAULT_FLUSH_EVERY_LINES, MicrosdLogger, MicrosdLoggerConfig};
