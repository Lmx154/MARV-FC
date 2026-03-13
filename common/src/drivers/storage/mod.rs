//! Storage drivers and adapters.

pub mod microsd;

pub use microsd::{MicrosdLogger, MicrosdLoggerConfig, DEFAULT_FLUSH_EVERY_LINES};
