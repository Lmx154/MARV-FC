//! Medium-loop portable task bodies belong here.

pub mod sensor_logging;

pub use sensor_logging::run_core0_sensor_logging_task;
