//! Fault, reset-cause, and watchdog messages.

pub mod health_report;
pub mod reset_reason;
pub mod watchdog_status;

pub use health_report::HealthReport;
pub use reset_reason::ResetReason;
pub use watchdog_status::WatchdogStatus;
