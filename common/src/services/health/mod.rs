//! Health aggregation and watchdog supervision services.

pub mod watchdog_supervisor;
pub mod liveness;
pub mod deadlines;
pub mod fault_aggregation;

pub use deadlines::DeadlineWindow;
pub use fault_aggregation::FaultAggregation;
pub use liveness::{LivenessClass, LivenessEvidence};
pub use watchdog_supervisor::{FeedDecision, WatchdogSupervisor};
