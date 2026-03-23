//! Health aggregation and watchdog supervision services.

pub mod deadlines;
pub mod fault_aggregation;
pub mod liveness;
pub mod watchdog_supervisor;

pub use deadlines::DeadlineWindow;
pub use fault_aggregation::FaultAggregation;
pub use liveness::{LivenessClass, LivenessEvidence, LivenessUpdate};
pub use watchdog_supervisor::{
    FeedDecision, WatchdogContract, WatchdogEvaluation, WatchdogSource, WatchdogSupervisor,
};
