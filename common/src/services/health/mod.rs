//! Health aggregation and watchdog supervision services.

pub mod deadlines;
pub mod fault_aggregation;
pub mod liveness;
pub mod tasks;
pub mod watchdog_supervisor;

pub use deadlines::DeadlineWindow;
pub use fault_aggregation::FaultAggregation;
pub use liveness::{LivenessClass, LivenessEvidence, LivenessUpdate};
pub use tasks::{
    FlightPhaseSource, LivenessUpdateReceiver, PureSubscriber, WatchdogDriver,
    run_watchdog_liveness_loop, run_watchdog_supervisor_loop,
};
pub use watchdog_supervisor::{
    FeedDecision, WatchdogContract, WatchdogEvaluation, WatchdogSource, WatchdogSupervisor,
};
