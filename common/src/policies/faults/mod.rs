//! Fault escalation and reset policies belong here.

pub mod escalation;
pub mod reset_policy;

pub use escalation::FaultEscalation;
pub use reset_policy::ResetPolicy;
