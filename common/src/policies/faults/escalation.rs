//! Escalation outcomes for health and watchdog faults.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultEscalation {
    ReportOnly,
    Degrade,
    Reset,
}
