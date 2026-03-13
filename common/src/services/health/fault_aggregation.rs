//! Aggregated health summary for supervisory use.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultAggregation {
    pub critical_faults: u16,
    pub degraded_faults: u16,
}
