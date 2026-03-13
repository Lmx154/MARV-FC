//! Reset policy options consumed by supervisory logic.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetPolicy {
    Never,
    OnFeedCriticalLoss,
    OnSupervisorFault,
}
