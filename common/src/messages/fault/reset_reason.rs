//! Portable reset-cause vocabulary shared across targets and SITL.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetReason {
    Unknown,
    PowerOn,
    Software,
    Watchdog,
    BrownOut,
}
