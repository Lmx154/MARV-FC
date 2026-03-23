//! Runtime state messages shared across targets.

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FlightPhase {
    Init,
    Hil,
    Ready,
    Active,
    Fault,
}

impl FlightPhase {
    pub const fn is_hil(self) -> bool {
        matches!(self, Self::Hil)
    }

    pub const fn is_fault(self) -> bool {
        matches!(self, Self::Fault)
    }
}
