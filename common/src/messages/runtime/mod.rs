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
    pub const fn wire_code(self) -> u8 {
        match self {
            Self::Init => 0,
            Self::Hil => 1,
            Self::Ready => 2,
            Self::Active => 3,
            Self::Fault => 4,
        }
    }

    pub const fn from_wire_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Init),
            1 => Some(Self::Hil),
            2 => Some(Self::Ready),
            3 => Some(Self::Active),
            4 => Some(Self::Fault),
            _ => None,
        }
    }

    pub const fn is_hil(self) -> bool {
        matches!(self, Self::Hil)
    }

    pub const fn is_fault(self) -> bool {
        matches!(self, Self::Fault)
    }
}
