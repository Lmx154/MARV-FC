//! Backend selection for acquisition ownership.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, defmt::Format)]
pub enum SensorBackend {
    #[default]
    Real,
    Hil,
    Replay,
}

impl SensorBackend {
    pub const fn wire_code(self) -> u8 {
        match self {
            Self::Real => 0,
            Self::Hil => 1,
            Self::Replay => 2,
        }
    }

    pub const fn from_wire_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Real),
            1 => Some(Self::Hil),
            2 => Some(Self::Replay),
            _ => None,
        }
    }

    pub fn from_wire_param(value: f32) -> Option<Self> {
        if !value.is_finite() || value < 0.0 {
            return None;
        }

        let code = value as u8;
        if (code as f32) != value {
            return None;
        }

        Self::from_wire_code(code)
    }
}
