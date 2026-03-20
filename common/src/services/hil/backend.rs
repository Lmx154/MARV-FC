//! Backend selection for acquisition ownership.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, defmt::Format)]
pub enum SensorBackend {
    #[default]
    Real,
    Hil,
    Replay,
}
