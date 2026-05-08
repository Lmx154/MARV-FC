//! Control-domain messages belong here.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RgbLedCommand {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl RgbLedCommand {
    pub const OFF: Self = Self::new(0, 0, 0);

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StaticLedCommand {
    pub on: bool,
}

impl StaticLedCommand {
    pub const OFF: Self = Self { on: false };
    pub const ON: Self = Self { on: true };
}
