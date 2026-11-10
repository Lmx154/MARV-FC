//! Colours and the 24-bit wire word.

/// One LED colour, 8 bits per channel.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rgb {
    /// Red, 0 to 255.
    pub r: u8,
    /// Green, 0 to 255.
    pub g: u8,
    /// Blue, 0 to 255.
    pub b: u8,
}

impl Rgb {
    /// All channels off.
    pub const OFF: Self = Self::new(0, 0, 0);
    /// Red at full level.
    pub const RED: Self = Self::new(255, 0, 0);
    /// Green at full level.
    pub const GREEN: Self = Self::new(0, 255, 0);
    /// Blue at full level.
    pub const BLUE: Self = Self::new(0, 0, 255);
    /// Red and green at full level.
    pub const YELLOW: Self = Self::new(255, 255, 0);
    /// Green and blue at full level.
    pub const CYAN: Self = Self::new(0, 255, 255);
    /// Red and blue at full level.
    pub const MAGENTA: Self = Self::new(255, 0, 255);
    /// All channels at full level.
    pub const WHITE: Self = Self::new(255, 255, 255);

    /// A colour from its three channels.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Every channel multiplied by `brightness / 255`, rounded to nearest: 255 leaves the colour
    /// unchanged and 0 turns it off.
    pub const fn scaled(self, brightness: u8) -> Self {
        Self {
            r: scale(self.r, brightness),
            g: scale(self.g, brightness),
            b: scale(self.b, brightness),
        }
    }

    /// The word the transports send: green in bits 31..24, red in 23..16, blue in 15..8, and
    /// bits 7..0 zero. The LED takes the 24 bits from bit 31 down (datasheet page 4, "Composition
    /// of 24bit Data": order GRB, high bit first).
    pub const fn wire(self) -> u32 {
        (self.g as u32) << 24 | (self.r as u32) << 16 | (self.b as u32) << 8
    }
}

const fn scale(channel: u8, brightness: u8) -> u8 {
    ((channel as u16 * brightness as u16 + 127) / 255) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_word_is_grb_with_green_in_the_top_byte() {
        assert_eq!(Rgb::new(0x3C, 0xA5, 0x0F).wire(), 0xA53C_0F00);
        assert_eq!(Rgb::RED.wire(), 0x00FF_0000);
        assert_eq!(Rgb::GREEN.wire(), 0xFF00_0000);
        assert_eq!(Rgb::BLUE.wire(), 0x0000_FF00);
        assert_eq!(Rgb::WHITE.wire(), 0xFFFF_FF00);
        assert_eq!(Rgb::OFF.wire(), 0);
    }

    #[test]
    fn named_colours_combine_the_primaries() {
        assert_eq!(Rgb::YELLOW, Rgb::new(255, 255, 0));
        assert_eq!(Rgb::CYAN, Rgb::new(0, 255, 255));
        assert_eq!(Rgb::MAGENTA, Rgb::new(255, 0, 255));
        assert_eq!(Rgb::default(), Rgb::OFF);
    }

    #[test]
    fn scaling_keeps_the_ends_and_rounds_to_nearest() {
        let c = Rgb::new(255, 128, 1);
        assert_eq!(c.scaled(255), c);
        assert_eq!(c.scaled(0), Rgb::OFF);
        assert_eq!(Rgb::WHITE.scaled(32), Rgb::new(32, 32, 32));
        assert_eq!(c.scaled(128), Rgb::new(128, 64, 1));
        for b in 0..=255u8 {
            let s = Rgb::WHITE.scaled(b);
            assert_eq!(s, Rgb::new(b, b, b));
        }
    }
}
