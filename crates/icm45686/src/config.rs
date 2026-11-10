//! Typed configuration and the encoders that turn it into register values.

use crate::reg;

/// Accelerometer full-scale range, ACCEL_UI_FS_SEL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFullScale {
    /// +-32 g.
    G32 = 0,
    /// +-16 g.
    G16 = 1,
    /// +-8 g.
    G8 = 2,
    /// +-4 g.
    G4 = 3,
    /// +-2 g.
    G2 = 4,
}

impl AccelFullScale {
    /// ACCEL_UI_FS_SEL code.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Full-scale range in g.
    pub const fn range_g(self) -> f32 {
        match self {
            Self::G32 => 32.0,
            Self::G16 => 16.0,
            Self::G8 => 8.0,
            Self::G4 => 4.0,
            Self::G2 => 2.0,
        }
    }

    /// Sensitivity in LSB per g (1024 at +-32 g up to 16384 at +-2 g).
    pub const fn lsb_per_g(self) -> f32 {
        32768.0 / self.range_g()
    }
}

/// Gyroscope full-scale range, GYRO_UI_FS_SEL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroFullScale {
    /// +-4000 dps.
    Dps4000 = 0,
    /// +-2000 dps.
    Dps2000 = 1,
    /// +-1000 dps.
    Dps1000 = 2,
    /// +-500 dps.
    Dps500 = 3,
    /// +-250 dps.
    Dps250 = 4,
    /// +-125 dps.
    Dps125 = 5,
    /// +-62.5 dps.
    Dps62_5 = 6,
    /// +-31.25 dps.
    Dps31_25 = 7,
    /// +-15.625 dps.
    Dps15_625 = 8,
}

impl GyroFullScale {
    /// GYRO_UI_FS_SEL code.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Full-scale range in degrees per second.
    pub const fn range_dps(self) -> f32 {
        match self {
            Self::Dps4000 => 4000.0,
            Self::Dps2000 => 2000.0,
            Self::Dps1000 => 1000.0,
            Self::Dps500 => 500.0,
            Self::Dps250 => 250.0,
            Self::Dps125 => 125.0,
            Self::Dps62_5 => 62.5,
            Self::Dps31_25 => 31.25,
            Self::Dps15_625 => 15.625,
        }
    }

    /// Sensitivity in LSB per degree per second (131.07 at +-250 dps).
    pub const fn lsb_per_dps(self) -> f32 {
        32768.0 / self.range_dps()
    }
}

/// Output data rate, ACCEL_ODR / GYRO_ODR. The device runs in low-noise mode, which supports
/// 12.5 Hz and above; the three lowest rates are low-power-only and `init` rejects them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Odr {
    /// 6.4 kHz (low noise only).
    Hz6400 = 0x3,
    /// 3.2 kHz (low noise only).
    Hz3200 = 0x4,
    /// 1.6 kHz (low noise only).
    Hz1600 = 0x5,
    /// 800 Hz (low noise only).
    Hz800 = 0x6,
    /// 400 Hz.
    Hz400 = 0x7,
    /// 200 Hz.
    Hz200 = 0x8,
    /// 100 Hz.
    Hz100 = 0x9,
    /// 50 Hz.
    Hz50 = 0xA,
    /// 25 Hz.
    Hz25 = 0xB,
    /// 12.5 Hz.
    Hz12_5 = 0xC,
    /// 6.25 Hz (low power only).
    Hz6_25 = 0xD,
    /// 3.125 Hz (low power only).
    Hz3_125 = 0xE,
    /// 1.5625 Hz (low power only).
    Hz1_5625 = 0xF,
}

impl Odr {
    /// ODR code, bits 3:0 of ACCEL_CONFIG0 / GYRO_CONFIG0.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Nominal rate in hertz (CLKIN 32 kHz scales it exactly; the internal oscillator is about 0.4 % fast).
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz6400 => 6400.0,
            Self::Hz3200 => 3200.0,
            Self::Hz1600 => 1600.0,
            Self::Hz800 => 800.0,
            Self::Hz400 => 400.0,
            Self::Hz200 => 200.0,
            Self::Hz100 => 100.0,
            Self::Hz50 => 50.0,
            Self::Hz25 => 25.0,
            Self::Hz12_5 => 12.5,
            Self::Hz6_25 => 6.25,
            Self::Hz3_125 => 3.125,
            Self::Hz1_5625 => 1.5625,
        }
    }

    /// Whether low-noise mode supports this rate.
    pub const fn low_noise(self) -> bool {
        self.code() <= Self::Hz12_5.code()
    }
}

/// Source of the sensor clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Internal oscillator.
    Internal,
    /// External 32 kHz clock on pin 9 (CLKIN).
    ExternalClkin,
}

/// Driver configuration for `init`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_fs: AccelFullScale,
    /// Gyroscope full scale.
    pub gyro_fs: GyroFullScale,
    /// Output data rate of both sensors.
    pub odr: Odr,
    /// Sensor clock source.
    pub clock: ClockSource,
    /// Route the data-ready event to INT1 as an 8 us push-pull, active-high pulse.
    pub int1_data_ready: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            accel_fs: AccelFullScale::G4,
            gyro_fs: GyroFullScale::Dps2000,
            odr: Odr::Hz800,
            clock: ClockSource::Internal,
            int1_data_ready: false,
        }
    }
}

/// ACCEL_CONFIG0 value.
pub const fn accel_config0(fs: AccelFullScale, odr: Odr) -> u8 {
    (fs.code() << 4) | odr.code()
}

/// GYRO_CONFIG0 value.
pub const fn gyro_config0(fs: GyroFullScale, odr: Odr) -> u8 {
    (fs.code() << 4) | odr.code()
}

/// INT1_CONFIG0 value: data-ready enable only (reset-done reporting off).
pub const fn int1_config0(data_ready: bool) -> u8 {
    if data_ready { reg::DRDY } else { 0 }
}
