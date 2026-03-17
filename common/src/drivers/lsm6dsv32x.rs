//! Generic LSM6DSV32X IMU Driver (Accelerometer + Gyroscope)
//!
//! Async, MCU-agnostic driver using embedded-hal 1.0 traits.
//! - SPI: embedded_hal_async::spi::SpiDevice<u8>
//! - Delays: provided by a simple async trait defined in common::utils::delay

#![allow(dead_code)]

use defmt::{Format, debug, info, warn};
use embedded_hal_async::spi::SpiDevice;

use crate::utils::delay::DelayMs;

mod reg {
    pub const WHO_AM_I: u8 = 0x0F;
    pub const WHO_AM_I_VAL: u8 = 0x70;

    pub const CTRL1: u8 = 0x10;
    pub const CTRL2: u8 = 0x11;
    pub const CTRL3: u8 = 0x12;
    pub const CTRL6: u8 = 0x15;
    pub const CTRL8: u8 = 0x17;

    pub const OUTX_L_G: u8 = 0x22;
    pub const OUTX_L_A: u8 = 0x28;

    pub const SPI_READ: u8 = 0x80;

    pub const CTRL3_SW_RESET: u8 = 0x01;
    pub const CTRL3_IF_INC: u8 = 0x04;
    pub const CTRL3_BDU: u8 = 0x40;
    pub const CTRL8_REQUIRED_ONE: u8 = 0x04;

    pub const ODR_1920: u8 = 0x0A;
    pub const OP_MODE_XL_HIGH_PERF: u8 = 0x00;
    pub const OP_MODE_G_HIGH_PERF: u8 = 0x00;

    pub const FS_G_2000DPS: u8 = 0x04;
    pub const FS_XL_16G: u8 = 0x02;
}

#[derive(Debug, Format)]
pub enum Error {
    Spi,
    ChipId(u8),
    ResetTimeout,
    InvalidCalibrationSamples,
}

#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Lsm6dsv32xCalibration {
    pub accel_offset: [i16; 3],
    pub gyro_offset: [i16; 3],
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelRange {
    G4,
    G8,
    G16,
    G32,
}

impl AccelRange {
    const fn reg_value(self) -> u8 {
        match self {
            Self::G4 => 0x00,
            Self::G8 => 0x01,
            Self::G16 => reg::FS_XL_16G,
            Self::G32 => 0x03,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroRange {
    Dps125,
    Dps250,
    Dps500,
    Dps1000,
    Dps2000,
    Dps4000,
}

impl GyroRange {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Dps125 => 0x00,
            Self::Dps250 => 0x01,
            Self::Dps500 => 0x02,
            Self::Dps1000 => 0x03,
            Self::Dps2000 => reg::FS_G_2000DPS,
            Self::Dps4000 => 0x0C,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelOdr {
    Hz1920,
}

impl AccelOdr {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Hz1920 => reg::ODR_1920,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroOdr {
    Hz1920,
}

impl GyroOdr {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Hz1920 => reg::ODR_1920,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelMode {
    HighPerformance,
}

impl AccelMode {
    const fn reg_value(self) -> u8 {
        match self {
            Self::HighPerformance => reg::OP_MODE_XL_HIGH_PERF,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroMode {
    HighPerformance,
}

impl GyroMode {
    const fn reg_value(self) -> u8 {
        match self {
            Self::HighPerformance => reg::OP_MODE_G_HIGH_PERF,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct Lsm6dsv32xConfig {
    pub accel_range: AccelRange,
    pub accel_odr: AccelOdr,
    pub accel_mode: AccelMode,
    pub gyro_range: GyroRange,
    pub gyro_odr: GyroOdr,
    pub gyro_mode: GyroMode,
}

impl Default for Lsm6dsv32xConfig {
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G16,
            accel_odr: AccelOdr::Hz1920,
            accel_mode: AccelMode::HighPerformance,
            gyro_range: GyroRange::Dps2000,
            gyro_odr: GyroOdr::Hz1920,
            gyro_mode: GyroMode::HighPerformance,
        }
    }
}

impl Lsm6dsv32xConfig {
    pub const fn accel_ctrl1_reg(self) -> u8 {
        self.accel_odr.reg_value() | (self.accel_mode.reg_value() << 4)
    }

    pub const fn gyro_ctrl2_reg(self) -> u8 {
        self.gyro_odr.reg_value() | (self.gyro_mode.reg_value() << 4)
    }
}

/// Raw accelerometer + gyro frame
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Lsm6dsv32xRaw {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

/// Generic LSM6DSV32X driver using async SPI device
pub struct Lsm6dsv32x<SPI>
where
    SPI: SpiDevice<u8>,
{
    spi: SPI,
    calibration: Lsm6dsv32xCalibration,
}

impl<SPI> Lsm6dsv32x<SPI>
where
    SPI: SpiDevice<u8>,
{
    /// Create a new LSM6DSV32X driver instance
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            calibration: Lsm6dsv32xCalibration::default(),
        }
    }

    /// Initialize the sensor with the default working configuration.
    pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.init_with_config(delay, Lsm6dsv32xConfig::default())
            .await
    }

    /// Initialize and apply an explicit runtime configuration.
    pub async fn init_with_config<D: DelayMs>(
        &mut self,
        delay: &mut D,
        config: Lsm6dsv32xConfig,
    ) -> Result<(), Error> {
        info!("LSM6DSV32X: Starting initialization sequence");

        self.soft_reset(delay).await?;
        self.verify_chip_id(delay).await?;
        self.enable_bdu_and_auto_increment().await?;
        self.apply_config(config).await?;

        info!("LSM6DSV32X: Initialization complete");
        Ok(())
    }

    /// Soft-reset the device and wait for the reset bit to clear.
    pub async fn soft_reset<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.write_reg(reg::CTRL3, reg::CTRL3_SW_RESET).await?;
        delay.delay_ms(5).await;

        for _ in 0..10 {
            let ctrl3 = self.read_reg(reg::CTRL3).await?;
            if (ctrl3 & reg::CTRL3_SW_RESET) == 0 {
                return Ok(());
            }
            delay.delay_ms(2).await;
        }

        warn!("LSM6DSV32X: Reset timeout");
        Err(Error::ResetTimeout)
    }

    /// Validate the chip ID with a few retries after reset.
    pub async fn verify_chip_id<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        let mut who = 0u8;

        for attempt in 0..5 {
            who = self.read_reg(reg::WHO_AM_I).await?;
            debug!("LSM6DSV32X: WHO_AM_I attempt {} -> 0x{:02X}", attempt, who);
            if who == reg::WHO_AM_I_VAL {
                return Ok(());
            }
            delay.delay_ms(2).await;
        }

        warn!(
            "LSM6DSV32X: ID mismatch 0x{:02X} (expected 0x{:02X})",
            who,
            reg::WHO_AM_I_VAL
        );
        Err(Error::ChipId(who))
    }

    /// Apply measurement range and ODR/performance configuration.
    pub async fn apply_config(&mut self, config: Lsm6dsv32xConfig) -> Result<(), Error> {
        self.set_gyro_range(config.gyro_range).await?;
        self.set_accel_range(config.accel_range).await?;
        self.set_accel_odr_mode(config.accel_odr, config.accel_mode)
            .await?;
        self.set_gyro_odr_mode(config.gyro_odr, config.gyro_mode)
            .await
    }

    /// For SPI devices like LSM6DSV32X, bus address selection is not used.
    pub const fn supports_address_selection() -> bool {
        false
    }

    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error> {
        self.write_reg(reg::CTRL8, reg::CTRL8_REQUIRED_ONE | range.reg_value())
            .await
    }

    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error> {
        self.write_reg(reg::CTRL6, range.reg_value()).await
    }

    pub async fn set_accel_odr_mode(
        &mut self,
        odr: AccelOdr,
        mode: AccelMode,
    ) -> Result<(), Error> {
        self.write_reg(reg::CTRL1, odr.reg_value() | (mode.reg_value() << 4))
            .await
    }

    pub async fn set_gyro_odr_mode(&mut self, odr: GyroOdr, mode: GyroMode) -> Result<(), Error> {
        self.write_reg(reg::CTRL2, odr.reg_value() | (mode.reg_value() << 4))
            .await
    }

    pub fn calibration(&self) -> Lsm6dsv32xCalibration {
        self.calibration
    }

    pub fn set_calibration(&mut self, calibration: Lsm6dsv32xCalibration) {
        self.calibration = calibration;
    }

    pub fn clear_calibration(&mut self) {
        self.calibration = Lsm6dsv32xCalibration::default();
    }

    /// Capture average gyro static bias in current conditions.
    pub async fn calibrate_gyro_bias(&mut self, samples: u16) -> Result<[i16; 3], Error> {
        if samples == 0 {
            return Err(Error::InvalidCalibrationSamples);
        }

        let mut sum = [0i32; 3];
        for _ in 0..samples {
            let sample = self.read_gyro_raw().await?;
            sum[0] += sample[0] as i32;
            sum[1] += sample[1] as i32;
            sum[2] += sample[2] as i32;
        }

        let denom = samples as i32;
        let offset = [
            (sum[0] / denom) as i16,
            (sum[1] / denom) as i16,
            (sum[2] / denom) as i16,
        ];
        self.calibration.gyro_offset = offset;
        Ok(offset)
    }

    /// Read accelerometer data (X, Y, Z) in raw i16 counts.
    pub async fn read_accel_raw(&mut self) -> Result<[i16; 3], Error> {
        let mut tx_buf = [0u8; 7];
        let mut rx_buf = [0u8; 7];
        tx_buf[0] = reg::OUTX_L_A | reg::SPI_READ;

        self.spi
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;

        let x = i16::from_le_bytes([rx_buf[1], rx_buf[2]]);
        let y = i16::from_le_bytes([rx_buf[3], rx_buf[4]]);
        let z = i16::from_le_bytes([rx_buf[5], rx_buf[6]]);
        Ok([x, y, z])
    }

    /// Read gyroscope data (X, Y, Z) in raw i16 counts.
    pub async fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error> {
        let mut tx_buf = [0u8; 7];
        let mut rx_buf = [0u8; 7];
        tx_buf[0] = reg::OUTX_L_G | reg::SPI_READ;

        self.spi
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;

        let x = i16::from_le_bytes([rx_buf[1], rx_buf[2]]);
        let y = i16::from_le_bytes([rx_buf[3], rx_buf[4]]);
        let z = i16::from_le_bytes([rx_buf[5], rx_buf[6]]);
        Ok([x, y, z])
    }

    /// Read accelerometer counts with current software calibration offsets applied.
    pub async fn read_accel(&mut self) -> Result<[i16; 3], Error> {
        let raw = self.read_accel_raw().await?;
        Ok(apply_offsets(raw, self.calibration.accel_offset))
    }

    /// Read gyroscope counts with current software calibration offsets applied.
    pub async fn read_gyro(&mut self) -> Result<[i16; 3], Error> {
        let raw = self.read_gyro_raw().await?;
        Ok(apply_offsets(raw, self.calibration.gyro_offset))
    }

    /// Combined calibrated read of both accelerometer and gyroscope.
    pub async fn read_raw(&mut self) -> Result<Lsm6dsv32xRaw, Error> {
        let raw = self.read_raw_uncalibrated().await?;
        Ok(Lsm6dsv32xRaw {
            accel: apply_offsets(raw.accel, self.calibration.accel_offset),
            gyro: apply_offsets(raw.gyro, self.calibration.gyro_offset),
        })
    }

    /// Combined uncalibrated read of both accelerometer and gyroscope.
    pub async fn read_raw_uncalibrated(&mut self) -> Result<Lsm6dsv32xRaw, Error> {
        let mut tx_buf = [0u8; 13];
        let mut rx_buf = [0u8; 13];
        tx_buf[0] = reg::OUTX_L_G | reg::SPI_READ;

        self.spi
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;

        let gx = i16::from_le_bytes([rx_buf[1], rx_buf[2]]);
        let gy = i16::from_le_bytes([rx_buf[3], rx_buf[4]]);
        let gz = i16::from_le_bytes([rx_buf[5], rx_buf[6]]);
        let ax = i16::from_le_bytes([rx_buf[7], rx_buf[8]]);
        let ay = i16::from_le_bytes([rx_buf[9], rx_buf[10]]);
        let az = i16::from_le_bytes([rx_buf[11], rx_buf[12]]);

        Ok(Lsm6dsv32xRaw {
            accel: [ax, ay, az],
            gyro: [gx, gy, gz],
        })
    }

    async fn enable_bdu_and_auto_increment(&mut self) -> Result<(), Error> {
        self.write_reg(reg::CTRL3, reg::CTRL3_BDU | reg::CTRL3_IF_INC)
            .await
    }

    async fn read_reg(&mut self, addr: u8) -> Result<u8, Error> {
        let tx_buf = [addr | reg::SPI_READ, 0x00];
        let mut rx_buf = [0u8; 2];

        self.spi
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;
        Ok(rx_buf[1])
    }

    async fn write_reg(&mut self, addr: u8, val: u8) -> Result<(), Error> {
        let tx_buf = [addr & 0x7F, val];
        self.spi.write(&tx_buf).await.map_err(|_| Error::Spi)
    }
}

fn apply_offsets(raw: [i16; 3], offsets: [i16; 3]) -> [i16; 3] {
    [
        raw[0].wrapping_sub(offsets[0]),
        raw[1].wrapping_sub(offsets[1]),
        raw[2].wrapping_sub(offsets[2]),
    ]
}

#[cfg(test)]
mod tests {
    use super::{
        AccelMode, AccelOdr, AccelRange, GyroMode, GyroOdr, GyroRange, Lsm6dsv32xCalibration,
        Lsm6dsv32xConfig, apply_offsets,
    };

    #[test]
    fn config_builds_expected_default_ctrl_regs() {
        let config = Lsm6dsv32xConfig {
            accel_range: AccelRange::G16,
            accel_odr: AccelOdr::Hz1920,
            accel_mode: AccelMode::HighPerformance,
            gyro_range: GyroRange::Dps2000,
            gyro_odr: GyroOdr::Hz1920,
            gyro_mode: GyroMode::HighPerformance,
        };

        assert_eq!(config.accel_ctrl1_reg(), 0x0A);
        assert_eq!(config.gyro_ctrl2_reg(), 0x0A);
    }

    #[test]
    fn range_encodings_match_datasheet_values() {
        assert_eq!(AccelRange::G4.reg_value(), 0x00);
        assert_eq!(AccelRange::G8.reg_value(), 0x01);
        assert_eq!(AccelRange::G16.reg_value(), 0x02);
        assert_eq!(AccelRange::G32.reg_value(), 0x03);

        assert_eq!(GyroRange::Dps125.reg_value(), 0x00);
        assert_eq!(GyroRange::Dps250.reg_value(), 0x01);
        assert_eq!(GyroRange::Dps500.reg_value(), 0x02);
        assert_eq!(GyroRange::Dps1000.reg_value(), 0x03);
        assert_eq!(GyroRange::Dps2000.reg_value(), 0x04);
        assert_eq!(GyroRange::Dps4000.reg_value(), 0x0C);
    }

    #[test]
    fn accel_range_write_includes_required_ctrl8_bit() {
        assert_eq!(
            super::reg::CTRL8_REQUIRED_ONE | AccelRange::G4.reg_value(),
            0x04
        );
        assert_eq!(
            super::reg::CTRL8_REQUIRED_ONE | AccelRange::G8.reg_value(),
            0x05
        );
        assert_eq!(
            super::reg::CTRL8_REQUIRED_ONE | AccelRange::G16.reg_value(),
            0x06
        );
        assert_eq!(
            super::reg::CTRL8_REQUIRED_ONE | AccelRange::G32.reg_value(),
            0x07
        );
    }

    #[test]
    fn apply_offsets_subtracts_calibration_bias() {
        let calibration = Lsm6dsv32xCalibration {
            accel_offset: [10, -20, 30],
            gyro_offset: [1, 2, 3],
        };

        assert_eq!(
            apply_offsets([100, 200, 300], calibration.accel_offset),
            [90, 220, 270]
        );
        assert_eq!(
            apply_offsets([10, 20, 30], calibration.gyro_offset),
            [9, 18, 27]
        );
    }
}
