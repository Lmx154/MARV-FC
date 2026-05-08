//! Generic MPU6050 IMU Driver (Accelerometer + Gyroscope)
//!
//! Async, MCU-agnostic driver using embedded-hal-async I2C traits.

#![allow(dead_code)]

use defmt::{Format, info};
use embedded_hal_async::i2c::I2c;

use crate::utils::delay::DelayMs;

pub const MPU6050_ADDR_AD0_LOW: u8 = 0x68;
pub const MPU6050_ADDR_AD0_HIGH: u8 = 0x69;
pub const MPU6050_CHIP_ID: u8 = 0x68;

const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;
const REG_SIGNAL_PATH_RESET: u8 = 0x68;
const REG_WHO_AM_I: u8 = 0x75;

const PWR_DEVICE_RESET: u8 = 0x80;
const PWR_CLKSEL_PLL_XGYRO: u8 = 0x01;
const SIGNAL_PATH_RESET_ALL: u8 = 0x07;

const RESET_DELAY_MS: u32 = 100;
const WAKE_DELAY_MS: u32 = 10;

#[derive(Debug, Format)]
pub enum Error {
    I2c,
    ChipId(u8),
}

#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Mpu6050Calibration {
    pub accel_offset: [i16; 3],
    pub gyro_offset: [i16; 3],
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelRange {
    G2,
    G4,
    G8,
    G16,
}

impl AccelRange {
    pub const fn reg_value(self) -> u8 {
        match self {
            Self::G2 => 0x00,
            Self::G4 => 0x01,
            Self::G8 => 0x02,
            Self::G16 => 0x03,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroRange {
    Dps250,
    Dps500,
    Dps1000,
    Dps2000,
}

impl GyroRange {
    pub const fn reg_value(self) -> u8 {
        match self {
            Self::Dps250 => 0x00,
            Self::Dps500 => 0x01,
            Self::Dps1000 => 0x02,
            Self::Dps2000 => 0x03,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum DigitalLowPassFilter {
    Hz260,
    Hz184,
    Hz94,
    Hz44,
    Hz21,
    Hz10,
    Hz5,
}

impl DigitalLowPassFilter {
    pub const fn reg_value(self) -> u8 {
        match self {
            Self::Hz260 => 0x00,
            Self::Hz184 => 0x01,
            Self::Hz94 => 0x02,
            Self::Hz44 => 0x03,
            Self::Hz21 => 0x04,
            Self::Hz10 => 0x05,
            Self::Hz5 => 0x06,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct Mpu6050Config {
    pub accel_range: AccelRange,
    pub gyro_range: GyroRange,
    pub dlpf: DigitalLowPassFilter,
    pub sample_rate_divider: u8,
}

impl Default for Mpu6050Config {
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G16,
            gyro_range: GyroRange::Dps2000,
            dlpf: DigitalLowPassFilter::Hz184,
            sample_rate_divider: 0,
        }
    }
}

/// Raw accelerometer + temperature + gyro frame
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Mpu6050Raw {
    pub accel: [i16; 3],
    pub temperature_raw: i16,
    pub gyro: [i16; 3],
}

impl Mpu6050Raw {
    pub fn temperature_c(self) -> f32 {
        self.temperature_raw as f32 / 340.0 + 36.53
    }
}

pub struct Mpu6050<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
    address: u8,
    calibration: Mpu6050Calibration,
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            calibration: Mpu6050Calibration::default(),
        }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    pub fn calibration(&self) -> Mpu6050Calibration {
        self.calibration
    }

    pub fn set_calibration(&mut self, calibration: Mpu6050Calibration) {
        self.calibration = calibration;
    }

    pub fn clear_calibration(&mut self) {
        self.calibration = Mpu6050Calibration::default();
    }

    pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.init_with_config(delay, Mpu6050Config::default()).await
    }

    pub async fn init_with_config<D: DelayMs>(
        &mut self,
        delay: &mut D,
        config: Mpu6050Config,
    ) -> Result<(), Error> {
        info!("MPU6050: starting initialization");
        self.reset(delay).await?;

        let chip_id = self.read_chip_id().await?;
        if !chip_id_matches(chip_id) {
            return Err(Error::ChipId(chip_id));
        }

        self.write_reg(REG_PWR_MGMT_1, PWR_CLKSEL_PLL_XGYRO).await?;
        delay.delay_ms(WAKE_DELAY_MS).await;
        self.write_reg(REG_PWR_MGMT_2, 0x00).await?;
        self.write_reg(REG_SMPLRT_DIV, config.sample_rate_divider)
            .await?;
        self.write_reg(REG_CONFIG, config.dlpf.reg_value()).await?;
        self.write_reg(REG_GYRO_CONFIG, config.gyro_range.reg_value() << 3)
            .await?;
        self.write_reg(REG_ACCEL_CONFIG, config.accel_range.reg_value() << 3)
            .await?;

        info!("MPU6050: initialization complete");
        Ok(())
    }

    pub async fn reset<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.write_reg(REG_PWR_MGMT_1, PWR_DEVICE_RESET).await?;
        delay.delay_ms(RESET_DELAY_MS).await;
        self.write_reg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_ALL)
            .await?;
        delay.delay_ms(WAKE_DELAY_MS).await;
        Ok(())
    }

    pub async fn read_chip_id(&mut self) -> Result<u8, Error> {
        self.read_reg(REG_WHO_AM_I).await
    }

    pub async fn read_raw(&mut self) -> Result<Mpu6050Raw, Error> {
        let mut buf = [0u8; 14];
        self.read_many(REG_ACCEL_XOUT_H, &mut buf).await?;

        let mut raw = Mpu6050Raw {
            accel: [
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
            ],
            temperature_raw: i16::from_be_bytes([buf[6], buf[7]]),
            gyro: [
                i16::from_be_bytes([buf[8], buf[9]]),
                i16::from_be_bytes([buf[10], buf[11]]),
                i16::from_be_bytes([buf[12], buf[13]]),
            ],
        };

        for axis in 0..3 {
            raw.accel[axis] = raw.accel[axis].saturating_sub(self.calibration.accel_offset[axis]);
            raw.gyro[axis] = raw.gyro[axis].saturating_sub(self.calibration.gyro_offset[axis]);
        }

        Ok(raw)
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c
            .write(self.address, &[reg, val])
            .await
            .map_err(|_| Error::I2c)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .await
            .map_err(|_| Error::I2c)?;
        Ok(buf[0])
    }

    async fn read_many(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .await
            .map_err(|_| Error::I2c)
    }
}

pub async fn read_mpu6050_chip_id<I2C>(i2c: &mut I2C, address: u8) -> Result<u8, I2C::Error>
where
    I2C: I2c,
{
    let mut buf = [0u8; 1];
    i2c.write_read(address, &[REG_WHO_AM_I], &mut buf).await?;
    Ok(buf[0])
}

const fn chip_id_matches(chip_id: u8) -> bool {
    chip_id == MPU6050_CHIP_ID || chip_id == MPU6050_ADDR_AD0_HIGH
}

#[cfg(test)]
mod tests {
    use super::{AccelRange, DigitalLowPassFilter, GyroRange, Mpu6050Config, Mpu6050Raw};

    #[test]
    fn config_encodes_expected_register_fields() {
        let config = Mpu6050Config {
            accel_range: AccelRange::G8,
            gyro_range: GyroRange::Dps500,
            dlpf: DigitalLowPassFilter::Hz44,
            sample_rate_divider: 7,
        };

        assert_eq!(config.accel_range.reg_value() << 3, 0x10);
        assert_eq!(config.gyro_range.reg_value() << 3, 0x08);
        assert_eq!(config.dlpf.reg_value(), 0x03);
        assert_eq!(config.sample_rate_divider, 7);
    }

    #[test]
    fn temperature_conversion_matches_datasheet_formula() {
        let raw = Mpu6050Raw {
            temperature_raw: 0,
            ..Default::default()
        };

        assert!((raw.temperature_c() - 36.53).abs() < 0.01);
    }
}
