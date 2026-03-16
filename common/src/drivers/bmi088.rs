//! Generic BMI088 IMU Driver (Accelerometer + Gyroscope)
//!
//! Async, MCU-agnostic driver using embedded-hal 1.0 traits.
//! - SPI: embedded_hal_async::spi::SpiDevice<u8>
//! - Delays: provided by a simple async trait defined in common::utils::delay

#![allow(dead_code)]

use defmt::{Format, debug, info, warn};
use embedded_hal_async::spi::SpiDevice;

use crate::utils::delay::DelayMs;

const BMI088_ACC_CHIP_ID: u8 = 0x1E;
const BMI088_GYR_CHIP_ID: u8 = 0x0F;

/// BMI088 Accelerometer register map (subset)
mod acc {
    pub const CHIP_ID: u8 = 0x00; // Expect 0x1E
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const ACC_X_L: u8 = 0x12; // Burst sequence: XL, XH, YL, YH, ZL, ZH
    pub const SOFTRESET: u8 = 0x7E; // Write 0xB6 to reset accel core
    pub const PWR_CONF: u8 = 0x7C; // Set to 0x00 for active (disable suspend)
    pub const PWR_CTRL: u8 = 0x7D; // Bit 2 (0x04) enables accelerometer
    pub const ACC_RANGE: u8 = 0x41; // 0x01 => ±6g
    pub const ACC_CONF: u8 = 0x40; // Bandwidth / ODR config (0xAC for 1600 Hz)
}

/// BMI088 Gyroscope register map (subset)
mod gyr {
    pub const CHIP_ID: u8 = 0x00; // Expect 0x0F
    pub const RATE_X_L: u8 = 0x02; // Burst data base (6 bytes)
    pub const RANGE: u8 = 0x0F;
    pub const BW: u8 = 0x10; // Bandwidth / ODR (0x00 for 2000 Hz)
    pub const LPM1: u8 = 0x11;
    pub const SOFTRESET: u8 = 0x14; // Write 0xB6 to reset gyro core
}

#[derive(Debug, Format)]
pub enum Error {
    Spi,
    ChipIdAccel(u8),
    ChipIdGyro(u8),
    InvalidAccelRange(u8),
    InvalidGyroRange(u8),
    InvalidAccelConf(u8),
    InvalidGyroBw(u8),
    InvalidCalibrationSamples,
    Unknown,
}

#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Bmi088Calibration {
    pub accel_offset: [i16; 3],
    pub gyro_offset: [i16; 3],
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelRange {
    G3,
    G6,
    G12,
    G24,
}

impl AccelRange {
    const fn reg_value(self) -> u8 {
        match self {
            Self::G3 => 0x00,
            Self::G6 => 0x01,
            Self::G12 => 0x02,
            Self::G24 => 0x03,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroRange {
    Dps2000,
    Dps1000,
    Dps500,
    Dps250,
    Dps125,
}

impl GyroRange {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Dps2000 => 0x00,
            Self::Dps1000 => 0x01,
            Self::Dps500 => 0x02,
            Self::Dps250 => 0x03,
            Self::Dps125 => 0x04,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelOdr {
    Hz12,
    Hz25,
    Hz50,
    Hz100,
    Hz200,
    Hz400,
    Hz800,
    Hz1600,
}

impl AccelOdr {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Hz12 => 0x05,
            Self::Hz25 => 0x06,
            Self::Hz50 => 0x07,
            Self::Hz100 => 0x08,
            Self::Hz200 => 0x09,
            Self::Hz400 => 0x0A,
            Self::Hz800 => 0x0B,
            Self::Hz1600 => 0x0C,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum AccelBandwidth {
    Osr4,
    Osr2,
    Normal,
}

impl AccelBandwidth {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Osr4 => 0x08,
            Self::Osr2 => 0x09,
            Self::Normal => 0x0A,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum GyroBwOdr {
    Hz2000Bw532,
    Hz2000Bw230,
    Hz1000Bw116,
    Hz400Bw47,
    Hz200Bw23,
    Hz100Bw12,
    Hz200Bw64,
    Hz100Bw32,
}

impl GyroBwOdr {
    const fn reg_value(self) -> u8 {
        match self {
            Self::Hz2000Bw532 => 0x00,
            Self::Hz2000Bw230 => 0x01,
            Self::Hz1000Bw116 => 0x02,
            Self::Hz400Bw47 => 0x03,
            Self::Hz200Bw23 => 0x04,
            Self::Hz100Bw12 => 0x05,
            Self::Hz200Bw64 => 0x06,
            Self::Hz100Bw32 => 0x07,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct Bmi088Config {
    pub accel_range: AccelRange,
    pub accel_odr: AccelOdr,
    pub accel_bandwidth: AccelBandwidth,
    pub gyro_range: GyroRange,
    pub gyro_bw_odr: GyroBwOdr,
}

impl Default for Bmi088Config {
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G6,
            accel_odr: AccelOdr::Hz1600,
            accel_bandwidth: AccelBandwidth::Normal,
            gyro_range: GyroRange::Dps2000,
            gyro_bw_odr: GyroBwOdr::Hz2000Bw532,
        }
    }
}

impl Bmi088Config {
    pub const fn accel_conf_reg(self) -> u8 {
        (self.accel_bandwidth.reg_value() << 4) | self.accel_odr.reg_value()
    }
}

/// Raw accelerometer + gyro frame
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Bmi088Raw {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

/// Generic BMI088 driver using async SPI devices for accel + gyro
pub struct Bmi088<ACC, GYR>
where
    ACC: SpiDevice<u8>,
    GYR: SpiDevice<u8>,
{
    acc: ACC,
    gyr: GYR,
    calibration: Bmi088Calibration,
}

impl<ACC, GYR> Bmi088<ACC, GYR>
where
    ACC: SpiDevice<u8>,
    GYR: SpiDevice<u8>,
{
    /// Create a new BMI088 driver instance
    pub fn new(acc: ACC, gyr: GYR) -> Self {
        Self {
            acc,
            gyr,
            calibration: Bmi088Calibration::default(),
        }
    }

    /// Initialize the BMI088 sensor with proper power-up sequence
    pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.init_with_config(delay, Bmi088Config::default()).await
    }

    /// Initialize and apply an explicit runtime configuration.
    pub async fn init_with_config<D: DelayMs>(
        &mut self,
        delay: &mut D,
        config: Bmi088Config,
    ) -> Result<(), Error> {
        info!("BMI088: Starting initialization sequence");

        self.soft_reset(delay).await?;
        self.verify_chip_ids(delay).await?;
        self.apply_config(config).await?;

        info!("BMI088: Initialization complete");
        Ok(())
    }

    /// Soft-reset both BMI088 cores and execute required bring-up delays.
    pub async fn soft_reset<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        // Step 1: Reset gyroscope first (datasheet recommendation)
        info!("BMI088: Resetting gyroscope");
        self.write_gyr(gyr::SOFTRESET, 0xB6).await?;
        delay.delay_ms(100).await;

        // Step 2: Reset accelerometer
        info!("BMI088: Resetting accelerometer");
        self.write_acc(acc::SOFTRESET, 0xB6).await?;
        delay.delay_ms(100).await;

        // Dummy read to switch back to SPI mode after reset
        let _ = self.read_acc(acc::CHIP_ID).await;
        delay.delay_ms(10).await;

        // Step 3: Critical BMI088 accelerometer power-up sequence
        info!("BMI088: Configuring power management");
        self.write_acc(acc::PWR_CONF, 0x00).await?; // Active mode (exit suspend)
        delay.delay_ms(5).await;
        self.write_acc(acc::PWR_CTRL, 0x04).await?; // Enable accelerometer
        delay.delay_ms(50).await;

        // Additional stability delay
        delay.delay_ms(50).await;

        Ok(())
    }

    /// Validate accelerometer and gyroscope chip IDs with retries.
    pub async fn verify_chip_ids<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        // Step 4: Verify chip IDs with multiple attempts
        let mut acc_id = 0u8;
        let mut gyr_id = 0u8;
        let mut success = false;

        for attempt in 0..10 {
            delay.delay_ms(20).await;

            if let Ok(id) = self.read_acc(acc::CHIP_ID).await {
                acc_id = id;
                if id == BMI088_ACC_CHIP_ID {
                    info!("BMI088: Accelerometer ID found on attempt {}", attempt);
                }
            }

            if let Ok(id) = self.read_gyr(gyr::CHIP_ID).await {
                gyr_id = id;
                if id == BMI088_GYR_CHIP_ID {
                    info!("BMI088: Gyroscope ID found on attempt {}", attempt);
                }
            }

            debug!(
                "BMI088: Attempt {} IDs acc=0x{:02X} gyr=0x{:02X}",
                attempt, acc_id, gyr_id
            );

            if acc_id == BMI088_ACC_CHIP_ID && gyr_id == BMI088_GYR_CHIP_ID {
                success = true;
                break;
            }
        }

        if !success {
            warn!(
                "BMI088: ID mismatch acc=0x{:02X} (expected 0x{:02X}) gyr=0x{:02X} (expected 0x{:02X})",
                acc_id, BMI088_ACC_CHIP_ID, gyr_id, BMI088_GYR_CHIP_ID
            );
            if acc_id != BMI088_ACC_CHIP_ID {
                return Err(Error::ChipIdAccel(acc_id));
            }
            if gyr_id != BMI088_GYR_CHIP_ID {
                return Err(Error::ChipIdGyro(gyr_id));
            }
        }

        Ok(())
    }

    /// Apply measurement range and ODR/bandwidth configuration.
    pub async fn apply_config(&mut self, config: Bmi088Config) -> Result<(), Error> {
        self.set_accel_range(config.accel_range).await?;
        self.set_accel_conf_raw(config.accel_conf_reg()).await?;
        self.set_gyro_range(config.gyro_range).await?;
        self.set_gyro_bw_odr(config.gyro_bw_odr).await
    }

    /// For SPI devices like BMI088, bus address selection is not used.
    pub const fn supports_address_selection() -> bool {
        false
    }

    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error> {
        self.write_acc(acc::ACC_RANGE, range.reg_value()).await
    }

    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error> {
        self.write_gyr(gyr::RANGE, range.reg_value()).await
    }

    pub async fn set_accel_odr(
        &mut self,
        odr: AccelOdr,
        bandwidth: AccelBandwidth,
    ) -> Result<(), Error> {
        let reg = (bandwidth.reg_value() << 4) | odr.reg_value();
        self.set_accel_conf_raw(reg).await
    }

    pub async fn set_gyro_bw_odr(&mut self, bw_odr: GyroBwOdr) -> Result<(), Error> {
        self.set_gyro_bw_raw(bw_odr.reg_value()).await
    }

    pub async fn set_accel_conf_raw(&mut self, reg_value: u8) -> Result<(), Error> {
        validate_accel_conf(reg_value)?;
        self.write_acc(acc::ACC_CONF, reg_value).await
    }

    pub async fn set_gyro_bw_raw(&mut self, reg_value: u8) -> Result<(), Error> {
        validate_gyro_bw(reg_value)?;
        self.write_gyr(gyr::BW, reg_value).await
    }

    pub fn calibration(&self) -> Bmi088Calibration {
        self.calibration
    }

    pub fn set_calibration(&mut self, calibration: Bmi088Calibration) {
        self.calibration = calibration;
    }

    pub fn clear_calibration(&mut self) {
        self.calibration = Bmi088Calibration::default();
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
        // BMI088 accelerometer requires dummy byte after register address.
        // Frame: [addr | 0x80, dummy, XL, XH, YL, YH, ZL, ZH].
        let mut tx_buf = [0u8; 8];
        let mut rx_buf = [0u8; 8];
        tx_buf[0] = acc::ACC_X_L | 0x80;

        self.acc
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;

        // Parse data starting from byte 2 (skip addr and dummy).
        let x = i16::from_le_bytes([rx_buf[2], rx_buf[3]]);
        let y = i16::from_le_bytes([rx_buf[4], rx_buf[5]]);
        let z = i16::from_le_bytes([rx_buf[6], rx_buf[7]]);

        Ok([x, y, z])
    }

    /// Read gyroscope data (X, Y, Z) in raw i16 counts.
    pub async fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error> {
        // Frame: [addr | 0x80, XL, XH, YL, YH, ZL, ZH].
        let mut tx_buf = [0u8; 7];
        let mut rx_buf = [0u8; 7];
        tx_buf[0] = gyr::RATE_X_L | 0x80;

        self.gyr
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;

        // Parse data starting from byte 1.
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

    /// Combined read of both accelerometer and gyroscope
    pub async fn read_raw(&mut self) -> Result<Bmi088Raw, Error> {
        let accel = self.read_accel().await?;
        let gyro = self.read_gyro().await?;
        Ok(Bmi088Raw { accel, gyro })
    }

    /// Read a single register from the accelerometer
    async fn read_acc(&mut self, reg: u8) -> Result<u8, Error> {
        // BMI088 accelerometer requires dummy byte
        let tx_buf = [reg | 0x80, 0x00, 0x00];
        let mut rx_buf = [0u8; 3];

        self.acc
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;
        Ok(rx_buf[2]) // Data is in the third byte
    }

    /// Write a single register to the accelerometer
    async fn write_acc(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let tx_buf = [reg & 0x7F, val];

        self.acc.write(&tx_buf).await.map_err(|_| Error::Spi)
    }

    /// Read a single register from the gyroscope
    async fn read_gyr(&mut self, reg: u8) -> Result<u8, Error> {
        let tx_buf = [reg | 0x80, 0x00];
        let mut rx_buf = [0u8; 2];

        self.gyr
            .transfer(&mut rx_buf, &tx_buf)
            .await
            .map_err(|_| Error::Spi)?;
        Ok(rx_buf[1])
    }

    /// Write a single register to the gyroscope
    async fn write_gyr(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let tx_buf = [reg & 0x7F, val];

        self.gyr.write(&tx_buf).await.map_err(|_| Error::Spi)
    }
}

fn validate_accel_conf(reg_value: u8) -> Result<(), Error> {
    let odr_bits = reg_value & 0x0F;
    if !(0x05..=0x0C).contains(&odr_bits) {
        return Err(Error::InvalidAccelConf(reg_value));
    }
    Ok(())
}

fn validate_gyro_bw(reg_value: u8) -> Result<(), Error> {
    if reg_value > 0x07 {
        return Err(Error::InvalidGyroBw(reg_value));
    }
    Ok(())
}

fn apply_offsets(sample: [i16; 3], offsets: [i16; 3]) -> [i16; 3] {
    [
        sample[0].saturating_sub(offsets[0]),
        sample[1].saturating_sub(offsets[1]),
        sample[2].saturating_sub(offsets[2]),
    ]
}
