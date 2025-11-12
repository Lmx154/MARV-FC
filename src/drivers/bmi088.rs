//! BMI088 IMU Driver (Accelerometer + Gyroscope) for Embassy
//!
//! Async driver for reading raw accelerometer and gyroscope data from the BMI088 sensor
//! using Embassy's async SPI interface with DMA support.
//!
//! The BMI088 exposes TWO separate SPI targets with distinct chip-selects:
//!   * Accelerometer (ACC) 8-bit registers, CHIP_ID = 0x1E
//!   * Gyroscope (GYR) 8-bit registers, CHIP_ID = 0x0F
//!
//! SPI mode: CPOL=0, CPHA=0 (Mode 0)
//! Read protocol: Set MSB=1 of register address for reads
//! Clock speed: Up to 10 MHz for data operations

#![allow(dead_code)]

use defmt::{debug, info, warn, Format};
use embassy_rp::gpio::Output;
use embassy_rp::spi::Spi;
use embassy_time::Timer;

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
    Bus,
    Spi,
    ChipIdAccel(u8),
    ChipIdGyro(u8),
    AccelNotReady,
    Unknown,
}

/// Raw accelerometer + gyro frame
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Bmi088Raw {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

/// BMI088 driver using Embassy async SPI
pub struct Bmi088<'d, T: embassy_rp::spi::Instance> {
    spi: Spi<'d, T, embassy_rp::spi::Async>,
    cs_accel: Output<'d>,
    cs_gyro: Output<'d>,
}

impl<'d, T: embassy_rp::spi::Instance> Bmi088<'d, T> {
    /// Create a new BMI088 driver instance
    pub fn new(
        spi: Spi<'d, T, embassy_rp::spi::Async>,
        cs_accel: Output<'d>,
        cs_gyro: Output<'d>,
    ) -> Self {
        Self {
            spi,
            cs_accel,
            cs_gyro,
        }
    }

    /// Initialize the BMI088 sensor with proper power-up sequence
    pub async fn init(&mut self) -> Result<(), Error> {
        info!("BMI088: Starting initialization sequence");

        // Ensure CS lines are high initially
        self.cs_accel.set_high();
        self.cs_gyro.set_high();
        Timer::after_millis(10).await;

        // Step 1: Reset gyroscope first (datasheet recommendation)
        info!("BMI088: Resetting gyroscope");
        self.write_gyr(gyr::SOFTRESET, 0xB6).await?;
        Timer::after_millis(100).await;

        // Step 2: Reset accelerometer
        info!("BMI088: Resetting accelerometer");
        self.write_acc(acc::SOFTRESET, 0xB6).await?;
        Timer::after_millis(100).await;

        // Dummy read to switch back to SPI mode after reset
        let _ = self.read_acc(acc::CHIP_ID).await;
        Timer::after_millis(10).await;

        // Step 3: Critical BMI088 accelerometer power-up sequence
        info!("BMI088: Configuring power management");
        self.write_acc(acc::PWR_CONF, 0x00).await?; // Active mode (exit suspend)
        Timer::after_millis(5).await;
        self.write_acc(acc::PWR_CTRL, 0x04).await?; // Enable accelerometer
        Timer::after_millis(50).await;

        // Additional stability delay
        Timer::after_millis(50).await;

        // Step 4: Verify chip IDs with multiple attempts
        let mut acc_id = 0u8;
        let mut gyr_id = 0u8;
        let mut success = false;

        for attempt in 0..10 {
            Timer::after_millis(20).await;

            if let Ok(id) = self.read_acc(acc::CHIP_ID).await {
                acc_id = id;
                if id == 0x1E {
                    info!("BMI088: Accelerometer ID found on attempt {}", attempt);
                }
            }

            if let Ok(id) = self.read_gyr(gyr::CHIP_ID).await {
                gyr_id = id;
                if id == 0x0F {
                    info!("BMI088: Gyroscope ID found on attempt {}", attempt);
                }
            }

            debug!(
                "BMI088: Attempt {} IDs acc=0x{:02X} gyr=0x{:02X}",
                attempt, acc_id, gyr_id
            );

            if acc_id == 0x1E && gyr_id == 0x0F {
                success = true;
                break;
            }
        }

        if !success {
            warn!(
                "BMI088: ID mismatch acc=0x{:02X} (expected 0x1E) gyr=0x{:02X} (expected 0x0F)",
                acc_id, gyr_id
            );
            if acc_id != 0x1E {
                return Err(Error::ChipIdAccel(acc_id));
            }
            if gyr_id != 0x0F {
                return Err(Error::ChipIdGyro(gyr_id));
            }
        }

        // Configure accel range & bandwidth for max ODR (1600 Hz)
        self.write_acc(acc::ACC_RANGE, 0x01).await?; // ±6g
        self.write_acc(acc::ACC_CONF, 0xAC).await?; // 1600 Hz ODR, normal mode

        // Configure gyro bandwidth to max ODR (2000 Hz)
        self.write_gyr(gyr::BW, 0x00).await?;
        Timer::after_millis(2).await;

        info!("BMI088: Initialization complete (accel 1600Hz, gyro 2000Hz)");
        Ok(())
    }

    /// Read accelerometer data (X, Y, Z) in raw i16 counts
    pub async fn read_accel(&mut self) -> Result<[i16; 3], Error> {
        // BMI088 accelerometer requires dummy byte after register address
        // Frame: [addr | 0x80, dummy, XL, XH, YL, YH, ZL, ZH]
        let mut tx_buf = [0u8; 8];
        let mut rx_buf = [0u8; 8];
        tx_buf[0] = acc::ACC_X_L | 0x80;

        self.cs_accel.set_low();
        let result = self.spi.transfer(&mut rx_buf, &tx_buf).await;
        self.cs_accel.set_high();

        result.map_err(|_| Error::Spi)?;

        // Parse data starting from byte 2 (skip addr and dummy)
        let x = i16::from_le_bytes([rx_buf[2], rx_buf[3]]);
        let y = i16::from_le_bytes([rx_buf[4], rx_buf[5]]);
        let z = i16::from_le_bytes([rx_buf[6], rx_buf[7]]);

        Ok([x, y, z])
    }

    /// Read gyroscope data (X, Y, Z) in raw i16 counts
    pub async fn read_gyro(&mut self) -> Result<[i16; 3], Error> {
        // Frame: [addr | 0x80, XL, XH, YL, YH, ZL, ZH]
        let mut tx_buf = [0u8; 7];
        let mut rx_buf = [0u8; 7];
        tx_buf[0] = gyr::RATE_X_L | 0x80;

        self.cs_gyro.set_low();
        let result = self.spi.transfer(&mut rx_buf, &tx_buf).await;
        self.cs_gyro.set_high();

        result.map_err(|_| Error::Spi)?;

        // Parse data starting from byte 1
        let x = i16::from_le_bytes([rx_buf[1], rx_buf[2]]);
        let y = i16::from_le_bytes([rx_buf[3], rx_buf[4]]);
        let z = i16::from_le_bytes([rx_buf[5], rx_buf[6]]);

        Ok([x, y, z])
    }

    /// Combined read of both accelerometer and gyroscope
    pub async fn read_raw(&mut self) -> Result<Bmi088Raw, Error> {
        let accel = self.read_accel().await?;
        let gyro = self.read_gyro().await.unwrap_or([0; 3]);
        Ok(Bmi088Raw { accel, gyro })
    }

    /// Read a single register from the accelerometer
    async fn read_acc(&mut self, reg: u8) -> Result<u8, Error> {
        // BMI088 accelerometer requires dummy byte
        let tx_buf = [reg | 0x80, 0x00, 0x00];
        let mut rx_buf = [0u8; 3];

        self.cs_accel.set_low();
        let result = self.spi.transfer(&mut rx_buf, &tx_buf).await;
        self.cs_accel.set_high();

        result.map_err(|_| Error::Spi)?;
        Ok(rx_buf[2]) // Data is in the third byte
    }

    /// Write a single register to the accelerometer
    async fn write_acc(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let tx_buf = [reg & 0x7F, val];

        self.cs_accel.set_low();
        let result = self.spi.write(&tx_buf).await;
        self.cs_accel.set_high();

        result.map_err(|_| Error::Spi)
    }

    /// Read a single register from the gyroscope
    async fn read_gyr(&mut self, reg: u8) -> Result<u8, Error> {
        let tx_buf = [reg | 0x80, 0x00];
        let mut rx_buf = [0u8; 2];

        self.cs_gyro.set_low();
        let result = self.spi.transfer(&mut rx_buf, &tx_buf).await;
        self.cs_gyro.set_high();

        result.map_err(|_| Error::Spi)?;
        Ok(rx_buf[1])
    }

    /// Write a single register to the gyroscope
    async fn write_gyr(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let tx_buf = [reg & 0x7F, val];

        self.cs_gyro.set_low();
        let result = self.spi.write(&tx_buf).await;
        self.cs_gyro.set_high();

        result.map_err(|_| Error::Spi)
    }
}