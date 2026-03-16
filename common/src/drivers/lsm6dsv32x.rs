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
}

impl<SPI> Lsm6dsv32x<SPI>
where
    SPI: SpiDevice<u8>,
{
    /// Create a new LSM6DSV32X driver instance
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Initialize the LSM6DSV32X sensor with a simple power-up sequence
    pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
        info!("LSM6DSV32X: Starting initialization sequence");

        // Soft reset
        self.write_reg(reg::CTRL3, reg::CTRL3_SW_RESET).await?;
        delay.delay_ms(5).await;

        let mut reset_cleared = false;
        for _ in 0..10 {
            let ctrl3 = self.read_reg(reg::CTRL3).await?;
            if (ctrl3 & reg::CTRL3_SW_RESET) == 0 {
                reset_cleared = true;
                break;
            }
            delay.delay_ms(2).await;
        }
        if !reset_cleared {
            warn!("LSM6DSV32X: Reset timeout");
            return Err(Error::ResetTimeout);
        }

        // Check chip ID
        let who = self.read_reg(reg::WHO_AM_I).await?;
        debug!("LSM6DSV32X: WHO_AM_I=0x{:02X}", who);
        if who != reg::WHO_AM_I_VAL {
            warn!(
                "LSM6DSV32X: ID mismatch 0x{:02X} (expected 0x{:02X})",
                who,
                reg::WHO_AM_I_VAL
            );
            return Err(Error::ChipId(who));
        }

        // Enable register auto-increment and block data update
        self.write_reg(reg::CTRL3, reg::CTRL3_BDU | reg::CTRL3_IF_INC)
            .await?;

        // Configure full-scale ranges
        self.write_reg(reg::CTRL6, reg::FS_G_2000DPS).await?;
        self.write_reg(reg::CTRL8, reg::FS_XL_16G).await?;

        // Configure ODR and performance modes
        let ctrl1 = (reg::ODR_1920 & 0x0F) | ((reg::OP_MODE_XL_HIGH_PERF & 0x07) << 4);
        let ctrl2 = (reg::ODR_1920 & 0x0F) | ((reg::OP_MODE_G_HIGH_PERF & 0x07) << 4);
        self.write_reg(reg::CTRL1, ctrl1).await?;
        self.write_reg(reg::CTRL2, ctrl2).await?;

        info!("LSM6DSV32X: Initialization complete (accel 16g, gyro 2000dps, ODR 1920Hz)");
        Ok(())
    }

    /// Read accelerometer data (X, Y, Z) in raw i16 counts
    pub async fn read_accel(&mut self) -> Result<[i16; 3], Error> {
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

    /// Read gyroscope data (X, Y, Z) in raw i16 counts
    pub async fn read_gyro(&mut self) -> Result<[i16; 3], Error> {
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

    /// Combined read of both accelerometer and gyroscope
    pub async fn read_raw(&mut self) -> Result<Lsm6dsv32xRaw, Error> {
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
