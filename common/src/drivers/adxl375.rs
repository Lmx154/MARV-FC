//! Generic ADXL375 High-g Accelerometer Driver (I2C)
//!
//! Async, MCU-agnostic driver using embedded-hal 1.0 + embedded-hal-async 1.0.
//! - I2C: embedded_hal_async::i2c::I2c
//! - Optional: INT pin via embedded_hal::digital::InputPin
//! - Delays: provided by common::utils::delay::DelayMs (your trait)

#![allow(dead_code)]

use defmt::{debug, info, warn, Format};
use embedded_hal::digital::InputPin;
use embedded_hal_async::i2c::I2c;

use crate::utils::delay::DelayMs;

/// ADXL375 register map (subset, I2C)
mod regs {
    pub const DEVID: u8 = 0x00;        // expect 0xE5
    pub const THRESH_SHOCK: u8 = 0x1D;
    pub const DUR: u8 = 0x21;
    pub const BW_RATE: u8 = 0x2C;
    pub const POWER_CTL: u8 = 0x2D;
    pub const INT_ENABLE: u8 = 0x2E;
    pub const INT_MAP: u8 = 0x2F;
    pub const INT_SOURCE: u8 = 0x30;
    pub const DATA_FORMAT: u8 = 0x31;
    pub const DATAX0: u8 = 0x32;       // 6-byte burst: X0,X1,Y0,Y1,Z0,Z1
    pub const FIFO_CTL: u8 = 0x38;
    pub const FIFO_STATUS: u8 = 0x39;

    // Bit fields
    pub const POWER_CTL_MEASURE: u8 = 1 << 3; // Measure bit
    // Add other bits as needed (e.g. link, auto_sleep, etc.)
}

/// ADXL375 driver errors
#[derive(Debug)]
pub enum Error<I2cE> {
    I2c(I2cE),
    IntPin,
    ChipId(u8),
    /// Generic error / configuration invalid
    Config,
}

/// Raw ADXL375 accelerometer frame
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct Adxl375Raw {
    pub accel: [i16; 3],
}

/// Generic ADXL375 driver using async I2C bus and optional INT pin
pub struct Adxl375<I2C, INT>
where
    I2C: I2c,
    INT: InputPin,
{
    i2c: I2C,
    addr: u8,
    int_pin: Option<INT>,
    /// scale factor in mg/LSB (~49 mg/LSB for ADXL375)
    scale_mg_per_lsb: f32,
}

impl<I2C, INT> Adxl375<I2C, INT>
where
    I2C: I2c,
    INT: InputPin,
{
    /// Create a new ADXL375 driver instance
    pub fn new(i2c: I2C, addr: u8, int_pin: Option<INT>) -> Self {
        Self {
            i2c,
            addr,
            int_pin,
            scale_mg_per_lsb: 49.0, // typical sensitivity ~49 mg/LSB
        }
    }

    /// Mutable access to underlying I2C
    pub fn i2c_mut(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    /// Initialize the ADXL375 with a basic power-up configuration.
    ///
    /// - Verifies chip ID
    /// - Puts device into standby
    /// - Configures BW_RATE, DATA_FORMAT, FIFO, interrupts
    /// - Enables measurement mode
    pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        info!("ADXL375: Starting initialization sequence");

        delay.delay_ms(10).await;

        // 1) Check device ID
        let devid = self.read_reg(regs::DEVID).await?;
        if devid != 0xE5 {
            warn!(
                "ADXL375: Chip ID mismatch: got 0x{:02X}, expected 0xE5",
                devid
            );
            return Err(Error::ChipId(devid));
        }
        info!("ADXL375: Device ID OK (0xE5)");

        // 2) Put device into standby (clear MEASURE bit)
        let mut pctl = self.read_reg(regs::POWER_CTL).await?;
        pctl &= !regs::POWER_CTL_MEASURE;
        self.write_reg(regs::POWER_CTL, pctl).await?;
        delay.delay_ms(5).await;

        // 3) Configure bandwidth / data rate (example value)
        //
        // You can make this configurable later.
        // For now, choose a typical ODR setting code, e.g. 0x0C (400 Hz) or 0x0F (3200 Hz).
        let bw_rate = 0x0C;
        self.write_reg(regs::BW_RATE, bw_rate).await?;

        // 4) Configure data format.
        //
        // Range is fixed at ±200 g, so we mostly care about FULL_RES / justify / interrupt polarity.
        // Simple default: right-justified, interrupts non-inverted.
        let data_format = 0x00;
        self.write_reg(regs::DATA_FORMAT, data_format).await?;

        // 5) Configure FIFO (optional, default: bypass)
        let fifo_ctl = 0x00;
        self.write_reg(regs::FIFO_CTL, fifo_ctl).await?;

        // 6) Configure interrupts (optional basic setup)
        //
        // For now, disable all interrupts; you can add API methods to enable them as needed.
        let int_enable = 0x00;
        self.write_reg(regs::INT_ENABLE, int_enable).await?;

        // Map all enabled interrupts to one pin if you want (e.g., INT1)
        let int_map = 0x00;
        self.write_reg(regs::INT_MAP, int_map).await?;

        // 7) Enable measurement (set MEASURE bit)
        let mut pctl = self.read_reg(regs::POWER_CTL).await?;
        pctl |= regs::POWER_CTL_MEASURE;
        self.write_reg(regs::POWER_CTL, pctl).await?;

        delay.delay_ms(10).await;
        info!("ADXL375: Initialization complete");
        Ok(())
    }

    /// Read a single register
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Write a single register
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I2C::Error>> {
        let buf = [reg, val];
        self.i2c
            .write(self.addr, &buf)
            .await
            .map_err(Error::I2c)
    }

    /// Read multiple bytes starting at a register (e.g., for burst reads)
    async fn read_multi(&mut self, start_reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.addr, &[start_reg], buf)
            .await
            .map_err(Error::I2c)
    }

    /// Read raw acceleration (X, Y, Z) in i16 counts
    pub async fn read_accel_raw(&mut self) -> Result<[i16; 3], Error<I2C::Error>> {
        let mut raw_buf = [0u8; 6];
        self.read_multi(regs::DATAX0, &mut raw_buf).await?;

        let x = i16::from_le_bytes([raw_buf[0], raw_buf[1]]);
        let y = i16::from_le_bytes([raw_buf[2], raw_buf[3]]);
        let z = i16::from_le_bytes([raw_buf[4], raw_buf[5]]);

        Ok([x, y, z])
    }

    /// Read acceleration in g units (f32)
    pub async fn read_accel_g(&mut self) -> Result<[f32; 3], Error<I2C::Error>> {
        let raw = self.read_accel_raw().await?;
        let s = self.scale_mg_per_lsb / 1000.0; // mg -> g

        Ok([
            raw[0] as f32 * s,
            raw[1] as f32 * s,
            raw[2] as f32 * s,
        ])
    }

    /// Optional: poll interrupt source register, returning raw flags.
    pub async fn read_int_source(&mut self) -> Result<u8, Error<I2C::Error>> {
        let src = self.read_reg(regs::INT_SOURCE).await?;
        debug!("ADXL375: INT_SOURCE = 0x{:02X}", src);
        Ok(src)
    }

    /// Optional: configure basic shock detection.
    ///
    /// - `thresh_g`: threshold in g
    /// - `dur_ms`: max duration in milliseconds
    ///
    /// Note: the actual LSB size for THRESH_SHOCK/DUR should be confirmed with the datasheet.
    pub async fn config_shock_detection(
        &mut self,
        thresh_g: f32,
        dur_ms: f32,
    ) -> Result<(), Error<I2C::Error>> {
        // Rough example conversions (replace with exact constants from datasheet).
        //
        // Assume ~780 mg/LSB for THRESH_SHOCK and 625 µs/LSB for DUR.
        let thresh_mg = thresh_g * 1000.0;
        let mut t = thresh_mg / 780.0 + 0.5; // simple "round"
        if t < 0.0 {
            t = 0.0;
        }
        if t > 255.0 {
            t = 255.0;
        }
        let thresh_lsb = t as u8;

        let dur_us = dur_ms * 1000.0;
        let mut d = dur_us / 625.0 + 0.5; // simple "round"
        if d < 0.0 {
            d = 0.0;
        }
        if d > 255.0 {
            d = 255.0;
        }
        let dur_lsb = d as u8;

        self.write_reg(regs::THRESH_SHOCK, thresh_lsb).await?;
        self.write_reg(regs::DUR, dur_lsb).await?;

        // Here you’d also enable the shock interrupt in INT_ENABLE and map it.
        // For now, just leave that to higher-level config or add more helpers.

        Ok(())
    }

    /// Quick check if INT pin is asserted (if provided).
    ///
    /// Note: takes &mut self because embedded-hal's InputPin requires &mut self.
    pub fn int_pin_is_high(&mut self) -> Result<bool, Error<I2C::Error>> {
        if let Some(ref mut pin) = self.int_pin {
            pin.is_high().map_err(|_| Error::IntPin)
        } else {
            Ok(false)
        }
    }

    /// Put device into standby (clear MEASURE)
    pub async fn set_standby(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut pctl = self.read_reg(regs::POWER_CTL).await?;
        pctl &= !regs::POWER_CTL_MEASURE;
        self.write_reg(regs::POWER_CTL, pctl).await
    }

    /// Put device into measurement mode (set MEASURE)
    pub async fn set_measure(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut pctl = self.read_reg(regs::POWER_CTL).await?;
        pctl |= regs::POWER_CTL_MEASURE;
        self.write_reg(regs::POWER_CTL, pctl).await
    }
}
