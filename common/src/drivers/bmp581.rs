//! Async BMP581 driver wrapper (BMP5x family) using bmp5 crate.
//!
//! Provides a small adapter to return temp/pressure in the same units as
//! the existing BMP390 path (temperature in celsius * 100, pressure in Pa).

use bmp5::i2c::{BMP5_ADDRESS, BMP5_ADDRESS_ALT, Bmp5, Error as Bmp5Error};
pub use bmp5::{Config as Bmp581Config, IIRFilter, OutputDataRate, Oversampling};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub const BMP581_ADDR_PRIMARY: u8 = BMP5_ADDRESS;
pub const BMP581_ADDR_ALT: u8 = BMP5_ADDRESS_ALT;
pub const BMP581_REG_CHIP_ID: u8 = 0x01;
pub const BMP581_CHIP_ID_PRIMARY: u8 = 0x50;
pub const BMP581_CHIP_ID_ALT: u8 = 0x51;

pub type Error<E> = Bmp5Error<E>;

pub struct Bmp581<I2C, D> {
    dev: Bmp5<I2C, D>,
}

impl<I2C, D, E> Bmp581<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D, address: u8, config: Bmp581Config) -> Self {
        Self {
            dev: Bmp5::new(i2c, delay, address, config),
        }
    }

    pub async fn init(&mut self) -> Result<(), Error<E>> {
        self.dev.init().await
    }

    pub async fn read_compensated(&mut self) -> Result<(i32, i32), Error<E>> {
        let sample = self.dev.measure().await?;
        let temp_c_x100 = round_f32(sample.temperature * 100.0);
        let press_pa = round_f32(sample.pressure);
        Ok((temp_c_x100, press_pa))
    }
}

pub async fn read_bmp581_chip_id<I2C>(i2c: &mut I2C, address: u8) -> Result<u8, I2C::Error>
where
    I2C: I2c,
{
    let mut buf = [0u8; 1];
    i2c.write_read(address, &[BMP581_REG_CHIP_ID], &mut buf)
        .await?;
    Ok(buf[0])
}

fn round_f32(value: f32) -> i32 {
    if value.is_finite() {
        (value + if value >= 0.0 { 0.5 } else { -0.5 }) as i32
    } else {
        0
    }
}
