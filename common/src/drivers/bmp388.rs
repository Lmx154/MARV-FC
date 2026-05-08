//! Explicit BMP388 portable driver surface.
//!
//! The underlying implementation is shared with the legacy BMP3x/BMP390 path,
//! but payload-controller code should depend on this sensor-specific module.

use embedded_hal_async::i2c::I2c;

pub use crate::drivers::bmp390::{
    BMP3X_ADDR_SDO_HIGH, BMP3X_ADDR_SDO_LOW, BMP388_ADDR_SDO_HIGH, BMP388_ADDR_SDO_LOW,
    CalibData as Bmp388Calibration, Error as Bmp388Error,
};

pub const BMP388_REG_CHIP_ID: u8 = 0x00;
pub const BMP388_CHIP_ID: u8 = 0x50;

pub type Bmp388<I2C> = crate::drivers::bmp390::Bmp390<I2C>;

pub async fn read_bmp388_chip_id<I2C>(i2c: &mut I2C, address: u8) -> Result<u8, I2C::Error>
where
    I2C: I2c,
{
    let mut buf = [0u8; 1];
    i2c.write_read(address, &[BMP388_REG_CHIP_ID], &mut buf)
        .await?;
    Ok(buf[0])
}
