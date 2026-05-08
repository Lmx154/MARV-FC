//! BMP581-backed barometer source adapter for the generic producer service.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::drivers::bmp581::{Bmp581, Error as Bmp581Error};
use crate::interfaces::sensors::BarometerSource;
use crate::messages::sensor::BarometerSample;

pub struct Bmp581BarometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    driver: Bmp581<I2C, D>,
}

impl<I2C, D> Bmp581BarometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    pub fn new(driver: Bmp581<I2C, D>) -> Self {
        Self { driver }
    }

    pub fn driver_mut(&mut self) -> &mut Bmp581<I2C, D> {
        &mut self.driver
    }

    pub fn into_driver(self) -> Bmp581<I2C, D> {
        self.driver
    }
}

impl<I2C, D> BarometerSource for Bmp581BarometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    type Error = Bmp581Error<I2C::Error>;

    fn read_barometer_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<BarometerSample, Self::Error>> + '_ {
        async move {
            let (temperature_c_x100, pressure_pa) = self.driver.read_compensated().await?;

            Ok(BarometerSample {
                pressure_pa: pressure_pa as f32,
                temperature_c: temperature_c_x100 as f32 / 100.0,
            })
        }
    }
}
