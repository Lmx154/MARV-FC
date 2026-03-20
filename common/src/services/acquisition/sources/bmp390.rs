//! BMP390-backed barometer source adapter for the generic producer service.

use embedded_hal_async::i2c::I2c;

use crate::drivers::bmp390::{Bmp390, Error as Bmp390Error};
use crate::interfaces::sensors::BarometerSource;
use crate::messages::sensor::BarometerSample;

pub struct Bmp390BarometerSource<I2C>
where
    I2C: I2c,
{
    driver: Bmp390<I2C>,
}

impl<I2C> Bmp390BarometerSource<I2C>
where
    I2C: I2c,
{
    pub fn new(driver: Bmp390<I2C>) -> Self {
        Self { driver }
    }

    pub fn driver_mut(&mut self) -> &mut Bmp390<I2C> {
        &mut self.driver
    }

    pub fn into_driver(self) -> Bmp390<I2C> {
        self.driver
    }
}

impl<I2C> BarometerSource for Bmp390BarometerSource<I2C>
where
    I2C: I2c,
{
    type Error = Bmp390Error;

    fn read_barometer_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<BarometerSample, Self::Error>> + '_ {
        async move {
            let (temperature_c_x100, pressure_pa) = self.driver.try_read_compensated().await?;

            Ok(BarometerSample {
                pressure_pa: pressure_pa as f32,
                temperature_c: temperature_c_x100 as f32 / 100.0,
            })
        }
    }
}
