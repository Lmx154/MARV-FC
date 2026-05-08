//! BMM350-backed magnetometer source adapter for the generic producer service.

use embedded_hal_async::i2c::I2c;

use crate::drivers::bmm350::{Bmm350, Error as Bmm350Error};
use crate::interfaces::sensors::MagnetometerSource;
use crate::messages::sensor::MagnetometerSample;
use crate::utils::delay::DelayMs;

pub struct Bmm350MagnetometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayMs,
{
    driver: Bmm350<I2C>,
    delay: D,
}

impl<I2C, D> Bmm350MagnetometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayMs,
{
    pub fn new(driver: Bmm350<I2C>, delay: D) -> Self {
        Self { driver, delay }
    }

    pub fn driver_mut(&mut self) -> &mut Bmm350<I2C> {
        &mut self.driver
    }

    pub fn delay_mut(&mut self) -> &mut D {
        &mut self.delay
    }

    pub fn into_parts(self) -> (Bmm350<I2C>, D) {
        (self.driver, self.delay)
    }
}

impl<I2C, D> MagnetometerSource for Bmm350MagnetometerSource<I2C, D>
where
    I2C: I2c,
    D: DelayMs,
{
    type Error = Bmm350Error;

    fn read_magnetometer_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<MagnetometerSample, Self::Error>> + '_ {
        async move {
            let (x_ut, y_ut, z_ut) = self.driver.read_ut(&mut self.delay).await?;

            Ok(MagnetometerSample {
                field_ut: [x_ut, y_ut, z_ut],
            })
        }
    }
}
