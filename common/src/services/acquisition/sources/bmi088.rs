//! BMI088-backed IMU source adapter for the generic producer service.

use core::f32::consts::PI;

use embedded_hal_async::spi::SpiDevice;

use crate::drivers::bmi088::{AccelRange, Bmi088, Error as Bmi088Error, GyroRange};
use crate::interfaces::sensors::ImuSource;
use crate::messages::sensor::ImuSample;

const STANDARD_GRAVITY_MPS2: f32 = 9.80665;

pub struct Bmi088ImuSource<ACC, GYR>
where
    ACC: SpiDevice<u8>,
    GYR: SpiDevice<u8>,
{
    driver: Bmi088<ACC, GYR>,
    accel_mps2_per_lsb: f32,
    gyro_rad_s_per_lsb: f32,
}

impl<ACC, GYR> Bmi088ImuSource<ACC, GYR>
where
    ACC: SpiDevice<u8>,
    GYR: SpiDevice<u8>,
{
    pub fn new(driver: Bmi088<ACC, GYR>, accel_range: AccelRange, gyro_range: GyroRange) -> Self {
        let accel_mps2_per_lsb = STANDARD_GRAVITY_MPS2 / accel_lsb_per_g(accel_range);
        let gyro_rad_s_per_lsb = (PI / 180.0) / gyro_lsb_per_dps(gyro_range);

        Self {
            driver,
            accel_mps2_per_lsb,
            gyro_rad_s_per_lsb,
        }
    }

    pub fn driver_mut(&mut self) -> &mut Bmi088<ACC, GYR> {
        &mut self.driver
    }

    pub fn into_driver(self) -> Bmi088<ACC, GYR> {
        self.driver
    }
}

impl<ACC, GYR> ImuSource for Bmi088ImuSource<ACC, GYR>
where
    ACC: SpiDevice<u8>,
    GYR: SpiDevice<u8>,
{
    type Error = Bmi088Error;

    fn read_imu_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
        async move {
            let raw = self.driver.read_raw().await?;

            Ok(ImuSample {
                accel_mps2: [
                    raw.accel[0] as f32 * self.accel_mps2_per_lsb,
                    raw.accel[1] as f32 * self.accel_mps2_per_lsb,
                    raw.accel[2] as f32 * self.accel_mps2_per_lsb,
                ],
                gyro_rad_s: [
                    raw.gyro[0] as f32 * self.gyro_rad_s_per_lsb,
                    raw.gyro[1] as f32 * self.gyro_rad_s_per_lsb,
                    raw.gyro[2] as f32 * self.gyro_rad_s_per_lsb,
                ],
            })
        }
    }
}

const fn accel_lsb_per_g(range: AccelRange) -> f32 {
    match range {
        AccelRange::G3 => 10_920.0,
        AccelRange::G6 => 5_460.0,
        AccelRange::G12 => 2_730.0,
        AccelRange::G24 => 1_365.0,
    }
}

const fn gyro_lsb_per_dps(range: GyroRange) -> f32 {
    match range {
        GyroRange::Dps2000 => 16.4,
        GyroRange::Dps1000 => 32.8,
        GyroRange::Dps500 => 65.6,
        GyroRange::Dps250 => 131.2,
        GyroRange::Dps125 => 262.4,
    }
}

#[cfg(test)]
mod tests {
    use super::{accel_lsb_per_g, gyro_lsb_per_dps};
    use crate::drivers::bmi088::{AccelRange, GyroRange};

    #[test]
    fn accel_range_scaling_constants_match_expected_order() {
        assert!(accel_lsb_per_g(AccelRange::G3) > accel_lsb_per_g(AccelRange::G6));
        assert!(accel_lsb_per_g(AccelRange::G6) > accel_lsb_per_g(AccelRange::G12));
        assert!(accel_lsb_per_g(AccelRange::G12) > accel_lsb_per_g(AccelRange::G24));
    }

    #[test]
    fn gyro_range_scaling_constants_match_expected_order() {
        assert!(gyro_lsb_per_dps(GyroRange::Dps125) > gyro_lsb_per_dps(GyroRange::Dps250));
        assert!(gyro_lsb_per_dps(GyroRange::Dps250) > gyro_lsb_per_dps(GyroRange::Dps500));
        assert!(gyro_lsb_per_dps(GyroRange::Dps500) > gyro_lsb_per_dps(GyroRange::Dps1000));
        assert!(gyro_lsb_per_dps(GyroRange::Dps1000) > gyro_lsb_per_dps(GyroRange::Dps2000));
    }
}
