//! MPU6050-backed IMU source adapter for the generic producer service.

use core::f32::consts::PI;

use embedded_hal_async::i2c::I2c;

use crate::drivers::mpu6050::{AccelRange, Error as Mpu6050Error, GyroRange, Mpu6050};
use crate::interfaces::sensors::ImuSource;
use crate::messages::sensor::ImuSample;

const STANDARD_GRAVITY_MPS2: f32 = 9.80665;
const DEG_TO_RAD: f32 = PI / 180.0;

pub struct Mpu6050ImuSource<I2C>
where
    I2C: I2c,
{
    driver: Mpu6050<I2C>,
    accel_mps2_per_lsb: f32,
    gyro_rad_s_per_lsb: f32,
}

impl<I2C> Mpu6050ImuSource<I2C>
where
    I2C: I2c,
{
    pub fn new(driver: Mpu6050<I2C>, accel_range: AccelRange, gyro_range: GyroRange) -> Self {
        Self {
            driver,
            accel_mps2_per_lsb: accel_mps2_per_lsb(accel_range),
            gyro_rad_s_per_lsb: gyro_rad_s_per_lsb(gyro_range),
        }
    }

    pub fn driver_mut(&mut self) -> &mut Mpu6050<I2C> {
        &mut self.driver
    }

    pub fn into_driver(self) -> Mpu6050<I2C> {
        self.driver
    }
}

impl<I2C> ImuSource for Mpu6050ImuSource<I2C>
where
    I2C: I2c,
{
    type Error = Mpu6050Error;

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

const fn accel_mps2_per_lsb(range: AccelRange) -> f32 {
    let g_per_lsb = match range {
        AccelRange::G2 => 1.0 / 16_384.0,
        AccelRange::G4 => 1.0 / 8_192.0,
        AccelRange::G8 => 1.0 / 4_096.0,
        AccelRange::G16 => 1.0 / 2_048.0,
    };

    g_per_lsb * STANDARD_GRAVITY_MPS2
}

const fn gyro_rad_s_per_lsb(range: GyroRange) -> f32 {
    let dps_per_lsb = match range {
        GyroRange::Dps250 => 1.0 / 131.0,
        GyroRange::Dps500 => 1.0 / 65.5,
        GyroRange::Dps1000 => 1.0 / 32.8,
        GyroRange::Dps2000 => 1.0 / 16.4,
    };

    dps_per_lsb * DEG_TO_RAD
}

#[cfg(test)]
mod tests {
    use super::{accel_mps2_per_lsb, gyro_rad_s_per_lsb};
    use crate::drivers::mpu6050::{AccelRange, GyroRange};

    #[test]
    fn accel_scaling_increases_with_range() {
        assert!(accel_mps2_per_lsb(AccelRange::G2) < accel_mps2_per_lsb(AccelRange::G4));
        assert!(accel_mps2_per_lsb(AccelRange::G4) < accel_mps2_per_lsb(AccelRange::G8));
        assert!(accel_mps2_per_lsb(AccelRange::G8) < accel_mps2_per_lsb(AccelRange::G16));
    }

    #[test]
    fn gyro_scaling_increases_with_range() {
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps250) < gyro_rad_s_per_lsb(GyroRange::Dps500));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps500) < gyro_rad_s_per_lsb(GyroRange::Dps1000));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps1000) < gyro_rad_s_per_lsb(GyroRange::Dps2000));
    }
}
