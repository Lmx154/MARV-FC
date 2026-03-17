//! LSM6DSV32X-backed IMU source adapter for the generic producer service.

use core::f32::consts::PI;

use embedded_hal_async::spi::SpiDevice;

use crate::drivers::lsm6dsv32x::{AccelRange, Error as Lsm6dsv32xError, GyroRange, Lsm6dsv32x};
use crate::interfaces::sensors::ImuSource;
use crate::messages::sensor::ImuSample;

const STANDARD_GRAVITY_MPS2: f32 = 9.80665;
const DEG_TO_RAD: f32 = PI / 180.0;

pub const LSM6DSV32X_ACCEL_MG_PER_LSB_4G: f32 = 0.122;
pub const LSM6DSV32X_ACCEL_MG_PER_LSB_8G: f32 = 0.244;
pub const LSM6DSV32X_ACCEL_MG_PER_LSB_16G: f32 = 0.488;
pub const LSM6DSV32X_ACCEL_MG_PER_LSB_32G: f32 = 0.976;

pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_125DPS: f32 = 4.375;
pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_250DPS: f32 = 8.75;
pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_500DPS: f32 = 17.50;
pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_1000DPS: f32 = 35.0;
pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_2000DPS: f32 = 70.0;
pub const LSM6DSV32X_GYRO_MDPS_PER_LSB_4000DPS: f32 = 140.0;

pub struct Lsm6dsv32xImuSource<SPI>
where
    SPI: SpiDevice<u8>,
{
    driver: Lsm6dsv32x<SPI>,
    accel_mps2_per_lsb: f32,
    gyro_rad_s_per_lsb: f32,
}

impl<SPI> Lsm6dsv32xImuSource<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn new(driver: Lsm6dsv32x<SPI>, accel_range: AccelRange, gyro_range: GyroRange) -> Self {
        Self {
            driver,
            accel_mps2_per_lsb: accel_mps2_per_lsb(accel_range),
            gyro_rad_s_per_lsb: gyro_rad_s_per_lsb(gyro_range),
        }
    }

    pub fn driver_mut(&mut self) -> &mut Lsm6dsv32x<SPI> {
        &mut self.driver
    }

    pub fn into_driver(self) -> Lsm6dsv32x<SPI> {
        self.driver
    }
}

impl<SPI> ImuSource for Lsm6dsv32xImuSource<SPI>
where
    SPI: SpiDevice<u8>,
{
    type Error = Lsm6dsv32xError;

    fn read_imu_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
        async move {
            // Avoid relying on a combined gyro+accel burst until LSM6DSV32X
            // multi-register read behavior is verified on the shared SPI bus.
            let accel = self.driver.read_accel().await?;
            let gyro = self.driver.read_gyro().await?;

            Ok(ImuSample {
                accel_mps2: [
                    accel[0] as f32 * self.accel_mps2_per_lsb,
                    accel[1] as f32 * self.accel_mps2_per_lsb,
                    accel[2] as f32 * self.accel_mps2_per_lsb,
                ],
                gyro_rad_s: [
                    gyro[0] as f32 * self.gyro_rad_s_per_lsb,
                    gyro[1] as f32 * self.gyro_rad_s_per_lsb,
                    gyro[2] as f32 * self.gyro_rad_s_per_lsb,
                ],
            })
        }
    }
}

const fn accel_mps2_per_lsb(range: AccelRange) -> f32 {
    let mg_per_lsb = match range {
        AccelRange::G4 => LSM6DSV32X_ACCEL_MG_PER_LSB_4G,
        AccelRange::G8 => LSM6DSV32X_ACCEL_MG_PER_LSB_8G,
        AccelRange::G16 => LSM6DSV32X_ACCEL_MG_PER_LSB_16G,
        AccelRange::G32 => LSM6DSV32X_ACCEL_MG_PER_LSB_32G,
    };

    mg_per_lsb * 1.0e-3 * STANDARD_GRAVITY_MPS2
}

const fn gyro_rad_s_per_lsb(range: GyroRange) -> f32 {
    let mdps_per_lsb = match range {
        GyroRange::Dps125 => LSM6DSV32X_GYRO_MDPS_PER_LSB_125DPS,
        GyroRange::Dps250 => LSM6DSV32X_GYRO_MDPS_PER_LSB_250DPS,
        GyroRange::Dps500 => LSM6DSV32X_GYRO_MDPS_PER_LSB_500DPS,
        GyroRange::Dps1000 => LSM6DSV32X_GYRO_MDPS_PER_LSB_1000DPS,
        GyroRange::Dps2000 => LSM6DSV32X_GYRO_MDPS_PER_LSB_2000DPS,
        GyroRange::Dps4000 => LSM6DSV32X_GYRO_MDPS_PER_LSB_4000DPS,
    };

    mdps_per_lsb * 1.0e-3 * DEG_TO_RAD
}

#[cfg(test)]
mod tests {
    use super::{accel_mps2_per_lsb, gyro_rad_s_per_lsb};
    use crate::drivers::lsm6dsv32x::{AccelRange, GyroRange};

    #[test]
    fn accel_scaling_increases_with_range() {
        assert!(accel_mps2_per_lsb(AccelRange::G4) < accel_mps2_per_lsb(AccelRange::G8));
        assert!(accel_mps2_per_lsb(AccelRange::G8) < accel_mps2_per_lsb(AccelRange::G16));
        assert!(accel_mps2_per_lsb(AccelRange::G16) < accel_mps2_per_lsb(AccelRange::G32));
    }

    #[test]
    fn gyro_scaling_increases_with_range() {
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps125) < gyro_rad_s_per_lsb(GyroRange::Dps250));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps250) < gyro_rad_s_per_lsb(GyroRange::Dps500));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps500) < gyro_rad_s_per_lsb(GyroRange::Dps1000));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps1000) < gyro_rad_s_per_lsb(GyroRange::Dps2000));
        assert!(gyro_rad_s_per_lsb(GyroRange::Dps2000) < gyro_rad_s_per_lsb(GyroRange::Dps4000));
    }
}
