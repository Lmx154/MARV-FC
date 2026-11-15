//! The two IMUs' calibration offsets: gyro in rad/s and accelerometer in m/s^2 per axis, which the
//! adapter on the flight core subtracts from every sample in software (the chips' offset registers
//! stay zero). `Live`: no validator, no apply; written by the gyro and one-position accelerometer
//! calibrations (`fw_core::calib`), whose limits are the ranges here.

use fw_core::calib::{ACCEL_OFFSET_MAX_MSS, GYRO_OFFSET_MAX_RADS, IMUS};
use fw_core::params::{Apply, ParamDef};
use mavlink_device::marv::MarvDevice;

const IMU1: u8 = MarvDevice::MARV_DEVICE_IMU1 as u8;
const IMU2: u8 = MarvDevice::MARV_DEVICE_IMU2 as u8;

/// Per IMU (IMU1, IMU2): the gyro offsets X, Y, Z in rad/s.
pub const GYR_OFF: [[&str; 3]; IMUS] = [
    ["IMU1_GYR_OFFX", "IMU1_GYR_OFFY", "IMU1_GYR_OFFZ"],
    ["IMU2_GYR_OFFX", "IMU2_GYR_OFFY", "IMU2_GYR_OFFZ"],
];
/// Per IMU (IMU1, IMU2): the accelerometer offsets X, Y, Z in m/s^2.
pub const ACC_OFF: [[&str; 3]; IMUS] = [
    ["IMU1_ACC_OFFX", "IMU1_ACC_OFFY", "IMU1_ACC_OFFZ"],
    ["IMU2_ACC_OFFX", "IMU2_ACC_OFFY", "IMU2_ACC_OFFZ"],
];
/// The device id of each IMU, in the order of [`GYR_OFF`] and [`ACC_OFF`].
pub const DEVICES: [u8; IMUS] = [IMU1, IMU2];

/// The number of IMU parameters.
pub const COUNT: usize = 12;

const fn gyr(name: &'static str, device: u8) -> ParamDef {
    ParamDef::f32(
        name,
        device,
        0.0,
        -GYRO_OFFSET_MAX_RADS,
        GYRO_OFFSET_MAX_RADS,
        Apply::Live,
    )
    .with_calibration()
}

const fn acc(name: &'static str, device: u8) -> ParamDef {
    ParamDef::f32(
        name,
        device,
        0.0,
        -ACCEL_OFFSET_MAX_MSS,
        ACCEL_OFFSET_MAX_MSS,
        Apply::Live,
    )
    .with_calibration()
}

/// The IMUs' parameter table: IMU1 gyro, IMU1 accelerometer, IMU2 gyro, IMU2 accelerometer.
pub static TABLE: [ParamDef; COUNT] = [
    gyr(GYR_OFF[0][0], IMU1),
    gyr(GYR_OFF[0][1], IMU1),
    gyr(GYR_OFF[0][2], IMU1),
    acc(ACC_OFF[0][0], IMU1),
    acc(ACC_OFF[0][1], IMU1),
    acc(ACC_OFF[0][2], IMU1),
    gyr(GYR_OFF[1][0], IMU2),
    gyr(GYR_OFF[1][1], IMU2),
    gyr(GYR_OFF[1][2], IMU2),
    acc(ACC_OFF[1][0], IMU2),
    acc(ACC_OFF[1][1], IMU2),
    acc(ACC_OFF[1][2], IMU2),
];

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};

    fn name(n: &str) -> ParamName {
        ParamName::new(n).unwrap()
    }

    #[test]
    fn twelve_live_calibration_offsets_default_zero() {
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        assert_eq!(p.count() as usize, COUNT);
        for (i, def) in TABLE.iter().enumerate() {
            let imu = i / 6;
            assert_eq!(def.device, DEVICES[imu]);
            assert!(def.name.starts_with(["IMU1_", "IMU2_"][imu]) && def.name.len() <= 16);
            assert_eq!(def.apply, Apply::Live);
            assert!(def.calibration);
            assert_eq!(def.default, ParamValue::F32(0.0));
        }
        for imu in 0..IMUS {
            for n in GYR_OFF[imu].iter().chain(ACC_OFF[imu].iter()) {
                assert!(p.find(&name(n)).is_some(), "{n}");
            }
        }
        assert_eq!(p.take_live_mask(), 0);
    }

    #[test]
    fn ranges_are_the_calibration_limits_and_a_set_is_live() {
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        let g = p.find(&name("IMU2_GYR_OFFZ")).unwrap();
        let max = GYRO_OFFSET_MAX_RADS;
        assert_eq!(p.set(g, ParamValue::F32(max)), Ok(ParamValue::F32(max)));
        assert_eq!(p.set(g, ParamValue::F32(-max * 1.01)), Err(ParamError::OutOfRange));
        assert_eq!(p.set(g, ParamValue::F32(f32::NAN)), Err(ParamError::OutOfRange));
        assert_eq!(p.take_live_mask(), 1 << IMU2);
        let a = p.find(&name("IMU1_ACC_OFFX")).unwrap();
        assert_eq!(p.set(a, ParamValue::F32(-9.8)), Ok(ParamValue::F32(-9.8)));
        assert_eq!(p.set(a, ParamValue::F32(9.81)), Err(ParamError::OutOfRange));
        assert_eq!(p.take_live_mask(), 1 << IMU1);
        assert_eq!(p.take_configure_mask(), 0);
        assert!(p.dirty());
        assert_eq!(p.reset_calibration(), 2);
        assert_eq!(p.get(g), Some(ParamValue::F32(0.0)));
        assert_eq!(p.get(a), Some(ParamValue::F32(0.0)));
    }
}
