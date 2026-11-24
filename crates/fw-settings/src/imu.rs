//! The two IMUs' parameters: the calibration offsets and the chip settings.
//!
//! The calibration offsets are gyro in rad/s and accelerometer in m/s^2 per axis, which the
//! adapter on the flight core subtracts from every sample in software (the chips' offset registers
//! stay zero). `Live`: no validator, no apply; written by the gyro and one-position accelerometer
//! calibrations (`fw_core::calib`). The gyro range is the gyros' full scale (2000 dps); the
//! accelerometer range is ArduPilot's largest accepted offset (one gravity).
//!
//! The chip settings (`IMUn_ACC_FS`, `IMUn_GYR_FS`, `IMUn_ODR_HZ`, in [`SETTINGS`]) are
//! `Reboot`-class: [`imu1_setup`] and [`imu2_setup`] turn them into the chip's whole configuration,
//! its scale factors and the nominal sample period, and are the one place that knows which values
//! the chips offer. The store checks only the parameters' ranges when they are written (they are
//! `Reboot`-class, so the store runs no device validator for them); the boot runs the setups on the
//! stored values and puts an IMU whose values the chip does not offer back to its defaults. Pure;
//! the flight core only reads the plain data of the result, once, at boot.

use fw_core::calib::{ACCEL_OFFSET_MAX_MSS, GYRO_FULL_SCALE_RADS, IMUS};
use fw_core::params::{Apply, ParamDef};
use fw_core::rotation::{self, Rotation};
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

/// Per IMU (IMU1, IMU2): the accelerometer full scale in g.
pub const ACC_FS: [&str; IMUS] = ["IMU1_ACC_FS", "IMU2_ACC_FS"];
/// Per IMU (IMU1, IMU2): the gyroscope full scale in dps.
pub const GYR_FS: [&str; IMUS] = ["IMU1_GYR_FS", "IMU2_GYR_FS"];
/// Per IMU (IMU1, IMU2): the output data rate in Hz of both sensors.
pub const ODR_HZ: [&str; IMUS] = ["IMU1_ODR_HZ", "IMU2_ODR_HZ"];

/// The board's orientation, board to vehicle: ArduPilot's name and numbering (0 to 15, see
/// [`Rotation`]).
pub const AHRS_ORIENTATION: &str = "AHRS_ORIENTATION";

/// IMU1's (ICM-45686) fixed chip-to-board rotation, an hwdef line in ArduPilot's terms: the owner's
/// axes are +X forward, +Y left, +Z up, the board's FRD (x forward, y right, z down) is
/// `ROLL_180` of them (`x, -y, -z`).
pub const IMU1_MOUNT: Rotation = Rotation::Roll180;
/// IMU2's (LSM6DSK320X) fixed chip-to-board rotation: the same axes as IMU1, so `ROLL_180`.
pub const IMU2_MOUNT: Rotation = Rotation::Roll180;

/// The number of IMU calibration parameters.
pub const COUNT: usize = 12;
/// The number of IMU chip settings.
pub const SETTINGS_COUNT: usize = 6;

const fn gyr(name: &'static str, device: u8) -> ParamDef {
    ParamDef::f32(
        name,
        device,
        0.0,
        -GYRO_FULL_SCALE_RADS,
        GYRO_FULL_SCALE_RADS,
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

/// The IMUs' chip settings, appended to the firmware's table after the radio's (indices 70 to 75):
/// IMU1's accelerometer full scale, gyroscope full scale and rate, then IMU2's. The defaults are
/// 16 g, 2000 dps and 800 Hz (Betaflight's); each default makes a setup, which the tests prove.
pub static SETTINGS: [ParamDef; SETTINGS_COUNT] = [
    ParamDef::i32("IMU1_ACC_FS", IMU1, 16, 2, 32, Apply::Reboot),
    ParamDef::i32("IMU1_GYR_FS", IMU1, 2000, 125, 4000, Apply::Reboot),
    ParamDef::i32("IMU1_ODR_HZ", IMU1, 800, 25, 800, Apply::Reboot),
    ParamDef::i32("IMU2_ACC_FS", IMU2, 16, 2, 16, Apply::Reboot),
    ParamDef::i32("IMU2_GYR_FS", IMU2, 2000, 250, 4000, Apply::Reboot),
    ParamDef::i32("IMU2_ODR_HZ", IMU2, 800, 25, 800, Apply::Reboot),
];

/// The number of orientation parameters the IMUs read.
pub const BOARD_COUNT: usize = 1;

/// `AHRS_ORIENTATION`, appended to the firmware's table after the IMUs' chip settings (index 76):
/// the rotation from the board's frame to the vehicle's (body FRD), applied by both IMU adapters
/// after the offsets (ArduPilot: board orientation, next boot). Device 0, the system: a refused
/// IMU setup never resets it. `Reboot`: it reaches core 1 by value at boot, no cell. Default 0:
/// the board lies in the vehicle's axes.
pub static BOARD: [ParamDef; BOARD_COUNT] = [ParamDef::i32(
    AHRS_ORIENTATION,
    0,
    0,
    rotation::MIN,
    rotation::MAX,
    Apply::Reboot,
)];

/// The stored `AHRS_ORIENTATION` as a rotation (a value outside 0 to 15 cannot be stored; none
/// reads as no rotation).
pub fn board_rotation(value: Option<i32>) -> Rotation {
    value.and_then(Rotation::from_i32).unwrap_or_default()
}

/// One IMU's configuration for the boot: the chip's whole `Config`, the full scales the adapter
/// scales the samples with (the same values as in `config`), the nominal sample period and the
/// board's orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuSetup<C, A, G> {
    /// The driver's configuration: today's `config()` of the adapter with the three values in place.
    pub config: C,
    /// The accelerometer full scale of `config`.
    pub accel_fs: A,
    /// The gyroscope full scale of `config`.
    pub gyro_fs: G,
    /// The nominal sample period in microseconds, 1e6 / the rate.
    pub period_us: u32,
    /// `AHRS_ORIENTATION`: applied after the offsets, read once at boot. The setup functions give
    /// no rotation; [`ImuSetup::with_board`] sets it.
    pub board: Rotation,
}

impl<C, A, G> ImuSetup<C, A, G> {
    /// The same setup with the board's orientation `board`.
    pub fn with_board(self, board: Rotation) -> Self {
        Self { board, ..self }
    }
}

/// IMU1's setup (ICM-45686).
pub type Imu1Setup = ImuSetup<icm45686::Config, icm45686::AccelFullScale, icm45686::GyroFullScale>;
/// IMU2's setup (LSM6DSK320X).
pub type Imu2Setup =
    ImuSetup<lsm6dsk320x::Config, lsm6dsk320x::AccelFullScale, lsm6dsk320x::GyroFullScale>;

const BAD_ACC_1: &str = "not 2, 4, 8, 16 or 32";
const BAD_ACC_2: &str = "not 2, 4, 8 or 16";
const BAD_GYR_1: &str = "not 125, 250, 500, 1000, 2000 or 4000";
const BAD_GYR_2: &str = "not 250, 500, 1000, 2000 or 4000";
const BAD_RATE: &str = "rate not offered by the chip";
const NOT_ALLOWED: &str = "combination not allowed by the chip";

/// The sample period of a rate in whole hertz, 1e6 / `odr_hz` (exact for 25, 50, 100, 200, 400, 800).
fn period_us(odr_hz: i32) -> Option<u32> {
    u32::try_from(odr_hz)
        .ok()
        .and_then(|hz| 1_000_000u32.checked_div(hz))
}

/// IMU1's setup from `IMU1_ACC_FS` (g), `IMU1_GYR_FS` (dps) and `IMU1_ODR_HZ` (Hz): the
/// accelerometer in 2, 4, 8, 16 or 32 g, the gyroscope in 125, 250, 500, 1000, 2000 or 4000 dps,
/// both sensors at 25, 50, 100, 200, 400 or 800 Hz (the single-sample path stops at 800 Hz);
/// everything else as the adapter has it: internal clock, the driver's default filters, no FIFO,
/// INT1 data-ready LATCHED. The first value the chip does not offer gives its reason; then the
/// driver's own `Config::validate`.
pub fn imu1_setup(acc_fs: i32, gyr_fs: i32, odr_hz: i32) -> Result<Imu1Setup, &'static str> {
    use icm45686::{AccelFullScale, ClockSource, Config, GyroFullScale, Int1Config, Int1Mode, Odr};
    let accel_fs = match acc_fs {
        2 => AccelFullScale::G2,
        4 => AccelFullScale::G4,
        8 => AccelFullScale::G8,
        16 => AccelFullScale::G16,
        32 => AccelFullScale::G32,
        _ => return Err(BAD_ACC_1),
    };
    let gyro_fs = match gyr_fs {
        125 => GyroFullScale::Dps125,
        250 => GyroFullScale::Dps250,
        500 => GyroFullScale::Dps500,
        1000 => GyroFullScale::Dps1000,
        2000 => GyroFullScale::Dps2000,
        4000 => GyroFullScale::Dps4000,
        _ => return Err(BAD_GYR_1),
    };
    let odr = match odr_hz {
        25 => Odr::Hz25,
        50 => Odr::Hz50,
        100 => Odr::Hz100,
        200 => Odr::Hz200,
        400 => Odr::Hz400,
        800 => Odr::Hz800,
        _ => return Err(BAD_RATE),
    };
    let config = Config::new()
        .with_accel_fs(accel_fs)
        .with_gyro_fs(gyro_fs)
        .with_accel_odr(odr)
        .with_gyro_odr(odr)
        .with_clock(ClockSource::Internal)
        .with_int1(
            Int1Config::new()
                .with_data_ready(true)
                .with_mode(Int1Mode::Latched),
        );
    config.validate().map_err(|_| NOT_ALLOWED)?;
    Ok(ImuSetup {
        config,
        accel_fs,
        gyro_fs,
        period_us: period_us(odr_hz).ok_or(BAD_RATE)?,
        board: Rotation::None,
    })
}

/// IMU2's setup from `IMU2_ACC_FS` (g), `IMU2_GYR_FS` (dps) and `IMU2_ODR_HZ` (Hz): the low-g
/// accelerometer in 2, 4, 8 or 16 g, the gyroscope in 250, 500, 1000, 2000 or 4000 dps, both at 25,
/// 50, 100, 200, 400 or 800 Hz through the high-accuracy ODR set "800" (the ODR codes 30, 60, 120,
/// 240, 480 and 960 are those rates there); everything else as the adapter has it: the driver's
/// default filters, high-g off, no FIFO, the low-g data-ready on INT1 LATCHED.
pub fn imu2_setup(acc_fs: i32, gyr_fs: i32, odr_hz: i32) -> Result<Imu2Setup, &'static str> {
    use lsm6dsk320x::{
        AccelFullScale, Config, DataReady, DataReadyMode, GyroFullScale, HaodrSet, IntRouting, Odr,
        OperatingMode,
    };
    let accel_fs = match acc_fs {
        2 => AccelFullScale::G2,
        4 => AccelFullScale::G4,
        8 => AccelFullScale::G8,
        16 => AccelFullScale::G16,
        _ => return Err(BAD_ACC_2),
    };
    let gyro_fs = match gyr_fs {
        250 => GyroFullScale::Dps250,
        500 => GyroFullScale::Dps500,
        1000 => GyroFullScale::Dps1000,
        2000 => GyroFullScale::Dps2000,
        4000 => GyroFullScale::Dps4000,
        _ => return Err(BAD_GYR_2),
    };
    let odr = match odr_hz {
        25 => Odr::Hz30,
        50 => Odr::Hz60,
        100 => Odr::Hz120,
        200 => Odr::Hz240,
        400 => Odr::Hz480,
        800 => Odr::Hz960,
        _ => return Err(BAD_RATE),
    };
    let config = Config::new()
        .with_accel_fs(accel_fs)
        .with_accel_odr(odr)
        .with_gyro_fs(gyro_fs)
        .with_gyro_odr(odr)
        .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base800))
        .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_accel(true)))
        .with_int2(IntRouting::new())
        .with_data_ready_mode(DataReadyMode::Latched);
    config.validate().map_err(|_| NOT_ALLOWED)?;
    Ok(ImuSetup {
        config,
        accel_fs,
        gyro_fs,
        period_us: period_us(odr_hz).ok_or(BAD_RATE)?,
        board: Rotation::None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};

    /// The MAVLink STATUSTEXT text field: a refusal text is sent after `<NAME>: `, within it.
    const STATUSTEXT_BYTES: usize = 50;

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
    fn the_setups_with_todays_values_are_todays_configs() {
        // src/imu1.rs `config()` with 4 g, 250 dps, 800 Hz.
        let today1 = icm45686::Config::new()
            .with_accel_fs(icm45686::AccelFullScale::G4)
            .with_gyro_fs(icm45686::GyroFullScale::Dps250)
            .with_accel_odr(icm45686::Odr::Hz800)
            .with_gyro_odr(icm45686::Odr::Hz800)
            .with_clock(icm45686::ClockSource::Internal)
            .with_int1(
                icm45686::Int1Config::new()
                    .with_data_ready(true)
                    .with_mode(icm45686::Int1Mode::Latched),
            );
        let s1 = imu1_setup(4, 250, 800).unwrap();
        assert_eq!(s1.config, today1);
        assert_eq!(s1.period_us, 1250);
        // src/imu2.rs `config()` with 4 g, 500 dps, the code 960 in the set "800".
        use lsm6dsk320x as l;
        let today2 = l::Config::new()
            .with_accel_fs(l::AccelFullScale::G4)
            .with_accel_odr(l::Odr::Hz960)
            .with_gyro_fs(l::GyroFullScale::Dps500)
            .with_gyro_odr(l::Odr::Hz960)
            .with_mode(l::OperatingMode::HighAccuracyOdr(l::HaodrSet::Base800))
            .with_int1(l::IntRouting::new().with_data_ready(l::DataReady::new().with_accel(true)))
            .with_int2(l::IntRouting::new())
            .with_data_ready_mode(l::DataReadyMode::Latched);
        let s2 = imu2_setup(4, 500, 800).unwrap();
        assert_eq!(s2.config, today2);
        assert_eq!(s2.period_us, 1250);
    }

    #[test]
    fn every_offered_value_makes_a_setup_and_no_other_does() {
        const RATES: [i32; 6] = [25, 50, 100, 200, 400, 800];
        for fs in 0..=40 {
            let ok1 = [2, 4, 8, 16, 32].contains(&fs);
            let ok2 = [2, 4, 8, 16].contains(&fs);
            assert_eq!(imu1_setup(fs, 2000, 800).is_ok(), ok1, "{fs}");
            assert_eq!(imu2_setup(fs, 2000, 800).is_ok(), ok2, "{fs}");
        }
        for dps in 0..=4100 {
            let ok1 = [125, 250, 500, 1000, 2000, 4000].contains(&dps);
            let ok2 = [250, 500, 1000, 2000, 4000].contains(&dps);
            assert_eq!(imu1_setup(16, dps, 800).is_ok(), ok1, "{dps}");
            assert_eq!(imu2_setup(16, dps, 800).is_ok(), ok2, "{dps}");
        }
        for hz in 0..=1700 {
            let ok = RATES.contains(&hz);
            assert_eq!(imu1_setup(16, 2000, hz).is_ok(), ok, "{hz}");
            assert_eq!(imu2_setup(16, 2000, hz).is_ok(), ok, "{hz}");
            if ok {
                assert_eq!(
                    imu1_setup(16, 2000, hz).unwrap().period_us,
                    1_000_000 / hz as u32
                );
                assert_eq!(
                    imu2_setup(16, 2000, hz).unwrap().period_us,
                    1_000_000 / hz as u32
                );
            }
        }
    }

    #[test]
    fn every_refusal_has_its_text_of_at_most_50_bytes() {
        assert_eq!(imu1_setup(3, 2000, 800), Err(BAD_ACC_1));
        assert_eq!(imu1_setup(16, 3, 800), Err(BAD_GYR_1));
        assert_eq!(imu1_setup(16, 2000, 3), Err(BAD_RATE));
        assert_eq!(imu2_setup(32, 2000, 800), Err(BAD_ACC_2));
        assert_eq!(imu2_setup(16, 125, 800), Err(BAD_GYR_2));
        assert_eq!(imu2_setup(16, 2000, 1600), Err(BAD_RATE));
        for text in [
            BAD_ACC_1,
            BAD_ACC_2,
            BAD_GYR_1,
            BAD_GYR_2,
            BAD_RATE,
            NOT_ALLOWED,
        ] {
            assert!(text.len() <= STATUSTEXT_BYTES, "{text}");
        }
        assert_eq!(BAD_ACC_1, "not 2, 4, 8, 16 or 32");
        assert_eq!(BAD_GYR_1, "not 125, 250, 500, 1000, 2000 or 4000");
        assert_eq!(NOT_ALLOWED, "combination not allowed by the chip");
    }

    /// H2: both IMUs' mounting turns chip gravity (0, 0, +g) at rest, level, into (0, 0, -g) (FRD,
    /// specific force up), and +X forward, +Y left stay forward and right.
    #[test]
    fn the_imu_mounts_turn_chip_axes_into_frd() {
        for mount in [IMU1_MOUNT, IMU2_MOUNT] {
            assert_eq!(mount.rotate([0.0, 0.0, 9.81]), [0.0, 0.0, -9.81]);
            assert_eq!(mount.rotate([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);
            // Chip +Y is left: body -Y (right is positive).
            assert_eq!(mount.rotate([0.0, 1.0, 0.0]), [0.0, -1.0, 0.0]);
        }
    }

    /// The whole firmware table, as the firmware builds the store.
    fn store() -> Params<{ crate::COUNT }> {
        Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap()
    }

    #[test]
    fn the_six_settings_follow_rc_with_their_defaults_ranges_and_reboot_class() {
        assert_eq!(crate::COUNT, 123);
        let expected = [
            ("IMU1_ACC_FS", IMU1, 16, 2, 32),
            ("IMU1_GYR_FS", IMU1, 2000, 125, 4000),
            ("IMU1_ODR_HZ", IMU1, 800, 25, 800),
            ("IMU2_ACC_FS", IMU2, 16, 2, 16),
            ("IMU2_GYR_FS", IMU2, 2000, 250, 4000),
            ("IMU2_ODR_HZ", IMU2, 800, 25, 800),
        ];
        for (k, (n, device, default, min, max)) in expected.iter().enumerate() {
            let d = &crate::TABLE[70 + k];
            assert_eq!(d.name, *n);
            assert_eq!(d.device, *device);
            assert_eq!(d.default, ParamValue::I32(*default));
            assert_eq!(d.min, ParamValue::I32(*min));
            assert_eq!(d.max, ParamValue::I32(*max));
            assert_eq!(d.apply, Apply::Reboot);
            assert_eq!(SETTINGS[k].name, *n);
        }
    }

    /// A rate above 800 Hz and IMU1's 32 g for IMU2 are outside the range: refused at the write.
    #[test]
    fn a_rate_of_1600_hz_and_32_g_on_imu2_are_refused() {
        let mut p = store();
        for n in ["IMU1_ODR_HZ", "IMU2_ODR_HZ"] {
            let i = p.find(&name(n)).unwrap();
            assert_eq!(p.set(i, ParamValue::I32(1600)), Err(ParamError::OutOfRange));
            assert_eq!(p.get(i), Some(ParamValue::I32(800)));
        }
        let i = p.find(&name("IMU2_ACC_FS")).unwrap();
        assert_eq!(p.set(i, ParamValue::I32(32)), Err(ParamError::OutOfRange));
        assert_eq!(p.get(i), Some(ParamValue::I32(16)));
        assert_eq!(imu2_setup(32, 2000, 800), Err(BAD_ACC_2));
        let j = p.find(&name("IMU1_ACC_FS")).unwrap();
        assert_eq!(p.set(j, ParamValue::I32(32)), Ok(ParamValue::I32(32)));
    }

    #[test]
    fn a_setting_write_applies_after_reboot_and_changes_no_live_or_configure_mask() {
        let mut p = store();
        let i = p.find(&name("IMU1_ODR_HZ")).unwrap();
        assert_eq!(p.set(i, ParamValue::I32(400)), Ok(ParamValue::I32(400)));
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_live_mask(), 0);
        assert_eq!(
            p.take_notice().unwrap().kind,
            fw_core::params::NoticeKind::AppliesAfterReboot
        );
    }

    #[test]
    fn ranges_are_the_calibration_limits_and_a_set_is_live() {
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        let g = p.find(&name("IMU2_GYR_OFFZ")).unwrap();
        let max = GYRO_FULL_SCALE_RADS;
        assert_eq!(p.set(g, ParamValue::F32(max)), Ok(ParamValue::F32(max)));
        assert_eq!(
            p.set(g, ParamValue::F32(-max * 1.01)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(
            p.set(g, ParamValue::F32(f32::NAN)),
            Err(ParamError::OutOfRange)
        );
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
