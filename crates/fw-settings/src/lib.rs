//! Device-specific pure logic of the MARV V2 firmware: per device, the parameter table and the pure
//! `settings_to_config` with its validator. No hardware, no clock; tested on the host. Generic
//! logic (the store, the health machine, admission) is in `fw-core`.
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

#[cfg(test)]
extern crate std;

pub mod baro;
pub mod batt;
pub mod imu;
pub mod mag;
pub mod rc;
pub mod sd;

use fw_core::params::{Apply, ParamDef, Params};
use mavlink_device::ParamName;

/// The number of parameters of the whole firmware.
pub const COUNT: usize = baro::COUNT
    + mag::COUNT
    + batt::COUNT
    + imu::COUNT
    + sd::COUNT
    + rc::COUNT
    + imu::SETTINGS_COUNT;

/// The firmware's one parameter table: every device's table in the order BARO, MAG, BATT, IMU, SD, RC,
/// then the IMUs' chip settings (appended, so the first 70 indices did not change).
pub static TABLE: [ParamDef; COUNT] = concat();

/// A device whose stored settings the boot refused: its `Configure`- and `Reboot`-class parameters
/// (settings) go back to their defaults in RAM; its `Live` parameters (the
/// calibration offsets) are kept. Returns the number of parameters put back.
pub fn reset_device_settings<const N: usize>(params: &mut Params<N>, device: u8) -> usize {
    let mut n = 0;
    for def in TABLE
        .iter()
        .filter(|d| d.device == device && d.apply != Apply::Live)
    {
        if let Ok(name) = ParamName::new(def.name)
            && params.load(&name, def.default).is_ok()
        {
            n += 1;
        }
    }
    n
}

#[allow(clippy::indexing_slicing)] // the loops run exactly over each source table's length
const fn concat() -> [ParamDef; COUNT] {
    let mut out = [baro::TABLE[0]; COUNT];
    let mut n = 0;
    let mut i = 0;
    while i < baro::COUNT {
        out[n] = baro::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < mag::COUNT {
        out[n] = mag::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < batt::COUNT {
        out[n] = batt::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < imu::COUNT {
        out[n] = imu::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < sd::COUNT {
        out[n] = sd::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < rc::COUNT {
        out[n] = rc::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < imu::SETTINGS_COUNT {
        out[n] = imu::SETTINGS[i];
        n += 1;
        i += 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::ParamStore;

    #[test]
    fn the_whole_table_is_accepted_with_every_validator() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(params.set_validator(baro::TABLE[0].device, baro::validate));
        assert!(params.set_validator(mag::TABLE[0].device, mag::validate));
        assert_eq!(params.count() as usize, COUNT);
        assert_eq!(COUNT, 76);
        let mut expected = baro::TABLE
            .iter()
            .chain(mag::TABLE.iter())
            .chain(batt::TABLE.iter())
            .chain(imu::TABLE.iter())
            .chain(sd::TABLE.iter())
            .chain(rc::TABLE.iter())
            .chain(imu::SETTINGS.iter());
        for def in &TABLE {
            assert_eq!(def.name, expected.next().unwrap().name);
        }
        assert!(expected.next().is_none());
    }

    /// The first 70 parameters keep their indices (a stored record is indexed by name, but the tools list by index): the
    /// six IMU chip settings are appended after them.
    #[test]
    fn the_first_70_names_keep_their_indices() {
        const FIRST_70: [&str; 70] = [
            "BARO_ODR_HZ",
            "BARO_OSR_P",
            "BARO_OSR_T",
            "BARO_IIR_P",
            "BARO_IIR_T",
            "MAG_ODR_HZ",
            "MAG_AVG",
            "MAG_OFFX",
            "MAG_OFFY",
            "MAG_OFFZ",
            "BATT_V_SCALE",
            "BATT_A_SCALE",
            "BATT_A_OFFSET",
            "IMU1_GYR_OFFX",
            "IMU1_GYR_OFFY",
            "IMU1_GYR_OFFZ",
            "IMU1_ACC_OFFX",
            "IMU1_ACC_OFFY",
            "IMU1_ACC_OFFZ",
            "IMU2_GYR_OFFX",
            "IMU2_GYR_OFFY",
            "IMU2_GYR_OFFZ",
            "IMU2_ACC_OFFX",
            "IMU2_ACC_OFFY",
            "IMU2_ACC_OFFZ",
            "LOG_BACKEND_TYPE",
            "RC1_MIN",
            "RC1_TRIM",
            "RC1_MAX",
            "RC1_REVERSED",
            "RC1_DZ",
            "RC2_MIN",
            "RC2_TRIM",
            "RC2_MAX",
            "RC2_REVERSED",
            "RC2_DZ",
            "RC3_MIN",
            "RC3_TRIM",
            "RC3_MAX",
            "RC3_REVERSED",
            "RC3_DZ",
            "RC4_MIN",
            "RC4_TRIM",
            "RC4_MAX",
            "RC4_REVERSED",
            "RC4_DZ",
            "RC5_MIN",
            "RC5_TRIM",
            "RC5_MAX",
            "RC5_REVERSED",
            "RC5_DZ",
            "RC6_MIN",
            "RC6_TRIM",
            "RC6_MAX",
            "RC6_REVERSED",
            "RC6_DZ",
            "RC7_MIN",
            "RC7_TRIM",
            "RC7_MAX",
            "RC7_REVERSED",
            "RC7_DZ",
            "RC8_MIN",
            "RC8_TRIM",
            "RC8_MAX",
            "RC8_REVERSED",
            "RC8_DZ",
            "RCMAP_ROLL",
            "RCMAP_PITCH",
            "RCMAP_THROTTLE",
            "RCMAP_YAW",
        ];
        for (i, name) in FIRST_70.iter().enumerate() {
            assert_eq!(TABLE[i].name, *name, "index {i}");
        }
        let appended = [
            "IMU1_ACC_FS",
            "IMU1_GYR_FS",
            "IMU1_ODR_HZ",
            "IMU2_ACC_FS",
            "IMU2_GYR_FS",
            "IMU2_ODR_HZ",
        ];
        for (k, name) in appended.iter().enumerate() {
            assert_eq!(TABLE[70 + k].name, *name, "index {}", 70 + k);
        }
    }

    /// decision-sd-logging Q4: the start burst of a log (the header, a PARM for every parameter,
    /// the boot texts that wait for the log's text cursor: the ring's depth and its sticky slots)
    /// is at most 16 KiB, also with the writer's alignment filler (under one 4 KiB piece) in the
    /// ring before it. Encoded from the table, not counted from literals.
    #[test]
    fn the_start_burst_is_at_most_16_kib() {
        use fw_core::dflog::{HEADER_SEQUENCE_LEN, encode_msg, encode_parm, write_header};
        use fw_core::text_ring::{DEPTH, STICKY};
        use mavlink_device::ParamValue;
        let as_f32 = |v: ParamValue| match v {
            ParamValue::I32(i) => i as f32,
            ParamValue::F32(f) => f,
        };
        let mut buf = [0u8; 256];
        let mut header = [0u8; HEADER_SEQUENCE_LEN];
        let mut burst = write_header(&mut header);
        assert_eq!(burst, HEADER_SEQUENCE_LEN);
        for def in &TABLE {
            let d = as_f32(def.default);
            let n = encode_parm(&mut buf, u64::MAX, def.name.as_bytes(), d, d);
            assert!(n > 0, "{}", def.name);
            burst += n;
        }
        let longest = [b'x'; 64];
        for _ in 0..DEPTH + STICKY {
            let n = encode_msg(&mut buf, u64::MAX, &longest);
            assert!(n > 0);
            burst += n;
        }
        const FILLER_MAX: usize = 4096 - 1;
        assert!(burst <= 16 * 1024, "start burst {burst} B");
        assert!(
            burst + FILLER_MAX <= 16 * 1024,
            "with the filler {} B",
            burst + FILLER_MAX
        );
    }

    /// The narrowed boot reset: a refused IMU setting resets the IMU's settings, not
    /// its calibration offsets; the barometer's and the magnetometer's likewise (`MAG_OFF*` kept).
    #[test]
    fn a_refused_device_resets_its_settings_but_not_its_calibration() {
        use mavlink_device::ParamValue;
        let name = |n: &str| ParamName::new(n).unwrap();
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        // Stored values as a record can bring them (range checked only): 3 g is not a range.
        for (n, v) in [
            ("IMU1_ACC_FS", ParamValue::I32(3)),
            ("IMU1_ODR_HZ", ParamValue::I32(100)),
            ("IMU1_ACC_OFFX", ParamValue::F32(0.25)),
            ("IMU1_GYR_OFFZ", ParamValue::F32(-0.01)),
            ("IMU2_ACC_FS", ParamValue::I32(8)),
            ("MAG_OFFX", ParamValue::F32(1.5)),
            ("MAG_AVG", ParamValue::I32(3)),
        ] {
            assert_eq!(p.load(&name(n), v), Ok(true), "{n}");
        }
        assert_eq!(reset_device_settings(&mut p, imu::DEVICES[0]), 3);
        let get = |p: &Params<COUNT>, n: &str| p.get_named(n).unwrap();
        assert_eq!(get(&p, "IMU1_ACC_FS"), ParamValue::I32(16));
        assert_eq!(get(&p, "IMU1_ODR_HZ"), ParamValue::I32(800));
        assert_eq!(get(&p, "IMU1_ACC_OFFX"), ParamValue::F32(0.25));
        assert_eq!(get(&p, "IMU1_GYR_OFFZ"), ParamValue::F32(-0.01));
        // The other IMU and the magnetometer are untouched.
        assert_eq!(get(&p, "IMU2_ACC_FS"), ParamValue::I32(8));
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::I32(3));
        assert_eq!(reset_device_settings(&mut p, mag::TABLE[0].device), 2);
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::I32(2));
        assert_eq!(get(&p, "MAG_OFFX"), ParamValue::F32(1.5));
        assert!(!p.dirty());
    }

    /// The defaults make a setup for both IMUs: after the boot reset the setups cannot fail.
    #[test]
    fn the_default_values_make_both_setups() {
        use mavlink_device::ParamValue;
        let get = |n: &str| match TABLE.iter().find(|d| d.name == n).unwrap().default {
            ParamValue::I32(v) => v,
            ParamValue::F32(_) => panic!("{n}"),
        };
        assert!(
            imu::imu1_setup(get("IMU1_ACC_FS"), get("IMU1_GYR_FS"), get("IMU1_ODR_HZ")).is_ok()
        );
        assert!(
            imu::imu2_setup(get("IMU2_ACC_FS"), get("IMU2_GYR_FS"), get("IMU2_ODR_HZ")).is_ok()
        );
    }
}
