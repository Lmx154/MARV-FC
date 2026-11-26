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
pub mod esc;
pub mod flight;
pub mod gnss;
pub mod imu;
pub mod led;
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
    + imu::SETTINGS_COUNT
    + imu::BOARD_COUNT
    + mag::ORIENT_COUNT
    + esc::COUNT
    + led::COUNT
    + gnss::COUNT
    + mag::CAL_COUNT
    + gnss::SET_COUNT
    + flight::COUNT
    + batt::FS_COUNT;

/// The firmware's one parameter table: every device's table in the order BARO, MAG, BATT, IMU, SD, RC, then the IMUs' chip settings, `AHRS_ORIENTATION`, `MAG_ORIENT`, `MOT_POLES`, `LED_BRIGHT`, `GPS_RATE_MS` and the compass's soft-iron matrix and calibration record (`mag::CAL_TABLE`), `GPS_GNSS`, `PILOT_SPEED_XY`, `GDN_RADIUS`, the 14 `ATC_*` gains, `GDN_ALT_MIN` and `GDN_ALT_MAX`, then the five low-battery monitor parameters `BATT_CELLS` to `BATT_FS_CRT_ACT` (appended, so the first 81 indices did not change).
pub static TABLE: [ParamDef; COUNT] = concat();

/// A device whose stored settings the boot refused: its `Configure`- and `Reboot`-class parameters (settings) go back to their defaults in RAM; its `Live` parameters (the calibration offsets) are kept.
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
    i = 0;
    while i < imu::BOARD_COUNT {
        out[n] = imu::BOARD[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < mag::ORIENT_COUNT {
        out[n] = mag::ORIENT_TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < esc::COUNT {
        out[n] = esc::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < led::COUNT {
        out[n] = led::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < gnss::COUNT {
        out[n] = gnss::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < mag::CAL_COUNT {
        out[n] = mag::CAL_TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < gnss::SET_COUNT {
        out[n] = gnss::SET_TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < flight::COUNT {
        out[n] = flight::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < batt::FS_COUNT {
        out[n] = batt::FS_TABLE[i];
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

    /// Names unique and at most 16 characters, the count the table's length, every default of its range's type and
    /// within it.
    #[test]
    fn the_table_is_sane() {
        use mavlink_device::ParamValue::{F32, I32};
        assert_eq!(TABLE.len(), COUNT);
        assert_eq!(Params::<COUNT>::new(&TABLE).unwrap().count() as usize, TABLE.len());
        for (i, def) in TABLE.iter().enumerate() {
            assert!(!def.name.is_empty() && def.name.len() <= 16, "{}", def.name);
            assert!(TABLE[..i].iter().all(|d| d.name != def.name), "{} twice", def.name);
            match (def.default, def.min, def.max) {
                (I32(v), I32(lo), I32(hi)) => assert!(lo <= v && v <= hi, "{}", def.name),
                (F32(v), F32(lo), F32(hi)) => assert!(lo <= v && v <= hi, "{}", def.name),
                _ => panic!("{}: mixed types", def.name),
            }
        }
    }

    #[test]
    fn the_whole_table_is_accepted_with_every_validator() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(params.set_validator(baro::TABLE[0].device, baro::validate));
        assert!(params.set_validator(mag::TABLE[0].device, mag::validate));
        assert_eq!(params.count() as usize, COUNT);
        let mut expected = baro::TABLE
            .iter()
            .chain(mag::TABLE.iter())
            .chain(batt::TABLE.iter())
            .chain(imu::TABLE.iter())
            .chain(sd::TABLE.iter())
            .chain(rc::TABLE.iter())
            .chain(imu::SETTINGS.iter())
            .chain(imu::BOARD.iter())
            .chain(mag::ORIENT_TABLE.iter())
            .chain(esc::TABLE.iter())
            .chain(led::TABLE.iter())
            .chain(gnss::TABLE.iter())
            .chain(mag::CAL_TABLE.iter())
            .chain(gnss::SET_TABLE.iter())
            .chain(flight::TABLE.iter())
            .chain(batt::FS_TABLE.iter());
        for def in &TABLE {
            assert_eq!(def.name, expected.next().unwrap().name);
        }
        assert!(expected.next().is_none());
    }

    /// "calibration to defaults" gives a coherent "not calibrated": 81 to 89 and `MAG_OFF*` at their defaults (S the identity, offsets 0, type 0, no expected field); the two health tolerances and `MAG_ORIENT` are not calibration results and keep their values.
    #[test]
    fn reset_calibration_returns_the_compass_calibration_to_not_calibrated() {
        use mavlink_device::ParamValue::{self, F32, I32};
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        let changed = [
            ("MAG_OFFX", F32(-14.2)),
            ("MAG_OFFY", F32(-4.2)),
            ("MAG_OFFZ", F32(1.5)),
            ("MAG_DIAGX", F32(1.0625)),
            ("MAG_DIAGY", F32(0.9375)),
            ("MAG_DIAGZ", F32(0.78125)),
            ("MAG_ODIAGXY", F32(0.03125)),
            ("MAG_ODIAGXZ", F32(-0.046875)),
            ("MAG_ODIAGYZ", F32(0.0625)),
            ("MAG_CAL_TYPE", I32(2)),
            ("MAG_FLD_UT", F32(48.5)),
            ("MAG_FLD_INC", F32(58.0)),
            ("MAG_HLT_STR", F32(15.0)),
            ("MAG_HLT_INC", F32(15.0)),
            ("MAG_ORIENT", I32(3)),
        ];
        for (n, v) in changed {
            let i = p.find(&ParamName::new(n).unwrap()).unwrap();
            assert_eq!(p.set(i, v), Ok(v), "{n}");
        }
        p.reset_calibration();
        let default = |n: &str| TABLE.iter().find(|d| d.name == n).unwrap().default;
        for (n, v) in changed {
            let reset = TABLE.iter().find(|d| d.name == n).unwrap().calibration;
            let want: ParamValue = if reset { default(n) } else { v };
            assert_eq!(p.get_named(n), Some(want), "{n}");
        }
        for def in &TABLE[81..=89] {
            assert_eq!(p.get_named(def.name), Some(def.default), "{}", def.name);
        }
        assert_eq!(p.get_named("MAG_HLT_STR"), Some(F32(15.0)));
        assert_eq!(p.get_named("MAG_ORIENT"), Some(I32(3)));
    }

    /// A refused IMU setup puts `AHRS_ORIENTATION` back to nothing: it is device 0, never reset; a refused magnetometer keeps `MAG_ORIENT` (`Live`).
    #[test]
    fn a_refused_device_keeps_the_orientation_parameters() {
        use mavlink_device::ParamValue;
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        assert_eq!(
            p.load(
                &ParamName::new("AHRS_ORIENTATION").unwrap(),
                ParamValue::I32(2)
            ),
            Ok(true)
        );
        assert_eq!(
            p.load(&ParamName::new("MAG_ORIENT").unwrap(), ParamValue::I32(3)),
            Ok(true)
        );
        reset_device_settings(&mut p, imu::DEVICES[0]);
        reset_device_settings(&mut p, imu::DEVICES[1]);
        reset_device_settings(&mut p, mag::TABLE[0].device);
        assert_eq!(p.get_named("AHRS_ORIENTATION"), Some(ParamValue::I32(2)));
        assert_eq!(p.get_named("MAG_ORIENT"), Some(ParamValue::I32(3)));
    }

    /// The start burst of a log (the header, a PARM for every parameter, the boot texts that wait for the log's text cursor: the ring's depth and its sticky slots) is at most 16 KiB, also with the writer's alignment filler (under one 4 KiB piece) in the ring before it.
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

    /// The narrowed boot reset: a refused IMU setting resets the IMU's settings, not its calibration offsets; the barometer's and the magnetometer's likewise (`MAG_OFF*` kept).
    #[test]
    fn a_refused_device_resets_its_settings_but_not_its_calibration() {
        use mavlink_device::ParamValue;
        let name = |n: &str| ParamName::new(n).unwrap();
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
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
        assert_eq!(get(&p, "IMU2_ACC_FS"), ParamValue::I32(8));
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::I32(3));
        assert_eq!(reset_device_settings(&mut p, mag::TABLE[0].device), 2);
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::I32(2));
        assert_eq!(get(&p, "MAG_OFFX"), ParamValue::F32(1.5));
        assert!(!p.dirty());
    }

    /// A record with only `MAG_OFF*` non-default loads by the boot rule with S the identity, `MAG_CAL_TYPE` 0 and no expected field; the output is then the plain offset subtraction.
    #[test]
    fn an_offsets_only_record_loads_with_the_identity_matrix_and_type_0() {
        use fw_core::record::{self, Boot, Entry, SECTOR_LEN};
        use mavlink_device::ParamValue::{self, F32};
        let stored = [("MAG_OFFX", -14.2f32), ("MAG_OFFY", -4.2), ("MAG_OFFZ", 1.5)];
        let entries = stored.iter().map(|(n, v)| Entry {
            name: ParamName::new(n).unwrap(),
            value: F32(*v),
        });
        let mut a = [0u8; SECTOR_LEN];
        let b = [0xFFu8; SECTOR_LEN];
        record::encode(3, entries, &mut a).unwrap();
        let Boot::Current { record, .. } = record::choose(&a, &b).boot else {
            panic!("the record is not current");
        };
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        p.reset_all();
        for entry in record.entries() {
            assert_eq!(p.load(&entry.name, entry.value), Ok(true));
        }
        let f = |n: &str| match p.get_named(n) {
            Some(F32(v)) => v,
            other => panic!("{n}: {other:?}"),
        };
        for n in ["MAG_DIAGX", "MAG_DIAGY", "MAG_DIAGZ"] {
            assert_eq!(f(n), 1.0, "{n}");
        }
        for n in ["MAG_ODIAGXY", "MAG_ODIAGXZ", "MAG_ODIAGYZ", "MAG_FLD_UT", "MAG_FLD_INC"] {
            assert_eq!(f(n), 0.0, "{n}");
        }
        assert_eq!(p.get_named("MAG_CAL_TYPE"), Some(ParamValue::I32(0)));
        let cal = mag::Calibration {
            offsets_ut: [f("MAG_OFFX"), f("MAG_OFFY"), f("MAG_OFFZ")],
            diag: [f("MAG_DIAGX"), f("MAG_DIAGY"), f("MAG_DIAGZ")],
            odiag: [f("MAG_ODIAGXY"), f("MAG_ODIAGXZ"), f("MAG_ODIAGYZ")],
        };
        assert_eq!(cal.offsets_ut, [-14.2, -4.2, 1.5]);
        let r = [20.5f32, -3.25, 38.0];
        assert_eq!(
            mag::apply(r, &cal).map(f32::to_bits),
            [r[0] + 14.2, r[1] + 4.2, r[2] - 1.5].map(f32::to_bits)
        );
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
