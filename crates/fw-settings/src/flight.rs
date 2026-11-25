//! The flight code's parameters: the pilot's horizontal speed in position hold, the RPM notch switch,
//! the guidance's targets and the 14 gains of the cascaded
//! controller.

use fw_core::params::{Apply, ParamDef};
use fw_flight::tuning::{GAIN_COUNT, GAIN_NAMES, Gains, Tuning};

/// The horizontal speed at full roll or pitch stick in position hold, and the largest horizontal velocity setpoint, m/s.
pub const SPEED_XY: &str = "PILOT_SPEED_XY";

/// The RPM notch switch: 1 filters the measured body rates with the rotor-frequency notches, 0 leaves them as they are.
pub const RPM_NOTCH_EN: &str = "RPM_NOTCH_EN";

/// How many targets the guidance has slots for.
pub const GDN_SLOTS: usize = 4;

/// The number of targets in use, 0 to 4; 0 is off.
pub const GDN_COUNT: &str = "GDN_COUNT";

/// The arrival radius of a target, m.
pub const GDN_RADIUS: &str = "GDN_RADIUS";

/// Latitude of target 1 to 4, 1e-7 degree (south negative), `GDN_LAT1` to `GDN_LAT4`.
pub const GDN_LAT: [&str; GDN_SLOTS] = ["GDN_LAT1", "GDN_LAT2", "GDN_LAT3", "GDN_LAT4"];

/// Longitude of target 1 to 4, 1e-7 degree (west negative), `GDN_LON1` to `GDN_LON4`.
pub const GDN_LON: [&str; GDN_SLOTS] = ["GDN_LON1", "GDN_LON2", "GDN_LON3", "GDN_LON4"];

/// Where the guidance takes its targets from: 0 = the `GDN_*` parameters (default), 1 = the committed mission.
pub const GDN_SRC: &str = "GDN_SRC";

/// The lowest `z` above home, m, of a waypoint or the return item of an uploaded mission (checked at upload only).
pub const GDN_ALT_MIN: &str = "GDN_ALT_MIN";

/// The highest `z` above home, m, of a waypoint or the return item of an uploaded mission (checked at upload only).
pub const GDN_ALT_MAX: &str = "GDN_ALT_MAX";

/// The hands-off switch: 1 lets stick loss in the air hand the vehicle to the hands-off state machine,
/// 0 keeps only PILOT and STOP.
pub const FS_HOLD_EN: &str = "FS_HOLD_EN";

/// The number of flight parameters.
pub const COUNT: usize = 16 + GAIN_COUNT;

/// The index of [`SPEED_XY`] in the firmware's table: the one after `GPS_GNSS` (92).
pub const SPEED_XY_AT: usize = 93;

/// `PILOT_SPEED_XY`, appended to the firmware's table at index 93: `F32`, default 5.0 m/s (ArduPilot POSCONTROL_SPEED_MS), range 3 to 20 m/s (PX4 MPC_VEL_MANUAL's range), `Live`: core 0 pushes the stored value to the flight core, which takes the value in force at the arming step and keeps it until the next disarm (`fw_flight::step::Flight`).
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(SPEED_XY, 0, 5.0, 3.0, 20.0, Apply::Live),
    ParamDef::i32(RPM_NOTCH_EN, 0, 0, 0, 1, Apply::Live),
    ParamDef::i32(GDN_COUNT, 0, 0, 0, GDN_SLOTS as i32, Apply::Live),
    ParamDef::f32(GDN_RADIUS, 0, 2.0, 0.05, 10.0, Apply::Live),
    ParamDef::i32(GDN_LAT[0], 0, 0, -900_000_000, 900_000_000, Apply::Live),
    ParamDef::i32(GDN_LON[0], 0, 0, -1_800_000_000, 1_800_000_000, Apply::Live),
    ParamDef::i32(GDN_LAT[1], 0, 0, -900_000_000, 900_000_000, Apply::Live),
    ParamDef::i32(GDN_LON[1], 0, 0, -1_800_000_000, 1_800_000_000, Apply::Live),
    ParamDef::i32(GDN_LAT[2], 0, 0, -900_000_000, 900_000_000, Apply::Live),
    ParamDef::i32(GDN_LON[2], 0, 0, -1_800_000_000, 1_800_000_000, Apply::Live),
    ParamDef::i32(GDN_LAT[3], 0, 0, -900_000_000, 900_000_000, Apply::Live),
    ParamDef::i32(GDN_LON[3], 0, 0, -1_800_000_000, 1_800_000_000, Apply::Live),
    gain(0),
    gain(1),
    gain(2),
    gain(3),
    gain(4),
    gain(5),
    gain(6),
    gain(7),
    gain(8),
    gain(9),
    gain(10),
    gain(11),
    gain(12),
    gain(13),
    ParamDef::i32(GDN_SRC, 0, 0, 0, 1, Apply::Live),
    ParamDef::f32(GDN_ALT_MIN, 0, 5.0, 3.0, 120.0, Apply::Live),
    ParamDef::f32(GDN_ALT_MAX, 0, 50.0, 3.0, 120.0, Apply::Live),
    ParamDef::i32(FS_HOLD_EN, 0, 0, 0, 1, Apply::Live),
];

/// The index of [`GAIN_NAMES`]`[0]`, `ATC_ANG_RP_P`, in the firmware's table: the one after `GDN_LON4` (104).
pub const GAINS_AT: usize = 105;

/// The index of [`GDN_SRC`] in the firmware's table: the one after the last gain, `ATC_RAT_FLTD` (118).
pub const GDN_SRC_AT: usize = 119;

/// The index of [`GDN_ALT_MIN`] in the firmware's table: the one after `GDN_SRC` (119). `GDN_ALT_MAX` is 121.
pub const GDN_ALT_MIN_AT: usize = 120;

/// The index of [`FS_HOLD_EN`] in the firmware's table: the one after `GDN_ALT_MAX` (121).
pub const FS_HOLD_EN_AT: usize = 122;

/// The definition of gain `i` of `GAIN_NAMES`: `F32`, device 0, `Live`; the default is the field of `Tuning::DEFAULT` and the range is `Gains::MIN` to `Gains::MAX`, so the table has no number of its own.
#[allow(clippy::indexing_slicing)] // `i` is a literal below `GAIN_COUNT`; the table's length is checked by its type
const fn gain(i: usize) -> ParamDef {
    const DEFAULT: Gains = Gains::of(&Tuning::DEFAULT);
    ParamDef::f32(GAIN_NAMES[i], 0, DEFAULT.0[i], Gains::MIN.0[i], Gains::MAX.0[i], Apply::Live)
}

/// The index of [`RPM_NOTCH_EN`] in the firmware's table: the one after `PILOT_SPEED_XY` (93).
pub const RPM_NOTCH_EN_AT: usize = 94;

/// The index of [`GDN_COUNT`] in the firmware's table: the one after `RPM_NOTCH_EN` (94).
pub const GDN_COUNT_AT: usize = 95;

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};

    #[test]
    fn pilot_speed_xy_is_appended_at_index_93_with_its_range_default_and_class() {
        let def = &crate::TABLE[SPEED_XY_AT];
        assert_eq!(def.name, "PILOT_SPEED_XY");
        assert!(def.name.len() <= 16);
        assert_eq!(def.device, 0);
        assert_eq!(def.default, ParamValue::F32(5.0));
        assert_eq!((def.min, def.max), (ParamValue::F32(3.0), ParamValue::F32(20.0)));
        assert_eq!(def.apply, Apply::Live);
        assert!(!def.calibration && !def.read_only);
        assert_eq!(crate::TABLE[SPEED_XY_AT - 1].name, "GPS_GNSS");
        assert_eq!(SPEED_XY_AT + COUNT + crate::batt::FS_COUNT, crate::COUNT);
    }

    #[test]
    fn rpm_notch_en_is_appended_at_index_94_off_by_default_live_and_0_or_1() {
        let def = &crate::TABLE[RPM_NOTCH_EN_AT];
        assert_eq!(def.name, "RPM_NOTCH_EN");
        assert!(def.name.len() <= 16);
        assert_eq!(def.device, 0);
        assert_eq!(def.default, ParamValue::I32(0));
        assert_eq!((def.min, def.max), (ParamValue::I32(0), ParamValue::I32(1)));
        assert_eq!(def.apply, Apply::Live);
        assert!(!def.calibration && !def.read_only);
        assert_eq!(crate::TABLE[RPM_NOTCH_EN_AT - 1].name, "PILOT_SPEED_XY");
        assert_eq!(crate::TABLE[RPM_NOTCH_EN_AT + 1].name, "GDN_COUNT");
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        let i = u16::try_from(RPM_NOTCH_EN_AT).unwrap();
        for ok in [0, 1] {
            assert_eq!(p.set(i, ParamValue::I32(ok)), Ok(ParamValue::I32(ok)));
        }
        for bad in [-1, 2] {
            assert_eq!(p.set(i, ParamValue::I32(bad)), Err(ParamError::OutOfRange), "{bad}");
        }
        assert_eq!(p.get_named(RPM_NOTCH_EN), Some(ParamValue::I32(1)));
    }

    #[test]
    fn the_gdn_parameters_are_appended_at_95_to_104_with_their_types_defaults_ranges_and_class() {
        use ParamValue::{F32, I32};
        assert_eq!(GDN_COUNT_AT, RPM_NOTCH_EN_AT + 1);
        assert_eq!(GDN_COUNT_AT + 10 + GAIN_COUNT + 4 + crate::batt::FS_COUNT, crate::COUNT);
        let mut expected = std::vec![("GDN_COUNT", I32(0), I32(0), I32(4)), ("GDN_RADIUS", F32(2.0), F32(0.05), F32(10.0))];
        for k in 0..GDN_SLOTS {
            expected.push((GDN_LAT[k], I32(0), I32(-900_000_000), I32(900_000_000)));
            expected.push((GDN_LON[k], I32(0), I32(-1_800_000_000), I32(1_800_000_000)));
        }
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        for (k, (name, default, min, max)) in expected.into_iter().enumerate() {
            let at = GDN_COUNT_AT + k;
            let def = &crate::TABLE[at];
            assert_eq!(def.name, name, "index {at}");
            assert!(def.name.len() <= 16);
            assert_eq!((def.device, def.default, def.min, def.max, def.apply), (0, default, min, max, Apply::Live), "{name}");
            assert!(!def.calibration && !def.read_only, "{name}");
            let i = u16::try_from(at).unwrap();
            assert_eq!(p.set(i, min), Ok(min), "{name}");
            assert_eq!(p.set(i, max), Ok(max), "{name}");
            let (below, above) = match (min, max) {
                (I32(lo), I32(hi)) => (I32(lo - 1), I32(hi + 1)),
                _ => (F32(0.049), F32(10.01)),
            };
            assert_eq!(p.set(i, below), Err(ParamError::OutOfRange), "{name}");
            assert_eq!(p.set(i, above), Err(ParamError::OutOfRange), "{name}");
            assert_eq!(p.get_named(name), Some(max), "{name}: a refused value leaves the stored one");
        }
        assert!(p.set(u16::try_from(GDN_COUNT_AT + 1).unwrap(), F32(f32::NAN)).is_err());
    }

    #[test]
    fn gdn_src_is_appended_at_index_119_after_the_gains_i32_0_or_1_default_0_live() {
        use ParamValue::I32;
        let def = &crate::TABLE[GDN_SRC_AT];
        assert_eq!(def.name, "GDN_SRC");
        assert!(def.name.len() <= 16);
        assert_eq!((def.device, def.default, def.min, def.max, def.apply), (0, I32(0), I32(0), I32(1), Apply::Live));
        assert!(!def.calibration && !def.read_only);
        assert_eq!(crate::TABLE[GDN_SRC_AT - 1].name, "ATC_RAT_FLTD");
        assert_eq!(GDN_SRC_AT, 119);
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        let i = u16::try_from(GDN_SRC_AT).unwrap();
        for ok in [0, 1] {
            assert_eq!(p.set(i, I32(ok)), Ok(I32(ok)));
        }
        for bad in [-1, 2] {
            assert_eq!(p.set(i, I32(bad)), Err(ParamError::OutOfRange), "{bad}");
        }
        assert_eq!(p.get_named(GDN_SRC), Some(I32(1)));
        assert!(p.set(i, ParamValue::F32(1.0)).is_err());
    }

    #[test]
    fn gdn_alt_min_and_max_are_appended_at_120_and_121_after_gdn_src_f32_m_live_device_0() {
        use ParamValue::F32;
        assert_eq!(GDN_ALT_MIN_AT, GDN_SRC_AT + 1);
        assert_eq!(crate::TABLE[GDN_SRC_AT].name, "GDN_SRC");
        for (at, name, default) in [(GDN_ALT_MIN_AT, "GDN_ALT_MIN", 5.0), (GDN_ALT_MIN_AT + 1, "GDN_ALT_MAX", 50.0)] {
            let def = &crate::TABLE[at];
            assert_eq!(def.name, name);
            assert!(def.name.len() <= 16);
            assert_eq!(
                (def.device, def.default, def.min, def.max, def.apply),
                (0, F32(default), F32(3.0), F32(120.0), Apply::Live)
            );
            assert!(!def.calibration && !def.read_only);
            let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
            let i = u16::try_from(at).unwrap();
            assert_eq!(p.get_named(name), Some(F32(default)));
            for ok in [3.0, 120.0, 77.5] {
                assert_eq!(p.set(i, F32(ok)), Ok(F32(ok)), "{name} {ok}");
            }
            for bad in [2.999, 120.001, 0.0, -5.0, f32::INFINITY, f32::NEG_INFINITY] {
                assert_eq!(p.set(i, F32(bad)), Err(ParamError::OutOfRange), "{name} {bad}");
            }
            assert!(p.set(i, F32(f32::NAN)).is_err());
            assert_eq!(p.get_named(name), Some(F32(77.5)), "a refused value leaves the stored one");
            assert!(p.set(i, ParamValue::I32(10)).is_err());
        }
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert_eq!(p.set(u16::try_from(GDN_ALT_MIN_AT).unwrap(), F32(100.0)), Ok(F32(100.0)));
        assert_eq!(p.get_named(GDN_ALT_MAX), Some(F32(50.0)));
    }

    #[test]
    fn the_gain_parameters_are_appended_at_105_to_118_in_table_order_f32_device_0_live() {
        assert_eq!(GAINS_AT, GDN_COUNT_AT + 10);
        assert_eq!(GAINS_AT + GAIN_COUNT, GDN_SRC_AT);
        assert_eq!(GDN_SRC_AT + 4 + crate::batt::FS_COUNT, crate::COUNT);
        assert_eq!(crate::COUNT, 128);
        assert_eq!(crate::TABLE[GAINS_AT - 1].name, "GDN_LON4");
        let names = [
            "ATC_ANG_RP_P",
            "ATC_ANG_YAW_P",
            "ATC_RAT_RLL_P",
            "ATC_RAT_PIT_P",
            "ATC_RAT_YAW_P",
            "ATC_RAT_RLL_I",
            "ATC_RAT_PIT_I",
            "ATC_RAT_YAW_I",
            "ATC_RAT_RLL_D",
            "ATC_RAT_PIT_D",
            "ATC_RAT_RLL_IMAX",
            "ATC_RAT_PIT_IMAX",
            "ATC_RAT_YAW_IMAX",
            "ATC_RAT_FLTD",
        ];
        assert_eq!(names, GAIN_NAMES);
        for (k, name) in names.iter().enumerate() {
            let def = &crate::TABLE[GAINS_AT + k];
            assert_eq!(def.name, *name, "index {}", GAINS_AT + k);
            assert!(def.name.len() <= 16, "{name}");
            assert_eq!(def.device, 0, "{name}");
            assert!(matches!(def.default, ParamValue::F32(_)), "{name}");
            assert_eq!(def.apply, Apply::Live, "{name}");
            assert!(!def.calibration && !def.read_only, "{name}");
        }
    }

    /// The defaults are the fields of `Tuning::DEFAULT`, read here by name and not through `Gains`, bit for bit.
    #[test]
    fn the_gain_defaults_are_the_tuning_fields_bit_for_bit() {
        let t = Tuning::DEFAULT;
        let fields = [
            ("ATC_ANG_RP_P", t.atc_ang_rp_p),
            ("ATC_ANG_YAW_P", t.atc_ang_yaw_p),
            ("ATC_RAT_RLL_P", t.atc_rat_p[0]),
            ("ATC_RAT_PIT_P", t.atc_rat_p[1]),
            ("ATC_RAT_YAW_P", t.atc_rat_p[2]),
            ("ATC_RAT_RLL_I", t.atc_rat_i[0]),
            ("ATC_RAT_PIT_I", t.atc_rat_i[1]),
            ("ATC_RAT_YAW_I", t.atc_rat_i[2]),
            ("ATC_RAT_RLL_D", t.atc_rat_d[0]),
            ("ATC_RAT_PIT_D", t.atc_rat_d[1]),
            ("ATC_RAT_RLL_IMAX", t.atc_rat_imax[0]),
            ("ATC_RAT_PIT_IMAX", t.atc_rat_imax[1]),
            ("ATC_RAT_YAW_IMAX", t.atc_rat_imax[2]),
            ("ATC_RAT_FLTD", t.atc_rat_fltd_hz),
        ];
        for (k, (name, field)) in fields.into_iter().enumerate() {
            let def = &crate::TABLE[GAINS_AT + k];
            assert_eq!(def.name, name);
            let ParamValue::F32(d) = def.default else { panic!("{name}") };
            assert_eq!(d.to_bits(), field.to_bits(), "{name}");
            let (ParamValue::F32(lo), ParamValue::F32(hi)) = (def.min, def.max) else { panic!("{name}") };
            assert_eq!(lo.to_bits(), Gains::MIN.0[k].to_bits(), "{name}");
            assert_eq!(hi.to_bits(), Gains::MAX.0[k].to_bits(), "{name}");
        }
    }

    /// The range is the whole check: both ends are accepted, one f32 step outside either is refused, and so are NaN and the infinities; a refused value leaves the stored one.
    #[test]
    fn the_gain_ranges_are_enforced_at_both_ends() {
        for k in 0..GAIN_COUNT {
            let name = GAIN_NAMES[k];
            let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
            let i = u16::try_from(GAINS_AT + k).unwrap();
            let (lo, hi) = (Gains::MIN.0[k], Gains::MAX.0[k]);
            assert_eq!(p.set(i, ParamValue::F32(lo)), Ok(ParamValue::F32(lo)), "{name}");
            assert_eq!(p.set(i, ParamValue::F32(hi)), Ok(ParamValue::F32(hi)), "{name}");
            let below = f32::from_bits(lo.to_bits() - 1);
            let above = f32::from_bits(hi.to_bits() + 1);
            for bad in [below, above, f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.0, -lo] {
                assert_eq!(p.set(i, ParamValue::F32(bad)), Err(ParamError::OutOfRange), "{name} = {bad}");
            }
            assert_eq!(p.get_named(name), Some(ParamValue::F32(hi)), "{name}: a refused value leaves the stored one");
            assert!(p.set(i, ParamValue::I32(1)).is_err(), "{name}");
        }
    }

    /// All 128 parameters stored as non-default values (the worst case of a record) fit one record in one sector.
    #[test]
    fn all_120_parameters_as_non_default_values_fit_one_record_and_one_sector() {
        use fw_core::record::{self, Boot, ENTRY_LEN, Entry, HEADER_LEN, MAX_ENTRIES, SECTOR_LEN};
        assert!(crate::COUNT <= MAX_ENTRIES);
        let entries = crate::TABLE.iter().map(|def| Entry {
            name: mavlink_device::ParamName::new(def.name).unwrap(),
            value: match def.default {
                ParamValue::I32(v) => ParamValue::I32(v + 1),
                ParamValue::F32(v) => ParamValue::F32(v * 1.5 + 1.0),
            },
        });
        let mut a = [0u8; SECTOR_LEN];
        let b = [0xFFu8; SECTOR_LEN];
        let len = record::encode(1, entries, &mut a).unwrap();
        assert_eq!(len, HEADER_LEN + crate::COUNT * ENTRY_LEN + 4);
        assert!(len <= SECTOR_LEN, "{len} B");
        let Boot::Current { record, .. } = record::choose(&a, &b).boot else {
            panic!("the record is not current");
        };
        assert_eq!(record.declared_count(), crate::COUNT);
        assert_eq!(record.entries().count(), crate::COUNT);
    }

    #[test]
    fn fs_hold_en_is_appended_at_index_122_after_gdn_alt_max_i32_0_or_1_default_0_live_device_0() {
        use ParamValue::I32;
        let def = &crate::TABLE[FS_HOLD_EN_AT];
        assert_eq!(def.name, "FS_HOLD_EN");
        assert!(def.name.len() <= 16);
        assert_eq!((def.device, def.default, def.min, def.max, def.apply), (0, I32(0), I32(0), I32(1), Apply::Live));
        assert!(!def.calibration && !def.read_only);
        assert_eq!(crate::TABLE[FS_HOLD_EN_AT - 1].name, "GDN_ALT_MAX");
        assert_eq!(FS_HOLD_EN_AT + 1, crate::batt::FS_AT);
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert_eq!(p.get_named(FS_HOLD_EN), Some(I32(0)));
        let i = u16::try_from(FS_HOLD_EN_AT).unwrap();
        for ok in [0, 1] {
            assert_eq!(p.set(i, I32(ok)), Ok(I32(ok)));
        }
        for bad in [-1, 2] {
            assert_eq!(p.set(i, I32(bad)), Err(ParamError::OutOfRange), "{bad}");
        }
        assert_eq!(p.get_named(FS_HOLD_EN), Some(I32(1)), "a refused value leaves the stored one");
        assert!(p.set(i, ParamValue::F32(1.0)).is_err());
    }

    /// The range is the whole check: the ends are accepted, a hair outside, NaN and infinity are refused.
    #[test]
    fn pilot_speed_xy_range_is_enforced() {
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        let i = u16::try_from(SPEED_XY_AT).unwrap();
        for ok in [3.0_f32, 5.0, 7.5, 20.0] {
            assert_eq!(p.set(i, ParamValue::F32(ok)), Ok(ParamValue::F32(ok)));
        }
        for bad in [2.99, 20.01, 0.0, -5.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(p.set(i, ParamValue::F32(bad)), Err(ParamError::OutOfRange), "{bad}");
        }
        assert_eq!(p.get(i), Some(ParamValue::F32(20.0)), "a refused value leaves the stored one");
        assert_eq!(p.get_named(SPEED_XY), Some(ParamValue::F32(20.0)));
    }
}
