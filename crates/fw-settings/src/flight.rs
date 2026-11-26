//! The flight code's parameters: the pilot's horizontal speed in position hold, the guidance's arrival radius and height limits and the 14 gains of the cascaded
//! controller.

use fw_core::params::{Apply, ParamDef};
use fw_flight::tuning::{GAIN_COUNT, GAIN_NAMES, Gains, Tuning};

/// The horizontal speed at full roll or pitch stick in position hold, and the largest horizontal velocity setpoint, m/s.
pub const SPEED_XY: &str = "PILOT_SPEED_XY";

/// The arrival radius of a target, m.
pub const GDN_RADIUS: &str = "GDN_RADIUS";

/// The lowest `z` above home, m, of a waypoint or the return item of an uploaded mission (checked at upload only).
pub const GDN_ALT_MIN: &str = "GDN_ALT_MIN";

/// The highest `z` above home, m, of a waypoint or the return item of an uploaded mission (checked at upload only).
pub const GDN_ALT_MAX: &str = "GDN_ALT_MAX";

/// The number of flight parameters.
pub const COUNT: usize = 4 + GAIN_COUNT;

/// `PILOT_SPEED_XY`: `F32`, default 5.0 m/s (ArduPilot POSCONTROL_SPEED_MS), range 3 to 20 m/s (PX4 MPC_VEL_MANUAL's range), `Live`: core 0 pushes the stored value to the flight core, which takes the value in force at the arming step and keeps it until the next disarm (`fw_flight::step::Flight`).
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(SPEED_XY, 0, 5.0, 3.0, 20.0, Apply::Live),
    ParamDef::f32(GDN_RADIUS, 0, 2.0, 0.05, 10.0, Apply::Live),
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
    ParamDef::f32(GDN_ALT_MIN, 0, 5.0, 3.0, 120.0, Apply::Live),
    ParamDef::f32(GDN_ALT_MAX, 0, 50.0, 3.0, 120.0, Apply::Live),
];

/// The definition of gain `i` of `GAIN_NAMES`: `F32`, device 0, `Live`; the default is the field of `Tuning::DEFAULT` and the range is `Gains::MIN` to `Gains::MAX`, so the table has no number of its own.
#[allow(clippy::indexing_slicing)] // `i` is a literal below `GAIN_COUNT`; the table's length is checked by its type
const fn gain(i: usize) -> ParamDef {
    const DEFAULT: Gains = Gains::of(&Tuning::DEFAULT);
    ParamDef::f32(GAIN_NAMES[i], 0, DEFAULT.0[i], Gains::MIN.0[i], Gains::MAX.0[i], Apply::Live)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};

    /// The index of `name` in the firmware's table.
    fn at(name: &str) -> u16 {
        u16::try_from(crate::TABLE.iter().position(|d| d.name == name).unwrap()).unwrap()
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
            let def = &crate::TABLE[usize::from(at(name))];
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
            let i = at(name);
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

    /// Every parameter stored as a non-default value (the worst case of a record) fits one record in one sector.
    #[test]
    fn every_parameter_as_a_non_default_value_fits_one_record_and_one_sector() {
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

    /// The range is the whole check: the ends are accepted, a hair outside, NaN and infinity are refused.
    #[test]
    fn pilot_speed_xy_range_is_enforced() {
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        let i = at(SPEED_XY);
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
