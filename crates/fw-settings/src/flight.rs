//! The flight code's parameters: the pilot's horizontal speed in position hold (decision-nav-eskf-20261120, Amendment P,
//! P9; the owner's answer 2026-11-22: a parameter, saved to flash), the RPM notch switch (decision-rpm-notch-20261123) and
//! the shadow guidance's targets (decision-guidance-shadow-20261123, section 1 item 4).

use fw_core::params::{Apply, ParamDef};

/// The horizontal speed at full roll or pitch stick in position hold, and the largest horizontal velocity setpoint, m/s.
pub const SPEED_XY: &str = "PILOT_SPEED_XY";

/// The RPM notch switch: 1 filters the measured body rates with the rotor-frequency notches, 0 leaves them as they are.
pub const RPM_NOTCH_EN: &str = "RPM_NOTCH_EN";

/// How many targets the shadow guidance has slots for.
pub const GDN_SLOTS: usize = 4;

/// The number of targets in use, 0 to 4; 0 is off.
pub const GDN_COUNT: &str = "GDN_COUNT";

/// The arrival radius of a target, m.
pub const GDN_RADIUS: &str = "GDN_RADIUS";

/// Latitude of target 1 to 4, 1e-7 degree (south negative), `GDN_LAT1` to `GDN_LAT4`.
pub const GDN_LAT: [&str; GDN_SLOTS] = ["GDN_LAT1", "GDN_LAT2", "GDN_LAT3", "GDN_LAT4"];

/// Longitude of target 1 to 4, 1e-7 degree (west negative), `GDN_LON1` to `GDN_LON4`.
pub const GDN_LON: [&str; GDN_SLOTS] = ["GDN_LON1", "GDN_LON2", "GDN_LON3", "GDN_LON4"];

/// The number of flight parameters.
pub const COUNT: usize = 12;

/// The index of [`SPEED_XY`] in the firmware's table: the one after `GPS_GNSS` (92).
pub const SPEED_XY_AT: usize = 93;

/// `PILOT_SPEED_XY`, appended to the firmware's table at index 93: `F32`, default 5.0 m/s (ArduPilot POSCONTROL_SPEED_MS),
/// range 3 to 20 m/s (PX4 MPC_VEL_MANUAL's range), `Live`: core 0 pushes the stored value to the flight core, which takes
/// the value in force at the arming step and keeps it until the next disarm (`fw_flight::step::Flight`). Device 0, the
/// system, like `AHRS_ORIENTATION` and `LED_BRIGHT`: there is no flight device row. The range is the whole validator
/// (`Params::set` refuses a value outside it, so does a load from flash).
///
/// `RPM_NOTCH_EN` follows at index 94: `I32`, 0 or 1, default 0 (off: one change per flight), `Live`: core 0 pushes the
/// stored value to the flight core, which takes it at the arming step like `PILOT_SPEED_XY`. Device 0, the system.
///
/// The shadow guidance's parameters follow, indices 95 to 104, all device 0 and `Live` (core 0 pushes them to the flight
/// core, which takes new targets on a disarmed step): `GDN_COUNT` (`I32`, 0 to 4, default 0 = off), `GDN_RADIUS` (`F32`, m,
/// default 2.0, range 0.05 to 10: ArduPilot WP_RADIUS_M 2.0, PX4 NAV_ACC_RAD 10), then `GDN_LAT1`, `GDN_LON1`, ...,
/// `GDN_LAT4`, `GDN_LON4` (`I32`, 1e-7 degree, default 0; latitude -900000000 to 900000000, longitude -1800000000 to
/// 1800000000). Whole-degree values at 1e-7 degree travel exactly as INT32 parameters.
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
];

/// The index of [`RPM_NOTCH_EN`] in the firmware's table: the one after `PILOT_SPEED_XY` (93).
pub const RPM_NOTCH_EN_AT: usize = 94;

/// The index of [`GDN_COUNT`] in the firmware's table: the one after `RPM_NOTCH_EN` (94). `GDN_RADIUS` follows, then the
/// latitude and longitude of each target in turn.
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
        assert_eq!(SPEED_XY_AT + COUNT, crate::COUNT);
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
        assert_eq!(GDN_COUNT_AT + 10, crate::COUNT);
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
            // The range is the whole check: both ends are accepted, one past either is refused.
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
