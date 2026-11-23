//! The flight code's parameters: the pilot's horizontal speed in position hold (decision-nav-eskf-20261120, Amendment P,
//! P9; the owner's answer 2026-11-22: a parameter, saved to flash) and the RPM notch switch (decision-rpm-notch-20261123).

use fw_core::params::{Apply, ParamDef};

/// The horizontal speed at full roll or pitch stick in position hold, and the largest horizontal velocity setpoint, m/s.
pub const SPEED_XY: &str = "PILOT_SPEED_XY";

/// The RPM notch switch: 1 filters the measured body rates with the rotor-frequency notches, 0 leaves them as they are.
pub const RPM_NOTCH_EN: &str = "RPM_NOTCH_EN";

/// The number of flight parameters.
pub const COUNT: usize = 2;

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
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(SPEED_XY, 0, 5.0, 3.0, 20.0, Apply::Live),
    ParamDef::i32(RPM_NOTCH_EN, 0, 0, 0, 1, Apply::Live),
];

/// The index of [`RPM_NOTCH_EN`] in the firmware's table: the one after `PILOT_SPEED_XY` (93).
pub const RPM_NOTCH_EN_AT: usize = 94;

#[cfg(test)]
mod tests {
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
        assert_eq!(RPM_NOTCH_EN_AT + 1, crate::COUNT);
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
