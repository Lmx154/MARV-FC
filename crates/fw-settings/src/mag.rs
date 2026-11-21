//! The magnetometer's parameters and the pure rule that turns them into the driver's `Config`.
//!
//! Nothing here touches hardware or a clock. The table has two `Configure` parameters (the output
//! data rate and the averaging, validated together by the driver's own `Config::validate`) and the
//! three calibration offsets o; the soft-iron matrix S and the calibration's record are appended at
//! indices 81 to 91. The device task applies m = S (r - o) to every sample in software ([`apply`];
//! `Live`: no validator, no apply).

use bmm350::{Averaging, Config, ConfigError, OutputDataRate};
use fw_core::params::{Apply, Candidate, ParamDef};
use fw_core::rotation::{self, Rotation};
use fw_core::sample::MagExpected;
use mavlink_device::marv::MarvDevice;
use mavlink_device::{ParamName, ParamStore, ParamValue};

const DEVICE: u8 = MarvDevice::MARV_DEVICE_MAG as u8;

/// Output data rate in hertz, an `F32` that is one of [`RATES`] (12.5, 6.25, 3.125 and 1.5625 are
/// exact in `f32`).
pub const ODR_HZ: &str = "MAG_ODR_HZ";
/// Samples averaged per output sample: 1, 2, 4 or 8 (the highest the rate allows is the driver's rule).
pub const AVG: &str = "MAG_AVG";
/// Hard-iron offset of the X axis in microtesla, subtracted from every sample.
pub const OFF_X: &str = "MAG_OFFX";
/// As [`OFF_X`], Y axis.
pub const OFF_Y: &str = "MAG_OFFY";
/// As [`OFF_X`], Z axis.
pub const OFF_Z: &str = "MAG_OFFZ";

/// The external compass's orientation, chip to vehicle: ArduPilot's `COMPASS_ORIENT` numbering
/// (0 to 15, see `fw_core::rotation::Rotation`), applied before the offsets.
pub const ORIENT: &str = "MAG_ORIENT";

/// The number of magnetometer parameters.
pub const COUNT: usize = 5;

/// The magnetometer's parameter table; the two configuration defaults are `Config::new()` as the
/// driver's bench programs ran it on this board (100 Hz, averaging 2). The offsets default to 0.
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(ODR_HZ, DEVICE, 100.0, 1.5625, 400.0, Apply::Configure),
    ParamDef::i32(AVG, DEVICE, 2, 1, 8, Apply::Configure),
    ParamDef::f32(OFF_X, DEVICE, 0.0, -200.0, 200.0, Apply::Live).with_calibration(),
    ParamDef::f32(OFF_Y, DEVICE, 0.0, -200.0, 200.0, Apply::Live).with_calibration(),
    ParamDef::f32(OFF_Z, DEVICE, 0.0, -200.0, 200.0, Apply::Live).with_calibration(),
];

/// The number of orientation parameters, appended after the other tables.
pub const ORIENT_COUNT: usize = 1;

/// `MAG_ORIENT`, appended to the firmware's table at index 77: the rotation from the chip's axes
/// to the vehicle's (body FRD), applied to every sample before `MAG_OFF*` is subtracted; the
/// offsets are in the body frame. Default 10, `ROLL_180_YAW_90`: the BMM350 with +X right, +Y
/// forward, +Z up (the owner's axes). `Live`: read for every sample, as the offsets are.
pub static ORIENT_TABLE: [ParamDef; ORIENT_COUNT] = [ParamDef::i32(
    ORIENT,
    DEVICE,
    10,
    rotation::MIN,
    rotation::MAX,
    Apply::Live,
)];

/// S's diagonal term XX (`F32`, 1.0 = no scaling).
pub const DIAG_X: &str = "MAG_DIAGX";
/// As [`DIAG_X`], YY.
pub const DIAG_Y: &str = "MAG_DIAGY";
/// As [`DIAG_X`], ZZ.
pub const DIAG_Z: &str = "MAG_DIAGZ";
/// S's off-diagonal term XY (= YX).
pub const ODIAG_XY: &str = "MAG_ODIAGXY";
/// As [`ODIAG_XY`], XZ (= ZX).
pub const ODIAG_XZ: &str = "MAG_ODIAGXZ";
/// As [`ODIAG_XY`], YZ (= ZY).
pub const ODIAG_YZ: &str = "MAG_ODIAGYZ";
/// How the calibration in force was made: 0 none or unknown, 1 quick (best effort), 2 guided.
pub const CAL_TYPE: &str = "MAG_CAL_TYPE";
/// The expected field strength in microtesla (World Magnetic Model at the calibration site); 0 = not set.
pub const FLD_UT: &str = "MAG_FLD_UT";
/// The expected inclination in degrees, positive down; meaningful only when `MAG_FLD_UT` > 0.
pub const FLD_INC: &str = "MAG_FLD_INC";
/// The health test's strength tolerance in % of `MAG_FLD_UT` (read for every compass sample, `expected`).
pub const HLT_STR: &str = "MAG_HLT_STR";
/// The health test's inclination tolerance in degrees (read for every compass sample, `expected`).
pub const HLT_INC: &str = "MAG_HLT_INC";

/// `MAG_CAL_TYPE` of the quick (minimum/maximum) calibration.
pub const CAL_TYPE_QUICK: i32 = 1;

/// The number of soft-iron and calibration-record parameters, appended after `GPS_RATE_MS`.
pub const CAL_COUNT: usize = 11;

/// The index of [`CAL_TABLE`]'s first entry (`MAG_DIAGX`) in the firmware's table: the last block.
pub const CAL_OFFSET: usize = crate::COUNT - CAL_COUNT;

/// Appended to the firmware's table at indices 81 to 91 (decision-compass-cal-20261120, section 1).
/// All `Live`; 81 to 89 are calibration results (`param reset-calibration` resets them with
/// `MAG_OFF*`: S the identity, type 0, no expected field), the two health tolerances are not.
pub static CAL_TABLE: [ParamDef; CAL_COUNT] = [
    ParamDef::f32(DIAG_X, DEVICE, 1.0, 0.2, 5.0, Apply::Live).with_calibration(),
    ParamDef::f32(DIAG_Y, DEVICE, 1.0, 0.2, 5.0, Apply::Live).with_calibration(),
    ParamDef::f32(DIAG_Z, DEVICE, 1.0, 0.2, 5.0, Apply::Live).with_calibration(),
    ParamDef::f32(ODIAG_XY, DEVICE, 0.0, -1.0, 1.0, Apply::Live).with_calibration(),
    ParamDef::f32(ODIAG_XZ, DEVICE, 0.0, -1.0, 1.0, Apply::Live).with_calibration(),
    ParamDef::f32(ODIAG_YZ, DEVICE, 0.0, -1.0, 1.0, Apply::Live).with_calibration(),
    ParamDef::i32(CAL_TYPE, DEVICE, 0, 0, 2, Apply::Live).with_calibration(),
    ParamDef::f32(FLD_UT, DEVICE, 0.0, 0.0, 95.0, Apply::Live).with_calibration(),
    ParamDef::f32(FLD_INC, DEVICE, 0.0, -90.0, 90.0, Apply::Live).with_calibration(),
    ParamDef::f32(HLT_STR, DEVICE, 10.0, 1.0, 50.0, Apply::Live),
    ParamDef::f32(HLT_INC, DEVICE, 10.0, 1.0, 45.0, Apply::Live),
];

/// The stored `MAG_ORIENT` as a rotation (a value outside 0 to 15 cannot be stored; none reads as
/// the default, 10).
pub fn orientation(value: Option<i32>) -> Rotation {
    value
        .and_then(Rotation::from_i32)
        .unwrap_or(Rotation::Roll180Yaw90)
}

/// The rates the chip offers, by the value of `MAG_ODR_HZ`.
pub const RATES: [(f32, OutputDataRate); 9] = [
    (400.0, OutputDataRate::Hz400),
    (200.0, OutputDataRate::Hz200),
    (100.0, OutputDataRate::Hz100),
    (50.0, OutputDataRate::Hz50),
    (25.0, OutputDataRate::Hz25),
    (12.5, OutputDataRate::Hz12_5),
    (6.25, OutputDataRate::Hz6_25),
    (3.125, OutputDataRate::Hz3_125),
    (1.5625, OutputDataRate::Hz1_5625),
];

const BAD_RATE: &str = "rate not offered by the chip";
const BAD_AVG: &str = "not 1, 2, 4 or 8";
const TOO_AVERAGED: &str = "averaging too high for the rate";
const NOT_ALLOWED: &str = "combination not allowed by the chip";
const MISSING: &str = "parameter missing";

fn averaging(x: i32) -> Option<Averaging> {
    Some(match x {
        1 => Averaging::Avg1,
        2 => Averaging::Avg2,
        4 => Averaging::Avg4,
        8 => Averaging::Avg8,
        _ => return None,
    })
}

/// The device's whole `Config` from a view of the parameters (`rate` is `MAG_ODR_HZ`, `avg` is
/// `MAG_AVG`), or the reason it is not one the chip offers. Everything else in the `Config` is the
/// driver's default. The rate against the averaging is the driver's own check (`Config::validate`).
pub fn settings_to_config(rate: Option<f32>, avg: Option<i32>) -> Result<Config, &'static str> {
    let rate = rate.ok_or(MISSING)?;
    let odr = RATES
        .iter()
        .find_map(|(hz, code)| (*hz == rate).then_some(*code))
        .ok_or(BAD_RATE)?;
    let avg = averaging(avg.ok_or(MISSING)?).ok_or(BAD_AVG)?;
    let config = Config::new().with_odr(odr).with_averaging(avg);
    match config.validate() {
        Ok(()) => Ok(config),
        Err(ConfigError::AveragingTooHigh { .. }) => Err(TOO_AVERAGED),
        Err(_) => Err(NOT_ALLOWED),
    }
}

/// The store's validator for the magnetometer: the whole candidate must make a `Config`.
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    settings_to_config(candidate.f32(ODR_HZ), candidate.i32(AVG)).map(|_| ())
}

/// The nominal sample period P in microseconds for a rate.
pub fn period_us(odr: OutputDataRate) -> u32 {
    odr.period_us()
}

/// The calibration in force, as stored: the offsets o (`MAG_OFF*`) and the symmetric matrix S, its
/// diagonal (`MAG_DIAGX/Y/Z`) and its off-diagonal terms (`MAG_ODIAGXY/XZ/YZ`), body frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// o, microtesla (X, Y, Z).
    pub offsets_ut: [f32; 3],
    /// S's diagonal (XX, YY, ZZ).
    pub diag: [f32; 3],
    /// S's off-diagonal terms (XY, XZ, YZ).
    pub odiag: [f32; 3],
}

impl Calibration {
    /// The parameters' defaults: offsets 0, S the identity.
    pub const NONE: Self = Self {
        offsets_ut: [0.0; 3],
        diag: [1.0; 3],
        odiag: [0.0; 3],
    };
}

/// m = S (r - o): the field `raw_ut` (r, microtesla, body frame after `MAG_ORIENT`) with the
/// calibration applied. Single precision in this order: d = r - o, then per row
/// m_i = (S_i0 d_0 + S_i1 d_1) + S_i2 d_2 (no fused multiply-add), with
/// S = [[XX, XY, XZ], [XY, YY, YZ], [XZ, YZ, ZZ]].
pub fn apply(raw_ut: [f32; 3], cal: &Calibration) -> [f32; 3] {
    let [rx, ry, rz] = raw_ut;
    let [ox, oy, oz] = cal.offsets_ut;
    let [xx, yy, zz] = cal.diag;
    let [xy, xz, yz] = cal.odiag;
    let (dx, dy, dz) = (rx - ox, ry - oy, rz - oz);
    [
        (xx * dx + xy * dy) + xz * dz,
        (xy * dx + yy * dy) + yz * dz,
        (xz * dx + yz * dy) + zz * dz,
    ]
}

/// `MAG_CAL_TYPE` of the guided calibration, the only one whose expected field is used.
pub const CAL_TYPE_GUIDED: i32 = 2;

/// The index of `MAG_CAL_TYPE` in [`CAL_TABLE`]; it, `MAG_FLD_UT`, `MAG_FLD_INC`, `MAG_HLT_STR` and
/// `MAG_HLT_INC` follow one another (pinned by a test), so [`expected_from_store`] reads them by index.
const EXPECTED_AT: usize = 6;

/// The health test's expected field from the five stored values, `MAG_CAL_TYPE`, `MAG_FLD_UT` (microtesla),
/// `MAG_FLD_INC` (degrees, positive down), `MAG_HLT_STR` (percent) and `MAG_HLT_INC` (degrees); `None` unless the type is 2
/// and the field is above 0 (a value that could not be read is `None` too). The units are converted here, once:
/// degrees to radians, percent to a fraction.
pub fn expected(
    cal_type: Option<i32>,
    field_ut: Option<f32>,
    inclination_deg: Option<f32>,
    tol_strength_pct: Option<f32>,
    tol_inclination_deg: Option<f32>,
) -> Option<MagExpected> {
    let strength_ut = field_ut.filter(|f| *f > 0.0)?;
    if cal_type? != CAL_TYPE_GUIDED {
        return None;
    }
    Some(MagExpected {
        strength_ut,
        inclination_rad: inclination_deg?.to_radians(),
        tol_strength_frac: tol_strength_pct? / 100.0,
        tol_inclination_rad: tol_inclination_deg?.to_radians(),
    })
}

/// [`expected`] from the store, five lookups by index (no name search: this runs inside the store's critical
/// section, for every compass sample).
pub fn expected_from_store<P: ParamStore>(params: &P) -> Option<MagExpected> {
    let at = |k: usize| {
        u16::try_from(CAL_OFFSET + EXPECTED_AT + k)
            .ok()
            .and_then(|i| params.get(i))
    };
    let f = |k: usize| match at(k) {
        Some(ParamValue::F32(v)) => Some(v),
        _ => None,
    };
    let kind = match at(0) {
        Some(ParamValue::I32(v)) => Some(v),
        _ => None,
    };
    expected(kind, f(1), f(2), f(3), f(4))
}

/// The quick calibration's result as one coherent set, written into `params` in one call: `MAG_OFF*`
/// = `offsets_ut`, S = the identity, `MAG_CAL_TYPE` = 1 (best effort), `MAG_FLD_UT` = 0 (no field
/// model). `MAG_FLD_INC` and `MAG_HLT_*` are left as they are. `false` when the store refused a value.
pub fn write_quick_result<P: ParamStore>(params: &mut P, offsets_ut: [f32; 3]) -> bool {
    let [ox, oy, oz] = offsets_ut;
    let set = [
        (OFF_X, ParamValue::F32(ox)),
        (OFF_Y, ParamValue::F32(oy)),
        (OFF_Z, ParamValue::F32(oz)),
        (DIAG_X, ParamValue::F32(1.0)),
        (DIAG_Y, ParamValue::F32(1.0)),
        (DIAG_Z, ParamValue::F32(1.0)),
        (ODIAG_XY, ParamValue::F32(0.0)),
        (ODIAG_XZ, ParamValue::F32(0.0)),
        (ODIAG_YZ, ParamValue::F32(0.0)),
        (CAL_TYPE, ParamValue::I32(CAL_TYPE_QUICK)),
        (FLD_UT, ParamValue::F32(0.0)),
    ];
    let mut all = true;
    for (name, value) in set {
        let index = ParamName::new(name).ok().and_then(|n| params.find(&n));
        let written = index.map(|i| params.set(i, value));
        all &= matches!(written, Some(Ok(_)));
    }
    all
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};
    use std::vec::Vec;

    const AVGS: [i32; 4] = [1, 2, 4, 8];

    fn with(rate: f32, avg: i32) -> Result<Config, &'static str> {
        settings_to_config(Some(rate), Some(avg))
    }

    fn name(n: &str) -> ParamName {
        ParamName::new(n).unwrap()
    }

    #[test]
    fn defaults_are_the_drivers_bench_config() {
        assert_eq!(with(100.0, 2), Ok(Config::new()));
        assert_eq!(
            Config::new(),
            Config::new()
                .with_odr(OutputDataRate::Hz100)
                .with_averaging(Averaging::Avg2)
        );
        assert_eq!(period_us(OutputDataRate::Hz100), 10_000);
        let defaults = settings_to_config(
            match TABLE[0].default {
                ParamValue::F32(v) => Some(v),
                ParamValue::I32(_) => None,
            },
            match TABLE[1].default {
                ParamValue::I32(v) => Some(v),
                ParamValue::F32(_) => None,
            },
        );
        assert_eq!(defaults, Ok(Config::new()));
    }

    #[test]
    fn every_refusal_has_its_text() {
        assert_eq!(with(33.0, 2), Err(BAD_RATE));
        assert_eq!(with(0.0, 2), Err(BAD_RATE));
        assert_eq!(with(12.0, 2), Err(BAD_RATE));
        assert_eq!(with(100.0, 3), Err(BAD_AVG));
        assert_eq!(with(100.0, 0), Err(BAD_AVG));
        assert_eq!(with(400.0, 2), Err(TOO_AVERAGED));
        assert_eq!(with(100.0, 8), Err(TOO_AVERAGED));
        assert_eq!(settings_to_config(None, Some(2)), Err(MISSING));
        assert_eq!(settings_to_config(Some(100.0), None), Err(MISSING));
        for text in [BAD_RATE, BAD_AVG, TOO_AVERAGED, NOT_ALLOWED, MISSING] {
            assert!(
                text.len() + "MAG_ODR_HZ: ".len() <= 50,
                "a refusal text with its parameter name must fit a 50-byte STATUSTEXT: {text}"
            );
        }
    }

    #[test]
    fn every_offered_value_alone_is_accepted_or_refused_only_by_the_averaging_rule() {
        // On top of the default averaging (2) only the fastest rate is refused.
        let refused_rates: Vec<f32> = RATES
            .iter()
            .map(|(hz, _)| *hz)
            .filter(|hz| match with(*hz, 2) {
                Ok(_) => false,
                Err(reason) => {
                    assert_eq!(reason, TOO_AVERAGED, "{hz}");
                    true
                }
            })
            .collect();
        assert_eq!(refused_rates, [400.0]);
        // On top of the default rate (100 Hz, up to 4x) only 8 is refused.
        let refused_avgs: Vec<i32> = AVGS
            .iter()
            .copied()
            .filter(|avg| match with(100.0, *avg) {
                Ok(_) => false,
                Err(reason) => {
                    assert_eq!(reason, TOO_AVERAGED, "{avg}");
                    true
                }
            })
            .collect();
        assert_eq!(refused_avgs, [8]);
        // Every offered rate is accepted with no averaging, and the periods are the rates' inverses.
        for (hz, code) in RATES {
            assert_eq!(with(hz, 1).map(|c| c.odr), Ok(code), "{hz}");
            let p = period_us(code) as f32;
            assert!((p * hz / 1e6 - 1.0).abs() < 0.001, "{hz}");
        }
        // The highest averaging each rate allows is accepted.
        for (hz, code) in RATES {
            let max = match code.max_averaging() {
                Averaging::Avg1 => 1,
                Averaging::Avg2 => 2,
                Averaging::Avg4 => 4,
                Averaging::Avg8 => 8,
            };
            assert!(with(hz, max).is_ok(), "{hz} x{max}");
        }
    }

    /// H2: the default `MAG_ORIENT` maps the BMM350's axes (+X right, +Y forward, +Z up) to FRD:
    /// chip +Y to body +X, chip +X to body +Y, chip +Z to body -Z.
    #[test]
    fn the_default_orientation_turns_the_chip_axes_into_frd() {
        let ParamValue::I32(default) = ORIENT_TABLE[0].default else {
            panic!("MAG_ORIENT is an i32");
        };
        let r = orientation(Some(default));
        assert_eq!(default, 10);
        assert_eq!(r, Rotation::Roll180Yaw90);
        assert_eq!(r.rotate([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0]);
        assert_eq!(r.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_eq!(r.rotate([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0]);
        assert_eq!(orientation(None), r);
    }

    fn offsets(offsets_ut: [f32; 3]) -> Calibration {
        Calibration {
            offsets_ut,
            ..Calibration::NONE
        }
    }

    #[test]
    fn offsets_are_subtracted_per_axis() {
        assert_eq!(
            apply([30.0, -10.0, 40.0], &offsets([10.0, -5.0, 0.0])),
            [20.0, -5.0, 40.0]
        );
        assert_eq!(
            apply([1.5, 2.5, 3.5], &offsets([0.0; 3])),
            [1.5, 2.5, 3.5]
        );
    }

    /// The function this build replaced, kept here as it was: `sample - offset` per axis.
    fn old_apply_offsets(field_ut: [f32; 3], offsets_ut: [f32; 3]) -> [f32; 3] {
        let [x, y, z] = field_ut;
        let [ox, oy, oz] = offsets_ut;
        [x - ox, y - oy, z - oz]
    }

    /// I2: with the default S the output equals the old `r - o`: bit for bit, except that a
    /// difference of -0.0 may come out as +0.0 (the S-row sum adds 0 * d of the other axes); those
    /// are equal in value and compared by value.
    #[test]
    fn with_the_default_matrix_the_output_is_the_old_offset_subtraction() {
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let edges = [0.0f32, -0.0, 1e-30, -1e-30, 199.99, -200.0, 200.0, 0.1, -67.25];
        let mut neg_zero = 0;
        for k in 0..200_000u32 {
            let mut pick = |range: f32| {
                let bits = next();
                if bits % 8 == 0 {
                    edges[(bits >> 8) as usize % edges.len()]
                } else {
                    ((bits >> 40) as f32 / (1u64 << 24) as f32 * 2.0 - 1.0) * range
                }
            };
            let r = [pick(400.0), pick(400.0), pick(400.0)];
            let o = [pick(200.0), pick(200.0), pick(200.0)];
            let old = old_apply_offsets(r, o);
            let new = apply(r, &offsets(o));
            for i in 0..3 {
                if old[i].to_bits() == (-0.0f32).to_bits() {
                    neg_zero += 1;
                    assert!(new[i] == old[i], "{k}: {r:?} {o:?}");
                } else {
                    assert_eq!(new[i].to_bits(), old[i].to_bits(), "{k}: {r:?} {o:?}");
                }
            }
        }
        assert!(neg_zero > 0, "the -0.0 case was exercised");
        // -0.0 - 0.0 on X with a positive Y difference: +0.0 now, -0.0 before.
        let new = apply([-0.0, 1.0, 0.0], &offsets([0.0; 3]));
        assert_eq!(new[0].to_bits(), 0.0f32.to_bits());
        assert_eq!(old_apply_offsets([-0.0, 1.0, 0.0], [0.0; 3])[0].to_bits(), (-0.0f32).to_bits());
    }

    /// I3: the worked vector of decision-compass-cal-20261120 (shared with the station), bit for bit.
    #[test]
    fn the_worked_vector_bit_for_bit() {
        let cal = Calibration {
            offsets_ut: [-14.2, -4.2, 1.5],
            diag: [1.0625, 0.9375, 0.78125],
            odiag: [0.03125, -0.046875, 0.0625],
        };
        let r = [20.5f32, -3.25, 38.0];
        let d = [r[0] - cal.offsets_ut[0], r[1] - cal.offsets_ut[1], r[2] - cal.offsets_ut[2]];
        assert_eq!(d.map(f32::to_bits), [0x420A_CCCD, 0x3F73_3330, 0x4212_0000]);
        assert_eq!(
            apply(r, &cal).map(f32::to_bits),
            [0x420C_C000, 0x4088_3333, 0x41D7_9666]
        );
    }

    /// I7: the quick calibration's result writer leaves the coherent set in one call: the
    /// offsets, S the identity, type 1, no expected field; `MAG_FLD_INC` and the health
    /// tolerances as they were.
    #[test]
    fn the_quick_result_is_written_as_one_coherent_set() {
        use crate::{COUNT as ALL, TABLE as FIRMWARE};
        let mut p = Params::<ALL>::new(&FIRMWARE).unwrap();
        let before = [
            (OFF_X, ParamValue::F32(3.0)),
            (DIAG_X, ParamValue::F32(1.0625)),
            (DIAG_Z, ParamValue::F32(0.78125)),
            (ODIAG_XZ, ParamValue::F32(-0.046875)),
            (CAL_TYPE, ParamValue::I32(2)),
            (FLD_UT, ParamValue::F32(48.5)),
            (FLD_INC, ParamValue::F32(58.0)),
            (HLT_STR, ParamValue::F32(15.0)),
            (HLT_INC, ParamValue::F32(12.0)),
        ];
        for (n, v) in before {
            let i = p.find(&name(n)).unwrap();
            assert_eq!(p.set(i, v), Ok(v), "{n}");
        }
        let _ = (p.take_notice(), p.take_configure_mask(), p.take_live_mask());
        assert!(write_quick_result(&mut p, [10.0, -20.5, 30.25]));
        let expected = [
            (OFF_X, ParamValue::F32(10.0)),
            (OFF_Y, ParamValue::F32(-20.5)),
            (OFF_Z, ParamValue::F32(30.25)),
            (DIAG_X, ParamValue::F32(1.0)),
            (DIAG_Y, ParamValue::F32(1.0)),
            (DIAG_Z, ParamValue::F32(1.0)),
            (ODIAG_XY, ParamValue::F32(0.0)),
            (ODIAG_XZ, ParamValue::F32(0.0)),
            (ODIAG_YZ, ParamValue::F32(0.0)),
            (CAL_TYPE, ParamValue::I32(1)),
            (FLD_UT, ParamValue::F32(0.0)),
            (FLD_INC, ParamValue::F32(58.0)),
            (HLT_STR, ParamValue::F32(15.0)),
            (HLT_INC, ParamValue::F32(12.0)),
        ];
        for (n, v) in expected {
            assert_eq!(p.get_named(n), Some(v), "{n}");
        }
        assert_eq!(p.take_configure_mask(), 0);
        assert!(p.dirty());
        // An offset the store refuses is reported (the pure check keeps them in range).
        assert!(!write_quick_result(&mut p, [250.0, 0.0, 0.0]));
    }

    /// F1b: the expected field exists only for the guided calibration (type 2) with a field above 0, in the
    /// converted units; a tolerance written to the store shows in the next read.
    #[test]
    fn the_expected_field_is_none_unless_guided_with_a_field() {
        use crate::{COUNT as ALL, TABLE as FIRMWARE};
        // The five values are consecutive in the firmware's table, in the order `expected_from_store` reads.
        for (k, n) in [CAL_TYPE, FLD_UT, FLD_INC, HLT_STR, HLT_INC].into_iter().enumerate() {
            assert_eq!(FIRMWARE[CAL_OFFSET + EXPECTED_AT + k].name, n);
        }
        let mut p = Params::<ALL>::new(&FIRMWARE).unwrap();
        let mut put = |p: &mut Params<ALL>, n: &str, v: ParamValue| {
            let i = p.find(&name(n)).unwrap();
            assert_eq!(p.set(i, v), Ok(v), "{n}");
        };
        // The defaults: type 0, field 0.
        assert_eq!(expected_from_store(&p), None);
        put(&mut p, FLD_UT, ParamValue::F32(48.5));
        put(&mut p, FLD_INC, ParamValue::F32(58.0));
        assert_eq!(expected_from_store(&p), None, "type 0 with a field");
        put(&mut p, CAL_TYPE, ParamValue::I32(1));
        assert_eq!(expected_from_store(&p), None, "type 1 with a field");
        put(&mut p, CAL_TYPE, ParamValue::I32(2));
        put(&mut p, FLD_UT, ParamValue::F32(0.0));
        assert_eq!(expected_from_store(&p), None, "type 2 with field 0");
        put(&mut p, FLD_UT, ParamValue::F32(48.5));
        let e = expected_from_store(&p).unwrap();
        assert_eq!(e.strength_ut, 48.5);
        assert_eq!(e.inclination_rad, 58.0f32.to_radians());
        assert_eq!(e.tol_strength_frac, 10.0f32 / 100.0);
        assert_eq!(e.tol_inclination_rad, 10.0f32.to_radians());
        // A tolerance written now is in the next sample's value; so is a negative inclination.
        put(&mut p, HLT_STR, ParamValue::F32(25.0));
        put(&mut p, HLT_INC, ParamValue::F32(5.0));
        put(&mut p, FLD_INC, ParamValue::F32(-12.5));
        let e = expected_from_store(&p).unwrap();
        assert_eq!(e.tol_strength_frac, 0.25);
        assert_eq!(e.tol_inclination_rad, 5.0f32.to_radians());
        assert_eq!(e.inclination_rad, (-12.5f32).to_radians());
        // The pure rule, and a value that could not be read.
        assert_eq!(expected(Some(2), Some(0.0), Some(1.0), Some(10.0), Some(10.0)), None);
        assert_eq!(expected(Some(3), Some(50.0), Some(1.0), Some(10.0), Some(10.0)), None);
        assert_eq!(expected(None, Some(50.0), Some(1.0), Some(10.0), Some(10.0)), None);
        assert_eq!(expected(Some(2), Some(50.0), Some(1.0), None, Some(10.0)), None);
    }

    /// I4: the quick calibration measures the raw field r (what the adapter hands it beside m),
    /// so its offsets are the same whatever S and o are in force; undoing only the offsets of a
    /// published m (the old path) would not be, once S is not the identity.
    #[test]
    fn the_quick_calibration_gives_the_same_offsets_whatever_calibration_is_in_force() {
        use fw_core::calib::{MagCal, MagOutcome};
        const CENTRE: [f32; 3] = [10.0, -20.0, 30.0];
        let raw = |i: u32| {
            let z = 1.0 - 2.0 * (i as f32 + 0.5) / 3_000.0;
            let r = (1.0 - z * z).sqrt();
            let a = i as f32 * 2.399_963;
            [
                CENTRE[0] + 50.0 * r * a.cos(),
                CENTRE[1] + 50.0 * r * a.sin(),
                CENTRE[2] + 50.0 * z,
            ]
        };
        let run = |fed: &dyn Fn([f32; 3]) -> [f32; 3]| {
            let mut cal = MagCal::new();
            for i in 0..3_100u32 {
                cal.push(i, 1_000_000 + i as u64 * 10_000, fed(raw(i)));
            }
            match cal.outcome() {
                MagOutcome::Done(fit) => fit.offset_ut,
                other => panic!("{other:?}"),
            }
        };
        let in_force = [
            Calibration::NONE,
            offsets([3.0, -4.0, 5.0]),
            Calibration {
                offsets_ut: [-14.2, -4.2, 1.5],
                diag: [1.0625, 0.9375, 0.78125],
                odiag: [0.03125, -0.046875, 0.0625],
            },
            Calibration {
                offsets_ut: [40.0, 0.0, -60.0],
                diag: [0.5, 2.0, 1.25],
                odiag: [-0.5, 0.25, 0.75],
            },
        ];
        let reference = run(&|r| r);
        assert!(reference.iter().zip(CENTRE).all(|(a, b)| (a - b).abs() < 0.5));
        for cal in &in_force {
            // The adapter publishes m = apply(r, cal) and hands r to the calibration.
            let fed = |r: [f32; 3]| {
                let _published = apply(r, cal);
                r
            };
            assert_eq!(run(&fed), reference, "{cal:?}");
        }
        // Negative control: m + o with S not the identity is not r, and its offsets differ.
        let cal = in_force[2];
        let undone = run(&|r| {
            let m = apply(r, &cal);
            [
                m[0] + cal.offsets_ut[0],
                m[1] + cal.offsets_ut[1],
                m[2] + cal.offsets_ut[2],
            ]
        });
        assert!(
            undone.iter().zip(reference).any(|(a, b)| (a - b).abs() > 1.0),
            "{undone:?}"
        );
    }

    /// The matrix is symmetric and each term lands where S says: one unit difference per axis.
    #[test]
    fn each_matrix_term_lands_in_its_row_and_column() {
        let cal = Calibration {
            offsets_ut: [0.0; 3],
            diag: [2.0, 3.0, 4.0],
            odiag: [0.5, 0.25, 0.125],
        };
        assert_eq!(apply([1.0, 0.0, 0.0], &cal), [2.0, 0.5, 0.25]);
        assert_eq!(apply([0.0, 1.0, 0.0], &cal), [0.5, 3.0, 0.125]);
        assert_eq!(apply([0.0, 0.0, 1.0], &cal), [0.25, 0.125, 4.0]);
    }

    #[test]
    fn table_is_accepted_and_its_defaults_pass_the_validator() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(params.set_validator(DEVICE, validate));
        assert_eq!(params.count() as usize, COUNT);
        for (i, def) in TABLE.iter().enumerate() {
            assert!(def.name.starts_with("MAG_") && def.name.len() <= 16);
            assert_eq!(def.device, DEVICE);
            let i = u16::try_from(i).unwrap();
            assert_eq!(params.set(i, def.default), Ok(def.default));
        }
        for def in &TABLE[2..] {
            assert_eq!(def.apply, Apply::Live);
            assert!(def.calibration);
            assert_eq!(def.default, ParamValue::F32(0.0));
        }
        for def in &TABLE[..2] {
            assert_eq!(def.apply, Apply::Configure);
            assert!(!def.calibration);
        }
        assert_eq!(params.take_configure_mask(), 0);
        assert_eq!(params.take_live_mask(), 0);
    }

    #[test]
    fn the_store_refuses_before_storing_and_offsets_are_live() {
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(p.set_validator(DEVICE, validate));
        let avg = p.find(&name(AVG)).unwrap();
        assert_eq!(p.set(avg, ParamValue::I32(3)), Err(ParamError::Rejected));
        assert_eq!(p.take_notice().unwrap().kind.text(), BAD_AVG);
        assert_eq!(p.set(avg, ParamValue::I32(9)), Err(ParamError::OutOfRange));
        assert_eq!(p.set(avg, ParamValue::I32(8)), Err(ParamError::Rejected));
        assert_eq!(p.take_notice().unwrap().kind.text(), TOO_AVERAGED);
        assert_eq!(p.get(avg), Some(ParamValue::I32(2)));
        let odr = p.find(&name(ODR_HZ)).unwrap();
        assert_eq!(p.set(odr, ParamValue::F32(33.0)), Err(ParamError::Rejected));
        assert_eq!(
            p.set(odr, ParamValue::F32(500.0)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(
            p.set(odr, ParamValue::F32(f32::NAN)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(p.set(odr, ParamValue::F32(50.0)), Ok(ParamValue::F32(50.0)));
        assert_eq!(p.take_configure_mask(), 1 << DEVICE);
        assert_eq!(p.set(avg, ParamValue::I32(8)), Ok(ParamValue::I32(8)));
        let off = p.find(&name(OFF_X)).unwrap();
        assert_eq!(p.set(off, ParamValue::F32(10.0)), Ok(ParamValue::F32(10.0)));
        assert_eq!(p.take_configure_mask(), 1 << DEVICE);
        assert_eq!(p.take_live_mask(), 1 << DEVICE);
        assert_eq!(
            p.set(off, ParamValue::F32(250.0)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(p.reset_calibration(), 1);
        assert_eq!(p.get(off), Some(ParamValue::F32(0.0)));
    }
}
