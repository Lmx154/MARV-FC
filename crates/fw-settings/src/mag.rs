//! The magnetometer's parameters and the pure rule that turns them into the driver's `Config`.
//!
//! Nothing here touches hardware or a clock. The table has two `Configure` parameters (the output
//! data rate and the averaging, validated together by the driver's own `Config::validate`) and the
//! three calibration offsets, which the device task subtracts from every sample in software
//! (`Live`: no validator, no apply).

use bmm350::{Averaging, Config, ConfigError, OutputDataRate};
use fw_core::params::{Apply, Candidate, ParamDef};
use mavlink_device::marv::MarvDevice;

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

/// `sample - offset` per axis, the field in microtesla after the calibration offsets.
pub fn apply_offsets(field_ut: [f32; 3], offsets_ut: [f32; 3]) -> [f32; 3] {
    let [x, y, z] = field_ut;
    let [ox, oy, oz] = offsets_ut;
    [x - ox, y - oy, z - oz]
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

    #[test]
    fn offsets_are_subtracted_per_axis() {
        assert_eq!(
            apply_offsets([30.0, -10.0, 40.0], [10.0, -5.0, 0.0]),
            [20.0, -5.0, 40.0]
        );
        assert_eq!(apply_offsets([1.5, 2.5, 3.5], [0.0; 3]), [1.5, 2.5, 3.5]);
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
