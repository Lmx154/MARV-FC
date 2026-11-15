//! The barometer's parameters and the pure rule that turns them into the driver's `Config`.
//!
//! Nothing here touches hardware or a clock: the table, [`settings_to_config`] (the one place that
//! knows which values mean what, used for the check before a write is stored and for the `Config`
//! the device task applies) and [`validate`] (the store's validator).

use bmp581::{Config, ConfigError, IirFilter, OutputDataRate, Oversampling};
use fw_core::params::{Apply, Candidate, ParamDef};
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_BARO as u8;

/// Output data rate in whole hertz, one of [`RATES`].
pub const ODR_HZ: &str = "BARO_ODR_HZ";
/// Pressure oversampling, 1, 2, 4, ..., 128.
pub const OSR_P: &str = "BARO_OSR_P";
/// Temperature oversampling, 1, 2, 4, ..., 128.
pub const OSR_T: &str = "BARO_OSR_T";
/// Pressure IIR filter coefficient: 0 (off), 1, 3, 7, 15, 31, 63 or 127.
pub const IIR_P: &str = "BARO_IIR_P";
/// Temperature IIR filter coefficient, as `BARO_IIR_P`.
pub const IIR_T: &str = "BARO_IIR_T";

/// The number of barometer parameters.
pub const COUNT: usize = 5;

/// The barometer's parameter table; defaults are `Config::new()`: 50 Hz, pressure x16, temperature
/// x2, no filter. Ranges are the driver's extremes; the values in between that the chip does not
/// offer are refused by [`validate`].
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::i32(ODR_HZ, DEVICE, 50, 1, 240, Apply::Configure),
    ParamDef::i32(OSR_P, DEVICE, 16, 1, 128, Apply::Configure),
    ParamDef::i32(OSR_T, DEVICE, 2, 1, 128, Apply::Configure),
    ParamDef::i32(IIR_P, DEVICE, 0, 0, 127, Apply::Configure),
    ParamDef::i32(IIR_T, DEVICE, 0, 0, 127, Apply::Configure),
];

/// The rates the chip offers that have a whole-hertz name: the printed rate and the chip's code
/// (50 Hz is 50.056 Hz, 100 Hz is 100.299 Hz, and so on; the driver's rate list). The sub-hertz
/// codes and 89.6, 110.164, 149.333 Hz have no whole-hertz name and are not offered.
pub const RATES: [(i32, OutputDataRate); 26] = [
    (1, OutputDataRate::Hz1),
    (2, OutputDataRate::Hz2),
    (3, OutputDataRate::Hz3),
    (4, OutputDataRate::Hz4),
    (5, OutputDataRate::Hz5),
    (10, OutputDataRate::Hz10),
    (15, OutputDataRate::Hz15),
    (20, OutputDataRate::Hz20),
    (25, OutputDataRate::Hz25_005),
    (30, OutputDataRate::Hz30),
    (35, OutputDataRate::Hz35),
    (40, OutputDataRate::Hz40),
    (45, OutputDataRate::Hz45_025),
    (50, OutputDataRate::Hz50_056),
    (60, OutputDataRate::Hz60),
    (70, OutputDataRate::Hz70),
    (80, OutputDataRate::Hz80),
    (100, OutputDataRate::Hz100_299),
    (120, OutputDataRate::Hz120),
    (130, OutputDataRate::Hz129_855),
    (140, OutputDataRate::Hz140),
    (160, OutputDataRate::Hz160),
    (180, OutputDataRate::Hz179_2),
    (200, OutputDataRate::Hz199_111),
    (220, OutputDataRate::Hz218_537),
    (240, OutputDataRate::Hz240),
];

const BAD_RATE: &str = "rate not offered by the chip";
const BAD_OSR: &str = "not 1, 2, 4, 8, 16, 32, 64 or 128";
const BAD_IIR: &str = "not 0, 1, 3, 7, 15, 31, 63 or 127";
const TOO_FAST: &str = "rate too high for the oversampling";
const NOT_ALLOWED: &str = "combination not allowed by the chip";
const MISSING: &str = "parameter missing";

fn oversampling(x: i32) -> Option<Oversampling> {
    Some(match x {
        1 => Oversampling::X1,
        2 => Oversampling::X2,
        4 => Oversampling::X4,
        8 => Oversampling::X8,
        16 => Oversampling::X16,
        32 => Oversampling::X32,
        64 => Oversampling::X64,
        128 => Oversampling::X128,
        _ => return None,
    })
}

fn iir(x: i32) -> Option<IirFilter> {
    Some(match x {
        0 => IirFilter::Bypass,
        1 => IirFilter::Coeff1,
        3 => IirFilter::Coeff3,
        7 => IirFilter::Coeff7,
        15 => IirFilter::Coeff15,
        31 => IirFilter::Coeff31,
        63 => IirFilter::Coeff63,
        127 => IirFilter::Coeff127,
        _ => return None,
    })
}

/// The device's whole `Config` from a view of the parameters (`get` returns the `I32` value of a
/// name), or the reason it is not one the chip offers. Everything else in the `Config` is the
/// driver's default. The rate against the oversampling is the driver's own check
/// (`Config::validate`).
pub fn settings_to_config(get: impl Fn(&str) -> Option<i32>) -> Result<Config, &'static str> {
    let odr = get(ODR_HZ).ok_or(MISSING)?;
    let odr = RATES
        .iter()
        .find_map(|(hz, code)| (*hz == odr).then_some(*code))
        .ok_or(BAD_RATE)?;
    let osr_p = oversampling(get(OSR_P).ok_or(MISSING)?).ok_or(BAD_OSR)?;
    let osr_t = oversampling(get(OSR_T).ok_or(MISSING)?).ok_or(BAD_OSR)?;
    let iir_p = iir(get(IIR_P).ok_or(MISSING)?).ok_or(BAD_IIR)?;
    let iir_t = iir(get(IIR_T).ok_or(MISSING)?).ok_or(BAD_IIR)?;
    let config = Config::new()
        .with_odr(odr)
        .with_pressure_oversampling(osr_p)
        .with_temperature_oversampling(osr_t)
        .with_pressure_iir(iir_p)
        .with_temperature_iir(iir_t);
    match config.validate() {
        Ok(()) => Ok(config),
        Err(ConfigError::OdrTooHighForOversampling { .. }) => Err(TOO_FAST),
        Err(_) => Err(NOT_ALLOWED),
    }
}

/// The store's validator for the barometer: the whole candidate must make a `Config`.
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    settings_to_config(|name| candidate.i32(name)).map(|_| ())
}

/// The nominal sample period P in microseconds for a rate.
pub fn period_us(odr: OutputDataRate) -> u32 {
    (1_000_000.0 / odr.hz() + 0.5) as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};
    use std::vec::Vec;

    const OSR: [i32; 8] = [1, 2, 4, 8, 16, 32, 64, 128];
    const IIR: [i32; 8] = [0, 1, 3, 7, 15, 31, 63, 127];
    const EXPECTED_RATES_REFUSED: [i32; 9] = [100, 120, 130, 140, 160, 180, 200, 220, 240];
    const EXPECTED_OSR_P_REFUSED: [i32; 3] = [32, 64, 128];
    const EXPECTED_OSR_T_REFUSED: [i32; 2] = [64, 128];

    /// The configuration of the parameters at their defaults, one of them replaced.
    fn with_one(name: &str, value: i32) -> Result<Config, &'static str> {
        settings_to_config(|n| {
            if n == name {
                Some(value)
            } else {
                TABLE
                    .iter()
                    .find(|d| d.name == n)
                    .and_then(|d| match d.default {
                        ParamValue::I32(v) => Some(v),
                        ParamValue::F32(_) => None,
                    })
            }
        })
    }

    fn with(odr: i32, p: i32, t: i32, ip: i32, it: i32) -> Result<Config, &'static str> {
        settings_to_config(|n| match n {
            ODR_HZ => Some(odr),
            OSR_P => Some(p),
            OSR_T => Some(t),
            IIR_P => Some(ip),
            IIR_T => Some(it),
            _ => None,
        })
    }

    fn name(n: &str) -> ParamName {
        ParamName::new(n).unwrap()
    }

    #[test]
    fn defaults_are_the_drivers_default() {
        assert_eq!(with(50, 16, 2, 0, 0), Ok(Config::new()));
        assert_eq!(period_us(OutputDataRate::Hz50_056), 19_978);
    }

    #[test]
    fn every_refusal_has_its_text() {
        assert_eq!(with(33, 16, 2, 0, 0), Err(BAD_RATE));
        assert_eq!(with(0, 16, 2, 0, 0), Err(BAD_RATE));
        assert_eq!(with(50, 3, 2, 0, 0), Err(BAD_OSR));
        assert_eq!(with(50, 16, 0, 0, 0), Err(BAD_OSR));
        assert_eq!(with(50, 16, 2, 5, 0), Err(BAD_IIR));
        assert_eq!(with(50, 16, 2, 0, 128), Err(BAD_IIR));
        assert_eq!(with(25, 128, 2, 0, 0), Err(TOO_FAST));
        assert_eq!(with(240, 16, 2, 0, 0), Err(TOO_FAST));
        assert!(with(240, 1, 1, 0, 0).is_ok());
        assert_eq!(settings_to_config(|_| None), Err(MISSING));
        assert_eq!(
            [BAD_RATE, BAD_OSR, BAD_IIR, TOO_FAST, NOT_ALLOWED, MISSING]
                .iter()
                .filter(|t| t.len() + "BARO_OSR_P: ".len() <= 50)
                .count(),
            6,
            "a refusal text with its parameter name must fit a 50-byte STATUSTEXT"
        );
    }

    #[test]
    fn every_offered_rate_is_near_its_name_and_all_are_distinct() {
        for (i, (hz, code)) in RATES.iter().enumerate() {
            let hz_f = *hz as f32;
            assert!((code.hz() - hz_f).abs() / hz_f < 0.01, "{hz}");
            assert!(
                RATES
                    .iter()
                    .skip(i + 1)
                    .all(|(_, c)| c.code() != code.code())
            );
            assert!(with(*hz, 1, 1, 0, 0).is_ok(), "{hz} at 1x/1x");
        }
    }

    #[test]
    fn table_is_accepted_and_its_defaults_pass_the_validator() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(params.set_validator(DEVICE, validate));
        assert_eq!(params.count() as usize, COUNT);
        for (i, def) in TABLE.iter().enumerate() {
            assert!(def.name.starts_with("BARO_") && def.name.len() <= 16);
            assert_eq!(def.device, DEVICE);
            assert_eq!(def.apply, Apply::Configure);
            // Writing the default back is a no-op, and the all-defaults candidate is valid.
            let i = u16::try_from(i).unwrap();
            assert_eq!(params.set(i, def.default), Ok(def.default));
        }
        assert_eq!(params.take_configure_mask(), 0);
        let defaults = settings_to_config(|n| match params.get_named(n) {
            Some(ParamValue::I32(v)) => Some(v),
            _ => None,
        });
        assert_eq!(defaults, Ok(Config::new()));
    }

    #[test]
    fn every_offered_value_alone_is_accepted_or_refused_only_by_the_rate_rule() {
        let refused = |name: &str, values: &[i32]| -> Vec<i32> {
            values
                .iter()
                .copied()
                .filter(|v| match with_one(name, *v) {
                    Ok(_) => false,
                    Err(reason) => {
                        assert_eq!(reason, TOO_FAST, "{name} = {v}");
                        true
                    }
                })
                .collect()
        };
        let rates: Vec<i32> = RATES.iter().map(|(hz, _)| *hz).collect();
        // On top of the defaults (pressure 16x, temperature 2x) only the fastest rates are too fast.
        assert_eq!(refused(ODR_HZ, &rates), EXPECTED_RATES_REFUSED);
        // At 50 Hz the largest oversamplings are too slow to keep up.
        assert_eq!(refused(OSR_P, &OSR), EXPECTED_OSR_P_REFUSED);
        assert_eq!(refused(OSR_T, &OSR), EXPECTED_OSR_T_REFUSED);
        // The filters do not take part in the rule.
        assert!(refused(IIR_P, &IIR).is_empty());
        assert!(refused(IIR_T, &IIR).is_empty());
        for v in IIR {
            assert!(with_one(IIR_P, v).is_ok() && with_one(IIR_T, v).is_ok());
        }
    }

    #[test]
    fn the_store_refuses_before_storing() {
        let mut p = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(p.set_validator(DEVICE, validate));
        let osr = p.find(&name(OSR_P)).unwrap();
        assert_eq!(p.set(osr, ParamValue::I32(3)), Err(ParamError::Rejected));
        assert_eq!(p.get(osr), Some(ParamValue::I32(16)));
        assert_eq!(p.take_configure_mask(), 0);
        let notice = p.take_notice().unwrap();
        assert_eq!(notice.kind.text(), BAD_OSR);
        let odr = p.find(&name(ODR_HZ)).unwrap();
        assert_eq!(
            p.set(odr, ParamValue::I32(500)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(p.set(odr, ParamValue::I32(33)), Err(ParamError::Rejected));
        assert_eq!(p.get(odr), Some(ParamValue::I32(50)));
        assert_eq!(p.set(odr, ParamValue::I32(25)), Ok(ParamValue::I32(25)));
        assert_eq!(p.take_configure_mask(), 1 << DEVICE);
        // The candidate is the whole configuration: 128x pressure is too slow at 25 Hz.
        assert_eq!(p.set(osr, ParamValue::I32(128)), Err(ParamError::Rejected));
        assert_eq!(p.take_notice().unwrap().kind.text(), TOO_FAST);
        assert_eq!(p.get(osr), Some(ParamValue::I32(16)));
        assert_eq!(p.reset_all(), 1);
        assert_eq!(p.take_configure_mask(), 1 << DEVICE);
    }
}
