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
