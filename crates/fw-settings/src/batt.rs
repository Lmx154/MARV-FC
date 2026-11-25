//! The battery monitor's parameters and the pure rule that turns them into the `adc-battery`
//! crate's configuration.
//!
//! All three are applied in software to the next sample (`Live`): the device task reads them for
//! every sample, there is no chip to configure.

use adc_battery::{AdcConfig, BatteryConfig, CurrentInput, VoltageInput};
use fw_core::params::{Apply, ParamDef};
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_BATT as u8;

/// Gain on the battery voltage, 1.0 = the divider's nominal ratio (100k / 10k) and a 3.3 V
/// reference; the crate's `gain_ppm` is `(scale - 1) * 1e6`. 2.0 doubles the reported voltage.
pub const V_SCALE: &str = "BATT_V_SCALE";
/// Volts at the ADC pin per ampere of the ESC's current output (the crate's `microvolts_per_amp`
/// / 1e6). NOT calibrated: the default is a PLACEHOLDER, 0.020 V/A. 0 reads 0 A.
pub const A_SCALE: &str = "BATT_A_SCALE";
/// Volts at the ADC pin with no current (the crate's `zero_uv` / 1e6), subtracted before the scale.
pub const A_OFFSET: &str = "BATT_A_OFFSET";

/// The number of battery parameters.
pub const COUNT: usize = 3;

/// Cells in series of the pack; 0 turns the low-battery monitor off.
pub const CELLS: &str = "BATT_CELLS";
/// LOW threshold per cell, V.
pub const LOW_CELL: &str = "BATT_LOW_CELL";
/// CRITICAL threshold per cell, V.
pub const CRT_CELL: &str = "BATT_CRT_CELL";
/// Seconds the voltage must stay below a level before it is reached; 0 turns the monitor off.
pub const LOW_TIMER: &str = "BATT_LOW_TIMER";
/// What CRITICAL does: 0 warns, 1 descends (the flight core's descent, off by default).
pub const FS_CRT_ACT: &str = "BATT_FS_CRT_ACT";

/// The number of low-battery monitor parameters.
pub const FS_COUNT: usize = 5;

/// Where [`FS_TABLE`] starts in `fw_settings::TABLE`: appended last, after `FS_HOLD_EN` (122). Pinned by a test.
pub const FS_AT: usize = 123;
/// The battery monitor's parameter table; the defaults are the `adc-battery` crate's
/// configuration (no gain correction, 20 000 uV/A placeholder, zero at 0 V).
/// program ran on this board (no gain correction, 20 000 uV/A placeholder, zero at 0 V).
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(V_SCALE, DEVICE, 1.0, 0.1, 10.0, Apply::Live),
    ParamDef::f32(A_SCALE, DEVICE, 0.02, 0.0, 10.0, Apply::Live),
    ParamDef::f32(A_OFFSET, DEVICE, 0.0, -3.3, 3.3, Apply::Live),
];

/// The low-battery monitor's parameters, appended at indices 123 to 127 after
/// the last existing one, all device BATT and `Live` (core 0 reads them at 10 Hz). `BATT_CELLS` 0 to 12 default 4 (0 is off; PX4 `BAT_N_CELLS`), `BATT_LOW_CELL` 3.0 to 4.2 V
/// default 3.6, `BATT_CRT_CELL` 3.0 to 4.2 V default 3.5, `BATT_LOW_TIMER` 0 to 120 s default 10 (ArduPilot `BATT_LOW_TIMER`)
/// and `BATT_FS_CRT_ACT` 0 to 1 default 0 (warn only). The counts and the action are `I32`, the
/// thresholds `F32`, the timer whole seconds as `I32` (ArduPilot's is an integer too).
pub static FS_TABLE: [ParamDef; FS_COUNT] = [
    ParamDef::i32(CELLS, DEVICE, 4, 0, 12, Apply::Live),
    ParamDef::f32(LOW_CELL, DEVICE, 3.6, 3.0, 4.2, Apply::Live),
    ParamDef::f32(CRT_CELL, DEVICE, 3.5, 3.0, 4.2, Apply::Live),
    ParamDef::i32(LOW_TIMER, DEVICE, 10, 0, 120, Apply::Live),
    ParamDef::i32(FS_CRT_ACT, DEVICE, 0, 0, 1, Apply::Live),
];

/// The monitor's settings from a view of the parameters (`get_i32` and `get_f32` return the value of a name); a value that
/// cannot be read is the table's default.
pub fn monitor_config(
    get_i32: impl Fn(&str) -> Option<i32>,
    get_f32: impl Fn(&str) -> Option<f32>,
) -> fw_core::battery::Config {
    let d = fw_core::battery::Config::DEFAULT;
    fw_core::battery::Config {
        cells: get_i32(CELLS).unwrap_or(d.cells),
        low_cell_v: get_f32(LOW_CELL).unwrap_or(d.low_cell_v),
        crt_cell_v: get_f32(CRT_CELL).unwrap_or(d.crt_cell_v),
        timer_s: get_i32(LOW_TIMER).unwrap_or(d.timer_s),
        act: get_i32(FS_CRT_ACT).unwrap_or(d.act),
    }
}

/// The battery divider on this board: 100k on top, 10k at the bottom.
const R_TOP_OHM: u32 = 100_000;
const R_BOTTOM_OHM: u32 = 10_000;

/// `x` rounded to the nearest integer, halves away from zero (`core` has no `f32::round`); values
/// outside the range of `i32` saturate.
fn round(x: f32) -> i32 {
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        -((-x + 0.5) as i32)
    }
}

/// The crate's whole configuration from a view of the parameters (`get` returns the `F32` value of
/// a name), or the reason a value is missing.
pub fn settings_to_config(
    get: impl Fn(&str) -> Option<f32>,
) -> Result<BatteryConfig, &'static str> {
    let v_scale = get(V_SCALE).ok_or("parameter missing")?;
    let a_scale = get(A_SCALE).ok_or("parameter missing")?;
    let a_offset = get(A_OFFSET).ok_or("parameter missing")?;
    let gain_ppm = round((v_scale - 1.0) * 1_000_000.0);
    let microvolts_per_amp = round(a_scale * 1_000_000.0).max(0) as u32;
    let zero_uv = round(a_offset * 1_000_000.0);
    Ok(BatteryConfig::new()
        .with_adc(AdcConfig::new())
        .with_voltage(VoltageInput::new(R_TOP_OHM, R_BOTTOM_OHM).with_gain_ppm(gain_ppm))
        .with_current(CurrentInput::new(microvolts_per_amp).with_zero_uv(zero_uv)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use adc_battery::Reading;
    use fw_core::params::Params;
    use mavlink_device::{ParamStore, ParamValue};

    fn defaults(n: &str) -> Option<f32> {
        TABLE
            .iter()
            .find(|d| d.name == n)
            .and_then(|d| match d.default {
                ParamValue::F32(v) => Some(v),
                ParamValue::I32(_) => None,
            })
    }

    fn with(name: &str, value: f32) -> BatteryConfig {
        settings_to_config(|n| if n == name { Some(value) } else { defaults(n) }).unwrap()
    }

    #[test]
    fn defaults_are_the_bench_configuration() {
        let bench = BatteryConfig::new()
            .with_adc(AdcConfig::new())
            .with_voltage(VoltageInput::new(100_000, 10_000))
            .with_current(CurrentInput::new(20_000));
        assert_eq!(settings_to_config(defaults), Ok(bench));
    }

    #[test]
    fn a_missing_parameter_is_named_and_nothing_else_is_refused() {
        assert_eq!(settings_to_config(|_| None), Err("parameter missing"));
        assert_eq!(
            settings_to_config(|n| if n == A_OFFSET { None } else { defaults(n) }),
            Err("parameter missing")
        );
    }

    #[test]
    fn doubling_the_voltage_scale_doubles_the_voltage() {
        // 1 701.6 counts mean on the battery input (15.08 V).
        let adc = AdcConfig::new();
        let pin_uv = adc.pin_microvolts(Reading::from_sum(27_226, 16).unwrap());
        let one = with(V_SCALE, 1.0).voltage.millivolts(pin_uv);
        let two = with(V_SCALE, 2.0).voltage.millivolts(pin_uv);
        assert!((15_000..15_200).contains(&one), "{one}");
        assert!(two.abs_diff(2 * one) <= 1, "{one} {two}");
        // A one-point trim of +7603 ppm is a scale of 1.007603.
        assert_eq!(with(V_SCALE, 1.007_603).voltage.gain_ppm, 7_603);
        assert_eq!(with(V_SCALE, 0.5).voltage.gain_ppm, -500_000);
    }

    #[test]
    fn current_scale_and_offset_are_volts() {
        let c = with(A_SCALE, 0.04).current;
        assert_eq!(c.microvolts_per_amp, 40_000);
        assert_eq!(c.milliamps(40_000), 1_000);
        let c = with(A_OFFSET, 1.0).current;
        assert_eq!(c.zero_uv, 1_000_000);
        assert_eq!(c.milliamps(1_020_000), 1_000);
        assert_eq!(c.milliamps(980_000), -1_000);
        assert_eq!(with(A_SCALE, 0.0).current.milliamps(500_000), 0);
        assert_eq!(with(A_OFFSET, -0.5).current.zero_uv, -500_000);
    }

    /// The monitor's five parameters are at 123 to 127 with their defaults and ranges; the range is the whole
    /// validator; the table's defaults are the monitor's `Config::DEFAULT`.
    #[test]
    fn the_monitor_parameters_are_appended_at_123_to_127_with_the_amendments_numbers() {
        use mavlink_device::{ParamError, ParamValue::{F32, I32}};
        assert_eq!(FS_AT, 123);
        let expected = [
            ("BATT_CELLS", I32(4), I32(0), I32(12)),
            ("BATT_LOW_CELL", F32(3.6), F32(3.0), F32(4.2)),
            ("BATT_CRT_CELL", F32(3.5), F32(3.0), F32(4.2)),
            ("BATT_LOW_TIMER", I32(10), I32(0), I32(120)),
            ("BATT_FS_CRT_ACT", I32(0), I32(0), I32(1)),
        ];
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        for (k, (name, default, min, max)) in expected.into_iter().enumerate() {
            let def = &crate::TABLE[FS_AT + k];
            assert_eq!(def.name, name);
            assert_eq!((def.default, def.min, def.max), (default, min, max), "{name}");
            assert_eq!((def.device, def.apply), (DEVICE, Apply::Live), "{name}");
            assert!(!def.calibration && !def.read_only && name.len() <= 16);
            let i = u16::try_from(FS_AT + k).unwrap();
            assert_eq!(p.set(i, min), Ok(min));
            assert_eq!(p.set(i, max), Ok(max));
            let (below, above) = match (min, max) {
                (I32(lo), I32(hi)) => (I32(lo - 1), I32(hi + 1)),
                (F32(lo), F32(hi)) => (F32(lo - 0.01), F32(hi + 0.01)),
                _ => panic!(),
            };
            assert_eq!(p.set(i, below), Err(ParamError::OutOfRange), "{name}");
            assert_eq!(p.set(i, above), Err(ParamError::OutOfRange), "{name}");
        }
        // The defaults of the table are the monitor's.
        let get = |p: &Params<{ crate::COUNT }>| {
            monitor_config(
                |n| match p.get_named(n) { Some(I32(v)) => Some(v), _ => None },
                |n| match p.get_named(n) { Some(F32(v)) => Some(v), _ => None },
            )
        };
        let p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert_eq!(get(&p), fw_core::battery::Config::DEFAULT);
        assert_eq!(monitor_config(|_| None, |_| None), fw_core::battery::Config::DEFAULT);
    }

    #[test]
    fn table_is_accepted_and_every_value_in_its_range_makes_a_configuration() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert_eq!(params.count() as usize, COUNT);
        for (i, def) in TABLE.iter().enumerate() {
            assert!(def.name.starts_with("BATT_") && def.name.len() <= 16);
            assert_eq!(def.device, DEVICE);
            assert_eq!(def.apply, Apply::Live);
            assert!(!def.calibration);
            let i = u16::try_from(i).unwrap();
            assert_eq!(params.set(i, def.default), Ok(def.default));
            for v in [def.min, def.max] {
                let ParamValue::F32(v) = v else { panic!() };
                assert!(
                    settings_to_config(|n| if n == def.name { Some(v) } else { defaults(n) })
                        .is_ok()
                );
            }
        }
        assert_eq!(params.take_live_mask(), 0);
        assert_eq!(params.take_configure_mask(), 0);
        // A write is Live: the live mask of the device, nothing to configure, no validator.
        assert_eq!(
            params.set(0, ParamValue::F32(2.0)),
            Ok(ParamValue::F32(2.0))
        );
        assert_eq!(params.take_live_mask(), 1 << DEVICE);
        assert_eq!(params.take_configure_mask(), 0);
    }
}
