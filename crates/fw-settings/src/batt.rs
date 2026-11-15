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
/// / 1e6). NOT calibrated: the default is the crate's bench PLACEHOLDER, 0.020 V/A. 0 reads 0 A.
pub const A_SCALE: &str = "BATT_A_SCALE";
/// Volts at the ADC pin with no current (the crate's `zero_uv` / 1e6), subtracted before the scale.
pub const A_OFFSET: &str = "BATT_A_OFFSET";

/// The number of battery parameters.
pub const COUNT: usize = 3;

/// The battery monitor's parameter table; the defaults are the configuration the `adc-battery`
/// bench program ran on this board (no gain correction, 20 000 uV/A placeholder, zero at 0 V).
pub static TABLE: [ParamDef; COUNT] = [
    ParamDef::f32(V_SCALE, DEVICE, 1.0, 0.1, 10.0, Apply::Live),
    ParamDef::f32(A_SCALE, DEVICE, 0.02, 0.0, 10.0, Apply::Live),
    ParamDef::f32(A_OFFSET, DEVICE, 0.0, -3.3, 3.3, Apply::Live),
];

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
        // 1 701.6 counts mean on the battery input (the bench's 15.08 V reading).
        let adc = AdcConfig::new();
        let pin_uv = adc.pin_microvolts(Reading::from_sum(27_226, 16).unwrap());
        let one = with(V_SCALE, 1.0).voltage.millivolts(pin_uv);
        let two = with(V_SCALE, 2.0).voltage.millivolts(pin_uv);
        assert!((15_000..15_200).contains(&one), "{one}");
        assert!(two.abs_diff(2 * one) <= 1, "{one} {two}");
        // The board's one-point trim of the bench (+7603 ppm) is a scale of 1.007603.
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
