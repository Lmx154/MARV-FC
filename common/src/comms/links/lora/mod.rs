//! LoRa physical link profiles.

pub mod frame;
pub mod state;
pub mod stats;
pub mod timing;

pub use lora_modulation::{Bandwidth, BaseBandModulationParams, CodingRate, SpreadingFactor};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcxoVoltage {
    V1_6,
    V1_7,
    V1_8,
    V2_2,
    V2_4,
    V2_7,
    V3_0,
    V3_3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TcxoConfig {
    pub voltage: TcxoVoltage,
    pub startup_delay_ms: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoraProfile {
    pub frequency_hz: u32,
    pub tcxo: Option<TcxoConfig>,
    pub use_dcdc: bool,
    pub tx_power_dbm: i8,
    pub modulation: BaseBandModulationParams,
    pub preamble_len: u16,
    pub explicit_header: bool,
    pub crc_on: bool,
    pub invert_iq: bool,
    pub sync_word: u8,
}

impl LoraProfile {
    pub const fn known_good_915() -> Self {
        Self {
            frequency_hz: 915_000_000,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            tx_power_dbm: 0,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_7,
                Bandwidth::_125KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn fast_915() -> Self {
        Self {
            frequency_hz: 915_000_000,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            tx_power_dbm: 22,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_7,
                Bandwidth::_500KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn robust_915() -> Self {
        Self {
            frequency_hz: 915_000_000,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            tx_power_dbm: 22,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_8,
                Bandwidth::_500KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn fallback_915() -> Self {
        Self {
            frequency_hz: 915_000_000,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            tx_power_dbm: 22,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_8,
                Bandwidth::_250KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn long_range_915() -> Self {
        Self {
            frequency_hz: 915_000_000,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            tx_power_dbm: 22,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_10,
                Bandwidth::_125KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn irec_33cm_mode_c(frequency_hz: u32) -> Self {
        Self {
            frequency_hz,
            tcxo: Some(TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                startup_delay_ms: 20,
            }),
            use_dcdc: true,
            // 17 dBm is about 50 mW, matching the low end of the competition's
            // recommended dense-RF practice while leaving room to raise it if approved.
            tx_power_dbm: 17,
            modulation: BaseBandModulationParams::new(
                SpreadingFactor::_7,
                Bandwidth::_62KHz,
                CodingRate::_4_5,
            ),
            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            sync_word: 0x12,
        }
    }

    pub const fn time_on_air_us(&self, payload_len: u8) -> u32 {
        self.modulation.time_on_air_us(
            Some(self.preamble_len as u8),
            self.explicit_header,
            payload_len,
        )
    }
}

/// Build a runtime [`LoraProfile`] from a band-plan preset id plus an operator-chosen frequency
/// and transmit power. Returns `None` if the preset id is unknown or its spec cannot be mapped to
/// the typed modulation enums.
///
/// This does NOT enforce the band plan — call [`band_plan::validate`] first. All other PHY
/// settings (TCXO, DCDC, preamble, CRC, sync word, …) inherit the Mode C baseline from
/// [`LoraProfile::irec_33cm_mode_c`], so only modulation, frequency, and power vary by preset.
///
/// [`band_plan::validate`]: crate::protocol::hilink::band_plan::validate
pub fn lora_profile_for_preset(
    preset: u8,
    frequency_hz: u32,
    tx_power_dbm: i8,
) -> Option<LoraProfile> {
    let spec = crate::protocol::hilink::band_plan::preset_spec(preset)?;
    let modulation = BaseBandModulationParams::new(
        spreading_factor_from_u8(spec.spreading_factor)?,
        bandwidth_from_hz(spec.bandwidth_hz)?,
        coding_rate_from_denom(spec.coding_rate_denom)?,
    );

    let mut profile = LoraProfile::irec_33cm_mode_c(frequency_hz);
    profile.modulation = modulation;
    profile.tx_power_dbm = tx_power_dbm;
    Some(profile)
}

const fn spreading_factor_from_u8(sf: u8) -> Option<SpreadingFactor> {
    Some(match sf {
        5 => SpreadingFactor::_5,
        6 => SpreadingFactor::_6,
        7 => SpreadingFactor::_7,
        8 => SpreadingFactor::_8,
        9 => SpreadingFactor::_9,
        10 => SpreadingFactor::_10,
        11 => SpreadingFactor::_11,
        12 => SpreadingFactor::_12,
        _ => return None,
    })
}

/// Maps the band plan's integer bandwidth to the typed enum. The `_hz` values match
/// `lora_modulation::Bandwidth::value()` exactly (see `band_plan::PRESETS`).
const fn bandwidth_from_hz(bandwidth_hz: u32) -> Option<Bandwidth> {
    Some(match bandwidth_hz {
        7_810 => Bandwidth::_7KHz,
        10_420 => Bandwidth::_10KHz,
        15_630 => Bandwidth::_15KHz,
        20_830 => Bandwidth::_20KHz,
        31_250 => Bandwidth::_31KHz,
        41_670 => Bandwidth::_41KHz,
        62_500 => Bandwidth::_62KHz,
        125_000 => Bandwidth::_125KHz,
        250_000 => Bandwidth::_250KHz,
        500_000 => Bandwidth::_500KHz,
        _ => return None,
    })
}

const fn coding_rate_from_denom(coding_rate_denom: u8) -> Option<CodingRate> {
    Some(match coding_rate_denom {
        5 => CodingRate::_4_5,
        6 => CodingRate::_4_6,
        7 => CodingRate::_4_7,
        8 => CodingRate::_4_8,
        _ => return None,
    })
}

#[cfg(test)]
mod preset_builder_tests {
    use super::*;
    use crate::protocol::hilink::band_plan;
    use crate::protocol::hilink::rf::lora_profile;

    #[test]
    fn fast_preset_maps_to_sf7_62khz_cr45() {
        let profile = lora_profile_for_preset(lora_profile::FAST, 902_500_000, 14).unwrap();
        assert_eq!(profile.frequency_hz, 902_500_000);
        assert_eq!(profile.tx_power_dbm, 14);
        assert_eq!(profile.modulation.sf, SpreadingFactor::_7);
        assert_eq!(profile.modulation.bw, Bandwidth::_62KHz);
        assert_eq!(profile.modulation.cr, CodingRate::_4_5);
    }

    #[test]
    fn every_band_plan_preset_maps_to_a_profile() {
        for spec in band_plan::PRESETS {
            assert!(
                lora_profile_for_preset(spec.preset, 905_000_000, 10).is_some(),
                "{} must map to a LoraProfile",
                spec.name
            );
        }
    }

    #[test]
    fn unknown_preset_yields_none() {
        assert_eq!(
            lora_profile_for_preset(lora_profile::COUNT, 905_000_000, 10),
            None
        );
    }
}

pub const KNOWN_GOOD_915: LoraProfile = LoraProfile::known_good_915();
pub const FAST_915: LoraProfile = LoraProfile::fast_915();
pub const ROBUST_915: LoraProfile = LoraProfile::robust_915();
pub const FALLBACK_915: LoraProfile = LoraProfile::fallback_915();
pub const LONG_RANGE_915: LoraProfile = LoraProfile::long_range_915();
pub const IREC_33CM_MODE_C_DEFAULT: LoraProfile = LoraProfile::irec_33cm_mode_c(902_080_000);
pub const ACTIVE: LoraProfile = FAST_915;
