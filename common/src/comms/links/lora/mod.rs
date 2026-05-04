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

    pub const fn time_on_air_us(&self, payload_len: u8) -> u32 {
        self.modulation.time_on_air_us(
            Some(self.preamble_len as u8),
            self.explicit_header,
            payload_len,
        )
    }
}

pub const KNOWN_GOOD_915: LoraProfile = LoraProfile::known_good_915();
pub const FAST_915: LoraProfile = LoraProfile::fast_915();
pub const ROBUST_915: LoraProfile = LoraProfile::robust_915();
pub const FALLBACK_915: LoraProfile = LoraProfile::fallback_915();
pub const LONG_RANGE_915: LoraProfile = LoraProfile::long_range_915();
pub const ACTIVE: LoraProfile = FAST_915;
