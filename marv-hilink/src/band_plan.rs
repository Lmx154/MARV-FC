//! IREC 33 cm band plan, Mode C narrowband limits, and the legal preset table.
//!
//! This is pure integer logic over `core` only (no_std, zero-dep) so the exact same
//! definition of "legal RF" is shared by:
//!   - the radio firmware runtime (validates ground-commanded profile changes), and
//!   - the `MARV-RADIO-RP2354A` build script (validates the compile-time default).
//!
//! As an SRAD flight controller we transmit only in the **SRAD** sub-band. We operate
//! **Mode C — narrowband**: occupied bandwidth `<= 100 kHz`, no frequency hopping (a single
//! fixed channel by construction), and amateur power discipline (50 mW default, never more
//! than necessary in the dense IREC RF environment).

use crate::rf::lora_profile;

/// 33 cm sub-band edges (Hz). Only [`SRAD_LOWER_HZ`]`..=`[`SRAD_UPPER_HZ`] is ours to transmit in;
/// the others are listed for reference so the band plan lives in exactly one place.
pub const SRAD_LOWER_HZ: u32 = 902_000_000;
pub const SRAD_UPPER_HZ: u32 = 909_000_000;
pub const PIT_LOWER_HZ: u32 = 909_000_000;
pub const PIT_UPPER_HZ: u32 = 910_000_000;
pub const COTS_GPS_LOWER_HZ: u32 = 910_000_000;
pub const COTS_GPS_UPPER_HZ: u32 = 925_000_000;
pub const FW_NET_LOWER_HZ: u32 = 925_000_000;
pub const FW_NET_UPPER_HZ: u32 = 928_000_000;

/// Mode C narrowband ceiling: occupied bandwidth must not exceed 100 kHz.
pub const MODE_C_MAX_BANDWIDTH_HZ: u32 = 100_000;

/// Transmit-power policy (dBm). The SX1262 high-power PA spans roughly -9..+22 dBm.
/// The default ceiling matches the low end of COTS practice (~50 mW); raising it up to the
/// hardware max requires the explicit [`lora_profile_flags::POWER_OVERRIDE`] flag.
///
/// [`lora_profile_flags::POWER_OVERRIDE`]: crate::rf::lora_profile_flags::POWER_OVERRIDE
pub const MIN_TX_POWER_DBM: i8 = -9;
pub const DEFAULT_MAX_TX_POWER_DBM: i8 = 17;
pub const OVERRIDE_MAX_TX_POWER_DBM: i8 = 22;

/// Concrete modulation parameters behind a [`lora_profile`] preset id. `bandwidth_hz` matches
/// the `lora_modulation::Bandwidth` value table exactly so the firmware can map it back to the
/// typed enum (see `common::comms::links::lora::lora_profile_for_preset`).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PresetSpec {
    pub preset: u8,
    pub spreading_factor: u8,
    pub bandwidth_hz: u32,
    /// Coding-rate denominator `N` for `4/N` (5..=8).
    pub coding_rate_denom: u8,
    pub name: &'static str,
}

/// The legal preset table. Order is incidental — look up by id with [`preset_spec`].
pub const PRESETS: [PresetSpec; lora_profile::COUNT as usize] = [
    PresetSpec {
        preset: lora_profile::FAST,
        spreading_factor: 7,
        bandwidth_hz: 62_500,
        coding_rate_denom: 5,
        name: "FAST",
    },
    PresetSpec {
        preset: lora_profile::BALANCED,
        spreading_factor: 9,
        bandwidth_hz: 62_500,
        coding_rate_denom: 5,
        name: "BALANCED",
    },
    PresetSpec {
        preset: lora_profile::LONG_RANGE,
        spreading_factor: 10,
        bandwidth_hz: 62_500,
        coding_rate_denom: 8,
        name: "LONG_RANGE",
    },
    PresetSpec {
        preset: lora_profile::RECOVERY_BEACON,
        spreading_factor: 12,
        bandwidth_hz: 20_830,
        coding_rate_denom: 8,
        name: "RECOVERY_BEACON",
    },
];

/// Look up the modulation spec for a preset id, or `None` if the id is unknown.
pub const fn preset_spec(preset: u8) -> Option<PresetSpec> {
    let mut i = 0;
    while i < PRESETS.len() {
        if PRESETS[i].preset == preset {
            return Some(PRESETS[i]);
        }
        i += 1;
    }
    None
}

/// Reverse of [`preset_spec`]: find the preset id whose spec matches this modulation, or `None`.
/// Each preset has a distinct (spreading factor, bandwidth) pair so the match is unambiguous —
/// used to report a radio's *active* preset back to the operator from its live profile.
pub const fn preset_for_modulation(spreading_factor: u8, bandwidth_hz: u32) -> Option<u8> {
    let mut i = 0;
    while i < PRESETS.len() {
        if PRESETS[i].spreading_factor == spreading_factor && PRESETS[i].bandwidth_hz == bandwidth_hz
        {
            return Some(PRESETS[i].preset);
        }
        i += 1;
    }
    None
}

/// Sentinel `active_preset` for a profile that matches no known preset (should not occur for
/// firmware-built profiles, but keeps the host readout honest).
pub const UNKNOWN_PRESET: u8 = 0xFF;

/// Why a requested profile was rejected. Carries [`RejectReason::as_str`] for `defmt`-free
/// logging (this crate has no `defmt` dependency).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RejectReason {
    UnknownPreset,
    BandwidthExceedsModeC,
    OutsideSradBand,
    PowerTooLow,
    PowerTooHigh,
}

impl RejectReason {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::UnknownPreset => "unknown preset",
            Self::BandwidthExceedsModeC => "bandwidth exceeds Mode C ceiling",
            Self::OutsideSradBand => "occupied channel leaves the SRAD band",
            Self::PowerTooLow => "tx power below hardware minimum",
            Self::PowerTooHigh => "tx power above policy ceiling",
        }
    }
}

/// Validate a requested `(preset, frequency, power)` against the SRAD band, the Mode C
/// narrowband ceiling, and the power policy. `allow_power_override` (gated upstream by
/// [`lora_profile_flags::POWER_OVERRIDE`]) raises the power ceiling from the dense-RF default
/// to the hardware maximum.
///
/// The whole occupied channel — center `±` bandwidth/2 — must sit inside the SRAD range, not
/// just the center frequency.
///
/// [`lora_profile_flags::POWER_OVERRIDE`]: crate::rf::lora_profile_flags::POWER_OVERRIDE
pub const fn validate(
    preset: u8,
    frequency_hz: u32,
    tx_power_dbm: i8,
    allow_power_override: bool,
) -> Result<(), RejectReason> {
    let spec = match preset_spec(preset) {
        Some(spec) => spec,
        None => return Err(RejectReason::UnknownPreset),
    };

    if spec.bandwidth_hz > MODE_C_MAX_BANDWIDTH_HZ {
        return Err(RejectReason::BandwidthExceedsModeC);
    }

    let half_bw = spec.bandwidth_hz / 2;
    let lower_edge = match frequency_hz.checked_sub(half_bw) {
        Some(edge) => edge,
        None => return Err(RejectReason::OutsideSradBand),
    };
    let upper_edge = match frequency_hz.checked_add(half_bw) {
        Some(edge) => edge,
        None => return Err(RejectReason::OutsideSradBand),
    };
    if lower_edge < SRAD_LOWER_HZ || upper_edge > SRAD_UPPER_HZ {
        return Err(RejectReason::OutsideSradBand);
    }

    if tx_power_dbm < MIN_TX_POWER_DBM {
        return Err(RejectReason::PowerTooLow);
    }
    let max_power = if allow_power_override {
        OVERRIDE_MAX_TX_POWER_DBM
    } else {
        DEFAULT_MAX_TX_POWER_DBM
    };
    if tx_power_dbm > max_power {
        return Err(RejectReason::PowerTooHigh);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rf::lora_profile_flags;

    #[test]
    fn every_preset_is_legal_mode_c_inside_srad_at_default_power() {
        // A center that keeps even the widest preset inside SRAD.
        let center = 902_080_000;
        for spec in PRESETS {
            assert!(spec.bandwidth_hz <= MODE_C_MAX_BANDWIDTH_HZ, "{}", spec.name);
            assert_eq!(
                validate(spec.preset, center, DEFAULT_MAX_TX_POWER_DBM, false),
                Ok(()),
                "{}",
                spec.name
            );
        }
    }

    #[test]
    fn rejects_unknown_preset() {
        assert_eq!(
            validate(lora_profile::COUNT, 902_500_000, 14, false),
            Err(RejectReason::UnknownPreset)
        );
    }

    #[test]
    fn rejects_channel_straddling_the_srad_lower_edge() {
        // Center exactly on the lower edge: the lower half of the channel spills below 902.0.
        assert_eq!(
            validate(lora_profile::FAST, SRAD_LOWER_HZ, 14, false),
            Err(RejectReason::OutsideSradBand)
        );
    }

    #[test]
    fn rejects_channel_straddling_the_srad_upper_edge() {
        assert_eq!(
            validate(lora_profile::FAST, SRAD_UPPER_HZ, 14, false),
            Err(RejectReason::OutsideSradBand)
        );
    }

    #[test]
    fn power_ceiling_depends_on_override_flag() {
        let center = 905_000_000;
        // 20 dBm is above the dense-RF default but within the hardware max.
        assert_eq!(
            validate(lora_profile::FAST, center, 20, false),
            Err(RejectReason::PowerTooHigh)
        );
        assert_eq!(validate(lora_profile::FAST, center, 20, true), Ok(()));
        // Even with override, 23 dBm exceeds the hardware ceiling.
        assert_eq!(
            validate(lora_profile::FAST, center, 23, true),
            Err(RejectReason::PowerTooHigh)
        );
        assert_eq!(
            validate(lora_profile::FAST, center, MIN_TX_POWER_DBM - 1, true),
            Err(RejectReason::PowerTooLow)
        );
    }

    #[test]
    fn override_flag_bit_is_distinct() {
        assert_eq!(
            lora_profile_flags::POWER_OVERRIDE
                & (lora_profile_flags::TEMPORARY
                    | lora_profile_flags::SAVE_DEFAULT
                    | lora_profile_flags::ENTER_RECOVERY_RX_WINDOWS),
            0
        );
    }
}
