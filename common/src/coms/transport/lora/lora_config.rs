// common/src/lora/lora_config.rs
#![allow(dead_code)]

#[derive(Clone, Copy)]
pub struct LoRaConfig {
    pub freq_hz: u32,

    // TCXO
    pub tcxo_enable: bool,
    pub tcxo_voltage: u8,
    pub tcxo_delay_ms: u32,

    // Power
    pub use_dcdc: bool,
    pub tx_power: i8,

    // Modulation
    pub sf: u8,   // 5..12
    pub bw: u8,   // Semtech encoding (0x04 = 125 kHz)
    pub cr: u8,   // 0x01 = 4/5, 0x02 = 4/6, ...
    pub ldro: bool,

    // Packet
    pub preamble_len: u16,
    pub explicit_header: bool,
    pub crc_on: bool,
    pub invert_iq: bool,
    pub sync_word: u16,

    // RF switch wiring: some modules require TX/RX control pins swapped.
    pub rf_switch_swap: bool,
}

impl LoRaConfig {
    /// EXACT known-good working preset.
    pub const fn preset_known_good() -> Self {
        Self {
            freq_hz: 915_000_000,

            // TCXO on DIO3
            tcxo_enable: true,
            tcxo_voltage: 0x02, // 1.8V known-good
            tcxo_delay_ms: 20,

            use_dcdc: true,
            tx_power: 0,        // conservative for lab testing

            // Modulation: SF7 / BW125 / CR4/5 / LDRO off
            sf: 7,
            bw: 0x04,  // BW125 (Semtech encoding)
            cr: 0x01,  // CR 4/5
            ldro: false,

            // Packet: same as legacy driver
            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,

            // Public LoRa sync word 0x3444 (exact legacy setting)
            // You can later change to MAVLink private (0x1424) on BOTH ends.
            sync_word: 0x3444,

            // External RF switch wiring uses swapped TX/RX control lines.
            rf_switch_swap: true,
        }
    }

    /// Backwards-compatible alias for the known-good preset.
    pub const fn preset_default() -> Self {
        Self::preset_known_good()
    }

    /// High-throughput preset for short-range bench testing.
    pub const fn preset_fast() -> Self {
        Self {
            freq_hz: 915_000_000,

            tcxo_enable: true,
            tcxo_voltage: 0x02,
            tcxo_delay_ms: 20,

            use_dcdc: true,
            tx_power: 0,

            // Modulation: SF6 / BW500 / CR4/5 / LDRO off
            sf: 6,
            bw: 0x06,
            cr: 0x01,
            ldro: false,

            // Shorter preamble for lower airtime.
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,

            sync_word: 0x3444,

            rf_switch_swap: true,
        }
    }

    /// Long-range preset for maximum link margin.
    pub const fn preset_long_range() -> Self {
        Self {
            freq_hz: 915_000_000,

            tcxo_enable: true,
            tcxo_voltage: 0x02,
            tcxo_delay_ms: 20,

            use_dcdc: true,
            tx_power: 14,

            // Modulation: SF11 / BW125 / CR4/8 / LDRO on
            sf: 11,
            bw: 0x04,
            cr: 0x04,
            ldro: true,

            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,

            sync_word: 0x3444,

            rf_switch_swap: true,
        }
    }

    /// Modulation parameters (SetModulationParams).
    pub fn mod_params(&self) -> [u8; 4] {
        let ldro = if self.ldro { 1 } else { 0 };
        [self.sf, self.bw, self.cr, ldro]
    }

    /// RX packet parameters: max payload length (0xFF).
    ///
    /// This mirrors the old driver behavior for RX:
    /// - Preamble = `preamble_len`
    /// - Explicit header
    /// - CRC on/off
    /// - IQ inversion on/off
    /// - Payload length = 0xFF (max; actual length comes from explicit header)
    pub fn pkt_params_rx(&self) -> [u8; 6] {
        let pre = self.preamble_len.to_be_bytes();
        let head = if self.explicit_header { 0 } else { 1 };
        let crc = if self.crc_on { 1 } else { 0 };
        let iq = if self.invert_iq { 1 } else { 0 };
        [pre[0], pre[1], head, 0xFF, crc, iq]
    }

    /// TX packet parameters: real payload length.
    ///
    /// This is what the old driver did before every TX.
    /// IMPORTANT: This length directly affects on-air airtime.
    pub fn pkt_params_tx(&self, len: u8) -> [u8; 6] {
        let pre = self.preamble_len.to_be_bytes();
        let head = if self.explicit_header { 0 } else { 1 };
        let crc = if self.crc_on { 1 } else { 0 };
        let iq = if self.invert_iq { 1 } else { 0 };
        [pre[0], pre[1], head, len, crc, iq]
    }

    /// Backwards-compatible: RX-style params.
    pub fn pkt_params(&self) -> [u8; 6] {
        self.pkt_params_rx()
    }

    /// PA config (SetPaConfig).
    pub fn pa_config(&self) -> [u8; 4] {
        // Same as legacy: duty=0x04, hp_max=0x07, device=0x00, pa_lut=0x01
        [0x04, 0x07, 0x00, 0x01]
    }

    /// TX params (SetTxParams).
    pub fn tx_params(&self) -> [u8; 2] {
        let pwr = if self.tx_power < 0 {
            (256 + self.tx_power as i16) as u8
        } else {
            self.tx_power as u8
        };
        // 0x02 = 40us ramp (same as legacy driver)
        [pwr, 0x02]
    }

    /// Compute LoRa time-on-air (microseconds) using Semtech SX126x formula.
    pub fn toa_us(&self, payload_len: usize) -> u64 {
        let bw_hz = match self.bw {
            0x00 => 7_800,
            0x08 => 10_400,
            0x01 => 15_600,
            0x09 => 20_800,
            0x02 => 31_250,
            0x0A => 41_700,
            0x03 => 62_500,
            0x04 => 125_000,
            0x05 => 250_000,
            0x06 => 500_000,
            _ => return 0,
        } as u64;

        let cr = match self.cr {
            0x01 => 1,
            0x02 => 2,
            0x03 => 3,
            0x04 => 4,
            _ => return 0,
        } as i64;

        let sf = self.sf as u32;
        if sf < 5 || sf > 12 || bw_hz == 0 {
            return 0;
        }

        let mut preamble_len = self.preamble_len;
        if self.sf <= 6 && preamble_len < 12 {
            preamble_len = 12;
        }

        let ih = if self.explicit_header { 0i64 } else { 1i64 };
        let crc = if self.crc_on { 1i64 } else { 0i64 };
        let de = if ((self.sf == 11 || self.sf == 12) && self.bw == 0x04)
            || (self.sf == 12 && self.bw == 0x05)
        {
            1i64
        } else {
            0i64
        };

        let payload_len = payload_len.min(255) as i64;
        let sf_i = self.sf as i64;
        let payload_term = 8 * payload_len - 4 * sf_i + 28 + 16 * crc - 20 * ih;
        let denom = 4 * (sf_i - 2 * de);
        let mut payload_symb = 8i64;
        if payload_term > 0 {
            let ceil_div = (payload_term + denom - 1) / denom;
            payload_symb += ceil_div * (cr + 4);
        }

        let tsym_num: u128 = (1_000_000u128) << sf;
        let tsym_den: u128 = bw_hz as u128;
        let preamble_quarters = (preamble_len as u128) * 4 + 17;
        let total_quarters = preamble_quarters + (payload_symb as u128) * 4;
        div_ceil(tsym_num * total_quarters, tsym_den * 4) as u64
    }
}

fn div_ceil(n: u128, d: u128) -> u128 {
    if d == 0 {
        return 0;
    }
    (n + d - 1) / d
}

#[cfg(test)]
mod tests {
    use super::LoRaConfig;

    #[test]
    fn toa_matches_sf7_example() {
        let cfg = LoRaConfig {
            sf: 7,
            bw: 0x04,
            cr: 0x01,
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            ..LoRaConfig::preset_known_good()
        };
        assert_eq!(cfg.toa_us(12), 41_216);
    }

    #[test]
    fn toa_matches_sf10_example() {
        let cfg = LoRaConfig {
            sf: 10,
            bw: 0x04,
            cr: 0x01,
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            ..LoRaConfig::preset_known_good()
        };
        assert_eq!(cfg.toa_us(12), 288_768);
    }

    #[test]
    fn toa_matches_sf12_ldro_example() {
        let cfg = LoRaConfig {
            sf: 12,
            bw: 0x04,
            cr: 0x01,
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            ..LoRaConfig::preset_known_good()
        };
        assert_eq!(cfg.toa_us(12), 1_155_072);
    }
}
