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

    // LoRa Link Layer (not used yet, but kept for later)
    pub link_ack_timeout_ms: u32,
    pub link_retries: usize,
}

impl LoRaConfig {
    /// EXACT known-good working preset (mirrors old driver).
    pub const fn preset_default() -> Self {
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

            link_ack_timeout_ms: 200,
            link_retries: 4,
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
}
