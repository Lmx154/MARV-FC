// src/lora/lora_config.rs
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
    pub sf: u8,
    pub bw: u8,
    pub cr: u8,
    pub ldro: bool,

    // Packet
    pub preamble_len: u16,
    pub explicit_header: bool,
    pub crc_on: bool,
    pub invert_iq: bool,
    pub sync_word: u16,

    //LoRa Link Layer
    pub link_ack_timeout_ms: u32,
    pub link_retries: usize,
}

impl LoRaConfig {
    /// EXACT known-good working preset
    pub const fn preset_default() -> Self {
        Self {
            freq_hz: 915_000_000,

            tcxo_enable: true,
            tcxo_voltage: 0x02, // 1.8V known-good
            tcxo_delay_ms: 20,

            use_dcdc: true,
            tx_power: 0,

            sf: 7,
            bw: 0x04,  // BW125
            cr: 0x01,  // CR45
            ldro: false,

            preamble_len: 12,
            explicit_header: true,
            crc_on: true,
            invert_iq: false,

            sync_word: 0x1424, // MAVLink private sync later
            link_ack_timeout_ms: 200,
            link_retries: 4,
        }
    }

    pub fn mod_params(&self) -> [u8; 4] {
        let ldro = if self.ldro { 1 } else { 0 };
        [self.sf, self.bw, self.cr, ldro]
    }

    pub fn pkt_params(&self) -> [u8; 6] {
        let pre = self.preamble_len.to_be_bytes();
        let head = if self.explicit_header { 0 } else { 1 };
        let crc = if self.crc_on { 1 } else { 0 };
        let iq = if self.invert_iq { 1 } else { 0 };
        [pre[0], pre[1], head, 0xFF, crc, iq]
    }

    pub fn pa_config(&self) -> [u8; 4] {
        [0x04, 0x07, 0x00, 0x01]
    }

    pub fn tx_params(&self) -> [u8; 2] {
        let pwr = if self.tx_power < 0 {
            (256 + self.tx_power as i16) as u8
        } else {
            self.tx_power as u8
        };
        [pwr, 0x02] // ramp 40us
    }
}
