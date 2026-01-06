//! MAC layer schedule/payload config with derived timing helpers.
use super::mac_codec::HEADER_LEN;
use super::rf_config::RfConfig;

// NOTE: These fields are selected from vehicle-specific packet sizes and lane
// targets, so presets must remain per-vehicle even if values match.
#[derive(Clone, Copy, Debug)]
pub struct MacConfig {
    pub tick_hz: u32,
    pub slot_ratio_r: u16,
    pub tx_guard_us: u64,
    pub dl_tx_offset_us: u64,
    pub rx_ready_guard_us: u64,
    pub rx_timeout_symbols: u16,
    pub rx_timeout_auto: bool,
    pub uplink_payload_len: usize,
    pub downlink_payload_len: usize,
}

impl MacConfig {
    pub const fn tick_period_us(self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1_000_000u64 / hz as u64
    }

    pub const fn tick_period_ms(self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1000u64 / hz as u64
    }
}

pub fn rx_timeout_symbols(rf: RfConfig, mac: MacConfig) -> u16 {
    // Not shareable: this helper absorbs per-vehicle packet sizes and schedule
    // policy (payload lengths, tick rate, guards, offsets, readiness window).
    if !mac.rx_timeout_auto {
        return mac.rx_timeout_symbols;
    }

    let symbol_us = lora_symbol_time_us(rf);
    if symbol_us == 0 {
        return mac.rx_timeout_symbols;
    }

    let uplink_len = HEADER_LEN + mac.uplink_payload_len;
    let downlink_len = HEADER_LEN + mac.downlink_payload_len;
    let uplink_toa_us = rf.toa_us(uplink_len);
    let downlink_toa_us = rf.toa_us(downlink_len);
    let tick_period_us = mac.tick_period_us();

    let max_tx_delay_us = tick_period_us
        .saturating_sub(mac.tx_guard_us)
        .saturating_sub(uplink_toa_us);

    let target_rx_us = mac
        .dl_tx_offset_us
        .saturating_add(downlink_toa_us)
        .saturating_add(mac.rx_ready_guard_us);

    let rx_us = if max_tx_delay_us == 0 {
        target_rx_us.min(tick_period_us)
    } else {
        target_rx_us.min(max_tx_delay_us)
    };

    let mut symbols = div_ceil_u64(rx_us.max(1), symbol_us) as u16;
    if symbols < 4 {
        symbols = 4;
    }
    if symbols < mac.rx_timeout_symbols {
        mac.rx_timeout_symbols
    } else {
        symbols
    }
}

pub fn slot_rx_symbols(rf: RfConfig, mac: MacConfig) -> u16 {
    // Not shareable: the slot window depends on per-vehicle schedule settings.
    let symbol_us = lora_symbol_time_us(rf);
    if symbol_us == 0 {
        return mac.rx_timeout_symbols;
    }

    let slot_us = mac.tick_period_us().saturating_sub(mac.tx_guard_us);
    let mut symbols = div_ceil_u64(slot_us.max(1), symbol_us) as u16;
    if symbols < mac.rx_timeout_symbols {
        symbols = mac.rx_timeout_symbols;
    }
    symbols
}

fn lora_symbol_time_us(cfg: RfConfig) -> u64 {
    let bw_hz = match cfg.bw {
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

    let sf = cfg.sf as u32;
    if sf < 5 || sf > 12 || bw_hz == 0 {
        return 0;
    }

    let tsym_num: u128 = (1_000_000u128) << sf;
    let tsym_den: u128 = bw_hz as u128;
    div_ceil_u128(tsym_num, tsym_den) as u64
}

fn div_ceil_u64(n: u64, d: u64) -> u64 {
    if d == 0 {
        return 0;
    }
    (n + d - 1) / d
}

fn div_ceil_u128(n: u128, d: u128) -> u128 {
    if d == 0 {
        return 0;
    }
    (n + d - 1) / d
}
