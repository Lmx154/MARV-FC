use super::lora_config::LoRaConfig;
use super::mac_codec::HEADER_LEN;

#[derive(Clone, Copy)]
pub struct LinkTestConfig {
    pub lora: LoRaConfig,
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

impl LinkTestConfig {
    pub const fn tick_period_us(self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1_000_000u64 / hz as u64
    }

    pub const fn tick_period_ms(self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1000u64 / hz as u64
    }
}

pub fn rx_timeout_symbols(cfg: LinkTestConfig) -> u16 {
    if !cfg.rx_timeout_auto {
        return cfg.rx_timeout_symbols;
    }

    let symbol_us = lora_symbol_time_us(cfg.lora);
    if symbol_us == 0 {
        return cfg.rx_timeout_symbols;
    }

    let uplink_len = HEADER_LEN + cfg.uplink_payload_len;
    let downlink_len = HEADER_LEN + cfg.downlink_payload_len;
    let uplink_toa_us = cfg.lora.toa_us(uplink_len);
    let downlink_toa_us = cfg.lora.toa_us(downlink_len);
    let tick_period_us = cfg.tick_period_us();

    let max_tx_delay_us = tick_period_us
        .saturating_sub(cfg.tx_guard_us)
        .saturating_sub(uplink_toa_us);

    let target_rx_us = cfg
        .dl_tx_offset_us
        .saturating_add(downlink_toa_us)
        .saturating_add(cfg.rx_ready_guard_us);

    let rx_us = if max_tx_delay_us == 0 {
        target_rx_us.min(tick_period_us)
    } else {
        target_rx_us.min(max_tx_delay_us)
    };

    let mut symbols = div_ceil_u64(rx_us.max(1), symbol_us) as u16;
    if symbols < 4 {
        symbols = 4;
    }
    if symbols < cfg.rx_timeout_symbols {
        cfg.rx_timeout_symbols
    } else {
        symbols
    }
}

pub fn slot_rx_symbols(cfg: LinkTestConfig) -> u16 {
    let symbol_us = lora_symbol_time_us(cfg.lora);
    if symbol_us == 0 {
        return cfg.rx_timeout_symbols;
    }

    let slot_us = cfg.tick_period_us().saturating_sub(cfg.tx_guard_us);
    let mut symbols = div_ceil_u64(slot_us.max(1), symbol_us) as u16;
    if symbols < cfg.rx_timeout_symbols {
        symbols = cfg.rx_timeout_symbols;
    }
    symbols
}

fn lora_symbol_time_us(cfg: LoRaConfig) -> u64 {
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

// Fast-mid profile: better margin than SF6, still high throughput.
const LORA_SF7_BW500_CR45: LoRaConfig = LoRaConfig {
    sf: 7,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Fast profile: default bench throughput preset (SF6/BW500/CR4/5).
const LORA_SF6_BW500_CR45: LoRaConfig = LoRaConfig::preset_fast();

// Max bandwidth LoRa (short-range / stress testing).
const LORA_SF5_BW500_CR45: LoRaConfig = LoRaConfig {
    sf: 5,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Robust mid-range profile.
const LORA_SF9_BW250_CR46: LoRaConfig = LoRaConfig {
    sf: 9,
    bw: 0x05,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

// Long-range-ish profile (slower, larger link budget).
const LORA_SF10_BW125_CR46: LoRaConfig = LoRaConfig {
    sf: 10,
    bw: 0x04,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

pub const FAST_TEST_CONFIG: LinkTestConfig = LinkTestConfig {
    lora: LoRaConfig::preset_fast(),
    tick_hz: 50,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 8,
};

// SF7/BW500 @ 50 Hz: "fast mid" with better margin than SF6.
pub const FAST_SF7_BW500_50HZ: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF7_BW500_CR45,
    tick_hz: 50,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 8,
};

// SF6/BW500 @ 100 Hz: higher refresh with tiny frames.
pub const FAST_SF6_BW500_100HZ_TINY: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF6_BW500_CR45,
    tick_hz: 100,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

// SF5/BW500 @ 150 Hz: aggressive refresh, minimal margin.
pub const FAST_SF5_BW500_150HZ_TINY: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF5_BW500_CR45,
    tick_hz: 150,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 1_500,
    rx_ready_guard_us: 400,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

// SF5/BW500 @ 200 Hz: extreme test, essentially header-only frames.
pub const FAST_SF5_BW500_200HZ_MIN: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF5_BW500_CR45,
    tick_hz: 200,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 500,
    rx_ready_guard_us: 300,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 0,
    downlink_payload_len: 0,
};

// SF9/BW250 @ 10 Hz: robust mid-range telemetry.
pub const ROBUST_SF9_BW250_10HZ: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF9_BW250_CR46,
    tick_hz: 10,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 8,
};

// SF10/BW125 @ 2 Hz: slow, higher link budget.
pub const RANGE_SF10_BW125_2HZ: LinkTestConfig = LinkTestConfig {
    lora: LORA_SF10_BW125_CR46,
    tick_hz: 2,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 128,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 8,
};

// SF11/BW125/CR4/8 @ 1 Hz: maximum range preset (very slow).
pub const LONG_TEST_CONFIG: LinkTestConfig = LinkTestConfig {
    lora: LoRaConfig::preset_long_range(),
    tick_hz: 1,
    slot_ratio_r: 5,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 8,
};

// Toggle this to switch profiles for both GS and Radio during testing.
pub const ACTIVE: LinkTestConfig = FAST_TEST_CONFIG;
