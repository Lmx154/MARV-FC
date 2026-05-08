use super::{LoraProfile, frame::FRAME_HEADER_LEN};

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub struct LoraLinkTiming {
    pub frame_airtime_us: u32,
    pub pong_wait_ms: u64,
    pub peer_timeout_ms: u64,
    pub rx_window_symbols: u16,
    pub pong_rx_attempts: u8,
    pub peer_rx_attempts: u8,
    pub degraded_after_misses: u8,
    pub lost_after_misses: u8,
    pub rx_restart_retry_limit: u8,
    pub radio_fault_recovery_limit: u8,
    pub reconnect_backoff_ms: u64,
}

impl LoraLinkTiming {
    pub const fn from_profile(
        profile: &LoraProfile,
        max_payload_len: u8,
        keepalive_period_ms: u64,
    ) -> Self {
        let frame_len = FRAME_HEADER_LEN as u8 + max_payload_len;
        let frame_airtime_us = profile.time_on_air_us(frame_len);
        let frame_airtime_ms = ceil_div_u64(frame_airtime_us as u64, 1_000);
        let exchange_margin_ms = frame_airtime_ms.saturating_mul(6).saturating_add(250);
        let period_margin_ms = keepalive_period_ms.saturating_mul(3) / 4;
        let pong_wait_ms = max_u64(exchange_margin_ms, period_margin_ms);
        let peer_timeout_ms = keepalive_period_ms
            .saturating_mul(4)
            .saturating_add(pong_wait_ms);
        let rx_window_symbols = clamp_u16(
            profile.modulation.delay_in_symbols(250),
            8,
            SX126X_MAX_RX_TIMEOUT_SYMBOLS,
        );
        let rx_window_ms = max_u64(
            profile.modulation.symbols_to_ms(rx_window_symbols as u32) as u64,
            1,
        );

        Self {
            frame_airtime_us,
            pong_wait_ms,
            peer_timeout_ms,
            rx_window_symbols,
            pong_rx_attempts: clamp_u64_to_u8(ceil_div_u64(pong_wait_ms, rx_window_ms), 1, u8::MAX),
            peer_rx_attempts: clamp_u64_to_u8(
                ceil_div_u64(peer_timeout_ms, rx_window_ms),
                1,
                u8::MAX,
            ),
            degraded_after_misses: 1,
            lost_after_misses: 3,
            rx_restart_retry_limit: 3,
            radio_fault_recovery_limit: 3,
            reconnect_backoff_ms: max_u64(frame_airtime_ms.saturating_mul(2), 100),
        }
    }
}

const SX126X_MAX_RX_TIMEOUT_SYMBOLS: u16 = 248;

const fn ceil_div_u64(value: u64, divisor: u64) -> u64 {
    value.saturating_add(divisor - 1) / divisor
}

const fn max_u64(left: u64, right: u64) -> u64 {
    if left > right { left } else { right }
}

const fn clamp_u16(value: u16, min: u16, max: u16) -> u16 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

const fn clamp_u64_to_u8(value: u64, min: u8, max: u8) -> u8 {
    if value < min as u64 {
        min
    } else if value > max as u64 {
        max
    } else {
        value as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comms::links::lora::LoraProfile;

    #[test]
    fn bounded_rx_windows_cover_link_timeouts() {
        let timing = LoraLinkTiming::from_profile(&LoraProfile::known_good_915(), 4, 2_000);
        let window_ms = LoraProfile::known_good_915()
            .modulation
            .symbols_to_ms(timing.rx_window_symbols as u32) as u64;

        assert!(timing.rx_window_symbols > 0);
        assert!(timing.rx_window_symbols <= SX126X_MAX_RX_TIMEOUT_SYMBOLS);
        assert!(window_ms.saturating_mul(timing.pong_rx_attempts as u64) >= timing.pong_wait_ms);
        assert!(window_ms.saturating_mul(timing.peer_rx_attempts as u64) >= timing.peer_timeout_ms);
    }
}
