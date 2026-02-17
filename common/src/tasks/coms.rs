//! Transport tasks shared across devices (LoRa, UART, etc).
//! Keep pinout-specific setup in device crates and pass configured buses here.

#![allow(async_fn_in_trait)]

#[cfg(feature = "mavlink")]
pub use crate::protocol::mavlink::encode::{
    build_statustext, build_statustext_frame, build_telemetry_frame, statustext_to_str,
    MavEndpointConfig, TelemetrySample, TelemetrySource, STATUSTEXT_CAP,
};

use defmt::{info, warn};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::coms::transport::uart::AsyncUartBus;
use crate::coms::transport::uart_packet::{
    recv_packet_over_uart, send_packet_over_uart, UART_PACKET_MAX_FRAME,
};
use crate::drivers::neom9n::GpsData;
use crate::protocol::packet::{
    Packet, PacketType, TelemetryBaro, TelemetryGps, TelemetryImu, TelemetryMag,
};
use crate::utils::delay::DelayMs;

#[derive(Clone, Copy)]
pub struct UartRxConfig {
    pub log_every: u32,
    pub log_payload_max: usize,
    pub summary_every: u32,
}

impl Default for UartRxConfig {
    fn default() -> Self {
        Self {
            log_every: 1,
            log_payload_max: 32,
            summary_every: 50,
        }
    }
}

#[derive(Clone, Copy)]
pub struct UartTxConfig {
    pub log_every: u32,
    pub log_payload_max: usize,
    pub summary_every: u32,
}

impl Default for UartTxConfig {
    fn default() -> Self {
        Self {
            log_every: 1,
            log_payload_max: 32,
            summary_every: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct UartTelemetrySnapshot {
    pub imu_seq: u32,
    pub imu_accel: [i16; 3],
    pub imu_gyro: [i16; 3],
    pub mag_seq: u32,
    pub mag_xyz: [i32; 3],
    pub baro_seq: u32,
    pub baro_t_c_x100: i32,
    pub baro_p_pa: i32,
    pub gps_seq: u32,
    pub gps_fix: Option<GpsData>,
}

pub trait UartTelemetrySource {
    async fn snapshot(&self) -> UartTelemetrySnapshot;
}

pub trait UartTelemetryRate {
    async fn rate_hz(&self) -> u32;
}

pub async fn uart_rx_task<U, M, const N: usize>(
    mut uart: U,
    tx: Sender<'static, M, Packet, N>,
    cfg: UartRxConfig,
) -> !
where
    U: AsyncUartBus,
    M: RawMutex,
{
    let mut scratch = [0u8; UART_PACKET_MAX_FRAME];
    let mut frames: u32 = 0;
    let mut drops: u32 = 0;
    let mut errors: u32 = 0;
    loop {
        match recv_packet_over_uart(&mut uart, &mut scratch).await {
            Ok(packet) => {
                let packet_type = packet.packet_type;
                frames = frames.wrapping_add(1);
                if should_log(frames, cfg.log_every) {
                    let payload = packet.payload.as_slice();
                    let show_len = payload.len().min(cfg.log_payload_max);
                    if show_len < payload.len() {
                        info!(
                            "UART RX frame={} type={} frame_len={} payload_head={=[u8]} (trunc)",
                            frames,
                            packet_type.name(),
                            1usize.saturating_add(payload.len()),
                            &payload[..show_len]
                        );
                    } else {
                        info!(
                            "UART RX frame={} type={} frame_len={} payload={=[u8]}",
                            frames,
                            packet_type.name(),
                            1usize.saturating_add(payload.len()),
                            payload
                        );
                    }
                } else if should_log(frames, cfg.summary_every) {
                    info!(
                        "UART RX packet type={} count={}",
                        packet_type.name(),
                        frames
                    );
                }
                if tx.try_send(packet).is_err() {
                    drops = drops.wrapping_add(1);
                    if drops == 1 || drops % 50 == 0 {
                        warn!("UART RX drop count={}", drops);
                    }
                    continue;
                }
            }
            Err(_) => {
                errors = errors.wrapping_add(1);
                if errors == 1 || errors % 50 == 0 {
                    warn!("UART RX error count={}", errors);
                }
            }
        }
    }
}

pub async fn uart_packet_tx_task<U, M, const N: usize>(
    mut uart: U,
    rx: Receiver<'static, M, Packet, N>,
    cfg: UartTxConfig,
) -> !
where
    U: AsyncUartBus,
    M: RawMutex,
{
    let mut scratch = [0u8; UART_PACKET_MAX_FRAME];
    let mut sent: u32 = 0;
    let mut errors: u32 = 0;
    loop {
        let packet = rx.receive().await;
        match send_packet_over_uart(&mut uart, &packet, &mut scratch).await {
            Ok(()) => {
                sent = sent.wrapping_add(1);
                if should_log(sent, cfg.log_every) {
                    log_uart_tx(&packet, sent, cfg);
                } else if should_log(sent, cfg.summary_every) {
                    info!(
                        "UART TX packet type={} count={}",
                        packet.packet_type.name(),
                        sent
                    );
                }
            }
            Err(_) => {
                errors = errors.wrapping_add(1);
                if errors == 1 || errors % 50 == 0 {
                    warn!("UART TX error count={}", errors);
                }
            }
        }
    }
}

pub async fn uart_telemetry_task<U, S, R, D>(
    mut uart: U,
    source: S,
    rate: R,
    mut delay: D,
    cfg: UartTxConfig,
) -> !
where
    U: AsyncUartBus,
    S: UartTelemetrySource,
    R: UartTelemetryRate,
    D: DelayMs,
{
    info!("UART: Starting custom link task for radio");
    let mut tx_buf = [0u8; UART_PACKET_MAX_FRAME];
    let mut tx_logs: u32 = 0;
    let mut tx_errors: u32 = 0;
    let mut last_imu_seq: u32 = 0;
    let mut last_mag_seq: u32 = 0;
    let mut last_baro_seq: u32 = 0;
    let mut last_gps_seq: u32 = 0;
    let mut interval_ms: u32 = 10;

    loop {
        let rad_tel_hz = rate.rate_hz().await.clamp(1, 1000);
        let next_interval_ms = hz_to_interval_ms(rad_tel_hz);
        if next_interval_ms != interval_ms {
            interval_ms = next_interval_ms;
        }

        let s = source.snapshot().await;

        if s.imu_seq != last_imu_seq {
            let imu = TelemetryImu {
                accel: s.imu_accel,
                gyro: s.imu_gyro,
            };
            let packet = Packet::with_payload(PacketType::TelemetryImu, imu.encode());
            match send_packet_over_uart(&mut uart, &packet, &mut tx_buf).await {
                Ok(()) => {
                    tx_logs = tx_logs.wrapping_add(1);
                    log_uart_tx(&packet, tx_logs, cfg);
                    last_imu_seq = s.imu_seq;
                }
                Err(_) => {
                    tx_errors = tx_errors.wrapping_add(1);
                    if tx_errors == 1 || tx_errors % 50 == 0 {
                        warn!(
                            "UART TX error type={} count={}",
                            packet.packet_type.name(),
                            tx_errors
                        );
                    }
                }
            }
        }

        if s.mag_seq != last_mag_seq {
            let mag = TelemetryMag {
                mag: [
                    clamp_i16(s.mag_xyz[0]),
                    clamp_i16(s.mag_xyz[1]),
                    clamp_i16(s.mag_xyz[2]),
                ],
            };
            let packet = Packet::with_payload(PacketType::TelemetryMag, mag.encode());
            match send_packet_over_uart(&mut uart, &packet, &mut tx_buf).await {
                Ok(()) => {
                    tx_logs = tx_logs.wrapping_add(1);
                    log_uart_tx(&packet, tx_logs, cfg);
                    last_mag_seq = s.mag_seq;
                }
                Err(_) => {
                    tx_errors = tx_errors.wrapping_add(1);
                    if tx_errors == 1 || tx_errors % 50 == 0 {
                        warn!(
                            "UART TX error type={} count={}",
                            packet.packet_type.name(),
                            tx_errors
                        );
                    }
                }
            }
        }

        if s.baro_seq != last_baro_seq {
            let baro = TelemetryBaro {
                pressure_pa: s.baro_p_pa,
                temp_c_x10: clamp_i16((s.baro_t_c_x100 / 10) as i32),
            };
            let packet = Packet::with_payload(PacketType::TelemetryBaro, baro.encode());
            match send_packet_over_uart(&mut uart, &packet, &mut tx_buf).await {
                Ok(()) => {
                    tx_logs = tx_logs.wrapping_add(1);
                    log_uart_tx(&packet, tx_logs, cfg);
                    last_baro_seq = s.baro_seq;
                }
                Err(_) => {
                    tx_errors = tx_errors.wrapping_add(1);
                    if tx_errors == 1 || tx_errors % 50 == 0 {
                        warn!(
                            "UART TX error type={} count={}",
                            packet.packet_type.name(),
                            tx_errors
                        );
                    }
                }
            }
        }

        if s.gps_seq != last_gps_seq {
            if let Some(fix) = s.gps_fix {
                let gps = TelemetryGps {
                    lat: fix.latitude,
                    lon: fix.longitude,
                    alt_mm: fix.altitude,
                    sats: fix.satellites,
                    fix: fix.fix_type,
                };
                let packet = Packet::with_payload(PacketType::TelemetryGps, gps.encode());
                match send_packet_over_uart(&mut uart, &packet, &mut tx_buf).await {
                    Ok(()) => {
                        tx_logs = tx_logs.wrapping_add(1);
                        log_uart_tx(&packet, tx_logs, cfg);
                        last_gps_seq = s.gps_seq;
                    }
                    Err(_) => {
                        tx_errors = tx_errors.wrapping_add(1);
                        if tx_errors == 1 || tx_errors % 50 == 0 {
                            warn!(
                                "UART TX error type={} count={}",
                                packet.packet_type.name(),
                                tx_errors
                            );
                        }
                    }
                }
            }
        }

        delay.delay_ms(interval_ms).await;
    }
}

fn log_uart_tx(packet: &Packet, count: u32, cfg: UartTxConfig) {
    if !should_log(count, cfg.log_every) {
        return;
    }
    let payload = packet.payload.as_slice();
    let show_len = payload.len().min(cfg.log_payload_max);
    if show_len < payload.len() {
        info!(
            "UART TX frame={} type={} payload_len={} payload_head={=[u8]} (trunc)",
            count,
            packet.packet_type.name(),
            payload.len(),
            &payload[..show_len]
        );
    } else {
        info!(
            "UART TX frame={} type={} payload_len={} payload={=[u8]}",
            count,
            packet.packet_type.name(),
            payload.len(),
            payload
        );
    }
}

fn should_log(count: u32, every: u32) -> bool {
    every > 0 && (count == 1 || count % every == 0)
}

fn clamp_i16(v: i32) -> i16 {
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn hz_to_interval_ms(hz: u32) -> u32 {
    if hz == 0 {
        return 1000;
    }
    let ms = 1000 / hz;
    ms.max(1)
}
