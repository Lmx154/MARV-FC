use common::comms::links::lora::ACTIVE;
use common::comms::links::lora::frame::{
    LoraFrame, LoraFrameKind, LoraNodeRole, MAX_FRAME_LEN, decode_frame, encode_frame,
};
use common::comms::links::lora::state::{LoraLinkHealth, LoraLinkPolicy, LoraLinkState};
use common::comms::links::lora::timing::LoraLinkTiming;
use common::drivers::sx1262::{RawRx, Sx1262, Sx1262Error};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::buses::LoraSpiBus;
use crate::channels::{STATUS_INDICATOR_CHANNEL, StatusIndicatorEvent, StatusIndicatorSender};
use crate::config::{self, FirmwareRole};
use crate::resources::LoraPins;
use crate::watchdog::WatchdogResources;

const DEMO_FRAME_PAYLOAD_LEN: u8 = 4;

#[embassy_executor::task]
async fn lora_demo_task(
    bus: LoraSpiBus,
    pins: LoraPins,
    _watchdog: WatchdogResources,
    role: FirmwareRole,
) -> ! {
    let indicator = STATUS_INDICATOR_CHANNEL.sender();
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = config::LORA_SPI_FREQUENCY_HZ;

    let spi = Spi::new(
        bus.spi, pins.sck, pins.mosi, pins.miso, bus.tx_dma, bus.rx_dma, spi_config,
    );
    let cs = Output::new(pins.cs, Level::High);
    let spi_device = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let reset = Output::new(pins.reset, Level::High);
    let busy = Input::new(pins.busy, Pull::None);
    let dio1 = Input::new(pins.dio1, Pull::None);
    let _dio2 = Input::new(pins.dio2, Pull::None);
    let txen = Output::new(pins.txen, Level::Low);
    let rxen = Output::new(pins.rxen, Level::Low);

    let radio = Sx1262::new(spi_device, reset, busy, dio1, txen, rxen, ACTIVE, Delay).await;
    let mut radio = match radio {
        Ok(radio) => radio,
        Err(_) => {
            warn!("sx1262 init failed; lora ping/pong demo stopped");
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    let timing = LoraLinkTiming::from_profile(
        radio.profile(),
        DEMO_FRAME_PAYLOAD_LEN,
        config::LORA_PING_PERIOD_MS,
    );
    let policy = LoraLinkPolicy::new(
        timing.degraded_after_misses,
        timing.lost_after_misses,
        timing.radio_fault_recovery_limit,
    );
    let mut health = LoraLinkHealth::new(policy);
    publish_state(indicator, health.state()).await;

    info!(
        "sx1262 lora demo ready: role={:?} freq={=u32}Hz spi={=u32}Hz airtime={=u32}us pong_wait={=u64}ms peer_timeout={=u64}ms rx_window_symbols={=u16}",
        role,
        ACTIVE.frequency_hz,
        config::LORA_SPI_FREQUENCY_HZ,
        timing.frame_airtime_us,
        timing.pong_wait_ms,
        timing.peer_timeout_ms,
        timing.rx_window_symbols
    );

    match role {
        FirmwareRole::Radio => run_ping_side(&mut radio, indicator, &timing, &mut health).await,
        FirmwareRole::GroundStation => {
            run_pong_side(&mut radio, indicator, &timing, &mut health).await
        }
    }
}

fn node_role(role: FirmwareRole) -> LoraNodeRole {
    match role {
        FirmwareRole::Radio => LoraNodeRole::Radio,
        FirmwareRole::GroundStation => LoraNodeRole::GroundStation,
    }
}

fn encode_control_frame(
    source: LoraNodeRole,
    kind: LoraFrameKind,
    sequence: u32,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let payload = sequence.to_le_bytes();
    encode_frame(
        LoraFrame {
            source,
            kind,
            payload: &payload,
        },
        out,
    )
    .expect("fixed lora control frame must fit")
}

fn decode_control_sequence(frame: LoraFrame<'_>) -> Option<u32> {
    if frame.payload.len() != DEMO_FRAME_PAYLOAD_LEN as usize {
        return None;
    }

    Some(u32::from_le_bytes([
        frame.payload[0],
        frame.payload[1],
        frame.payload[2],
        frame.payload[3],
    ]))
}

fn post_indicator(indicator: StatusIndicatorSender, event: StatusIndicatorEvent) {
    let _ = indicator.try_send(event);
}

async fn publish_state(indicator: StatusIndicatorSender, state: LoraLinkState) {
    post_indicator(indicator, StatusIndicatorEvent::LinkState(state));
}

async fn publish_state_change(
    indicator: StatusIndicatorSender,
    previous: LoraLinkState,
    health: &LoraLinkHealth,
) {
    let current = health.state();
    if current != previous {
        post_indicator(indicator, StatusIndicatorEvent::LinkState(current));
        info!(
            "lora link state {:?}->{:?} misses={=u8} tx={=u32} rx={=u32} tx_err={=u32} rx_err={=u32} malformed={=u32}",
            previous,
            current,
            health.consecutive_misses(),
            health.stats().tx_packets,
            health.stats().rx_packets,
            health.stats().tx_errors,
            health.stats().rx_errors,
            health.stats().malformed_frames
        );
    }
}

fn should_log_packet(count: u32) -> bool {
    config::LORA_PACKET_LOG_EVERY != 0 && count % config::LORA_PACKET_LOG_EVERY == 0
}

fn should_log_miss(previous: LoraLinkState, current: LoraLinkState, misses: u8) -> bool {
    previous != current
        || config::LORA_MISS_LOG_EVERY != 0 && misses % config::LORA_MISS_LOG_EVERY == 0
}

async fn note_peer_missed(
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    reason: &'static str,
) {
    let previous = health.state();
    let current = health.note_missed_peer_packet();
    let misses = health.consecutive_misses();
    if (current == LoraLinkState::Degraded || current == LoraLinkState::Lost)
        && should_log_miss(previous, current, misses)
    {
        warn!(
            "lora peer miss reason={=str} state={:?} misses={=u8}",
            reason, current, misses
        );
    }
    publish_state_change(indicator, previous, health).await;
}

async fn note_valid_peer_rx(
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    rssi: i16,
    snr_x4: i16,
) {
    let previous = health.state();
    health.note_valid_rx(rssi, snr_x4);
    publish_state_change(indicator, previous, health).await;
}

async fn restart_or_recover_rx<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    timing: &LoraLinkTiming,
    reason: &'static str,
    force_reinitialize: bool,
) -> bool
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    if !force_reinitialize {
        for attempt in 1..=timing.rx_restart_retry_limit {
            match radio.start_rx_continuous().await {
                Ok(()) => {
                    health.note_rx_restart();
                    if attempt > 1 {
                        info!(
                            "lora continuous rx restarted reason={=str} attempts={=u8}",
                            reason, attempt
                        );
                    }
                    return true;
                }
                Err(_) => {
                    warn!(
                        "lora continuous rx restart failed reason={=str} attempt={=u8}",
                        reason, attempt
                    );
                    Timer::after(Duration::from_millis(timing.reconnect_backoff_ms)).await;
                }
            }
        }
    }

    warn!(
        "lora attempting sx1262 reinitialize reason={=str} forced={=bool}",
        reason, force_reinitialize
    );
    match radio.recover_rx_continuous().await {
        Ok(()) => {
            let previous = health.state();
            health.note_radio_recovery_ok();
            publish_state_change(indicator, previous, health).await;
            info!("lora sx1262 reinitialize recovered reason={=str}", reason);
            true
        }
        Err(_) => {
            let previous = health.state();
            health.note_radio_recovery_failed();
            publish_state_change(indicator, previous, health).await;
            warn!(
                "lora sx1262 reinitialize failed reason={=str} state={:?}",
                reason,
                health.state()
            );
            false
        }
    }
}

fn peer_control_sequence(
    frame: LoraFrame<'_>,
    source: LoraNodeRole,
    kind: LoraFrameKind,
) -> Option<u32> {
    if frame.source != source || frame.kind != kind {
        return None;
    }

    decode_control_sequence(frame)
}

fn lost_recovery_due(health: &LoraLinkHealth, timing: &LoraLinkTiming) -> bool {
    health.state() == LoraLinkState::Lost
        && health.consecutive_misses() % timing.lost_after_misses == 0
}

async fn receive_for_attempts<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    timing: &LoraLinkTiming,
    attempts: u8,
    rx_buf: &mut [u8; MAX_FRAME_LEN],
) -> Result<Option<RawRx>, Sx1262Error>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    for attempt in 1..=attempts {
        radio.start_rx_single(timing.rx_window_symbols).await?;
        match radio.receive(rx_buf).await {
            Ok(Some(rx)) => return Ok(Some(rx)),
            Ok(None) => return Ok(None),
            Err(err) if err.is_receive_timeout() => {
                if attempt == attempts {
                    return Ok(None);
                }
            }
            Err(err) => return Err(err),
        }
    }

    Ok(None)
}

async fn run_ping_side<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    indicator: StatusIndicatorSender,
    timing: &LoraLinkTiming,
    health: &mut LoraLinkHealth,
) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let mut tx_buf = [0u8; MAX_FRAME_LEN];
    let mut rx_buf = [0u8; MAX_FRAME_LEN];
    let mut count = 0u32;
    let source = node_role(FirmwareRole::Radio);

    loop {
        count = count.wrapping_add(1);
        let ping_len = encode_control_frame(source, LoraFrameKind::Ping, count, &mut tx_buf);
        match radio.transmit(&tx_buf[..ping_len]).await {
            Ok(()) => {
                health.note_tx_packet();
                if should_log_packet(count) {
                    info!("lora ping tx count={=u32} seq={=u32}", count, count);
                }
                post_indicator(indicator, StatusIndicatorEvent::Ping);
            }
            Err(_) => {
                let previous = health.state();
                health.note_tx_error();
                publish_state_change(indicator, previous, health).await;
                warn!("lora ping tx failed count={=u32}", count);
                restart_or_recover_rx(
                    radio,
                    indicator,
                    health,
                    timing,
                    "ping tx failed",
                    health.state() == LoraLinkState::Lost,
                )
                .await;
                Timer::after(Duration::from_millis(config::LORA_PING_PERIOD_MS)).await;
                continue;
            }
        }

        match receive_for_attempts(radio, timing, timing.pong_rx_attempts, &mut rx_buf).await {
            Ok(Some(rx)) => match decode_frame(&rx_buf[..rx.len as usize]) {
                Ok(frame) => {
                    if let Some(seq) = peer_control_sequence(
                        frame,
                        LoraNodeRole::GroundStation,
                        LoraFrameKind::Pong,
                    ) {
                        note_valid_peer_rx(indicator, health, rx.rssi, rx.snr_x4).await;
                        if seq != count {
                            warn!(
                                "lora pong rx sequence mismatch expected={=u32} got={=u32}",
                                count, seq
                            );
                        }
                        if should_log_packet(count) {
                            info!(
                                "lora pong rx count={=u32} seq={=u32} rssi={=i16} snr_x4={=i16}",
                                count, seq, rx.rssi, rx.snr_x4
                            );
                        }
                        post_indicator(indicator, StatusIndicatorEvent::Pong);
                    } else {
                        health.note_unexpected_frame();
                        warn!(
                            "lora unexpected frame source={:?} kind={:?} len={=usize} rssi={=i16}",
                            frame.source,
                            frame.kind,
                            frame.payload.len(),
                            rx.rssi
                        );
                        note_peer_missed(indicator, health, "unexpected frame").await;
                    }
                }
                Err(err) => {
                    health.note_malformed_frame();
                    warn!("lora malformed frame err={:?} rx_len={=u8}", err, rx.len);
                    note_peer_missed(indicator, health, "malformed frame").await;
                }
            },
            Ok(None) => {
                note_peer_missed(indicator, health, "pong wait timeout").await;
                let force_reinitialize = lost_recovery_due(health, timing);
                restart_or_recover_rx(
                    radio,
                    indicator,
                    health,
                    timing,
                    "pong wait timeout",
                    force_reinitialize,
                )
                .await;
            }
            Err(_) => {
                health.note_rx_error();
                warn!("lora pong rx failed");
                note_peer_missed(indicator, health, "pong rx failed").await;
                let force_reinitialize = lost_recovery_due(health, timing);
                restart_or_recover_rx(
                    radio,
                    indicator,
                    health,
                    timing,
                    "pong rx failed",
                    force_reinitialize,
                )
                .await;
            }
        }

        Timer::after(Duration::from_millis(config::LORA_PING_PERIOD_MS)).await;
    }
}

async fn run_pong_side<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    indicator: StatusIndicatorSender,
    timing: &LoraLinkTiming,
    health: &mut LoraLinkHealth,
) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let mut tx_buf = [0u8; MAX_FRAME_LEN];
    let mut rx_buf = [0u8; MAX_FRAME_LEN];
    let mut count = 0u32;
    let source = node_role(FirmwareRole::GroundStation);

    while !restart_or_recover_rx(radio, indicator, health, timing, "startup", false).await {
        Timer::after(Duration::from_millis(timing.reconnect_backoff_ms)).await;
    }

    loop {
        match receive_for_attempts(radio, timing, timing.peer_rx_attempts, &mut rx_buf).await {
            Ok(Some(rx)) => match decode_frame(&rx_buf[..rx.len as usize]) {
                Ok(frame) => {
                    if let Some(seq) =
                        peer_control_sequence(frame, LoraNodeRole::Radio, LoraFrameKind::Ping)
                    {
                        note_valid_peer_rx(indicator, health, rx.rssi, rx.snr_x4).await;
                        count = count.wrapping_add(1);
                        if should_log_packet(count) {
                            info!(
                                "lora ping rx count={=u32} seq={=u32} rssi={=i16} snr_x4={=i16}",
                                count, seq, rx.rssi, rx.snr_x4
                            );
                        }
                        post_indicator(indicator, StatusIndicatorEvent::Ping);

                        let pong_len =
                            encode_control_frame(source, LoraFrameKind::Pong, seq, &mut tx_buf);
                        match radio.transmit(&tx_buf[..pong_len]).await {
                            Ok(()) => {
                                health.note_tx_packet();
                                if should_log_packet(count) {
                                    info!("lora pong tx count={=u32} seq={=u32}", count, seq);
                                }
                                post_indicator(indicator, StatusIndicatorEvent::Pong);
                            }
                            Err(_) => {
                                let previous = health.state();
                                health.note_tx_error();
                                publish_state_change(indicator, previous, health).await;
                                warn!("lora pong tx failed count={=u32} seq={=u32}", count, seq);
                                restart_or_recover_rx(
                                    radio,
                                    indicator,
                                    health,
                                    timing,
                                    "pong tx failed",
                                    health.state() == LoraLinkState::Lost,
                                )
                                .await;
                            }
                        }
                    } else {
                        health.note_unexpected_frame();
                        warn!(
                            "lora unexpected frame source={:?} kind={:?} len={=usize} rssi={=i16}",
                            frame.source,
                            frame.kind,
                            frame.payload.len(),
                            rx.rssi
                        );
                    }
                }
                Err(err) => {
                    health.note_malformed_frame();
                    warn!("lora malformed frame err={:?} rx_len={=u8}", err, rx.len);
                }
            },
            Ok(None) => {
                let previous = health.state();
                health.note_idle_timeout();
                publish_state_change(indicator, previous, health).await;
                if should_log_miss(previous, health.state(), health.consecutive_misses()) {
                    warn!(
                        "lora link idle timeout after {=u64}ms misses={=u8}",
                        timing.peer_timeout_ms,
                        health.consecutive_misses()
                    );
                }
                let force_reinitialize = lost_recovery_due(health, timing);
                restart_or_recover_rx(
                    radio,
                    indicator,
                    health,
                    timing,
                    "idle timeout",
                    force_reinitialize,
                )
                .await;
            }
            Err(Sx1262Error::Radio(_)) => {
                health.note_rx_error();
                warn!("lora rx radio error");
                restart_or_recover_rx(radio, indicator, health, timing, "rx radio error", false)
                    .await;
            }
            Err(Sx1262Error::InvalidParam) => {
                health.note_rx_error();
                warn!("lora rx invalid parameter");
                restart_or_recover_rx(
                    radio,
                    indicator,
                    health,
                    timing,
                    "rx invalid parameter",
                    false,
                )
                .await;
            }
        }
    }
}

pub fn spawn(
    spawner: &Spawner,
    bus: LoraSpiBus,
    pins: LoraPins,
    watchdog: WatchdogResources,
    role: FirmwareRole,
) {
    spawner
        .spawn(lora_demo_task(bus, pins, watchdog, role))
        .expect("lora demo task spawn failed");
}
