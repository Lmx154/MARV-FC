use common::comms::links::lora::ACTIVE;
use common::comms::links::lora::frame::{
    LoraFrame, LoraFrameKind, LoraNodeRole, MAX_FRAME_LEN, decode_frame, encode_frame,
};
use common::comms::links::lora::state::{LoraLinkHealth, LoraLinkPolicy, LoraLinkState};
use common::comms::links::lora::timing::LoraLinkTiming;
use common::drivers::sx1262::Sx1262;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::buses::LoraSpiBus;
use crate::channels::{
    HILINK_BRIDGE_CHANNEL_DEPTH, HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame,
    LORA_TO_HOST_CHANNEL, STATUS_INDICATOR_CHANNEL, StatusIndicatorEvent, StatusIndicatorSender,
};
use crate::config::{self, FirmwareRole};
use crate::radio_dialect::{policy, scheduler, state_cache::RadioStateCache, translate};
use crate::resources::LoraPins;
use crate::watchdog::WatchdogResources;

#[embassy_executor::task]
async fn lora_bridge_task(
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
            warn!("sx1262 init failed; lora bridge stopped");
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    let timing = LoraLinkTiming::from_profile(
        radio.profile(),
        HILINK_BRIDGE_FRAME_BYTES as u8,
        config::LORA_HEARTBEAT_PERIOD_MS,
    );
    let policy = LoraLinkPolicy::new(
        timing.degraded_after_misses,
        timing.lost_after_misses,
        timing.radio_fault_recovery_limit,
    );
    let mut health = LoraLinkHealth::new(policy);
    publish_state(indicator, health.state()).await;

    info!(
        "sx1262 lora bridge ready: role={:?} freq={=u32}Hz spi={=u32}Hz airtime_max={=u32}us rx_window_symbols={=u16}",
        role,
        ACTIVE.frequency_hz,
        config::LORA_SPI_FREQUENCY_HZ,
        timing.frame_airtime_us,
        timing.rx_window_symbols
    );

    run_bridge(&mut radio, role, indicator, &timing, &mut health).await;
}

fn node_role(role: FirmwareRole) -> LoraNodeRole {
    match role {
        FirmwareRole::Radio => LoraNodeRole::Radio,
        FirmwareRole::GroundStation => LoraNodeRole::GroundStation,
    }
}

fn peer_role(role: FirmwareRole) -> LoraNodeRole {
    match role {
        FirmwareRole::Radio => LoraNodeRole::GroundStation,
        FirmwareRole::GroundStation => LoraNodeRole::Radio,
    }
}

fn status_time_ms() -> u32 {
    Instant::now().as_millis().min(u32::MAX as u64) as u32
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
            "lora bridge state {:?}->{:?} misses={=u8} tx={=u32} rx={=u32} tx_err={=u32} rx_err={=u32} malformed={=u32}",
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

async fn note_peer_keepalive_missed(
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    now: Instant,
    schedule: &mut BridgeSchedule,
) {
    if !health.linked_once() || now < schedule.next_peer_keepalive_due_at {
        return;
    }

    let previous = health.state();
    let current = health.note_missed_peer_packet();
    schedule.next_peer_keepalive_due_at = now + schedule.keepalive_period;
    publish_state_change(indicator, previous, health).await;
    if current == LoraLinkState::Degraded || current == LoraLinkState::Lost {
        warn!(
            "lora bridge peer keepalive missed state={:?} misses={=u8}",
            current,
            health.consecutive_misses()
        );
    }
}

struct BridgeSchedule {
    keepalive_period: Duration,
    next_keepalive_at: Instant,
    next_peer_keepalive_due_at: Instant,
}

impl BridgeSchedule {
    fn new(role: FirmwareRole, timing: &LoraLinkTiming) -> Self {
        let now = Instant::now();
        let keepalive_period = Duration::from_millis(config::LORA_HEARTBEAT_PERIOD_MS);
        let phase_offset = match role {
            FirmwareRole::Radio => Duration::from_millis(0),
            FirmwareRole::GroundStation => {
                Duration::from_millis(config::LORA_HEARTBEAT_PERIOD_MS / 2)
            }
        };
        let peer_keepalive_timeout = Duration::from_millis(timing.peer_timeout_ms);

        Self {
            keepalive_period,
            next_keepalive_at: now + phase_offset,
            next_peer_keepalive_due_at: now + peer_keepalive_timeout,
        }
    }

    fn note_peer_rx(&mut self, timing: &LoraLinkTiming) {
        self.next_peer_keepalive_due_at =
            Instant::now() + Duration::from_millis(timing.peer_timeout_ms);
    }
}

async fn recover_rx<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    reason: &'static str,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    warn!(
        "lora bridge attempting sx1262 recovery reason={=str}",
        reason
    );
    match radio.recover_rx_continuous().await {
        Ok(()) => {
            let previous = health.state();
            health.note_radio_recovery_ok();
            publish_state_change(indicator, previous, health).await;
            info!("lora bridge sx1262 recovery ok reason={=str}", reason);
        }
        Err(_) => {
            let previous = health.state();
            health.note_radio_recovery_failed();
            publish_state_change(indicator, previous, health).await;
            warn!("lora bridge sx1262 recovery failed reason={=str}", reason);
            Timer::after(Duration::from_millis(250)).await;
        }
    }
}

async fn transmit_lora_frame<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    source: LoraNodeRole,
    kind: LoraFrameKind,
    payload: &[u8],
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    reason: &'static str,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let mut tx_buf = [0u8; MAX_FRAME_LEN];
    let tx_len = match encode_frame(
        LoraFrame {
            source,
            kind,
            payload,
        },
        &mut tx_buf,
    ) {
        Ok(len) => len,
        Err(_) => {
            warn!(
                "lora bridge failed to encode frame kind={:?} bytes={=usize}",
                kind,
                payload.len()
            );
            return;
        }
    };

    match radio.transmit(&tx_buf[..tx_len]).await {
        Ok(()) => {
            health.note_tx_packet();
            if kind == LoraFrameKind::Data {
                info!("lora bridge data tx bytes={=usize}", payload.len());
            }
        }
        Err(_) => {
            let previous = health.state();
            health.note_tx_error();
            publish_state_change(indicator, previous, health).await;
            warn!(
                "lora bridge tx failed kind={:?} bytes={=usize}",
                kind,
                payload.len()
            );
            recover_rx(radio, indicator, health, reason).await;
        }
    }
}

async fn transmit_queued_host_frame<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    role: FirmwareRole,
    source: LoraNodeRole,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    dialect_cache: &mut RadioStateCache,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    for _ in 0..HILINK_BRIDGE_CHANNEL_DEPTH * 5 {
        let Some(host_frame) = receive_next_host_frame() else {
            break;
        };
        let translated = translate::host_to_rf(role, &host_frame, dialect_cache, status_time_ms());
        match translated {
            translate::HostToRfDecision::Translated(frame) => {
                transmit_lora_frame(
                    radio,
                    source,
                    LoraFrameKind::Data,
                    frame.as_slice(),
                    indicator,
                    health,
                    "data tx failed",
                )
                .await;
                return;
            }
            translate::HostToRfDecision::Cached => {}
            translate::HostToRfDecision::Drop => {
                warn!(
                    "radio dialect dropped unsupported host frame bytes={=usize}",
                    host_frame.len
                );
            }
        }
    }

    let now_ms = status_time_ms();
    dialect_cache.refresh_lora_link_status(
        now_ms,
        policy::LINK_STATUS_PERIOD_MS,
        health.stats(),
        0,
        policy::DEFAULT_TELEMETRY_RATE_HZ,
    );

    let Some(payload) = scheduler::select_scheduled_rf_frame(role, dialect_cache, now_ms) else {
        return;
    };
    transmit_lora_frame(
        radio,
        source,
        LoraFrameKind::Data,
        payload.as_slice(),
        indicator,
        health,
        "scheduled data tx failed",
    )
    .await;
}

fn receive_next_host_frame() -> Option<HilinkBridgeFrame> {
    scheduler::receive_next_host_frame()
}

async fn transmit_keepalive_if_due<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    source: LoraNodeRole,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    schedule: &mut BridgeSchedule,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let now = Instant::now();
    if now < schedule.next_keepalive_at {
        return;
    }

    schedule.next_keepalive_at = now + schedule.keepalive_period;
    let kind = if health.linked_once() {
        LoraFrameKind::Heartbeat
    } else {
        LoraFrameKind::Beacon
    };
    transmit_lora_frame(
        radio,
        source,
        kind,
        &[],
        indicator,
        health,
        "keepalive tx failed",
    )
    .await;
}

async fn handle_rx_frame(
    frame: LoraFrame<'_>,
    role: FirmwareRole,
    rx_len: u8,
    rssi: i16,
    snr_x4: i16,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    timing: &LoraLinkTiming,
    schedule: &mut BridgeSchedule,
    dialect_cache: &mut RadioStateCache,
) {
    if frame.source != peer_role(role) {
        warn!(
            "lora bridge ignoring frame from unexpected source={:?} kind={:?} payload={=usize}",
            frame.source,
            frame.kind,
            frame.payload.len()
        );
        health.note_unexpected_frame();
        return;
    }

    if frame.kind == LoraFrameKind::Beacon
        || frame.kind == LoraFrameKind::Heartbeat
        || frame.kind == LoraFrameKind::LinkStatus
    {
        schedule.note_peer_rx(timing);
        note_valid_peer_rx(indicator, health, rssi, snr_x4).await;
        return;
    }

    if frame.kind != LoraFrameKind::Data {
        warn!(
            "lora bridge ignoring peer frame kind={:?} payload={=usize}",
            frame.kind,
            frame.payload.len()
        );
        health.note_unexpected_frame();
        return;
    }

    let host_frame =
        match translate::rf_to_host(role, frame.payload, dialect_cache, status_time_ms()) {
            translate::RfToHostDecision::Translated(frame) => frame,
            translate::RfToHostDecision::Handled => {
                schedule.note_peer_rx(timing);
                note_valid_peer_rx(indicator, health, rssi, snr_x4).await;
                return;
            }
            translate::RfToHostDecision::Drop => {
                warn!(
                    "lora bridge rx data dropped payload={=usize} rx_len={=u8}",
                    frame.payload.len(),
                    rx_len
                );
                return;
            }
        };

    if LORA_TO_HOST_CHANNEL.sender().try_send(host_frame).is_err() {
        warn!("lora-to-host bridge channel full; dropping frame");
        return;
    }

    schedule.note_peer_rx(timing);
    note_valid_peer_rx(indicator, health, rssi, snr_x4).await;
    info!(
        "lora bridge data rx bytes={=usize} rssi={=i16} snr_x4={=i16}",
        host_frame.len, rssi, snr_x4
    );
}

async fn run_bridge<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    role: FirmwareRole,
    indicator: StatusIndicatorSender,
    timing: &LoraLinkTiming,
    health: &mut LoraLinkHealth,
) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let source = node_role(role);
    let mut rx_buf = [0u8; MAX_FRAME_LEN];
    let mut schedule = BridgeSchedule::new(role, timing);
    let mut dialect_cache = RadioStateCache::new();

    loop {
        transmit_queued_host_frame(radio, role, source, indicator, health, &mut dialect_cache)
            .await;
        transmit_keepalive_if_due(radio, source, indicator, health, &mut schedule).await;

        if let Err(_) = radio.start_rx_single(timing.rx_window_symbols).await {
            let previous = health.state();
            health.note_rx_error();
            publish_state_change(indicator, previous, health).await;
            recover_rx(radio, indicator, health, "rx start failed").await;
            continue;
        }

        match radio.receive(&mut rx_buf).await {
            Ok(Some(rx)) => match decode_frame(&rx_buf[..rx.len as usize]) {
                Ok(frame) => {
                    handle_rx_frame(
                        frame,
                        role,
                        rx.len,
                        rx.rssi,
                        rx.snr_x4,
                        indicator,
                        health,
                        timing,
                        &mut schedule,
                        &mut dialect_cache,
                    )
                    .await;
                }
                Err(err) => {
                    health.note_malformed_frame();
                    warn!(
                        "lora bridge malformed frame err={:?} rx_len={=u8}",
                        err, rx.len
                    );
                }
            },
            Ok(None) => {
                note_peer_keepalive_missed(indicator, health, Instant::now(), &mut schedule).await;
            }
            Err(err) if err.is_receive_timeout() => {
                note_peer_keepalive_missed(indicator, health, Instant::now(), &mut schedule).await;
            }
            Err(_) => {
                let previous = health.state();
                health.note_rx_error();
                publish_state_change(indicator, previous, health).await;
                warn!("lora bridge rx failed");
                recover_rx(radio, indicator, health, "rx failed").await;
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
        .spawn(lora_bridge_task(bus, pins, watchdog, role))
        .expect("lora bridge task spawn failed");
}
