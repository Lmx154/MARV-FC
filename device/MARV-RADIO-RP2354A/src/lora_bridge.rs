use common::comms::links::lora::ACTIVE;
use common::comms::links::lora::frame::{
    LoraFrame, LoraFrameKind, LoraNodeRole, MAX_FRAME_LEN, decode_frame, encode_frame,
};
use common::comms::links::lora::state::{LoraLinkHealth, LoraLinkPolicy, LoraLinkState};
use common::comms::links::lora::stats::LoraLinkStats;
use common::comms::links::lora::timing::LoraLinkTiming;
use common::drivers::sx1262::Sx1262;
use common::protocol::hilink::{self, WirePayload};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::buses::LoraSpiBus;
use crate::channels::{
    HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame, LORA_TO_HOST_CHANNEL, STATUS_INDICATOR_CHANNEL,
    StatusIndicatorEvent, StatusIndicatorSender,
};
use crate::config::{self, FirmwareRole};
use crate::radio_dialect::{rf, scheduler, translate};
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

fn clamp_i16_to_i8(value: i16, min: i8, max: i8) -> i8 {
    value.clamp(min as i16, max as i16) as i8
}

fn delta_u16(current: u32, previous: u32) -> u16 {
    current.wrapping_sub(previous).min(u16::MAX as u32) as u16
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
    link_status_period: Duration,
    next_link_status_at: Instant,
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
        let link_status_period = Duration::from_millis(config::LORA_LINK_STATUS_PERIOD_MS);

        Self {
            keepalive_period,
            next_keepalive_at: now + phase_offset,
            next_peer_keepalive_due_at: now + peer_keepalive_timeout,
            link_status_period,
            next_link_status_at: now + link_status_period,
        }
    }

    fn note_peer_rx(&mut self, timing: &LoraLinkTiming) {
        self.next_peer_keepalive_due_at =
            Instant::now() + Duration::from_millis(timing.peer_timeout_ms);
    }
}

struct LinkStatusEmitter {
    seq: u16,
    last_stats: LoraLinkStats,
}

impl LinkStatusEmitter {
    const fn new() -> Self {
        Self {
            seq: 0,
            last_stats: LoraLinkStats {
                tx_packets: 0,
                rx_packets: 0,
                tx_errors: 0,
                rx_errors: 0,
                malformed_frames: 0,
                unexpected_frames: 0,
                missed_peer_packets: 0,
                idle_timeouts: 0,
                rx_restarts: 0,
                radio_recoveries: 0,
                radio_recovery_failures: 0,
                last_rssi: 0,
                last_snr_x4: 0,
            },
        }
    }

    fn emit(&mut self, role: FirmwareRole, health: &LoraLinkHealth) {
        let stats = *health.stats();
        let (uplink_rssi_dbm, uplink_snr_x4, downlink_rssi_dbm, downlink_snr_x4) =
            link_quality_fields(role, &stats);
        let payload = hilink::LoRaLinkStatusPayload {
            time_ms: status_time_ms(),
            uplink_rssi_dbm,
            uplink_snr_x4,
            downlink_rssi_dbm,
            downlink_snr_x4,
            rx_packets_delta: delta_u16(stats.rx_packets, self.last_stats.rx_packets),
            tx_packets_delta: delta_u16(stats.tx_packets, self.last_stats.tx_packets),
            lost_packets_delta: delta_u16(
                stats.missed_peer_packets,
                self.last_stats.missed_peer_packets,
            ),
            active_profile: hilink::lora_profile::SF7_500,
            telemetry_rate_hz: telemetry_rate_hz(),
            reserved: 0,
        };
        let mut raw = [0u8; hilink::raw_frame_len(hilink::LoRaLinkStatusPayload::WIRE_LEN)];
        let mut encoded = [0u8; hilink::encoded_frame_len(hilink::LoRaLinkStatusPayload::WIRE_LEN)];

        match hilink::encode_packet(&payload, self.seq, payload.time_ms, &mut raw, &mut encoded) {
            Ok(len) => {
                self.seq = self.seq.wrapping_add(1);
                self.last_stats = stats;
                let mut frame = HilinkBridgeFrame::new();
                frame.bytes[..len].copy_from_slice(&encoded[..len]);
                frame.len = len;
                if LORA_TO_HOST_CHANNEL.sender().try_send(frame).is_err() {
                    warn!("lora link status channel full; dropping status frame");
                }
            }
            Err(_) => warn!("lora link status encode failed"),
        }
    }
}

struct HilinkBridgeSmokeTest {
    seq: u16,
    next_ping_at: Instant,
}

impl HilinkBridgeSmokeTest {
    fn new() -> Self {
        Self {
            seq: 0,
            next_ping_at: Instant::now()
                + Duration::from_millis(config::HILINK_BRIDGE_SMOKE_TEST_PERIOD_MS),
        }
    }

    fn next_seq(&mut self) -> u16 {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        seq
    }

    fn note_ping_sent(&mut self, now: Instant) {
        self.next_ping_at = now + Duration::from_millis(config::HILINK_BRIDGE_SMOKE_TEST_PERIOD_MS);
    }
}

fn encode_hilink_smoke_frame<P: WirePayload>(payload: &P, seq: u16) -> Option<HilinkBridgeFrame> {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let mut encoded = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let len = match hilink::encode_packet(payload, seq, status_time_ms(), &mut raw, &mut encoded) {
        Ok(len) => len,
        Err(_) => {
            warn!("hilink bridge smoke-test encode failed");
            return None;
        }
    };

    let mut frame = HilinkBridgeFrame::new();
    frame.bytes[..len].copy_from_slice(&encoded[..len]);
    frame.len = len;
    Some(frame)
}

fn decode_hilink_smoke_ping(payload: &[u8]) -> Option<u16> {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let packet = hilink::decode_packet(payload, &mut raw).ok()?;
    if packet.header.message_type().ok()? != hilink::MsgType::Ping {
        return None;
    }
    hilink::decode_payload::<hilink::PingPayload>(&packet).ok()?;
    Some(packet.header.seq)
}

fn decode_hilink_smoke_pong(payload: &[u8]) -> Option<u16> {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let packet = hilink::decode_packet(payload, &mut raw).ok()?;
    if packet.header.message_type().ok()? != hilink::MsgType::Pong {
        return None;
    }
    hilink::decode_payload::<hilink::PongPayload>(&packet).ok()?;
    Some(packet.header.seq)
}

fn link_quality_fields(role: FirmwareRole, stats: &LoraLinkStats) -> (i8, i8, i8, i8) {
    let rssi = if stats.rx_packets == 0 {
        hilink::lora_scaling::RSSI_MIN_DBM
    } else {
        clamp_i16_to_i8(
            stats.last_rssi,
            hilink::lora_scaling::RSSI_MIN_DBM,
            hilink::lora_scaling::RSSI_MAX_DBM,
        )
    };
    let snr = if stats.rx_packets == 0 {
        hilink::lora_scaling::SNR_MIN_X4
    } else {
        clamp_i16_to_i8(
            stats.last_snr_x4,
            hilink::lora_scaling::SNR_MIN_X4,
            hilink::lora_scaling::SNR_MAX_X4,
        )
    };

    match role {
        FirmwareRole::Radio => (
            hilink::lora_scaling::RSSI_MIN_DBM,
            hilink::lora_scaling::SNR_MIN_X4,
            rssi,
            snr,
        ),
        FirmwareRole::GroundStation => (
            rssi,
            snr,
            hilink::lora_scaling::RSSI_MIN_DBM,
            hilink::lora_scaling::SNR_MIN_X4,
        ),
    }
}

fn telemetry_rate_hz() -> u8 {
    if config::LORA_LINK_STATUS_PERIOD_MS == 0 {
        return 0;
    }

    let rate = 1_000_u64.div_ceil(config::LORA_LINK_STATUS_PERIOD_MS);
    rate.clamp(1, u8::MAX as u64) as u8
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
    source: LoraNodeRole,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let Some(host_frame) = receive_next_host_frame() else {
        return;
    };
    let translate::HostToRfDecision::LegacyPassThrough(payload) =
        translate::host_to_rf_legacy_pass_through(&host_frame);

    transmit_lora_frame(
        radio,
        source,
        LoraFrameKind::Data,
        payload,
        indicator,
        health,
        "data tx failed",
    )
    .await;
}

async fn transmit_hilink_smoke_ping_if_due<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    role: FirmwareRole,
    source: LoraNodeRole,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    smoke_test: &mut HilinkBridgeSmokeTest,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    if !config::HILINK_BRIDGE_SMOKE_TEST_ENABLED || role != FirmwareRole::Radio {
        return;
    }

    let now = Instant::now();
    if now < smoke_test.next_ping_at {
        return;
    }

    smoke_test.note_ping_sent(now);
    let seq = smoke_test.next_seq();
    let Some(frame) = encode_hilink_smoke_frame(&hilink::PingPayload, seq) else {
        return;
    };

    info!(
        "hilink bridge smoke-test ping tx seq={=u16} bytes={=usize}",
        seq, frame.len
    );
    transmit_lora_frame(
        radio,
        source,
        LoraFrameKind::Data,
        frame.as_slice(),
        indicator,
        health,
        "hilink smoke-test ping tx failed",
    )
    .await;
}

fn prepare_hilink_smoke_pong(
    role: FirmwareRole,
    frame: LoraFrame<'_>,
    smoke_test: &mut HilinkBridgeSmokeTest,
) -> Option<(u16, u16, HilinkBridgeFrame)> {
    if !config::HILINK_BRIDGE_SMOKE_TEST_ENABLED
        || role != FirmwareRole::GroundStation
        || frame.source != LoraNodeRole::Radio
        || frame.kind != LoraFrameKind::Data
    {
        return None;
    }

    let peer_seq = decode_hilink_smoke_ping(frame.payload)?;
    let reply_seq = smoke_test.next_seq();
    let reply = encode_hilink_smoke_frame(&hilink::PongPayload, reply_seq)?;
    Some((peer_seq, reply_seq, reply))
}

fn note_hilink_smoke_rx(role: FirmwareRole, frame: LoraFrame<'_>) {
    if !config::HILINK_BRIDGE_SMOKE_TEST_ENABLED || frame.kind != LoraFrameKind::Data {
        return;
    }

    match role {
        FirmwareRole::Radio if frame.source == LoraNodeRole::GroundStation => {
            if let Some(seq) = decode_hilink_smoke_pong(frame.payload) {
                info!("hilink bridge smoke-test pong rx seq={=u16}", seq);
            }
        }
        FirmwareRole::GroundStation if frame.source == LoraNodeRole::Radio => {
            if let Some(seq) = decode_hilink_smoke_ping(frame.payload) {
                info!("hilink bridge smoke-test ping rx seq={=u16}", seq);
            }
        }
        _ => {}
    }
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

async fn emit_link_status_if_due(
    role: FirmwareRole,
    health: &LoraLinkHealth,
    schedule: &mut BridgeSchedule,
    emitter: &mut LinkStatusEmitter,
) {
    let now = Instant::now();
    if now < schedule.next_link_status_at {
        return;
    }

    schedule.next_link_status_at = now + schedule.link_status_period;
    emitter.emit(role, health);
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

    let mut host_frame = HilinkBridgeFrame::new();
    if rf::copy_legacy_payload_to_hilink_frame(frame.payload, &mut host_frame).is_err() {
        warn!(
            "lora bridge rx data too large payload={=usize} rx_len={=u8}",
            frame.payload.len(),
            rx_len
        );
        return;
    }

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
    let mut link_status = LinkStatusEmitter::new();
    let mut smoke_test = HilinkBridgeSmokeTest::new();

    loop {
        transmit_queued_host_frame(radio, source, indicator, health).await;
        transmit_hilink_smoke_ping_if_due(radio, role, source, indicator, health, &mut smoke_test)
            .await;
        transmit_keepalive_if_due(radio, source, indicator, health, &mut schedule).await;
        emit_link_status_if_due(role, health, &mut schedule, &mut link_status).await;

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
                    let smoke_pong = prepare_hilink_smoke_pong(role, frame, &mut smoke_test);
                    note_hilink_smoke_rx(role, frame);
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
                    )
                    .await;
                    if let Some((peer_seq, reply_seq, reply)) = smoke_pong {
                        info!(
                            "hilink bridge smoke-test pong tx peer_seq={=u16} seq={=u16} bytes={=usize}",
                            peer_seq, reply_seq, reply.len
                        );
                        transmit_lora_frame(
                            radio,
                            source,
                            LoraFrameKind::Data,
                            reply.as_slice(),
                            indicator,
                            health,
                            "hilink smoke-test pong tx failed",
                        )
                        .await;
                    }
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
