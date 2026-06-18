use common::comms::links::lora::frame::{
    LoraFrame, LoraFrameKind, LoraNodeRole, MAX_FRAME_LEN, MAX_FRAME_PAYLOAD_LEN, decode_frame,
    encode_frame,
};
use common::comms::links::lora::state::{LoraLinkHealth, LoraLinkPolicy, LoraLinkState};
use common::comms::links::lora::timing::LoraLinkTiming;
use common::comms::links::lora::{LoraProfile, lora_profile_for_preset};
use common::drivers::sx1262::Sx1262;
use common::protocol::hilink;
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
use crate::radio_dialect::airtime::{AirtimeBudget, DownlinkClass};
use crate::radio_dialect::{policy, profile_switch, scheduler, state_cache::RadioStateCache, translate};
use crate::resources::LoraPins;
use crate::watchdog::WatchdogResources;

/// Profile-derived link configuration. `timing` (RX windows / keepalive timeouts) and `budget`
/// (per-class downlink periods) are both pure functions of the RF `profile`, so they are bundled
/// here and recomputed together by [`apply_runtime_profile`] whenever the profile changes. This
/// is what makes the scheduler and link timing track the live RF settings instead of a value
/// frozen at boot.
#[derive(Clone, Copy)]
struct LinkRuntime {
    profile: LoraProfile,
    timing: LoraLinkTiming,
    budget: AirtimeBudget,
}

impl LinkRuntime {
    fn from_profile(profile: LoraProfile) -> Self {
        let timing = LoraLinkTiming::from_profile(
            &profile,
            HILINK_BRIDGE_FRAME_BYTES as u8,
            config::LORA_HEARTBEAT_PERIOD_MS,
        );
        let budget = AirtimeBudget::from_profile(&profile);
        Self {
            profile,
            timing,
            budget,
        }
    }
}

/// Derive the `(preset, frequency, power)` reported to the host as this radio's active profile,
/// mapping the live modulation back to a preset id (`UNKNOWN_PRESET` if it matches none).
fn local_active_fields(profile: &LoraProfile) -> (u8, u32, i8) {
    let preset = hilink::band_plan::preset_for_modulation(
        profile.modulation.sf.factor() as u8,
        profile.modulation.bw.hz(),
    )
    .unwrap_or(hilink::band_plan::UNKNOWN_PRESET);
    (preset, profile.frequency_hz, profile.tx_power_dbm)
}

/// Push the radio's current profile into the cache so the host's RadioStatus reflects it.
fn sync_local_active(cache: &mut RadioStateCache, runtime: &LinkRuntime) {
    let (preset, frequency_hz, tx_power_dbm) = local_active_fields(&runtime.profile);
    cache.set_local_active_profile(preset, frequency_hz, tx_power_dbm);
}

/// Reconfigure the radio onto `new_profile` and recompute every profile-derived link parameter
/// (RX windows, keepalive timeouts, per-class downlink periods). The peer-keepalive deadline is
/// rebased so the freshly-switched peer gets a full timeout to reappear on the new profile.
///
/// Returns `true` on success. The coordinated link-wide switch (see [`drive_profile_switch`])
/// calls this on both ends; here it is the reusable mechanism.
async fn apply_runtime_profile<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    runtime: &mut LinkRuntime,
    schedule: &mut BridgeSchedule,
    new_profile: LoraProfile,
) -> bool
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    match radio.apply_profile(new_profile).await {
        Ok(()) => {
            *runtime = LinkRuntime::from_profile(new_profile);
            schedule.note_peer_rx(&runtime.timing);
            info!(
                "lora bridge applied RF profile: freq={=u32}Hz bw={=u32}Hz power={=i8}dBm airtime_max={=u32}us rx_window_symbols={=u16}",
                runtime.profile.frequency_hz,
                runtime.profile.modulation.bw.hz(),
                runtime.profile.tx_power_dbm,
                runtime.timing.frame_airtime_us,
                runtime.timing.rx_window_symbols
            );
            true
        }
        Err(_) => {
            warn!("lora bridge failed to apply RF profile; staying on current settings");
            false
        }
    }
}

/// Poll the profile-switch FSM and carry out the hardware effect of the returned action. The
/// previous profile is held in `rollback_to` between `Apply` and `Committed`/`Revert` so a
/// switch that the peer never confirms returns the radio to exactly its prior settings — the
/// symmetric rollback that keeps the link recoverable when a change does not take effect on both
/// ends.
async fn drive_profile_switch<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    runtime: &mut LinkRuntime,
    schedule: &mut BridgeSchedule,
    cache: &mut RadioStateCache,
    rollback_to: &mut Option<LoraProfile>,
    now_ms: u32,
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    match cache.poll_profile_switch(now_ms) {
        profile_switch::SwitchAction::None => {}
        profile_switch::SwitchAction::Apply(change) => {
            let Some(target) = change.to_profile() else {
                cache.abort_profile_switch();
                warn!(
                    "profile switch seq={=u16}: target invalid at apply; aborted",
                    change.command_seq
                );
                return;
            };
            *rollback_to = Some(runtime.profile);
            if apply_runtime_profile(radio, runtime, schedule, target).await {
                sync_local_active(cache, runtime);
                info!(
                    "profile switch seq={=u16}: applied target, verifying peer",
                    change.command_seq
                );
            } else {
                if let Some(previous) = rollback_to.take() {
                    let _ = apply_runtime_profile(radio, runtime, schedule, previous).await;
                }
                sync_local_active(cache, runtime);
                cache.abort_profile_switch();
            }
        }
        profile_switch::SwitchAction::Committed(seq) => {
            *rollback_to = None;
            info!("profile switch seq={=u16}: verified link-wide; committed", seq);
        }
        profile_switch::SwitchAction::Revert(seq) => {
            if let Some(previous) = rollback_to.take() {
                let _ = apply_runtime_profile(radio, runtime, schedule, previous).await;
            }
            sync_local_active(cache, runtime);
            warn!(
                "profile switch seq={=u16}: peer not verified; rolled back to previous profile",
                seq
            );
        }
        profile_switch::SwitchAction::Failed(seq) => {
            warn!(
                "profile switch seq={=u16}: peer never acked; aborted on current profile",
                seq
            );
        }
    }
}

/// Idle-fallback watchdog: if no peer frame has arrived for the configured window and this radio
/// has drifted off the boot/"setup" profile, unilaterally return to it.
///
/// The coordinated switch is useless here — the link is gone, so the peer cannot be commanded — but
/// both ends run this same watchdog and re-home to the *same absolute* setup channel, so a
/// genuinely dead link re-rendezvous there with no handshake. It is the last-resort recovery the
/// short-timescale coordinated-switch rollback leaves off (that rollback only restores the
/// *previous* profile; if that one is dead too, this returns all the way to setup). A coordinated
/// switch in flight is left to finish first.
async fn maybe_fallback_to_home<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    runtime: &mut LinkRuntime,
    schedule: &mut BridgeSchedule,
    cache: &mut RadioStateCache,
    rollback_to: &mut Option<LoraProfile>,
    home_profile: LoraProfile,
    home_fields: (u8, u32, i8),
) where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    if cache.profile_switch_active() {
        return;
    }
    let idle_window = Duration::from_millis(u64::from(cache.idle_fallback_ms()));
    if Instant::now() < schedule.last_peer_activity_at + idle_window {
        return;
    }
    // Already on the setup profile → nothing to recover. Rebase the timer so we neither re-check nor
    // re-log every loop while genuinely idle at home.
    if local_active_fields(&runtime.profile) == home_fields {
        schedule.last_peer_activity_at = Instant::now();
        return;
    }

    warn!(
        "lora bridge idle for {=u32}ms with no peer; returning to setup profile freq={=u32}Hz",
        cache.idle_fallback_ms(),
        home_profile.frequency_hz
    );
    // A unilateral re-home, not a coordinated switch: drop any captured rollback target and abort a
    // stale switch so neither can fight the recovery.
    *rollback_to = None;
    cache.abort_profile_switch();
    apply_runtime_profile(radio, runtime, schedule, home_profile).await;
    sync_local_active(cache, runtime);
    // Rebase the window regardless of whether the apply succeeded: on success `apply_runtime_profile`
    // already did (via `note_peer_rx`); on failure this prevents a tight re-home/log loop while the
    // radio is wedged (the rx-error path handles actual recovery).
    schedule.last_peer_activity_at = Instant::now();
}

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

    // Boot on the default preset, built through the same (preset, frequency, power) path a
    // ground-commanded switch uses, so there is one way profiles come into existence.
    let active_profile = lora_profile_for_preset(
        config::DEFAULT_PRESET,
        config::LORA_FREQUENCY_HZ,
        config::DEFAULT_TX_POWER_DBM,
    )
    .unwrap_or(config::LORA_PROFILE);
    let radio = Sx1262::new(
        spi_device,
        reset,
        busy,
        dio1,
        txen,
        rxen,
        active_profile,
        Delay,
    )
    .await;
    let mut radio = match radio {
        Ok(radio) => radio,
        Err(_) => {
            warn!("sx1262 init failed; lora bridge stopped");
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    // Bundle the profile with its derived timing + airtime budget. Recomputed wholesale by
    // `apply_runtime_profile` on a profile change so the scheduler/timing never drift from the RF.
    let mut runtime = LinkRuntime::from_profile(*radio.profile());
    let policy = LoraLinkPolicy::new(
        runtime.timing.degraded_after_misses,
        runtime.timing.lost_after_misses,
        runtime.timing.radio_fault_recovery_limit,
    );
    let mut health = LoraLinkHealth::new(policy);
    publish_state(indicator, health.state()).await;

    info!(
        "sx1262 lora bridge ready: role={:?} freq={=u32}Hz bw={=u32}Hz power={=i8}dBm spi={=u32}Hz airtime_max={=u32}us rx_window_symbols={=u16}",
        role,
        runtime.profile.frequency_hz,
        runtime.profile.modulation.bw.hz(),
        runtime.profile.tx_power_dbm,
        config::LORA_SPI_FREQUENCY_HZ,
        runtime.timing.frame_airtime_us,
        runtime.timing.rx_window_symbols
    );
    info!("lora downlink airtime budget: {:?}", runtime.budget);
    if config::AMATEUR_CALLSIGN.is_empty() {
        warn!("amateur callsign missing; station ID frames disabled");
    } else {
        info!(
            "amateur station ID enabled callsign={=str} interval_ms={=u64}",
            config::AMATEUR_CALLSIGN,
            config::LORA_STATION_ID_PERIOD_MS
        );
    }

    // The boot profile *is* the "setup"/home profile the idle-fallback watchdog returns to.
    run_bridge(
        &mut radio,
        role,
        indicator,
        &mut runtime,
        &mut health,
        active_profile,
    )
    .await;
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
    station_id_period: Duration,
    next_station_id_at: Instant,
    /// Last time a valid peer frame was decoded. The idle-fallback watchdog re-homes to the setup
    /// profile once this is older than the configured window. Seeded at boot so a never-linked
    /// radio still counts down (it boots on the setup profile, so re-homing is then a no-op).
    last_peer_activity_at: Instant,
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
            station_id_period: Duration::from_millis(config::LORA_STATION_ID_PERIOD_MS),
            next_station_id_at: now,
            last_peer_activity_at: now,
        }
    }

    fn note_peer_rx(&mut self, timing: &LoraLinkTiming) {
        let now = Instant::now();
        self.next_peer_keepalive_due_at = now + Duration::from_millis(timing.peer_timeout_ms);
        self.last_peer_activity_at = now;
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
) -> bool
where
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
            return false;
        }
    };

    match radio.transmit(&tx_buf[..tx_len]).await {
        Ok(()) => {
            health.note_tx_packet();
            if kind == LoraFrameKind::Data {
                info!("lora bridge data tx bytes={=usize}", payload.len());
            } else if kind == LoraFrameKind::StationId {
                info!("lora bridge station ID tx bytes={=usize}", payload.len());
            }
            true
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
            false
        }
    }
}

fn station_id_payload(out: &mut [u8; MAX_FRAME_PAYLOAD_LEN]) -> Option<&[u8]> {
    const PREFIX: &[u8] = b"DE ";

    let callsign = config::AMATEUR_CALLSIGN.as_bytes();
    if callsign.is_empty() || PREFIX.len() + callsign.len() > out.len() {
        return None;
    }

    out[..PREFIX.len()].copy_from_slice(PREFIX);
    out[PREFIX.len()..PREFIX.len() + callsign.len()].copy_from_slice(callsign);
    Some(&out[..PREFIX.len() + callsign.len()])
}

async fn transmit_station_id_if_due<SPI, CTRL, WAIT>(
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
    if now < schedule.next_station_id_at {
        return;
    }

    let mut payload = [0u8; MAX_FRAME_PAYLOAD_LEN];
    let Some(payload) = station_id_payload(&mut payload) else {
        schedule.next_station_id_at = now + schedule.station_id_period;
        return;
    };

    let sent = transmit_lora_frame(
        radio,
        source,
        LoraFrameKind::StationId,
        payload,
        indicator,
        health,
        "station ID tx failed",
    )
    .await;

    schedule.next_station_id_at = Instant::now()
        + if sent {
            schedule.station_id_period
        } else {
            Duration::from_millis(5_000)
        };
}

async fn transmit_queued_host_frame<SPI, CTRL, WAIT>(
    radio: &mut Sx1262<SPI, CTRL, WAIT, Delay>,
    role: FirmwareRole,
    source: LoraNodeRole,
    indicator: StatusIndicatorSender,
    health: &mut LoraLinkHealth,
    budget: &AirtimeBudget,
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
        budget.period_ms(DownlinkClass::LinkStatus),
        health.stats(),
        0,
        policy::DEFAULT_TELEMETRY_RATE_HZ,
    );

    let Some(payload) = scheduler::select_scheduled_rf_frame(role, dialect_cache, budget, now_ms)
    else {
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

    // Any decoded peer frame proves the peer is on our current profile (a mismatched SF/BW/freq
    // would not demodulate), which is exactly the verification the profile-switch FSM waits for.
    dialect_cache.note_switch_peer_seen();

    if frame.kind == LoraFrameKind::Beacon
        || frame.kind == LoraFrameKind::Heartbeat
        || frame.kind == LoraFrameKind::LinkStatus
        || frame.kind == LoraFrameKind::StationId
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
    runtime: &mut LinkRuntime,
    health: &mut LoraLinkHealth,
    home_profile: LoraProfile,
) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
{
    let source = node_role(role);
    let mut rx_buf = [0u8; MAX_FRAME_LEN];
    let mut schedule = BridgeSchedule::new(role, &runtime.timing);
    let mut dialect_cache = RadioStateCache::new();
    sync_local_active(&mut dialect_cache, runtime);
    // Profile captured at apply time so a switch the peer never confirms can be rolled back.
    let mut rollback_to: Option<LoraProfile> = None;
    // The setup-profile identity (preset, freq, power) the idle-fallback watchdog homes to.
    let home_fields = local_active_fields(&home_profile);

    loop {
        drive_profile_switch(
            radio,
            runtime,
            &mut schedule,
            &mut dialect_cache,
            &mut rollback_to,
            status_time_ms(),
        )
        .await;
        maybe_fallback_to_home(
            radio,
            runtime,
            &mut schedule,
            &mut dialect_cache,
            &mut rollback_to,
            home_profile,
            home_fields,
        )
        .await;
        transmit_station_id_if_due(radio, source, indicator, health, &mut schedule).await;
        transmit_queued_host_frame(
            radio,
            role,
            source,
            indicator,
            health,
            &runtime.budget,
            &mut dialect_cache,
        )
        .await;
        transmit_keepalive_if_due(radio, source, indicator, health, &mut schedule).await;

        if let Err(_) = radio.start_rx_single(runtime.timing.rx_window_symbols).await {
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
                        &runtime.timing,
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
