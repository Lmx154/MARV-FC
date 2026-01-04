use defmt::{info, warn};
use embassy_futures::select;
use embassy_rp::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Instant, Timer};

use common::coms::transport::lora::link_profile::LinkProfile;
use common::coms::transport::lora::mac_codec::{
    decode_frame, encode_frame, FrameHeader, FrameType, HEADER_LEN,
};
use common::coms::transport::lora::mac_scheduler::{
    LinkEvent, TickClock, TickTracker, DEFAULT_LOCK_TIMEOUT_MS,
};
use common::coms::transport::lora::phy_service::{Phy, PhyError, RxIndication};
use common::coms::transport::lora::link_transport::{LoraTransport, RxMeta};
use common::protocol::packet::{decode_packet, encode_packet_fixed, Packet, PacketType};

pub const LED_QUEUE_LEN: usize = 16;

const DEFAULT_SYNC_POLL_MS: u64 = 100;
const DEFAULT_TICK_PULSE_US: u64 = 50;
const DEFAULT_TICK_LOG_INTERVAL: u32 = 50;
const DEFAULT_LQ_LOG_INTERVAL: u32 = 100;

#[derive(Clone, Copy)]
pub enum LedEvent {
    Tick,
    RxOk,
    TxOk,
    Error,
}

pub type LedSender = Sender<'static, RawMutex, LedEvent, LED_QUEUE_LEN>;

#[derive(Clone, Copy, Debug)]
pub struct MacEngineConfig {
    pub sync_poll_ms: u64,
    pub tick_pulse_us: u64,
    pub tick_log_interval: u32,
    pub lq_log_interval: u32,
    pub lock_timeout_ms: u64,
}

impl Default for MacEngineConfig {
    fn default() -> Self {
        Self {
            sync_poll_ms: DEFAULT_SYNC_POLL_MS,
            tick_pulse_us: DEFAULT_TICK_PULSE_US,
            tick_log_interval: DEFAULT_TICK_LOG_INTERVAL,
            lq_log_interval: DEFAULT_LQ_LOG_INTERVAL,
            lock_timeout_ms: DEFAULT_LOCK_TIMEOUT_MS,
        }
    }
}

pub struct MacEngine<const TXQ: usize, const RXQ: usize> {
    phy: Phy<'static, TXQ, RXQ>,
    transport: LoraTransport,
    profile: LinkProfile,
    tracker: TickTracker,
    tick_clock: TickClock,
    next_tick_seq: Option<u16>,
    tick_logs: u32,
    constraint_violations: u32,
    lq_expected: u32,
    lq_received: u32,
    lq_tick_count: u32,
    last_rx_seq: Option<u16>,
    led_tx: Option<LedSender>,
    tick_pin: Option<Output<'static>>,
    config: MacEngineConfig,
}

impl<const TXQ: usize, const RXQ: usize> MacEngine<TXQ, RXQ> {
    pub fn new(
        phy: Phy<'static, TXQ, RXQ>,
        transport: LoraTransport,
        profile: LinkProfile,
        tick_pin: Option<Output<'static>>,
        led_tx: Option<LedSender>,
        config: MacEngineConfig,
    ) -> Self {
        let now_us = Instant::now().as_micros();
        let tick_clock = TickClock::new(now_us, profile.tick_period_us());
        let tracker = TickTracker::new(config.lock_timeout_ms, profile.tick_period_ms());
        Self {
            phy,
            transport,
            profile,
            tracker,
            tick_clock,
            next_tick_seq: None,
            tick_logs: 0,
            constraint_violations: 0,
            lq_expected: 0,
            lq_received: 0,
            lq_tick_count: 0,
            last_rx_seq: None,
            led_tx,
            tick_pin,
            config,
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let now_us = Instant::now().as_micros();
            let wait_us = self
                .tick_clock
                .next_tick_boundary_us()
                .saturating_sub(now_us);
            let rx_fut = self.phy.rx();
            let tick_fut = Timer::after_micros(wait_us);
            let poll_fut = Timer::after_millis(self.config.sync_poll_ms);

            match select::select3(rx_fut, tick_fut, poll_fut).await {
                select::Either3::First(rx) => self.handle_rx(rx).await,
                select::Either3::Second(_) => self.handle_tick().await,
                select::Either3::Third(_) => self.handle_poll().await,
            }
        }
    }

    async fn handle_rx(&mut self, rx: RxIndication) {
        match decode_frame(rx.bytes.as_slice()) {
            Ok((header, payload)) => {
                self.send_led(LedEvent::RxOk).await;
                let meta = RxMeta {
                    rssi: rx.rssi,
                    snr: rx.snr,
                };
                self.transport.note_link_stats(meta);
                if header.frame_type != FrameType::ControlUp {
                    warn!(
                        "RX unexpected frame type={} tick={}",
                        header.frame_type.name(),
                        header.tick_seq
                    );
                    return;
                }

                let rx_toa_us = self.profile.uplink_toa_us;
                let tick_start_us = rx.rx_done_instant_us.saturating_sub(rx_toa_us);
                self.tick_clock.align(tick_start_us);
                self.next_tick_seq = Some(header.tick_seq.wrapping_add(1));

                let prev_seq = self.tracker.last_tick_seq();
                let update = self.tracker.on_uplink_tdma(
                    rx.rx_done_instant_us / 1000,
                    header.tick_seq,
                    self.profile.slot_ratio_r,
                );
                if update.event == LinkEvent::LockAcquired {
                    info!("SYNC LOCKED tick={}", header.tick_seq);
                }
                if update.missed != 0 {
                    warn!(
                        "UL slot gap missed={} total_missed={}",
                        update.missed,
                        self.tracker.missed_ticks()
                    );
                }
                if update.seq_anomaly != 0 {
                    warn!(
                        "TICK seq anomaly delta={} total_seq_anom={}",
                        update.seq_anomaly,
                        self.tracker.seq_anomaly_count()
                    );
                }
                self.tick_logs = self.tick_logs.wrapping_add(1);
                if self.tick_logs % self.config.tick_log_interval == 0 {
                    info!(
                        "TICK rx seq={} missed_total={} rssi={} snr={}",
                        header.tick_seq,
                        self.tracker.missed_ticks(),
                        rx.rssi,
                        rx.snr
                    );
                }

                if prev_seq != Some(header.tick_seq) {
                    self.last_rx_seq = Some(header.tick_seq);
                    self.lq_received = self.lq_received.wrapping_add(1);
                }

                match decode_packet(payload) {
                    Ok(packet) => {
                        self.transport.on_uplink_rx(&packet, meta);
                    }
                    Err(err) => {
                        warn!("RX packet decode error: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
            Err(err) => {
                self.send_led(LedEvent::Error).await;
                warn!("RX L3 decode error: {:?}", defmt::Debug2Format(&err));
            }
        }
    }

    async fn handle_tick(&mut self) {
        let now_us = Instant::now().as_micros();
        let Some(tick_start_us) = self.tick_clock.poll(now_us) else {
            return;
        };
        self.send_led(LedEvent::Tick).await;
        self.pulse_tick_pin().await;

        let Some(tick_seq) = self.next_tick_seq else {
            return;
        };
        self.next_tick_seq = Some(tick_seq.wrapping_add(1));
        self.lq_tick_count = self.lq_tick_count.wrapping_add(1);
        if tick_seq % self.profile.slot_ratio_r != 0 {
            self.lq_expected = self.lq_expected.wrapping_add(1);
        }
        self.maybe_log_lq("UL").await;

        if tick_seq % self.profile.slot_ratio_r != 0 {
            return;
        }

        let payload_len = self.profile.downlink_payload_len;
        let packet = if payload_len == 0 {
            Packet::new(PacketType::KeepAlive)
        } else {
            self.transport
                .next_downlink(payload_len)
                .unwrap_or_else(|| Packet::new(PacketType::KeepAlive))
        };

        let payload = match encode_packet_fixed(&packet, payload_len) {
            Ok(bytes) => bytes,
            Err(err) => {
                warn!(
                    "TX packet encode error type={} err={:?}",
                    packet.packet_type.name(),
                    defmt::Debug2Format(&err)
                );
                return;
            }
        };

        let header = FrameHeader::new(FrameType::ControlDown, tick_seq);
        let tx = encode_frame(&header, payload.as_slice());
        if tx.len() < HEADER_LEN {
            warn!("TX encode error");
            return;
        }

        let window = self.tick_clock.window(tick_start_us, self.profile.tx_guard_us);
        let planned_tx_start_us = tick_start_us.saturating_add(self.profile.dl_tx_offset_us);
        let now_us = Instant::now().as_micros();
        let tx_start_us = planned_tx_start_us.max(now_us);
        let duration_us = self.profile.downlink_toa_us;
        if !window.fits_tx(tx_start_us, duration_us) {
            self.constraint_violations = self.constraint_violations.wrapping_add(1);
            self.send_led(LedEvent::Error).await;
            warn!(
                "Refusing DL TX: ToA={}ms > SlotBudget={}ms (violations={})",
                duration_us / 1000,
                window.budget_us_from(tx_start_us) / 1000,
                self.constraint_violations
            );
            return;
        }

        if tx_start_us > now_us {
            Timer::after_micros(tx_start_us - now_us).await;
        }
        match self.phy.tx(tx.as_slice()).await {
            Ok(()) => {
                self.send_led(LedEvent::TxOk).await;
            }
            Err(PhyError::PayloadTooLarge) => {
                warn!("TX payload too large");
            }
        }
        info!(
            "TX CONTROL_DOWN tick={} type={}",
            tick_seq,
            packet.packet_type.name()
        );
    }

    async fn handle_poll(&mut self) {
        if self.tracker.poll(Instant::now().as_millis()) == LinkEvent::LockLost {
            info!("SYNC LOST: timeout -> SEARCH");
            self.reset_lq();
        }
    }

    async fn send_led(&self, event: LedEvent) {
        let Some(led_tx) = self.led_tx.as_ref() else {
            return;
        };
        led_tx.send(event).await;
    }

    async fn pulse_tick_pin(&mut self) {
        let Some(pin) = self.tick_pin.as_mut() else {
            return;
        };
        pin.set_high();
        Timer::after_micros(self.config.tick_pulse_us).await;
        pin.set_low();
    }

    fn reset_lq(&mut self) {
        self.lq_expected = 0;
        self.lq_received = 0;
        self.lq_tick_count = 0;
        self.last_rx_seq = None;
    }

    async fn maybe_log_lq(&mut self, label: &str) {
        let interval = self.config.lq_log_interval.max(1);
        if self.lq_tick_count < interval {
            return;
        }

        let expected = self.lq_expected.max(1);
        let received = self.lq_received.min(expected);
        let lost = expected.saturating_sub(received);
        let lq = ((received as u64 * 100) / expected as u64) as u8;
        self.transport.set_lq(lq);
        info!(
            "LQ {} {}% rx={} lost={} window={}",
            label,
            lq,
            received,
            lost,
            expected
        );
        self.lq_expected = 0;
        self.lq_received = 0;
        self.lq_tick_count = 0;
    }
}
