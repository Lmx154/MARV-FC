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
use common::coms::transport::lora::mac_scheduler::TickClock;
use common::coms::transport::lora::phy_service::{Phy, PhyError, RxIndication};
use common::coms::transport::lora::link_transport::{LoraTransport, RxMeta};
use common::protocol::packet::{decode_packet, encode_packet_fixed, Packet, PacketType};

pub const LED_QUEUE_LEN: usize = 16;
pub const UART_PACKET_QUEUE_LEN: usize = 32;

const DEFAULT_TICK_PULSE_US: u64 = 50;
const DEFAULT_LQ_LOG_INTERVAL: u32 = 100;
const DEFAULT_TX_LOG_INTERVAL: u32 = 50;
const DEFAULT_RX_LATE_LOG_INTERVAL: u32 = 50;
const DEFAULT_TX_REFUSE_LOG_INTERVAL: u32 = 50;

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
    pub tick_pulse_us: u64,
    pub lq_log_interval: u32,
    pub tx_log_interval: u32,
    pub rx_late_log_interval: u32,
    pub tx_refuse_log_interval: u32,
}

impl Default for MacEngineConfig {
    fn default() -> Self {
        Self {
            tick_pulse_us: DEFAULT_TICK_PULSE_US,
            lq_log_interval: DEFAULT_LQ_LOG_INTERVAL,
            tx_log_interval: DEFAULT_TX_LOG_INTERVAL,
            rx_late_log_interval: DEFAULT_RX_LATE_LOG_INTERVAL,
            tx_refuse_log_interval: DEFAULT_TX_REFUSE_LOG_INTERVAL,
        }
    }
}

pub struct MacEngine<const TXQ: usize, const RXQ: usize> {
    phy: Phy<'static, TXQ, RXQ>,
    transport: LoraTransport,
    profile: LinkProfile,
    tick_clock: TickClock,
    tick_seq: u16,
    constraint_violations: u32,
    lq_expected: u32,
    lq_received: u32,
    lq_tick_count: u32,
    tx_logs: u32,
    rx_late_logs: u32,
    tx_refuse_logs: u32,
    last_rx_seq: Option<u16>,
    led_tx: Option<LedSender>,
    uart_tx: Option<Sender<'static, RawMutex, Packet, UART_PACKET_QUEUE_LEN>>,
    uart_drops: u32,
    tick_pin: Option<Output<'static>>,
    slot_rx_symbols: u16,
    config: MacEngineConfig,
}

impl<const TXQ: usize, const RXQ: usize> MacEngine<TXQ, RXQ> {
    pub fn new(
        phy: Phy<'static, TXQ, RXQ>,
        transport: LoraTransport,
        profile: LinkProfile,
        tick_pin: Option<Output<'static>>,
        led_tx: Option<LedSender>,
        uart_tx: Option<Sender<'static, RawMutex, Packet, UART_PACKET_QUEUE_LEN>>,
        slot_rx_symbols: u16,
        config: MacEngineConfig,
    ) -> Self {
        let now_us = Instant::now().as_micros();
        let tick_clock = TickClock::new(now_us, profile.tick_period_us());
        Self {
            phy,
            transport,
            profile,
            tick_clock,
            tick_seq: 0,
            constraint_violations: 0,
            lq_expected: 0,
            lq_received: 0,
            lq_tick_count: 0,
            tx_logs: 0,
            rx_late_logs: 0,
            tx_refuse_logs: 0,
            last_rx_seq: None,
            led_tx,
            uart_tx,
            uart_drops: 0,
            tick_pin,
            slot_rx_symbols,
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
            let wait_fut = Timer::after_micros(wait_us);

            match select::select(rx_fut, wait_fut).await {
                select::Either::First(rx) => self.handle_rx(rx).await,
                select::Either::Second(_) => self.handle_tick().await,
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
                match header.frame_type {
                    FrameType::ControlDown => {
                        info!(
                            "RX CONTROL_DOWN tick={} len={} rssi={} snr={}",
                            header.tick_seq,
                            payload.len(),
                            rx.rssi,
                            rx.snr
                        );
                        if self.last_rx_seq != Some(header.tick_seq) {
                            self.last_rx_seq = Some(header.tick_seq);
                            self.lq_received = self.lq_received.wrapping_add(1);
                        }
                    }
                    _ => {
                        warn!(
                            "RX unexpected frame type={} tick={} len={} rssi={} snr={}",
                            header.frame_type.name(),
                            header.tick_seq,
                            payload.len(),
                            rx.rssi,
                            rx.snr
                        );
                    }
                }

                match decode_packet(payload) {
                    Ok(packet) => {
                        info!(
                            "RX packet type={} len={}",
                            packet.packet_type.name(),
                            packet.payload.len()
                        );
                        self.transport.on_downlink_rx(&packet, meta);
                        if let Some(uart_tx) = self.uart_tx.as_ref() {
                            if uart_tx.try_send(packet).is_err() {
                                self.uart_drops = self.uart_drops.wrapping_add(1);
                                if self.uart_drops == 1 || self.uart_drops % 50 == 0 {
                                    warn!("UART TX drop count={}", self.uart_drops);
                                }
                            }
                        }
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

        self.tick_seq = self.tick_seq.wrapping_add(1);
        self.lq_tick_count = self.lq_tick_count.wrapping_add(1);
        if self.tick_seq % self.profile.slot_ratio_r == 0 {
            self.lq_expected = self.lq_expected.wrapping_add(1);
            self.phy.arm_rx(self.slot_rx_symbols);
            let rx_ready_deadline_us = tick_start_us
                .saturating_add(self.profile.dl_tx_offset_us)
                .saturating_sub(self.profile.rx_ready_guard_us);
            let now_us = Instant::now().as_micros();
            if now_us > rx_ready_deadline_us {
                self.rx_late_logs = self.rx_late_logs.wrapping_add(1);
                let interval = self.config.rx_late_log_interval.max(1);
                if self.rx_late_logs == 1 || self.rx_late_logs % interval == 0 {
                    warn!(
                        "RX arm late for DL slot tick={} now={}us deadline={}us",
                        self.tick_seq,
                        now_us,
                        rx_ready_deadline_us
                    );
                }
            }
            self.maybe_log_lq("DL");
            return;
        }

        self.maybe_log_lq("DL");

        let payload_len = self.profile.uplink_payload_len;
        let now_ms = Instant::now().as_millis();
        let packet = if payload_len == 0 {
            Packet::new(PacketType::KeepAlive)
        } else {
            self.transport
                .next_uplink(now_ms, payload_len)
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

        let header = FrameHeader::new(FrameType::ControlUp, self.tick_seq);
        let tx = encode_frame(&header, payload.as_slice());
        if tx.len() < HEADER_LEN {
            warn!("TX encode error");
            return;
        }

        let window = self.tick_clock.window(tick_start_us, self.profile.tx_guard_us);
        let tx_start_us = Instant::now().as_micros();
        let duration_us = self.profile.uplink_toa_us;
        if !window.fits_tx(tx_start_us, duration_us) {
            self.constraint_violations = self.constraint_violations.wrapping_add(1);
            self.send_led(LedEvent::Error).await;
            self.tx_refuse_logs = self.tx_refuse_logs.wrapping_add(1);
            let interval = self.config.tx_refuse_log_interval.max(1);
            if self.tx_refuse_logs == 1 || self.tx_refuse_logs % interval == 0 {
                warn!(
                    "Refusing TX: ToA={}ms > SlotBudget={}ms (violations={})",
                    duration_us / 1000,
                    window.budget_us_from(tx_start_us) / 1000,
                    self.constraint_violations
                );
            }
            return;
        }

        match self.phy.tx(tx.as_slice()).await {
            Ok(()) => {
                self.send_led(LedEvent::TxOk).await;
            }
            Err(PhyError::PayloadTooLarge) => {
                warn!("TX payload too large");
            }
        }
        self.tx_logs = self.tx_logs.wrapping_add(1);
        let interval = self.config.tx_log_interval.max(1);
        if self.tx_logs == 1 || self.tx_logs % interval == 0 {
            info!(
                "TX CONTROL_UP tick={} type={}",
                self.tick_seq,
                packet.packet_type.name()
            );
        }
    }

    async fn send_led(&self, event: LedEvent) {
        let Some(led_tx) = self.led_tx.as_ref() else {
            return;
        };
        let _ = led_tx.try_send(event);
    }

    async fn pulse_tick_pin(&mut self) {
        let Some(pin) = self.tick_pin.as_mut() else {
            return;
        };
        pin.set_high();
        Timer::after_micros(self.config.tick_pulse_us).await;
        pin.set_low();
    }

    fn maybe_log_lq(&mut self, label: &str) {
        let interval = self.config.lq_log_interval.max(1);
        if self.lq_tick_count < interval {
            return;
        }

        let expected = self.lq_expected.max(1);
        let received = self.lq_received.min(expected);
        let lost = expected.saturating_sub(received);
        let lq = ((received as u64 * 100) / expected as u64) as u8;
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
