// common/src/lora/link.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn};
use crate::drivers::sx1262::{Sx1262, RawRx, Result as RadioResult};
use crate::log_config;
use crate::utils::delay::DelayMs;

// -----------------------------------------------------------------------------
// Link Layer Types
// -----------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum LinkError {
    Radio,
    Timeout,
    CorruptFrame,
    TooLarge,
}

pub type Result<T> = core::result::Result<T, LinkError>;

// -------------------------------------------------------------------------
// Light-weight link observability + pacing
// -------------------------------------------------------------------------

/// Most recent RSSI/SNR figures observed on RX (data or ACK).
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkStats {
    pub last_rssi_dbm: Option<i16>,
    pub last_snr_x4: Option<i16>,
}

impl LinkStats {
    fn observe_rx(&mut self, raw: &RawRx) {
        self.last_rssi_dbm = Some(raw.rssi);
        self.last_snr_x4 = Some(raw.snr_x4);
    }
}

/// Simple gap controller to slow down TX when link quality is poor and speed
/// back up when it looks healthy.
#[derive(Clone, Copy, Debug)]
pub struct TxPacer {
    pub min_gap_ms: u32,
    pub max_gap_ms: u32,
    pub step_ms: u32,
    pub snr_good_x4: i16,
    pub snr_bad_x4: i16,
    current_gap_ms: u32,
}

impl TxPacer {
    pub const fn new(
        min_gap_ms: u32,
        max_gap_ms: u32,
        step_ms: u32,
        snr_good_x4: i16,
        snr_bad_x4: i16,
    ) -> Self {
        Self {
            min_gap_ms,
            max_gap_ms,
            step_ms,
            snr_good_x4,
            snr_bad_x4,
            current_gap_ms: min_gap_ms,
        }
    }

    pub fn on_ack(&mut self, stats: &LinkStats) {
        if let Some(snr_x4) = stats.last_snr_x4 {
            if snr_x4 >= self.snr_good_x4 {
                self.current_gap_ms =
                    (self.current_gap_ms.saturating_sub(self.step_ms)).max(self.min_gap_ms);
                return;
            }
            if snr_x4 <= self.snr_bad_x4 {
                self.current_gap_ms =
                    (self.current_gap_ms + self.step_ms).min(self.max_gap_ms);
                return;
            }
        }

        // No SNR yet: gently ease toward min to avoid stalling.
        self.current_gap_ms =
            (self.current_gap_ms.saturating_sub(self.step_ms)).max(self.min_gap_ms);
    }

    pub fn on_timeout(&mut self) {
        self.current_gap_ms = (self.current_gap_ms + self.step_ms).min(self.max_gap_ms);
    }

    pub fn recommended_gap_ms(&self) -> u32 {
        self.current_gap_ms
    }
}

impl Default for TxPacer {
    fn default() -> Self {
        // snr_x4 is SNR * 4. Good ~10 dB, bad ~0 dB by default.
        Self::new(500, 1500, 200, 40, 0)
    }
}

/// LoRaLink header format
///
/// Byte 0: seq
/// Byte 1: ack
/// Byte 2: flags
/// Byte 3: len
///
/// Then payload bytes.
#[derive(Clone, Copy)]
pub struct LinkHeader {
    pub seq: u8,
    pub ack: u8,
    pub flags: u8,
    pub len: u8,
}

impl LinkHeader {
    pub const SIZE: usize = 4;

    pub fn encode(&self, out: &mut [u8]) {
        out[0] = self.seq;
        out[1] = self.ack;
        out[2] = self.flags;
        out[3] = self.len;
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        Some(Self {
            seq: buf[0],
            ack: buf[1],
            flags: buf[2],
            len: buf[3],
        })
    }
}

// Flags
pub const FLAG_ACK: u8 = 0x01;

// -----------------------------------------------------------------------------
// LoRaLink Implementation
// -----------------------------------------------------------------------------

/// LoRaLink Layer — Reliable, async datagrams.
///
/// Relies on SX1262 raw driver underneath.
pub struct LoRaLink<'a, RADIO> {
    radio: &'a mut RADIO,
    next_seq: u8,
    expected_seq: u8,

    /// How long to wait for ACK after TX
    pub ack_timeout_ms: u32,

    /// How many times to retry if ACK not received
    pub retries: usize,

    /// Last-observed RSSI/SNR from the radio.
    stats: LinkStats,

    /// Simple TX pacing helper (tune in LoRaConfig if desired).
    pub pacer: TxPacer,
}

impl<'a, RADIO> LoRaLink<'a, RADIO>
where
    RADIO: Sx1262Interface,
{
    /// Max payload size in bytes (not including LoRaLink header).
    /// 4-byte header + 240-byte payload <= 255-byte SX1262 limit.
    pub const MTU: usize = 240;

    pub fn mtu(&self) -> usize {
        Self::MTU
    }

    /// Suggested inter-TX delay based on recent link quality observations.
    pub fn recommended_tx_gap_ms(&self) -> u32 {
        self.pacer.recommended_gap_ms()
    }

    /// Last observed RSSI/SNR values.
    pub fn stats(&self) -> LinkStats {
        self.stats
    }

    pub fn new(radio: &'a mut RADIO) -> Self {
        Self {
            radio,
            next_seq: 1,
            expected_seq: 1,
            ack_timeout_ms: 200,
            retries: 4,
            stats: LinkStats::default(),
            pacer: TxPacer::default(),
        }
    }

    pub async fn start_rx(&mut self, delay: &mut impl DelayMs) -> Result<()> {
        self.radio
            .start_rx_continuous(delay)
            .await
            .map_err(|_| LinkError::Radio)
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// Send a reliable frame (payload only).
    ///
    /// Stop-and-wait ARQ:
    ///  - TX data frame
    ///  - Wait for ACK
    ///  - Retry up to self.retries
    pub async fn send(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> Result<()> {
        if payload.len() > Self::MTU {
            return Err(LinkError::TooLarge);
        }

        let seq = self.next_seq;
        self.next_seq = seq.wrapping_add(1);

        // Construct packet
        let mut frame = [0u8; 255];

        let header = LinkHeader {
            seq,
            ack: 0,
            flags: 0,
            len: payload.len() as u8,
        };

        header.encode(&mut frame);
        frame[LinkHeader::SIZE..LinkHeader::SIZE + payload.len()]
            .copy_from_slice(payload);

        // TX + WAIT FOR ACK
        for attempt in 0..=self.retries {
            if log_config::LOG_LINK_TRAFFIC {
                info!("LoRaLink TX seq={} attempt={}", seq, attempt + 1);
            }

            self.radio
                .tx_raw(
                    delay,
                    &frame[..header.len as usize + LinkHeader::SIZE],
                )
                .await
                .map_err(|_| LinkError::Radio)?;

            // After TX, immediately switch to RX
            self.radio
                .start_rx_continuous(delay)
                .await
                .map_err(|_| LinkError::Radio)?;

            // Wait for ACK
            let mut elapsed = 0;
            let mut buf = [0u8; 255];

            while elapsed < self.ack_timeout_ms {
                if let Some(rx) = self
                    .radio
                    .poll_raw(delay, &mut buf)
                    .await
                    .map_err(|_| LinkError::Radio)?
                {
                    self.stats.observe_rx(&rx);

                    if rx.len >= LinkHeader::SIZE as u8 {
                        if let Some(h) = LinkHeader::decode(&buf[..4]) {
                            if (h.flags & FLAG_ACK) != 0 && h.ack == seq {
                                if log_config::LOG_LINK_ACKS {
                                    info!("LoRaLink ACK received for seq={}", seq);
                                }
                                self.pacer.on_ack(&self.stats);
                                return Ok(());
                            }
                        }
                    }
                }

                delay.delay_ms(10).await;
                elapsed += 10;
            }

            warn!(
                "LoRaLink: ACK timeout seq={} (attempt {})",
                seq,
                attempt + 1
            );
        }

        self.pacer.on_timeout();
        Err(LinkError::Timeout)
    }

    /// Blocking receive: waits until a valid frame and returns its payload len.
    ///
    /// This is built on top of `try_recv`, which does a single poll step.
    pub async fn recv(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> Result<usize> {
        loop {
            if let Some(len) = self.try_recv(delay, buf).await? {
                return Ok(len);
            }

            // No frame yet – back off a bit.
            delay.delay_ms(10).await;
        }
    }

    /// Non-blocking receive helper.
    ///
    /// - Returns Ok(Some(len)) if a valid frame was received (and ACKed).
    /// - Returns Ok(None) if no frame is available right now.
    /// - Returns Err(_) on radio / framing errors.
    ///
    /// This is what you’ll want to use inside a higher-level MAVLink task.
    pub async fn try_recv(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> Result<Option<usize>> {
        let mut rx_buf = [0u8; 255];

        // Single poll
        let raw = match self
            .radio
            .poll_raw(delay, &mut rx_buf)
            .await
            .map_err(|_| LinkError::Radio)?
        {
            Some(r) => r,
            None => return Ok(None),
        };
        self.stats.observe_rx(&raw);

        if raw.len < LinkHeader::SIZE as u8 {
            // Too short to be a valid frame; drop.
            return Ok(None);
        }

        let Some(h) = LinkHeader::decode(&rx_buf[..4]) else {
            // Malformed header; drop.
            return Ok(None);
        };

        // ACK frames are control traffic for send() and should not surface
        // to higher layers (e.g. MAVLink decoding). Drop them here.
        if (h.flags & FLAG_ACK) != 0 {
            return Ok(None);
        }

        let payload_len = h.len as usize;
        if payload_len > buf.len() || payload_len + LinkHeader::SIZE > raw.len as usize {
            return Err(LinkError::CorruptFrame);
        }

        // Expected sequence?
        if h.seq != self.expected_seq {
            if log_config::LOG_LINK_ORDER_WARN {
                warn!(
                    "LoRaLink: out-of-order seq={} expected={}",
                    h.seq,
                    self.expected_seq
                );
            }
        } else {
            self.expected_seq = self.expected_seq.wrapping_add(1);
        }

        // Copy payload for caller
        buf[..payload_len]
            .copy_from_slice(&rx_buf[LinkHeader::SIZE..LinkHeader::SIZE + payload_len]);

        // Send ACK
        let mut ack_frame = [0u8; LinkHeader::SIZE];
        let ack_header = LinkHeader {
            seq: 0,
            ack: h.seq,
            flags: FLAG_ACK,
            len: 0,
        };
        ack_header.encode(&mut ack_frame);

        if log_config::LOG_LINK_ACKS {
            info!("LoRaLink: Sending ACK for seq={}", h.seq);
        }
        self.radio
            .tx_raw(delay, &ack_frame)
            .await
            .map_err(|_| LinkError::Radio)?;

        // IMPORTANT: After transmitting an ACK, return to RX mode so we
        // continue to receive future frames.
        self.radio
            .start_rx_continuous(delay)
            .await
            .map_err(|_| LinkError::Radio)?;
        if log_config::LOG_LINK_ACKS {
            info!("LoRaLink: ACK TX done -> entered RX");
        }

        Ok(Some(payload_len))
    }
}

// -----------------------------------------------------------------------------
// Trait so LoRaLink works with your SX1262
// -----------------------------------------------------------------------------

pub trait Sx1262Interface {
    async fn tx_raw(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> RadioResult<()>;

    async fn start_rx_continuous(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> RadioResult<()>;

    async fn poll_raw(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> RadioResult<Option<RawRx>>;
}

// Blanket implementation
impl<SPI, NSS, RESET, BUSY, DIO1, SW> Sx1262Interface
    for Sx1262<SPI, NSS, RESET, BUSY, DIO1, SW>
where
    SPI: embedded_hal_async::spi::SpiBus<u8>,
    NSS: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DIO1: embedded_hal::digital::InputPin,
    SW: crate::drivers::sx1262::RfSwitch,
{
    async fn tx_raw(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> RadioResult<()> {
        self.tx_raw(delay, payload).await
    }

    async fn start_rx_continuous(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> RadioResult<()> {
        self.start_rx_continuous(delay).await
    }

    async fn poll_raw(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> RadioResult<Option<RawRx>> {
        self.poll_raw(delay, buf).await
    }
}

// -----------------------------------------------------------------------------
// Optional Fault Injection Wrapper for Stress Testing
// -----------------------------------------------------------------------------

/// Wraps any Sx1262Interface and can drop RX frames or ACKs
/// deterministically to exercise retries/timeouts.
///
/// Not used by default – but available for dedicated stress builds.
pub struct FaultyRadio<R> {
    inner: R,
    drop_every_nth_rx: u8,
    rx_count: u8,
    drop_every_nth_ack: u8,
    ack_count: u8,
}

impl<R> FaultyRadio<R> {
    pub fn new(inner: R, drop_rx_every: u8, drop_ack_every: u8) -> Self {
        Self {
            inner,
            drop_every_nth_rx: drop_rx_every,
            rx_count: 0,
            drop_every_nth_ack: drop_ack_every,
            ack_count: 0,
        }
    }

    pub fn into_inner(self) -> R {
        self.inner
    }
}

impl<R> Sx1262Interface for FaultyRadio<R>
where
    R: Sx1262Interface,
{
    async fn tx_raw(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> RadioResult<()> {
        // Heuristic: small (len == 4) frames are ACKs
        let is_ack = payload.len() == LinkHeader::SIZE;

        if is_ack && self.drop_every_nth_ack != 0 {
            self.ack_count = self.ack_count.wrapping_add(1);
            if self.ack_count % self.drop_every_nth_ack == 0 {
                warn!("FaultyRadio: DROPPING ACK TX");
                // Pretend success, but do nothing (ACK lost on air).
                return Ok(());
            }
        }

        self.inner.tx_raw(delay, payload).await
    }

    async fn start_rx_continuous(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> RadioResult<()> {
        self.inner.start_rx_continuous(delay).await
    }

    async fn poll_raw(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> RadioResult<Option<RawRx>> {
        let res = self.inner.poll_raw(delay, buf).await?;
        if let Some(raw) = res {
            if self.drop_every_nth_rx != 0 {
                self.rx_count = self.rx_count.wrapping_add(1);
                if self.rx_count % self.drop_every_nth_rx == 0 {
                    warn!("FaultyRadio: DROPPING RX frame");
                    return Ok(None);
                }
            }
            Ok(Some(raw))
        } else {
            Ok(None)
        }
    }
}
