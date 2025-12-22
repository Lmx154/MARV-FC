// common/src/lora/link.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn};
use crate::drivers::sx1262::{Sx1262, RawRx, Result as RadioResult};
use crate::coms::transport::lora::lora_config::LoRaConfig;
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
// Light-weight link observability
// -------------------------------------------------------------------------

/// Most recent RSSI/SNR figures observed on RX (data or ACK).
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkStats {
    pub last_rssi_dbm: Option<i16>,
    pub last_snr_x4: Option<i16>,
    pub rx_packets: u32,
    pub tx_packets: u32,
}

impl LinkStats {
    fn observe_rx(&mut self, raw: &RawRx) {
        self.last_rssi_dbm = Some(raw.rssi);
        self.last_snr_x4 = Some(raw.snr_x4);
        self.rx_packets = self.rx_packets.wrapping_add(1);
    }
}

/// LoRaLink header format
///
/// Byte 0: seq
/// Byte 1: flags
/// Byte 2: len
/// Byte 3: reserved (0)
///
/// Then payload bytes.
#[derive(Clone, Copy)]
pub struct LinkHeader {
    pub seq: u8,
    pub flags: u8,
    pub len: u8,
    pub reserved: u8,
}

impl LinkHeader {
    pub const SIZE: usize = 4;

    pub fn encode(&self, out: &mut [u8]) {
        out[0] = self.seq;
        out[1] = self.flags;
        out[2] = self.len;
        out[3] = self.reserved;
    }

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        Some(Self {
            seq: buf[0],
            flags: buf[1],
            len: buf[2],
            reserved: buf[3],
        })
    }
}

// Flags (reserved for future use). Kept for wire compatibility.
pub const FLAG_NO_ACK: u8 = 0x02;

// -----------------------------------------------------------------------------
// LoRaLink Implementation
// -----------------------------------------------------------------------------

/// LoRaLink Layer — async datagrams (no link-level ARQ).
///
/// This is intentionally best-effort: reliability (if needed) is handled above the link.
/// Relies on the SX1262 raw driver underneath.
pub struct LoRaLink<'a, RADIO> {
    radio: &'a mut RADIO,
    next_seq: u8,
    expected_seq: u8,

    /// Last-observed RSSI/SNR from the radio.
    stats: LinkStats,
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

    /// Last observed RSSI/SNR values.
    pub fn stats(&self) -> LinkStats {
        self.stats
    }

    pub fn new(radio: &'a mut RADIO) -> Self {
        Self {
            radio,
            next_seq: 1,
            expected_seq: 1,
            stats: LinkStats::default(),
        }
    }

    pub async fn start_rx(&mut self, delay: &mut impl DelayMs) -> Result<()> {
        self.radio
            .start_rx_continuous(delay)
            .await
            .map_err(|_| LinkError::Radio)
    }

    /// Apply a new radio configuration at runtime.
    ///
    /// Intended for staged RF reconfiguration.
    pub async fn apply_radio_config(
        &mut self,
        delay: &mut impl DelayMs,
        cfg: LoRaConfig,
    ) -> Result<()> {
        self.radio
            .apply_lora_config(delay, cfg)
            .await
            .map_err(|_| LinkError::Radio)
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// Send an unreliable frame (payload only).
    ///
    /// Best-effort:
    ///  - TX data frame
    ///  - No ACK, no retries
    pub async fn send_unreliable(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> Result<()> {
        if payload.len() > Self::MTU {
            return Err(LinkError::TooLarge);
        }

        let seq = self.next_seq;
        self.next_seq = seq.wrapping_add(1);

        let mut frame = [0u8; 255];
        let header = LinkHeader {
            seq,
            flags: FLAG_NO_ACK,
            len: payload.len() as u8,
            reserved: 0,
        };
        header.encode(&mut frame);
        frame[LinkHeader::SIZE..LinkHeader::SIZE + payload.len()].copy_from_slice(payload);

        if log_config::LOG_LINK_TRAFFIC {
            info!("LoRaLink TX (unreliable) seq={} len={}", seq, payload.len());
        }

        self.radio
            .tx_raw(delay, &frame[..header.len as usize + LinkHeader::SIZE])
            .await
            .map_err(|_| LinkError::Radio)?;

        self.stats.tx_packets = self.stats.tx_packets.wrapping_add(1);

        // Return to RX after TX.
        self.radio
            .start_rx_continuous(delay)
            .await
            .map_err(|_| LinkError::Radio)?;

        Ok(())
    }

    /// Blocking receive: waits until a valid frame and returns its payload len.
    ///
    /// This is built on top of `try_recv`, which does a single poll step.
    pub async fn recv(&mut self, delay: &mut impl DelayMs, buf: &mut [u8]) -> Result<usize> {
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

        let payload_len = h.len as usize;
        if payload_len > buf.len() || payload_len + LinkHeader::SIZE > raw.len as usize {
            return Err(LinkError::CorruptFrame);
        }

        // Expected sequence?
        if h.seq != self.expected_seq {
            if log_config::LOG_LINK_ORDER_WARN {
                warn!(
                    "LoRaLink: out-of-order seq={} expected={} ",
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

        Ok(Some(payload_len))
    }
}

// -----------------------------------------------------------------------------
// Trait so LoRaLink works with your SX1262
// -----------------------------------------------------------------------------

pub trait Sx1262Interface {
    async fn tx_raw(&mut self, delay: &mut impl DelayMs, payload: &[u8]) -> RadioResult<()>;

    async fn start_rx_continuous(&mut self, delay: &mut impl DelayMs) -> RadioResult<()>;

    async fn apply_lora_config(
        &mut self,
        delay: &mut impl DelayMs,
        cfg: LoRaConfig,
    ) -> RadioResult<()>;

    async fn poll_raw(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> RadioResult<Option<RawRx>>;
}

// Blanket implementation
impl<SPI, NSS, RESET, BUSY, DIO1, SW> Sx1262Interface for Sx1262<SPI, NSS, RESET, BUSY, DIO1, SW>
where
    SPI: embedded_hal_async::spi::SpiBus<u8>,
    NSS: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DIO1: embedded_hal::digital::InputPin,
    SW: crate::drivers::sx1262::RfSwitch,
{
    async fn tx_raw(&mut self, delay: &mut impl DelayMs, payload: &[u8]) -> RadioResult<()> {
        self.tx_raw(delay, payload).await
    }

    async fn start_rx_continuous(&mut self, delay: &mut impl DelayMs) -> RadioResult<()> {
        self.start_rx_continuous(delay).await
    }

    async fn apply_lora_config(
        &mut self,
        delay: &mut impl DelayMs,
        cfg: LoRaConfig,
    ) -> RadioResult<()> {
        self.apply_lora_config(delay, cfg).await
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
}

impl<R> FaultyRadio<R> {
    pub fn new(inner: R, drop_rx_every: u8, drop_ack_every: u8) -> Self {
        let _ = drop_ack_every;
        Self {
            inner,
            drop_every_nth_rx: drop_rx_every,
            rx_count: 0,
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
    async fn tx_raw(&mut self, delay: &mut impl DelayMs, payload: &[u8]) -> RadioResult<()> {
        self.inner.tx_raw(delay, payload).await
    }

    async fn start_rx_continuous(&mut self, delay: &mut impl DelayMs) -> RadioResult<()> {
        self.inner.start_rx_continuous(delay).await
    }

    async fn apply_lora_config(
        &mut self,
        delay: &mut impl DelayMs,
        cfg: LoRaConfig,
    ) -> RadioResult<()> {
        self.inner.apply_lora_config(delay, cfg).await
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
