// common/src/lora/link.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn};
use crate::drivers::sx1262::{Sx1262, RawRx, Result as RadioResult};
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
}

impl<'a, RADIO> LoRaLink<'a, RADIO>
where
    RADIO: Sx1262Interface,
{
    pub fn new(radio: &'a mut RADIO) -> Self {
        Self {
            radio,
            next_seq: 1,
            expected_seq: 1,
            ack_timeout_ms: 200,
            retries: 4,
        }
    }

    pub async fn start_rx(&mut self, delay: &mut impl DelayMs) -> Result<()> {
        self.radio.start_rx_continuous(delay).await.map_err(|_| LinkError::Radio)
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// Send a reliable frame (payload only)
    pub async fn send(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> Result<()> {
        if payload.len() > 240 {
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
        frame[4..4 + payload.len()].copy_from_slice(payload);

        // TX + WAIT FOR ACK
        for attempt in 0..=self.retries {
            info!("LoRaLink TX seq={} attempt={}", seq, attempt + 1);

            self.radio.tx_raw(delay, &frame[..header.len as usize + LinkHeader::SIZE]).await
                .map_err(|_| LinkError::Radio)?;

            // After TX, immediately switch to RX
            self.radio.start_rx_continuous(delay).await.map_err(|_| LinkError::Radio)?;

            // Wait for ACK
            let mut elapsed = 0;
            let mut buf = [0u8; 255];

            while elapsed < self.ack_timeout_ms {
                if let Some(rx) = self.radio.poll_raw(delay, &mut buf).await.map_err(|_| LinkError::Radio)? {
                    if rx.len >= 4 {
                        if let Some(h) = LinkHeader::decode(&buf[..4]) {
                            if (h.flags & FLAG_ACK) != 0 && h.ack == seq {
                                info!("LoRaLink ACK received for seq={}", seq);
                                return Ok(());
                            }
                        }
                    }
                }

                delay.delay_ms(10).await;
                elapsed += 10;
            }

            warn!("LoRaLink: ACK timeout seq={} (attempt {})", seq, attempt + 1);
        }

        Err(LinkError::Timeout)
    }

    /// Receive a reliable LoRaLink frame
    pub async fn recv(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> Result<usize> {
        let mut rx_buf = [0u8; 255];

        loop {
            if let Some(rx) = self.radio.poll_raw(delay, &mut rx_buf).await.map_err(|_| LinkError::Radio)? {
                if rx.len < 4 {
                    continue;
                }

                let Some(h) = LinkHeader::decode(&rx_buf[..4]) else {
                    continue;
                };

                let payload_len = h.len as usize;
                if payload_len > buf.len() || payload_len + 4 > rx.len as usize {
                    return Err(LinkError::CorruptFrame);
                }

                // Expected sequence?
                if h.seq != self.expected_seq {
                    warn!("LoRaLink: out-of-order seq={} expected={}", h.seq, self.expected_seq);
                } else {
                    self.expected_seq = self.expected_seq.wrapping_add(1);
                }

                // Copy payload
                buf[..payload_len].copy_from_slice(&rx_buf[4..4 + payload_len]);

                // Send ACK
                let mut ack_frame = [0u8; 4];
                let ack_header = LinkHeader {
                    seq: 0,
                    ack: h.seq,
                    flags: FLAG_ACK,
                    len: 0,
                };
                ack_header.encode(&mut ack_frame);

                info!("LoRaLink: Sending ACK for seq={}", h.seq);
                self.radio.tx_raw(delay, &ack_frame).await.map_err(|_| LinkError::Radio)?;

                // IMPORTANT: After transmitting an ACK, return to RX mode so we
                // continue to receive future frames. If we do not do this the
                // radio remains in TX/FS mode and will no longer receive any
                // frames (classic SX1262 behavior).
                self.radio.start_rx_continuous(delay).await.map_err(|_| LinkError::Radio)?;
                info!("LoRaLink: ACK TX done -> entered RX");

                // Return payload
                return Ok(payload_len);
            }

            delay.delay_ms(10).await;
        }
    }
}

// -----------------------------------------------------------------------------
// Trait so LoRaLink works with your SX1262
// -----------------------------------------------------------------------------

pub trait Sx1262Interface {
    async fn tx_raw(&mut self, delay: &mut impl DelayMs, payload: &[u8]) -> RadioResult<()>;
    async fn start_rx_continuous(&mut self, delay: &mut impl DelayMs) -> RadioResult<()>;
    async fn poll_raw(&mut self, delay: &mut impl DelayMs, buf: &mut [u8])
        -> RadioResult<Option<RawRx>>;
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
    async fn tx_raw(&mut self, delay: &mut impl DelayMs, payload: &[u8]) -> RadioResult<()> {
        self.tx_raw(delay, payload).await
    }

    async fn start_rx_continuous(&mut self, delay: &mut impl DelayMs) -> RadioResult<()> {
        self.start_rx_continuous(delay).await
    }

    async fn poll_raw(&mut self, delay: &mut impl DelayMs, buf: &mut [u8]) -> RadioResult<Option<RawRx>> {
        self.poll_raw(delay, buf).await
    }
}
