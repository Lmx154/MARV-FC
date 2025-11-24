//! Async, HAL-agnostic UART transport helpers for MAVLink frames.
//!
//! This module mirrors the style of the other drivers: fully generic over a
//! minimal async UART trait so it can be reused on any MCU/HAL. It only
//! performs framing/deframing; higher layers build the MAVLink messages.

#![allow(async_fn_in_trait)]

use defmt::{debug, warn};
use mavio::protocol::V2;
use mavio::Frame;

/// Maximum MAVLink2 frame size (payload 255 + 10-byte header + 2-byte CRC + 13-byte signature).
pub const MAVLINK_MAX_FRAME: usize = 280;

const MAVLINK2_MAGIC: u8 = 0xFD;
const MAVLINK2_HEADER_LEN: usize = 10;
const MAVLINK2_CRC_LEN: usize = 2;
const MAVLINK2_SIGNATURE_LEN: usize = 13;

/// Minimal async UART interface. Implement this in the device crate for your HAL type.
pub trait AsyncUartBus {
    type Error;

    /// Write the entire buffer.
    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;

    /// Read exactly `buf.len()` bytes into `buf`.
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// UART-specific errors when moving MAVLink frames.
#[derive(Debug)]
pub enum UartComError<E> {
    Transport(E),
    BadMagic(u8),
    FrameTooBig,
    Frame,
}

impl<E: defmt::Format> defmt::Format for UartComError<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Transport(e) => defmt::write!(f, "Transport({})", e),
            Self::BadMagic(m) => defmt::write!(f, "BadMagic(0x{:02X})", m),
            Self::FrameTooBig => defmt::write!(f, "FrameTooBig"),
            Self::Frame => defmt::write!(f, "Frame"),
        }
    }
}

/// Serialize and send a MAVLink2 frame over UART.
pub async fn send_frame_over_uart<U: AsyncUartBus>(
    uart: &mut U,
    frame: &Frame<V2>,
    scratch: &mut [u8],
) -> Result<(), UartComError<U::Error>> {
    let size = frame.size();
    if size > scratch.len() {
        warn!(
            "UartMav: frame too large for scratch (size={} cap={})",
            size,
            scratch.len()
        );
        return Err(UartComError::FrameTooBig);
    }

    let written = frame.serialize(scratch).map_err(|_| UartComError::Frame)?;
    debug!("UartMav: sending MAVLink2 frame len={}", written);
    uart.write(&scratch[..written])
        .await
        .map_err(UartComError::Transport)
}

/// Receive and decode a MAVLink2 frame from UART.
///
/// This waits for the MAVLink2 magic byte (0xFD), reads the header to learn the
/// payload length (and optional signature flag), then reads the remainder of
/// the frame before handing it to `mavio` for deserialization.
pub async fn recv_frame_over_uart<U: AsyncUartBus>(
    uart: &mut U,
    scratch: &mut [u8],
) -> Result<Frame<V2>, UartComError<U::Error>> {
    if scratch.len() < MAVLINK2_HEADER_LEN + MAVLINK2_CRC_LEN {
        warn!("UartMav: scratch buffer too small for MAVLink header");
        return Err(UartComError::FrameTooBig);
    }

    // Find start-of-frame byte.
    loop {
        let mut byte = [0u8; 1];
        uart.read_exact(&mut byte)
            .await
            .map_err(UartComError::Transport)?;
        if byte[0] == MAVLINK2_MAGIC {
            scratch[0] = MAVLINK2_MAGIC;
            break;
        } else {
            debug!("UartMav: skipped byte 0x{:02X} waiting for magic", byte[0]);
        }
    }

    // Read the rest of the header (length + flags + ids + msgid).
    uart.read_exact(&mut scratch[1..MAVLINK2_HEADER_LEN])
        .await
        .map_err(UartComError::Transport)?;

    let payload_len = scratch[1] as usize;
    let incompat_flags = scratch[2];
    let signature_len = if (incompat_flags & 0x01) != 0 {
        MAVLINK2_SIGNATURE_LEN
    } else {
        0
    };

    let frame_len =
        MAVLINK2_HEADER_LEN + MAVLINK2_CRC_LEN + payload_len + signature_len;
    if frame_len > scratch.len() {
        warn!(
            "UartMav: incoming frame len {} exceeds scratch cap {}",
            frame_len,
            scratch.len()
        );
        return Err(UartComError::FrameTooBig);
    }

    uart.read_exact(&mut scratch[MAVLINK2_HEADER_LEN..frame_len])
        .await
        .map_err(UartComError::Transport)?;

    // SAFETY: we collected a full MAVLink2 frame; mavio will validate checksum.
    let frame =
        unsafe { Frame::<V2>::deserialize(&scratch[..frame_len]) }
            .map_err(|_| UartComError::Frame)?;
    debug!(
        "UartMav: received MAVLink2 frame len={} seq={}",
        frame_len,
        frame.sequence()
    );
    Ok(frame)
}
