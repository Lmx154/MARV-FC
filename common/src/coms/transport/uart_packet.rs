//! UART framing for custom packets with CRC16-CCITT.
//!
//! Frame format (little-endian CRC):
//! [0xA5, 0x5A] [len] [packet_type + payload...] [crc16]
//! - `len` includes the packet_type byte plus payload bytes.
//! - CRC is computed over `len` + `packet_type/payload`.

#![allow(async_fn_in_trait)]

use defmt::{debug, warn};

use crate::coms::transport::uart::AsyncUartBus;
use crate::protocol::packet::{decode_packet, Packet, MAX_PACKET_BYTES};

const PREAMBLE: [u8; 2] = [0xA5, 0x5A];
const PREAMBLE_LEN: usize = 2;
const CRC_LEN: usize = 2;

pub const UART_PACKET_MAX_FRAME: usize = PREAMBLE_LEN + 1 + MAX_PACKET_BYTES + CRC_LEN;

#[derive(Debug)]
pub enum UartPacketError<E> {
    Transport(E),
    FrameTooBig,
    BadLength(u8),
    BadCrc { expected: u16, received: u16 },
    Decode,
}

impl<E: defmt::Format> defmt::Format for UartPacketError<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Transport(e) => defmt::write!(f, "Transport({})", e),
            Self::FrameTooBig => defmt::write!(f, "FrameTooBig"),
            Self::BadLength(len) => defmt::write!(f, "BadLength({})", len),
            Self::BadCrc { expected, received } => {
                defmt::write!(f, "BadCrc(exp=0x{:04X} got=0x{:04X})", expected, received)
            }
            Self::Decode => defmt::write!(f, "Decode"),
        }
    }
}

pub async fn send_packet_over_uart<U: AsyncUartBus>(
    uart: &mut U,
    packet: &Packet,
    scratch: &mut [u8],
) -> Result<(), UartPacketError<U::Error>> {
    let len = 1usize.saturating_add(packet.payload.len());
    if len > MAX_PACKET_BYTES {
        return Err(UartPacketError::BadLength(len as u8));
    }

    let needed = PREAMBLE_LEN + 1 + len + CRC_LEN;
    if scratch.len() < needed {
        warn!(
            "UartPkt: scratch too small (need={} cap={})",
            needed,
            scratch.len()
        );
        return Err(UartPacketError::FrameTooBig);
    }

    scratch[0] = PREAMBLE[0];
    scratch[1] = PREAMBLE[1];
    scratch[2] = len as u8;
    scratch[3] = packet.packet_type as u8;
    scratch[4..4 + packet.payload.len()].copy_from_slice(packet.payload.as_slice());

    let mut crc = crc16_ccitt_update(0xFFFF, &[len as u8]);
    crc = crc16_ccitt_update(crc, &scratch[3..3 + len]);
    let crc_bytes = crc.to_le_bytes();
    let crc_start = 3 + len;
    scratch[crc_start] = crc_bytes[0];
    scratch[crc_start + 1] = crc_bytes[1];

    debug!("UartPkt: tx len={} crc=0x{:04X}", len, crc);
    uart.write(&scratch[..needed])
        .await
        .map_err(UartPacketError::Transport)
}

pub async fn recv_packet_over_uart<U: AsyncUartBus>(
    uart: &mut U,
    scratch: &mut [u8],
) -> Result<Packet, UartPacketError<U::Error>> {
    if scratch.len() < MAX_PACKET_BYTES {
        return Err(UartPacketError::FrameTooBig);
    }

    let mut sync = 0u8;
    loop {
        let mut byte = [0u8; 1];
        uart.read_exact(&mut byte)
            .await
            .map_err(UartPacketError::Transport)?;
        let b = byte[0];
        if sync == 0 {
            if b == PREAMBLE[0] {
                sync = 1;
            }
        } else if b == PREAMBLE[1] {
            break;
        } else {
            sync = if b == PREAMBLE[0] { 1 } else { 0 };
        }
    }

    let mut len_byte = [0u8; 1];
    uart.read_exact(&mut len_byte)
        .await
        .map_err(UartPacketError::Transport)?;
    let len = len_byte[0] as usize;
    if len > MAX_PACKET_BYTES {
        return Err(UartPacketError::BadLength(len as u8));
    }

    if len != 0 {
        uart.read_exact(&mut scratch[..len])
            .await
            .map_err(UartPacketError::Transport)?;
    }

    let mut crc_bytes = [0u8; 2];
    uart.read_exact(&mut crc_bytes)
        .await
        .map_err(UartPacketError::Transport)?;
    let received = u16::from_le_bytes(crc_bytes);

    let mut expected = crc16_ccitt_update(0xFFFF, &[len as u8]);
    expected = crc16_ccitt_update(expected, &scratch[..len]);
    if expected != received {
        return Err(UartPacketError::BadCrc { expected, received });
    }

    debug!("UartPkt: rx len={} crc=0x{:04X}", len, expected);
    decode_packet(&scratch[..len]).map_err(|_| UartPacketError::Decode)
}

fn crc16_ccitt_update(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}
