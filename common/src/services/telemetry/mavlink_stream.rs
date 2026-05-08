//! Stream-oriented MAVLink framing built on top of the shared parser.

use heapless::Vec;

use crate::protocol::mavlink::{
    DecodeError, MAVLINK_V1_STX, MAVLINK_V2_STX, MIN_MAVLINK_FRAME_LEN, MavlinkFrame,
    try_consume_frame,
};

/// Small no_std byte pump that turns an arbitrary byte stream into MAVLink frames.
///
/// This sits below higher-level HIL message handling so the same parser/bridge stack can be reused
/// over UDP, USB CDC packets, or UART streams.
pub struct MavlinkStreamPump<const CAPACITY: usize> {
    buffer: Vec<u8, CAPACITY>,
}

impl<const CAPACITY: usize> Default for MavlinkStreamPump<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> MavlinkStreamPump<CAPACITY> {
    pub const fn new() -> Self {
        Self { buffer: Vec::new() }
    }

    pub fn ingest_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.buffer.is_full() {
                self.buffer.remove(0);
            }
            let _ = self.buffer.push(byte);
        }
    }

    pub fn try_next_frame(&mut self) -> Option<Result<MavlinkFrame, DecodeError>> {
        loop {
            let sync_offset = self
                .buffer
                .iter()
                .position(|byte| *byte == MAVLINK_V2_STX || *byte == MAVLINK_V1_STX)?;
            if sync_offset > 0 {
                self.drop_front(sync_offset);
            }

            if self.buffer.len() < MIN_MAVLINK_FRAME_LEN {
                return None;
            }

            match try_consume_frame(self.buffer.as_slice()) {
                Ok(Some((frame, consumed))) => {
                    self.drop_front(consumed);
                    return Some(Ok(frame));
                }
                Ok(None) => return None,
                Err(DecodeError::BadMagic(_)) => {
                    self.drop_front(1);
                }
                Err(error) => {
                    self.drop_front(1);
                    return Some(Err(error));
                }
            }
        }
    }

    fn drop_front(&mut self, count: usize) {
        for _ in 0..count.min(self.buffer.len()) {
            self.buffer.remove(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use heapless::Vec;

    use super::MavlinkStreamPump;
    use crate::protocol::mavlink::{DecodedMessage, MAVLINK_V2_STX, SYSTEM_TIME_ID};

    const CRC_EXTRA_SYSTEM_TIME: u8 = 137;

    #[test]
    fn pump_waits_for_complete_frame_across_multiple_ingests() {
        let frame = encode_system_time_v2(123);
        let mut pump = MavlinkStreamPump::<64>::new();

        pump.ingest_bytes(&frame[..5]);
        assert!(pump.try_next_frame().is_none());

        pump.ingest_bytes(&frame[5..]);
        let decoded = pump.try_next_frame().unwrap().unwrap();
        assert!(matches!(
            decoded.message,
            DecodedMessage::SystemTime(message) if message.time_boot_ms == 123
        ));
    }

    #[test]
    fn pump_skips_garbage_before_sync() {
        let frame = encode_system_time_v2(456);
        let mut pump = MavlinkStreamPump::<64>::new();

        pump.ingest_bytes(&[0x00, 0x01, 0x02]);
        pump.ingest_bytes(&frame);

        let decoded = pump.try_next_frame().unwrap().unwrap();
        assert!(matches!(
            decoded.message,
            DecodedMessage::SystemTime(message) if message.time_boot_ms == 456
        ));
    }

    fn encode_system_time_v2(time_boot_ms: u32) -> Vec<u8, 32> {
        let mut frame = Vec::<u8, 32>::new();
        let _ = frame.push(MAVLINK_V2_STX);
        let _ = frame.push(12);
        let _ = frame.push(0);
        let _ = frame.push(0);
        let _ = frame.push(7);
        let _ = frame.push(42);
        let _ = frame.push(1);
        let _ = frame.push((SYSTEM_TIME_ID & 0xFF) as u8);
        let _ = frame.push(((SYSTEM_TIME_ID >> 8) & 0xFF) as u8);
        let _ = frame.push(((SYSTEM_TIME_ID >> 16) & 0xFF) as u8);
        extend_slice(&mut frame, &0u64.to_le_bytes());
        extend_slice(&mut frame, &time_boot_ms.to_le_bytes());
        let checksum = checksum_v2(&frame[1..10], &frame[10..22], CRC_EXTRA_SYSTEM_TIME);
        extend_slice(&mut frame, &checksum.to_le_bytes());
        frame
    }

    fn extend_slice<const N: usize>(frame: &mut Vec<u8, N>, bytes: &[u8]) {
        for &byte in bytes {
            let _ = frame.push(byte);
        }
    }

    fn checksum_v2(header_without_magic: &[u8], payload: &[u8], crc_extra: u8) -> u16 {
        let mut crc = 0xFFFFu16;
        for byte in header_without_magic.iter().chain(payload.iter()) {
            crc = crc_accumulate(crc, *byte);
        }
        crc_accumulate(crc, crc_extra)
    }

    fn crc_accumulate(crc: u16, byte: u8) -> u16 {
        let tmp = byte ^ (crc as u8);
        let tmp = tmp ^ (tmp << 4);
        (crc >> 8) ^ (u16::from(tmp) << 8) ^ (u16::from(tmp) << 3) ^ (u16::from(tmp) >> 4)
    }
}
