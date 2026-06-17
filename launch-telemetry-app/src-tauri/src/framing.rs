//! Splits a raw UART byte stream into HILink frames.
//!
//! HILink frames are COBS-encoded and terminated by a `0x00` delimiter. Bytes arrive in
//! arbitrary chunks from the serial port, so we accumulate until a delimiter is seen and yield
//! complete frames (delimiter included, which is what `marv_hilink::decode_packet` expects).

/// Largest plausible encoded frame. Our biggest payload is `HilSensorFrame` (115 B); 256 B of
/// payload plus COBS/header/CRC overhead is a comfortable ceiling. Anything longer is treated as
/// a desync and dropped until the next delimiter.
pub const MAX_FRAME_LEN: usize = marv_hilink::encoded_frame_len(256);

pub struct FrameAccumulator {
    buf: Vec<u8>,
    /// True while skipping a garbage/oversized run until the next delimiter.
    resyncing: bool,
    /// Count of frames dropped due to overflow; the worker logs increments as ERROR lines.
    pub dropped: usize,
}

impl Default for FrameAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameAccumulator {
    pub fn new() -> Self {
        Self {
            buf: Vec::with_capacity(MAX_FRAME_LEN),
            resyncing: false,
            dropped: 0,
        }
    }

    /// Drop any partial frame (e.g. on reconnect) without affecting the dropped counter.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.resyncing = false;
    }

    /// Feed received bytes; returns every complete frame found (each ends with `0x00`).
    pub fn push(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();
        for &b in data {
            if self.resyncing {
                // Skip until the next delimiter, then start fresh.
                if b == 0 {
                    self.resyncing = false;
                }
                continue;
            }

            if b == 0 {
                if !self.buf.is_empty() {
                    let mut frame = std::mem::take(&mut self.buf);
                    frame.push(0);
                    frames.push(frame);
                }
                // Empty buffer + delimiter (e.g. back-to-back 0x00) is just an empty gap.
                continue;
            }

            self.buf.push(b);
            if self.buf.len() > MAX_FRAME_LEN {
                self.buf.clear();
                self.resyncing = true;
                self.dropped += 1;
            }
        }
        frames
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_on_delimiter_and_keeps_it() {
        let mut acc = FrameAccumulator::new();
        let frames = acc.push(&[1, 2, 3, 0, 4, 5, 0]);
        assert_eq!(frames, vec![vec![1, 2, 3, 0], vec![4, 5, 0]]);
    }

    #[test]
    fn reassembles_across_chunk_boundaries() {
        let mut acc = FrameAccumulator::new();
        assert!(acc.push(&[1, 2]).is_empty());
        assert!(acc.push(&[3]).is_empty());
        let frames = acc.push(&[0]);
        assert_eq!(frames, vec![vec![1, 2, 3, 0]]);
    }

    #[test]
    fn ignores_empty_gaps() {
        let mut acc = FrameAccumulator::new();
        let frames = acc.push(&[0, 0, 7, 0, 0]);
        assert_eq!(frames, vec![vec![7, 0]]);
    }

    #[test]
    fn oversized_run_is_dropped_then_resyncs() {
        let mut acc = FrameAccumulator::new();
        let garbage = vec![0xABu8; MAX_FRAME_LEN + 50];
        let frames = acc.push(&garbage);
        assert!(frames.is_empty());
        assert_eq!(acc.dropped, 1);
        // After a delimiter we recover and decode the next clean frame.
        let frames = acc.push(&[0, 9, 9, 0]);
        assert_eq!(frames, vec![vec![9, 9, 0]]);
    }
}
