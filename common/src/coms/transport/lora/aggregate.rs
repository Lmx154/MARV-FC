//! Link Aggregate Packet codec (aggregation, not fragmentation).
//!
//! This sits *inside* the MAC header (i.e., inside `INNER_MTU`).
//!
//! Format (v1):
//! - magic:   u8
//! - version: u8
//! - flags:   u8  (bit0 = has_fast)
//! - count:   u8  (#sections)
//! - sections: repeated `count` times:
//!     - len: u8
//!     - bytes[len]
//!
//! Semantics:
//! - If `has_fast` is set, the first section is FAST.
//! - Remaining sections are MAVLink frames (raw MAVLink2 bytes).
//! - No fragmentation: each section must fit fully.

use heapless::Vec;

use super::mac::INNER_MTU;

pub const AGG_MAGIC: u8 = 0xA7;
pub const AGG_VERSION: u8 = 0x01;

pub const AGG_FLAG_HAS_FAST: u8 = 0x01;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionKind {
    Fast,
    Mav,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    BadMagic,
    BadVersion,
    Truncated,
}

pub struct AggBuilder<'a> {
    out: &'a mut Vec<u8, INNER_MTU>,
    flags: u8,
    count: u8,
}

impl<'a> AggBuilder<'a> {
    pub fn new(out: &'a mut Vec<u8, INNER_MTU>) -> Self {
        out.clear();
        // Placeholder header; we will fill flags/count at finish.
        let _ = out.extend_from_slice(&[AGG_MAGIC, AGG_VERSION, 0, 0]);
        Self {
            out,
            flags: 0,
            count: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn remaining(&self) -> usize {
        INNER_MTU.saturating_sub(self.out.len())
    }

    pub fn can_push(&self, bytes_len: usize) -> bool {
        // Need 1 byte for section length + section bytes.
        let need = 1 + bytes_len;
        need <= self.remaining() && bytes_len <= u8::MAX as usize
    }

    pub fn push_fast(&mut self, bytes: &[u8]) -> bool {
        if self.count != 0 {
            // FAST must be first if present.
            return false;
        }
        if !self.can_push(bytes.len()) {
            return false;
        }
        self.flags |= AGG_FLAG_HAS_FAST;
        if self.out.push(bytes.len() as u8).is_err() {
            return false;
        }
        if self.out.extend_from_slice(bytes).is_err() {
            return false;
        }
        self.count = self.count.wrapping_add(1);
        true
    }

    pub fn push_mav(&mut self, bytes: &[u8]) -> bool {
        if !self.can_push(bytes.len()) {
            return false;
        }
        if self.out.push(bytes.len() as u8).is_err() {
            return false;
        }
        if self.out.extend_from_slice(bytes).is_err() {
            return false;
        }
        self.count = self.count.wrapping_add(1);
        true
    }

    pub fn finish(&mut self) {
        // Fill flags/count in header.
        if self.out.len() >= 4 {
            self.out[2] = self.flags;
            self.out[3] = self.count;
        }
    }
}

/// Iterate sections in an aggregate packet.
///
/// Returns:
/// - `Ok(true)` if buffer was an aggregate packet (and callback was invoked for each section)
/// - `Ok(false)` if buffer is not an aggregate packet (caller should treat it as a raw MAVLink frame)
/// - `Err(_)` if it looks like an aggregate packet but is malformed/truncated
pub fn for_each_section(buf: &[u8], mut f: impl FnMut(SectionKind, &[u8])) -> Result<bool, DecodeError> {
    if buf.len() < 4 {
        return Ok(false);
    }
    if buf[0] != AGG_MAGIC {
        return Ok(false);
    }
    if buf[1] != AGG_VERSION {
        return Err(DecodeError::BadVersion);
    }

    let flags = buf[2];
    let count = buf[3] as usize;
    let mut i = 0usize;
    let mut off = 4usize;

    let has_fast = (flags & AGG_FLAG_HAS_FAST) != 0;

    while i < count {
        if off >= buf.len() {
            return Err(DecodeError::Truncated);
        }
        let len = buf[off] as usize;
        off += 1;
        if off + len > buf.len() {
            return Err(DecodeError::Truncated);
        }

        let kind = if has_fast && i == 0 {
            SectionKind::Fast
        } else {
            SectionKind::Mav
        };

        f(kind, &buf[off..off + len]);
        off += len;
        i += 1;
    }

    Ok(true)
}
