//! MAC packet codec helpers (wrap/unwrap + tick extraction).
//!
//! This is shared by GS + Radio so the on-wire format stays consistent.

use crate::coms::transport::lora::mac::{MAC_FLAG_HAS_INNER, MAC_HDR_LEN, MAC_MAGIC, MAC_VERSION};

/// Extract the master tick sequence from a packet if it is MAC-wrapped.
pub fn mac_tick_from_packet(buf: &[u8]) -> Option<u8> {
    if buf.len() < MAC_HDR_LEN {
        return None;
    }
    if buf[0] != MAC_MAGIC || buf[1] != MAC_VERSION {
        return None;
    }
    Some(buf[2])
}

/// Unwrap the MAC header and return the inner payload.
///
/// - If `buf` is not MAC-wrapped, returns `buf` unchanged.
/// - If `buf` is MAC-wrapped but SYNC-only, returns `&[]`.
pub fn unwrap_mac_inner<'a>(buf: &'a [u8]) -> &'a [u8] {
    if buf.len() < MAC_HDR_LEN {
        return buf;
    }
    if buf[0] != MAC_MAGIC || buf[1] != MAC_VERSION {
        return buf;
    }

    let flags = buf[3];
    if (flags & MAC_FLAG_HAS_INNER) == 0 {
        return &[];
    }

    &buf[MAC_HDR_LEN..]
}

/// Wrap `inner` bytes into a MAC packet.
///
/// When `has_inner` is false, this produces a SYNC-only packet.
pub fn wrap_mac_packet<const N: usize>(
    out: &mut heapless::Vec<u8, N>,
    tick_seq: u8,
    has_inner: bool,
    inner: &[u8],
) -> core::result::Result<(), ()> {
    out.clear();

    let flags = if has_inner { MAC_FLAG_HAS_INNER } else { 0 };
    out.extend_from_slice(&[MAC_MAGIC, MAC_VERSION, tick_seq, flags])
        .map_err(|_| ())?;

    if has_inner {
        out.extend_from_slice(inner).map_err(|_| ())?;
    }

    Ok(())
}
