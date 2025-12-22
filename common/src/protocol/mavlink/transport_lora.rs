#![allow(async_fn_in_trait)]

use defmt::{debug, warn};
use mavio::Frame;
use mavio::protocol::V2;

use crate::coms::transport::lora::link::{LoRaLink, Sx1262Interface};
use crate::coms::transport::lora::mac::{
    INNER_MTU, LORA_MAC_MTU, MAC_FLAG_HAS_INNER, MAC_HDR_LEN, MAC_MAGIC, MAC_VERSION,
};
use crate::protocol::mavlink::MavError;
use crate::utils::delay::DelayMs;

/// Send a MAVLink2 frame over LoRaLink.
///
/// This:
///   - Serializes the `Frame<V2>` into a stack buffer
///   - Verifies it fits inside the LoRaLink MTU
///   - Uses `LoRaLink::send_unreliable` (no link-level ARQ)
pub async fn send_frame_over_lora<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    frame: &Frame<V2>,
) -> Result<(), MavError>
where
    RADIO: Sx1262Interface,
{
    let size = frame.size();
    let mtu = LoRaLink::<RADIO>::MTU;

    if size > mtu {
        warn!(
            "MavLoRa: frame too big for MTU (size={} mtu={})",
            size, mtu
        );
        return Err(MavError::EncodeTooBig);
    }

    let mut buf = [0u8; 255];
    let written = frame.serialize(&mut buf[..mtu])?;
    debug!("MavLoRa: sending MAVLink2 frame len={}", written);

    link.send_unreliable(delay, &buf[..written])
        .await
        .map_err(MavError::from)
}

/// Receive a MAVLink2 frame over LoRaLink.
///
/// This:
///   - Blocks in `LoRaLink::recv` until a valid datagram arrives
///   - Interprets the entire datagram as a single MAVLink2 frame
///   - Deserializes into `Frame<V2>`
pub async fn recv_frame_over_lora<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    buf: &mut [u8],
) -> Result<Frame<V2>, MavError>
where
    RADIO: Sx1262Interface,
{
    let len = link.recv(delay, buf).await.map_err(MavError::from)?;

    // SAFETY: mavio marks this unsafe; on error we just return MavError::Frame.
    let frame = unsafe { Frame::<V2>::deserialize(&buf[..len]) }?;
    debug!("MavLoRa: received MAVLink2 frame len={}", len);
    Ok(frame)
}

/// Send a MAVLink2 frame over LoRaLink using the **new MAC header**.
///
/// - Prepends the 4-byte MAC sync header: `{magic, version, tick_seq, flags}`.
/// - Uses `LoRaLink::send_unreliable` (no link-level ARQ).
/// - Caller controls `tick_seq` (typically from the tick MAC).
///
/// This is the preferred transport for the new wireless link.
pub async fn send_frame_over_lora_mac<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    tick_seq: u8,
    frame: &Frame<V2>,
) -> Result<(), MavError>
where
    RADIO: Sx1262Interface,
{
    let size = frame.size();
    if size > INNER_MTU {
        warn!(
            "MavLoRa(MAC): frame too big for inner MTU (size={} inner_mtu={})",
            size, INNER_MTU
        );
        return Err(MavError::EncodeTooBig);
    }

    // Serialize into inner buffer.
    let mut inner = [0u8; INNER_MTU];
    let inner_len = frame.serialize(&mut inner)?;

    // Wrap into MAC packet.
    let mut out = [0u8; LORA_MAC_MTU];
    out[0] = MAC_MAGIC;
    out[1] = MAC_VERSION;
    out[2] = tick_seq;
    out[3] = MAC_FLAG_HAS_INNER;
    out[MAC_HDR_LEN..MAC_HDR_LEN + inner_len].copy_from_slice(&inner[..inner_len]);
    let out_len = MAC_HDR_LEN + inner_len;

    debug!("MavLoRa(MAC): sending MAVLink2 frame len={} tick_seq={}", inner_len, tick_seq);
    link.send_unreliable(delay, &out[..out_len])
        .await
        .map_err(MavError::from)
}

/// Receive a MAVLink2 frame over LoRaLink that may be wrapped in the **new MAC header**.
///
/// Behavior:
/// - If packet has MAC header + inner payload: returns `(Some(tick_seq), frame)`.
/// - If packet has MAC header but **no inner** (SYNC-only): ignored (waits for next).
/// - If packet has no MAC header: treated as legacy single-frame datagram and returns `(None, frame)`.
pub async fn recv_frame_over_lora_mac<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    buf: &mut [u8],
) -> Result<(Option<u8>, Frame<V2>), MavError>
where
    RADIO: Sx1262Interface,
{
    loop {
        let len = link.recv(delay, buf).await.map_err(MavError::from)?;
        let payload = &buf[..len];

        match split_mac_inner(payload) {
            MacDecoded::MacSyncOnly { .. } => {
                // Ignore: used only for tick/slot synchronization.
                continue;
            }
            MacDecoded::MacInner { tick_seq, inner } => {
                let frame = unsafe { Frame::<V2>::deserialize(inner) }?;
                debug!("MavLoRa(MAC): received MAVLink2 frame len={} tick_seq={}", inner.len(), tick_seq);
                return Ok((Some(tick_seq), frame));
            }
            MacDecoded::Legacy(inner) => {
                let frame = unsafe { Frame::<V2>::deserialize(inner) }?;
                debug!("MavLoRa(MAC): received legacy MAVLink2 frame len={}", inner.len());
                return Ok((None, frame));
            }
        }
    }
}

enum MacDecoded<'a> {
    Legacy(&'a [u8]),
    MacSyncOnly { tick_seq: u8 },
    MacInner { tick_seq: u8, inner: &'a [u8] },
}

fn split_mac_inner(buf: &[u8]) -> MacDecoded<'_> {
    if buf.len() < MAC_HDR_LEN {
        return MacDecoded::Legacy(buf);
    }
    if buf[0] != MAC_MAGIC || buf[1] != MAC_VERSION {
        return MacDecoded::Legacy(buf);
    }

    let tick_seq = buf[2];
    let flags = buf[3];
    if (flags & MAC_FLAG_HAS_INNER) == 0 {
        return MacDecoded::MacSyncOnly { tick_seq };
    }

    MacDecoded::MacInner {
        tick_seq,
        inner: &buf[MAC_HDR_LEN..],
    }
}
