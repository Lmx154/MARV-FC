#![allow(async_fn_in_trait)]

use defmt::{debug, warn};
use mavio::Frame;
use mavio::protocol::V2;

use crate::coms::transport::lora::link::{LoRaLink, Sx1262Interface};
use crate::protocol::mavlink::MavError;
use crate::utils::delay::DelayMs;

/// Send a MAVLink2 frame over LoRaLink.
///
/// This:
///   - Serializes the `Frame<V2>` into a stack buffer
///   - Verifies it fits inside the LoRaLink MTU
///   - Uses `LoRaLink::send` for reliable delivery
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

    link.send(delay, &buf[..written]).await.map_err(MavError::from)
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
