// common/src/mavlink2/mod.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{debug, warn};
use mavio::Frame;
use mavio::error::FrameError;
use mavio::protocol::V2;

use crate::coms::uart_coms::{self, AsyncUartBus, UartComError};
use crate::lora::link::{LoRaLink, LinkError, Sx1262Interface};
use crate::utils::delay::DelayMs;

/// Public re-exports so higher layers can use mavio types directly.
pub mod prelude {
    pub use mavio::Frame;
    pub use mavio::protocol::V2;
    pub use mavio::dialects;
    pub use crate::mavlink2::{
        MavError, recv_frame_over_lora, recv_frame_over_uart, send_frame_over_lora,
        send_frame_over_uart,
    };
    pub use crate::mavlink2::msg::*;
}

pub mod msg;
/// Error type for the MAVLink2-over-LoRa layer.
#[derive(Debug, defmt::Format)]
pub enum MavError {
    /// Underlying LoRaLink error (reliability / radio).
    Link(LinkError),
    /// UART/serial transport error.
    Serial,
    /// MAVLink frame is too large for LoRaLink MTU.
    EncodeTooBig,
    /// MAVLink frame (de)serialization error.
    Frame,
}

impl From<LinkError> for MavError {
    fn from(e: LinkError) -> Self {
        MavError::Link(e)
    }
}

impl From<FrameError> for MavError {
    fn from(_e: FrameError) -> Self {
        MavError::Frame
    }
}

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

    let mut buf = [0u8; 255]; // upper bound, LoRaLink will cap by MTU anyway
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

/// Send a MAVLink2 frame over a generic async UART.
pub async fn send_frame_over_uart<U>(
    uart: &mut U,
    frame: &Frame<V2>,
    scratch: &mut [u8],
) -> Result<(), MavError>
where
    U: AsyncUartBus,
    U::Error: defmt::Format,
{
    match uart_coms::send_frame_over_uart(uart, frame, scratch).await {
        Ok(()) => Ok(()),
        Err(UartComError::Transport(e)) => {
            warn!("MavUART: transport error {}", e);
            Err(MavError::Serial)
        }
        Err(UartComError::BadMagic(_)) | Err(UartComError::Frame) => Err(MavError::Frame),
        Err(UartComError::FrameTooBig) => Err(MavError::EncodeTooBig),
    }
}

/// Receive a MAVLink2 frame from a generic async UART.
pub async fn recv_frame_over_uart<U>(
    uart: &mut U,
    scratch: &mut [u8],
) -> Result<Frame<V2>, MavError>
where
    U: AsyncUartBus,
    U::Error: defmt::Format,
{
    match uart_coms::recv_frame_over_uart(uart, scratch).await {
        Ok(frame) => Ok(frame),
        Err(UartComError::Transport(e)) => {
            warn!("MavUART: transport error {}", e);
            Err(MavError::Serial)
        }
        Err(UartComError::BadMagic(_)) | Err(UartComError::Frame) => Err(MavError::Frame),
        Err(UartComError::FrameTooBig) => Err(MavError::EncodeTooBig),
    }
}
