#![allow(async_fn_in_trait)]

use defmt::warn;
use mavio::Frame;
use mavio::protocol::V2;

use crate::coms::transport::uart::{self, AsyncUartBus, UartComError};
use crate::protocol::mavlink::MavError;

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
    match uart::send_frame_over_uart(uart, frame, scratch).await {
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
    match uart::recv_frame_over_uart(uart, scratch).await {
        Ok(frame) => Ok(frame),
        Err(UartComError::Transport(e)) => {
            warn!("MavUART: transport error {}", e);
            Err(MavError::Serial)
        }
        Err(UartComError::BadMagic(_)) | Err(UartComError::Frame) => Err(MavError::Frame),
        Err(UartComError::FrameTooBig) => Err(MavError::EncodeTooBig),
    }
}
