//! Async, HAL-agnostic UART transport trait.
//!
//! Higher-level framing (custom packet framing, etc) lives in sibling modules
//! under `crate::coms::transport`.

#![allow(async_fn_in_trait)]

/// Minimal async UART interface. Implement this in the device crate for your HAL type.
pub trait AsyncUartBus {
    type Error;

    /// Write the entire buffer.
    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;

    /// Read exactly `buf.len()` bytes into `buf`.
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
}

