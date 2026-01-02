//! async USB CDC (Communications Device Class) transport implementation.
//! HAL and hardware agnostic: device crates provide the concrete class using
//! their USB stack (e.g. embassy-usb), higher layers just log bytes/lines.

#![allow(async_fn_in_trait)]

use defmt::warn;

/// Minimal async, write-only USB CDC interface.
///
/// Implement this in the device crate by wrapping your USB class (for example
/// `embassy_usb::class::cdc_acm::CdcAcmClass`). Keeping this small lets the
/// common code stay HAL-agnostic.
pub trait AsyncUsbCdc {
    type Error;

    /// Write the entire buffer to the host.
    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
}

#[derive(Debug)]
pub enum UsbCdcError<E> {
    Transport(E),
    BufferTooSmall,
}

impl<E: defmt::Format> defmt::Format for UsbCdcError<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Transport(e) => defmt::write!(f, "Transport({})", e),
            Self::BufferTooSmall => defmt::write!(f, "BufferTooSmall"),
        }
    }
}

/// Write a line to USB CDC, appending `\r\n`.
///
/// `scratch` provides temporary storage so callers can reuse a stack buffer and
/// avoid allocations. Returns `BufferTooSmall` if the provided scratch is not
/// large enough for the line + CRLF.
pub async fn write_line<U: AsyncUsbCdc>(
    usb: &mut U,
    line: &str,
    scratch: &mut [u8],
) -> Result<(), UsbCdcError<U::Error>> {
    let needed = line.len().saturating_add(2);
    if needed > scratch.len() {
        warn!(
            "UsbCdc: scratch too small (needed {} got {})",
            needed,
            scratch.len()
        );
        return Err(UsbCdcError::BufferTooSmall);
    }

    let bytes = line.as_bytes();
    scratch[..bytes.len()].copy_from_slice(bytes);
    scratch[bytes.len()] = b'\r';
    scratch[bytes.len() + 1] = b'\n';

    usb.write(&scratch[..needed])
        .await
        .map_err(UsbCdcError::Transport)
}
