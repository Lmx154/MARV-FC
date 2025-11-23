//! Simple async, HAL-agnostic I²C scanner.
//!
//! This uses only `embedded-hal-async` traits so it works with any HAL
//! (embassy-rp, stm32, etc.) as long as the bus implements
//! `embedded_hal_async::i2c::I2c`.
//!
//! NOTE: We use a 1-byte dummy write for probing instead of an empty write,
//! because some HALs (including some embassy-based I²C impls) don't support
//! zero-length writes reliably.
//!
//! Typical usage (from a binary, e.g. your `main.rs`):
//!
//! ```rust,ignore
//! use common::utils::i2cscanner::scan_i2c_bus_default;
//!
//! let mut bus = SharedI2c0 { bus: i2c0_mutex };
//! let res = scan_i2c_bus_default(&mut bus).await;
//! for addr in res.iter() {
//!     info!("Found device @ 0x{:02X}", addr);
//! }
//! ```

use embedded_hal_async::i2c::I2c;

/// Lowest 7-bit I²C address we normally care about (0x00–0x07 are reserved).
pub const DEFAULT_START_ADDR: u8 = 0x08;
/// Highest 7-bit I²C address we normally care about (>0x77 are invalid).
pub const DEFAULT_END_ADDR: u8 = 0x77;

/// Result from an I²C bus scan.
///
/// `addrs[0..count]` contains the 7-bit addresses that ACKed during the scan.
#[derive(Clone, Copy)]
pub struct I2cScanResult {
    /// Number of valid entries in `addrs`.
    pub count: usize,
    /// List of 7-bit addresses that responded. Max possible = 0x77 - 0x08 + 1 = 112.
    pub addrs: [u8; 112],
}

impl I2cScanResult {
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Iterate over the valid addresses.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &u8> {
        self.addrs[..self.count].iter()
    }
}

/// Generic async I²C scanner.
///
/// - Only depends on `embedded-hal-async`.
/// - Performs a 1-byte dummy write to each address in `[start_addr, end_addr]`.
/// - Any address that ACKs is recorded in the returned `I2cScanResult`.
///
/// This is designed to be used from an Embassy context but does not depend on
/// Embassy types directly; the executor just drives the `async` machinery.
///
/// NOTE: The 1-byte dummy write may bump a device's internal register pointer,
/// so this is primarily intended for debugging / bring-up / menus, *not* for
/// use in the middle of a live transaction sequence.
pub async fn scan_i2c_bus<I2C>(
    i2c: &mut I2C,
    start_addr: u8,
    end_addr: u8,
) -> I2cScanResult
where
    I2C: I2c,
{
    let mut result = I2cScanResult {
        count: 0,
        addrs: [0; 112],
    };

    if start_addr > end_addr {
        return result;
    }

    // Dummy payload used just to provoke an ACK/NACK.
    let probe: [u8; 1] = [0x00];

    for addr in start_addr..=end_addr {
        // Guard against silly ranges; we only care about 7-bit usable I²C.
        if addr < DEFAULT_START_ADDR || addr > DEFAULT_END_ADDR {
            continue;
        }

        // If the device NACKs or the bus errors, we just ignore it.
        if i2c.write(addr, &probe).await.is_ok() {
            if result.count < result.addrs.len() {
                result.addrs[result.count] = addr;
                result.count += 1;
            }
        }
    }

    result
}

/// Convenience wrapper that scans the standard 0x08–0x77 range.
#[inline]
pub async fn scan_i2c_bus_default<I2C>(i2c: &mut I2C) -> I2cScanResult
where
    I2C: I2c,
{
    scan_i2c_bus(i2c, DEFAULT_START_ADDR, DEFAULT_END_ADDR).await
}
