//! SD abstraction layer.
//!
//! This module centralizes SD responsibilities:
//! - SPI bus abstractions (`spi`)
//! - CRUD operations over a simple storage facade (`store`)

pub mod spi;
pub mod store;

pub use spi::{HalSdSpi, SdChipSelect, SdSpiBus, SdSpiDevice};
pub use store::{FileData, FileName, SdCrud, SdError, SdStore};
