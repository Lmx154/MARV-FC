//! Generic SPI traits for SD access.
//!
//! These are intentionally small and blocking so board code can adapt
//! concrete HAL types while keeping higher-level SD logic platform-agnostic.

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

/// Minimal SPI bus contract needed by SD logic.
pub trait SdSpiBus {
    type BusError;
    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::BusError>;
    fn transfer_in_place(&mut self, bytes: &mut [u8]) -> Result<(), Self::BusError>;
}

/// Chip-select control contract for SD SPI transactions.
pub trait SdChipSelect {
    type CsError;
    fn select(&mut self) -> Result<(), Self::CsError>;
    fn release(&mut self) -> Result<(), Self::CsError>;
}

/// Full SD SPI device contract (bus + CS).
pub trait SdSpiDevice: SdSpiBus + SdChipSelect {}

impl<T> SdSpiDevice for T where T: SdSpiBus + SdChipSelect {}

/// HAL adapter that wraps a concrete SPI bus and CS pin.
pub struct HalSdSpi<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> HalSdSpi<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }

    pub fn into_parts(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}

impl<SPI, CS> SdSpiBus for HalSdSpi<SPI, CS>
where
    SPI: SpiBus<u8>,
{
    type BusError = SPI::Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::BusError> {
        self.spi.write(bytes)
    }

    fn transfer_in_place(&mut self, bytes: &mut [u8]) -> Result<(), Self::BusError> {
        self.spi.transfer_in_place(bytes)
    }
}

impl<SPI, CS> SdChipSelect for HalSdSpi<SPI, CS>
where
    CS: OutputPin,
{
    type CsError = CS::Error;

    fn select(&mut self) -> Result<(), Self::CsError> {
        self.cs.set_low()
    }

    fn release(&mut self) -> Result<(), Self::CsError> {
        self.cs.set_high()
    }
}
