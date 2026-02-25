//! Shared bus adapters and executor-visible peripheral wrappers.

use common::coms::transport::uart::AsyncUartBus;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::{Async as I2cAsync, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::uart::{Async as UartAsync, Uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

pub(crate) type I2c0Type = I2c<'static, embassy_rp::peripherals::I2C0, I2cAsync>;
pub(crate) type I2c1Type = I2c<'static, embassy_rp::peripherals::I2C1, I2cAsync>;

// Shared I2C0/I2C1 bus managers for multiple I2C devices.
pub(crate) static I2C0_MUTEX: StaticCell<Mutex<RawMutex, I2c0Type>> = StaticCell::new();
pub(crate) static I2C1_MUTEX: StaticCell<Mutex<RawMutex, I2c1Type>> = StaticCell::new();

// Shared SPI1 bus for IMUs on the same bus.
pub(crate) type Spi1Type = Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>;
pub(crate) static SPI1_MUTEX: StaticCell<Mutex<RawMutex, Spi1Type>> = StaticCell::new();
pub(crate) type Spi1Device = SpiDevice<'static, RawMutex, Spi1Type, Output<'static>>;

#[derive(Clone, Copy)]
pub(crate) struct SharedI2c0<'a> {
    pub(crate) bus: &'a Mutex<RawMutex, I2c0Type>,
}

impl<'a> embedded_hal_async::i2c::ErrorType for SharedI2c0<'a> {
    type Error = embassy_rp::i2c::Error;
}

impl<'a> embedded_hal_async::i2c::I2c for SharedI2c0<'a> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write(address, write).await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.transaction(address, operations).await
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SharedI2c<'a> {
    pub(crate) bus: &'a Mutex<RawMutex, I2c1Type>,
}

impl<'a> embedded_hal_async::i2c::ErrorType for SharedI2c<'a> {
    type Error = embassy_rp::i2c::Error;
}

impl<'a> embedded_hal_async::i2c::I2c for SharedI2c<'a> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write(address, write).await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.transaction(address, operations).await
    }
}

// Generic async UART trait implementation for the RP UART peripheral.
pub(crate) struct RpUart<'d>(pub(crate) Uart<'d, UartAsync>);

impl<'d> AsyncUartBus for RpUart<'d> {
    type Error = embassy_rp::uart::Error;

    async fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), Self::Error> {
        Uart::write(&mut self.0, bytes).await
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> core::result::Result<(), Self::Error> {
        Uart::read(&mut self.0, buf).await
    }
}
