//! Shared SPI1 sensor bus + transaction-timeout wrappers.
//!
//! The sensor cluster (BMI088 accel/gyro + LSM6DSV32X) shares one SPI1
//! peripheral. Access is serialized through an async [`Mutex`] and the
//! battle-tested [`SpiDeviceWithConfig`] from `embassy-embedded-hal` (the same
//! shared-bus pattern the recovery altimeter uses for its I2C bus), instead of
//! a hand-rolled `RefCell` device that held its borrow across `.await`.
//!
//! Every transaction is additionally wrapped in [`TimeoutSpiDevice`] /
//! [`TimeoutI2c`]: an async bus transfer that parks (e.g. a DMA completion that
//! never arrives) is bounded by a timeout instead of hanging the acquisition
//! task forever. On timeout the inner transaction future is dropped, which
//! releases the bus mutex, deasserts CS (via the shared-bus `OnDrop` guard),
//! and aborts the in-flight DMA — so the caller can surface a fault and retry.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, with_timeout};
use embedded_hal::i2c::{
    Error as I2cError, ErrorKind as I2cErrorKind, ErrorType as I2cErrorType,
    Operation as I2cOperation, SevenBitAddress,
};
use embedded_hal::spi::{
    Error as SpiError, ErrorKind as SpiErrorKind, ErrorType as SpiErrorType,
    Operation as SpiOperation,
};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

/// The SPI1 peripheral in async (DMA) mode.
pub type SensorSpi = Spi<'static, SPI1, Async>;

/// Shared SPI1 bus. `NoopRawMutex` is correct because every chip on this bus is
/// driven from a single executor; switch to `CriticalSectionRawMutex` only if a
/// task on another executor/core ever touches SPI1.
pub type SharedSensorSpiBus = Mutex<NoopRawMutex, SensorSpi>;

/// Per-chip handle on the shared sensor bus: a config-applying shared-bus device
/// wrapped with a per-transaction timeout. Implements [`AsyncSpiDevice`], so it
/// drops straight into any driver that takes `impl SpiDevice<u8>`.
pub type SensorSpiDevice =
    TimeoutSpiDevice<SpiDeviceWithConfig<'static, NoopRawMutex, SensorSpi, Output<'static>>>;

/// Per-transaction timeout for the shared SPI sensor bus. Generous versus the
/// ~120 µs nominal 15-byte transfer at 1 MHz; only engages when a transfer
/// parks. Safe during init too: it bounds a single CS-framed transaction, not
/// the long `delay_ms` calls that init interleaves *between* transactions.
pub const DEFAULT_SENSOR_SPI_TXN_TIMEOUT: Duration = Duration::from_millis(5);

/// Per-transaction timeout for the I2C sensor buses (slower than SPI).
pub const DEFAULT_SENSOR_I2C_TXN_TIMEOUT: Duration = Duration::from_millis(10);

/// Build a per-chip SPI device on the shared sensor bus.
pub fn new_sensor_spi_device(
    bus: &'static SharedSensorSpiBus,
    cs: Output<'static>,
    config: SpiConfig,
    timeout: Duration,
) -> SensorSpiDevice {
    TimeoutSpiDevice::new(SpiDeviceWithConfig::new(bus, cs, config), timeout)
}

/// Wraps a [`SpiDevice`](AsyncSpiDevice) so every transaction is bounded by a
/// timeout. A parked transfer becomes a recoverable [`TimeoutSpiError::Timeout`]
/// instead of an indefinite hang.
pub struct TimeoutSpiDevice<D> {
    inner: D,
    timeout: Duration,
}

impl<D> TimeoutSpiDevice<D> {
    pub fn new(inner: D, timeout: Duration) -> Self {
        Self { inner, timeout }
    }
}

/// Error from a [`TimeoutSpiDevice`]: either the inner bus error, or a timeout.
#[derive(Debug)]
pub enum TimeoutSpiError<E> {
    Inner(E),
    Timeout,
}

impl<E: SpiError> SpiError for TimeoutSpiError<E> {
    fn kind(&self) -> SpiErrorKind {
        match self {
            Self::Inner(error) => error.kind(),
            Self::Timeout => SpiErrorKind::Other,
        }
    }
}

impl<D: AsyncSpiDevice<u8>> SpiErrorType for TimeoutSpiDevice<D> {
    type Error = TimeoutSpiError<D::Error>;
}

impl<D: AsyncSpiDevice<u8>> AsyncSpiDevice<u8> for TimeoutSpiDevice<D> {
    async fn transaction(
        &mut self,
        operations: &mut [SpiOperation<'_, u8>],
    ) -> Result<(), Self::Error> {
        match with_timeout(self.timeout, self.inner.transaction(operations)).await {
            Ok(result) => result.map_err(TimeoutSpiError::Inner),
            Err(_) => Err(TimeoutSpiError::Timeout),
        }
    }
}

/// Wraps an async [`I2c`](AsyncI2c) so every transaction is bounded by a
/// timeout, mirroring [`TimeoutSpiDevice`] for I2C sensor buses.
pub struct TimeoutI2c<T> {
    inner: T,
    timeout: Duration,
}

impl<T> TimeoutI2c<T> {
    pub fn new(inner: T, timeout: Duration) -> Self {
        Self { inner, timeout }
    }
}

/// Error from a [`TimeoutI2c`]: either the inner bus error, or a timeout.
#[derive(Debug)]
pub enum TimeoutI2cError<E> {
    Inner(E),
    Timeout,
}

impl<E: I2cError> I2cError for TimeoutI2cError<E> {
    fn kind(&self) -> I2cErrorKind {
        match self {
            Self::Inner(error) => error.kind(),
            Self::Timeout => I2cErrorKind::Other,
        }
    }
}

impl<T: AsyncI2c> I2cErrorType for TimeoutI2c<T> {
    type Error = TimeoutI2cError<T::Error>;
}

impl<T: AsyncI2c> AsyncI2c<SevenBitAddress> for TimeoutI2c<T> {
    async fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [I2cOperation<'_>],
    ) -> Result<(), Self::Error> {
        match with_timeout(self.timeout, self.inner.transaction(address, operations)).await {
            Ok(result) => result.map_err(TimeoutI2cError::Inner),
            Err(_) => Err(TimeoutI2cError::Timeout),
        }
    }
}
