//! The deadline around every I2C transaction of the core-0 devices (BARO on I2C1, MAG on I2C0),
//! as ArduPilot's `I2CDevice::_transfer`: [`Timed`] wraps the embassy-rp bus that the driver
//! uses and runs each transaction under [`fw_core::i2c::timeout_ms`]. A transaction that misses
//! it is abandoned (its embassy-rp future dropped) and answered [`BusError::TimedOut`], and from
//! then on the bus's [`Gate`] refuses every transaction with the same error until the adapter has
//! cleared the bus and built a new one (its `restart`, or its `start` when the gate is dirty). The
//! driver fails through its ordinary bus-error path. No retry here: the device's ladder retries.

use core::future::Future;
use core::pin::pin;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::{Error, ErrorKind, ErrorType, I2c, Operation};
use fw_core::i2c::{Gate, timeout_ms};

/// The bus's error, or the deadline missed (also the answer of a bus refused after one).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BusError<E> {
    I2c(E),
    TimedOut,
}

impl<E: Error> Error for BusError<E> {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::I2c(e) => e.kind(),
            Self::TimedOut => ErrorKind::Other,
        }
    }
}

/// An I2C bus whose transactions run under a deadline.
pub struct Timed<B> {
    inner: B,
    bus_hz: u32,
    gate: &'static Gate,
    name: &'static str,
}

impl<B> Timed<B> {
    /// A newly built bus: its gate admits transactions again.
    pub fn new(inner: B, bus_hz: u32, gate: &'static Gate, name: &'static str) -> Self {
        gate.rebuilt();
        Self {
            inner,
            bus_hz,
            gate,
            name,
        }
    }
}

impl<B: ErrorType> ErrorType for Timed<B> {
    type Error = BusError<B::Error>;
}

impl<B: I2c> I2c for Timed<B> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let bytes = read.len();
        guarded(
            self.gate,
            self.name,
            self.bus_hz,
            bytes,
            self.inner.read(address, read),
        )
        .await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let bytes = write.len();
        guarded(
            self.gate,
            self.name,
            self.bus_hz,
            bytes,
            self.inner.write(address, write),
        )
        .await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let bytes = write.len().saturating_add(read.len());
        let transfer = self.inner.write_read(address, write, read);
        guarded(self.gate, self.name, self.bus_hz, bytes, transfer).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let bytes = operations
            .iter()
            .map(|op| match op {
                Operation::Read(buf) => buf.len(),
                Operation::Write(buf) => buf.len(),
            })
            .fold(0usize, usize::saturating_add);
        let transfer = self.inner.transaction(address, operations);
        guarded(self.gate, self.name, self.bus_hz, bytes, transfer).await
    }
}

/// Runs `transfer` under the deadline (refused at once when the gate is dirty). A deadline that
/// a flash save overlapped is discarded and the wait starts again (the device layer's rule for
/// its data-ready deadline: a save stalls core 0).
async fn guarded<E>(
    gate: &Gate,
    name: &str,
    bus_hz: u32,
    bytes: usize,
    transfer: impl Future<Output = Result<(), E>>,
) -> Result<(), BusError<E>> {
    if gate.dirty() {
        return Err(BusError::TimedOut);
    }
    let limit = Duration::from_millis(u64::from(timeout_ms(bytes, bus_hz)));
    let mut transfer = pin!(transfer);
    let mut from = Instant::now();
    loop {
        match select(transfer.as_mut(), Timer::after(limit)).await {
            Either::First(result) => return result.map_err(BusError::I2c),
            Either::Second(()) => {
                let now = Instant::now();
                if crate::settings::overlaps_save(from, now) {
                    from = now;
                    continue;
                }
                gate.timed_out();
                defmt::warn!(
                    "[INFO] {=str}: transaction of {=usize} bytes timed out after {=u64} ms at {=u64} ms: abandoned, the bus is refused until it is cleared and rebuilt",
                    name,
                    bytes,
                    limit.as_millis(),
                    now.as_millis()
                );
                return Err(BusError::TimedOut);
            }
        }
    }
}
