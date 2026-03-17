use core::{cell::RefCell, convert::Infallible};

use embassy_rp::gpio::Output;
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Error as SpiError, Spi};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus as AsyncSpiBus, SpiDevice};
use embedded_hal_bus::spi::DeviceError;

pub type SensorSpi = Spi<'static, SPI1, Async>;
pub type SharedSensorSpiBus = RefCell<SensorSpi>;

pub struct SharedSpiDevice<'a> {
    bus: &'a SharedSensorSpiBus,
    cs: Output<'static>,
    delay: Delay,
}

impl<'a> SharedSpiDevice<'a> {
    pub fn new(bus: &'a SharedSensorSpiBus, mut cs: Output<'static>) -> Result<Self, Infallible> {
        cs.set_high();
        Ok(Self {
            bus,
            cs,
            delay: Delay,
        })
    }
}

impl ErrorType for SharedSpiDevice<'_> {
    type Error = DeviceError<SpiError, Infallible>;
}

impl SpiDevice<u8> for SharedSpiDevice<'_> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();

        let op_res = 'ops: {
            let mut bus = self.bus.borrow_mut();
            for operation in operations {
                let result = match operation {
                    Operation::Read(buffer) => AsyncSpiBus::read(&mut *bus, buffer).await,
                    Operation::Write(buffer) => AsyncSpiBus::write(&mut *bus, buffer).await,
                    Operation::Transfer(read, write) => {
                        AsyncSpiBus::transfer(&mut *bus, read, write).await
                    }
                    Operation::TransferInPlace(buffer) => {
                        AsyncSpiBus::transfer_in_place(&mut *bus, buffer).await
                    }
                    Operation::DelayNs(ns) => match AsyncSpiBus::flush(&mut *bus).await {
                        Err(error) => Err(error),
                        Ok(()) => {
                            self.delay.delay_ns(*ns).await;
                            Ok(())
                        }
                    },
                };

                if let Err(error) = result {
                    break 'ops Err(error);
                }
            }

            AsyncSpiBus::flush(&mut *bus).await
        };

        self.cs.set_high();

        op_res.map_err(DeviceError::Spi)?;

        Ok(())
    }
}
