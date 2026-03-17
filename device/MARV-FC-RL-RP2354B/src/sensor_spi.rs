use core::convert::Infallible;

use embassy_rp::gpio::Output;
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Error as SpiError, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus as AsyncSpiBus, SpiDevice};
use embedded_hal_bus::spi::DeviceError;

pub type SharedSensorSpiBus = Mutex<CriticalSectionRawMutex, Option<Spi<'static, SPI1, Async>>>;

pub struct SharedSpiDevice {
    bus: &'static SharedSensorSpiBus,
    cs: Output<'static>,
    delay: Delay,
}

impl SharedSpiDevice {
    pub fn new(
        bus: &'static SharedSensorSpiBus,
        mut cs: Output<'static>,
    ) -> Result<Self, Infallible> {
        cs.set_high();
        Ok(Self {
            bus,
            cs,
            delay: Delay,
        })
    }
}

impl ErrorType for SharedSpiDevice {
    type Error = DeviceError<SpiError, Infallible>;
}

impl SpiDevice<u8> for SharedSpiDevice {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();

        let op_res = 'ops: {
            let mut bus = self.bus.lock().await;
            let bus = bus.as_mut().expect("shared sensor SPI bus not initialized");
            for operation in operations {
                let result = match operation {
                    Operation::Read(buffer) => AsyncSpiBus::read(bus, buffer).await,
                    Operation::Write(buffer) => AsyncSpiBus::write(bus, buffer).await,
                    Operation::Transfer(read, write) => {
                        AsyncSpiBus::transfer(bus, read, write).await
                    }
                    Operation::TransferInPlace(buffer) => {
                        AsyncSpiBus::transfer_in_place(bus, buffer).await
                    }
                    Operation::DelayNs(ns) => match AsyncSpiBus::flush(bus).await {
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

            AsyncSpiBus::flush(bus).await
        };

        self.cs.set_high();

        op_res.map_err(DeviceError::Spi)?;

        Ok(())
    }
}
