use common::drivers::pressure_transducer::PressureTransducer;
use common::interfaces::sensors::PressureTransducerSource;
use common::messages::sensor::PressureTransducerSample;
use embassy_rp::adc::{Adc, Async, Channel, Error as AdcError};
use embassy_rp::gpio::Pull;
use embassy_rp::{Peri, peripherals};

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RpAdcPressureTransducerError {
    Adc(AdcError),
}

pub struct RpAdcPressureTransducerSource {
    adc: Adc<'static, Async>,
    channel: Channel<'static>,
    driver: PressureTransducer,
}

impl RpAdcPressureTransducerSource {
    pub fn new(
        adc: Adc<'static, Async>,
        pin: Peri<'static, peripherals::PIN_29>,
        driver: PressureTransducer,
    ) -> Self {
        Self {
            adc,
            channel: Channel::new_pin(pin, Pull::None),
            driver,
        }
    }
}

impl PressureTransducerSource for RpAdcPressureTransducerSource {
    type Error = RpAdcPressureTransducerError;

    fn read_pressure_transducer_sample(
        &mut self,
    ) -> impl core::future::Future<Output = Result<PressureTransducerSample, Self::Error>> + '_
    {
        async move {
            let counts = self
                .adc
                .read(&mut self.channel)
                .await
                .map_err(RpAdcPressureTransducerError::Adc)?;
            Ok(self.driver.sample_from_adc_counts(counts))
        }
    }
}
