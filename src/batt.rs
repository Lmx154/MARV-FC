//! The battery monitor: the RP2350 ADC on core 0, WITHOUT DMA (interrupt-driven single
//! conversions, each awaited to completion), 10 Hz. Battery voltage on GPIO42 (ADC2, 100k / 10k
//! divider), the ESC's current output on GPIO41 (ADC1, no pad pull). The `adc-battery` crate turns
//! the sums into millivolts and milliamps with the configuration of the `BATT_*` parameters, read
//! for every sample.
//!
//! There is no chip identity and no bus: "Ok" means the ADC converts, whatever the pins carry (with
//! the battery unplugged the voltage reads about 0 V). The device's "data ready" is the 100 ms
//! period timer; a conversion that does not finish within 3 periods is an error event
//! (`ErrorKind::Timeout`).

use adc_battery::{BatteryConfig, Reading as Counts};
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc, Async, Channel};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{ADC, PIN_41, PIN_42};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use fw_core::health::{ErrorKind, Limits};

use crate::device::{
    AdapterCounters, BATT, DeviceAdapter, Pending, Reading, StartFailure, VerifyError, run_device,
};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

/// The sample period: 10 Hz.
pub const PERIOD_US: u32 = 100_000;
/// Conversions averaged per channel and sample (after one discarded after the channel switch).
const CONVERSIONS: u32 = 16;
/// The pad's 100 nF takes a few milliseconds to charge once the pull-down is off; the bench waited
/// 150 ms before the first conversion (the only wait that was tested).
const SETTLE: Duration = Duration::from_millis(150);

pub struct BattAdapter {
    adc: Adc<'static, Async>,
    voltage: Channel<'static>,
    current: Channel<'static>,
    next: Instant,
    settled: bool,
    calls: u32,
}

/// One averaged reading of a channel: one conversion discarded, then [`CONVERSIONS`] added up.
async fn average(
    adc: &mut Adc<'static, Async>,
    ch: &mut Channel<'static>,
) -> Result<Counts, ErrorKind> {
    adc.read(ch).await.map_err(|_| ErrorKind::Bus)?;
    let mut sum = 0u32;
    for _ in 0..CONVERSIONS {
        sum += u32::from(adc.read(ch).await.map_err(|_| ErrorKind::Bus)?);
    }
    Counts::from_sum(sum, CONVERSIONS).ok_or(ErrorKind::Bus)
}

impl BattAdapter {
    pub fn new(
        adc: Peri<'static, ADC>,
        current: Peri<'static, PIN_41>,
        voltage: Peri<'static, PIN_42>,
    ) -> Self {
        let adapter = Self {
            adc: Adc::new(adc, Irqs, adc::Config::default()),
            voltage: Channel::new_pin(voltage, Pull::None),
            current: Channel::new_pin(current, Pull::None),
            next: Instant::now(),
            settled: false,
            calls: 0,
        };
        defmt::info!(
            "[INFO] BATT: ADC on core 0, no DMA: battery voltage GPIO42, current GPIO41, period {=u32} us, {=u32} conversions per channel",
            PERIOD_US,
            CONVERSIONS
        );
        adapter
    }

    /// Converts both channels once and returns the pair of averaged readings, giving up after
    /// three periods.
    async fn convert(&mut self) -> Result<(Counts, Counts), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        let limit = Duration::from_micros(u64::from(PERIOD_US) * 3);
        let Self {
            adc,
            voltage,
            current,
            ..
        } = self;
        let both = async {
            let v = average(adc, voltage).await?;
            let c = average(adc, current).await?;
            Ok::<_, ErrorKind>((v, c))
        };
        with_timeout(limit, both)
            .await
            .map_err(|_| ErrorKind::Timeout)?
    }
}

impl DeviceAdapter for BattAdapter {
    async fn data_ready(&mut self) {
        let now = Instant::now();
        if self.next + Duration::from_micros(u64::from(PERIOD_US)) < now {
            self.next = now;
        }
        Timer::at(self.next).await;
        self.next += Duration::from_micros(u64::from(PERIOD_US));
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        if !self.settled {
            Timer::after(SETTLE).await;
            self.settled = true;
        }
        self.next = Instant::now();
        match self.convert().await {
            Ok(_) => {
                defmt::info!("[INFO] BATT: started: both channels convert");
                Ok(())
            }
            Err(e) => {
                defmt::warn!(
                    "[INFO] BATT: start failed: {=str}",
                    crate::device::error_name(e)
                );
                Err(StartFailure { identified: false })
            }
        }
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        let (voltage, current) = self.convert().await?;
        let config: BatteryConfig = crate::store::batt_config();
        let mv = config
            .voltage
            .millivolts(config.adc.pin_microvolts(voltage));
        let ma = config.current.milliamps(config.adc.pin_microvolts(current));
        Ok(Reading::Batt {
            voltage_v: mv as f32 / 1000.0,
            current_a: ma as f32 / 1000.0,
        })
    }

    /// A period timer is never "missed": after a deadline miss a sample can always be taken.
    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        Ok(Pending::Data)
    }

    /// There is no configuration in the ADC to check.
    async fn verify(&mut self) -> Result<(), VerifyError> {
        Ok(())
    }

    /// The parameters are read for every sample: nothing to apply.
    async fn apply(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.convert().await.map(|_| ())
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.start().await.map_err(|_| ErrorKind::Timeout)
    }

    fn counters(&self) -> AdapterCounters {
        AdapterCounters { calls: self.calls }
    }
}

pub fn adapter(
    adc: Peri<'static, ADC>,
    current: Peri<'static, PIN_41>,
    voltage: Peri<'static, PIN_42>,
) -> BattAdapter {
    BattAdapter::new(adc, current, voltage)
}

#[embassy_executor::task]
pub async fn task(mut adapter: BattAdapter) -> ! {
    fw_flight::tagged(
        &crate::watchdog::RUNNING0,
        fw_core::watchdog::task::BATT,
        run_device(&BATT, &mut adapter, Limits::DESIGN),
    )
    .await
}
