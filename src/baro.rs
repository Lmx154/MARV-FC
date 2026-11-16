//! The barometer: BMP581 on I2C1 (SDA GPIO22, SCL GPIO23, 400 kHz, the board's 4.7k pull-ups),
//! data-ready on GPIO21; a core-0 device, async I2C without DMA. The `bmp581` driver is used as it
//! is; the bus clear before a restart is the firmware's (`busclear`; the driver and embassy-rp 0.10 have none).

use bmp581::{Bmp581, Config, DEFAULT_ADDRESS, Error};
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{I2C1, PIN_21, PIN_22, PIN_23};
use embassy_rp::{Peri, bind_interrupts};
use embassy_time::Delay;
use fw_core::health::{ErrorKind, Limits};
use fw_core::i2c::Gate;

pub use fw_settings::baro as settings;

use crate::device::{
    AdapterCounters, BARO, DeviceAdapter, Pending, Reading, StartFailure, VerifyError, run_device,
};
use crate::i2cbus::Timed;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

/// Nominal sample period P at boot: 1 / 50.056 Hz (the default of `BARO_ODR_HZ`); afterwards
/// [`settings::period_us`] of the configured rate.
pub const PERIOD_US: u32 = 19_978;
const I2C_HZ: u32 = 400_000;

/// The configuration before the first start: the driver's default, as `bmp581_smoke` ran it on this
/// board: pressure 16x, temperature 2x, normal mode at 50.056 Hz, no IIR filter, no FIFO, data-ready
/// interrupt pulsed, active high, push-pull. It equals the parameters' defaults; every start and
/// apply takes the stored parameters instead (`refresh_config`).
fn config() -> Config {
    Config::new()
}

#[cfg(not(feature = "inject-baro-hang"))]
type Raw = I2c<'static, I2C1, i2c::Async>;
#[cfg(feature = "inject-baro-hang")]
type Raw = crate::i2cbus::hang::Hang<I2c<'static, I2C1, i2c::Async>>;
type Bus = Timed<Raw>;

/// I2C1's gate: dirty from a timed-out transaction until the bus is cleared and rebuilt.
static GATE: Gate = Gate::new();

pub struct Bmp581Adapter {
    i2c: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_23>,
    sda: Peri<'static, PIN_22>,
    drdy: Input<'static>,
    chip: Option<Bmp581<Bus>>,
    config: Config,
    calls: u32,
}

impl Bmp581Adapter {
    pub fn new(
        i2c: Peri<'static, I2C1>,
        mut scl: Peri<'static, PIN_23>,
        mut sda: Peri<'static, PIN_22>,
        int: Peri<'static, PIN_21>,
    ) -> Self {
        crate::busclear::clear_at_boot("I2C1", &mut scl, &mut sda);
        let mut adapter = Self {
            i2c,
            scl,
            sda,
            drdy: Input::new(int, Pull::None),
            chip: None,
            config: config(),
            calls: 0,
        };
        adapter.chip = Some(Bmp581::new(adapter.bus(), DEFAULT_ADDRESS));
        defmt::info!(
            "[INFO] BARO: BMP581 on I2C1 at {=u32} Hz, output data rate {=f32} Hz, nominal period {=u32} us",
            I2C_HZ,
            adapter.config.odr.hz(),
            PERIOD_US
        );
        adapter
    }

    /// Builds the I2C1 controller on its pins (embassy-rp resets the block in the constructor).
    fn bus(&mut self) -> Bus {
        let mut cfg = i2c::Config::default();
        cfg.frequency = I2C_HZ;
        cfg.sda_pullup = false;
        cfg.scl_pullup = false;
        // SAFETY: the adapter owns these three peripherals for the whole run and builds a bus from
        // them only while no other bus or pin object made from them exists: in `new`, and in
        // `restart` after the old bus was dropped and the bus clear's pins went out of scope.
        let (i2c, scl, sda) = unsafe {
            (
                self.i2c.clone_unchecked(),
                self.scl.clone_unchecked(),
                self.sda.clone_unchecked(),
            )
        };
        Timed::new(
            Raw::from(I2c::new_async(i2c, scl, sda, Irqs, cfg)),
            I2C_HZ,
            &GATE,
            "I2C1",
        )
    }

    /// Takes the configuration from the stored parameters and sets the device's nominal period P
    /// from its rate (before the chip is configured, so the deadline T is never shorter than the
    /// new sample interval).
    fn refresh_config(&mut self) {
        match crate::store::baro_config() {
            Ok(config) => self.config = config,
            Err(reason) => defmt::warn!(
                "[INFO] BARO: stored settings refused ({=str}), keeping the previous configuration",
                reason
            ),
        }
        let period = settings::period_us(self.config.odr);
        BARO.set_period_us(period);
        defmt::info!(
            "[INFO] BARO: configuration: output data rate {=f32} Hz, pressure {=u8} (code), temperature {=u8} (code), IIR pressure {=u8} temperature {=u8} (codes), nominal period {=u32} us",
            self.config.odr.hz(),
            self.config.pressure_oversampling.code(),
            self.config.temperature_oversampling.code(),
            self.config.pressure_iir.code(),
            self.config.temperature_iir.code(),
            period
        );
    }

    /// The driver, counting one adapter call on the bus.
    fn chip(&mut self) -> Result<&mut Bmp581<Bus>, ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.chip.as_mut().ok_or(ErrorKind::Bus)
    }

    /// The bus clear, then a new I2C1 controller and driver (the gate admits again).
    fn rebuild(&mut self) {
        self.chip = None;
        let (sda_low, scl_low, pulses) = crate::busclear::bus_clear(&mut self.scl, &mut self.sda);
        defmt::info!(
            "[INFO] BARO: bus clear: SDA low {=bool}, SCL low {=bool}, {=u8} pulses",
            sda_low,
            scl_low,
            pulses
        );
        self.chip = Some(Bmp581::new(self.bus(), DEFAULT_ADDRESS));
    }
}

fn kind<E>(error: &Error<E>) -> ErrorKind {
    match error {
        Error::WrongId { .. } => ErrorKind::IdMismatch,
        Error::ConfigMismatch(_) => ErrorKind::ConfigMismatch,
        _ => ErrorKind::Bus,
    }
}

impl DeviceAdapter for Bmp581Adapter {
    async fn data_ready(&mut self) {
        self.drdy.wait_for_rising_edge().await;
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        if GATE.dirty() {
            defmt::info!(
                "[INFO] BARO: start on a bus refused after a time-out: clear and rebuild first"
            );
            self.rebuild();
        }
        self.refresh_config();
        let config = self.config;
        let chip = self
            .chip()
            .map_err(|_| StartFailure { identified: false })?;
        match chip.init(&mut Delay, &config).await {
            Ok(info) => {
                defmt::info!("[INFO] BARO: started: {}", info);
                Ok(())
            }
            Err(e) => {
                defmt::warn!("[INFO] BARO: start failed: {}", e);
                Err(StartFailure {
                    identified: !matches!(e, Error::Bus(_) | Error::WrongId { .. }),
                })
            }
        }
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        #[cfg(any(feature = "hang-core0", feature = "spin-1s", feature = "hang-irqoff"))]
        crate::watchdog::hang_point_core0();
        let chip = self.chip()?;
        // INT_STATUS is clear-on-read: reading it with every sample keeps the flag that `status` sees
        // after a data-ready timeout fresh (otherwise the first timeout of an outage finds a stale one).
        let int = chip.interrupt_status().await.map_err(|e| kind(&e))?;
        if int.power_on_reset {
            return Err(ErrorKind::Por);
        }
        // `NoSample` (data registers at their reset value) counts as a decode error.
        let sample = chip.read().await.map_err(|e| kind(&e))?;
        Ok(Reading::Baro {
            pressure_pa: sample.pascals(),
            temperature_c: sample.celsius(),
        })
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        let status = self
            .chip()?
            .interrupt_status()
            .await
            .map_err(|e| kind(&e))?;
        Ok(if status.power_on_reset {
            Pending::PowerOnReset
        } else if status.data_ready {
            Pending::Data
        } else {
            Pending::Nothing
        })
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        let chip = self.chip().map_err(VerifyError::Failed)?;
        match chip.verify().await {
            Ok(()) => Ok(()),
            Err(Error::ConfigMismatch(m)) => {
                defmt::warn!("[INFO] BARO: verify: {}", m);
                Err(VerifyError::Mismatch)
            }
            Err(e) => Err(VerifyError::Failed(kind(&e))),
        }
    }

    async fn apply(&mut self) -> Result<(), ErrorKind> {
        self.refresh_config();
        let config = self.config;
        let result = self
            .chip()?
            .configure(&mut Delay, &config)
            .await
            .map(|_| ())
            .map_err(|e| kind(&e));
        match &result {
            Ok(()) => defmt::info!("[INFO] BARO: applied"),
            Err(error) => defmt::warn!(
                "[INFO] BARO: apply failed ({=str})",
                crate::device::error_name(*error)
            ),
        }
        result
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.chip()?.recover(&mut Delay).await.map_err(|e| kind(&e))
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.rebuild();
        self.start().await.map_err(|_| ErrorKind::Bus)
    }

    fn counters(&self) -> AdapterCounters {
        AdapterCounters { calls: self.calls }
    }
}

#[cfg(not(feature = "inject"))]
type Adapter = Bmp581Adapter;
#[cfg(feature = "inject")]
type Adapter = crate::inject::Inject<Bmp581Adapter>;

/// The adapter for this build (with the fault-injection wrapper when an injection feature is on).
pub fn adapter(
    i2c: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_23>,
    sda: Peri<'static, PIN_22>,
    int: Peri<'static, PIN_21>,
) -> Adapter {
    wrap(Bmp581Adapter::new(i2c, scl, sda, int))
}

#[cfg(not(feature = "inject"))]
fn wrap(adapter: Bmp581Adapter) -> Adapter {
    adapter
}
#[cfg(feature = "inject")]
fn wrap(adapter: Bmp581Adapter) -> Adapter {
    crate::inject::Inject::new(adapter)
}

#[embassy_executor::task]
pub async fn task(mut adapter: Adapter) -> ! {
    fw_flight::tagged(
        &crate::watchdog::RUNNING0,
        fw_core::watchdog::task::BARO,
        run_device(&BARO, &mut adapter, Limits::DESIGN),
    )
    .await
}
