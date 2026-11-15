//! The barometer: BMP581 on I2C1 (SDA GPIO22, SCL GPIO23, 400 kHz, the board's 4.7k pull-ups),
//! data-ready on GPIO21; a core-0 device, async I2C without DMA. The `bmp581` driver is used as it
//! is; the bus clear before a restart is this adapter's (the driver and embassy-rp 0.10 have none).

use bmp581::{Bmp581, Config, DEFAULT_ADDRESS, Error};
use embassy_rp::gpio::{Flex, Input, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{I2C1, PIN_21, PIN_22, PIN_23};
use embassy_rp::{Peri, bind_interrupts};
use embassy_time::{Delay, Duration, block_for};
use fw_core::health::{ErrorKind, Limits};

use crate::device::{
    AdapterCounters, BARO, DeviceAdapter, Pending, Reading, StartFailure, VerifyError, run_device,
};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

/// Nominal sample period P: 1 / 50.056 Hz (the output data rate of [`config`]).
pub const PERIOD_US: u32 = 19_978;
const I2C_HZ: u32 = 400_000;

/// The fixed configuration until parameters arrive: the driver's default, as `bmp581_smoke` ran it
/// on this board: pressure 16x, temperature 2x, normal mode at 50.056 Hz, no IIR filter, no FIFO,
/// data-ready interrupt pulsed, active high, push-pull.
fn config() -> Config {
    Config::new()
}

type Bus = I2c<'static, I2C1, i2c::Async>;

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
        scl: Peri<'static, PIN_23>,
        sda: Peri<'static, PIN_22>,
        int: Peri<'static, PIN_21>,
    ) -> Self {
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
        I2c::new_async(i2c, scl, sda, Irqs, cfg)
    }

    /// Up to 9 SCL pulses while SDA is held low, then a START and a STOP (as `bmp581_cancel`'s
    /// `bus_clear`). Called only while no bus object exists. Returns (SDA low, SCL low, pulses).
    fn bus_clear(&mut self) -> (bool, bool, u8) {
        // SAFETY: as in `bus`: the bus was dropped before this call and these pins are released
        // again when this function returns, before the next bus is built.
        let (scl, sda) = unsafe { (self.scl.clone_unchecked(), self.sda.clone_unchecked()) };
        let mut scl = Flex::new(scl);
        let mut sda = Flex::new(sda);
        let half = Duration::from_micros(5);
        scl.set_pull(Pull::None);
        sda.set_pull(Pull::None);
        scl.set_low();
        sda.set_low();
        scl.set_as_input();
        sda.set_as_input();
        block_for(half);
        let (sda_low, scl_low) = (sda.is_low(), scl.is_low());
        let mut pulses = 0u8;
        while sda.is_low() && pulses < 9 {
            scl.set_as_output();
            block_for(half);
            scl.set_as_input();
            block_for(half);
            pulses += 1;
        }
        if sda.is_high() && scl.is_high() {
            sda.set_as_output();
            block_for(half);
            sda.set_as_input();
            block_for(half);
        }
        (sda_low, scl_low, pulses)
    }

    /// The driver, counting one adapter call on the bus.
    fn chip(&mut self) -> Result<&mut Bmp581<Bus>, ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.chip.as_mut().ok_or(ErrorKind::Bus)
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
        let config = self.config;
        self.chip()?
            .configure(&mut Delay, &config)
            .await
            .map(|_| ())
            .map_err(|e| kind(&e))
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.chip()?.recover(&mut Delay).await.map_err(|e| kind(&e))
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.chip = None;
        let (sda_low, scl_low, pulses) = self.bus_clear();
        defmt::info!(
            "[INFO] BARO: bus clear: SDA low {=bool}, SCL low {=bool}, {=u8} pulses",
            sda_low,
            scl_low,
            pulses
        );
        self.chip = Some(Bmp581::new(self.bus(), DEFAULT_ADDRESS));
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
    run_device(&BARO, &mut adapter, Limits::DESIGN).await
}
