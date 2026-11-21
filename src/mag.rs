//! The magnetometer: BMM350 on I2C0 (SDA GPIO12, SCL GPIO13, 400 kHz, the board's 4.7k pull-ups),
//! data-ready on GPIO4; a core-0 device, async I2C without DMA. The `bmm350` driver is used as it
//! is; the bus clear before a restart is the firmware's (`busclear`). The sample is the driver's
//! compensated field in microtesla rotated by `MAG_ORIENT` (chip axes to body FRD), r, then the
//! calibration m = S (r - o) with the `MAG_OFF*` offsets and the `MAG_DIAG*`/`MAG_ODIAG*` matrix
//! (body frame), applied here in software (the chip's own offset registers are not used); the
//! sample carries both m and r. `AHRS_ORIENTATION` is never read here: the compass frame does not
//! depend on the board's orientation.

use bmm350::{Bmm350, Config, DEFAULT_ADDRESS, Error};
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{I2C0, PIN_4, PIN_12, PIN_13};
use embassy_rp::{Peri, bind_interrupts};
use embassy_time::Delay;
use fw_core::health::{ErrorKind, Limits};
use fw_core::i2c::Gate;
use fw_core::sample::{MagExpected, MagReading, MagSample};
use fw_core::selftest::Verdict;
use fw_core::xcore::TripleWriter;

pub use fw_settings::mag as settings;

use crate::device::{
    AdapterCounters, DeviceAdapter, MAG, Pending, Reading, Sample, StartFailure, VerifyError,
    run_device,
};
use crate::i2cbus::Timed;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

/// Nominal sample period P at boot: 1 / 100 Hz (the default of `MAG_ODR_HZ`); afterwards
/// [`settings::period_us`] of the configured rate.
pub const PERIOD_US: u32 = 10_000;
const I2C_HZ: u32 = 400_000;

/// The configuration before the first start: the driver's default, as the bench programs ran it on
/// this board: 100 Hz, averaging 2, all axes, normal mode at start, data-ready interrupt pulsed,
/// active high, push-pull. It equals the parameters' defaults; every start and apply takes the
/// stored parameters instead (`refresh_config`).
fn config() -> Config {
    Config::new()
}

type Bus = Timed<I2c<'static, I2C0, i2c::Async>>;

/// I2C0's gate: dirty from a timed-out transaction until the bus is cleared and rebuilt.
static GATE: Gate = Gate::new();

pub struct Bmm350Adapter {
    i2c: Peri<'static, I2C0>,
    scl: Peri<'static, PIN_13>,
    sda: Peri<'static, PIN_12>,
    drdy: Input<'static>,
    chip: Option<Bmm350<Bus>>,
    config: Config,
    calls: u32,
    /// The writer of `crate::flight::FLIGHT_MAG` (`None` if it was taken already: nothing is written).
    flight: Option<TripleWriter<'static, MagSample>>,
    /// The expected field read with the newest sample's calibration (`acquire`), published with it (`to_flight`).
    expected: Option<MagExpected>,
}

impl Bmm350Adapter {
    pub fn new(
        i2c: Peri<'static, I2C0>,
        mut scl: Peri<'static, PIN_13>,
        mut sda: Peri<'static, PIN_12>,
        int: Peri<'static, PIN_4>,
    ) -> Self {
        crate::busclear::clear_at_boot("I2C0", &mut scl, &mut sda);
        let mut adapter = Self {
            i2c,
            scl,
            sda,
            drdy: Input::new(int, Pull::Down),
            chip: None,
            config: config(),
            calls: 0,
            flight: crate::flight::FLIGHT_MAG.take_writer(),
            expected: None,
        };
        adapter.chip = Some(Bmm350::new(adapter.bus(), DEFAULT_ADDRESS));
        defmt::info!(
            "[INFO] MAG: BMM350 on I2C0 at {=u32} Hz, output data rate {=f32} Hz, nominal period {=u32} us",
            I2C_HZ,
            adapter.config.odr.hertz(),
            PERIOD_US
        );
        adapter
    }

    /// Builds the I2C0 controller on its pins (embassy-rp resets the block in the constructor).
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
            I2c::new_async(i2c, scl, sda, Irqs, cfg),
            I2C_HZ,
            &GATE,
            "I2C0",
        )
    }

    /// Takes the configuration from the stored parameters and sets the device's nominal period P
    /// from its rate (before the chip is configured, so the deadline T is never shorter than the
    /// new sample interval).
    fn refresh_config(&mut self) {
        match crate::store::mag_config() {
            Ok(config) => self.config = config,
            Err(reason) => defmt::warn!(
                "[INFO] MAG: stored settings refused ({=str}), keeping the previous configuration",
                reason
            ),
        }
        let period = settings::period_us(self.config.odr);
        MAG.set_period_us(period);
        defmt::info!(
            "[INFO] MAG: configuration: output data rate {=f32} Hz, averaging {} (code {=u8}), nominal period {=u32} us",
            self.config.odr.hertz(),
            self.config.averaging,
            self.config.averaging.code(),
            period
        );
    }

    /// The driver, counting one adapter call on the bus.
    fn chip(&mut self) -> Result<&mut Bmm350<Bus>, ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.chip.as_mut().ok_or(ErrorKind::Bus)
    }

    /// The bus clear, then a new I2C0 controller and driver (the gate admits again).
    fn rebuild(&mut self) {
        self.chip = None;
        let (sda_low, scl_low, pulses) = crate::busclear::bus_clear(&mut self.scl, &mut self.sda);
        defmt::info!(
            "[INFO] MAG: bus clear: SDA low {=bool}, SCL low {=bool}, {=u8} pulses",
            sda_low,
            scl_low,
            pulses
        );
        self.chip = Some(Bmm350::new(self.bus(), DEFAULT_ADDRESS));
    }
}

fn kind<E>(error: &Error<E>) -> ErrorKind {
    match error {
        Error::WrongId { .. } => ErrorKind::IdMismatch,
        Error::ConfigMismatch(_) => ErrorKind::ConfigMismatch,
        _ => ErrorKind::Bus,
    }
}

impl DeviceAdapter for Bmm350Adapter {
    async fn data_ready(&mut self) {
        self.drdy.wait_for_rising_edge().await;
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        if GATE.dirty() {
            defmt::info!(
                "[INFO] MAG: start on a bus refused after a time-out: clear and rebuild first"
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
                defmt::info!("[INFO] MAG: started: {}", info);
                Ok(())
            }
            Err(e) => {
                defmt::warn!("[INFO] MAG: start failed: {}", e);
                Err(StartFailure {
                    identified: !matches!(e, Error::Bus(_) | Error::WrongId { .. }),
                })
            }
        }
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        let (orientation, calibration, expected) = crate::store::mag_frame();
        self.expected = expected;
        let chip = self.chip()?;
        // INT_STATUS is clear-on-read (and in pulsed mode clears itself after 1.25 ms): reading it
        // with every sample keeps the flag that `status` sees after a data-ready timeout fresh.
        chip.interrupt_status().await.map_err(|e| kind(&e))?;
        let field = chip.read_field().await.map_err(|e| kind(&e))?;
        let raw_ut = orientation.rotate([field.x, field.y, field.z]);
        Ok(Reading::Mag {
            reading: MagReading {
                field_ut: settings::apply(raw_ut, &calibration),
                temperature_c: field.temperature,
            },
            raw_ut,
        })
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        let status = self
            .chip()?
            .interrupt_status()
            .await
            .map_err(|e| kind(&e))?;
        // The BMM350 has no power-on-reset flag in INT_STATUS: a reset shows as a `verify` mismatch.
        Ok(if status.data_ready {
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
                defmt::warn!("[INFO] MAG: verify: {}", m);
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
            Ok(()) => defmt::info!("[INFO] MAG: applied"),
            Err(error) => defmt::warn!(
                "[INFO] MAG: apply failed ({=str})",
                crate::device::error_name(*error)
            ),
        }
        result
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.chip()?.recover(&mut Delay).await.map_err(|e| kind(&e))
    }

    /// The driver's self-test (X and Y, both polarities); it restores the configuration only on
    /// success, and the loop restarts the device afterwards in any case.
    async fn self_test(&mut self) -> Verdict {
        let chip = match self.chip() {
            Ok(chip) => chip,
            Err(kind) => return Verdict::Error(kind),
        };
        match chip.self_test(&mut Delay).await {
            Ok(report) => {
                defmt::info!("[INFO] MAG: self-test: {}", report);
                if report.passed() {
                    Verdict::Pass
                } else {
                    let failed = report
                        .entries
                        .iter()
                        .enumerate()
                        .filter(|(_, e)| !e.passed)
                        .fold(0u16, |mask, (i, _)| mask | (1 << i));
                    Verdict::Fail(failed)
                }
            }
            Err(e) => {
                defmt::warn!("[INFO] MAG: self-test failed: {}", e);
                Verdict::Error(kind(&e))
            }
        }
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.rebuild();
        self.start().await.map_err(|_| ErrorKind::Bus)
    }

    fn counters(&self) -> AdapterCounters {
        AdapterCounters { calls: self.calls }
    }

    fn to_flight(&mut self, sample: &Sample) {
        if let (Reading::Mag { reading: r, .. }, Some(w)) = (sample.reading, &mut self.flight) {
            w.write(MagSample {
                expected: self.expected,
                ..r.sample(sample.t_us, sample.seq)
            });
        }
    }
}

#[cfg(not(feature = "inject-mag"))]
type Adapter = Bmm350Adapter;
#[cfg(feature = "inject-mag")]
type Adapter = crate::inject::Inject<Bmm350Adapter>;

/// The adapter for this build (with the fault-injection wrapper when `inject-mag` is on).
pub fn adapter(
    i2c: Peri<'static, I2C0>,
    scl: Peri<'static, PIN_13>,
    sda: Peri<'static, PIN_12>,
    int: Peri<'static, PIN_4>,
) -> Adapter {
    wrap(Bmm350Adapter::new(i2c, scl, sda, int))
}

#[cfg(not(feature = "inject-mag"))]
fn wrap(adapter: Bmm350Adapter) -> Adapter {
    adapter
}
#[cfg(feature = "inject-mag")]
fn wrap(adapter: Bmm350Adapter) -> Adapter {
    crate::inject::Inject::new_mag(adapter)
}

#[embassy_executor::task]
pub async fn task(mut adapter: Adapter) -> ! {
    fw_flight::tagged(
        &crate::watchdog::RUNNING0,
        fw_core::watchdog::task::MAG,
        run_device(&MAG, &mut adapter, Limits::DESIGN),
    )
    .await
}
