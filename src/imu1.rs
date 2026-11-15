//! IMU1: ICM-45686 on SPI0 (SCK GPIO18, MOSI GPIO19, MISO GPIO16, CS GPIO20, 8 MHz, mode 0) with
//! DMA channels 0 and 1, data-ready on INT1 (GPIO15); a flight-core device. Everything here except
//! [`CELLS`] runs on core 1: [`adapter`] is called inside core 1's entry, so the DMA and GPIO
//! interrupts land there, and [`task`] runs `fw_flight::imu` on core 1's executor. No log here.
//! The `icm45686` driver is used as it is.

use core::convert::Infallible;

use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_15, PIN_16, PIN_18, PIN_19, PIN_20, SPI0};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::{Delay, Duration, block_for};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice, NoDelay};
use fw_core::health::{ErrorKind, Limits};
use fw_flight::imu::{
    ImuAdapter, ImuCells, ImuReading, ImuTask, Pending, StartFailure, VerifyError,
};
use icm45686::{
    AccelFullScale, ClockSource, Config, Error, GyroFullScale, Icm45686, Int1Config, Odr,
};

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// The cells between core 0 and the IMU1 task.
pub static CELLS: ImuCells = ImuCells::new();

/// Nominal sample period P: 800 Hz, the output data rate nearest to 1 kHz that the bench proved
/// (the ICM-45686 has no 1 kHz rate: 800 Hz or 1.6 kHz).
pub const PERIOD_US: u32 = 1250;
const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps250;

/// As `icm45686_smoke` ran it on this board ("800 Hz internal clock"): 4 g, 250 dps, 800 Hz on
/// both sensors, internal clock, data-ready on INT1 (8 us push-pull pulse, active high), the
/// driver's default filters; no FIFO.
fn config() -> Config {
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_gyro_fs(GYRO_FS)
        .with_accel_odr(Odr::Hz800)
        .with_gyro_odr(Odr::Hz800)
        .with_clock(ClockSource::Internal)
        .with_int1(Int1Config::new().with_data_ready(true))
}

/// The peripherals IMU1 owns, moved into core 1's entry.
pub struct Parts {
    pub spi: Peri<'static, SPI0>,
    pub sck: Peri<'static, PIN_18>,
    pub mosi: Peri<'static, PIN_19>,
    pub miso: Peri<'static, PIN_16>,
    pub cs: Peri<'static, PIN_20>,
    pub int1: Peri<'static, PIN_15>,
    pub tx_dma: Peri<'static, DMA_CH0>,
    pub rx_dma: Peri<'static, DMA_CH1>,
}

type Dev = ExclusiveDevice<Spi<'static, SPI0, spi::Async>, Output<'static>, NoDelay>;
type DevError = DeviceError<spi::Error, Infallible>;

/// A busy wait on the time driver's counter: no timer, no lock (`verify` between samples reads
/// indirect registers with a 10 us wait each).
struct SpinDelay;

impl DelayNs for SpinDelay {
    async fn delay_ns(&mut self, ns: u32) {
        block_for(Duration::from_micros(u64::from(ns.div_ceil(1000))));
    }

    async fn delay_us(&mut self, us: u32) {
        block_for(Duration::from_micros(u64::from(us)));
    }
}

pub struct Icm45686Adapter {
    chip: Icm45686<Dev>,
    int1: Input<'static>,
    calls: u32,
}

fn kind<E>(error: &Error<E>) -> ErrorKind {
    match error {
        Error::WrongId { .. } => ErrorKind::IdMismatch,
        Error::ConfigMismatch(_) => ErrorKind::ConfigMismatch,
        _ => ErrorKind::Bus,
    }
}

impl Icm45686Adapter {
    /// Builds SPI0 with its DMA channels and the INT1 input: call on core 1.
    fn new(p: Parts) -> Self {
        let mut cfg = spi::Config::default();
        cfg.frequency = SPI_HZ;
        let bus = Spi::new(p.spi, p.sck, p.mosi, p.miso, p.tx_dma, p.rx_dma, Irqs, cfg);
        let cs = Output::new(p.cs, Level::High);
        let device = match ExclusiveDevice::new_no_delay(bus, cs) {
            Ok(device) => device,
            Err(never) => match never {},
        };
        Self {
            chip: Icm45686::new(device),
            int1: Input::new(p.int1, Pull::None),
            calls: 0,
        }
    }

    fn call(&mut self) -> &mut Icm45686<Dev> {
        self.calls = self.calls.wrapping_add(1);
        &mut self.chip
    }

    /// Init and configure, then one status read: INT1_STATUS0 is clear-on-read, so the first
    /// sample's status is not the reset's.
    async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<DevError>> {
        let config = config();
        self.call().init(delay, &config).await?;
        self.call().interrupt_status().await?;
        Ok(())
    }
}

impl ImuAdapter for Icm45686Adapter {
    async fn data_ready(&mut self) {
        self.int1.wait_for_rising_edge().await;
    }

    async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure> {
        self.init(delay).await.map_err(|e| StartFailure {
            identified: !matches!(e, Error::Bus(_) | Error::WrongId { .. }),
        })
    }

    async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
        // INT1_STATUS0 is clear-on-read: reading it with every sample keeps the data-ready flag
        // that `status` sees after a late CHECK fresh.
        let int = self.call().interrupt_status().await.map_err(|e| kind(&e))?;
        if int.reset_done {
            return Err(ErrorKind::Por);
        }
        let sample = self.call().read().await.map_err(|e| kind(&e))?;
        Ok(ImuReading {
            gyro_rads: sample.gyro_rads(GYRO_FS),
            accel_ms2: sample.accel_ms2(ACCEL_FS),
            temperature_c: sample.temperature_c(),
        })
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        let int = self.call().interrupt_status().await.map_err(|e| kind(&e))?;
        Ok(if int.reset_done {
            Pending::PowerOnReset
        } else if int.data_ready {
            Pending::Data
        } else {
            Pending::Nothing
        })
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        match self.call().verify(&mut SpinDelay).await {
            Ok(()) => Ok(()),
            Err(Error::ConfigMismatch(_)) => Err(VerifyError::Mismatch),
            Err(e) => Err(VerifyError::Failed(kind(&e))),
        }
    }

    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        let config = config();
        self.call()
            .configure(delay, &config)
            .await
            .map(|_| ())
            .map_err(|e| kind(&e))
    }

    async fn recover<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.call().recover(delay).await.map_err(|e| kind(&e))
    }

    /// SPI has no bus clear: init and configure again.
    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.init(delay).await.map_err(|e| kind(&e))
    }

    fn calls(&self) -> u32 {
        self.calls
    }
}

#[cfg(not(feature = "inject-imu1"))]
pub type Adapter = Icm45686Adapter;
#[cfg(feature = "inject-imu1")]
pub type Adapter = Inject;

/// The adapter for this build: call on core 1.
pub fn adapter(parts: Parts) -> Adapter {
    #[cfg(not(feature = "inject-imu1"))]
    return Icm45686Adapter::new(parts);
    #[cfg(feature = "inject-imu1")]
    return Inject::new(Icm45686Adapter::new(parts));
}

#[embassy_executor::task]
pub async fn task(adapter: Adapter) -> ! {
    match ImuTask::new(
        &CELLS,
        adapter,
        Delay,
        crate::now_us,
        PERIOD_US,
        Limits::DESIGN,
    ) {
        Some(task) => task.run().await,
        None => match core::future::pending::<Infallible>().await {},
    }
}

/// Injection state words published through the cells (`ImuStatus::inject`).
#[cfg(feature = "inject-imu1")]
pub mod inject_state {
    /// Armed, waiting for 5 s of good samples.
    pub const ARMED: u32 = 1;
    /// Active: failing (persistent) or swallowing edges (missed-edge).
    pub const ACTIVE: u32 = 2;
    /// Over.
    pub const DONE: u32 = 3;
}

/// Fault injection around the ICM-45686 adapter (features `inject-imu1-persistent`,
/// `inject-imu1-missed-edge`), 5 s after the first good sample. No log: its state goes out through
/// `injection()` and core 0 logs it.
#[cfg(feature = "inject-imu1")]
pub struct Inject {
    inner: Icm45686Adapter,
    first_good_us: Option<u64>,
    state: u32,
    since_us: u64,
    restarts_at_start: u32,
}

#[cfg(feature = "inject-imu1")]
impl Inject {
    const AFTER_US: u64 = 5_000_000;
    #[cfg(feature = "inject-imu1-missed-edge")]
    const SWALLOW_US: u64 = 200_000;

    fn new(inner: Icm45686Adapter) -> Self {
        Self {
            inner,
            first_good_us: None,
            state: inject_state::ARMED,
            since_us: 0,
            restarts_at_start: 0,
        }
    }

    fn set(&mut self, state: u32, now: u64) {
        self.state = state;
        self.since_us = now;
    }

    fn due(&self, now: u64) -> bool {
        self.state == inject_state::ARMED
            && self
                .first_good_us
                .is_some_and(|t| now.saturating_sub(t) >= Self::AFTER_US)
    }

    /// Persistent: true while every acquire, recover and restart must fail (until a RESTART
    /// request has been raised after the failure began).
    #[cfg(feature = "inject-imu1-persistent")]
    fn failing(&mut self) -> bool {
        let now = crate::now_us();
        if self.due(now) {
            self.restarts_at_start = CELLS.restart_number();
            self.set(inject_state::ACTIVE, now);
        }
        if self.state == inject_state::ACTIVE && CELLS.restart_number() != self.restarts_at_start {
            self.set(inject_state::DONE, now);
        }
        self.state == inject_state::ACTIVE
    }

    #[cfg(not(feature = "inject-imu1-persistent"))]
    fn failing(&mut self) -> bool {
        let _ = self.restarts_at_start;
        false
    }
}

#[cfg(feature = "inject-imu1")]
impl ImuAdapter for Inject {
    async fn data_ready(&mut self) {
        #[cfg(not(feature = "inject-imu1-missed-edge"))]
        self.inner.data_ready().await;
        #[cfg(feature = "inject-imu1-missed-edge")]
        loop {
            self.inner.data_ready().await;
            let now = crate::now_us();
            if self.due(now) {
                self.set(inject_state::ACTIVE, now);
            }
            if self.state == inject_state::ACTIVE {
                if now.saturating_sub(self.since_us) < Self::SWALLOW_US {
                    continue;
                }
                self.set(inject_state::DONE, now);
            }
            return;
        }
    }

    async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure> {
        self.inner.start(delay).await
    }

    async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        let reading = self.inner.acquire().await?;
        self.first_good_us.get_or_insert_with(crate::now_us);
        Ok(reading)
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        self.inner.status().await
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        self.inner.verify().await
    }

    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.inner.apply(delay).await
    }

    async fn recover<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        self.inner.recover(delay).await
    }

    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        self.inner.restart(delay).await
    }

    fn calls(&self) -> u32 {
        self.inner.calls()
    }

    fn injection(&self) -> (u32, u64) {
        (self.state, self.since_us)
    }
}
