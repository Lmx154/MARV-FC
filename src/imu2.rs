//! IMU2: LSM6DSK320X on SPI1 (SCK GPIO10, MOSI GPIO11, MISO GPIO8, CS GPIO9, 8 MHz, mode 0) with
//! DMA channels 2 and 3, data-ready on INT1 (GPIO7; INT2 on GPIO6 is not used); a flight-core
//! device. Everything here except [`CELLS`] runs on core 1: [`adapter`] is called inside core 1's
//! entry, so the DMA and GPIO interrupts land there, and [`task`] runs `fw_flight::imu` on core 1's
//! executor. No log here. The `lsm6dsk320x` driver is used as it is.

use core::convert::Infallible;

use crate::flight_latency::{Stretch, measured};
use crate::imu1::SpinDelay;
use embassy_rp::Peri;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH2, DMA_CH3, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, SPI1};
use embassy_rp::spi::{self, Spi};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice, NoDelay};
use fw_core::calib::ImuOffsets;
use fw_core::health::{ErrorKind, Limits};
use fw_core::selftest::{Verdict, axes};
use fw_core::xcore::TripleReader;
use fw_flight::imu::{
    ImuAdapter, ImuCells, ImuReading, ImuTask, Latency, Pending, StartFailure, VerifyError,
    VerifyStep,
};
use fw_settings::imu::Imu2Setup;
use lsm6dsk320x::{ChannelResult, Error, Lsm6dsk320x, SelfTestConfig};

/// The cells between core 0 and the IMU2 task.
pub static CELLS: ImuCells = ImuCells::new();
/// The IMU2 task's longest stretch without a yield and its yield count.
static STRETCH: Stretch = Stretch::new();

const SPI_HZ: u32 = 8_000_000;

/// The peripherals IMU2 owns, moved into core 1's entry.
pub struct Parts {
    pub spi: Peri<'static, SPI1>,
    pub sck: Peri<'static, PIN_10>,
    pub mosi: Peri<'static, PIN_11>,
    pub miso: Peri<'static, PIN_8>,
    pub cs: Peri<'static, PIN_9>,
    pub int1: Peri<'static, PIN_7>,
    pub tx_dma: Peri<'static, DMA_CH2>,
    pub rx_dma: Peri<'static, DMA_CH3>,
    /// The offsets core 0 publishes, subtracted from every sample.
    pub offsets: TripleReader<'static, ImuOffsets>,
    /// The chip's configuration, its full scales and the sample period, read from the settings at
    /// boot; handed over by value, so no cell carries it (`fw_settings::imu::imu2_setup`).
    pub setup: Imu2Setup,
}

type Dev = ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, NoDelay>;
type DevError = DeviceError<spi::Error, Infallible>;

pub struct Lsm6dsk320xAdapter {
    chip: Lsm6dsk320x<Dev>,
    int1: Input<'static>,
    calls: u32,
    step: usize,
    offsets: TripleReader<'static, ImuOffsets>,
    /// The boot setup: `init`, `apply` and the restart configure with it, `acquire` scales with it.
    setup: Imu2Setup,
}

fn kind<E>(error: &Error<E>) -> ErrorKind {
    match error {
        Error::WrongId { .. } => ErrorKind::IdMismatch,
        Error::ConfigMismatch(_) => ErrorKind::ConfigMismatch,
        _ => ErrorKind::Bus,
    }
}

impl Lsm6dsk320xAdapter {
    /// Builds SPI1 with its DMA channels and the INT1 input: call on core 1.
    fn new(p: Parts) -> Self {
        let mut cfg = spi::Config::default();
        cfg.frequency = SPI_HZ;
        let bus = Spi::new(
            p.spi,
            p.sck,
            p.mosi,
            p.miso,
            p.tx_dma,
            p.rx_dma,
            crate::DmaIrqs,
            cfg,
        );
        let cs = Output::new(p.cs, Level::High);
        let device = match ExclusiveDevice::new_no_delay(bus, cs) {
            Ok(device) => device,
            Err(never) => match never {},
        };
        Self {
            chip: Lsm6dsk320x::new(device),
            int1: Input::new(p.int1, Pull::None),
            calls: 0,
            step: 0,
            offsets: p.offsets,
            setup: p.setup,
        }
    }

    fn call(&mut self) -> &mut Lsm6dsk320x<Dev> {
        self.calls = self.calls.wrapping_add(1);
        &mut self.chip
    }

    /// Software reset, identity check and the configuration.
    async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<DevError>> {
        self.step = 0;
        let config = self.setup.config;
        self.call().init(delay, &config).await.map(|_| ())
    }
}

impl ImuAdapter for Lsm6dsk320xAdapter {
    /// INT1 is latched: high until the sample is read, so the level (not an edge) says a sample
    /// is waiting.
    async fn data_ready(&mut self) {
        self.int1.wait_for_high().await;
    }

    async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure> {
        self.init(delay).await.map_err(|e| StartFailure {
            identified: !matches!(e, Error::Bus(_) | Error::WrongId { .. }),
        })
    }

    /// Temperature, gyroscope, low-g (and the unused high-g registers) in one transaction; the
    /// read clears INT1 and the data-ready flags. STATUS_REG is not clear-on-read (its flags clear
    /// when the data is read), so no status read is needed with the sample. The chip has no
    /// reset flag: a power-on reset shows as a timeout or a configuration mismatch.
    async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
        let all = self.call().read_all().await.map_err(|e| kind(&e))?;
        let (offsets, _) = self.offsets.read();
        let (gyro_rads, accel_ms2) = offsets.remove(
            all.gyro.gyro_rads(self.setup.gyro_fs),
            all.accel.accel_ms2(self.setup.accel_fs),
        );
        Ok(ImuReading {
            gyro_rads,
            accel_ms2,
            temperature_c: all.temperature_c(),
        })
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        let status = self.call().status().await.map_err(|e| kind(&e))?;
        Ok(if status.accel {
            Pending::Data
        } else {
            Pending::Nothing
        })
    }

    async fn verify_step(&mut self) -> Result<VerifyStep, VerifyError> {
        let index = self.step;
        let result = self.call().verify_step(index).await;
        self.step = index + 1;
        if self.step >= self.chip.verify_steps() {
            self.step = 0;
        }
        match result {
            Ok(()) if self.step == 0 => Ok(VerifyStep::PassComplete),
            Ok(()) => Ok(VerifyStep::More),
            Err(Error::ConfigMismatch(_)) => Err(VerifyError::Mismatch),
            Err(e) => Err(VerifyError::Failed(kind(&e))),
        }
    }

    /// K = samples per second / registers per pass: one pass in about 1 s.
    fn verify_every(&self) -> u32 {
        let steps = u32::try_from(self.chip.verify_steps()).unwrap_or(u32::MAX);
        (1_000_000 / self.setup.period_us.max(1) / steps.max(1)).max(1)
    }

    /// INT1 is latched: high while a sample waits.
    fn data_waiting(&mut self) -> bool {
        self.int1.is_high()
    }

    /// Configure in place, then one read discarded (a data-ready flag survives `configure`).
    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.step = 0;
        let config = self.setup.config;
        self.call()
            .configure(delay, &config)
            .await
            .map_err(|e| kind(&e))?;
        self.call()
            .read_all()
            .await
            .map(|_| ())
            .map_err(|e| kind(&e))
    }

    /// The driver's `recover` waits for nothing: re-frame, identity, `verify`.
    async fn recover<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), ErrorKind> {
        self.call().recover().await.map_err(|e| kind(&e))
    }

    /// SPI has no bus clear: init and configure again.
    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.init(delay).await.map_err(|e| kind(&e))
    }

    /// The driver's self-test with its defaults (low-g, gyroscope and high-g, positive stimulus,
    /// 100 ms settle); it restores the configuration only on success, and the restart that follows
    /// configures the device in any case.
    async fn self_test<D: DelayNs>(&mut self, delay: &mut D) -> Verdict {
        self.step = 0;
        let result = self.call().self_test(delay, &SelfTestConfig::new()).await;
        let failed = |c: Option<ChannelResult>| c.map_or([false; 3], |c| c.axes.map(|a| !a.pass));
        match result {
            Ok(report) if report.passed() => Verdict::Pass,
            Ok(report) => {
                let [lx, ly, lz] = failed(report.low_g);
                let [gx, gy, gz] = failed(report.gyro);
                let [hx, hy, hz] = failed(report.high_g);
                Verdict::Fail(axes(0, lx, ly, lz) | axes(1, gx, gy, gz) | axes(2, hx, hy, hz))
            }
            Err(e) => Verdict::Error(kind(&e)),
        }
    }

    fn calls(&self) -> u32 {
        self.calls
    }

    fn latency(&self) -> Latency {
        STRETCH.get()
    }
}

#[cfg(not(feature = "inject-imu2"))]
pub type Adapter = Lsm6dsk320xAdapter;
#[cfg(feature = "inject-imu2")]
pub type Adapter = crate::imu_inject::Inject<Lsm6dsk320xAdapter>;

/// The adapter for this build: call on core 1.
pub fn adapter(parts: Parts) -> Adapter {
    #[cfg(not(feature = "inject-imu2"))]
    return Lsm6dsk320xAdapter::new(parts);
    #[cfg(feature = "inject-imu2")]
    return crate::imu_inject::Inject::new(Lsm6dsk320xAdapter::new(parts), &CELLS, INJECT);
}

#[cfg(feature = "inject-imu2-persistent")]
const INJECT: crate::imu_inject::Mode = crate::imu_inject::Mode::Persistent;
#[cfg(all(
    feature = "inject-imu2-verify",
    not(feature = "inject-imu2-persistent")
))]
const INJECT: crate::imu_inject::Mode = crate::imu_inject::Mode::VerifyMismatch;

#[embassy_executor::task]
pub async fn task(adapter: Adapter, period_us: u32) -> ! {
    match ImuTask::new(
        &CELLS,
        adapter,
        SpinDelay,
        crate::now_us,
        period_us,
        Limits::DESIGN,
    ) {
        Some(task) => {
            measured(
                &STRETCH,
                fw_flight::tagged(
                    &crate::watchdog::RUNNING1,
                    fw_core::watchdog::task::IMU2,
                    task.run(),
                ),
            )
            .await
        }
        None => match core::future::pending::<Infallible>().await {},
    }
}
