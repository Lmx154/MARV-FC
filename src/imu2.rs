//! IMU2: LSM6DSK320X on SPI1 (SCK GPIO10, MOSI GPIO11, MISO GPIO8, CS GPIO9, 8 MHz, mode 0) with
//! DMA channels 2 and 3, data-ready on INT1 (GPIO7; INT2 on GPIO6 is not used); a flight-core
//! device. Everything here except [`CELLS`] runs on core 1: [`adapter`] is called inside core 1's
//! entry, so the DMA and GPIO interrupts land there, and [`task`] runs `fw_flight::imu` on core 1's
//! executor. No log here. The `lsm6dsk320x` driver is used as it is.

use core::convert::Infallible;

use crate::flight_yield::{Stretch, YieldSpi, measured};
use embassy_rp::Peri;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH2, DMA_CH3, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, SPI1};
use embassy_rp::spi::{self, Spi};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice, NoDelay};
use fw_core::health::{ErrorKind, Limits};
use fw_flight::imu::{
    ImuAdapter, ImuCells, ImuReading, ImuTask, Pending, StartFailure, VerifyError,
};
use lsm6dsk320x::{
    AccelFullScale, Config, DataReady, DataReadyMode, Error, GyroFullScale, HaodrSet, IntRouting,
    Lsm6dsk320x, Odr, OperatingMode,
};

/// The cells between core 0 and the IMU2 task.
pub static CELLS: ImuCells = ImuCells::new();
/// The IMU2 task's longest stretch without a yield and its yield count.
static STRETCH: Stretch = Stretch::new();

/// Nominal sample period P: 800 Hz, the high-accuracy ODR set "800" with the ODR code that is
/// 960 Hz in the normal mode. The normal mode offers 480 or 960 Hz, and its 960 Hz ran at
/// 942.66 Hz on this board (the clock trim); the set "800" ran at 800.664 Hz in
/// `lsm6dsk320x_modes` (datasheet: +-1 % over temperature and supply with the gyroscope on).
pub const PERIOD_US: u32 = 1250;
const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps500;

/// As `lsm6dsk320x_modes` ran it on this board: 4 g, 500 dps, both sensors at the ODR code 960 in
/// the high-accuracy ODR set "800" (check 4), the driver's default filters (LPF1 and LPF2 off, the
/// data-ready mask on), high-g off, no FIFO; the low-g data-ready on INT1 only, LATCHED
/// (push-pull, active high): asserted until the output registers are read, as check 5 ("latched")
/// showed. So `data_ready` waits for the level: a sample that arrived while the task was busy (for
/// example in `verify`) is not lost.
fn config() -> Config {
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_accel_odr(Odr::Hz960)
        .with_gyro_fs(GYRO_FS)
        .with_gyro_odr(Odr::Hz960)
        .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base800))
        .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_accel(true)))
        .with_int2(IntRouting::new())
        .with_data_ready_mode(DataReadyMode::Latched)
}

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
}

type Dev = YieldSpi<ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, NoDelay>>;
type DevError = DeviceError<spi::Error, Infallible>;

pub struct Lsm6dsk320xAdapter {
    chip: Lsm6dsk320x<Dev>,
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
            chip: Lsm6dsk320x::new(YieldSpi(device)),
            int1: Input::new(p.int1, Pull::None),
            calls: 0,
        }
    }

    fn call(&mut self) -> &mut Lsm6dsk320x<Dev> {
        self.calls = self.calls.wrapping_add(1);
        &mut self.chip
    }

    /// Software reset, identity check and the configuration.
    async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<DevError>> {
        self.call().init(delay, &config()).await.map(|_| ())
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
        Ok(ImuReading {
            gyro_rads: all.gyro.gyro_rads(GYRO_FS),
            accel_ms2: all.accel.accel_ms2(ACCEL_FS),
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

    async fn verify(&mut self) -> Result<(), VerifyError> {
        match self.call().verify().await {
            Ok(()) => Ok(()),
            Err(Error::ConfigMismatch(_)) => Err(VerifyError::Mismatch),
            Err(e) => Err(VerifyError::Failed(kind(&e))),
        }
    }

    /// Configure in place, then one read discarded (a data-ready flag survives `configure`).
    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.call()
            .configure(delay, &config())
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

    fn calls(&self) -> u32 {
        self.calls
    }

    fn latency(&self) -> (u32, u32) {
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
    return crate::imu_inject::Inject::new(
        Lsm6dsk320xAdapter::new(parts),
        &CELLS,
        crate::imu_inject::Mode::Persistent,
    );
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
