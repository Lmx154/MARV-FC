use common::drivers::storage::{MicrosdLogger, MicrosdLoggerConfig};
use common::interfaces::storage::LogError;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Blocking, ClkPin, Config as SpiConfig, MisoPin, MosiPin, Spi};
use embassy_rp::{Peri, PeripheralType};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::sdcard::{DummyCsPin, SdCard};
use embedded_sdmmc::{TimeSource, Timestamp};

type StorageSpi = Spi<'static, SPI0, Blocking>;
type StorageSpiDevice = ExclusiveDevice<StorageSpi, DummyCsPin, SdDelay>;
type StorageCard = SdCard<StorageSpiDevice, Output<'static>, SdDelay>;

pub type StorageLoggerEngine = MicrosdLogger<StorageCard, FixedTimeSource, 8, 4, 1>;

#[derive(Clone, Copy, Debug)]
pub struct MicrosdStorageConfig {
    pub sd_spi_frequency_hz: u32,
    pub sd_flush_every_lines: usize,
}

pub struct StoragePins<Sck, Mosi, Miso, Cs>
where
    Sck: PeripheralType + 'static,
    Mosi: PeripheralType + 'static,
    Miso: PeripheralType + 'static,
    Cs: PeripheralType + 'static,
{
    pub sck: Peri<'static, Sck>,
    pub mosi: Peri<'static, Mosi>,
    pub miso: Peri<'static, Miso>,
    pub cs: Peri<'static, Cs>,
}

pub fn build_logger_engine<Sck, Mosi, Miso, Cs>(
    pins: StoragePins<Sck, Mosi, Miso, Cs>,
    spi: Peri<'static, SPI0>,
    config: MicrosdStorageConfig,
) -> Result<StorageLoggerEngine, LogError>
where
    Sck: ClkPin<SPI0>,
    Mosi: MosiPin<SPI0>,
    Miso: MisoPin<SPI0>,
    Cs: Pin,
{
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = config.sd_spi_frequency_hz;

    let spi = Spi::new_blocking(spi, pins.sck, pins.mosi, pins.miso, spi_config);
    let cs = Output::new(pins.cs, Level::High);
    let spi_device =
        ExclusiveDevice::new(spi, DummyCsPin, SdDelay).map_err(|_| LogError::Device)?;
    let sdcard = SdCard::new(spi_device, cs, SdDelay);

    let mut logger_config = MicrosdLoggerConfig::default();
    logger_config.flush_every_lines = config.sd_flush_every_lines.max(1);
    MicrosdLogger::<_, _, 8, 4, 1>::new(sdcard, FixedTimeSource, logger_config)
}

pub struct FixedTimeSource;

impl TimeSource for FixedTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct SdDelay;

impl embedded_hal::delay::DelayNs for SdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let micros = (ns as u64).div_ceil(1_000);
        embassy_time::block_for(embassy_time::Duration::from_micros(micros.max(1)));
    }
}
