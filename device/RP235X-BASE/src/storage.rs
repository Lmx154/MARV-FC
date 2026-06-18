use common::drivers::storage::{MicrosdLogger, MicrosdLoggerConfig};
use common::interfaces::storage::LogError;
use defmt::{info, warn};
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Blocking, ClkPin, Config as SpiConfig, MisoPin, MosiPin, Spi};
use embassy_rp::{Peri, PeripheralType};
use embassy_time::{Duration, block_for};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::sdcard::{DummyCsPin, SdCard};
use embedded_sdmmc::{TimeSource, Timestamp};

/// SPI clock used **only** for the SD-card initialisation handshake. The SD
/// specification requires the CMD0/CMD8/ACMD41 sequence to run at <= 400 kHz;
/// driving it at the (much faster) data clock makes most cards fail to
/// initialise, after which nothing can ever be logged. The bus is re-clocked to
/// the data rate as soon as the card is acquired.
const SD_INIT_SPI_FREQUENCY_HZ: u32 = 400_000;
/// Init-handshake attempts. A freshly powered card can need a couple of tries to
/// leave power-up idle.
const SD_INIT_ATTEMPTS: u32 = 3;
/// Blocking back-off between failed init attempts. Kept small on purpose because
/// this runs inline in the caller's boot path.
const SD_INIT_RETRY_BACKOFF_MS: u64 = 25;

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
    // Stage 1: bring the bus up at the slow, spec-compliant init clock.
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = SD_INIT_SPI_FREQUENCY_HZ;

    let spi = Spi::new_blocking(spi, pins.sck, pins.mosi, pins.miso, spi_config);
    let cs = Output::new(pins.cs, Level::High);
    let spi_device =
        ExclusiveDevice::new(spi, DummyCsPin, SdDelay).map_err(|_| LogError::Device)?;
    let sdcard = SdCard::new(spi_device, cs, SdDelay);

    // Stage 2: force the (otherwise lazy) init handshake at the slow clock.
    // `num_bytes()` runs CMD0/CMD8/ACMD41/etc.; retry a few times to ride out
    // power-up idle before giving up and continuing without SD logging.
    let mut initialized = false;
    for attempt in 1..=SD_INIT_ATTEMPTS {
        match sdcard.num_bytes() {
            Ok(bytes) => {
                info!(
                    "SD card initialised at {} Hz ({} MiB)",
                    SD_INIT_SPI_FREQUENCY_HZ,
                    bytes / (1024 * 1024)
                );
                initialized = true;
                break;
            }
            Err(_) => {
                sdcard.mark_card_uninit();
                warn!(
                    "SD init attempt {}/{} failed at {} Hz",
                    attempt, SD_INIT_ATTEMPTS, SD_INIT_SPI_FREQUENCY_HZ
                );
                if attempt < SD_INIT_ATTEMPTS {
                    block_for(Duration::from_millis(SD_INIT_RETRY_BACKOFF_MS));
                }
            }
        }
    }
    if !initialized {
        return Err(LogError::Device);
    }

    // Stage 3: the card is in data mode now, so re-clock the bus up to the data
    // rate. This is where 100 Hz throughput comes from (~40 KB/s of CSV); the
    // init clock alone could not sustain it.
    let data_frequency_hz = config.sd_spi_frequency_hz.max(SD_INIT_SPI_FREQUENCY_HZ);
    sdcard.spi(|device| device.bus_mut().set_frequency(data_frequency_hz));
    info!("SD data clock set to {} Hz", data_frequency_hz);

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
