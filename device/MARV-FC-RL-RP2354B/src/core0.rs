use defmt::{info, warn};
use common::drivers::bmi088::{AccelRange, Bmi088, GyroRange};
use common::interfaces::timing::MonotonicClock;
use common::services::acquisition::{
    BarometerSampleSubscriber, Bmi088ImuSource, GpsFixSampleSubscriber, ImuProducerConfig,
    ImuSampleChannel, ImuSampleSubscriber, MagnetometerSampleSubscriber,
};
use common::services::logging::{LogChannel, SensorSnapshotLogger};
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::fast_loop::imu_acquisition::run_fast_imu_acquisition_task;
use common::tasks::medium_loop::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::channels;
use crate::config::{DeviceConfig, STATUS_HEARTBEAT_PERIOD_MS};
use crate::core1;
use crate::buses::SensorSpiBus;
use crate::pinmap;
use crate::resources::{DeviceResources, SensorPins};
use crate::sensor_spi::{SharedSensorSpiBus, SharedSpiDevice};
use crate::storage;
use crate::watchdog;

const IMU_CHANNEL_DEPTH: usize = 8;
const IMU_CHANNEL_SUBS: usize = 2;
const IMU_CHANNEL_PUBS: usize = 1;
const LOG_CHANNEL_DEPTH: usize = 32;

type FcImuChannel =
    ImuSampleChannel<ThreadModeRawMutex, IMU_CHANNEL_DEPTH, IMU_CHANNEL_SUBS, IMU_CHANNEL_PUBS>;
type FcImuSubscriber =
    ImuSampleSubscriber<'static, ThreadModeRawMutex, IMU_CHANNEL_DEPTH, IMU_CHANNEL_SUBS, IMU_CHANNEL_PUBS>;
type DisabledBarometerSubscriber = BarometerSampleSubscriber<'static, ThreadModeRawMutex, 1, 1, 1>;
type DisabledMagnetometerSubscriber =
    MagnetometerSampleSubscriber<'static, ThreadModeRawMutex, 1, 1, 1>;
type DisabledGpsSubscriber = GpsFixSampleSubscriber<'static, ThreadModeRawMutex, 1, 1, 1>;
type Bmi088Source = Bmi088ImuSource<SharedSpiDevice, SharedSpiDevice>;

static IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
static LOG_CHANNEL: LogChannel<ThreadModeRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
static SENSOR_SPI_BUS: SharedSensorSpiBus = Mutex::new(None);

struct EmbassyClock;

impl MonotonicClock for EmbassyClock {
    fn now(&self) -> MeasurementTimestamp {
        MeasurementTimestamp::from_micros(embassy_time::Instant::now().as_micros())
    }
}

struct EmbassyDelay;

impl common::utilities::time::delay::DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms as u64)).await;
    }
}

fn imu_period_ms(fast_loop_hz: u32) -> u32 {
    let hz = fast_loop_hz.max(1);
    1_000u32.div_ceil(hz).max(1)
}

fn measurement_now() -> MeasurementTimestamp {
    MeasurementTimestamp::from_micros(embassy_time::Instant::now().as_micros())
}

fn build_bmi088_source(pins: SensorPins, bus: SensorSpiBus) -> Bmi088Source {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;

    let spi = Spi::new(
        bus.spi,
        pins.sck,
        pins.mosi,
        pins.miso,
        bus.tx_dma,
        bus.rx_dma,
        spi_config,
    );
    let shared_bus: &'static SharedSensorSpiBus = &SENSOR_SPI_BUS;
    if let Ok(mut bus_guard) = shared_bus.try_lock() {
        *bus_guard = Some(spi);
    } else {
        panic!("sensor SPI bus already initialized");
    }
    let accel_cs = Output::new(pins.bmi088_accel_cs, Level::High);
    let gyro_cs = Output::new(pins.bmi088_gyro_cs, Level::High);
    let _lsm6dsv32x_cs = Output::new(pins.lsm6dsv32x_cs, Level::High);

    let driver = Bmi088::new(
        SharedSpiDevice::new(shared_bus, accel_cs).unwrap(),
        SharedSpiDevice::new(shared_bus, gyro_cs).unwrap(),
    );

    Bmi088ImuSource::new(driver, AccelRange::G6, GyroRange::Dps2000)
}

#[embassy_executor::task]
async fn bmi088_acquisition_task(mut source: Bmi088Source, config: ImuProducerConfig) -> ! {
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    run_fast_imu_acquisition_task(
        &IMU_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        config,
        |error| warn!("BMI088 acquisition error: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn sensor_logging_task(
    mut logger: SensorSnapshotLogger,
    mut imu_subscriber: Option<FcImuSubscriber>,
) -> ! {
    let mut delay = EmbassyDelay;

    run_core0_sensor_logging_task(
        &mut logger,
        &LOG_CHANNEL,
        imu_subscriber.as_mut(),
        None::<&mut DisabledBarometerSubscriber>,
        None::<&mut DisabledMagnetometerSubscriber>,
        None::<&mut DisabledGpsSubscriber>,
        &mut delay,
        measurement_now,
        |error| warn!("sensor logging error: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn sd_logging_task(engine: storage::StorageLoggerEngine) -> ! {
    run_sd_logging_task(&LOG_CHANNEL, engine, |error| warn!("sd logging error: {:?}", error)).await
}

pub async fn run(_spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins,
        buses,
        watchdog: _watchdog,
        system: _system,
    } = resources;
    let config = DeviceConfig::default();

    let mut bmi088_source = build_bmi088_source(pins.sensors, buses.sensors);
    let mut init_delay = EmbassyDelay;
    match bmi088_source.driver_mut().init(&mut init_delay).await {
        Ok(()) => info!("BMI088 acquisition source initialized"),
        Err(error) => warn!("BMI088 initialization failed: {:?}", error),
    }

    let imu_config = ImuProducerConfig {
        period_ms: imu_period_ms(config.fast_loop_hz),
    };
    _spawner.spawn(bmi088_acquisition_task(bmi088_source, imu_config)).unwrap();

    if config.logging.enabled {
        match storage::build_logger_engine(pins.storage, buses.storage) {
            Ok(mut engine) => {
                match common::tasks::slow_loop::run_sd_card_smoke_test(&mut engine) {
                    Ok(path) => info!("sd smoke test complete: {}", path.as_str()),
                    Err(error) => warn!("sd smoke test failed: {:?}", error),
                }

                match SensorSnapshotLogger::new(
                    config.logging.path,
                    config.logging.sensor_snapshot,
                ) {
                    Ok(logger) => {
                        let imu_subscriber = if logger.config().sensors.imu {
                            Some(IMU_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };

                        _spawner.spawn(sd_logging_task(engine)).unwrap();
                        _spawner
                            .spawn(sensor_logging_task(logger, imu_subscriber))
                            .unwrap();
                        info!("core0 sensor logging instantiated");
                    }
                    Err(error) => warn!("sensor logger config invalid: {:?}", error),
                }
            }
            Err(error) => warn!("sd logger engine unavailable: {:?}", error),
        }
    } else {
        info!("logging disabled by device config");
    }

    info!("MARV-FC-RL-RP2354B resource graph initialized");
    info!("core0 owns SPI1 sensors, I2C domains, actuator outputs, time-sensitive logging, and watchdog feed");
    info!("core1 planned ownership: {}", core1::ROLE_SUMMARY);
    info!("watchdog authority: {}", watchdog::FEED_AUTHORITY);
    info!(
        "channel plan: core0_local={} core1_local={} cross_core_bridges={}",
        channels::TOPOLOGY.core0_local.len(),
        channels::TOPOLOGY.core1_local.len(),
        channels::TOPOLOGY.cross_core_bridges.len()
    );
    info!(
        "key pins: radio=GP{=u8}/GP{=u8} sensor-spi=GP{=u8}/GP{=u8}/GP{=u8} sd-cs=GP{=u8}",
        pinmap::FC_RADIO_TX,
        pinmap::FC_RADIO_RX,
        pinmap::SENSOR_SPI_SCK,
        pinmap::SENSOR_SPI_MOSI,
        pinmap::SENSOR_SPI_MISO,
        pinmap::STORAGE_SPI_CS,
    );
    info!(
        "fast_loop_hz={} watchdog_timeout_ms={}",
        config.fast_loop_hz, config.watchdog_timeout_ms
    );

    loop {
        Timer::after(Duration::from_millis(STATUS_HEARTBEAT_PERIOD_MS)).await;
        info!("core0 heartbeat");
    }
}
