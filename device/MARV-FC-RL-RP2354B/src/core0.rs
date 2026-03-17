use common::drivers::bmi088::{
    AccelRange as Bmi088AccelRange, Bmi088, GyroRange as Bmi088GyroRange,
};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::logging::{LogSinkState, LoggedSensor};
use common::services::acquisition::{
    BarometerSampleSubscriber, Bmi088ImuSource, GpsFixSampleSubscriber, ImuProducerConfig,
    ImuSampleChannel, ImuSampleSubscriber, Lsm6dsv32xImuSource, MagnetometerSampleSubscriber,
};
use common::services::logging::{LogChannel, LogSinkStateChannel, SensorSnapshotLogger};
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::fast_loop::imu_acquisition::run_fast_imu_acquisition_task;
use common::tasks::medium_loop::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::{InterruptExecutor, SendSpawner, Spawner};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::buses::SensorSpiBus;
use crate::channels;
use crate::config::{DeviceConfig, STATUS_HEARTBEAT_PERIOD_MS};
use crate::core1;
use crate::pinmap;
use crate::resources::{DeviceResources, SensorPins};
use crate::sensor_spi::{SharedSensorSpiBus, SharedSpiDevice};
use crate::storage;
use crate::watchdog;
use common::drivers::lsm6dsv32x::{
    AccelRange as Lsm6dsv32xAccelRange, GyroRange as Lsm6dsv32xGyroRange, Lsm6dsv32x,
};

// Cover at least one 10 ms logging interval at a 1 kHz publish rate, with jitter margin.
const IMU_CHANNEL_DEPTH: usize = 16;
const IMU_CHANNEL_SUBS: usize = 2;
const IMU_CHANNEL_PUBS: usize = 1;
const LOG_CHANNEL_DEPTH: usize = 32;
const LOG_SINK_STATE_DEPTH: usize = 4;
const CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY: interrupt::Priority = interrupt::Priority::P2;

type FcImuChannel = ImuSampleChannel<
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
type FcImuSubscriber = ImuSampleSubscriber<
    'static,
    CriticalSectionRawMutex,
    IMU_CHANNEL_DEPTH,
    IMU_CHANNEL_SUBS,
    IMU_CHANNEL_PUBS,
>;
type FcLogSinkStateReceiver =
    Receiver<'static, CriticalSectionRawMutex, LogSinkState, LOG_SINK_STATE_DEPTH>;
type DisabledBarometerSubscriber =
    BarometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
type DisabledMagnetometerSubscriber =
    MagnetometerSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
type DisabledGpsSubscriber = GpsFixSampleSubscriber<'static, CriticalSectionRawMutex, 1, 1, 1>;
type Bmi088Source = Bmi088ImuSource<SharedSpiDevice, SharedSpiDevice>;
type Lsm6dsv32xSource = Lsm6dsv32xImuSource<SharedSpiDevice>;

static IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
static AUX_IMU_CHANNEL: FcImuChannel = FcImuChannel::new();
static LOG_CHANNEL: LogChannel<CriticalSectionRawMutex, LOG_CHANNEL_DEPTH> = LogChannel::new();
static LOG_SINK_STATE_CHANNEL: LogSinkStateChannel<CriticalSectionRawMutex, LOG_SINK_STATE_DEPTH> =
    LogSinkStateChannel::new();
static SENSOR_SPI_BUS: SharedSensorSpiBus = Mutex::new(None);
static CORE0_TIME_SENSITIVE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
#[unsafe(no_mangle)]
unsafe extern "C" fn SWI_IRQ_0() {
    unsafe {
        CORE0_TIME_SENSITIVE_EXECUTOR.on_interrupt();
    }
}

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

fn build_sensor_sources(pins: SensorPins, bus: SensorSpiBus) -> (Bmi088Source, Lsm6dsv32xSource) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;

    let spi = Spi::new(
        bus.spi, pins.sck, pins.mosi, pins.miso, bus.tx_dma, bus.rx_dma, spi_config,
    );
    let shared_bus: &'static SharedSensorSpiBus = &SENSOR_SPI_BUS;
    if let Ok(mut bus_guard) = shared_bus.try_lock() {
        *bus_guard = Some(spi);
    } else {
        panic!("sensor SPI bus already initialized");
    }
    let accel_cs = Output::new(pins.bmi088_accel_cs, Level::High);
    let gyro_cs = Output::new(pins.bmi088_gyro_cs, Level::High);
    let lsm6dsv32x_cs = Output::new(pins.lsm6dsv32x_cs, Level::High);

    let bmi088_driver = Bmi088::new(
        SharedSpiDevice::new(shared_bus, accel_cs).unwrap(),
        SharedSpiDevice::new(shared_bus, gyro_cs).unwrap(),
    );
    let lsm6dsv32x_driver =
        Lsm6dsv32x::new(SharedSpiDevice::new(shared_bus, lsm6dsv32x_cs).unwrap());

    (
        Bmi088ImuSource::new(
            bmi088_driver,
            Bmi088AccelRange::G6,
            Bmi088GyroRange::Dps2000,
        ),
        Lsm6dsv32xImuSource::new(
            lsm6dsv32x_driver,
            Lsm6dsv32xAccelRange::G16,
            Lsm6dsv32xGyroRange::Dps2000,
        ),
    )
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
async fn lsm6dsv32x_acquisition_task(mut source: Lsm6dsv32xSource, config: ImuProducerConfig) -> ! {
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    run_fast_imu_acquisition_task(
        &AUX_IMU_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        config,
        |error| warn!("LSM6DSV32X acquisition error: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn sensor_logging_task(
    mut logger: SensorSnapshotLogger,
    mut imu_subscriber: Option<FcImuSubscriber>,
    mut aux_imu_subscriber: Option<FcImuSubscriber>,
    sink_state_receiver: Option<FcLogSinkStateReceiver>,
) -> ! {
    let mut delay = EmbassyDelay;

    run_core0_sensor_logging_task(
        &mut logger,
        &LOG_CHANNEL,
        imu_subscriber.as_mut(),
        aux_imu_subscriber.as_mut(),
        None::<&mut DisabledBarometerSubscriber>,
        None::<&mut DisabledMagnetometerSubscriber>,
        None::<&mut DisabledGpsSubscriber>,
        sink_state_receiver.as_ref(),
        &mut delay,
        measurement_now,
        |error| warn!("sensor logging error: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn sd_logging_task(engine: storage::StorageLoggerEngine) -> ! {
    run_sd_logging_task(
        &LOG_CHANNEL,
        Some(&LOG_SINK_STATE_CHANNEL),
        engine,
        |error| warn!("sd logging error: {:?}", error),
    )
    .await
}

fn start_core0_time_sensitive_executor() -> SendSpawner {
    interrupt::SWI_IRQ_0.set_priority(CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY);
    CORE0_TIME_SENSITIVE_EXECUTOR.start(interrupt::SWI_IRQ_0)
}

pub async fn run(_spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins,
        buses,
        watchdog: _watchdog,
        system: _system,
    } = resources;
    let config = DeviceConfig::default();
    let time_sensitive_spawner = start_core0_time_sensitive_executor();
    let SensorPins {
        sck,
        mosi,
        miso,
        bmi088_accel_cs,
        bmi088_gyro_cs,
        lsm6dsv32x_cs,
    } = pins.sensors;
    let sensor_pins = SensorPins {
        sck,
        mosi,
        miso,
        bmi088_accel_cs,
        bmi088_gyro_cs,
        lsm6dsv32x_cs,
    };
    let storage_pins = pins.storage;
    let sensor_bus = buses.sensors;
    let storage_bus = buses.storage;

    let (mut bmi088_source, mut lsm6dsv32x_source) = build_sensor_sources(sensor_pins, sensor_bus);
    let mut init_delay = EmbassyDelay;
    let bmi088_ready = match bmi088_source.driver_mut().init(&mut init_delay).await {
        Ok(()) => {
            info!("BMI088 acquisition source initialized");
            true
        }
        Err(error) => {
            warn!("BMI088 initialization failed: {:?}", error);
            false
        }
    };
    let lsm6dsv32x_ready = match lsm6dsv32x_source.driver_mut().init(&mut init_delay).await {
        Ok(()) => {
            info!("LSM6DSV32X acquisition source initialized");
            true
        }
        Err(error) => {
            warn!("LSM6DSV32X initialization failed: {:?}", error);
            false
        }
    };

    let imu_config = ImuProducerConfig {
        period_ms: imu_period_ms(config.fast_loop_hz),
    };
    if bmi088_ready {
        time_sensitive_spawner
            .spawn(bmi088_acquisition_task(bmi088_source, imu_config))
            .unwrap();
    } else {
        warn!("BMI088 acquisition task not started because initialization did not complete");
    }
    if lsm6dsv32x_ready {
        time_sensitive_spawner
            .spawn(lsm6dsv32x_acquisition_task(lsm6dsv32x_source, imu_config))
            .unwrap();
    } else {
        warn!("LSM6DSV32X acquisition task not started because initialization did not complete");
    }

    if config.logging.enabled {
        match storage::build_logger_engine(storage_pins, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => match SensorSnapshotLogger::new(
                    log_path.as_str(),
                    config.logging.sensor_snapshot,
                ) {
                    Ok(mut logger) => {
                        if !bmi088_ready {
                            logger.note_sensor_fault(LoggedSensor::Imu);
                        }
                        if !lsm6dsv32x_ready {
                            logger.note_sensor_fault(LoggedSensor::AuxImu);
                        }

                        let imu_subscriber = if logger.config().sensors.imu && bmi088_ready {
                            Some(IMU_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };
                        let aux_imu_subscriber =
                            if logger.config().sensors.aux_imu && lsm6dsv32x_ready {
                                Some(AUX_IMU_CHANNEL.subscriber().unwrap())
                            } else {
                                None
                            };
                        let sink_state_receiver = Some(LOG_SINK_STATE_CHANNEL.receiver());

                        _spawner.spawn(sd_logging_task(engine)).unwrap();
                        time_sensitive_spawner
                            .spawn(sensor_logging_task(
                                logger,
                                imu_subscriber,
                                aux_imu_subscriber,
                                sink_state_receiver,
                            ))
                            .unwrap();
                        info!(
                            "core0 sensor logging instantiated on dedicated executor: {}",
                            log_path.as_str()
                        );
                    }
                    Err(error) => warn!("sensor logger config invalid: {:?}", error),
                },
                Err(error) => warn!("sd log file allocation failed: {:?}", error),
            },
            Err(error) => warn!("sd logger engine unavailable: {:?}", error),
        }
    } else {
        info!("logging disabled by device config");
    }

    info!("MARV-FC-RL-RP2354B resource graph initialized");
    info!(
        "core0 owns SPI1 sensors, I2C domains, actuator outputs, time-sensitive logging, and watchdog feed"
    );
    info!(
        "core0 time-sensitive executor: SWI_IRQ_0 priority {:?}",
        CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY
    );
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
