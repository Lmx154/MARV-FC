use core::cell::RefCell;

use common::drivers::bmi088::{
    AccelRange as Bmi088AccelRange, Bmi088, GyroRange as Bmi088GyroRange,
};
use common::drivers::lsm6dsv32x::{
    AccelRange as Lsm6dsv32xAccelRange, GyroRange as Lsm6dsv32xGyroRange, Lsm6dsv32x,
};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::logging::LoggedSensor;
use common::services::acquisition::{
    Bmi088ImuSource, Lsm6dsv32xImuSource, Spi1ImuServiceConfig, Spi1ImuServiceError,
    Spi1ImuSourceSchedule, run_spi1_imu_service,
};
use common::services::logging::SensorSnapshotLogger;
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::medium_loop::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::{InterruptExecutor, SendSpawner, Spawner};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Duration, Timer};

use crate::buses::SensorSpiBus;
use crate::channels::{
    self, AUX_IMU_CHANNEL, DisabledBarometerSubscriber, DisabledGpsSubscriber,
    DisabledMagnetometerSubscriber, FcImuSubscriber, FcLogSinkStateReceiver, FcSensorFaultReceiver,
    IMU_CHANNEL, IMU_INIT_SIGNAL, LOG_CHANNEL, LOG_SINK_STATE_CHANNEL, SENSOR_FAULT_CHANNEL,
};
use crate::config::{DeviceConfig, STATUS_HEARTBEAT_PERIOD_MS};
use crate::core1;
use crate::pinmap;
use crate::resources::{DeviceResources, SensorPins};
use crate::sensor_spi::{SharedSensorSpiBus, SharedSpiDevice};
use crate::storage;
use crate::watchdog;

const CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY: interrupt::Priority = interrupt::Priority::P2;
const SENSOR_SPI_FREQUENCY_HZ: u32 = 1_000_000;

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

fn try_report_sensor_fault(sensor: LoggedSensor) {
    let _ = SENSOR_FAULT_CHANNEL.try_send(sensor);
}

#[embassy_executor::task]
async fn spi1_imu_service_task(
    pins: SensorPins,
    bus: SensorSpiBus,
    config: Spi1ImuServiceConfig,
) -> ! {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = SENSOR_SPI_FREQUENCY_HZ;

    let spi = Spi::new(
        bus.spi, pins.sck, pins.mosi, pins.miso, bus.tx_dma, bus.rx_dma, spi_config,
    );
    let shared_bus: SharedSensorSpiBus = RefCell::new(spi);
    let accel_cs = Output::new(pins.bmi088_accel_cs, Level::High);
    let gyro_cs = Output::new(pins.bmi088_gyro_cs, Level::High);
    let lsm6dsv32x_cs = Output::new(pins.lsm6dsv32x_cs, Level::High);

    let bmi088_driver = Bmi088::new(
        SharedSpiDevice::new(&shared_bus, accel_cs).unwrap(),
        SharedSpiDevice::new(&shared_bus, gyro_cs).unwrap(),
    );
    let lsm6dsv32x_driver =
        Lsm6dsv32x::new(SharedSpiDevice::new(&shared_bus, lsm6dsv32x_cs).unwrap());

    let mut bmi088_source = Bmi088ImuSource::new(
        bmi088_driver,
        Bmi088AccelRange::G6,
        Bmi088GyroRange::Dps2000,
    );
    let mut lsm6dsv32x_source = Lsm6dsv32xImuSource::new(
        lsm6dsv32x_driver,
        Lsm6dsv32xAccelRange::G16,
        Lsm6dsv32xGyroRange::Dps2000,
    );

    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    let bmi088_ready = match bmi088_source.driver_mut().init(&mut delay).await {
        Ok(()) => {
            info!("BMI088 acquisition source initialized");
            true
        }
        Err(error) => {
            warn!("BMI088 initialization failed: {:?}", error);
            false
        }
    };
    let lsm6dsv32x_ready = match lsm6dsv32x_source.driver_mut().init(&mut delay).await {
        Ok(()) => {
            info!("LSM6DSV32X acquisition source initialized");
            true
        }
        Err(error) => {
            warn!("LSM6DSV32X initialization failed: {:?}", error);
            false
        }
    };

    IMU_INIT_SIGNAL.signal(channels::ImuInitReport {
        imu_ready: bmi088_ready,
        aux_imu_ready: lsm6dsv32x_ready,
    });

    if !bmi088_ready {
        warn!("BMI088 will remain unpublished until SPI1 service retry logic is added");
    }
    if !lsm6dsv32x_ready {
        warn!("LSM6DSV32X will remain unpublished until SPI1 service retry logic is added");
    }

    run_spi1_imu_service(
        &IMU_CHANNEL,
        &mut bmi088_source,
        &AUX_IMU_CHANNEL,
        &mut lsm6dsv32x_source,
        &clock,
        &mut delay,
        Spi1ImuServiceConfig {
            primary: Spi1ImuSourceSchedule::new(bmi088_ready, config.primary.period_ms),
            auxiliary: Spi1ImuSourceSchedule::new(lsm6dsv32x_ready, config.auxiliary.period_ms),
        },
        |error| match error {
            Spi1ImuServiceError::Primary(error) => {
                warn!("BMI088 acquisition error: {:?}", error);
                try_report_sensor_fault(LoggedSensor::Imu);
            }
            Spi1ImuServiceError::Auxiliary(error) => {
                warn!("LSM6DSV32X acquisition error: {:?}", error);
                try_report_sensor_fault(LoggedSensor::AuxImu);
            }
        },
    )
    .await
}

#[embassy_executor::task]
async fn sensor_logging_task(
    mut logger: SensorSnapshotLogger,
    mut imu_subscriber: Option<FcImuSubscriber>,
    mut aux_imu_subscriber: Option<FcImuSubscriber>,
    sink_state_receiver: Option<FcLogSinkStateReceiver>,
    sensor_fault_receiver: Option<FcSensorFaultReceiver>,
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
        sensor_fault_receiver.as_ref(),
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
    let sensor_pins = pins.sensors;
    let storage_pins = pins.storage;
    let sensor_bus = buses.sensors;
    let storage_bus = buses.storage;

    let imu_period_ms = imu_period_ms(config.fast_loop_hz);
    let imu_config = Spi1ImuServiceConfig {
        primary: Spi1ImuSourceSchedule::new(true, imu_period_ms),
        auxiliary: Spi1ImuSourceSchedule::new(true, imu_period_ms),
    };
    time_sensitive_spawner
        .spawn(spi1_imu_service_task(sensor_pins, sensor_bus, imu_config))
        .unwrap();
    let imu_init = IMU_INIT_SIGNAL.wait().await;

    if config.logging.enabled {
        match storage::build_logger_engine(storage_pins, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => match SensorSnapshotLogger::new(
                    log_path.as_str(),
                    config.logging.sensor_snapshot,
                ) {
                    Ok(mut logger) => {
                        if !imu_init.imu_ready {
                            logger.note_sensor_fault(LoggedSensor::Imu);
                        }
                        if !imu_init.aux_imu_ready {
                            logger.note_sensor_fault(LoggedSensor::AuxImu);
                        }

                        let imu_subscriber = if logger.config().sensors.imu && imu_init.imu_ready {
                            Some(IMU_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };
                        let aux_imu_subscriber =
                            if logger.config().sensors.aux_imu && imu_init.aux_imu_ready {
                                Some(AUX_IMU_CHANNEL.subscriber().unwrap())
                            } else {
                                None
                            };
                        let sink_state_receiver = Some(LOG_SINK_STATE_CHANNEL.receiver());
                        let sensor_fault_receiver = Some(SENSOR_FAULT_CHANNEL.receiver());

                        _spawner.spawn(sd_logging_task(engine)).unwrap();
                        time_sensitive_spawner
                            .spawn(sensor_logging_task(
                                logger,
                                imu_subscriber,
                                aux_imu_subscriber,
                                sink_state_receiver,
                                sensor_fault_receiver,
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
