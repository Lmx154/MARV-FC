use common::drivers::bmp390::{
    BMP390_ADDR_SDO_HIGH, BMP390_ADDR_SDO_LOW, BMP390_CHIP_ID, Bmp390, read_bmp390_chip_id,
};
use common::drivers::mpu6050::{
    MPU6050_ADDR_AD0_HIGH, MPU6050_ADDR_AD0_LOW, MPU6050_CHIP_ID, Mpu6050,
    read_mpu6050_chip_id,
};
use common::interfaces::sensors::{BarometerSource, ImuSource};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::sensor::{BarometerSampleStamped, ImuSampleStamped};
use common::services::acquisition::{Bmp390BarometerSource, Mpu6050ImuSource};
use common::services::logging::{
    SensorSnapshotLogger, SensorSnapshotLoggerError, TryEnqueueLogError,
};
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::utilities::time::MeasurementTimestamp;
use common::utils::delay::DelayMs;
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Spawner, task};
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use static_cell::StaticCell;

use crate::channels::{
    BAROMETER_CHANNEL, IMU_CHANNEL, LOG_CHANNEL, LOG_SINK_STATE_CHANNEL,
    RecoveryBarometerSubscriber, RecoveryImuSubscriber, RecoveryLogSinkStateReceiver,
};
use crate::config::{Bmp390RuntimeConfig, DeviceConfig, Mpu6050RuntimeConfig};
use crate::core1;
use crate::pinmap;
use crate::resources::{DeviceResources, RecoverySensorPins, SystemResources};
use crate::storage;
use crate::usb_cdc;

const SENSOR_RETRY_PERIOD_MS: u32 = 1_000;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

type RecoveryI2c = I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>;
type SharedRecoveryI2cBus = Mutex<NoopRawMutex, RecoveryI2c>;
type SharedRecoveryI2cDevice = I2cDevice<'static, NoopRawMutex, RecoveryI2c>;

static I2C_BUS: StaticCell<SharedRecoveryI2cBus> = StaticCell::new();

struct EmbassyClock;

impl MonotonicClock for EmbassyClock {
    fn now(&self) -> MeasurementTimestamp {
        MeasurementTimestamp::from_micros(embassy_time::Instant::now().as_micros())
    }
}

struct EmbassyDelay;

impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms as u64)).await;
    }
}

fn measurement_now() -> MeasurementTimestamp {
    MeasurementTimestamp::from_micros(embassy_time::Instant::now().as_micros())
}

fn deadline_after_ms(now: MeasurementTimestamp, delay_ms: u32) -> MeasurementTimestamp {
    MeasurementTimestamp::from_micros(
        now.as_micros()
            .saturating_add(u64::from(delay_ms.max(1)) * 1_000),
    )
}

fn next_due_delay_ms(
    first_due: MeasurementTimestamp,
    second_due: MeasurementTimestamp,
    now: MeasurementTimestamp,
) -> u32 {
    first_due
        .min(second_due)
        .saturating_delta_since(now)
        .as_micros()
        .div_ceil(1_000)
        .max(1) as u32
}

async fn probe_bmp390_address<I2C>(i2c: &mut I2C, preferred_address: u8) -> Option<u8>
where
    I2C: AsyncI2c,
{
    let candidates = if preferred_address == BMP390_ADDR_SDO_HIGH {
        [BMP390_ADDR_SDO_HIGH, BMP390_ADDR_SDO_LOW]
    } else {
        [BMP390_ADDR_SDO_LOW, BMP390_ADDR_SDO_HIGH]
    };

    for address in candidates {
        match read_bmp390_chip_id(i2c, address).await {
            Ok(BMP390_CHIP_ID) => {
                info!("BMP390 detected at I2C address 0x{=u8:02X}", address);
                return Some(address);
            }
            Ok(chip_id) => warn!(
                "BMP390 probe at 0x{=u8:02X} returned chip id 0x{=u8:02X}",
                address, chip_id
            ),
            Err(_) => warn!("BMP390 probe at 0x{=u8:02X} failed", address),
        }
    }

    None
}

async fn probe_mpu6050_address<I2C>(i2c: &mut I2C, preferred_address: u8) -> Option<u8>
where
    I2C: AsyncI2c,
{
    let candidates = if preferred_address == MPU6050_ADDR_AD0_HIGH {
        [MPU6050_ADDR_AD0_HIGH, MPU6050_ADDR_AD0_LOW]
    } else {
        [MPU6050_ADDR_AD0_LOW, MPU6050_ADDR_AD0_HIGH]
    };

    for address in candidates {
        match read_mpu6050_chip_id(i2c, address).await {
            Ok(MPU6050_CHIP_ID) | Ok(MPU6050_ADDR_AD0_HIGH) => {
                info!("MPU6050 detected at I2C address 0x{=u8:02X}", address);
                return Some(address);
            }
            Ok(chip_id) => warn!(
                "MPU6050 probe at 0x{=u8:02X} returned chip id 0x{=u8:02X}",
                address, chip_id
            ),
            Err(_) => warn!("MPU6050 probe at 0x{=u8:02X} failed", address),
        }
    }

    None
}

async fn try_init_bmp390_source<'a>(
    shared_bus: &'static SharedRecoveryI2cBus,
    config: Bmp390RuntimeConfig,
) -> Option<Bmp390BarometerSource<SharedRecoveryI2cDevice>> {
    let detected_address = {
        let mut probe_bus = I2cDevice::new(shared_bus);
        probe_bmp390_address(&mut probe_bus, config.address).await
    }?;

    let mut source = Bmp390BarometerSource::new(Bmp390::new(
        I2cDevice::new(shared_bus),
        detected_address,
    ));
    let mut delay = EmbassyDelay;

    match source.driver_mut().init(&mut delay).await {
        Ok(()) => {
            info!("BMP390 initialized");
            Some(source)
        }
        Err(error) => {
            warn!("BMP390 initialization failed: {:?}", error);
            None
        }
    }
}

async fn try_init_mpu6050_source<'a>(
    shared_bus: &'static SharedRecoveryI2cBus,
    config: Mpu6050RuntimeConfig,
) -> Option<Mpu6050ImuSource<SharedRecoveryI2cDevice>> {
    let detected_address = {
        let mut probe_bus = I2cDevice::new(shared_bus);
        probe_mpu6050_address(&mut probe_bus, config.address).await
    }?;

    let driver = Mpu6050::new(I2cDevice::new(shared_bus), detected_address);
    let mut source = Mpu6050ImuSource::new(driver, config.accel_range, config.gyro_range);
    let mut delay = EmbassyDelay;

    match source.driver_mut().init(&mut delay).await {
        Ok(()) => {
            info!("MPU6050 initialized");
            Some(source)
        }
        Err(error) => {
            warn!("MPU6050 initialization failed: {:?}", error);
            None
        }
    }
}

#[task]
async fn sd_logging_task(engine: storage::StorageLoggerEngine) -> ! {
    run_sd_logging_task(
        &LOG_CHANNEL,
        Some(&LOG_SINK_STATE_CHANNEL),
        engine,
        |error| warn!("sd logging error: {:?}", error),
    )
    .await
}

#[task]
async fn sensor_logging_task(
    mut logger: SensorSnapshotLogger,
    mut imu_subscriber: Option<RecoveryImuSubscriber>,
    mut barometer_subscriber: Option<RecoveryBarometerSubscriber>,
    sink_state_receiver: RecoveryLogSinkStateReceiver,
) -> ! {
    let mut delay = EmbassyDelay;

    loop {
        if let Some(subscriber) = imu_subscriber.as_mut() {
            logger.drain_imu(subscriber);
        }
        if let Some(subscriber) = barometer_subscriber.as_mut() {
            logger.drain_barometer(subscriber);
        }
        logger.drain_sink_states(&sink_state_receiver);

        if let Err(error) = logger.emit_snapshot(&LOG_CHANNEL, measurement_now()) {
            if !matches!(
                error,
                SensorSnapshotLoggerError::Queue(TryEnqueueLogError::ChannelFull)
            ) {
                warn!("sensor logging error: {:?}", error);
            }
        }

        delay.delay_ms(logger.period_ms()).await;
    }
}

#[task]
async fn recovery_i2c_sensor_task(
    sda: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_0>,
    scl: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_1>,
    bus: crate::buses::RecoveryI2cBus,
    bmp390_config: Bmp390RuntimeConfig,
    mpu6050_config: Mpu6050RuntimeConfig,
) -> ! {
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = bmp390_config.i2c_frequency_hz;

    let i2c = I2c::new_async(bus.i2c, scl, sda, Irqs, i2c_config);
    let shared_bus: &'static SharedRecoveryI2cBus = I2C_BUS.init(Mutex::new(i2c));
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;
    let mut barometer_source: Option<Bmp390BarometerSource<SharedRecoveryI2cDevice>> = None;
    let mut imu_source: Option<Mpu6050ImuSource<SharedRecoveryI2cDevice>> = None;
    let mut next_barometer_action = measurement_now();
    let mut next_imu_action = measurement_now();

    Timer::after(Duration::from_millis(100)).await;

    loop {
        let now = measurement_now();
        let mut did_work = false;

        if now >= next_barometer_action {
            did_work = true;

            if let Some(source) = barometer_source.as_mut() {
                let timestamp = clock.now();
                match source.read_barometer_sample().await {
                    Ok(sample) => {
                        BAROMETER_CHANNEL
                            .immediate_publisher()
                            .publish_immediate(BarometerSampleStamped { timestamp, sample });
                    }
                    Err(error) => warn!("BMP390 acquisition error: {:?}", error),
                }

                next_barometer_action = deadline_after_ms(measurement_now(), bmp390_config.period_ms);
            } else {
                barometer_source = try_init_bmp390_source(shared_bus, bmp390_config).await;

                if barometer_source.is_none() {
                    warn!(
                        "BMP390 not detected on 0x76/0x77; Bosch requires CSB high at POR for I2C and SDO strapped to GND or VDDIO"
                    );
                }

                next_barometer_action = deadline_after_ms(
                    measurement_now(),
                    if barometer_source.is_some() {
                        bmp390_config.period_ms
                    } else {
                        SENSOR_RETRY_PERIOD_MS
                    },
                );
            }
        }

        let now = measurement_now();
        if now >= next_imu_action {
            did_work = true;

            if let Some(source) = imu_source.as_mut() {
                let timestamp = clock.now();
                match source.read_imu_sample().await {
                    Ok(sample) => {
                        IMU_CHANNEL
                            .immediate_publisher()
                            .publish_immediate(ImuSampleStamped::new(timestamp, sample));
                    }
                    Err(error) => warn!("MPU6050 acquisition error: {:?}", error),
                }

                next_imu_action = deadline_after_ms(measurement_now(), mpu6050_config.period_ms);
            } else {
                imu_source = try_init_mpu6050_source(shared_bus, mpu6050_config).await;

                if imu_source.is_none() {
                    warn!(
                        "MPU6050 not detected on 0x68/0x69; verify AD0 strap and device power"
                    );
                }

                next_imu_action = deadline_after_ms(
                    measurement_now(),
                    if imu_source.is_some() {
                        mpu6050_config.period_ms
                    } else {
                        SENSOR_RETRY_PERIOD_MS
                    },
                );
            }
        }

        if !did_work {
            delay
                .delay_ms(next_due_delay_ms(
                    next_barometer_action,
                    next_imu_action,
                    measurement_now(),
                ))
                .await;
        }
    }
}

pub async fn run(spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins:
            crate::resources::PinResources {
                sensors:
                    RecoverySensorPins {
                        sda,
                        scl,
                        buzzer_continuity_adc: _buzzer_continuity_adc,
                        drogue_continuity_adc: _drogue_continuity_adc,
                        main_continuity_adc: _main_continuity_adc,
                    },
                storage,
            },
        buses:
            crate::buses::BusResources {
                recovery_i2c,
                recovery_adc: _recovery_adc,
                storage: storage_bus,
            },
        system:
            SystemResources {
                usb,
                flash: _flash,
                core1: _core1,
            },
    } = resources;
    let config = DeviceConfig::default();

    usb_cdc::spawn(&spawner, usb);
    spawner
        .spawn(recovery_i2c_sensor_task(
            sda,
            scl,
            recovery_i2c,
            config.bmp390,
            config.mpu6050,
        ))
        .unwrap();

    if config.logging.enabled {
        match storage::build_logger_engine(storage, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => {
                    match SensorSnapshotLogger::new(
                        log_path.as_str(),
                        config.logging.sensor_snapshot,
                    ) {
                        Ok(logger) => {
                            let imu_subscriber = if logger.config().sensors.imu {
                                Some(IMU_CHANNEL.subscriber().unwrap())
                            } else {
                                None
                            };
                            let barometer_subscriber = if logger.config().sensors.barometer {
                                Some(BAROMETER_CHANNEL.subscriber().unwrap())
                            } else {
                                None
                            };
                            let sink_state_receiver = LOG_SINK_STATE_CHANNEL.receiver();

                            spawner.spawn(sd_logging_task(engine)).unwrap();
                            spawner
                                .spawn(sensor_logging_task(
                                    logger,
                                    imu_subscriber,
                                    barometer_subscriber,
                                    sink_state_receiver,
                                ))
                                .unwrap();
                            info!("sensor snapshot logging initialized: {}", log_path.as_str());
                        }
                        Err(error) => warn!("sensor logger config invalid: {:?}", error),
                    }
                }
                Err(error) => warn!("sd log file allocation failed: {:?}", error),
            },
            Err(error) => warn!("sd logger engine unavailable: {:?}", error),
        }
    } else {
        info!("sd logging disabled by device config");
    }

    info!("recovery-altimeter-rp2350 resource graph initialized");
    info!(
        "recovery altimeter I2C0: BMP390 + MPU6050 on GP{=u8}/GP{=u8}",
        pinmap::RECOVERY_I2C_SDA,
        pinmap::RECOVERY_I2C_SCL,
    );
    info!(
        "BMP390 runtime: addr=0x{=u8:02X} sample_period_ms={=u32}",
        config.bmp390.address, config.bmp390.period_ms,
    );
    info!(
        "MPU6050 runtime: addr=0x{=u8:02X} sample_period_ms={=u32}",
        config.mpu6050.address, config.mpu6050.period_ms,
    );
    info!(
        "recovery altimeter SPI0: SD card on GP{=u8}/GP{=u8}/GP{=u8} cs=GP{=u8}",
        pinmap::STORAGE_SPI_SCK,
        pinmap::STORAGE_SPI_MOSI,
        pinmap::STORAGE_SPI_MISO,
        pinmap::STORAGE_SPI_CS,
    );
    info!(
        "continuity ADC inputs: buzzer=GP{=u8} drogue=GP{=u8} main=GP{=u8}, core1: {}",
        pinmap::BUZZER_CONTINUITY_ADC,
        pinmap::DROGUE_CONTINUITY_ADC,
        pinmap::MAIN_CONTINUITY_ADC,
        core1::ROLE_SUMMARY,
    );

    loop {
        Timer::after(Duration::from_millis(config.status_heartbeat_period_ms)).await;
        info!("recovery-altimeter-rp2350 heartbeat");
    }
}
