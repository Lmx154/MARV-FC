use common::drivers::bmp390::{
    BMP390_ADDR_SDO_HIGH, BMP390_ADDR_SDO_LOW, BMP390_CHIP_ID, Bmp390, read_bmp390_chip_id,
};
use common::drivers::mpu6050::{
    MPU6050_ADDR_AD0_HIGH, MPU6050_ADDR_AD0_LOW, MPU6050_CHIP_ID, Mpu6050, read_mpu6050_chip_id,
};
use common::interfaces::sensors::{BarometerSource, ImuSource};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::logging::LoggedSensor;
use common::messages::runtime::FlightPhase;
use common::messages::sensor::{BarometerSampleStamped, ImuSampleStamped};
use common::policies::modes::phase_after_init;
use common::services::acquisition::{Bmp390BarometerSource, Mpu6050ImuSource};
use common::services::health::LivenessUpdate;
use common::services::hil::{HilBootDecision, HilBootSelector, HilControlRuntime, SensorBackend};
use common::services::logging::{
    SensorSnapshotLogger, SensorSnapshotLoggerError, TryEnqueueLogError,
};
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::medium_loop::sensor_logging::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use common::utils::delay::DelayMs;
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Spawner, task};
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use static_cell::StaticCell;

use crate::channels::{
    BAROMETER_CHANNEL, FLIGHT_PHASE_CHANNEL, HIL_BOOT_SIGNAL, HIL_CONTROL_COMMAND_CHANNEL,
    HIL_EGRESS_CHANNEL, HIL_SESSION_STATE_CHANNEL, IMU_CHANNEL, LOG_CHANNEL,
    LOG_SINK_STATE_CHANNEL, RecoveryBarometerSubscriber, RecoveryFlightPhaseSubscriber,
    RecoveryHilControlCommandReceiver, RecoveryHilEgressSender, RecoveryImuSubscriber,
    RecoveryLogSinkStateReceiver, RecoveryTimeSubscriber, RecoveryWatchdogLivenessReceiver,
    TIME_CHANNEL, WATCHDOG_LIVENESS_CHANNEL,
};
use crate::config::{Bmp390RuntimeConfig, DeviceConfig, Mpu6050RuntimeConfig};
use crate::core1;
use crate::pinmap;
use crate::resources::{DeviceResources, RecoverySensorPins, SystemResources};
use crate::storage;
use crate::usb_cdc;
use crate::watchdog;

type RecoveryI2c = I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>;
type SharedRecoveryI2cBus = Mutex<NoopRawMutex, RecoveryI2c>;
type SharedRecoveryI2cDevice = I2cDevice<'static, NoopRawMutex, RecoveryI2c>;
type RecoveryBmp390Source = Bmp390BarometerSource<SharedRecoveryI2cDevice>;
type RecoveryMpu6050Source = Mpu6050ImuSource<SharedRecoveryI2cDevice>;

static I2C_BUS: StaticCell<SharedRecoveryI2cBus> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

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

fn watchdog_now_ms() -> u64 {
    Instant::now().as_millis()
}

fn report_watchdog_progress(mask: u32) {
    let _ = WATCHDOG_LIVENESS_CHANNEL.try_send(LivenessUpdate::new(mask, watchdog_now_ms()));
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

async fn try_init_bmp390_source(
    shared_bus: &'static SharedRecoveryI2cBus,
    config: Bmp390RuntimeConfig,
) -> Option<RecoveryBmp390Source> {
    let detected_address = {
        let mut probe_bus = I2cDevice::new(shared_bus);
        probe_bmp390_address(&mut probe_bus, config.address).await
    }?;

    let mut source =
        Bmp390BarometerSource::new(Bmp390::new(I2cDevice::new(shared_bus), detected_address));
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

async fn try_init_mpu6050_source(
    shared_bus: &'static SharedRecoveryI2cBus,
    config: Mpu6050RuntimeConfig,
) -> Option<RecoveryMpu6050Source> {
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
    mut time_subscriber: Option<RecoveryTimeSubscriber>,
    sink_state_receiver: RecoveryLogSinkStateReceiver,
) -> ! {
    let mut delay = EmbassyDelay;

    run_core0_sensor_logging_task(
        &mut logger,
        &LOG_CHANNEL,
        imu_subscriber.as_mut(),
        None::<&mut RecoveryImuSubscriber>,
        barometer_subscriber.as_mut(),
        None::<
            &mut embassy_sync::pubsub::Subscriber<
                '_,
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
                common::messages::sensor::PressureTransducerSampleStamped,
                1,
                1,
                1,
            >,
        >,
        None::<
            &mut embassy_sync::pubsub::Subscriber<
                '_,
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
                common::messages::sensor::MagnetometerSampleStamped,
                1,
                1,
                1,
            >,
        >,
        None::<
            &mut embassy_sync::pubsub::Subscriber<
                '_,
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
                common::messages::sensor::GpsFixSampleStamped,
                1,
                1,
                1,
            >,
        >,
        time_subscriber.as_mut(),
        Some(&sink_state_receiver),
        None::<
            &embassy_sync::channel::Receiver<
                '_,
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
                LoggedSensor,
                1,
            >,
        >,
        &mut delay,
        measurement_now,
        |error| {
            if !matches!(
                error,
                SensorSnapshotLoggerError::Queue(TryEnqueueLogError::ChannelFull)
            ) {
                warn!("sensor logging error: {:?}", error);
            }
        },
    )
    .await
}

#[task]
async fn watchdog_task(
    hardware: watchdog::HardwareWatchdog,
    receiver: RecoveryWatchdogLivenessReceiver,
    phase_subscriber: RecoveryFlightPhaseSubscriber,
    watchdog_enabled_in_hil: bool,
) -> ! {
    common::services::health::run_watchdog_supervisor_loop(
        hardware,
        receiver,
        phase_subscriber,
        watchdog::SOURCES,
        watchdog::INIT_CONTRACT,
        watchdog::contract_for_phase,
        watchdog_enabled_in_hil,
        watchdog_now_ms,
    )
    .await
}

#[task]
async fn time_liveness_task(subscriber: RecoveryTimeSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

#[task]
async fn imu_liveness_task(subscriber: RecoveryImuSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

#[task]
async fn barometer_liveness_task(subscriber: RecoveryBarometerSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

#[task]
async fn recovery_i2c_sensor_task(
    mut barometer_source: RecoveryBmp390Source,
    bmp390_config: Bmp390RuntimeConfig,
    mut imu_source: RecoveryMpu6050Source,
    mpu6050_config: Mpu6050RuntimeConfig,
) -> ! {
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;
    let mut next_barometer_action = measurement_now();
    let mut next_imu_action = measurement_now();

    loop {
        let now = measurement_now();
        let mut did_work = false;

        if now >= next_barometer_action {
            did_work = true;

            let timestamp = clock.now();
            match barometer_source.read_barometer_sample().await {
                Ok(sample) => {
                    BAROMETER_CHANNEL
                        .immediate_publisher()
                        .publish_immediate(BarometerSampleStamped { timestamp, sample });
                }
                Err(error) => warn!("BMP390 acquisition error: {:?}", error),
            }

            next_barometer_action = deadline_after_ms(measurement_now(), bmp390_config.period_ms);
        }

        let now = measurement_now();
        if now >= next_imu_action {
            did_work = true;

            let timestamp = clock.now();
            match imu_source.read_imu_sample().await {
                Ok(sample) => {
                    IMU_CHANNEL
                        .immediate_publisher()
                        .publish_immediate(ImuSampleStamped::new(timestamp, sample));
                }
                Err(error) => warn!("MPU6050 acquisition error: {:?}", error),
            }

            next_imu_action = deadline_after_ms(measurement_now(), mpu6050_config.period_ms);
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

#[task]
async fn hil_control_command_task(
    receiver: RecoveryHilControlCommandReceiver,
    egress: RecoveryHilEgressSender,
    runtime: HilControlRuntime,
) -> ! {
    common::services::hil::run_hil_control_command_loop(
        receiver,
        egress,
        &HIL_SESSION_STATE_CHANNEL,
        runtime,
    )
    .await
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
        watchdog: watchdog_resources,
        system:
            SystemResources {
                usb,
                flash: _flash,
                core1: _core1,
            },
    } = resources;
    let config = DeviceConfig::default();

    HIL_BOOT_SIGNAL.reset();
    usb_cdc::reset_host_connected();

    let usb_phase_subscriber = FLIGHT_PHASE_CHANNEL.subscriber().unwrap();
    let watchdog_phase_subscriber = FLIGHT_PHASE_CHANNEL.subscriber().unwrap();
    let phase_publisher = FLIGHT_PHASE_CHANNEL.immediate_publisher();
    let hardware_watchdog = watchdog::HardwareWatchdog::new(
        watchdog_resources.peripheral,
        watchdog_resources.timeout_ms,
    );
    let reset_reason = hardware_watchdog.reset_reason();

    spawner
        .spawn(hil_control_command_task(
            HIL_CONTROL_COMMAND_CHANNEL.receiver(),
            HIL_EGRESS_CHANNEL.sender(),
            HilControlRuntime::new(config.hil.system_id, config.hil.component_id),
        ))
        .unwrap();
    usb_cdc::spawn(
        &spawner,
        usb,
        config.hil,
        HIL_EGRESS_CHANNEL.receiver(),
        usb_phase_subscriber,
        HIL_SESSION_STATE_CHANNEL.subscriber().unwrap(),
        HIL_CONTROL_COMMAND_CHANNEL.sender(),
        WATCHDOG_LIVENESS_CHANNEL.sender(),
    );
    spawner
        .spawn(watchdog_task(
            hardware_watchdog,
            WATCHDOG_LIVENESS_CHANNEL.receiver(),
            watchdog_phase_subscriber,
            config.watchdog_enabled_in_hil,
        ))
        .unwrap();

    phase_publisher.publish_immediate(FlightPhase::Init);
    info!("boot reset reason: {:?}", reset_reason);

    let mut boot_ticker = Ticker::every(Duration::from_millis(100));
    let boot_started_at = Instant::now();
    let mut boot_selector = HilBootSelector::new();
    let mut hil_requested = false;
    loop {
        report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);
        match boot_selector.update(
            HIL_BOOT_SIGNAL.try_take().is_some(),
            usb_cdc::host_connected(),
            boot_started_at.elapsed().as_millis() as u32,
            config.hil_boot_window_ms,
        ) {
            HilBootDecision::Continue => {}
            HilBootDecision::EnterHil => {
                hil_requested = true;
                break;
            }
            HilBootDecision::SelectReal => break,
        }
        boot_ticker.next().await;
    }

    let selected_backend = if hil_requested {
        SensorBackend::Hil
    } else {
        SensorBackend::Real
    };

    let phase = match selected_backend {
        SensorBackend::Hil => {
            phase_publisher.publish_immediate(FlightPhase::Hil);
            spawner
                .spawn(time_liveness_task(
                    TIME_CHANNEL.subscriber().unwrap(),
                    watchdog::SOURCE_HIL_TIME,
                ))
                .unwrap();
            FlightPhase::Hil
        }
        SensorBackend::Real => {
            report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);

            let mut i2c_config = i2c::Config::default();
            i2c_config.frequency = config.bmp390.i2c_frequency_hz;

            let i2c = I2c::new_async(recovery_i2c.i2c, scl, sda, Irqs, i2c_config);
            let shared_bus: &'static SharedRecoveryI2cBus = I2C_BUS.init(Mutex::new(i2c));

            Timer::after(Duration::from_millis(100)).await;
            report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);

            let barometer_source = try_init_bmp390_source(shared_bus, config.bmp390).await;
            report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);
            if barometer_source.is_none() {
                warn!(
                    "BMP390 missing during INIT; Bosch requires CSB high at POR and SDO strapped to GND or VDDIO"
                );
            }

            let imu_source = try_init_mpu6050_source(shared_bus, config.mpu6050).await;
            report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);
            if imu_source.is_none() {
                warn!("MPU6050 missing during INIT; verify AD0 strap and device power");
            }

            let next_phase = phase_after_init(barometer_source.is_some() && imu_source.is_some());
            phase_publisher.publish_immediate(next_phase);

            if matches!(next_phase, FlightPhase::Ready) {
                spawner
                    .spawn(recovery_i2c_sensor_task(
                        barometer_source.unwrap(),
                        config.bmp390,
                        imu_source.unwrap(),
                        config.mpu6050,
                    ))
                    .unwrap();
                spawner
                    .spawn(barometer_liveness_task(
                        BAROMETER_CHANNEL.subscriber().unwrap(),
                        watchdog::SOURCE_BAROMETER,
                    ))
                    .unwrap();
                spawner
                    .spawn(imu_liveness_task(
                        IMU_CHANNEL.subscriber().unwrap(),
                        watchdog::SOURCE_IMU,
                    ))
                    .unwrap();
            } else {
                warn!("INIT failed: feed-critical recovery sensors are unavailable");
            }

            next_phase
        }
        SensorBackend::Replay => FlightPhase::Fault,
    };

    if config.logging.enabled && !phase.is_fault() {
        match storage::build_logger_engine(storage, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => {
                    match SensorSnapshotLogger::new(
                        log_path.as_str(),
                        config.logging.sensor_snapshot,
                    ) {
                        Ok(logger) => {
                            let imu_subscriber = if logger.config().sensors.imu {
                                match selected_backend {
                                    SensorBackend::Hil => Some(IMU_CHANNEL.subscriber().unwrap()),
                                    SensorBackend::Real => Some(IMU_CHANNEL.subscriber().unwrap()),
                                    SensorBackend::Replay => None,
                                }
                            } else {
                                None
                            };
                            let barometer_subscriber = if logger.config().sensors.barometer {
                                match selected_backend {
                                    SensorBackend::Hil => {
                                        Some(BAROMETER_CHANNEL.subscriber().unwrap())
                                    }
                                    SensorBackend::Real => {
                                        Some(BAROMETER_CHANNEL.subscriber().unwrap())
                                    }
                                    SensorBackend::Replay => None,
                                }
                            } else {
                                None
                            };
                            let time_subscriber = if matches!(selected_backend, SensorBackend::Hil)
                            {
                                Some(TIME_CHANNEL.subscriber().unwrap())
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
                                    time_subscriber,
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
    } else if phase.is_fault() {
        info!("logging skipped while waiting for watchdog reset");
    } else {
        info!("sd logging disabled by device config");
    }

    info!("recovery-altimeter-rp2350 resource graph initialized");
    info!("watchdog authority: {}", watchdog::FEED_AUTHORITY);
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
        "sensor_backend={:?} phase={:?} hil_boot_window_ms={} watchdog_timeout_ms={} watchdog_enabled_in_hil={}",
        selected_backend,
        phase,
        config.hil_boot_window_ms,
        config.watchdog_timeout_ms,
        config.watchdog_enabled_in_hil
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
