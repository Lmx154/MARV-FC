use core::cell::RefCell;

use common::drivers::bmi088::{
    AccelRange as Bmi088AccelRange, Bmi088, GyroRange as Bmi088GyroRange,
};
use common::drivers::bmp581::{
    BMP581_ADDR_ALT, BMP581_CHIP_ID_ALT, BMP581_CHIP_ID_PRIMARY, Bmp581, read_bmp581_chip_id,
};
use common::drivers::lsm6dsv32x::{
    AccelRange as Lsm6dsv32xAccelRange, GyroRange as Lsm6dsv32xGyroRange, Lsm6dsv32x,
};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::logging::LoggedSensor;
use common::messages::runtime::FlightPhase;
use common::policies::mission::BarometerRgbLedMission;
use common::policies::modes::phase_after_init;
use common::services::acquisition::{
    BarometerServiceConfig, Bmp581BarometerSource, run_barometer_service,
};
use common::services::acquisition::{Bmi088ImuSource, Lsm6dsv32xImuSource};
use common::services::health::LivenessUpdate;
use common::services::hil::{SensorBackend, runtime_phase_event};
use common::services::logging::SensorSnapshotLogger;
use common::tasks::background::mission::run_barometer_rgb_led_mission_task;
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::medium_loop::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::{InterruptExecutor, SendSpawner, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::delay::DelayNs;

use crate::buses::SensorSpiBus;
use crate::channels::{
    self, AUX_IMU_CHANNEL, BAROMETER_CHANNEL, DisabledMagnetometerSubscriber,
    DisabledPressureTransducerSubscriber, FLIGHT_PHASE_CHANNEL, FcBarometerSubscriber,
    FcFlightPhaseSubscriber, FcGpsSubscriber, FcImuSubscriber, FcLogSinkStateReceiver,
    FcRgbLedCommandSender, FcSensorFaultReceiver, FcTimeSubscriber, FcWatchdogLivenessReceiver,
    GPS_CHANNEL, HIL_EGRESS_CHANNEL, IMU_CHANNEL, IMU_INIT_SIGNAL, LOG_CHANNEL,
    LOG_SINK_STATE_CHANNEL, RGB_LED_COMMAND_CHANNEL, SENSOR_FAULT_CHANNEL, TIME_CHANNEL,
    WATCHDOG_LIVENESS_CHANNEL,
};
use crate::config::{DeviceConfig, STATUS_HEARTBEAT_PERIOD_MS};
use crate::core1;
use crate::pinmap;
use crate::radio_link;
use crate::resources::{DeviceResources, SensorPins};
use crate::sensor_spi::{SharedSensorSpiBus, SharedSpiDevice};
use crate::spi1_sensor_cluster::{
    ImuSchedule, SensorSpiClusterConfig, SensorSpiClusterError, run_spi1_sensor_cluster,
};
use crate::status_led;
use crate::storage;
use crate::usb_cdc;
use crate::watchdog;

const CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY: interrupt::Priority = interrupt::Priority::P2;
const SENSOR_SPI_FREQUENCY_HZ: u32 = 1_000_000;

static CORE0_TIME_SENSITIVE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

bind_interrupts!(struct EnvironmentalI2cIrqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

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

impl DelayNs for EmbassyDelay {
    async fn delay_ns(&mut self, ns: u32) {
        Timer::after_nanos(ns as u64).await;
    }
}

fn imu_period_ms(fast_loop_hz: u32) -> u32 {
    let hz = fast_loop_hz.max(1);
    1_000u32.div_ceil(hz).max(1)
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

fn try_report_sensor_fault(sensor: LoggedSensor) {
    let _ = SENSOR_FAULT_CHANNEL.try_send(sensor);
}

#[cfg(feature = "hil-sensor-backend")]
const SELECTED_SENSOR_BACKEND: SensorBackend = SensorBackend::Hil;
#[cfg(not(feature = "hil-sensor-backend"))]
const SELECTED_SENSOR_BACKEND: SensorBackend = SensorBackend::Real;

fn emit_hil_runtime_phase_event(phase: FlightPhase) {
    if !matches!(SELECTED_SENSOR_BACKEND, SensorBackend::Hil) {
        return;
    }

    let _ = HIL_EGRESS_CHANNEL
        .sender()
        .try_send(runtime_phase_event(measurement_now(), phase));
}

fn announce_selected_sensor_backend() {
    if matches!(SELECTED_SENSOR_BACKEND, SensorBackend::Hil) {
        info!("compile-time HIL sensor backend enabled");
    } else {
        info!("compile-time real sensor backend enabled");
    }
}

#[embassy_executor::task]
async fn watchdog_task(
    hardware: watchdog::HardwareWatchdog,
    receiver: FcWatchdogLivenessReceiver,
    phase_subscriber: FcFlightPhaseSubscriber,
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

#[embassy_executor::task]
async fn time_liveness_task(subscriber: FcTimeSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

#[embassy_executor::task(pool_size = 2)]
async fn imu_liveness_task(subscriber: FcImuSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

#[embassy_executor::task]
async fn barometer_liveness_task(subscriber: FcBarometerSubscriber, mask: u32) -> ! {
    common::services::health::run_watchdog_liveness_loop(subscriber, mask, report_watchdog_progress)
        .await
}

async fn probe_bmp581_address(
    i2c: &mut I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>,
    preferred_address: u8,
) -> Option<u8> {
    let alternate_address = if preferred_address == BMP581_ADDR_ALT {
        common::drivers::bmp581::BMP581_ADDR_PRIMARY
    } else {
        BMP581_ADDR_ALT
    };

    for address in [preferred_address, alternate_address] {
        match read_bmp581_chip_id(i2c, address).await {
            Ok(BMP581_CHIP_ID_PRIMARY) | Ok(BMP581_CHIP_ID_ALT) => {
                info!("BMP581 detected at I2C address 0x{=u8:02X}", address);
                return Some(address);
            }
            Ok(chip_id) => warn!(
                "BMP581 probe at 0x{=u8:02X} returned chip id 0x{=u8:02X}",
                address, chip_id
            ),
            Err(_) => warn!("BMP581 probe at 0x{=u8:02X} failed", address),
        }
    }

    None
}

#[embassy_executor::task]
async fn spi1_sensor_cluster_task(
    pins: SensorPins,
    bus: SensorSpiBus,
    config: SensorSpiClusterConfig,
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
        warn!("BMI088 will remain unpublished until SPI1 sensor-cluster retry logic is added");
    }
    if !lsm6dsv32x_ready {
        warn!("LSM6DSV32X will remain unpublished until SPI1 sensor-cluster retry logic is added");
    }

    run_spi1_sensor_cluster(
        &IMU_CHANNEL,
        &mut bmi088_source,
        &AUX_IMU_CHANNEL,
        &mut lsm6dsv32x_source,
        &clock,
        &mut delay,
        SensorSpiClusterConfig {
            primary_imu: ImuSchedule::new(bmi088_ready, config.primary_imu.period_ms),
            auxiliary_imu: ImuSchedule::new(lsm6dsv32x_ready, config.auxiliary_imu.period_ms),
        },
        |error| match error {
            SensorSpiClusterError::PrimaryImu(error) => {
                warn!("BMI088 acquisition error: {:?}", error);
                try_report_sensor_fault(LoggedSensor::Imu);
            }
            SensorSpiClusterError::AuxiliaryImu(error) => {
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
    mut barometer_subscriber: Option<FcBarometerSubscriber>,
    mut gps_subscriber: Option<FcGpsSubscriber>,
    mut time_subscriber: Option<FcTimeSubscriber>,
    sink_state_receiver: Option<FcLogSinkStateReceiver>,
    sensor_fault_receiver: Option<FcSensorFaultReceiver>,
) -> ! {
    let mut delay = EmbassyDelay;

    run_core0_sensor_logging_task(
        &mut logger,
        &LOG_CHANNEL,
        imu_subscriber.as_mut(),
        aux_imu_subscriber.as_mut(),
        barometer_subscriber.as_mut(),
        None::<&mut DisabledPressureTransducerSubscriber>,
        None::<&mut DisabledMagnetometerSubscriber>,
        gps_subscriber.as_mut(),
        time_subscriber.as_mut(),
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

#[embassy_executor::task]
async fn bmp581_barometer_task(
    i2c: I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>,
    config: crate::config::Bmp581RuntimeConfig,
    detected_address: u8,
) -> ! {
    let mut source = Bmp581BarometerSource::new(Bmp581::new(
        i2c,
        EmbassyDelay,
        detected_address,
        config.driver_config,
    ));
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    loop {
        match source.driver_mut().init().await {
            Ok(()) => {
                info!("BMP581 initialized");
                break;
            }
            Err(error) => {
                warn!("BMP581 initialization failed: {:?}", error);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    run_barometer_service(
        &BAROMETER_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        BarometerServiceConfig::new(config.enabled, config.period_ms),
        |error| {
            warn!("BMP581 acquisition error: {:?}", error);
            try_report_sensor_fault(LoggedSensor::Barometer);
        },
    )
    .await
}

#[embassy_executor::task]
async fn altitude_led_mission_task(
    mut barometer_subscriber: FcBarometerSubscriber,
    command_sender: FcRgbLedCommandSender,
    config: common::policies::mission::BarometerRgbLedMissionConfig,
) -> ! {
    let mut mission = BarometerRgbLedMission::new(config);
    run_barometer_rgb_led_mission_task(&mut barometer_subscriber, command_sender, &mut mission)
        .await
}

fn start_core0_time_sensitive_executor() -> SendSpawner {
    interrupt::SWI_IRQ_0.set_priority(CORE0_TIME_SENSITIVE_EXECUTOR_PRIORITY);
    CORE0_TIME_SENSITIVE_EXECUTOR.start(interrupt::SWI_IRQ_0)
}

pub async fn run(spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins,
        buses,
        watchdog: watchdog_resources,
        system:
            crate::resources::SystemResources {
                usb,
                flash: _flash,
                core1: _core1,
            },
    } = resources;
    let config = DeviceConfig::default();
    let time_sensitive_spawner = start_core0_time_sensitive_executor();
    let sensor_pins = pins.sensors;
    let storage_pins = pins.storage;
    let radio_link_pins = pins.radio_link;
    let status_pins = pins.status;
    let sensor_bus = buses.sensors;
    let storage_bus = buses.storage;
    let radio_link_bus = buses.radio_link;
    let environmental_bus = buses.environmental;
    let status_led_bus = buses.status_led;

    status_led::spawn(&spawner, status_pins.data, status_led_bus);
    radio_link::spawn(&spawner, radio_link_bus, radio_link_pins);
    IMU_INIT_SIGNAL.reset();

    let watchdog_phase_subscriber = FLIGHT_PHASE_CHANNEL.subscriber().unwrap();
    let phase_publisher = FLIGHT_PHASE_CHANNEL.immediate_publisher();
    let hardware_watchdog = watchdog::HardwareWatchdog::new(
        watchdog_resources.peripheral,
        watchdog_resources.timeout_ms,
    );
    let reset_reason = hardware_watchdog.reset_reason();

    usb_cdc::spawn(
        &spawner,
        usb,
        config.hil,
        HIL_EGRESS_CHANNEL.receiver(),
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
    emit_hil_runtime_phase_event(FlightPhase::Init);
    info!("boot reset reason: {:?}", reset_reason);
    announce_selected_sensor_backend();
    let selected_backend = SELECTED_SENSOR_BACKEND;

    let mut imu_init = channels::ImuInitReport {
        imu_ready: false,
        aux_imu_ready: false,
    };
    let mut barometer_ready = false;
    let phase = match selected_backend {
        SensorBackend::Hil => {
            phase_publisher.publish_immediate(FlightPhase::Hil);
            emit_hil_runtime_phase_event(FlightPhase::Hil);
            spawner
                .spawn(time_liveness_task(
                    TIME_CHANNEL.subscriber().unwrap(),
                    watchdog::SOURCE_HIL_TIME,
                ))
                .unwrap();
            FlightPhase::Hil
        }
        SensorBackend::Real => {
            let mut bmp581_i2c = None;
            let mut bmp581_detected_address = None;

            if config.bmp581.enabled {
                let mut i2c_config = i2c::Config::default();
                i2c_config.frequency = config.bmp581.i2c_frequency_hz;

                let mut environmental_i2c = I2c::new_async(
                    environmental_bus.i2c,
                    pins.environmental.scl,
                    pins.environmental.sda,
                    EnvironmentalI2cIrqs,
                    i2c_config,
                );
                loop {
                    bmp581_detected_address =
                        probe_bmp581_address(&mut environmental_i2c, config.bmp581.address).await;
                    if bmp581_detected_address.is_some() {
                        break;
                    }

                    warn!("BMP581 missing during INIT, retrying");
                    report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);
                    Timer::after(Duration::from_secs(1)).await;
                }
                bmp581_i2c = Some(environmental_i2c);
            }

            let imu_period_ms = imu_period_ms(config.fast_loop_hz);
            let imu_config = SensorSpiClusterConfig {
                primary_imu: ImuSchedule::new(true, imu_period_ms),
                auxiliary_imu: ImuSchedule::new(true, imu_period_ms),
            };
            time_sensitive_spawner
                .spawn(spi1_sensor_cluster_task(
                    sensor_pins,
                    sensor_bus,
                    imu_config,
                ))
                .unwrap();

            let mut imu_wait_ticker = Ticker::every(Duration::from_millis(50));
            loop {
                report_watchdog_progress(watchdog::SOURCE_BOOT_COORDINATOR);
                if let Some(report) = IMU_INIT_SIGNAL.try_take() {
                    imu_init = report;
                    break;
                }
                imu_wait_ticker.next().await;
            }

            let feed_critical_ready =
                imu_init.imu_ready && (!config.bmp581.enabled || bmp581_detected_address.is_some());
            let next_phase = phase_after_init(feed_critical_ready);
            phase_publisher.publish_immediate(next_phase);
            emit_hil_runtime_phase_event(next_phase);

            if let (Some(environmental_i2c), Some(detected_address)) =
                (bmp581_i2c, bmp581_detected_address)
            {
                barometer_ready = true;
                spawner
                    .spawn(bmp581_barometer_task(
                        environmental_i2c,
                        config.bmp581,
                        detected_address,
                    ))
                    .unwrap();
                spawner
                    .spawn(barometer_liveness_task(
                        BAROMETER_CHANNEL.subscriber().unwrap(),
                        watchdog::SOURCE_BAROMETER,
                    ))
                    .unwrap();
            }
            if imu_init.imu_ready {
                spawner
                    .spawn(imu_liveness_task(
                        IMU_CHANNEL.subscriber().unwrap(),
                        watchdog::SOURCE_PRIMARY_IMU,
                    ))
                    .unwrap();
            }
            if imu_init.aux_imu_ready {
                spawner
                    .spawn(imu_liveness_task(
                        AUX_IMU_CHANNEL.subscriber().unwrap(),
                        watchdog::SOURCE_AUX_IMU,
                    ))
                    .unwrap();
            }

            if !matches!(next_phase, FlightPhase::Ready) {
                warn!("INIT failed: feed-critical FC sensors are unavailable");
            }

            next_phase
        }
        SensorBackend::Replay => FlightPhase::Fault,
    };

    if matches!(phase, FlightPhase::Ready)
        && matches!(selected_backend, SensorBackend::Real)
        && config.mission.altitude_led_latch.enabled
    {
        spawner
            .spawn(altitude_led_mission_task(
                BAROMETER_CHANNEL.subscriber().unwrap(),
                RGB_LED_COMMAND_CHANNEL.sender(),
                config.mission.altitude_led_latch.mission,
            ))
            .unwrap();
        info!(
            "mission armed: RGB LED latches on at {} ft pressure altitude",
            config
                .mission
                .altitude_led_latch
                .mission
                .trigger_altitude_ft as u32
        );
    }

    if config.logging.enabled {
        match storage::build_logger_engine(storage_pins, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => match SensorSnapshotLogger::new(
                    log_path.as_str(),
                    config.logging.sensor_snapshot,
                ) {
                    Ok(mut logger) => {
                        if matches!(selected_backend, SensorBackend::Real) {
                            if !imu_init.imu_ready {
                                logger.note_sensor_fault(LoggedSensor::Imu);
                            }
                            if !imu_init.aux_imu_ready {
                                logger.note_sensor_fault(LoggedSensor::AuxImu);
                            }
                            if logger.config().sensors.barometer && !barometer_ready {
                                logger.note_sensor_fault(LoggedSensor::Barometer);
                            }
                        }

                        let imu_subscriber = if logger.config().sensors.imu {
                            match selected_backend {
                                SensorBackend::Hil => Some(IMU_CHANNEL.subscriber().unwrap()),
                                SensorBackend::Real if imu_init.imu_ready => {
                                    Some(IMU_CHANNEL.subscriber().unwrap())
                                }
                                _ => None,
                            }
                        } else {
                            None
                        };
                        let aux_imu_subscriber = if logger.config().sensors.aux_imu
                            && matches!(selected_backend, SensorBackend::Real)
                            && imu_init.aux_imu_ready
                        {
                            Some(AUX_IMU_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };
                        let barometer_subscriber = if logger.config().sensors.barometer {
                            match selected_backend {
                                SensorBackend::Hil => Some(BAROMETER_CHANNEL.subscriber().unwrap()),
                                SensorBackend::Real if barometer_ready => {
                                    Some(BAROMETER_CHANNEL.subscriber().unwrap())
                                }
                                _ => None,
                            }
                        } else {
                            None
                        };
                        let gps_subscriber = if logger.config().sensors.gps
                            && matches!(selected_backend, SensorBackend::Hil)
                        {
                            Some(GPS_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };
                        let time_subscriber = if matches!(selected_backend, SensorBackend::Hil) {
                            Some(TIME_CHANNEL.subscriber().unwrap())
                        } else {
                            None
                        };
                        let sink_state_receiver = Some(LOG_SINK_STATE_CHANNEL.receiver());
                        let sensor_fault_receiver = matches!(selected_backend, SensorBackend::Real)
                            .then_some(SENSOR_FAULT_CHANNEL.receiver());

                        spawner.spawn(sd_logging_task(engine)).unwrap();
                        time_sensitive_spawner
                            .spawn(sensor_logging_task(
                                logger,
                                imu_subscriber,
                                aux_imu_subscriber,
                                barometer_subscriber,
                                gps_subscriber,
                                time_subscriber,
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

    info!("MARV-FC-SP-RP2354B resource graph initialized");
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
        "sensor_backend={:?} phase={:?} fast_loop_hz={} watchdog_timeout_ms={} watchdog_enabled_in_hil={} compile_time_hil_sensor_backend={}",
        selected_backend,
        phase,
        config.fast_loop_hz,
        config.watchdog_timeout_ms,
        config.watchdog_enabled_in_hil,
        matches!(SELECTED_SENSOR_BACKEND, SensorBackend::Hil)
    );

    loop {
        Timer::after(Duration::from_millis(STATUS_HEARTBEAT_PERIOD_MS)).await;
        info!("core0 heartbeat");
    }
}
