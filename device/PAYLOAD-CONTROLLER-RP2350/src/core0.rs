use common::drivers::bmp388::{
    BMP388_ADDR_SDO_HIGH, BMP388_ADDR_SDO_LOW, BMP388_CHIP_ID, Bmp388, read_bmp388_chip_id,
};
use common::drivers::pressure_transducer::PressureTransducer;
use common::drivers::servo::{RcServo, ServoError};
use common::interfaces::storage::LoggerEngine;
use common::interfaces::timing::MonotonicClock;
use common::messages::control::StaticLedCommand;
use common::messages::logging::LoggedSensor;
use common::services::acquisition::{
    BarometerServiceConfig, Bmp388BarometerSource, PressureTransducerServiceConfig,
    run_barometer_service, run_pressure_transducer_service,
};
use common::services::hil::HilMissionEvent;
use common::services::hil::SensorBackend;
use common::services::logging::{
    SensorSnapshotLogger, SensorSnapshotLoggerError, TryEnqueueLogError,
};
use common::tasks::background::sd_logging::run_sd_logging_task;
use common::tasks::medium_loop::run_core0_sensor_logging_task;
use common::utilities::time::MeasurementTimestamp;
use common::utilities::units::pressure_altitude_ft;
use common::utils::delay::DelayMs;
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_rp::adc::{self, Adc};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_time::{Duration, Timer};

use crate::channels::{
    BAROMETER_CHANNEL, DisabledGpsSubscriber, DisabledImuSubscriber,
    DisabledMagnetometerSubscriber, HIL_MISSION_EVENT_CHANNEL, LOG_CHANNEL, LOG_SINK_STATE_CHANNEL,
    PRESSURE_TRANSDUCER_CHANNEL, PayloadBarometerSubscriber, PayloadHilMissionEventSender,
    PayloadLogSinkStateReceiver, PayloadPressureTransducerSubscriber,
    PayloadStatusLedCommandReceiver, PayloadStatusLedCommandSender, PayloadTimeSubscriber,
    STATUS_LED_COMMAND_CHANNEL, TIME_CHANNEL,
};
use crate::config::DeviceConfig;
use crate::core1;
use crate::pinmap;
use crate::pressure_transducer::RpAdcPressureTransducerSource;
use crate::resources::{DeviceResources, PayloadSensorPins, SystemResources};
use crate::storage;
use crate::usb_cdc;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

type PayloadI2c = I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>;

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

fn pwm_top_for_period(sys_freq_hz: u32, divider: u8, frame_period_us: u32) -> Option<u16> {
    if divider == 0 || frame_period_us == 0 {
        return None;
    }

    let numerator = sys_freq_hz as u64 * frame_period_us as u64;
    let denominator = divider as u64 * 1_000_000;
    let period_counts = (numerator + (denominator / 2)) / denominator;

    if period_counts == 0 || period_counts > u16::MAX as u64 + 1 {
        return None;
    }

    Some((period_counts - 1) as u16)
}

fn set_servo_angle(
    pwm: &mut Pwm<'static>,
    servo: &RcServo,
    angle_deg: u16,
    pwm_top: u16,
) -> Result<u16, ServoError> {
    let duty = servo.duty_for_angle_deg(angle_deg, pwm_top)?;
    pwm.set_duty_cycle(duty)
        .map_err(|_| ServoError::InvalidPwmTop)?;
    Ok(duty)
}

async fn probe_bmp388_address(i2c: &mut PayloadI2c, preferred_address: u8) -> Option<u8> {
    let candidates = if preferred_address == BMP388_ADDR_SDO_HIGH {
        [BMP388_ADDR_SDO_HIGH, BMP388_ADDR_SDO_LOW]
    } else {
        [BMP388_ADDR_SDO_LOW, BMP388_ADDR_SDO_HIGH]
    };

    for address in candidates {
        match read_bmp388_chip_id(i2c, address).await {
            Ok(BMP388_CHIP_ID) => {
                info!("BMP388 detected at I2C address 0x{=u8:02X}", address);
                return Some(address);
            }
            Ok(chip_id) => warn!(
                "BMP388 probe at 0x{=u8:02X} returned chip id 0x{=u8:02X}",
                address, chip_id
            ),
            Err(error) => warn!("BMP388 probe at 0x{=u8:02X} failed: {:?}", address, error),
        }
    }

    None
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
    mut barometer_subscriber: Option<PayloadBarometerSubscriber>,
    mut pressure_transducer_subscriber: Option<PayloadPressureTransducerSubscriber>,
    mut time_subscriber: Option<PayloadTimeSubscriber>,
    sink_state_receiver: PayloadLogSinkStateReceiver,
) -> ! {
    let mut delay = EmbassyDelay;

    run_core0_sensor_logging_task(
        &mut logger,
        &LOG_CHANNEL,
        None::<&mut DisabledImuSubscriber>,
        None::<&mut DisabledImuSubscriber>,
        barometer_subscriber.as_mut(),
        pressure_transducer_subscriber.as_mut(),
        None::<&mut DisabledMagnetometerSubscriber>,
        None::<&mut DisabledGpsSubscriber>,
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
async fn bmp388_barometer_task(
    sda: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_0>,
    scl: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_1>,
    bus: crate::buses::PayloadI2cBus,
    config: crate::config::Bmp388RuntimeConfig,
) -> ! {
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = config.i2c_frequency_hz;

    let mut i2c = I2c::new_async(bus.i2c, scl, sda, Irqs, i2c_config);
    Timer::after(Duration::from_millis(100)).await;
    let detected_address = loop {
        if let Some(address) = probe_bmp388_address(&mut i2c, config.address).await {
            break address;
        }

        warn!(
            "BMP388 not detected on 0x76/0x77; Bosch requires CSB high at POR for I2C and SDO strapped to GND or VDDIO"
        );
        Timer::after(Duration::from_secs(1)).await;
    };

    let driver = Bmp388::new(i2c, detected_address);
    let mut source = Bmp388BarometerSource::new(driver);
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    loop {
        match source.driver_mut().init(&mut delay).await {
            Ok(()) => {
                info!("BMP388 initialized");
                break;
            }
            Err(error) => {
                warn!("BMP388 initialization failed: {:?}", error);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    run_barometer_service(
        &BAROMETER_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        BarometerServiceConfig::new(true, config.period_ms),
        |error| warn!("BMP388 acquisition error: {:?}", error),
    )
    .await
}

#[task]
async fn pressure_transducer_task(
    pressure_adc_pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_29>,
    bus: crate::buses::PressureAdcBus,
    config: crate::config::PressureTransducerRuntimeConfig,
) -> ! {
    let driver = match PressureTransducer::try_new(config.sensor) {
        Ok(driver) => driver,
        Err(error) => {
            warn!("pressure transducer config invalid: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let adc = Adc::new(bus.adc, Irqs, adc::Config::default());
    let mut source = RpAdcPressureTransducerSource::new(adc, pressure_adc_pin, driver);
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;

    run_pressure_transducer_service(
        &PRESSURE_TRANSDUCER_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        PressureTransducerServiceConfig::new(config.enabled, config.period_ms),
        |error| warn!("pressure transducer acquisition error: {:?}", error),
    )
    .await
}

#[task]
async fn status_led_task(
    status_led: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_25>,
    receiver: PayloadStatusLedCommandReceiver,
) -> ! {
    let mut led = Output::new(status_led, Level::Low);
    led.set_low();

    loop {
        let command = receiver.receive().await;
        if command.on {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}

#[task]
async fn servo_deploy_task(
    servo_pwm_pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_28>,
    bus: crate::buses::ServoPwm,
    mut barometer_subscriber: PayloadBarometerSubscriber,
    config: crate::config::ServoRuntimeConfig,
    led_sender: PayloadStatusLedCommandSender,
    mission_event_sender: Option<PayloadHilMissionEventSender>,
    mission_event_command_id: u16,
) -> ! {
    if !config.enabled {
        info!("servo deployment disabled by device config");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    let servo = RcServo::ds3240_180();
    let pwm_top = match pwm_top_for_period(
        clocks::clk_sys_freq(),
        config.pwm_divider,
        config.frame_period_us,
    ) {
        Some(top) => top,
        None => {
            warn!(
                "servo PWM timing invalid: divider={=u8} frame_period_us={=u32}",
                config.pwm_divider, config.frame_period_us
            );
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let initial_duty = match servo.duty_for_angle_deg(config.closed_angle_deg, pwm_top) {
        Ok(duty) => duty,
        Err(error) => {
            warn!("servo initial position invalid: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut pwm_config = PwmConfig::default();
    pwm_config.divider = config.pwm_divider.into();
    pwm_config.top = pwm_top;
    pwm_config.compare_a = initial_duty;
    let mut pwm = Pwm::new_output_a(bus.pwm, servo_pwm_pin, pwm_config);

    info!(
        "servo PWM initialized on GP{=u8}: frame_period_us={=u32} divider={=u8} top={=u16}",
        pinmap::SERVO_PWM,
        config.frame_period_us,
        config.pwm_divider,
        pwm_top,
    );
    info!(
        "servo deployment armed: closed={=u16}deg open={=u16}deg trigger_altitude_ft={=u32}",
        config.closed_angle_deg, config.open_angle_deg, config.trigger_altitude_ft,
    );

    let mut deployed = false;
    loop {
        let sample = barometer_subscriber.next_message_pure().await;

        if deployed {
            continue;
        }

        let altitude_ft = match pressure_altitude_ft(
            sample.sample.pressure_pa,
            config.sea_level_pressure_pa as f32,
        ) {
            Some(altitude_ft) => altitude_ft,
            None => continue,
        };

        if altitude_ft < config.trigger_altitude_ft as f32 {
            continue;
        }

        match set_servo_angle(&mut pwm, &servo, config.open_angle_deg, pwm_top) {
            Ok(duty) => {
                deployed = true;
                let _ = led_sender.try_send(StaticLedCommand::ON);
                if let Some(sender) = mission_event_sender {
                    let _ = sender.try_send(HilMissionEvent {
                        timestamp: sample.timestamp,
                        command_id: mission_event_command_id,
                        params: [
                            config.trigger_altitude_ft as f32,
                            altitude_ft,
                            config.open_angle_deg as f32,
                            duty as f32,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    });
                }
                info!(
                    "servo deployed: altitude_ft={=u32} threshold_ft={=u32} angle_deg={=u16} duty={=u16}",
                    altitude_ft as u32, config.trigger_altitude_ft, config.open_angle_deg, duty,
                );
            }
            Err(error) => warn!(
                "servo deploy rejected: angle_deg={=u16} error={:?}",
                config.open_angle_deg, error
            ),
        }
    }
}

pub async fn run(spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins:
            crate::resources::PinResources {
                sensors:
                    PayloadSensorPins {
                        sda,
                        scl,
                        pressure_adc,
                    },
                storage,
                status,
                actuators,
            },
        buses:
            crate::buses::BusResources {
                payload_i2c,
                pressure_adc: pressure_adc_bus,
                storage: storage_bus,
                servo_pwm,
            },
        system:
            SystemResources {
                usb,
                flash: _flash,
                core1: _core1,
            },
    } = resources;
    let config = DeviceConfig::default();

    spawner
        .spawn(status_led_task(
            status.led,
            STATUS_LED_COMMAND_CHANNEL.receiver(),
        ))
        .unwrap();
    let _ = STATUS_LED_COMMAND_CHANNEL
        .sender()
        .try_send(StaticLedCommand::OFF);
    match config.sensor_backend {
        SensorBackend::Real => {
            spawner
                .spawn(bmp388_barometer_task(sda, scl, payload_i2c, config.bmp388))
                .unwrap();
            spawner
                .spawn(pressure_transducer_task(
                    pressure_adc,
                    pressure_adc_bus,
                    config.pressure_transducer,
                ))
                .unwrap();
        }
        SensorBackend::Hil => {
            usb_cdc::spawn(
                &spawner,
                usb,
                config.hil,
                HIL_MISSION_EVENT_CHANNEL.receiver(),
            );
        }
        SensorBackend::Replay => {}
    }
    if config.servo.enabled {
        let barometer_subscriber = BAROMETER_CHANNEL.subscriber().unwrap();
        spawner
            .spawn(servo_deploy_task(
                actuators.servo_pwm,
                servo_pwm,
                barometer_subscriber,
                config.servo,
                STATUS_LED_COMMAND_CHANNEL.sender(),
                matches!(config.sensor_backend, SensorBackend::Hil)
                    .then_some(HIL_MISSION_EVENT_CHANNEL.sender()),
                config.hil.payload_servo_test_command_id,
            ))
            .unwrap();
    } else {
        info!("servo deployment disabled by device config");
    }

    if config.logging.enabled {
        match storage::build_logger_engine(storage, storage_bus, config.logging) {
            Ok(mut engine) => match engine.create_new_csv(config.logging.file_prefix) {
                Ok(log_path) => {
                    match SensorSnapshotLogger::new(
                        log_path.as_str(),
                        config.logging.sensor_snapshot,
                    ) {
                        Ok(mut logger) => {
                            if !matches!(config.sensor_backend, SensorBackend::Real)
                                && logger.config().sensors.pressure_transducer
                            {
                                logger.note_sensor_fault(LoggedSensor::PressureTransducer);
                            }
                            if matches!(config.sensor_backend, SensorBackend::Replay)
                                && logger.config().sensors.barometer
                            {
                                logger.note_sensor_fault(LoggedSensor::Barometer);
                            }

                            let barometer_subscriber = if logger.config().sensors.barometer {
                                match config.sensor_backend {
                                    SensorBackend::Real | SensorBackend::Hil => {
                                        Some(BAROMETER_CHANNEL.subscriber().unwrap())
                                    }
                                    SensorBackend::Replay => None,
                                }
                            } else {
                                None
                            };
                            let pressure_transducer_subscriber =
                                if logger.config().sensors.pressure_transducer
                                    && matches!(config.sensor_backend, SensorBackend::Real)
                                {
                                    Some(PRESSURE_TRANSDUCER_CHANNEL.subscriber().unwrap())
                                } else {
                                    None
                                };
                            let time_subscriber =
                                if matches!(config.sensor_backend, SensorBackend::Hil) {
                                    Some(TIME_CHANNEL.subscriber().unwrap())
                                } else {
                                    None
                                };
                            let sink_state_receiver = LOG_SINK_STATE_CHANNEL.receiver();

                            spawner.spawn(sd_logging_task(engine)).unwrap();
                            spawner
                                .spawn(sensor_logging_task(
                                    logger,
                                    barometer_subscriber,
                                    pressure_transducer_subscriber,
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
    } else {
        info!("sd logging disabled by device config");
    }

    info!("payload-controller-rp2350 resource graph initialized");
    info!(
        "payload controller I2C0: BMP388 on GP{=u8}/GP{=u8}",
        pinmap::PAYLOAD_I2C_SDA,
        pinmap::PAYLOAD_I2C_SCL,
    );
    info!(
        "BMP388 runtime: addr=0x{=u8:02X} sample_period_ms={=u32}",
        config.bmp388.address, config.bmp388.period_ms,
    );
    info!(
        "pressure transducer ADC: GP{=u8} sample_period_ms={=u32}",
        pinmap::PRESSURE_TRANSDUCER_ADC,
        config.pressure_transducer.period_ms,
    );
    info!(
        "payload controller SPI0: SD card on GP{=u8}/GP{=u8}/GP{=u8} cs=GP{=u8}",
        pinmap::STORAGE_SPI_SCK,
        pinmap::STORAGE_SPI_MOSI,
        pinmap::STORAGE_SPI_MISO,
        pinmap::STORAGE_SPI_CS,
    );
    info!(
        "status LED: GP{=u8}, servo output: GP{=u8}, core1: {}",
        pinmap::STATUS_LED,
        pinmap::SERVO_PWM,
        core1::ROLE_SUMMARY,
    );
    info!("sensor_backend={:?}", config.sensor_backend);
    info!(
        "servo runtime: closed={=u16}deg open={=u16}deg trigger_altitude_ft={=u32}",
        config.servo.closed_angle_deg,
        config.servo.open_angle_deg,
        config.servo.trigger_altitude_ft,
    );

    loop {
        Timer::after(Duration::from_millis(config.status_heartbeat_period_ms)).await;
        info!("payload-controller-rp2350 heartbeat");
    }
}
