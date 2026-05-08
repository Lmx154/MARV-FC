use core::cell::RefCell;

use common::drivers::bmi088::{
    AccelRange as Bmi088AccelRange, Bmi088, GyroRange as Bmi088GyroRange,
};
use common::drivers::bmm350::{BMM350_ADDR, Bmm350};
use common::drivers::bmp581::{
    BMP581_ADDR_ALT, BMP581_ADDR_PRIMARY, BMP581_CHIP_ID_ALT, BMP581_CHIP_ID_PRIMARY, Bmp581,
    Bmp581Config, read_bmp581_chip_id,
};
use common::drivers::lsm6dsv32x::{
    AccelRange as Lsm6dsv32xAccelRange, GyroRange as Lsm6dsv32xGyroRange, Lsm6dsv32x,
};
use common::interfaces::timing::MonotonicClock;
use common::services::acquisition::{
    BarometerServiceConfig, Bmi088ImuSource, Bmm350MagnetometerSource, Bmp581BarometerSource,
    Lsm6dsv32xImuSource, MagnetometerServiceConfig, run_barometer_service,
    run_magnetometer_service,
};
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Duration, Timer};
use embedded_hal_async::delay::DelayNs;

use crate::buses::{AuxiliaryNavigationI2cBus, EnvironmentalI2cBus, SensorSpiBus};
use crate::channels::{
    AUX_IMU_CHANNEL, BAROMETER_CHANNEL, IMU_CHANNEL, IMU_INIT_SIGNAL, MAGNETOMETER_CHANNEL,
};
use crate::resources::{AuxiliaryNavigationPins, EnvironmentalPins, SensorPins};
use crate::sensor_spi::{SharedSensorSpiBus, SharedSpiDevice};
use crate::spi1_sensor_cluster::{
    ImuSchedule, SensorSpiClusterConfig, SensorSpiClusterError, run_spi1_sensor_cluster,
};

const SENSOR_SPI_FREQUENCY_HZ: u32 = 1_000_000;
const SENSOR_IMU_PERIOD_MS: u32 = 1;
const BMP581_I2C_FREQUENCY_HZ: u32 = 400_000;
const BMP581_PERIOD_MS: u32 = 20;
const BMM350_I2C_FREQUENCY_HZ: u32 = 400_000;
const BMM350_PERIOD_MS: u32 = 40;

bind_interrupts!(struct EnvironmentalI2cIrqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
});

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

pub fn spawn(
    spawner: &Spawner,
    sensor_bus: SensorSpiBus,
    sensor_pins: SensorPins,
    environmental_bus: EnvironmentalI2cBus,
    environmental_pins: EnvironmentalPins,
    auxiliary_navigation_bus: AuxiliaryNavigationI2cBus,
    auxiliary_navigation_pins: AuxiliaryNavigationPins,
) {
    IMU_INIT_SIGNAL.reset();
    spawner
        .spawn(spi1_sensor_cluster_task(sensor_pins, sensor_bus))
        .unwrap();
    spawner
        .spawn(bmp581_barometer_task(environmental_bus, environmental_pins))
        .unwrap();
    spawner
        .spawn(bmm350_magnetometer_task(
            auxiliary_navigation_bus,
            auxiliary_navigation_pins,
        ))
        .unwrap();
}

async fn probe_bmp581_address(
    i2c: &mut I2c<'static, embassy_rp::peripherals::I2C0, i2c::Async>,
) -> Option<u8> {
    for address in [BMP581_ADDR_PRIMARY, BMP581_ADDR_ALT] {
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
async fn spi1_sensor_cluster_task(pins: SensorPins, bus: SensorSpiBus) -> ! {
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

    IMU_INIT_SIGNAL.signal(crate::channels::ImuInitReport {
        imu_ready: bmi088_ready,
        aux_imu_ready: lsm6dsv32x_ready,
    });

    run_spi1_sensor_cluster(
        &IMU_CHANNEL,
        &mut bmi088_source,
        &AUX_IMU_CHANNEL,
        &mut lsm6dsv32x_source,
        &clock,
        &mut delay,
        SensorSpiClusterConfig {
            primary_imu: ImuSchedule::new(bmi088_ready, SENSOR_IMU_PERIOD_MS),
            auxiliary_imu: ImuSchedule::new(lsm6dsv32x_ready, SENSOR_IMU_PERIOD_MS),
        },
        |error| match error {
            SensorSpiClusterError::PrimaryImu(error) => {
                warn!("BMI088 acquisition error: {:?}", error);
            }
            SensorSpiClusterError::AuxiliaryImu(error) => {
                warn!("LSM6DSV32X acquisition error: {:?}", error);
            }
        },
    )
    .await
}

#[embassy_executor::task]
async fn bmp581_barometer_task(bus: EnvironmentalI2cBus, pins: EnvironmentalPins) -> ! {
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = BMP581_I2C_FREQUENCY_HZ;

    let mut i2c = I2c::new_async(
        bus.i2c,
        pins.scl,
        pins.sda,
        EnvironmentalI2cIrqs,
        i2c_config,
    );

    let address = loop {
        if let Some(address) = probe_bmp581_address(&mut i2c).await {
            break address;
        } else {
            warn!("BMP581 not detected, retrying");
            Timer::after(Duration::from_secs(1)).await;
        }
    };

    let mut source = Bmp581BarometerSource::new(Bmp581::new(
        i2c,
        EmbassyDelay,
        address,
        Bmp581Config::default(),
    ));

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

    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;
    run_barometer_service(
        &BAROMETER_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        BarometerServiceConfig::new(true, BMP581_PERIOD_MS),
        |error| warn!("BMP581 acquisition error: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn bmm350_magnetometer_task(
    bus: AuxiliaryNavigationI2cBus,
    pins: AuxiliaryNavigationPins,
) -> ! {
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = BMM350_I2C_FREQUENCY_HZ;

    let i2c = I2c::new_async(
        bus.i2c,
        pins.scl,
        pins.sda,
        EnvironmentalI2cIrqs,
        i2c_config,
    );

    let mut driver = Bmm350::new(i2c, BMM350_ADDR);
    let mut init_delay = EmbassyDelay;

    loop {
        match driver.init(&mut init_delay).await {
            Ok(()) => {
                info!("BMM350 initialized at I2C address 0x{=u8:02X}", BMM350_ADDR);
                break;
            }
            Err(error) => {
                warn!("BMM350 initialization failed: {:?}", error);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    let mut source = Bmm350MagnetometerSource::new(driver, EmbassyDelay);
    let clock = EmbassyClock;
    let mut delay = EmbassyDelay;
    run_magnetometer_service(
        &MAGNETOMETER_CHANNEL,
        &mut source,
        &clock,
        &mut delay,
        MagnetometerServiceConfig::new(true, BMM350_PERIOD_MS),
        |error| warn!("BMM350 acquisition error: {:?}", error),
    )
    .await
}
