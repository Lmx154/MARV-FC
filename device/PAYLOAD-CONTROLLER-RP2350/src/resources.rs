#![allow(dead_code)]

use embassy_rp::{Peri, Peripherals, peripherals};

use crate::buses::{BusResources, PayloadI2cBus, PressureAdcBus, ServoPwm, StorageSpiBus};
use crate::config;
use crate::watchdog::WatchdogResources;

pub struct PayloadSensorPins {
    pub sda: Peri<'static, peripherals::PIN_0>,
    pub scl: Peri<'static, peripherals::PIN_1>,
    pub pressure_adc: Peri<'static, peripherals::PIN_29>,
}

pub struct StoragePins {
    pub sck: Peri<'static, peripherals::PIN_2>,
    pub mosi: Peri<'static, peripherals::PIN_3>,
    pub miso: Peri<'static, peripherals::PIN_4>,
    pub cs: Peri<'static, peripherals::PIN_5>,
}

pub struct StatusPins {
    pub led: Peri<'static, peripherals::PIN_25>,
}

pub struct ActuatorPins {
    pub servo_pwm: Peri<'static, peripherals::PIN_28>,
}

pub struct PinResources {
    pub sensors: PayloadSensorPins,
    pub storage: StoragePins,
    pub status: StatusPins,
    pub actuators: ActuatorPins,
}

pub struct SystemResources {
    pub usb: Peri<'static, peripherals::USB>,
    pub flash: Peri<'static, peripherals::FLASH>,
    pub core1: Peri<'static, peripherals::CORE1>,
}

pub struct DeviceResources {
    pub pins: PinResources,
    pub buses: BusResources,
    pub watchdog: WatchdogResources,
    pub system: SystemResources,
}

pub fn split(peripherals: Peripherals) -> DeviceResources {
    let Peripherals {
        PIN_0: payload_i2c_sda,
        PIN_1: payload_i2c_scl,
        PIN_2: storage_spi_sck,
        PIN_3: storage_spi_mosi,
        PIN_4: storage_spi_miso,
        PIN_5: storage_spi_cs,
        PIN_25: status_led,
        PIN_28: servo_pwm,
        PIN_29: pressure_transducer_adc,
        ADC: adc,
        I2C0: i2c0,
        SPI0: spi0,
        DMA_CH0: spi0_tx_dma,
        DMA_CH1: spi0_rx_dma,
        PWM_SLICE6: pwm6,
        USB: usb,
        FLASH: flash,
        CORE1: core1,
        WATCHDOG: watchdog,
        ..
    } = peripherals;

    DeviceResources {
        pins: PinResources {
            sensors: PayloadSensorPins {
                sda: payload_i2c_sda,
                scl: payload_i2c_scl,
                pressure_adc: pressure_transducer_adc,
            },
            storage: StoragePins {
                sck: storage_spi_sck,
                mosi: storage_spi_mosi,
                miso: storage_spi_miso,
                cs: storage_spi_cs,
            },
            status: StatusPins { led: status_led },
            actuators: ActuatorPins { servo_pwm },
        },
        buses: BusResources {
            payload_i2c: PayloadI2cBus { i2c: i2c0 },
            pressure_adc: PressureAdcBus { adc },
            storage: StorageSpiBus {
                spi: spi0,
                tx_dma: spi0_tx_dma,
                rx_dma: spi0_rx_dma,
            },
            servo_pwm: ServoPwm { pwm: pwm6 },
        },
        watchdog: WatchdogResources {
            peripheral: watchdog,
            timeout_ms: config::WATCHDOG_TIMEOUT_MS,
        },
        system: SystemResources { usb, flash, core1 },
    }
}
