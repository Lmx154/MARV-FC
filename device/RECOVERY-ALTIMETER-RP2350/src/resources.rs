#![allow(dead_code)]

use embassy_rp::{Peri, Peripherals, peripherals};

use crate::buses::{BusResources, RecoveryAdcBus, RecoveryI2cBus, StorageSpiBus};
use crate::config;
use crate::watchdog::WatchdogResources;

pub struct RecoverySensorPins {
    pub sda: Peri<'static, peripherals::PIN_0>,
    pub scl: Peri<'static, peripherals::PIN_1>,
    pub buzzer_continuity_adc: Peri<'static, peripherals::PIN_26>,
    pub drogue_continuity_adc: Peri<'static, peripherals::PIN_27>,
    pub main_continuity_adc: Peri<'static, peripherals::PIN_28>,
}

pub struct StoragePins {
    pub sck: Peri<'static, peripherals::PIN_2>,
    pub mosi: Peri<'static, peripherals::PIN_3>,
    pub miso: Peri<'static, peripherals::PIN_4>,
    pub cs: Peri<'static, peripherals::PIN_5>,
}

pub struct PinResources {
    pub sensors: RecoverySensorPins,
    pub storage: StoragePins,
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
        PIN_0: recovery_i2c_sda,
        PIN_1: recovery_i2c_scl,
        PIN_2: storage_spi_sck,
        PIN_3: storage_spi_mosi,
        PIN_4: storage_spi_miso,
        PIN_5: storage_spi_cs,
        PIN_26: buzzer_continuity_adc,
        PIN_27: drogue_continuity_adc,
        PIN_28: main_continuity_adc,
        ADC: adc,
        I2C0: i2c0,
        SPI0: spi0,
        DMA_CH0: spi0_tx_dma,
        DMA_CH1: spi0_rx_dma,
        USB: usb,
        FLASH: flash,
        CORE1: core1,
        WATCHDOG: watchdog,
        ..
    } = peripherals;

    DeviceResources {
        pins: PinResources {
            sensors: RecoverySensorPins {
                sda: recovery_i2c_sda,
                scl: recovery_i2c_scl,
                buzzer_continuity_adc,
                drogue_continuity_adc,
                main_continuity_adc,
            },
            storage: StoragePins {
                sck: storage_spi_sck,
                mosi: storage_spi_mosi,
                miso: storage_spi_miso,
                cs: storage_spi_cs,
            },
        },
        buses: BusResources {
            recovery_i2c: RecoveryI2cBus { i2c: i2c0 },
            recovery_adc: RecoveryAdcBus { adc },
            storage: StorageSpiBus {
                spi: spi0,
                tx_dma: spi0_tx_dma,
                rx_dma: spi0_rx_dma,
            },
        },
        watchdog: WatchdogResources {
            peripheral: watchdog,
            timeout_ms: config::WATCHDOG_TIMEOUT_MS,
        },
        system: SystemResources { usb, flash, core1 },
    }
}
