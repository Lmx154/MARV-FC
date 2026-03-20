#![allow(dead_code)]

use embassy_rp::{Peri, peripherals};

pub struct RecoveryI2cBus {
    pub i2c: Peri<'static, peripherals::I2C0>,
}

pub struct RecoveryAdcBus {
    pub adc: Peri<'static, peripherals::ADC>,
}

pub struct StorageSpiBus {
    pub spi: Peri<'static, peripherals::SPI0>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH0>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH1>,
}

pub struct BusResources {
    pub recovery_i2c: RecoveryI2cBus,
    pub recovery_adc: RecoveryAdcBus,
    pub storage: StorageSpiBus,
}
