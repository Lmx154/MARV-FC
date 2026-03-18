#![allow(dead_code)]

use embassy_rp::{Peri, peripherals};

pub struct PayloadI2cBus {
    pub i2c: Peri<'static, peripherals::I2C0>,
}

pub struct PressureAdcBus {
    pub adc: Peri<'static, peripherals::ADC>,
}

pub struct StorageSpiBus {
    pub spi: Peri<'static, peripherals::SPI0>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH0>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH1>,
}

pub struct ServoPwm {
    pub pwm: Peri<'static, peripherals::PWM_SLICE6>,
}

pub struct BusResources {
    pub payload_i2c: PayloadI2cBus,
    pub pressure_adc: PressureAdcBus,
    pub storage: StorageSpiBus,
    pub servo_pwm: ServoPwm,
}
