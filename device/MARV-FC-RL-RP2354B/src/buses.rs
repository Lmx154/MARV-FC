#![allow(dead_code)]

use embassy_rp::{Peri, peripherals};

pub struct SensorSpiBus {
    pub spi: Peri<'static, peripherals::SPI1>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH0>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH1>,
}

pub struct StorageSpiBus {
    pub spi: Peri<'static, peripherals::SPI0>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH2>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH3>,
}

pub struct RadioLinkUart {
    pub uart: Peri<'static, peripherals::UART0>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH4>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH5>,
}

pub struct CompanionLinkUart {
    pub uart: Peri<'static, peripherals::UART1>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH6>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH7>,
}

pub struct EnvironmentalI2cBus {
    pub i2c: Peri<'static, peripherals::I2C0>,
}

pub struct AuxiliaryNavigationI2cBus {
    pub i2c: Peri<'static, peripherals::I2C1>,
}

pub struct StatusLedPio {
    pub pio: Peri<'static, peripherals::PIO0>,
    pub dma: Peri<'static, peripherals::DMA_CH8>,
}

pub struct ActuatorPwm {
    pub pwm8: Peri<'static, peripherals::PWM_SLICE8>,
    pub pwm9: Peri<'static, peripherals::PWM_SLICE9>,
    pub pwm10: Peri<'static, peripherals::PWM_SLICE10>,
    pub pwm11: Peri<'static, peripherals::PWM_SLICE11>,
}

pub struct BusResources {
    pub sensors: SensorSpiBus,
    pub storage: StorageSpiBus,
    pub environmental: EnvironmentalI2cBus,
    pub auxiliary_navigation: AuxiliaryNavigationI2cBus,
    pub radio_link: RadioLinkUart,
    pub companion_link: CompanionLinkUart,
    pub status_led: StatusLedPio,
    pub actuators: ActuatorPwm,
}
