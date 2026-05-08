#![allow(dead_code)]

use embassy_rp::{Peri, peripherals};

pub struct LoraSpiBus {
    pub spi: Peri<'static, peripherals::SPI0>,
    pub tx_dma: Peri<'static, peripherals::DMA_CH0>,
    pub rx_dma: Peri<'static, peripherals::DMA_CH1>,
}

pub struct HostUartBus {
    pub uart: Peri<'static, peripherals::UART0>,
}

pub struct StatusLedPio {
    pub pio: Peri<'static, peripherals::PIO0>,
    pub dma: Peri<'static, peripherals::DMA_CH2>,
}

pub struct BusResources {
    pub lora: LoraSpiBus,
    pub host_uart: HostUartBus,
    pub status_led: StatusLedPio,
}
