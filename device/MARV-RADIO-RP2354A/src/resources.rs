#![allow(dead_code)]

use embassy_rp::{Peri, Peripherals, peripherals};

use crate::buses::{BusResources, HostUartBus, LoraSpiBus, StatusLedPio};
use crate::config;
use crate::watchdog::WatchdogResources;

pub struct LoraPins {
    pub miso: Peri<'static, peripherals::PIN_0>,
    pub cs: Peri<'static, peripherals::PIN_1>,
    pub sck: Peri<'static, peripherals::PIN_2>,
    pub mosi: Peri<'static, peripherals::PIN_3>,
    pub busy: Peri<'static, peripherals::PIN_4>,
    pub reset: Peri<'static, peripherals::PIN_5>,
    pub rxen: Peri<'static, peripherals::PIN_6>,
    pub txen: Peri<'static, peripherals::PIN_7>,
    pub dio2: Peri<'static, peripherals::PIN_8>,
    pub dio1: Peri<'static, peripherals::PIN_9>,
}

pub struct StatusPins {
    pub data: Peri<'static, peripherals::PIN_11>,
}

pub struct HostUartPins {
    pub tx: Peri<'static, peripherals::PIN_12>,
    pub rx: Peri<'static, peripherals::PIN_13>,
}

pub struct PinResources {
    pub lora: LoraPins,
    pub status: StatusPins,
    pub host_uart: HostUartPins,
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
        PIN_0: lora_spi_miso,
        PIN_1: lora_spi_cs,
        PIN_2: lora_spi_sck,
        PIN_3: lora_spi_mosi,
        PIN_4: lora_busy,
        PIN_5: lora_reset,
        PIN_6: lora_rxen,
        PIN_7: lora_txen,
        PIN_8: lora_dio2,
        PIN_9: lora_dio1,
        PIN_11: status_led_data,
        PIN_12: host_uart_tx,
        PIN_13: host_uart_rx,
        SPI0: lora_spi,
        UART0: host_uart,
        DMA_CH0: lora_spi_tx_dma,
        DMA_CH1: lora_spi_rx_dma,
        DMA_CH2: status_led_dma,
        PIO0: status_led_pio,
        USB: usb,
        FLASH: flash,
        CORE1: core1,
        WATCHDOG: watchdog,
        ..
    } = peripherals;

    DeviceResources {
        pins: PinResources {
            lora: LoraPins {
                miso: lora_spi_miso,
                cs: lora_spi_cs,
                sck: lora_spi_sck,
                mosi: lora_spi_mosi,
                busy: lora_busy,
                reset: lora_reset,
                rxen: lora_rxen,
                txen: lora_txen,
                dio2: lora_dio2,
                dio1: lora_dio1,
            },
            status: StatusPins {
                data: status_led_data,
            },
            host_uart: HostUartPins {
                tx: host_uart_tx,
                rx: host_uart_rx,
            },
        },
        buses: BusResources {
            lora: LoraSpiBus {
                spi: lora_spi,
                tx_dma: lora_spi_tx_dma,
                rx_dma: lora_spi_rx_dma,
            },
            host_uart: HostUartBus { uart: host_uart },
            status_led: StatusLedPio {
                pio: status_led_pio,
                dma: status_led_dma,
            },
        },
        watchdog: WatchdogResources {
            peripheral: watchdog,
            timeout_ms: config::WATCHDOG_TIMEOUT_MS,
        },
        system: SystemResources { usb, flash, core1 },
    }
}
