#![allow(dead_code)]

use embassy_rp::{Peri, Peripherals, peripherals};

use crate::buses::{
    ActuatorPwm, AuxiliaryNavigationI2cBus, BusResources, CompanionLinkUart, EnvironmentalI2cBus,
    RadioLinkUart, SensorSpiBus, StatusLedPio, StorageSpiBus,
};
use crate::config;
use crate::watchdog::{FeedContract, WatchdogResources};

pub struct SensorPins {
    pub sck: Peri<'static, peripherals::PIN_10>,
    pub mosi: Peri<'static, peripherals::PIN_11>,
    pub miso: Peri<'static, peripherals::PIN_12>,
    pub bmi088_accel_cs: Peri<'static, peripherals::PIN_13>,
    pub bmi088_gyro_cs: Peri<'static, peripherals::PIN_14>,
    pub lsm6dsv32x_cs: Peri<'static, peripherals::PIN_16>,
}

pub struct StoragePins {
    pub sck: Peri<'static, peripherals::PIN_18>,
    pub mosi: Peri<'static, peripherals::PIN_19>,
    pub miso: Peri<'static, peripherals::PIN_20>,
    pub cs: Peri<'static, peripherals::PIN_21>,
}

pub struct EnvironmentalPins {
    pub sda: Peri<'static, peripherals::PIN_8>,
    pub scl: Peri<'static, peripherals::PIN_9>,
}

pub struct AuxiliaryNavigationPins {
    pub sda: Peri<'static, peripherals::PIN_2>,
    pub scl: Peri<'static, peripherals::PIN_3>,
}

pub struct RadioLinkPins {
    pub tx: Peri<'static, peripherals::PIN_0>,
    pub rx: Peri<'static, peripherals::PIN_1>,
}

pub struct CompanionLinkPins {
    pub tx: Peri<'static, peripherals::PIN_4>,
    pub rx: Peri<'static, peripherals::PIN_5>,
}

pub struct StatusPins {
    pub data: Peri<'static, peripherals::PIN_6>,
}

pub struct ExpansionPins {
    pub io1: Peri<'static, peripherals::PIN_28>,
    pub io2: Peri<'static, peripherals::PIN_29>,
    pub io3: Peri<'static, peripherals::PIN_30>,
    pub io4: Peri<'static, peripherals::PIN_31>,
}

pub struct VtxPins {
    pub tx: Peri<'static, peripherals::PIN_32>,
    pub rx: Peri<'static, peripherals::PIN_33>,
}

pub struct ActuatorPins {
    pub pwm1: Peri<'static, peripherals::PIN_39>,
    pub pwm2: Peri<'static, peripherals::PIN_38>,
    pub pwm3: Peri<'static, peripherals::PIN_35>,
    pub pwm4: Peri<'static, peripherals::PIN_36>,
    pub telemetry: Peri<'static, peripherals::PIN_37>,
}

pub struct PinResources {
    pub sensors: SensorPins,
    pub storage: StoragePins,
    pub environmental: EnvironmentalPins,
    pub auxiliary_navigation: AuxiliaryNavigationPins,
    pub radio_link: RadioLinkPins,
    pub companion_link: CompanionLinkPins,
    pub status: StatusPins,
    pub expansion: ExpansionPins,
    pub vtx: VtxPins,
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
        PIN_0: fc_radio_tx,
        PIN_1: fc_radio_rx,
        PIN_2: aux_i2c_sda,
        PIN_3: aux_i2c_scl,
        PIN_4: fc_sbc_tx,
        PIN_5: fc_sbc_rx,
        PIN_6: status_led_data,
        PIN_8: env_i2c_sda,
        PIN_9: env_i2c_scl,
        PIN_10: sensor_spi_sck,
        PIN_11: sensor_spi_mosi,
        PIN_12: sensor_spi_miso,
        PIN_13: bmi088_accel_cs,
        PIN_14: bmi088_gyro_cs,
        PIN_16: lsm6dsv32x_cs,
        PIN_18: storage_spi_sck,
        PIN_19: storage_spi_mosi,
        PIN_20: storage_spi_miso,
        PIN_21: storage_spi_cs,
        PIN_28: ext_io_1,
        PIN_29: ext_io_2,
        PIN_30: ext_io_3,
        PIN_31: ext_io_4,
        PIN_32: vtx_tx,
        PIN_33: vtx_rx,
        PIN_35: esc_pwm_3,
        PIN_36: esc_pwm_4,
        PIN_37: esc_telemetry,
        PIN_38: esc_pwm_2,
        PIN_39: esc_pwm_1,
        SPI0: storage_spi,
        SPI1: sensor_spi,
        I2C0: environmental_i2c,
        I2C1: auxiliary_navigation_i2c,
        UART0: radio_uart,
        UART1: companion_uart,
        DMA_CH0: sensor_spi_tx_dma,
        DMA_CH1: sensor_spi_rx_dma,
        DMA_CH2: storage_spi_tx_dma,
        DMA_CH3: storage_spi_rx_dma,
        DMA_CH4: radio_uart_tx_dma,
        DMA_CH5: radio_uart_rx_dma,
        DMA_CH6: companion_uart_tx_dma,
        DMA_CH7: companion_uart_rx_dma,
        DMA_CH8: status_led_dma,
        PWM_SLICE8: pwm8,
        PWM_SLICE9: pwm9,
        PWM_SLICE10: pwm10,
        PWM_SLICE11: pwm11,
        USB: usb,
        FLASH: flash,
        CORE1: core1,
        PIO0: pio0,
        WATCHDOG: watchdog,
        ..
    } = peripherals;

    DeviceResources {
        pins: PinResources {
            sensors: SensorPins {
                sck: sensor_spi_sck,
                mosi: sensor_spi_mosi,
                miso: sensor_spi_miso,
                bmi088_accel_cs,
                bmi088_gyro_cs,
                lsm6dsv32x_cs,
            },
            storage: StoragePins {
                sck: storage_spi_sck,
                mosi: storage_spi_mosi,
                miso: storage_spi_miso,
                cs: storage_spi_cs,
            },
            environmental: EnvironmentalPins {
                sda: env_i2c_sda,
                scl: env_i2c_scl,
            },
            auxiliary_navigation: AuxiliaryNavigationPins {
                sda: aux_i2c_sda,
                scl: aux_i2c_scl,
            },
            radio_link: RadioLinkPins {
                tx: fc_radio_tx,
                rx: fc_radio_rx,
            },
            companion_link: CompanionLinkPins {
                tx: fc_sbc_tx,
                rx: fc_sbc_rx,
            },
            status: StatusPins {
                data: status_led_data,
            },
            expansion: ExpansionPins {
                io1: ext_io_1,
                io2: ext_io_2,
                io3: ext_io_3,
                io4: ext_io_4,
            },
            vtx: VtxPins {
                tx: vtx_tx,
                rx: vtx_rx,
            },
            actuators: ActuatorPins {
                pwm1: esc_pwm_1,
                pwm2: esc_pwm_2,
                pwm3: esc_pwm_3,
                pwm4: esc_pwm_4,
                telemetry: esc_telemetry,
            },
        },
        buses: BusResources {
            sensors: SensorSpiBus {
                spi: sensor_spi,
                tx_dma: sensor_spi_tx_dma,
                rx_dma: sensor_spi_rx_dma,
            },
            storage: StorageSpiBus {
                spi: storage_spi,
                tx_dma: storage_spi_tx_dma,
                rx_dma: storage_spi_rx_dma,
            },
            environmental: EnvironmentalI2cBus {
                i2c: environmental_i2c,
            },
            auxiliary_navigation: AuxiliaryNavigationI2cBus {
                i2c: auxiliary_navigation_i2c,
            },
            radio_link: RadioLinkUart {
                uart: radio_uart,
                tx_dma: radio_uart_tx_dma,
                rx_dma: radio_uart_rx_dma,
            },
            companion_link: CompanionLinkUart {
                uart: companion_uart,
                tx_dma: companion_uart_tx_dma,
                rx_dma: companion_uart_rx_dma,
            },
            status_led: StatusLedPio {
                pio: pio0,
                dma: status_led_dma,
            },
            actuators: ActuatorPwm {
                pwm8,
                pwm9,
                pwm10,
                pwm11,
            },
        },
        watchdog: WatchdogResources {
            peripheral: watchdog,
            timeout_ms: config::WATCHDOG_TIMEOUT_MS,
            contract: FeedContract::marv_fc_default(),
        },
        system: SystemResources { usb, flash, core1 },
    }
}
