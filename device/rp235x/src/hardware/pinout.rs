//! Physical pin assignments for the RP235x FC binary.
//! Source of truth: `docs/pcbhardware.md` (Flight Controller section).
#![allow(dead_code)]

/// Flight-controller board pinout (GPIO numbers).
pub mod fc {
    /// Shared SPI1 bus for BMI088 (dual CS) and LSM6DSV32X.
    pub mod imu_spi1 {
        pub const MOSI: u8 = 11;
        pub const MISO: u8 = 12;
        pub const SCK: u8 = 10;
        pub const CS_ACCEL: u8 = 13;
        pub const CS_GYRO: u8 = 14;
        pub const CS_LSM: u8 = 16;
    }

    /// SPI0 bus for microSD card.
    pub mod sd_spi0 {
        pub const MOSI: u8 = 19;
        pub const MISO: u8 = 20;
        pub const SCK: u8 = 18;
        pub const CS: u8 = 21;
    }

    /// On-board I2C0 (BMP581) and external I2C0 pad share these lines.
    pub mod i2c0 {
        pub const SDA: u8 = 8;
        pub const SCL: u8 = 9;
    }

    /// On-board I2C1 (BMM350 + NEO-M9N) and external I2C1 pad share these lines.
    pub mod i2c1 {
        pub const SDA: u8 = 2;
        pub const SCL: u8 = 3;
    }

    /// Addressable RGB LED data pin.
    pub mod led {
        pub const DATA: u8 = 6;
    }

    /// External FC-RADIO UART0 pads.
    pub mod uart0_fc_radio {
        pub const TX: u8 = 0;
        pub const RX: u8 = 1;
    }

    /// External FC-SBC UART1 pads.
    pub mod uart1_fc_sbc {
        pub const TX: u8 = 4;
        pub const RX: u8 = 5;
    }

    /// External generic GPIO pads.
    pub mod external_gpio {
        pub const IO1: u8 = 28;
        pub const IO2: u8 = 29;
        pub const IO3: u8 = 30;
        pub const IO4: u8 = 31;
    }

    /// ESC pads.
    pub mod esc {
        pub const PWM1: u8 = 39;
        pub const PWM2: u8 = 38;
        pub const PWM3: u8 = 35;
        pub const PWM4: u8 = 36;
        pub const TELEMETRY: u8 = 37;
    }

    /// VTX UART pads.
    pub mod vtx {
        pub const TX: u8 = 32;
        pub const RX: u8 = 33;
    }
}
