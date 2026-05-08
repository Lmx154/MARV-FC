#![allow(dead_code)]

pub const PAYLOAD_I2C_SDA: u8 = 0;
pub const PAYLOAD_I2C_SCL: u8 = 1;

pub const STORAGE_SPI_SCK: u8 = 2;
pub const STORAGE_SPI_MOSI: u8 = 3;
pub const STORAGE_SPI_MISO: u8 = 4;
pub const STORAGE_SPI_CS: u8 = 5;

pub const STATUS_LED: u8 = 25;
pub const SERVO_PWM: u8 = 28;
pub const PRESSURE_TRANSDUCER_ADC: u8 = 29;
