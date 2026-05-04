#![allow(dead_code)]

pub const LORA_SPI_MISO: u8 = 0;
pub const LORA_SPI_CS: u8 = 1;
pub const LORA_SPI_SCK: u8 = 2;
pub const LORA_SPI_MOSI: u8 = 3;
pub const LORA_BUSY: u8 = 4;
pub const LORA_RESET: u8 = 5;
pub const LORA_RXEN: u8 = 6;
pub const LORA_TXEN: u8 = 7;
pub const LORA_DIO2: u8 = 8;
pub const LORA_DIO1: u8 = 9;

pub const STATUS_LED_DATA: u8 = 11;

pub const HOST_UART_TX: u8 = 12;
pub const HOST_UART_RX: u8 = 13;
pub const HOST_UART_BAUD: u32 = crate::config::HOST_UART_BAUD;
