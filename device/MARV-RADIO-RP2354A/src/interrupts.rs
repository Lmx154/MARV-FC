#![allow(dead_code)]

use embassy_rp::{bind_interrupts, peripherals, uart};

bind_interrupts!(pub struct HostUartIrqs {
    UART0_IRQ => uart::BufferedInterruptHandler<peripherals::UART0>;
});
