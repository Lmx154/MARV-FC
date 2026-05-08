#![allow(dead_code)]

use embassy_rp::{bind_interrupts, peripherals, uart};

bind_interrupts!(pub struct RadioLinkIrqs {
    UART1_IRQ => uart::BufferedInterruptHandler<peripherals::UART1>;
});

bind_interrupts!(pub struct CompanionLinkIrqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
});
