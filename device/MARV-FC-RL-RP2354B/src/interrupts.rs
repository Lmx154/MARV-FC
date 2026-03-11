#![allow(dead_code)]

use embassy_rp::{bind_interrupts, peripherals, uart};

bind_interrupts!(pub struct RadioLinkIrqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
});

bind_interrupts!(pub struct CompanionLinkIrqs {
    UART1_IRQ => uart::InterruptHandler<peripherals::UART1>;
});
