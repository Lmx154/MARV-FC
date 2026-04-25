use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use rp235x_base::pio_uart::new_duplex_pio_uart;

use crate::buses::GpsPioUartBus;
use crate::pinmap;
use crate::resources::GpsPioUartPins;

bind_interrupts!(struct GpsPioIrqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

pub fn spawn(spawner: &Spawner, bus: GpsPioUartBus, pins: GpsPioUartPins) {
    spawner.spawn(gps_pio_uart_task(bus.pio, pins)).unwrap();
}

#[embassy_executor::task]
async fn gps_pio_uart_task(pio_peripheral: Peri<'static, PIO1>, pins: GpsPioUartPins) -> ! {
    let mut pio = Pio::new(pio_peripheral, GpsPioIrqs);
    let (_gps_tx, mut gps_rx) = new_duplex_pio_uart(
        &mut pio.common,
        pio.sm0,
        pio.sm1,
        pins,
        pinmap::GPS_PIO_UART_BAUD,
    );

    info!(
        "sam-m10q gps pio uart ready: tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        pinmap::GPS_PIO_UART_TX,
        pinmap::GPS_PIO_UART_RX,
        pinmap::GPS_PIO_UART_BAUD,
    );

    loop {
        let _byte = gps_rx.read_u8().await;
    }
}
