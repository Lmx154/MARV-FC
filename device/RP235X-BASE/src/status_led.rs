use common::messages::control::RgbLedCommand;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma::Channel;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio, PioPin};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver;
use smart_leds::RGB8;

bind_interrupts!(struct Pio0Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

fn to_rgb8(command: RgbLedCommand) -> RGB8 {
    RGB8 {
        r: command.red,
        g: command.green,
        b: command.blue,
    }
}

pub async fn run_pio0_sm0_ws2812_status_led<Pin, Dma, Mutex, const DEPTH: usize>(
    data_pin: Peri<'static, Pin>,
    pio_peripheral: Peri<'static, PIO0>,
    dma: Peri<'static, Dma>,
    receiver: Receiver<'static, Mutex, RgbLedCommand, DEPTH>,
) -> !
where
    Pin: PioPin,
    Dma: Channel,
    Mutex: RawMutex,
{
    let mut pio = Pio::new(pio_peripheral, Pio0Irqs);
    let program = PioWs2812Program::new(&mut pio.common);
    let mut led = PioWs2812::<PIO0, 0, 1>::new(&mut pio.common, pio.sm0, dma, data_pin, &program);
    let mut colors = [to_rgb8(RgbLedCommand::OFF)];

    led.write(&colors).await;

    loop {
        let command = receiver.receive().await;
        colors[0] = to_rgb8(command);
        led.write(&colors).await;
    }
}
