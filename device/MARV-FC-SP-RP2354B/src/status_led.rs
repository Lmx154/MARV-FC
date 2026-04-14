use common::messages::control::RgbLedCommand;
use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_6, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use smart_leds::RGB8;

use crate::buses::StatusLedPio;
use crate::channels::{FcRgbLedCommandReceiver, RGB_LED_COMMAND_CHANNEL};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

fn to_rgb8(command: RgbLedCommand) -> RGB8 {
    RGB8 {
        r: command.red,
        g: command.green,
        b: command.blue,
    }
}

#[embassy_executor::task]
async fn status_led_task(
    data_pin: embassy_rp::Peri<'static, PIN_6>,
    pio: StatusLedPio,
    receiver: FcRgbLedCommandReceiver,
) -> ! {
    let StatusLedPio {
        pio: pio_peripheral,
        dma,
    } = pio;
    let mut pio = Pio::new(pio_peripheral, Irqs);
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

pub fn spawn(spawner: &Spawner, data_pin: embassy_rp::Peri<'static, PIN_6>, pio: StatusLedPio) {
    info!("status led task bound to RGB LED on GP6");
    spawner
        .spawn(status_led_task(
            data_pin,
            pio,
            RGB_LED_COMMAND_CHANNEL.receiver(),
        ))
        .unwrap();
}
