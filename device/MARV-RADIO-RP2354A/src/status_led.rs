use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::peripherals::PIN_11;

use crate::buses::StatusLedPio;
use crate::channels::{STATUS_LED_COMMAND_CHANNEL, StatusLedCommandReceiver};

#[embassy_executor::task]
async fn status_led_task(
    data_pin: embassy_rp::Peri<'static, PIN_11>,
    pio: StatusLedPio,
    receiver: StatusLedCommandReceiver,
) -> ! {
    let StatusLedPio {
        pio: pio_peripheral,
        dma,
    } = pio;

    rp235x_base::status_led::run_pio0_sm0_ws2812_status_led(data_pin, pio_peripheral, dma, receiver)
        .await
}

pub fn spawn(spawner: &Spawner, data_pin: embassy_rp::Peri<'static, PIN_11>, pio: StatusLedPio) {
    info!("status led task bound to SK6805 data on GP11");
    spawner
        .spawn(status_led_task(
            data_pin,
            pio,
            STATUS_LED_COMMAND_CHANNEL.receiver(),
        ))
        .unwrap();
}
