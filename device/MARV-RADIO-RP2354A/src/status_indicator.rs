use common::comms::links::lora::state::LoraLinkState;
use common::messages::control::RgbLedCommand;
use defmt::info;
use embassy_executor::Spawner;

use crate::channels::{
    STATUS_INDICATOR_CHANNEL, STATUS_LED_COMMAND_CHANNEL, StatusIndicatorEvent,
    StatusIndicatorReceiver,
};

const UNPAIRED_COLOR: RgbLedCommand = RgbLedCommand::new(4, 4, 4);
const PAIRED_COLOR: RgbLedCommand = RgbLedCommand::new(0, 0, 32);
const DEGRADED_COLOR: RgbLedCommand = RgbLedCommand::new(32, 8, 0);
const LOST_COLOR: RgbLedCommand = RgbLedCommand::new(16, 0, 0);
const RADIO_FAULT_COLOR: RgbLedCommand = RgbLedCommand::new(32, 0, 0);

fn state_color(state: LoraLinkState) -> RgbLedCommand {
    match state {
        LoraLinkState::Acquiring => UNPAIRED_COLOR,
        LoraLinkState::Linked => PAIRED_COLOR,
        LoraLinkState::Degraded => DEGRADED_COLOR,
        LoraLinkState::Lost => LOST_COLOR,
        LoraLinkState::RadioFault => RADIO_FAULT_COLOR,
    }
}

#[embassy_executor::task]
async fn status_indicator_task(receiver: StatusIndicatorReceiver) -> ! {
    let led = STATUS_LED_COMMAND_CHANNEL.sender();
    let mut link_state = LoraLinkState::Acquiring;

    led.send(state_color(link_state)).await;

    loop {
        let event = receiver.receive().await;

        match event {
            StatusIndicatorEvent::LinkState(state) => {
                if state != link_state {
                    link_state = state;
                    info!("lora link state changed: {:?}", link_state);
                }
                led.send(state_color(link_state)).await;
            }
        }
    }
}

pub fn spawn(spawner: &Spawner) {
    spawner
        .spawn(status_indicator_task(STATUS_INDICATOR_CHANNEL.receiver()))
        .expect("status indicator task spawn failed");
}
