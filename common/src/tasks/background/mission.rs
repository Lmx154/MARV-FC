//! Portable mission task bodies.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;

use crate::messages::control::RgbLedCommand;
use crate::messages::sensor::BarometerSampleStamped;
use crate::policies::mission::{Mission, MissionUpdate};

pub async fn run_barometer_rgb_led_mission_task<
    M,
    C,
    MissionPolicy,
    const BARO_DEPTH: usize,
    const BARO_SUBS: usize,
    const BARO_PUBS: usize,
    const COMMAND_DEPTH: usize,
>(
    barometer: &mut Subscriber<'_, M, BarometerSampleStamped, BARO_DEPTH, BARO_SUBS, BARO_PUBS>,
    command_sender: Sender<'_, C, RgbLedCommand, COMMAND_DEPTH>,
    mission: &mut MissionPolicy,
) -> !
where
    M: RawMutex,
    C: RawMutex,
    MissionPolicy: Mission<BarometerSampleStamped, Output = RgbLedCommand>,
{
    if let Some(initial) = mission.initial_output() {
        command_sender.send(initial).await;
    }

    loop {
        let sample = barometer.next_message_pure().await;
        if let MissionUpdate::Command(command) = mission.update(sample) {
            command_sender.send(command).await;
        }
    }
}
