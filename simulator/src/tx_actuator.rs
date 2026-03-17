#![allow(dead_code)]

use tokio::net::UdpSocket;

use crate::config::Config;
use crate::mavlink::{ActuatorControlTargetMessage, encode_actuator_control_target};

pub async fn send_actuator_command(
    socket: &UdpSocket,
    config: &Config,
    sequence: &mut u8,
    message: &ActuatorControlTargetMessage,
) -> std::io::Result<usize> {
    let frame =
        encode_actuator_control_target(message, *sequence, config.system_id, config.component_id);
    *sequence = sequence.wrapping_add(1);
    socket.send_to(&frame, config.actuator_addr).await
}
