//! Shared state and the control protocol between Tauri commands and the serial worker thread.

use std::sync::mpsc::Sender;

/// Commands the host can send back to the rocket (translated to LoRa by the radio bridge).
#[derive(Clone, Copy, Debug)]
pub enum CommandKind {
    Arm,
    Disarm,
    Ping,
    MotorStop,
}

/// Messages sent from the UI (via Tauri commands) to the serial worker thread.
pub enum ControlMsg {
    Connect { port: String, baud: u32 },
    Disconnect,
    SendCommand(CommandKind),
}

/// Managed Tauri state: the channel into the serial worker.
pub struct AppState {
    pub tx: Sender<ControlMsg>,
}
