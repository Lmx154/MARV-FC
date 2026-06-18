//! Shared state and the control protocol between Tauri commands and the serial worker thread.

use std::sync::mpsc::Sender;

/// Commands the host can send back to the rocket (translated to LoRa by the radio bridge).
#[derive(Clone, Copy, Debug)]
pub enum CommandKind {
    Arm,
    Disarm,
    Ping,
    MotorStop,
    /// Change the link RF profile (consumed by the ground-station radio, then driven link-wide).
    SetRadioProfile {
        preset: u8,
        tx_power_dbm: i8,
        frequency_hz: u32,
        flags: u16,
    },
    /// Set the idle-fallback window: how long a radio waits, hearing nothing from its peer, before
    /// returning to the boot/"setup" profile. The GS adopts it and relays it to the vehicle.
    SetIdleFallback { idle_fallback_ms: u32 },
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
