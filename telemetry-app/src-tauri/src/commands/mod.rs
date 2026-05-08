pub mod app;
pub mod gazebo;
pub mod hilink;
pub mod serial;
pub mod uart;

use tauri::State;

use crate::bridge::{AppBridge, AppState};
use crate::TelemetryBackend;

type CommandResult<T> = Result<T, String>;

fn with_bridge<T>(
    backend: State<'_, TelemetryBackend>,
    action: impl FnOnce(&mut AppBridge) -> CommandResult<T>,
) -> CommandResult<T> {
    let mut bridge = backend
        .bridge
        .lock()
        .map_err(|_| "telemetry backend lock was poisoned".to_string())?;
    action(&mut bridge)
}

fn mutate_and_snapshot(
    backend: State<'_, TelemetryBackend>,
    action: impl FnOnce(&mut AppBridge) -> CommandResult<()>,
) -> CommandResult<AppState> {
    with_bridge(backend, |bridge| {
        action(bridge)?;
        bridge.tick();
        Ok(bridge.snapshot())
    })
}
