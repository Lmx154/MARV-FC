use tauri::State;

use crate::bridge::AppState;
use crate::TelemetryBackend;

use super::{with_bridge, CommandResult};

#[tauri::command]
pub fn backend_snapshot(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    with_bridge(backend, |bridge| {
        bridge.tick();
        Ok(bridge.snapshot())
    })
}

#[tauri::command]
pub fn backend_tick(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    backend_snapshot(backend)
}
