use tauri::State;

use crate::bridge::AppState;
use crate::TelemetryBackend;

use super::{mutate_and_snapshot, CommandResult};

#[tauri::command]
pub fn clear_serial_monitor(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.clear_serial_monitor();
        Ok(())
    })
}

#[tauri::command]
pub fn set_serial_monitor_parse_enabled(
    backend: State<'_, TelemetryBackend>,
    enabled: bool,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.set_serial_monitor_parse_enabled(enabled);
        Ok(())
    })
}
