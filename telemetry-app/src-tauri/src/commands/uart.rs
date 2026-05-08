use tauri::State;

use crate::backend::UartPortInfo;
use crate::bridge::AppState;
use crate::TelemetryBackend;

use super::{mutate_and_snapshot, with_bridge, CommandResult};

#[tauri::command]
pub fn list_uart_ports(backend: State<'_, TelemetryBackend>) -> CommandResult<Vec<UartPortInfo>> {
    with_bridge(backend, |bridge| Ok(bridge.list_uart_ports()))
}

#[tauri::command]
pub fn open_uart(
    backend: State<'_, TelemetryBackend>,
    port_name: String,
    baud_rate: u32,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.open_uart_with_baud_rate(&port_name, baud_rate)
    })
}

#[tauri::command]
pub fn close_uart(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.close_uart();
        Ok(())
    })
}

#[tauri::command]
pub fn set_uart_baud_rate(
    backend: State<'_, TelemetryBackend>,
    baud_rate: u32,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.set_uart_baud_rate(baud_rate))
}
