use tauri::State;

use crate::bridge::AppState;
use crate::TelemetryBackend;

use super::{mutate_and_snapshot, CommandResult};

#[tauri::command]
pub fn start_gazebo_bridge_process(
    backend: State<'_, TelemetryBackend>,
    endpoint: String,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.start_gazebo_bridge_process(&endpoint)
    })
}

#[tauri::command]
pub fn stop_gazebo_bridge_process(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.stop_gazebo_bridge_process())
}

#[tauri::command]
pub fn connect_gazebo_bridge(
    backend: State<'_, TelemetryBackend>,
    endpoint: String,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.connect_gazebo_bridge(&endpoint))
}

#[tauri::command]
pub fn disconnect_gazebo_bridge(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.disconnect_gazebo_bridge();
        Ok(())
    })
}

#[tauri::command]
pub fn send_test_actuator_command(
    backend: State<'_, TelemetryBackend>,
    motor_speed: f32,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_test_actuator_command(motor_speed)
    })
}

#[tauri::command]
pub fn send_gazebo_sim_control_command(
    backend: State<'_, TelemetryBackend>,
    action: String,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_gazebo_sim_control_command(&action)
    })
}
