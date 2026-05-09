use tauri::State;

use crate::backend::{
    CvWaypointCommand, HilSensorFrameCommand, MotorSweepCommand, MotorTestCommand,
    TofWaypointCommand,
};
use crate::bridge::AppState;
use crate::TelemetryBackend;

use super::{mutate_and_snapshot, CommandResult};

#[tauri::command]
pub fn send_hilink_ping(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_ping())
}

#[tauri::command]
pub fn send_hilink_arm(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_arm())
}

#[tauri::command]
pub fn send_hilink_disarm(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_disarm())
}

#[tauri::command]
pub fn send_hilink_rtl(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_rtl())
}

#[tauri::command]
pub fn send_hilink_bench_enable(
    backend: State<'_, TelemetryBackend>,
    timeout_ms: u16,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_bench_enable(timeout_ms)
    })
}

#[tauri::command]
pub fn send_hilink_bench_disable(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_bench_disable())
}

#[tauri::command]
pub fn send_hilink_motor_test(
    backend: State<'_, TelemetryBackend>,
    command: MotorTestCommand,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_motor_test(command))
}

#[tauri::command]
pub fn send_hilink_motor_test_values(
    backend: State<'_, TelemetryBackend>,
    motor_mask: u8,
    mode: u8,
    value: u16,
    duration_ms: u16,
    ramp_ms: u16,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_motor_test_values(motor_mask, mode, value, duration_ms, ramp_ms)
    })
}

#[tauri::command]
pub fn send_hilink_motor_sweep(
    backend: State<'_, TelemetryBackend>,
    command: MotorSweepCommand,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_motor_sweep(command))
}

#[tauri::command]
pub fn send_hilink_motor_sweep_values(
    backend: State<'_, TelemetryBackend>,
    motor_mask: u8,
    mode: u8,
    start_value: u16,
    end_value: u16,
    step_value: u16,
    step_duration_ms: u16,
    zero_between_ms: u16,
    repeat_count: u8,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_motor_sweep_values(
            motor_mask,
            mode,
            start_value,
            end_value,
            step_value,
            step_duration_ms,
            zero_between_ms,
            repeat_count,
        )
    })
}

#[tauri::command]
pub fn send_hilink_motor_stop(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_motor_stop())
}

#[tauri::command]
pub fn run_radio_link_smoke_test(backend: State<'_, TelemetryBackend>) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.run_radio_link_smoke_test())
}

#[tauri::command]
pub fn send_hilink_dshot_command(
    backend: State<'_, TelemetryBackend>,
    motor_mask: u8,
    command: u8,
    repeat_count: u8,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_dshot_command(motor_mask, command, repeat_count)
    })
}

#[tauri::command]
pub fn send_hilink_actuator_status_request(
    backend: State<'_, TelemetryBackend>,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_actuator_status_request()
    })
}

#[tauri::command]
pub fn send_hilink_mixer_motor_order(
    backend: State<'_, TelemetryBackend>,
    output_for_motor: [u8; 4],
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_mixer_motor_order(output_for_motor)
    })
}

#[tauri::command]
pub fn send_hilink_control_waypoint(
    backend: State<'_, TelemetryBackend>,
    ref_sim_tick: u64,
    ref_sim_time_us: u64,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    yaw_deg: f32,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_control_waypoint(
            ref_sim_tick,
            ref_sim_time_us,
            lat_deg,
            lon_deg,
            alt_msl_m,
            yaw_deg,
        )
    })
}

#[tauri::command]
pub fn send_hilink_mission_waypoint(
    backend: State<'_, TelemetryBackend>,
    ref_sim_tick: u64,
    ref_sim_time_us: u64,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    yaw_deg: f32,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_mission_waypoint(
            ref_sim_tick,
            ref_sim_time_us,
            lat_deg,
            lon_deg,
            alt_msl_m,
            yaw_deg,
        )
    })
}

#[tauri::command]
pub fn send_hilink_cv_waypoint(
    backend: State<'_, TelemetryBackend>,
    command: CvWaypointCommand,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_cv_waypoint(
            command.ref_sim_tick,
            command.ref_sim_time_us,
            command.dir_body,
            command.confidence,
        )
    })
}

#[tauri::command]
pub fn send_hilink_tof_waypoint(
    backend: State<'_, TelemetryBackend>,
    command: TofWaypointCommand,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_tof_waypoint(
            command.ref_sim_tick,
            command.ref_sim_time_us,
            command.distance_m,
            command.bearing_deg,
            command.elevation_deg,
        )
    })
}

#[tauri::command]
pub fn send_hilink_sensor_frame(
    backend: State<'_, TelemetryBackend>,
    frame: HilSensorFrameCommand,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| bridge.send_hilink_sensor_frame(frame))
}

#[tauri::command]
pub fn send_hilink_sensor_frame_values(
    backend: State<'_, TelemetryBackend>,
    sim_tick: u64,
    sim_time_us: u64,
    valid_flags: u32,
    accel_mps2: [f32; 3],
    gyro_rps: [f32; 3],
    mag_ut: [f32; 3],
    pressure_pa: f32,
    baro_altitude_m: f32,
    temperature_c: f32,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    vel_ned_mps: [f32; 3],
    sats: u8,
    fix_type: u8,
    battery_voltage_v: f32,
    rssi_dbm: i16,
    snr_db_x100: i16,
    loss_pct_x100: u16,
) -> CommandResult<AppState> {
    mutate_and_snapshot(backend, |bridge| {
        bridge.send_hilink_sensor_frame_values(
            sim_tick,
            sim_time_us,
            valid_flags,
            accel_mps2,
            gyro_rps,
            mag_ut,
            pressure_pa,
            baro_altitude_m,
            temperature_c,
            lat_deg,
            lon_deg,
            alt_msl_m,
            vel_ned_mps,
            sats,
            fix_type,
            battery_voltage_v,
            rssi_dbm,
            snr_db_x100,
            loss_pct_x100,
        )
    })
}
