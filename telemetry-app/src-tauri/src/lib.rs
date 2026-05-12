use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use bridge::AppBridge;

mod backend;
mod bridge;
mod commands;

#[derive(Clone)]
pub struct TelemetryBackend {
    bridge: Arc<Mutex<AppBridge>>,
}

impl TelemetryBackend {
    fn new() -> Self {
        Self {
            bridge: Arc::new(Mutex::new(AppBridge::new())),
        }
    }

    fn start_tick_loop(&self) {
        let bridge = Arc::clone(&self.bridge);
        thread::spawn(move || loop {
            if let Ok(mut bridge) = bridge.lock() {
                bridge.tick();
            }
            thread::sleep(Duration::from_millis(20));
        });
    }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let backend = TelemetryBackend::new();
    backend.start_tick_loop();

    tauri::Builder::default()
        .manage(backend)
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![
            commands::app::backend_snapshot,
            commands::app::backend_tick,
            commands::uart::list_uart_ports,
            commands::uart::open_uart,
            commands::uart::close_uart,
            commands::uart::set_uart_baud_rate,
            commands::gazebo::start_gazebo_bridge_process,
            commands::gazebo::stop_gazebo_bridge_process,
            commands::gazebo::connect_gazebo_bridge,
            commands::gazebo::disconnect_gazebo_bridge,
            commands::gazebo::send_test_actuator_command,
            commands::gazebo::send_gazebo_sim_control_command,
            commands::hilink::send_hilink_ping,
            commands::hilink::send_hilink_arm,
            commands::hilink::send_hilink_disarm,
            commands::hilink::send_hilink_rtl,
            commands::hilink::send_hilink_bench_enable,
            commands::hilink::send_hilink_bench_disable,
            commands::hilink::send_hilink_motor_test,
            commands::hilink::send_hilink_motor_test_values,
            commands::hilink::send_hilink_motor_sweep,
            commands::hilink::send_hilink_motor_sweep_values,
            commands::hilink::send_hilink_motor_stop,
            commands::hilink::run_radio_link_smoke_test,
            commands::hilink::send_hilink_dshot_command,
            commands::hilink::send_hilink_actuator_status_request,
            commands::hilink::send_hilink_mixer_motor_order,
            commands::hilink::send_hilink_control_waypoint,
            commands::hilink::send_hilink_mission_waypoint,
            commands::hilink::send_hilink_cv_waypoint,
            commands::hilink::send_hilink_tof_waypoint,
            commands::hilink::send_hilink_sensor_frame,
            commands::hilink::send_hilink_sensor_frame_values,
            commands::serial::clear_serial_monitor,
            commands::serial::set_serial_monitor_parse_enabled,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
