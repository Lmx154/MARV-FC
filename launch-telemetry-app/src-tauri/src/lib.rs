mod aggregator;
mod commands;
mod decode;
mod framing;
mod model;
mod serial;
mod state;

#[cfg(test)]
mod pipeline_tests;

use std::sync::mpsc;

use tauri::Manager;

use state::AppState;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .setup(|app| {
            // Spawn the serial worker thread and hand the UI a channel into it.
            let (tx, rx) = mpsc::channel();
            let handle = app.handle().clone();
            std::thread::spawn(move || serial::run_worker(handle, rx));
            app.manage(AppState { tx });
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            commands::list_serial_ports,
            commands::connect,
            commands::disconnect,
            commands::send_command,
            commands::send_radio_profile,
            commands::send_idle_fallback,
        ])
        .run(tauri::generate_context!())
        .expect("error while running launch telemetry application");
}
