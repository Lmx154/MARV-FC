use eframe::egui;

use crate::bridge::{AppBridge, AppState};

pub struct HilTabState {
    pub endpoint_input: String,
    pub test_motor_speed: f32,
}

impl HilTabState {
    pub fn new() -> Self {
        Self {
            endpoint_input: "127.0.0.1:9000".to_string(),
            test_motor_speed: 1200.0,
        }
    }
}

pub fn render(
    ui: &mut egui::Ui,
    state: &AppState,
    hil_state: &mut HilTabState,
    bridge: &mut AppBridge,
) {
    render_contents(ui, state, hil_state, Some(bridge));
}

fn render_contents(
    ui: &mut egui::Ui,
    state: &AppState,
    hil_state: &mut HilTabState,
    bridge: Option<&mut AppBridge>,
) {
    ui.heading("HIL");
    ui.label("Connect to the Gazebo bridge and send a sample actuator command.");
    ui.add_space(12.0);

    ui.horizontal(|ui| {
        ui.label("Bridge endpoint:");
        ui.add_enabled(
            !state.gazebo_bridge.connected,
            egui::TextEdit::singleline(&mut hil_state.endpoint_input).desired_width(180.0),
        );
    });

    ui.add_space(10.0);

    ui.label("Test motor speed");
    ui.add(
        egui::Slider::new(&mut hil_state.test_motor_speed, 0.0..=2400.0)
            .text("velocity")
            .clamping(egui::SliderClamping::Always),
    );
    ui.label(format!(
        "Current test speed: {:.1}",
        hil_state.test_motor_speed
    ));

    ui.add_space(10.0);

    if let Some(bridge) = bridge {
        ui.horizontal(|ui| {
            let button_label = if state.gazebo_bridge.connected {
                "Disconnect"
            } else {
                "Connect"
            };

            if ui.button(button_label).clicked() {
                if state.gazebo_bridge.connected {
                    bridge.disconnect_gazebo_bridge();
                } else if let Err(error) = bridge.connect_gazebo_bridge(&hil_state.endpoint_input) {
                    eprintln!("failed to connect to gazebo bridge: {error}");
                }
            }

            let send_enabled = state.gazebo_bridge.connected;
            if ui
                .add_enabled(send_enabled, egui::Button::new("Send Test Data"))
                .clicked()
            {
                if let Err(error) = bridge.send_test_actuator_command(hil_state.test_motor_speed) {
                    eprintln!("failed to send actuator command: {error}");
                }
            }
        });
    }

    ui.add_space(16.0);
    ui.separator();
    ui.add_space(8.0);

    ui.label(format!(
        "Status: {}",
        if state.gazebo_bridge.connected {
            "Connected"
        } else {
            "Disconnected"
        }
    ));
    ui.label(format!(
        "Connected for: {}",
        state
            .gazebo_bridge
            .connected_for_secs
            .map(|secs| format!("{secs}s"))
            .unwrap_or_else(|| "--".to_string())
    ));
    ui.label(format!(
        "Connection attempts: {}",
        state.gazebo_bridge.connection_attempts
    ));
    ui.label(format!(
        "Actuator frames sent: {}",
        state.gazebo_bridge.actuator_frames_sent
    ));
    ui.label("Test preset range: 0 to 2400 motor speed on all four motors");
    ui.label(format!(
        "Sensor frames received: {}",
        state.gazebo_bridge.sensor_frames_received
    ));
    ui.label(format!(
        "Last sensor seq: {}",
        state
            .gazebo_bridge
            .last_sensor_sequence
            .map(|seq| seq.to_string())
            .unwrap_or_else(|| "--".to_string())
    ));
    ui.label(format!(
        "Last sensor sim time: {}",
        state
            .gazebo_bridge
            .last_sensor_time_us
            .map(|time| format!("{time} us"))
            .unwrap_or_else(|| "--".to_string())
    ));
    ui.label(format!(
        "Last sensor clock: {}",
        state
            .gazebo_bridge
            .last_sensor_clock_source
            .as_deref()
            .unwrap_or("--")
    ));

    if let Some(error) = &state.gazebo_bridge.last_error {
        ui.colored_label(egui::Color32::LIGHT_RED, format!("Last error: {error}"));
    } else {
        ui.label("Last error: --");
    }
}
