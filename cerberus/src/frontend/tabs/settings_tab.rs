use eframe::egui;

use crate::bridge::app_state::OperationMode;
use crate::bridge::{AppBridge, AppState};

const COMMON_BAUD_RATES: [u32; 8] = [
    9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800, 921_600,
];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GlobalTheme {
    Dark,
    Light,
}

impl GlobalTheme {
    pub fn apply(self, ctx: &egui::Context) {
        match self {
            Self::Dark => ctx.set_visuals(egui::Visuals::dark()),
            Self::Light => ctx.set_visuals(egui::Visuals::light()),
        }
    }
}

pub struct SettingsTabState {
    selected_serial_port: Option<String>,
    selected_baud_rate: u32,
}

impl SettingsTabState {
    pub fn new() -> Self {
        Self {
            selected_serial_port: None,
            selected_baud_rate: OperationMode::Hil.default_baud_rate(),
        }
    }
}

pub fn render(
    ui: &mut egui::Ui,
    state: &AppState,
    bridge: &mut AppBridge,
    theme: &mut GlobalTheme,
    settings: &mut SettingsTabState,
) {
    ui.heading("Settings");
    ui.label("Choose a global color theme for the full front end.");
    ui.add_space(10.0);

    let mut theme_changed = false;
    theme_changed |= ui.radio_value(theme, GlobalTheme::Dark, "Dark").changed();
    theme_changed |= ui.radio_value(theme, GlobalTheme::Light, "Light").changed();

    if theme_changed {
        theme.apply(ui.ctx());
    }

    ui.add_space(16.0);
    ui.separator();
    ui.add_space(8.0);

    render_operation_mode(ui, state, bridge, settings);
    ui.add_space(16.0);
    ui.separator();
    ui.add_space(8.0);

    render_serial_settings(ui, state, bridge, settings);
}

fn render_operation_mode(
    ui: &mut egui::Ui,
    state: &AppState,
    bridge: &mut AppBridge,
    settings: &mut SettingsTabState,
) {
    ui.label(egui::RichText::new("Operation").strong());
    ui.add_space(6.0);

    let mut selected_mode = state.operation_mode;
    ui.horizontal_wrapped(|ui| {
        ui.label("Mode");
        egui::ComboBox::from_id_salt("settings_operation_mode")
            .width(190.0)
            .selected_text(selected_mode.label())
            .show_ui(ui, |ui| {
                for mode in [
                    OperationMode::Hil,
                    OperationMode::FieldRadio,
                    OperationMode::FieldProgramming,
                ] {
                    ui.selectable_value(&mut selected_mode, mode, mode.label());
                }
            });

        if selected_mode != state.operation_mode {
            let baud_rate = selected_mode.default_baud_rate();
            bridge.set_operation_mode(selected_mode);
            settings.selected_baud_rate = baud_rate;
            if state.uart.connected {
                if let Err(error) = bridge.set_uart_baud_rate(baud_rate) {
                    eprintln!("failed to update UART baud rate: {error}");
                }
            }
            ui.ctx().request_repaint();
        }

        ui.separator();
        ui.label(selected_mode.link_label());
    });

    ui.add_space(4.0);
    ui.label(match selected_mode {
        OperationMode::Hil => {
            "HIL uses the Gazebo bridge for simulated sensors and a direct FC serial link."
        }
        OperationMode::FieldRadio => "Field operation uses the CP2102 GS radio link at 115200 8N1.",
        OperationMode::FieldProgramming => {
            "Field programming uses direct FC USB serial for higher throughput."
        }
    });
}

fn render_serial_settings(
    ui: &mut egui::Ui,
    state: &AppState,
    bridge: &mut AppBridge,
    settings: &mut SettingsTabState,
) {
    sync_selected_serial_port(settings, state);
    sync_selected_baud_rate(settings, state);

    ui.label(egui::RichText::new("Serial").strong());
    ui.add_space(6.0);

    ui.horizontal(|ui| {
        let button_label = if state.uart.connected {
            "Close"
        } else {
            "Open"
        };
        if ui.button(button_label).clicked() {
            if state.uart.connected {
                bridge.close_uart();
            } else {
                let port_name = settings.selected_serial_port.as_deref().unwrap_or_default();
                if let Err(error) =
                    bridge.open_uart_with_baud_rate(port_name, settings.selected_baud_rate)
                {
                    eprintln!("failed to open UART port: {error}");
                }
            }
            ui.ctx().request_repaint();
        }

        ui.add_space(12.0);

        ui.add_enabled_ui(!state.uart.connected, |ui| {
            let combo_response = egui::ComboBox::from_id_salt("settings_serial_port_selector")
                .width(220.0)
                .selected_text(selected_serial_port_label(settings, state))
                .show_ui(ui, |ui| {
                    if state.uart.available_ports.is_empty() {
                        ui.label("No UART ports found");
                    } else {
                        for port in &state.uart.available_ports {
                            ui.selectable_value(
                                &mut settings.selected_serial_port,
                                Some(port.port_name.clone()),
                                &port.display_name,
                            );
                        }
                    }
                });

            if combo_response.response.clicked() {
                bridge.list_uart_ports();
                ui.ctx().request_repaint();
            }

            ui.label("Baud");
            egui::ComboBox::from_id_salt("settings_baud_rate_selector")
                .width(110.0)
                .selected_text(settings.selected_baud_rate.to_string())
                .show_ui(ui, |ui| {
                    for baud_rate in COMMON_BAUD_RATES {
                        ui.selectable_value(
                            &mut settings.selected_baud_rate,
                            baud_rate,
                            baud_rate.to_string(),
                        );
                    }
                });
        });
    });

    ui.add_space(8.0);
    ui.label(format!(
        "UART status: {}",
        if state.uart.connected {
            format!(
                "{} @ {} {}",
                state.uart.selected_port.as_deref().unwrap_or("Open"),
                state.uart.baud_rate,
                state.uart.line_coding
            )
        } else {
            "Closed".to_string()
        }
    ));

    if let Some(error) = &state.uart.last_error {
        ui.colored_label(egui::Color32::LIGHT_RED, format!("UART error: {error}"));
    }
}

fn sync_selected_baud_rate(settings: &mut SettingsTabState, state: &AppState) {
    if state.uart.connected {
        settings.selected_baud_rate = state.uart.baud_rate;
    }
}

fn sync_selected_serial_port(settings: &mut SettingsTabState, state: &AppState) {
    if state.uart.connected {
        settings.selected_serial_port = state.uart.selected_port.clone();
        return;
    }

    let selected_is_available = settings
        .selected_serial_port
        .as_ref()
        .is_some_and(|selected| {
            state
                .uart
                .available_ports
                .iter()
                .any(|port| port.port_name == *selected)
        });

    if !selected_is_available {
        settings.selected_serial_port = state
            .uart
            .available_ports
            .first()
            .map(|port| port.port_name.clone());
    }
}

fn selected_serial_port_label(settings: &SettingsTabState, state: &AppState) -> String {
    if let Some(port_name) = &settings.selected_serial_port {
        if let Some(port) = state
            .uart
            .available_ports
            .iter()
            .find(|port| port.port_name == *port_name)
        {
            return port.display_name.clone();
        }
        return port_name.clone();
    }

    if state.uart.available_ports.is_empty() {
        "No UART ports found".to_string()
    } else {
        "Select UART port".to_string()
    }
}
