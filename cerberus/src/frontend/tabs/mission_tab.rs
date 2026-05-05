use eframe::egui;

use crate::bridge::{AppBridge, AppState};

const DISPLAY_SELECTOR_COUNT: usize = 3;
const SELECTOR_COLUMN_WIDTH: f32 = 240.0;
const CENTER_FRAME_WIDTH: f32 = 560.0;
const CENTER_FRAME_HEIGHT: f32 = 320.0;
const DISPLAY_GAP: f32 = 12.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum CenterFrameMode {
    TelemetryList,
    Visual3d,
}

struct TelemetryField {
    group: String,
    parameter: String,
    unit: String,
    value: String,
}

struct MissionCommandForm {
    control_waypoint: GlobalWaypointForm,
    mission_waypoint: GlobalWaypointForm,
    cv_waypoint: CvWaypointForm,
    tof_waypoint: TofWaypointForm,
    sensor_frame: HilSensorFrameForm,
}

impl MissionCommandForm {
    fn new() -> Self {
        Self {
            control_waypoint: GlobalWaypointForm::default(),
            mission_waypoint: GlobalWaypointForm::default(),
            cv_waypoint: CvWaypointForm::default(),
            tof_waypoint: TofWaypointForm::default(),
            sensor_frame: HilSensorFrameForm::default(),
        }
    }
}

#[derive(Clone)]
struct GlobalWaypointForm {
    ref_sim_tick: u64,
    ref_sim_time_us: u64,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    yaw_deg: f32,
}

impl Default for GlobalWaypointForm {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            lat_deg: 30.2672,
            lon_deg: -97.7431,
            alt_msl_m: 171.0,
            yaw_deg: 0.0,
        }
    }
}

#[derive(Clone)]
struct CvWaypointForm {
    ref_sim_tick: u64,
    ref_sim_time_us: u64,
    dir_body: [f32; 3],
    confidence: f32,
}

impl Default for CvWaypointForm {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            dir_body: [1.0, 0.0, 0.0],
            confidence: 1.0,
        }
    }
}

#[derive(Clone)]
struct TofWaypointForm {
    ref_sim_tick: u64,
    ref_sim_time_us: u64,
    distance_m: f32,
    bearing_deg: f32,
    elevation_deg: f32,
}

impl Default for TofWaypointForm {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            distance_m: 1.0,
            bearing_deg: 0.0,
            elevation_deg: 0.0,
        }
    }
}

#[derive(Clone)]
struct HilSensorFrameForm {
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
}

impl Default for HilSensorFrameForm {
    fn default() -> Self {
        Self {
            sim_tick: 1,
            sim_time_us: 20_000,
            valid_flags: 0x7F,
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.0, 0.0, 0.0],
            mag_ut: [0.0, 0.0, 0.0],
            pressure_pa: 101_325.0,
            baro_altitude_m: 171.0,
            temperature_c: 25.0,
            lat_deg: 30.2672,
            lon_deg: -97.7431,
            alt_msl_m: 171.0,
            vel_ned_mps: [0.0, 0.0, 0.0],
            sats: 12,
            fix_type: 3,
            battery_voltage_v: 16.2,
            rssi_dbm: -48,
            snr_db_x100: 1150,
            loss_pct_x100: 0,
        }
    }
}

pub struct MissionTab {
    telemetry_fields: Vec<TelemetryField>,
    selector_indices: [[usize; DISPLAY_SELECTOR_COUNT]; 2],
    center_frame_mode: CenterFrameMode,
    command_form: MissionCommandForm,
    command_status: Option<String>,
}

impl MissionTab {
    pub fn new() -> Self {
        Self {
            telemetry_fields: fallback_telemetry_fields(),
            selector_indices: [[0, 1, 2], [3, 4, 5]],
            center_frame_mode: CenterFrameMode::TelemetryList,
            command_form: MissionCommandForm::new(),
            command_status: None,
        }
    }

    pub fn render(&mut self, ui: &mut egui::Ui, state: &AppState, bridge: &mut AppBridge) {
        self.telemetry_fields = telemetry_fields_from_state(state);
        self.clamp_selector_indices();

        ui.heading("Mission");
        ui.label("Live HILink UART telemetry workspace.");
        ui.add_space(8.0);

        self.render_protocol_status(ui, state);
        ui.add_space(8.0);

        self.render_primary_display(ui);
        ui.add_space(10.0);

        self.render_serial_monitor(ui, state, bridge);
        ui.add_space(10.0);

        self.render_command_section(ui, state, bridge);
        ui.add_space(10.0);

        egui::CollapsingHeader::new("Telemetry Fields")
            .id_salt("mission_telemetry_fields")
            .default_open(false)
            .show(ui, |ui| {
                for (index, group) in self.group_names().into_iter().enumerate() {
                    if index > 0 {
                        ui.add_space(8.0);
                    }
                    self.render_group(ui, &group);
                }
            });
    }

    fn render_protocol_status(&self, ui: &mut egui::Ui, state: &AppState) {
        ui.horizontal_wrapped(|ui| {
            ui.label(format!(
                "UART: {}",
                if state.uart.connected {
                    format!(
                        "{} @ {} {}",
                        state.uart.selected_port.as_deref().unwrap_or("open"),
                        state.uart.baud_rate,
                        state.uart.line_coding
                    )
                } else {
                    "closed".to_string()
                }
            ));
            ui.separator();
            ui.label(format!("HILink RX: {}", state.hilink.rx_frames));
            ui.separator();
            ui.label(format!("TX: {}", state.hilink.tx_frames));
            ui.separator();
            ui.label(format!("Parse errors: {}", state.hilink.parse_errors));
            if let Some(last_message) = &state.hilink.last_message {
                ui.separator();
                ui.label(format!("Last: {last_message}"));
            }
        });

        if let Some(error) = &state.hilink.last_error {
            ui.colored_label(egui::Color32::LIGHT_RED, format!("HILink parser: {error}"));
        }
    }

    fn render_serial_monitor(
        &mut self,
        ui: &mut egui::Ui,
        state: &AppState,
        bridge: &mut AppBridge,
    ) {
        egui::CollapsingHeader::new("Serial Monitor")
            .id_salt("mission_serial_monitor")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    let mut parse_enabled = state.serial_monitor.parse_enabled;

                    ui.label("Parse");
                    if live_parse_switch(ui, &mut parse_enabled).changed() {
                        bridge.set_serial_monitor_parse_enabled(parse_enabled);
                    }

                    ui.separator();
                    ui.label(if state.serial_monitor.parse_enabled {
                        format!("{} parsed", state.serial_monitor.parsed_lines.len())
                    } else {
                        format!("{} raw lines", state.serial_monitor.lines.len())
                    });
                    ui.separator();
                    ui.label(if state.serial_monitor.parse_enabled {
                        format!("{} dropped", state.serial_monitor.dropped_parsed_lines)
                    } else {
                        format!("{} dropped", state.serial_monitor.dropped_lines)
                    });
                    ui.separator();
                    ui.label(if state.serial_monitor.parse_enabled {
                        "Live parsed HILink frames"
                    } else {
                        "RX/TX raw UART bytes"
                    });
                    if ui.button("Clear").clicked() {
                        bridge.clear_serial_monitor();
                    }
                });

                egui::Frame::canvas(ui.style()).show(ui, |ui| {
                    ui.set_min_size(egui::vec2(ui.available_width(), 180.0));
                    egui::ScrollArea::vertical()
                        .id_salt("mission_serial_monitor_scroll")
                        .stick_to_bottom(true)
                        .max_height(240.0)
                        .show(ui, |ui| {
                            if state.serial_monitor.parse_enabled {
                                if state.serial_monitor.parsed_lines.is_empty() {
                                    ui.monospace("Waiting for complete HILink frames...");
                                } else {
                                    for line in &state.serial_monitor.parsed_lines {
                                        ui.monospace(line);
                                    }
                                }
                            } else if state.serial_monitor.lines.is_empty() {
                                ui.monospace("Waiting for UART bytes...");
                            } else {
                                for line in &state.serial_monitor.lines {
                                    ui.monospace(line);
                                }
                            }
                        });
                });
            });
    }

    fn render_command_section(
        &mut self,
        ui: &mut egui::Ui,
        state: &AppState,
        bridge: &mut AppBridge,
    ) {
        egui::CollapsingHeader::new("Commands")
            .id_salt("mission_command_section")
            .default_open(true)
            .show(ui, |ui| {
                let commands_enabled = state.uart.connected;

                if !commands_enabled {
                    ui.colored_label(
                        egui::Color32::LIGHT_RED,
                        "Open a UART port to send commands.",
                    );
                }

                ui.add_enabled_ui(commands_enabled, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("Ping").clicked() {
                            self.command_status =
                                Some(command_status("Ping", bridge.send_hilink_ping()));
                        }
                        if ui.button("Arm").clicked() {
                            self.command_status =
                                Some(command_status("Arm", bridge.send_hilink_arm()));
                        }
                        if ui.button("Disarm").clicked() {
                            self.command_status =
                                Some(command_status("Disarm", bridge.send_hilink_disarm()));
                        }
                        if ui.button("MotorStop").clicked() {
                            self.command_status =
                                Some(command_status("MotorStop", bridge.send_hilink_motor_stop()));
                        }
                        if ui.button("RTL").clicked() {
                            self.command_status =
                                Some(command_status("RTL", bridge.send_hilink_rtl()));
                        }
                        if ui.button("Radio Link Smoke Test").clicked() {
                            self.command_status = Some(command_status(
                                "Ping + Arm + Disarm + MotorStop",
                                bridge.run_radio_link_smoke_test(),
                            ));
                        }
                    });

                    self.render_command_responses(ui, state);

                    ui.add_space(8.0);

                    egui::CollapsingHeader::new("Control Waypoint")
                        .id_salt("mission_control_waypoint_command")
                        .default_open(false)
                        .show(ui, |ui| {
                            render_global_waypoint_form(
                                ui,
                                &mut self.command_form.control_waypoint,
                            );
                            if ui.button("Send ControlWaypoint").clicked() {
                                let form = self.command_form.control_waypoint.clone();
                                self.command_status = Some(command_status(
                                    "ControlWaypoint",
                                    bridge.send_hilink_control_waypoint(
                                        form.ref_sim_tick,
                                        form.ref_sim_time_us,
                                        form.lat_deg,
                                        form.lon_deg,
                                        form.alt_msl_m,
                                        form.yaw_deg,
                                    ),
                                ));
                            }
                        });

                    egui::CollapsingHeader::new("Mission Waypoint")
                        .id_salt("mission_mission_waypoint_command")
                        .default_open(false)
                        .show(ui, |ui| {
                            render_global_waypoint_form(
                                ui,
                                &mut self.command_form.mission_waypoint,
                            );
                            if ui.button("Send MissionWaypoint").clicked() {
                                let form = self.command_form.mission_waypoint.clone();
                                self.command_status = Some(command_status(
                                    "MissionWaypoint",
                                    bridge.send_hilink_mission_waypoint(
                                        form.ref_sim_tick,
                                        form.ref_sim_time_us,
                                        form.lat_deg,
                                        form.lon_deg,
                                        form.alt_msl_m,
                                        form.yaw_deg,
                                    ),
                                ));
                            }
                        });

                    egui::CollapsingHeader::new("CV Waypoint")
                        .id_salt("mission_cv_waypoint_command")
                        .default_open(false)
                        .show(ui, |ui| {
                            render_cv_waypoint_form(ui, &mut self.command_form.cv_waypoint);
                            if ui.button("Send CvWaypoint").clicked() {
                                let form = self.command_form.cv_waypoint.clone();
                                self.command_status = Some(command_status(
                                    "CvWaypoint",
                                    bridge.send_hilink_cv_waypoint(
                                        form.ref_sim_tick,
                                        form.ref_sim_time_us,
                                        form.dir_body,
                                        form.confidence,
                                    ),
                                ));
                            }
                        });

                    egui::CollapsingHeader::new("ToF Waypoint")
                        .id_salt("mission_tof_waypoint_command")
                        .default_open(false)
                        .show(ui, |ui| {
                            render_tof_waypoint_form(ui, &mut self.command_form.tof_waypoint);
                            if ui.button("Send TofWaypoint").clicked() {
                                let form = self.command_form.tof_waypoint.clone();
                                self.command_status = Some(command_status(
                                    "TofWaypoint",
                                    bridge.send_hilink_tof_waypoint(
                                        form.ref_sim_tick,
                                        form.ref_sim_time_us,
                                        form.distance_m,
                                        form.bearing_deg,
                                        form.elevation_deg,
                                    ),
                                ));
                            }
                        });

                    egui::CollapsingHeader::new("HIL Sensor Frame")
                        .id_salt("mission_hil_sensor_frame_command")
                        .default_open(false)
                        .show(ui, |ui| {
                            render_sensor_frame_form(ui, &mut self.command_form.sensor_frame);
                            if ui.button("Send HilSensorFrame").clicked() {
                                let form = self.command_form.sensor_frame.clone();
                                self.command_status = Some(command_status(
                                    "HilSensorFrame",
                                    bridge.send_hilink_sensor_frame_values(
                                        form.sim_tick,
                                        form.sim_time_us,
                                        form.valid_flags,
                                        form.accel_mps2,
                                        form.gyro_rps,
                                        form.mag_ut,
                                        form.pressure_pa,
                                        form.baro_altitude_m,
                                        form.temperature_c,
                                        form.lat_deg,
                                        form.lon_deg,
                                        form.alt_msl_m,
                                        form.vel_ned_mps,
                                        form.sats,
                                        form.fix_type,
                                        form.battery_voltage_v,
                                        form.rssi_dbm,
                                        form.snr_db_x100,
                                        form.loss_pct_x100,
                                    ),
                                ));
                            }
                        });
                });

                if let Some(status) = &self.command_status {
                    ui.add_space(6.0);
                    ui.label(status);
                }
            });
    }

    fn render_command_responses(&self, ui: &mut egui::Ui, state: &AppState) {
        ui.add_space(8.0);
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("Pong: {}", state.hilink_commands.pong_count));
            ui.separator();
            ui.label(format!("Ack: {}", state.hilink_commands.ack_count));
            ui.separator();
            ui.label(format!("Nack: {}", state.hilink_commands.nack_count));
            ui.separator();
            ui.label(format!("Gps: {}", state.hilink_commands.gps_count));
            ui.separator();
            ui.label(format!(
                "TelemetrySnapshot: {}",
                state.hilink_commands.telemetry_snapshot_count
            ));
        });

        if let Some(event) = &state.hilink_commands.last_event {
            ui.label(format!("Last response: {event}"));
        }

        if !state.hilink_commands.pending.is_empty() {
            egui::Grid::new("mission_pending_commands")
                .num_columns(5)
                .striped(true)
                .show(ui, |ui| {
                    ui.strong("Seq");
                    ui.strong("Type");
                    ui.strong("Command");
                    ui.strong("Sent");
                    ui.strong("Status");
                    ui.end_row();

                    for command in state.hilink_commands.pending.iter().rev().take(8) {
                        ui.monospace(command.seq.to_string());
                        ui.monospace(command.msg_type.to_string());
                        ui.label(&command.label);
                        ui.label(format!("+{}ms", command.sent_elapsed_ms));
                        ui.label(command.status.label());
                        ui.end_row();
                    }
                });
        }
    }

    fn render_primary_display(&mut self, ui: &mut egui::Ui) {
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new("Mission Display").strong());
            ui.add_space(4.0);

            let total_width =
                (SELECTOR_COLUMN_WIDTH * 2.0) + CENTER_FRAME_WIDTH + (DISPLAY_GAP * 2.0);

            ui.horizontal(|ui| {
                let lead_space = (ui.available_width() - total_width).max(0.0) * 0.5;
                if lead_space > 0.0 {
                    ui.add_space(lead_space);
                }

                ui.allocate_ui_with_layout(
                    egui::vec2(SELECTOR_COLUMN_WIDTH, CENTER_FRAME_HEIGHT),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| self.render_selector_column(ui, 0, "Left"),
                );
                ui.add_space(DISPLAY_GAP);

                ui.allocate_ui_with_layout(
                    egui::vec2(CENTER_FRAME_WIDTH, CENTER_FRAME_HEIGHT),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| self.render_center_frame(ui),
                );
                ui.add_space(DISPLAY_GAP);

                ui.allocate_ui_with_layout(
                    egui::vec2(SELECTOR_COLUMN_WIDTH, CENTER_FRAME_HEIGHT),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| self.render_selector_column(ui, 1, "Right"),
                );
            });
        });
    }

    fn render_selector_column(&mut self, ui: &mut egui::Ui, side_index: usize, side_name: &str) {
        let field_options: Vec<(usize, String)> = self
            .telemetry_fields
            .iter()
            .enumerate()
            .map(|(idx, field)| (idx, Self::field_row_label(field)))
            .collect();

        ui.vertical(|ui| {
            ui.label(egui::RichText::new(side_name).strong());

            for slot in 0..DISPLAY_SELECTOR_COUNT {
                let selected_idx = self.selector_indices[side_index][slot];
                let selected_text = field_options
                    .iter()
                    .find(|(idx, _)| *idx == selected_idx)
                    .map(|(_, label)| label.clone())
                    .unwrap_or_else(|| "Select telemetry".to_string());
                let selector_idx = &mut self.selector_indices[side_index][slot];

                egui::ComboBox::from_id_salt(format!("mission_selector_{side_index}_{slot}"))
                    .width(SELECTOR_COLUMN_WIDTH - 10.0)
                    .selected_text(selected_text)
                    .show_ui(ui, |ui| {
                        for (field_index, option) in &field_options {
                            ui.selectable_value(selector_idx, *field_index, option);
                        }
                    });
            }
        });
    }

    fn render_center_frame(&mut self, ui: &mut egui::Ui) {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_min_size(egui::vec2(
                CENTER_FRAME_WIDTH - 16.0,
                CENTER_FRAME_HEIGHT - 16.0,
            ));
            ui.label(egui::RichText::new("Center Frame").strong());
            ui.separator();

            match self.center_frame_mode {
                CenterFrameMode::TelemetryList => self.render_long_telemetry_list(ui),
                CenterFrameMode::Visual3d => self.render_visual_placeholder(ui),
            }

            ui.separator();
            ui.horizontal(|ui| {
                ui.label("View");
                egui::ComboBox::from_id_salt("mission_center_frame_mode")
                    .width(220.0)
                    .selected_text(self.center_frame_mode.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.center_frame_mode,
                            CenterFrameMode::TelemetryList,
                            CenterFrameMode::TelemetryList.label(),
                        );
                        ui.selectable_value(
                            &mut self.center_frame_mode,
                            CenterFrameMode::Visual3d,
                            CenterFrameMode::Visual3d.label(),
                        );
                    });
            });
        });
    }

    fn render_long_telemetry_list(&self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical()
            .id_salt("mission_center_long_telemetry")
            .max_height(250.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.add_sized(
                        [280.0, 0.0],
                        egui::Label::new(egui::RichText::new("Parameter").strong()),
                    );
                    ui.add_sized(
                        [90.0, 0.0],
                        egui::Label::new(egui::RichText::new("Value").strong()),
                    );
                    ui.add_sized(
                        [80.0, 0.0],
                        egui::Label::new(egui::RichText::new("Unit").strong()),
                    );
                });
                ui.separator();

                for field in &self.telemetry_fields {
                    ui.horizontal(|ui| {
                        ui.add_sized(
                            [280.0, 0.0],
                            egui::Label::new(egui::RichText::new(&field.parameter).monospace()),
                        );
                        ui.add_sized(
                            [90.0, 0.0],
                            egui::Label::new(egui::RichText::new(&field.value).monospace().weak()),
                        );
                        ui.add_sized(
                            [80.0, 0.0],
                            egui::Label::new(egui::RichText::new(&field.unit).monospace()),
                        );
                    });
                }
            });
    }

    fn render_visual_placeholder(&self, ui: &mut egui::Ui) {
        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            ui.set_min_size(egui::vec2(540.0, 250.0));
            ui.centered_and_justified(|ui| {
                ui.label(
                    egui::RichText::new("3D visual placeholder")
                        .heading()
                        .italics(),
                );
            });
        });
    }

    fn field_row_label(field: &TelemetryField) -> String {
        format!("{} | {} | {}", field.parameter, field.value, field.unit)
    }

    fn render_group(&self, ui: &mut egui::Ui, group: &str) {
        ui.label(egui::RichText::new(group).strong());

        egui::Grid::new(format!("mission_telemetry_grid_{group}"))
            .num_columns(3)
            .striped(true)
            .show(ui, |ui| {
                ui.strong("Parameter");
                ui.strong("Value");
                ui.strong("Unit");
                ui.end_row();

                for field in self
                    .telemetry_fields
                    .iter()
                    .filter(|field| field.group == group)
                {
                    ui.monospace(&field.parameter);
                    ui.label(egui::RichText::new(&field.value).monospace().weak());
                    ui.monospace(&field.unit);
                    ui.end_row();
                }
            });
    }

    fn group_names(&self) -> Vec<String> {
        let mut groups = Vec::new();
        for field in &self.telemetry_fields {
            if !groups.iter().any(|group| group == &field.group) {
                groups.push(field.group.clone());
            }
        }
        groups
    }

    fn clamp_selector_indices(&mut self) {
        let max_index = self.telemetry_fields.len().saturating_sub(1);
        for side in &mut self.selector_indices {
            for index in side {
                *index = (*index).min(max_index);
            }
        }
    }
}

impl CenterFrameMode {
    fn label(self) -> &'static str {
        match self {
            CenterFrameMode::TelemetryList => "Long Telemetry List",
            CenterFrameMode::Visual3d => "3D Visual",
        }
    }
}

fn telemetry_fields_from_state(state: &AppState) -> Vec<TelemetryField> {
    let mut fields = fallback_telemetry_fields();

    for live_field in &state.hilink.fields {
        if let Some(field) = fields.iter_mut().find(|field| {
            field.group == live_field.group && field.parameter == live_field.parameter
        }) {
            field.value = live_field.value.clone();
            field.unit = live_field.unit.to_string();
        } else {
            fields.push(TelemetryField {
                group: live_field.group.to_string(),
                parameter: live_field.parameter.clone(),
                value: live_field.value.clone(),
                unit: live_field.unit.to_string(),
            });
        }
    }

    fields
}

fn command_status(label: &str, result: Result<(), String>) -> String {
    match result {
        Ok(()) => format!("Sent {label}"),
        Err(error) => format!("Failed to send {label}: {error}"),
    }
}

fn live_parse_switch(ui: &mut egui::Ui, enabled: &mut bool) -> egui::Response {
    let desired_size = egui::vec2(42.0, 22.0);
    let (rect, mut response) = ui.allocate_exact_size(desired_size, egui::Sense::click());

    if response.clicked() {
        *enabled = !*enabled;
        response.mark_changed();
    }

    if ui.is_rect_visible(rect) {
        let visuals = ui.style().interact_selectable(&response, *enabled);
        let radius = rect.height() * 0.5;
        let knob_radius = radius - 3.0;
        let knob_x = if *enabled {
            rect.right() - radius
        } else {
            rect.left() + radius
        };
        let fill = if *enabled {
            ui.visuals().selection.bg_fill
        } else {
            visuals.bg_fill
        };

        ui.painter().rect(
            rect,
            radius,
            fill,
            visuals.bg_stroke,
            egui::StrokeKind::Inside,
        );
        ui.painter().circle_filled(
            egui::pos2(knob_x, rect.center().y),
            knob_radius,
            visuals.fg_stroke.color,
        );
    }

    response.on_hover_text("Enable live HILink parsing")
}

fn render_global_waypoint_form(ui: &mut egui::Ui, form: &mut GlobalWaypointForm) {
    egui::Grid::new(ui.next_auto_id())
        .num_columns(2)
        .show(ui, |ui| {
            drag_u64(ui, "ref_sim_tick", &mut form.ref_sim_tick);
            drag_u64(ui, "ref_sim_time_us", &mut form.ref_sim_time_us);
            drag_f64(ui, "lat_deg", &mut form.lat_deg, 0.000001);
            drag_f64(ui, "lon_deg", &mut form.lon_deg, 0.000001);
            drag_f32(ui, "alt_msl_m", &mut form.alt_msl_m, 0.1);
            drag_f32(ui, "yaw_deg", &mut form.yaw_deg, 1.0);
        });
}

fn render_cv_waypoint_form(ui: &mut egui::Ui, form: &mut CvWaypointForm) {
    egui::Grid::new(ui.next_auto_id())
        .num_columns(2)
        .show(ui, |ui| {
            drag_u64(ui, "ref_sim_tick", &mut form.ref_sim_tick);
            drag_u64(ui, "ref_sim_time_us", &mut form.ref_sim_time_us);
            drag_vec3(ui, "dir_body", &mut form.dir_body, 0.01);
            drag_f32(ui, "confidence", &mut form.confidence, 0.01);
        });
}

fn render_tof_waypoint_form(ui: &mut egui::Ui, form: &mut TofWaypointForm) {
    egui::Grid::new(ui.next_auto_id())
        .num_columns(2)
        .show(ui, |ui| {
            drag_u64(ui, "ref_sim_tick", &mut form.ref_sim_tick);
            drag_u64(ui, "ref_sim_time_us", &mut form.ref_sim_time_us);
            drag_f32(ui, "distance_m", &mut form.distance_m, 0.1);
            drag_f32(ui, "bearing_deg", &mut form.bearing_deg, 1.0);
            drag_f32(ui, "elevation_deg", &mut form.elevation_deg, 1.0);
        });
}

fn render_sensor_frame_form(ui: &mut egui::Ui, form: &mut HilSensorFrameForm) {
    ui.horizontal_wrapped(|ui| {
        valid_flag_checkbox(ui, "ACCEL", &mut form.valid_flags, 1 << 0);
        valid_flag_checkbox(ui, "GYRO", &mut form.valid_flags, 1 << 1);
        valid_flag_checkbox(ui, "MAG", &mut form.valid_flags, 1 << 2);
        valid_flag_checkbox(ui, "BARO", &mut form.valid_flags, 1 << 3);
        valid_flag_checkbox(ui, "GPS", &mut form.valid_flags, 1 << 4);
        valid_flag_checkbox(ui, "BATTERY", &mut form.valid_flags, 1 << 5);
        valid_flag_checkbox(ui, "RADIO", &mut form.valid_flags, 1 << 6);
    });

    egui::Grid::new(ui.next_auto_id())
        .num_columns(2)
        .show(ui, |ui| {
            drag_u64(ui, "sim_tick", &mut form.sim_tick);
            drag_u64(ui, "sim_time_us", &mut form.sim_time_us);
            drag_vec3(ui, "accel_mps2", &mut form.accel_mps2, 0.01);
            drag_vec3(ui, "gyro_rps", &mut form.gyro_rps, 0.01);
            drag_vec3(ui, "mag_ut", &mut form.mag_ut, 0.1);
            drag_f32(ui, "pressure_pa", &mut form.pressure_pa, 1.0);
            drag_f32(ui, "baro_altitude_m", &mut form.baro_altitude_m, 0.1);
            drag_f32(ui, "temperature_c", &mut form.temperature_c, 0.1);
            drag_f64(ui, "lat_deg", &mut form.lat_deg, 0.000001);
            drag_f64(ui, "lon_deg", &mut form.lon_deg, 0.000001);
            drag_f32(ui, "alt_msl_m", &mut form.alt_msl_m, 0.1);
            drag_vec3(ui, "vel_ned_mps", &mut form.vel_ned_mps, 0.01);
            drag_u8(ui, "sats", &mut form.sats);
            drag_u8(ui, "fix_type", &mut form.fix_type);
            drag_f32(ui, "battery_voltage_v", &mut form.battery_voltage_v, 0.1);
            drag_i16(ui, "rssi_dbm", &mut form.rssi_dbm);
            drag_i16(ui, "snr_db_x100", &mut form.snr_db_x100);
            drag_u16(ui, "loss_pct_x100", &mut form.loss_pct_x100);
        });
}

fn valid_flag_checkbox(ui: &mut egui::Ui, label: &str, flags: &mut u32, bit: u32) {
    let mut enabled = (*flags & bit) != 0;
    if ui.checkbox(&mut enabled, label).changed() {
        if enabled {
            *flags |= bit;
        } else {
            *flags &= !bit;
        }
    }
}

fn drag_u64(ui: &mut egui::Ui, label: &str, value: &mut u64) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(1));
    ui.end_row();
}

fn drag_u16(ui: &mut egui::Ui, label: &str, value: &mut u16) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(1));
    ui.end_row();
}

fn drag_u8(ui: &mut egui::Ui, label: &str, value: &mut u8) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(1));
    ui.end_row();
}

fn drag_i16(ui: &mut egui::Ui, label: &str, value: &mut i16) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(1));
    ui.end_row();
}

fn drag_f32(ui: &mut egui::Ui, label: &str, value: &mut f32, speed: f64) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(speed));
    ui.end_row();
}

fn drag_f64(ui: &mut egui::Ui, label: &str, value: &mut f64, speed: f64) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(speed));
    ui.end_row();
}

fn drag_vec3(ui: &mut egui::Ui, label: &str, values: &mut [f32; 3], speed: f64) {
    ui.label(label);
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut values[0]).speed(speed));
        ui.add(egui::DragValue::new(&mut values[1]).speed(speed));
        ui.add(egui::DragValue::new(&mut values[2]).speed(speed));
    });
    ui.end_row();
}

fn fallback_telemetry_fields() -> Vec<TelemetryField> {
    [
        ("Protocol", "LAST_MSG_TYPE", ""),
        ("Protocol", "LAST_SEQ", ""),
        ("Timing", "SIM_TICK", "tick"),
        ("Timing", "SIM_TIME", "us"),
        ("Sensors", "IMU_ACCEL_X", "m/s^2"),
        ("Sensors", "IMU_ACCEL_Y", "m/s^2"),
        ("Sensors", "IMU_ACCEL_Z", "m/s^2"),
        ("Sensors", "IMU_GYRO_X", "rad/s"),
        ("Sensors", "IMU_GYRO_Y", "rad/s"),
        ("Sensors", "IMU_GYRO_Z", "rad/s"),
        ("Sensors", "MAG_FIELD_X", "uT"),
        ("Sensors", "MAG_FIELD_Y", "uT"),
        ("Sensors", "MAG_FIELD_Z", "uT"),
        ("Sensors", "BARO_PRESSURE", "Pa"),
        ("Sensors", "BARO_ALTITUDE", "m"),
        ("Sensors", "TEMPERATURE", "C"),
        ("Navigation", "GPS_LAT", "deg"),
        ("Navigation", "GPS_LON", "deg"),
        ("Navigation", "GPS_ALT_MSL", "m"),
        ("Navigation", "GPS_VEL_NED_X", "m/s"),
        ("Navigation", "GPS_VEL_NED_Y", "m/s"),
        ("Navigation", "GPS_VEL_NED_Z", "m/s"),
        ("Navigation", "POSITION_NED_X", "m"),
        ("Navigation", "POSITION_NED_Y", "m"),
        ("Navigation", "POSITION_NED_Z", "m"),
        ("Navigation", "ATTITUDE_QUAT_W", ""),
        ("Navigation", "ATTITUDE_QUAT_X", ""),
        ("Navigation", "ATTITUDE_QUAT_Y", ""),
        ("Navigation", "ATTITUDE_QUAT_Z", ""),
        ("System", "SYS_STATE", "state"),
        ("System", "RESPONSE_FLAGS", ""),
        ("System", "BAT_VOLTAGE", "V"),
        ("System", "RADIO_RSSI", "dBm"),
        ("System", "RADIO_SNR", "dB"),
        ("System", "RADIO_LOSS", "%"),
        ("Actuators", "MOTOR_CMD_1", "normalized"),
        ("Actuators", "MOTOR_CMD_2", "normalized"),
        ("Actuators", "MOTOR_CMD_3", "normalized"),
        ("Actuators", "MOTOR_CMD_4", "normalized"),
        ("Commands", "WAYPOINT_LAT", "deg"),
        ("Commands", "WAYPOINT_LON", "deg"),
        ("Commands", "WAYPOINT_ALT_MSL", "m"),
        ("Commands", "WAYPOINT_YAW", "deg"),
    ]
    .into_iter()
    .map(|(group, parameter, unit)| TelemetryField {
        group: group.to_string(),
        parameter: parameter.to_string(),
        unit: unit.to_string(),
        value: "--".to_string(),
    })
    .collect()
}
