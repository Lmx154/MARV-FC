use std::time::{Duration, Instant};

use eframe::egui;

use crate::bridge::{AppBridge, AppState};

const MOTOR_MASK_M1: u8 = 0x01;
const MOTOR_MASK_M2: u8 = 0x02;
const MOTOR_MASK_M3: u8 = 0x04;
const MOTOR_MASK_M4: u8 = 0x08;
const MOTOR_MASK_ALL: u8 = 0x0F;

const MOTOR_MODE_STOP: u8 = 0;
const MOTOR_MODE_RAW_DSHOT: u8 = 1;
const MOTOR_MODE_NORMALIZED: u8 = 2;

#[derive(Clone)]
struct BenchSequenceStep {
    motor_mask: u8,
    mode: u8,
    value: u16,
    duration_ms: u16,
    ramp_ms: u16,
}

impl BenchSequenceStep {
    fn label(&self) -> String {
        format!(
            "{} {} value={} duration={}ms ramp={}ms",
            motor_mask_label(self.motor_mask),
            motor_mode_label(self.mode),
            self.value,
            self.duration_ms,
            self.ramp_ms
        )
    }
}

pub struct DebugTabState {
    bench_timeout_ms: u16,
    motor_mask: u8,
    mode: u8,
    value: u16,
    duration_ms: u16,
    ramp_ms: u16,
    sweep_start: u16,
    sweep_end: u16,
    sweep_step: u16,
    sweep_step_duration_ms: u16,
    sweep_zero_between_ms: u16,
    sweep_repeat_count: u8,
    dshot_command: u8,
    dshot_repeat_count: u8,
    sequence_steps: Vec<BenchSequenceStep>,
    sequence_running: bool,
    sequence_next_step_idx: usize,
    sequence_next_send_at: Option<Instant>,
    sequence_auto_stop: bool,
    status: Option<String>,
}

impl DebugTabState {
    pub fn new() -> Self {
        Self {
            bench_timeout_ms: 30_000,
            motor_mask: MOTOR_MASK_ALL,
            mode: MOTOR_MODE_RAW_DSHOT,
            value: 120,
            duration_ms: 1_000,
            ramp_ms: 0,
            sweep_start: 120,
            sweep_end: 200,
            sweep_step: 20,
            sweep_step_duration_ms: 500,
            sweep_zero_between_ms: 150,
            sweep_repeat_count: 1,
            dshot_command: 0,
            dshot_repeat_count: 1,
            sequence_steps: vec![
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_M1,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 150,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_M2,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 150,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_M3,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 150,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_M4,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 150,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_ALL,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 120,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
                BenchSequenceStep {
                    motor_mask: MOTOR_MASK_ALL,
                    mode: MOTOR_MODE_RAW_DSHOT,
                    value: 200,
                    duration_ms: 1000,
                    ramp_ms: 0,
                },
            ],
            sequence_running: false,
            sequence_next_step_idx: 0,
            sequence_next_send_at: None,
            sequence_auto_stop: true,
            status: None,
        }
    }

    fn start_sequence(&mut self) {
        if self.sequence_steps.is_empty() {
            self.status = Some("Sequence has no steps".to_string());
            self.sequence_running = false;
            self.sequence_next_step_idx = 0;
            self.sequence_next_send_at = None;
            return;
        }

        self.sequence_running = true;
        self.sequence_next_step_idx = 0;
        self.sequence_next_send_at = Some(Instant::now());
        self.status = Some(format!(
            "Sequence started ({} steps)",
            self.sequence_steps.len()
        ));
    }

    fn cancel_sequence(&mut self, reason: &str) {
        self.sequence_running = false;
        self.sequence_next_send_at = None;
        self.status = Some(reason.to_string());
    }

    fn sequence_progress_label(&self) -> String {
        if !self.sequence_running {
            return "Idle".to_string();
        }

        let total = self.sequence_steps.len();
        let next = (self.sequence_next_step_idx + 1).min(total);
        if let Some(next_send_at) = self.sequence_next_send_at {
            let remaining = next_send_at.saturating_duration_since(Instant::now());
            return format!(
                "Running step {next}/{total} (next send in {} ms)",
                remaining.as_millis()
            );
        }

        format!("Running step {next}/{total}")
    }

    fn run_sequence_tick(&mut self, bridge: &mut AppBridge, uart_ready: bool) {
        if !self.sequence_running {
            return;
        }

        if !uart_ready {
            self.cancel_sequence("Sequence cancelled: UART disconnected");
            return;
        }

        let Some(next_send_at) = self.sequence_next_send_at else {
            self.sequence_next_send_at = Some(Instant::now());
            return;
        };

        let now = Instant::now();
        if now < next_send_at {
            return;
        }

        if self.sequence_next_step_idx >= self.sequence_steps.len() {
            self.sequence_running = false;
            self.sequence_next_send_at = None;
            self.status = Some("Sequence complete".to_string());
            return;
        }

        let step = self.sequence_steps[self.sequence_next_step_idx].clone();
        if let Err(error) = bridge.send_hilink_motor_test_values(
            step.motor_mask,
            step.mode,
            step.value,
            step.duration_ms,
            step.ramp_ms,
        ) {
            self.cancel_sequence(&format!(
                "Sequence failed on step {}: {}",
                self.sequence_next_step_idx + 1,
                error
            ));
            return;
        }

        self.sequence_next_step_idx += 1;
        let total = self.sequence_steps.len();
        let sent = self.sequence_next_step_idx;
        let delay = Duration::from_millis(u64::from(step.duration_ms));

        if sent >= total {
            self.sequence_running = false;
            self.sequence_next_send_at = None;

            if self.sequence_auto_stop {
                self.status = Some(match bridge.send_hilink_motor_stop() {
                    Ok(()) => format!("Sequence complete ({sent} steps) + MotorStop"),
                    Err(error) => format!(
                        "Sequence complete ({sent} steps), MotorStop failed: {}",
                        error
                    ),
                });
            } else {
                self.status = Some(format!("Sequence complete ({sent} steps)"));
            }
            return;
        }

        self.sequence_next_send_at = Some(now + delay);
        self.status = Some(format!("Sequence running: step {sent}/{total} sent"));
    }
}

pub fn render(
    ui: &mut egui::Ui,
    state: &AppState,
    debug_state: &mut DebugTabState,
    bridge: &mut AppBridge,
) {
    debug_state.run_sequence_tick(bridge, state.uart.connected);

    ui.heading("Debug");
    ui.label("Bench testing controls for direct motor validation over HILink.");
    ui.add_space(10.0);

    let uart_ready = state.uart.connected;
    if !uart_ready {
        ui.colored_label(
            egui::Color32::LIGHT_RED,
            "Open a UART port in Settings before sending bench commands.",
        );
    }

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(egui::RichText::new("Bench Control").strong());
        ui.add_space(6.0);

        ui.add_enabled_ui(uart_ready, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Arm").clicked() {
                    debug_state.status =
                        Some(command_status("Arm", bridge.send_hilink_arm()));
                }
                if ui.button("Disarm").clicked() {
                    debug_state.status =
                        Some(command_status("Disarm", bridge.send_hilink_disarm()));
                }
                if ui.button("Bench Enable").clicked() {
                    debug_state.status = Some(command_status(
                        "BenchEnable",
                        bridge.send_hilink_bench_enable(debug_state.bench_timeout_ms),
                    ));
                }
                if ui.button("Bench Disable").clicked() {
                    debug_state.status = Some(command_status(
                        "BenchDisable",
                        bridge.send_hilink_bench_disable(),
                    ));
                }
                if ui
                    .add(
                        egui::Button::new(egui::RichText::new("Motor Stop").strong())
                            .fill(egui::Color32::from_rgb(120, 24, 24)),
                    )
                    .clicked()
                {
                    debug_state.status =
                        Some(command_status("MotorStop", bridge.send_hilink_motor_stop()));
                }
                if ui.button("Request Actuator Status").clicked() {
                    debug_state.status = Some(command_status(
                        "ActuatorStatusRequest",
                        bridge.send_hilink_actuator_status_request(),
                    ));
                }
            });
        });

        ui.horizontal(|ui| {
            ui.label("Bench timeout ms");
            ui.add(
                egui::DragValue::new(&mut debug_state.bench_timeout_ms)
                    .speed(25)
                    .range(100..=65_535),
            );
        });

        if let Some(status) = &debug_state.status {
            ui.label(status);
        }
    });

    ui.add_space(10.0);

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(egui::RichText::new("Motor Test").strong());
        ui.label("Run one shot tests on selected motors.");
        ui.add_space(6.0);

        render_motor_mask_selector(ui, &mut debug_state.motor_mask);

        ui.horizontal(|ui| {
            ui.label("Mode");
            egui::ComboBox::from_id_salt("debug_motor_mode")
                .selected_text(motor_mode_label(debug_state.mode))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut debug_state.mode, MOTOR_MODE_STOP, "STOP");
                    ui.selectable_value(
                        &mut debug_state.mode,
                        MOTOR_MODE_RAW_DSHOT,
                        "RAW_DSHOT",
                    );
                    ui.selectable_value(
                        &mut debug_state.mode,
                        MOTOR_MODE_NORMALIZED,
                        "NORMALIZED",
                    );
                });
        });

        let max_value = if debug_state.mode == MOTOR_MODE_NORMALIZED {
            65_535
        } else {
            2_047
        };

        ui.horizontal(|ui| {
            ui.label("Value");
            ui.add(
                egui::Slider::new(&mut debug_state.value, 0..=max_value)
                    .text("command")
                    .clamping(egui::SliderClamping::Always),
            );
        });

        ui.horizontal(|ui| {
            ui.label("Duration ms");
            ui.add(
                egui::DragValue::new(&mut debug_state.duration_ms)
                    .speed(10)
                    .range(0..=5_000),
            );
            ui.label("Ramp ms");
            ui.add(
                egui::DragValue::new(&mut debug_state.ramp_ms)
                    .speed(5)
                    .range(0..=5_000),
            );
        });

        ui.add_space(6.0);

        ui.add_enabled_ui(uart_ready, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Send MotorTest").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest",
                        bridge.send_hilink_motor_test_values(
                            debug_state.motor_mask,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
                if ui.button("M1").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest M1",
                        bridge.send_hilink_motor_test_values(
                            MOTOR_MASK_M1,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
                if ui.button("M2").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest M2",
                        bridge.send_hilink_motor_test_values(
                            MOTOR_MASK_M2,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
                if ui.button("M3").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest M3",
                        bridge.send_hilink_motor_test_values(
                            MOTOR_MASK_M3,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
                if ui.button("M4").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest M4",
                        bridge.send_hilink_motor_test_values(
                            MOTOR_MASK_M4,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
                if ui.button("All Motors").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorTest All",
                        bridge.send_hilink_motor_test_values(
                            MOTOR_MASK_ALL,
                            debug_state.mode,
                            debug_state.value,
                            debug_state.duration_ms,
                            debug_state.ramp_ms,
                        ),
                    ));
                }
            });
        });
    });

    ui.add_space(10.0);

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(egui::RichText::new("Motor Sweep + DShot").strong());
        ui.horizontal(|ui| {
            ui.label("Sweep start/end/step");
            ui.add(egui::DragValue::new(&mut debug_state.sweep_start).speed(5));
            ui.add(egui::DragValue::new(&mut debug_state.sweep_end).speed(5));
            ui.add(
                egui::DragValue::new(&mut debug_state.sweep_step)
                    .speed(1)
                    .range(1..=2_047),
            );
        });

        ui.horizontal(|ui| {
            ui.label("Step ms");
            ui.add(
                egui::DragValue::new(&mut debug_state.sweep_step_duration_ms)
                    .speed(10)
                    .range(10..=5_000),
            );
            ui.label("Zero between ms");
            ui.add(
                egui::DragValue::new(&mut debug_state.sweep_zero_between_ms)
                    .speed(10)
                    .range(0..=5_000),
            );
            ui.label("Repeat");
            ui.add(
                egui::DragValue::new(&mut debug_state.sweep_repeat_count)
                    .speed(1)
                    .range(1..=20),
            );
        });

        ui.horizontal(|ui| {
            ui.label("DShot command");
            ui.add(
                egui::DragValue::new(&mut debug_state.dshot_command)
                    .speed(1)
                    .range(0..=255),
            );
            ui.label("Repeat");
            ui.add(
                egui::DragValue::new(&mut debug_state.dshot_repeat_count)
                    .speed(1)
                    .range(1..=20),
            );
        });

        ui.add_enabled_ui(uart_ready, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Send MotorSweep").clicked() {
                    debug_state.status = Some(command_status(
                        "MotorSweep",
                        bridge.send_hilink_motor_sweep_values(
                            debug_state.motor_mask,
                            debug_state.mode,
                            debug_state.sweep_start,
                            debug_state.sweep_end,
                            debug_state.sweep_step,
                            debug_state.sweep_step_duration_ms,
                            debug_state.sweep_zero_between_ms,
                            debug_state.sweep_repeat_count,
                        ),
                    ));
                }

                if ui.button("Send DshotCommand").clicked() {
                    debug_state.status = Some(command_status(
                        "DshotCommand",
                        bridge.send_hilink_dshot_command(
                            debug_state.motor_mask,
                            debug_state.dshot_command,
                            debug_state.dshot_repeat_count,
                        ),
                    ));
                }
            });
        });
    });

    ui.add_space(10.0);

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(egui::RichText::new("Bench Sequence").strong());
        ui.label("Queue up steps to run individual motors or all motors in any order.");
        ui.label(debug_state.sequence_progress_label());

        for (index, step) in debug_state.sequence_steps.iter().enumerate() {
            ui.horizontal_wrapped(|ui| {
                let marker = if debug_state.sequence_running
                    && index == debug_state.sequence_next_step_idx
                {
                    ">"
                } else {
                    " "
                };
                ui.monospace(format!("{marker} {:02}. {}", index + 1, step.label()));
            });
        }

        ui.horizontal_wrapped(|ui| {
            ui.checkbox(
                &mut debug_state.sequence_auto_stop,
                "Auto MotorStop after sequence",
            );

            if ui
                .add_enabled(
                    !debug_state.sequence_running,
                    egui::Button::new("Add Current As Step"),
                )
                .clicked()
            {
                debug_state.sequence_steps.push(BenchSequenceStep {
                    motor_mask: debug_state.motor_mask,
                    mode: debug_state.mode,
                    value: debug_state.value,
                    duration_ms: debug_state.duration_ms,
                    ramp_ms: debug_state.ramp_ms,
                });
            }

            if ui
                .add_enabled(
                    !debug_state.sequence_running,
                    egui::Button::new("Clear Steps"),
                )
                .clicked()
            {
                debug_state.sequence_steps.clear();
            }

            ui.add_enabled_ui(uart_ready, |ui| {
                if ui
                    .add_enabled(
                        !debug_state.sequence_running,
                        egui::Button::new("Run Sequence"),
                    )
                    .clicked()
                {
                    debug_state.start_sequence();
                }

                if ui
                    .add_enabled(
                        debug_state.sequence_running,
                        egui::Button::new("Stop Sequence"),
                    )
                    .clicked()
                {
                    debug_state.cancel_sequence("Sequence cancelled by operator");
                }
            });
        });
    });

    ui.add_space(10.0);
    ui.label("Live time feed:");
    ui.label(egui::RichText::new(&state.current_time).size(24.0).strong());
}

fn command_status(label: &str, result: Result<(), String>) -> String {
    match result {
        Ok(()) => format!("Sent {label}"),
        Err(error) => format!("Failed to send {label}: {error}"),
    }
}

fn render_motor_mask_selector(ui: &mut egui::Ui, mask: &mut u8) {
    ui.horizontal_wrapped(|ui| {
        ui.label("Motors");
        motor_bit_checkbox(ui, "M1", mask, MOTOR_MASK_M1);
        motor_bit_checkbox(ui, "M2", mask, MOTOR_MASK_M2);
        motor_bit_checkbox(ui, "M3", mask, MOTOR_MASK_M3);
        motor_bit_checkbox(ui, "M4", mask, MOTOR_MASK_M4);
        if ui.button("All").clicked() {
            *mask = MOTOR_MASK_ALL;
        }
        if ui.button("Clear").clicked() {
            *mask = 0;
        }
        ui.label(format!("Selected: {}", motor_mask_label(*mask)));
    });
}

fn motor_bit_checkbox(ui: &mut egui::Ui, label: &str, mask: &mut u8, bit: u8) {
    let mut enabled = (*mask & bit) != 0;
    if ui.checkbox(&mut enabled, label).changed() {
        if enabled {
            *mask |= bit;
        } else {
            *mask &= !bit;
        }
    }
}

fn motor_mode_label(mode: u8) -> &'static str {
    match mode {
        MOTOR_MODE_STOP => "STOP",
        MOTOR_MODE_RAW_DSHOT => "RAW_DSHOT",
        MOTOR_MODE_NORMALIZED => "NORMALIZED",
        _ => "UNKNOWN",
    }
}

fn motor_mask_label(mask: u8) -> String {
    if mask == 0 {
        return "none".to_string();
    }
    if mask == MOTOR_MASK_ALL {
        return "ALL".to_string();
    }

    let mut labels = Vec::new();
    if (mask & MOTOR_MASK_M1) != 0 {
        labels.push("M1");
    }
    if (mask & MOTOR_MASK_M2) != 0 {
        labels.push("M2");
    }
    if (mask & MOTOR_MASK_M3) != 0 {
        labels.push("M3");
    }
    if (mask & MOTOR_MASK_M4) != 0 {
        labels.push("M4");
    }
    labels.join("|")
}
