use std::collections::VecDeque;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use crate::backend::{
    encode_hilink_command, hilink_msg_type_label, parse_hilink_frames_for_display,
    BenchEnableCommand, CvWaypointCommand, DshotCommand, GlobalWaypointCommand,
    HilSensorFrameCommand, HilinkActuatorCommand, HilinkCommand, HilinkEvent, HilinkParserService,
    MixerMotorOrderCommand, MotorSweepCommand, MotorTestCommand, TimeService, TofWaypointCommand,
    UartPortInfo, UartService, DEFAULT_UART_BAUD_RATE,
};

use super::app_state::{
    GazeboBridgeProcessState, HilComparisonState, HilForwardedActuatorFrame,
    HilOutstandingSourceFrame, HilResponseFrame, HilSourceFrame, HilinkCommandState,
    PendingCommandStatus, PendingHilinkCommand,
};
use super::gazebo_bridge_client::{GazeboBridgeClient, GazeboSensorFrame};
use super::AppState;

const SERIAL_MONITOR_MAX_LINES: usize = 600;
const SERIAL_MONITOR_BYTES_PER_LINE: usize = 24;
const SERIAL_PARSE_MAX_LINES: usize = 600;
const PENDING_COMMAND_MAX_ITEMS: usize = 64;
const HIL_SOURCE_FRESH_FOR: Duration = Duration::from_secs(2);
const HIL_RESPONSE_TIMEOUT: Duration = Duration::from_millis(500);
const RESPONSE_FLAG_ARMED: u32 = 1 << 0;
const RESPONSE_FLAG_FAILSAFE: u32 = 1 << 1;
const RESPONSE_FLAG_ESTIMATOR_VALID: u32 = 1 << 2;
const RESPONSE_FLAG_MOTORS_VALID: u32 = 1 << 3;
const RESPONSE_FLAG_CONTROL_VALID: u32 = 1 << 4;
const RESPONSE_FLAG_SENSOR_INPUT_VALID: u32 = 1 << 6;
const RESPONSE_FLAG_BENCH_OVERRIDE: u32 = 1 << 7;
const SENSOR_VALID_ACCEL: u32 = 1 << 0;
const SENSOR_VALID_GYRO: u32 = 1 << 1;
const SENSOR_VALID_MAG: u32 = 1 << 2;
const SENSOR_VALID_BARO: u32 = 1 << 3;
const SENSOR_VALID_GPS: u32 = 1 << 4;
const MIN_HIL_GPS_FIX_TYPE: u8 = 3;
const MIN_HIL_GPS_SATS: u8 = 4;

#[derive(Clone, Copy)]
struct OutstandingHilSource {
    sequence: u64,
    sim_time_us: u64,
    sent_at: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct HilSourceStamp {
    sequence: u64,
    sim_time_us: u64,
}

pub struct AppBridge {
    time_service: TimeService,
    uart_service: UartService,
    hilink_parser: HilinkParserService,
    hilink_tx_seq: u16,
    serial_monitor_lines: VecDeque<String>,
    serial_monitor_dropped_lines: u64,
    serial_parse_enabled: bool,
    serial_parse_lines: VecDeque<String>,
    serial_parse_rx_buffer: Vec<u8>,
    serial_parse_tx_buffer: Vec<u8>,
    serial_parse_dropped_lines: u64,
    latest_hil_source: Option<HilSourceFrame>,
    latest_hil_response: Option<HilResponseFrame>,
    latest_forwarded_actuator: Option<HilForwardedActuatorFrame>,
    last_hil_source_at: Option<Instant>,
    hil_ready: bool,
    outstanding_hil_source: Option<OutstandingHilSource>,
    last_forwarded_hil_source_stamp: Option<HilSourceStamp>,
    completed_hil_responses: VecDeque<(u64, u64)>,
    hil_protocol_fault_count: u64,
    latest_hil_protocol_fault: Option<String>,
    pending_commands: VecDeque<PendingHilinkCommand>,
    command_state: HilinkCommandState,
    state: AppState,
    gazebo_bridge_process: Option<Child>,
    gazebo_bridge_process_state: GazeboBridgeProcessState,
    app_started_at: Instant,
    last_refresh: Instant,
    refresh_interval: Duration,
    gazebo_client: GazeboBridgeClient,
}

impl AppBridge {
    pub fn new() -> Self {
        let time_service = TimeService::new();
        let uart_service = UartService::new();
        let hilink_parser = HilinkParserService::new();
        let current_time = time_service.current_time();
        let gazebo_client = GazeboBridgeClient::new("127.0.0.1:9000");
        let uart = Self::uart_state_from(&uart_service);
        let hilink = hilink_parser.telemetry();

        Self {
            time_service,
            uart_service,
            hilink_parser,
            hilink_tx_seq: 0,
            serial_monitor_lines: VecDeque::new(),
            serial_monitor_dropped_lines: 0,
            serial_parse_enabled: true,
            serial_parse_lines: VecDeque::new(),
            serial_parse_rx_buffer: Vec::new(),
            serial_parse_tx_buffer: Vec::new(),
            serial_parse_dropped_lines: 0,
            latest_hil_source: None,
            latest_hil_response: None,
            latest_forwarded_actuator: None,
            last_hil_source_at: None,
            hil_ready: false,
            outstanding_hil_source: None,
            last_forwarded_hil_source_stamp: None,
            completed_hil_responses: VecDeque::new(),
            hil_protocol_fault_count: 0,
            latest_hil_protocol_fault: None,
            pending_commands: VecDeque::new(),
            command_state: HilinkCommandState::default(),
            state: AppState {
                current_time,
                gazebo_bridge: gazebo_client.stats(),
                gazebo_bridge_process: Self::default_gazebo_bridge_process_state(),
                uart,
                hil_comparison: Default::default(),
                hilink,
                serial_monitor: Default::default(),
                hilink_commands: Default::default(),
            },
            gazebo_bridge_process: None,
            gazebo_bridge_process_state: Self::default_gazebo_bridge_process_state(),
            app_started_at: Instant::now(),
            last_refresh: Instant::now(),
            refresh_interval: Duration::from_millis(250),
            gazebo_client,
        }
    }

    pub fn tick(&mut self) {
        self.uart_service.poll_connection();
        self.refresh_gazebo_bridge_process_state();
        self.poll_uart_hilink();
        self.gazebo_client.poll();
        self.expire_hil_response_timeout();
        self.forward_gazebo_sensors_to_fc();

        if self.last_refresh.elapsed() >= self.refresh_interval {
            self.state.current_time = self.time_service.current_time();
            self.refresh_uart_state();
            self.refresh_gazebo_state();
            self.refresh_hil_comparison_state();
            self.last_refresh = Instant::now();
        }
    }

    pub fn snapshot(&self) -> AppState {
        self.state.clone()
    }

    pub fn open_uart_with_baud_rate(
        &mut self,
        port_name: &str,
        baud_rate: u32,
    ) -> Result<(), String> {
        let result = self.uart_service.open_with_baud_rate(port_name, baud_rate);
        if result.is_ok() {
            self.hil_ready = false;
            self.outstanding_hil_source = None;
            self.last_forwarded_hil_source_stamp = None;
            self.completed_hil_responses.clear();
        }
        self.refresh_uart_state();
        result
    }

    pub fn close_uart(&mut self) {
        self.uart_service.close();
        self.hil_ready = false;
        self.outstanding_hil_source = None;
        self.last_forwarded_hil_source_stamp = None;
        self.refresh_uart_state();
    }

    #[allow(dead_code)]
    pub fn set_uart_baud_rate(&mut self, baud_rate: u32) -> Result<(), String> {
        let result = self.uart_service.set_baud_rate(baud_rate);
        self.refresh_uart_state();
        result
    }

    pub fn list_uart_ports(&mut self) -> Vec<UartPortInfo> {
        let ports = self.uart_service.list_ports();
        self.refresh_uart_state();
        ports
    }

    pub fn connect_gazebo_bridge(&mut self, endpoint: &str) -> Result<(), String> {
        self.gazebo_client.set_endpoint(endpoint.to_string());
        let result = self.gazebo_client.connect();
        if result.is_ok() {
            self.outstanding_hil_source = None;
            self.last_forwarded_hil_source_stamp = None;
            self.completed_hil_responses.clear();
        }
        self.refresh_gazebo_state();
        result
    }

    pub fn start_gazebo_bridge_process(&mut self, endpoint: &str) -> Result<(), String> {
        self.refresh_gazebo_bridge_process_state();
        if self.gazebo_bridge_process_state.running {
            return Ok(());
        }

        let bridge_path = gazebo_bridge_executable_path();
        self.gazebo_bridge_process_state.launch_path = bridge_path.display().to_string();
        self.gazebo_bridge_process_state.last_exit_status = None;

        if !bridge_path.is_file() {
            let error = format!(
                "Gazebo bridge executable is missing at {}. Build it with cmake -S telemetry-app/gazebo_bridge -B telemetry-app/gazebo_bridge/build && cmake --build telemetry-app/gazebo_bridge/build",
                bridge_path.display()
            );
            self.gazebo_bridge_process_state.last_error = Some(error.clone());
            self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
            return Err(error);
        }

        let port = parse_endpoint_port(endpoint).unwrap_or(9000);
        let mut command = Command::new(&bridge_path);
        command
            .arg("--port")
            .arg(port.to_string())
            .current_dir(gazebo_bridge_dir())
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());
        let config_path = gazebo_bridge_config_path();
        if config_path.is_file() {
            command.arg("--config").arg(config_path);
        }

        match command.spawn() {
            Ok(child) => {
                self.gazebo_bridge_process_state.running = true;
                self.gazebo_bridge_process_state.pid = Some(child.id());
                self.gazebo_bridge_process_state.last_error = None;
                self.gazebo_bridge_process = Some(child);
                self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
                Ok(())
            }
            Err(error) => {
                let error = format!("failed to start Gazebo bridge: {error}");
                self.gazebo_bridge_process_state.running = false;
                self.gazebo_bridge_process_state.pid = None;
                self.gazebo_bridge_process_state.last_error = Some(error.clone());
                self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
                Err(error)
            }
        }
    }

    pub fn stop_gazebo_bridge_process(&mut self) -> Result<(), String> {
        let Some(mut child) = self.gazebo_bridge_process.take() else {
            self.refresh_gazebo_bridge_process_state();
            return Ok(());
        };

        if let Err(error) = child.kill() {
            let error = format!("failed to stop Gazebo bridge process: {error}");
            self.gazebo_bridge_process_state.last_error = Some(error.clone());
            self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
            return Err(error);
        }

        match child.wait() {
            Ok(status) => {
                self.gazebo_bridge_process_state.running = false;
                self.gazebo_bridge_process_state.pid = None;
                self.gazebo_bridge_process_state.last_exit_status = Some(status.to_string());
                self.gazebo_bridge_process_state.last_error = None;
                self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
                Ok(())
            }
            Err(error) => {
                let error = format!("failed to wait for Gazebo bridge process shutdown: {error}");
                self.gazebo_bridge_process_state.last_error = Some(error.clone());
                self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
                Err(error)
            }
        }
    }

    pub fn disconnect_gazebo_bridge(&mut self) {
        self.gazebo_client.disconnect();
        self.outstanding_hil_source = None;
        self.last_forwarded_hil_source_stamp = None;
        self.refresh_gazebo_state();
    }

    pub fn send_test_actuator_command(&mut self, motor_speed: f32) -> Result<(), String> {
        let result = self.gazebo_client.send_test_actuator_command(motor_speed);
        self.refresh_gazebo_state();
        result
    }

    pub fn send_gazebo_sim_control_command(&mut self, action: &str) -> Result<(), String> {
        let result = self.gazebo_client.send_sim_control_command(action);
        self.refresh_gazebo_state();
        result
    }

    pub fn send_hilink_command(&mut self, command: HilinkCommand) -> Result<(), String> {
        let send_time_ms = self.current_hilink_send_time_ms();
        self.send_hilink_command_at(command, send_time_ms)
    }

    fn send_hilink_command_at(
        &mut self,
        command: HilinkCommand,
        send_time_ms: u32,
    ) -> Result<(), String> {
        let seq = self.hilink_tx_seq;
        self.hilink_tx_seq = self.hilink_tx_seq.wrapping_add(1);
        let msg_type = command.msg_type_id();
        let label = command.label().to_string();
        let encoded = encode_hilink_command(&command, seq, send_time_ms)?;
        let result = self.uart_service.write_all(&encoded);

        if result.is_ok() {
            self.track_pending_command(seq, msg_type, label);
            self.append_serial_monitor_bytes("TX", &encoded);
            self.hilink_parser.push_tx_bytes(&encoded);
        }

        self.refresh_uart_state();
        result
    }

    pub fn send_hilink_ping(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::Ping)
    }

    pub fn send_hilink_arm(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::Arm)
    }

    pub fn send_hilink_disarm(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::Disarm)
    }

    pub fn send_hilink_rtl(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::Rtl)
    }

    pub fn send_hilink_bench_enable(&mut self, timeout_ms: u16) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::BenchEnable(BenchEnableCommand {
            timeout_ms,
        }))
    }

    pub fn send_hilink_bench_disable(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::BenchDisable)
    }

    pub fn send_hilink_motor_test(&mut self, command: MotorTestCommand) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::MotorTest(command))
    }

    pub fn send_hilink_motor_test_values(
        &mut self,
        motor_mask: u8,
        mode: u8,
        value: u16,
        duration_ms: u16,
        ramp_ms: u16,
    ) -> Result<(), String> {
        self.send_hilink_motor_test(MotorTestCommand {
            motor_mask,
            mode,
            value,
            duration_ms,
            ramp_ms,
        })
    }

    pub fn send_hilink_motor_sweep(&mut self, command: MotorSweepCommand) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::MotorSweep(command))
    }

    #[allow(clippy::too_many_arguments)]
    pub fn send_hilink_motor_sweep_values(
        &mut self,
        motor_mask: u8,
        mode: u8,
        start_value: u16,
        end_value: u16,
        step_value: u16,
        step_duration_ms: u16,
        zero_between_ms: u16,
        repeat_count: u8,
    ) -> Result<(), String> {
        self.send_hilink_motor_sweep(MotorSweepCommand {
            motor_mask,
            mode,
            start_value,
            end_value,
            step_value,
            step_duration_ms,
            zero_between_ms,
            repeat_count,
        })
    }

    pub fn send_hilink_motor_stop(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::MotorStop)
    }

    pub fn run_radio_link_smoke_test(&mut self) -> Result<(), String> {
        self.send_hilink_ping()?;
        self.send_hilink_arm()?;
        self.send_hilink_disarm()?;
        self.send_hilink_motor_stop()
    }

    pub fn send_hilink_dshot_command(
        &mut self,
        motor_mask: u8,
        command: u8,
        repeat_count: u8,
    ) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::DshotCommand(DshotCommand {
            motor_mask,
            command,
            repeat_count,
        }))
    }

    pub fn send_hilink_actuator_status_request(&mut self) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::ActuatorStatusRequest)
    }

    pub fn send_hilink_mixer_motor_order(
        &mut self,
        output_for_motor: [u8; 4],
    ) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::MixerMotorOrder(MixerMotorOrderCommand {
            output_for_motor,
        }))
    }

    pub fn send_hilink_control_waypoint(
        &mut self,
        ref_sim_tick: u64,
        ref_sim_time_us: u64,
        lat_deg: f64,
        lon_deg: f64,
        alt_msl_m: f32,
        yaw_deg: f32,
    ) -> Result<(), String> {
        self.ensure_waypoint_control_ready()?;
        let (ref_sim_tick, ref_sim_time_us) =
            self.fill_empty_sim_stamp(ref_sim_tick, ref_sim_time_us);
        self.send_hilink_command(HilinkCommand::ControlWaypoint(GlobalWaypointCommand {
            ref_sim_tick,
            ref_sim_time_us,
            lat_deg,
            lon_deg,
            alt_msl_m,
            yaw_deg,
        }))
    }

    pub fn send_hilink_mission_waypoint(
        &mut self,
        ref_sim_tick: u64,
        ref_sim_time_us: u64,
        lat_deg: f64,
        lon_deg: f64,
        alt_msl_m: f32,
        yaw_deg: f32,
    ) -> Result<(), String> {
        self.ensure_waypoint_control_ready()?;
        let (ref_sim_tick, ref_sim_time_us) =
            self.fill_empty_sim_stamp(ref_sim_tick, ref_sim_time_us);
        self.send_hilink_command(HilinkCommand::MissionWaypoint(GlobalWaypointCommand {
            ref_sim_tick,
            ref_sim_time_us,
            lat_deg,
            lon_deg,
            alt_msl_m,
            yaw_deg,
        }))
    }

    pub fn send_hilink_cv_waypoint(
        &mut self,
        ref_sim_tick: u64,
        ref_sim_time_us: u64,
        dir_body: [f32; 3],
        confidence: f32,
    ) -> Result<(), String> {
        let (ref_sim_tick, ref_sim_time_us) =
            self.fill_empty_sim_stamp(ref_sim_tick, ref_sim_time_us);
        self.send_hilink_command(HilinkCommand::CvWaypoint(CvWaypointCommand {
            ref_sim_tick,
            ref_sim_time_us,
            dir_body,
            confidence,
        }))
    }

    pub fn send_hilink_tof_waypoint(
        &mut self,
        ref_sim_tick: u64,
        ref_sim_time_us: u64,
        distance_m: f32,
        bearing_deg: f32,
        elevation_deg: f32,
    ) -> Result<(), String> {
        let (ref_sim_tick, ref_sim_time_us) =
            self.fill_empty_sim_stamp(ref_sim_tick, ref_sim_time_us);
        self.send_hilink_command(HilinkCommand::TofWaypoint(TofWaypointCommand {
            ref_sim_tick,
            ref_sim_time_us,
            distance_m,
            bearing_deg,
            elevation_deg,
        }))
    }

    pub fn send_hilink_sensor_frame(&mut self, frame: HilSensorFrameCommand) -> Result<(), String> {
        self.send_hilink_command(HilinkCommand::HilSensorFrame(frame))
    }

    pub fn send_hilink_sensor_frame_values(
        &mut self,
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
    ) -> Result<(), String> {
        self.send_hilink_sensor_frame(HilSensorFrameCommand {
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
        })
    }

    pub fn clear_serial_monitor(&mut self) {
        self.serial_monitor_lines.clear();
        self.serial_monitor_dropped_lines = 0;
        self.clear_serial_parse_output();
        self.refresh_serial_monitor_state();
    }

    pub fn set_serial_monitor_parse_enabled(&mut self, enabled: bool) {
        if self.serial_parse_enabled == enabled {
            return;
        }

        self.serial_parse_enabled = enabled;
        self.clear_serial_parse_output();
        self.refresh_serial_monitor_state();
    }

    fn refresh_uart_state(&mut self) {
        self.drain_hilink_events();
        self.state.uart = Self::uart_state_from(&self.uart_service);
        self.state.hilink = self.hilink_parser.telemetry();
        self.state.hilink_commands = self.command_state.clone();
        self.refresh_hil_comparison_state();
        self.refresh_serial_monitor_state();
    }

    fn refresh_gazebo_state(&mut self) {
        self.state.gazebo_bridge = self.gazebo_client.stats();
        self.state.gazebo_bridge.connected_for_secs = self
            .gazebo_client
            .connected_for()
            .map(|duration| duration.as_secs());
    }

    fn refresh_gazebo_bridge_process_state(&mut self) {
        if let Some(child) = self.gazebo_bridge_process.as_mut() {
            match child.try_wait() {
                Ok(Some(status)) => {
                    self.gazebo_bridge_process = None;
                    self.gazebo_bridge_process_state.running = false;
                    self.gazebo_bridge_process_state.pid = None;
                    self.gazebo_bridge_process_state.last_exit_status = Some(status.to_string());
                }
                Ok(None) => {
                    self.gazebo_bridge_process_state.running = true;
                    self.gazebo_bridge_process_state.pid = Some(child.id());
                }
                Err(error) => {
                    self.gazebo_bridge_process_state.running = false;
                    self.gazebo_bridge_process_state.pid = None;
                    self.gazebo_bridge_process_state.last_error =
                        Some(format!("failed to inspect Gazebo bridge process: {error}"));
                }
            }
        } else {
            self.gazebo_bridge_process_state.running = false;
            self.gazebo_bridge_process_state.pid = None;
        }

        self.state.gazebo_bridge_process = self.gazebo_bridge_process_state.clone();
    }

    fn poll_uart_hilink(&mut self) {
        if !self.uart_service.is_open() {
            return;
        }

        match self.uart_service.read_available() {
            Ok(bytes) if !bytes.is_empty() => {
                self.append_serial_monitor_bytes("RX", &bytes);
                self.hilink_parser.push_rx_bytes(&bytes);
                self.drain_hilink_events();
                self.handle_hilink_actuator_commands();
            }
            Ok(_) => {}
            Err(error) => eprintln!("failed to poll UART HILink bytes: {error}"),
        }
    }

    fn append_serial_monitor_bytes(&mut self, direction: &str, bytes: &[u8]) {
        let elapsed_ms = self.app_started_at.elapsed().as_millis();

        for (offset, chunk) in bytes.chunks(SERIAL_MONITOR_BYTES_PER_LINE).enumerate() {
            let byte_offset = offset * SERIAL_MONITOR_BYTES_PER_LINE;
            let line = format_serial_monitor_line(direction, elapsed_ms, byte_offset, chunk);
            self.serial_monitor_lines.push_back(line);

            if self.serial_monitor_lines.len() > SERIAL_MONITOR_MAX_LINES {
                self.serial_monitor_lines.pop_front();
                self.serial_monitor_dropped_lines =
                    self.serial_monitor_dropped_lines.saturating_add(1);
            }
        }

        if self.serial_parse_enabled {
            self.append_serial_parse_bytes(direction, bytes);
        }
    }

    fn refresh_serial_monitor_state(&mut self) {
        self.state.serial_monitor = super::app_state::SerialMonitorState {
            lines: self.serial_monitor_lines.iter().cloned().collect(),
            dropped_lines: self.serial_monitor_dropped_lines,
            parse_enabled: self.serial_parse_enabled,
            parsed_lines: self.serial_parse_lines.iter().cloned().collect(),
            dropped_parsed_lines: self.serial_parse_dropped_lines,
        };
    }

    fn append_serial_parse_bytes(&mut self, direction: &str, bytes: &[u8]) {
        let parse_buffer = match direction {
            "RX" => &mut self.serial_parse_rx_buffer,
            "TX" => &mut self.serial_parse_tx_buffer,
            _ => return,
        };

        let mut summaries = Vec::new();
        for &byte in bytes {
            parse_buffer.push(byte);
            if byte == 0x00 {
                if parse_buffer.len() > 1 {
                    summaries.extend(parse_hilink_frames_for_display(parse_buffer, direction));
                }
                parse_buffer.clear();
            }
        }

        for summary in summaries {
            self.serial_parse_lines.push_back(summary);
            if self.serial_parse_lines.len() > SERIAL_PARSE_MAX_LINES {
                self.serial_parse_lines.pop_front();
                self.serial_parse_dropped_lines = self.serial_parse_dropped_lines.saturating_add(1);
            }
        }
    }

    fn clear_serial_parse_output(&mut self) {
        self.serial_parse_lines.clear();
        self.serial_parse_rx_buffer.clear();
        self.serial_parse_tx_buffer.clear();
        self.serial_parse_dropped_lines = 0;
    }

    fn handle_hilink_actuator_commands(&mut self) {
        for command in self.hilink_parser.drain_actuator_commands() {
            let Some(motor_cmd) = self.validated_hil_motor_command(&command) else {
                continue;
            };

            let response = HilResponseFrame::from(command);
            self.latest_hil_response = Some(response.clone());

            match self.gazebo_client.send_hil_actuator_command(
                response.sim_tick,
                response.sim_time_us,
                motor_cmd,
            ) {
                Ok(()) => {
                    if hil_response_allows_motion(response.flags) {
                        self.latest_hil_protocol_fault = None;
                    }
                    self.latest_forwarded_actuator = Some(HilForwardedActuatorFrame {
                        sequence: response.sim_tick,
                        sim_time_us: response.sim_time_us,
                        motor_cmd,
                    });
                    self.completed_hil_responses
                        .push_back((response.sim_tick, response.sim_time_us));
                    while self.completed_hil_responses.len() > 64 {
                        self.completed_hil_responses.pop_front();
                    }
                    self.outstanding_hil_source = None;
                }
                Err(error) => {
                    eprintln!("failed to forward HIL actuator command to Gazebo: {error}")
                }
            }

            self.refresh_hil_comparison_state();
        }
    }

    fn forward_gazebo_sensors_to_fc(&mut self) {
        let frames = self.gazebo_client.drain_sensor_frames();
        if frames.is_empty() {
            return;
        }

        let mut latest_frame = None;
        for frame in frames {
            self.record_hil_source(frame.clone());
            latest_frame = Some(frame);
        }

        let Some(frame) = latest_frame else {
            return;
        };

        let frame_stamp = HilSourceStamp {
            sequence: frame.sequence,
            sim_time_us: frame.sim_time_us,
        };
        if self.source_stream_reset_detected(frame_stamp) {
            self.reset_hil_lockstep_for_source_reset(frame_stamp);
        }

        if !self.uart_service.is_open() || !self.hil_ready || self.outstanding_hil_source.is_some()
        {
            self.refresh_gazebo_state();
            self.refresh_hil_comparison_state();
            return;
        }

        if self.duplicate_or_stale_hil_source(frame_stamp) {
            self.refresh_gazebo_state();
            self.refresh_hil_comparison_state();
            return;
        }

        let send_time_ms = sim_time_us_to_send_time_ms(frame.sim_time_us);
        let command =
            HilinkCommand::HilSensorFrame(Self::hil_sensor_frame_from_gazebo(frame.clone()));
        if let Err(error) = self.send_hilink_command_at(command, send_time_ms) {
            eprintln!("failed to forward Gazebo sensor frame to FC: {error}");
        } else {
            self.outstanding_hil_source = Some(OutstandingHilSource {
                sequence: frame.sequence,
                sim_time_us: frame.sim_time_us,
                sent_at: Instant::now(),
            });
            self.last_forwarded_hil_source_stamp = Some(frame_stamp);
        }

        self.refresh_gazebo_state();
        self.refresh_hil_comparison_state();
    }

    fn record_hil_source(&mut self, frame: GazeboSensorFrame) {
        self.latest_hil_source = Some(HilSourceFrame::from(frame));
        self.last_hil_source_at = Some(Instant::now());
    }

    fn source_stream_reset_detected(&self, stamp: HilSourceStamp) -> bool {
        self.last_forwarded_hil_source_stamp.is_some_and(|last| {
            stamp.sequence < last.sequence || stamp.sim_time_us < last.sim_time_us
        })
    }

    fn duplicate_or_stale_hil_source(&self, stamp: HilSourceStamp) -> bool {
        self.last_forwarded_hil_source_stamp.is_some_and(|last| {
            stamp.sequence == last.sequence || stamp.sim_time_us <= last.sim_time_us
        })
    }

    fn reset_hil_lockstep_for_source_reset(&mut self, stamp: HilSourceStamp) {
        self.outstanding_hil_source = None;
        self.last_forwarded_hil_source_stamp = None;
        self.completed_hil_responses.clear();
        self.latest_hil_response = None;
        self.latest_forwarded_actuator = None;
        self.record_hil_protocol_fault(format!(
            "resynced HIL source after clock reset to tick {} time {}us",
            stamp.sequence, stamp.sim_time_us
        ));
    }

    fn hil_source_active(&self) -> bool {
        self.last_hil_source_at
            .is_some_and(|last_source_at| last_source_at.elapsed() <= HIL_SOURCE_FRESH_FOR)
    }

    fn ensure_waypoint_control_ready(&self) -> Result<(), String> {
        if !self.hil_ready {
            return Err(
                "MARV HIL link is not ready yet; wait for HilReady before sending waypoints"
                    .to_string(),
            );
        }

        if !self.hil_source_active() {
            return Err(
                "HIL sensor source is inactive; start/connect the Gazebo HIL source before sending waypoints"
                    .to_string(),
            );
        }

        let Some(source) = self.latest_hil_source.as_ref() else {
            return Err("HIL sensor source has not produced a frame yet".to_string());
        };

        if source.fix_type < MIN_HIL_GPS_FIX_TYPE || source.sats < MIN_HIL_GPS_SATS {
            return Err(format!(
                "HIL GPS reference is not valid enough for global waypoints: fix={} sats={}",
                source.fix_type, source.sats
            ));
        }

        if !source.lat_deg.is_finite()
            || !source.lon_deg.is_finite()
            || !source.alt_msl_m.is_finite()
            || !source.vel_ned_mps.iter().all(|value| value.is_finite())
        {
            return Err("HIL GPS reference contains non-finite values".to_string());
        }

        Ok(())
    }

    fn refresh_hil_comparison_state(&mut self) {
        let source_active = self.hil_source_active();
        let latest_source = source_active
            .then(|| self.latest_hil_source.clone())
            .flatten();
        let latest_response = latest_source.as_ref().and(self.latest_hil_response.clone());
        let latest_forwarded_actuator = latest_source
            .as_ref()
            .and(self.latest_forwarded_actuator.clone());

        let (matched, sim_time_delta_us) = match (latest_source.as_ref(), latest_response.as_ref())
        {
            (Some(source), Some(response)) => {
                let delta = response.sim_time_us as i128 - source.sim_time_us as i128;
                let delta = delta.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64;
                let matched = response.sim_tick == source.sequence
                    || response.sim_time_us == source.sim_time_us;
                (matched, Some(delta))
            }
            _ => (false, None),
        };

        self.state.hil_comparison = HilComparisonState {
            source_active,
            hil_ready: self.hil_ready,
            outstanding_source: self.outstanding_hil_source.map(|source| {
                HilOutstandingSourceFrame {
                    sequence: source.sequence,
                    sim_time_us: source.sim_time_us,
                }
            }),
            latest_source,
            latest_response,
            latest_forwarded_actuator,
            matched,
            sim_time_delta_us,
            protocol_fault_count: self.hil_protocol_fault_count,
            latest_protocol_fault: self.latest_hil_protocol_fault.clone(),
        };
    }

    fn hil_sensor_frame_from_gazebo(frame: GazeboSensorFrame) -> HilSensorFrameCommand {
        let mut command = HilSensorFrameCommand::default();
        command.sim_tick = frame.sequence;
        command.sim_time_us = frame.sim_time_us;
        command.valid_flags = sanitized_sensor_valid_flags(&frame);
        command.accel_mps2 = frame.accel_mps2;
        command.gyro_rps = frame.gyro_rps;
        command.mag_ut = frame.mag_ut;
        command.pressure_pa = frame.pressure_pa;
        command.baro_altitude_m = frame.baro_altitude_m;
        command.temperature_c = frame.temperature_c;
        command.lat_deg = frame.lat_deg;
        command.lon_deg = frame.lon_deg;
        command.alt_msl_m = frame.alt_msl_m;
        command.vel_ned_mps = frame.vel_ned_mps;
        command.sats = frame.sats;
        command.fix_type = frame.fix_type;
        command.battery_voltage_v = frame.battery_voltage_v;
        command.rssi_dbm = frame.rssi_dbm;
        command.snr_db_x100 = frame.snr_db_x100;
        command.loss_pct_x100 = frame.loss_pct_x100;
        command
    }

    fn validated_hil_motor_command(&mut self, command: &HilinkActuatorCommand) -> Option<[u16; 4]> {
        if !self.hil_source_active() {
            self.record_hil_protocol_fault("dropped HIL response while Gazebo source is inactive");
            return None;
        }

        if self
            .completed_hil_responses
            .iter()
            .any(|stamp| *stamp == (command.sim_tick, command.sim_time_us))
        {
            self.record_hil_protocol_fault(format!(
                "dropped duplicate HIL response tick {} time {}us",
                command.sim_tick, command.sim_time_us
            ));
            return None;
        }

        let Some(outstanding) = self.outstanding_hil_source else {
            self.record_hil_protocol_fault(format!(
                "dropped stale HIL response tick {} time {}us with no outstanding source",
                command.sim_tick, command.sim_time_us
            ));
            return None;
        };

        if command.sim_tick != outstanding.sequence
            || command.sim_time_us != outstanding.sim_time_us
        {
            self.record_hil_protocol_fault(format!(
                "dropped mismatched HIL response tick {} time {}us; expected tick {} time {}us",
                command.sim_tick,
                command.sim_time_us,
                outstanding.sequence,
                outstanding.sim_time_us
            ));
            return None;
        }

        if command.flags & RESPONSE_FLAG_MOTORS_VALID == 0 {
            self.record_hil_protocol_fault(format!(
                "dropped HIL response tick {} because MOTORS_VALID is clear",
                command.sim_tick
            ));
            self.outstanding_hil_source = None;
            return None;
        }

        if command.flags & RESPONSE_FLAG_BENCH_OVERRIDE != 0 {
            if command.flags & RESPONSE_FLAG_SENSOR_INPUT_VALID == 0 {
                self.record_hil_protocol_fault(format!(
                    "forcing zero motors for HIL bench response tick {} because SENSOR_INPUT_VALID is clear",
                    command.sim_tick
                ));
                return Some([0; 4]);
            }

            return Some(command.motor_cmd);
        }

        if command.flags & RESPONSE_FLAG_FAILSAFE != 0 {
            self.record_hil_protocol_fault(format!(
                "forcing zero motors for HIL response tick {} because FAILSAFE is set",
                command.sim_tick
            ));
            return Some([0; 4]);
        }

        if command.flags & RESPONSE_FLAG_SENSOR_INPUT_VALID == 0 {
            self.record_hil_protocol_fault(format!(
                "forcing zero motors for HIL response tick {} because SENSOR_INPUT_VALID is clear",
                command.sim_tick
            ));
            return Some([0; 4]);
        }

        if command.flags & RESPONSE_FLAG_ESTIMATOR_VALID == 0 {
            self.record_hil_protocol_fault(format!(
                "forcing zero motors for HIL response tick {} because ESTIMATOR_VALID is clear",
                command.sim_tick
            ));
            return Some([0; 4]);
        }

        if command.flags & RESPONSE_FLAG_CONTROL_VALID == 0 {
            self.record_hil_protocol_fault(format!(
                "forcing zero motors for HIL response tick {} because CONTROL_VALID is clear",
                command.sim_tick
            ));
            return Some([0; 4]);
        }

        if command.flags & RESPONSE_FLAG_ARMED == 0 {
            self.record_hil_protocol_fault(format!(
                "forcing zero motors for HIL response tick {} because ARMED is clear",
                command.sim_tick
            ));
            return Some([0; 4]);
        }

        Some(command.motor_cmd)
    }

    fn expire_hil_response_timeout(&mut self) {
        let Some(outstanding) = self.outstanding_hil_source else {
            return;
        };

        if outstanding.sent_at.elapsed() <= HIL_RESPONSE_TIMEOUT {
            return;
        }

        self.record_hil_protocol_fault(format!(
            "timed out waiting for HIL response tick {} time {}us",
            outstanding.sequence, outstanding.sim_time_us
        ));
        self.outstanding_hil_source = None;
        self.refresh_hil_comparison_state();
    }

    fn record_hil_protocol_fault(&mut self, fault: impl Into<String>) {
        let fault = fault.into();
        self.hil_protocol_fault_count = self.hil_protocol_fault_count.saturating_add(1);
        self.latest_hil_protocol_fault = Some(fault);
    }

    fn current_hilink_send_time_ms(&self) -> u32 {
        self.gazebo_client
            .stats()
            .last_sensor_time_us
            .map(sim_time_us_to_send_time_ms)
            .unwrap_or_else(|| self.app_started_at.elapsed().as_millis() as u32)
    }

    fn fill_empty_sim_stamp(&self, sim_tick: u64, sim_time_us: u64) -> (u64, u64) {
        if sim_tick != 0 || sim_time_us != 0 {
            return (sim_tick, sim_time_us);
        }

        let gazebo_stats = self.gazebo_client.stats();
        match (
            gazebo_stats.last_sensor_sequence,
            gazebo_stats.last_sensor_time_us,
        ) {
            (Some(latest_tick), Some(latest_time_us)) => (latest_tick, latest_time_us),
            _ => (sim_tick, sim_time_us),
        }
    }

    fn uart_state_from(uart_service: &UartService) -> super::app_state::UartState {
        super::app_state::UartState {
            available_ports: uart_service.available_ports().to_vec(),
            connected: uart_service.is_open(),
            selected_port: uart_service.port_name().map(ToString::to_string),
            baud_rate: if uart_service.is_open() {
                uart_service.baud_rate()
            } else {
                DEFAULT_UART_BAUD_RATE
            },
            line_coding: uart_service.line_coding_label().to_string(),
            last_error: uart_service.last_error().map(ToString::to_string),
        }
    }

    fn default_gazebo_bridge_process_state() -> GazeboBridgeProcessState {
        GazeboBridgeProcessState {
            launch_path: gazebo_bridge_executable_path().display().to_string(),
            ..GazeboBridgeProcessState::default()
        }
    }

    fn track_pending_command(&mut self, seq: u16, msg_type: u8, label: String) {
        self.pending_commands.push_back(PendingHilinkCommand {
            seq,
            msg_type,
            label,
            sent_elapsed_ms: self.app_started_at.elapsed().as_millis(),
            status: PendingCommandStatus::Waiting,
        });

        while self.pending_commands.len() > PENDING_COMMAND_MAX_ITEMS {
            self.pending_commands.pop_front();
        }

        self.sync_command_state();
    }

    fn drain_hilink_events(&mut self) {
        for event in self.hilink_parser.drain_events() {
            self.apply_hilink_event(event);
        }
        self.sync_command_state();
    }

    fn apply_hilink_event(&mut self, event: HilinkEvent) {
        match event {
            HilinkEvent::Pong { seq } => {
                self.command_state.pong_count = self.command_state.pong_count.saturating_add(1);
                if let Some(command) = self.pending_commands.iter_mut().find(|command| {
                    command.msg_type == HilinkCommand::Ping.msg_type_id()
                        && matches!(command.status, PendingCommandStatus::Waiting)
                }) {
                    command.status = PendingCommandStatus::Pong;
                    self.command_state.last_event = Some(format!(
                        "Pong seq {seq} matched pending Ping seq {}",
                        command.seq
                    ));
                } else {
                    self.command_state.last_event = Some(format!("Pong seq {seq} unsolicited"));
                }
            }
            HilinkEvent::Ack {
                seq,
                acked_seq,
                acked_msg_type,
                status,
            } => {
                self.command_state.ack_count = self.command_state.ack_count.saturating_add(1);
                self.apply_ack_status(
                    acked_seq,
                    acked_msg_type,
                    PendingCommandStatus::Ack { status },
                );
                self.command_state.last_event = Some(format!(
                    "Ack seq {seq} for {} seq {acked_seq} status {status}",
                    hilink_msg_type_label(acked_msg_type)
                ));
            }
            HilinkEvent::Nack {
                seq,
                rejected_seq,
                rejected_msg_type,
                reason,
            } => {
                self.command_state.nack_count = self.command_state.nack_count.saturating_add(1);
                self.apply_ack_status(
                    rejected_seq,
                    rejected_msg_type,
                    PendingCommandStatus::Nack { reason },
                );
                self.command_state.last_event = Some(format!(
                    "Nack seq {seq} for {} seq {rejected_seq} reason {reason}",
                    hilink_msg_type_label(rejected_msg_type)
                ));
            }
            HilinkEvent::Gps { seq } => {
                self.command_state.gps_count = self.command_state.gps_count.saturating_add(1);
                self.command_state.last_event = Some(format!("Gps seq {seq} unsolicited"));
            }
            HilinkEvent::TelemetrySnapshot { seq } => {
                self.command_state.telemetry_snapshot_count = self
                    .command_state
                    .telemetry_snapshot_count
                    .saturating_add(1);
                self.command_state.last_event =
                    Some(format!("TelemetrySnapshot seq {seq} unsolicited"));
            }
            HilinkEvent::HilReady { seq } => {
                self.hil_ready = true;
                self.command_state.last_event = Some(format!("HilReady seq {seq}"));
                self.refresh_hil_comparison_state();
            }
            HilinkEvent::Heartbeat { seq } => {
                self.hil_ready = true;
                self.command_state.last_event = Some(format!("Heartbeat seq {seq}"));
                self.refresh_hil_comparison_state();
            }
        }
    }

    fn apply_ack_status(&mut self, seq: u16, msg_type: u8, status: PendingCommandStatus) {
        if let Some(command) = self
            .pending_commands
            .iter_mut()
            .find(|command| command.seq == seq && command.msg_type == msg_type)
        {
            command.status = status;
        }
    }

    fn sync_command_state(&mut self) {
        self.command_state.pending = self.pending_commands.iter().cloned().collect();
        self.state.hilink_commands = self.command_state.clone();
    }
}

impl Drop for AppBridge {
    fn drop(&mut self) {
        if let Some(mut child) = self.gazebo_bridge_process.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

fn gazebo_bridge_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../gazebo_bridge")
}

fn gazebo_bridge_executable_path() -> PathBuf {
    gazebo_bridge_dir()
        .join("build")
        .join("cerberus_gazebo_bridge")
}

fn gazebo_bridge_config_path() -> PathBuf {
    gazebo_bridge_dir().join("config").join("bridge_config")
}

fn parse_endpoint_port(endpoint: &str) -> Option<u16> {
    endpoint.rsplit_once(':')?.1.parse().ok()
}

fn sim_time_us_to_send_time_ms(sim_time_us: u64) -> u32 {
    (sim_time_us / 1_000).min(u64::from(u32::MAX)) as u32
}

fn sanitized_sensor_valid_flags(frame: &GazeboSensorFrame) -> u32 {
    let mut flags = if frame.valid_flags != 0 {
        frame.valid_flags
    } else {
        SENSOR_VALID_ACCEL
            | SENSOR_VALID_GYRO
            | SENSOR_VALID_MAG
            | SENSOR_VALID_BARO
            | SENSOR_VALID_GPS
    };

    if !finite_f32x3(frame.accel_mps2) {
        flags &= !SENSOR_VALID_ACCEL;
    }
    if !finite_f32x3(frame.gyro_rps) {
        flags &= !SENSOR_VALID_GYRO;
    }
    if !finite_f32x3(frame.mag_ut) {
        flags &= !SENSOR_VALID_MAG;
    }
    if !frame.pressure_pa.is_finite()
        || !frame.baro_altitude_m.is_finite()
        || !frame.temperature_c.is_finite()
    {
        flags &= !SENSOR_VALID_BARO;
    }
    if !frame.lat_deg.is_finite()
        || !frame.lon_deg.is_finite()
        || !frame.alt_msl_m.is_finite()
        || !finite_f32x3(frame.vel_ned_mps)
    {
        flags &= !SENSOR_VALID_GPS;
    }

    flags
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn hil_response_allows_motion(flags: u32) -> bool {
    if flags & RESPONSE_FLAG_BENCH_OVERRIDE != 0 {
        return flags & RESPONSE_FLAG_MOTORS_VALID != 0
            && flags & RESPONSE_FLAG_SENSOR_INPUT_VALID != 0
            && flags & RESPONSE_FLAG_FAILSAFE == 0;
    }

    flags & RESPONSE_FLAG_FAILSAFE == 0
        && flags & RESPONSE_FLAG_ARMED != 0
        && flags & RESPONSE_FLAG_MOTORS_VALID != 0
        && flags & RESPONSE_FLAG_ESTIMATOR_VALID != 0
        && flags & RESPONSE_FLAG_CONTROL_VALID != 0
        && flags & RESPONSE_FLAG_SENSOR_INPUT_VALID != 0
}

fn format_serial_monitor_line(
    direction: &str,
    elapsed_ms: u128,
    byte_offset: usize,
    bytes: &[u8],
) -> String {
    let mut hex = String::new();
    let mut ascii = String::new();

    for byte in bytes {
        let _ = write!(hex, "{byte:02X} ");
        ascii.push(if byte.is_ascii_graphic() || *byte == b' ' {
            char::from(*byte)
        } else {
            '.'
        });
    }

    format!(
        "{direction} +{elapsed_ms:>8}ms @{byte_offset:04X} {:>2} bytes  {:<72} |{ascii}|",
        bytes.len(),
        hex
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hil_comparison_is_driven_by_fresh_gazebo_source_data() {
        let mut bridge = AppBridge::new();

        bridge.refresh_hil_comparison_state();
        assert!(!bridge.state.hil_comparison.source_active);
        assert!(bridge.state.hil_comparison.latest_source.is_none());

        bridge.record_hil_source(GazeboSensorFrame {
            sequence: 42,
            sim_time_us: 840_000,
            clock_source: Some("gazebo".to_string()),
            valid_flags: 31,
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rps: [0.1, 0.2, 0.3],
            orientation_quat: Some([1.0, 0.0, 0.0, 0.0]),
            mag_ut: [21.1, -4.2, 42.8],
            pressure_pa: 99_123.5,
            baro_altitude_m: 182.25,
            temperature_c: 23.5,
            lat_deg: 30.2672345,
            lon_deg: -97.7431567,
            alt_msl_m: 181.75,
            vel_ned_mps: [1.25, -2.5, 0.75],
            sats: 14,
            fix_type: 3,
            battery_voltage_v: 12.3,
            rssi_dbm: 0,
            snr_db_x100: 0,
            loss_pct_x100: 0,
        });
        bridge.latest_hil_response = Some(HilResponseFrame {
            sim_tick: 42,
            sim_time_us: 840_000,
            flags: RESPONSE_FLAG_ARMED | RESPONSE_FLAG_MOTORS_VALID,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [1200, 1201, 1202, 1203],
        });

        bridge.refresh_hil_comparison_state();
        assert!(bridge.state.hil_comparison.source_active);
        assert!(bridge.state.hil_comparison.matched);
        assert_eq!(bridge.state.hil_comparison.sim_time_delta_us, Some(0));
        assert_eq!(
            bridge
                .state
                .hil_comparison
                .latest_source
                .as_ref()
                .and_then(|source| source.clock_source.as_deref()),
            Some("gazebo")
        );

        bridge.last_hil_source_at =
            Some(Instant::now() - HIL_SOURCE_FRESH_FOR - Duration::from_millis(1));
        bridge.refresh_hil_comparison_state();
        assert!(!bridge.state.hil_comparison.source_active);
        assert!(bridge.state.hil_comparison.latest_source.is_none());
        assert!(bridge.state.hil_comparison.latest_response.is_none());
    }

    #[test]
    fn hil_response_validator_accepts_matching_armed_motor_response() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;
        bridge.record_hil_source(test_gazebo_frame(42, 840_000));
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now(),
        });

        let command = HilinkActuatorCommand {
            sim_tick: 42,
            sim_time_us: 840_000,
            flags: RESPONSE_FLAG_ARMED
                | RESPONSE_FLAG_ESTIMATOR_VALID
                | RESPONSE_FLAG_MOTORS_VALID
                | RESPONSE_FLAG_CONTROL_VALID
                | RESPONSE_FLAG_SENSOR_INPUT_VALID,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [1000, 1001, 1002, 1003],
        };

        assert_eq!(
            bridge.validated_hil_motor_command(&command),
            Some([1000, 1001, 1002, 1003])
        );
        assert_eq!(bridge.hil_protocol_fault_count, 0);
    }

    #[test]
    fn hil_response_validator_accepts_bench_override_while_disarmed() {
        let mut bridge = AppBridge::new();
        bridge.record_hil_source(test_gazebo_frame(42, 840_000));
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now(),
        });

        let command = HilinkActuatorCommand {
            sim_tick: 42,
            sim_time_us: 840_000,
            flags: RESPONSE_FLAG_MOTORS_VALID
                | RESPONSE_FLAG_CONTROL_VALID
                | RESPONSE_FLAG_SENSOR_INPUT_VALID
                | RESPONSE_FLAG_BENCH_OVERRIDE,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [32_000, 0, 0, 0],
        };

        assert_eq!(
            bridge.validated_hil_motor_command(&command),
            Some([32_000, 0, 0, 0])
        );
        assert_eq!(bridge.hil_protocol_fault_count, 0);
    }

    #[test]
    fn hil_response_validator_rejects_mismatched_stamp() {
        let mut bridge = AppBridge::new();
        bridge.record_hil_source(test_gazebo_frame(42, 840_000));
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now(),
        });

        let command = HilinkActuatorCommand {
            sim_tick: 43,
            sim_time_us: 860_000,
            flags: RESPONSE_FLAG_ARMED | RESPONSE_FLAG_MOTORS_VALID,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [1000; 4],
        };

        assert_eq!(bridge.validated_hil_motor_command(&command), None);
        assert_eq!(bridge.hil_protocol_fault_count, 1);
        assert!(bridge
            .latest_hil_protocol_fault
            .as_deref()
            .is_some_and(|fault| fault.contains("mismatched")));
    }

    #[test]
    fn hil_response_validator_forces_zero_when_disarmed() {
        let mut bridge = AppBridge::new();
        bridge.record_hil_source(test_gazebo_frame(42, 840_000));
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now(),
        });

        let command = HilinkActuatorCommand {
            sim_tick: 42,
            sim_time_us: 840_000,
            flags: RESPONSE_FLAG_MOTORS_VALID,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [1000, 1001, 1002, 1003],
        };

        assert_eq!(bridge.validated_hil_motor_command(&command), Some([0; 4]));
        assert_eq!(bridge.hil_protocol_fault_count, 1);
    }

    #[test]
    fn gazebo_forwarding_waits_for_hil_ready() {
        let mut bridge = AppBridge::new();
        bridge
            .gazebo_client
            .push_test_sensor_frame(test_gazebo_frame(42, 840_000));

        bridge.forward_gazebo_sensors_to_fc();

        assert!(bridge.latest_hil_source.is_some());
        assert!(bridge.outstanding_hil_source.is_none());
        assert_eq!(bridge.hil_protocol_fault_count, 0);
    }

    #[test]
    fn heartbeat_recovers_hil_ready_state_when_ready_packet_was_missed() {
        let mut bridge = AppBridge::new();

        bridge.apply_hilink_event(HilinkEvent::Heartbeat { seq: 10 });

        assert!(bridge.hil_ready);
        assert_eq!(
            bridge.command_state.last_event.as_deref(),
            Some("Heartbeat seq 10")
        );
    }

    #[test]
    fn waypoint_commands_report_inactive_hil_source_before_uart_write() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;

        let error = bridge
            .send_hilink_control_waypoint(0, 0, 26.0, -98.0, 30.0, 0.0)
            .unwrap_err();

        assert!(error.contains("HIL sensor source is inactive"));
    }

    #[test]
    fn waypoint_commands_require_valid_hil_gps_reference() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;
        let mut frame = test_gazebo_frame(42, 840_000);
        frame.fix_type = 1;
        frame.sats = 3;
        bridge.record_hil_source(frame);

        let error = bridge
            .send_hilink_mission_waypoint(0, 0, 26.0, -98.0, 30.0, 0.0)
            .unwrap_err();

        assert!(error.contains("HIL GPS reference is not valid enough"));
    }

    #[test]
    fn gazebo_forwarding_holds_new_frames_while_response_is_outstanding() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now(),
        });
        bridge
            .gazebo_client
            .push_test_sensor_frame(test_gazebo_frame(43, 860_000));

        bridge.forward_gazebo_sensors_to_fc();

        let outstanding = bridge.outstanding_hil_source.unwrap();
        assert_eq!(outstanding.sequence, 42);
        assert_eq!(outstanding.sim_time_us, 840_000);
        assert_eq!(
            bridge
                .latest_hil_source
                .as_ref()
                .map(|source| source.sequence),
            Some(43)
        );
        assert_eq!(bridge.hil_protocol_fault_count, 0);
    }

    #[test]
    fn gazebo_forwarding_skips_duplicate_hil_ticks_after_response() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;
        bridge.last_forwarded_hil_source_stamp = Some(HilSourceStamp {
            sequence: 42,
            sim_time_us: 840_000,
        });
        bridge
            .gazebo_client
            .push_test_sensor_frame(test_gazebo_frame(42, 860_000));

        bridge.forward_gazebo_sensors_to_fc();

        assert!(bridge.outstanding_hil_source.is_none());
        assert_eq!(
            bridge
                .latest_hil_source
                .as_ref()
                .map(|source| (source.sequence, source.sim_time_us)),
            Some((42, 860_000))
        );
        assert_eq!(bridge.hil_protocol_fault_count, 0);
    }

    #[test]
    fn gazebo_forwarding_resyncs_after_source_clock_reset() {
        let mut bridge = AppBridge::new();
        bridge.hil_ready = true;
        bridge.last_forwarded_hil_source_stamp = Some(HilSourceStamp {
            sequence: 58_460,
            sim_time_us: 1_169_200_000,
        });
        bridge.latest_hil_response = Some(HilResponseFrame {
            sim_tick: 58_460,
            sim_time_us: 1_169_200_000,
            flags: RESPONSE_FLAG_ARMED | RESPONSE_FLAG_MOTORS_VALID,
            position_ned_m: [0.0; 3],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [0; 4],
        });
        bridge.latest_forwarded_actuator = Some(HilForwardedActuatorFrame {
            sequence: 58_460,
            sim_time_us: 1_169_200_000,
            motor_cmd: [0; 4],
        });
        bridge
            .completed_hil_responses
            .push_back((58_460, 1_169_200_000));
        bridge
            .gazebo_client
            .push_test_sensor_frame(test_gazebo_frame(4_476, 89_526_000));

        bridge.forward_gazebo_sensors_to_fc();

        assert!(bridge.outstanding_hil_source.is_none());
        assert_eq!(bridge.last_forwarded_hil_source_stamp, None);
        assert!(bridge.latest_hil_response.is_none());
        assert!(bridge.latest_forwarded_actuator.is_none());
        assert!(bridge.completed_hil_responses.is_empty());
        assert!(bridge
            .latest_hil_protocol_fault
            .as_deref()
            .is_some_and(|fault| fault.contains("resynced HIL source")));
    }

    #[test]
    fn hil_response_timeout_releases_outstanding_source() {
        let mut bridge = AppBridge::new();
        bridge.outstanding_hil_source = Some(OutstandingHilSource {
            sequence: 42,
            sim_time_us: 840_000,
            sent_at: Instant::now() - HIL_RESPONSE_TIMEOUT - Duration::from_millis(1),
        });

        bridge.expire_hil_response_timeout();

        assert!(bridge.outstanding_hil_source.is_none());
        assert_eq!(bridge.hil_protocol_fault_count, 1);
        assert!(bridge
            .latest_hil_protocol_fault
            .as_deref()
            .is_some_and(|fault| fault.contains("timed out waiting for HIL response")));
    }

    fn test_gazebo_frame(sequence: u64, sim_time_us: u64) -> GazeboSensorFrame {
        GazeboSensorFrame {
            sequence,
            sim_time_us,
            clock_source: Some("gazebo".to_string()),
            valid_flags: 31,
            accel_mps2: [1.0, 2.0, -9.81],
            gyro_rps: [0.1, 0.2, 0.3],
            orientation_quat: Some([1.0, 0.0, 0.0, 0.0]),
            mag_ut: [21.1, -4.2, 42.8],
            pressure_pa: 99_123.5,
            baro_altitude_m: 182.25,
            temperature_c: 23.5,
            lat_deg: 30.2672345,
            lon_deg: -97.7431567,
            alt_msl_m: 181.75,
            vel_ned_mps: [1.25, -2.5, 0.75],
            sats: 14,
            fix_type: 3,
            battery_voltage_v: 12.3,
            rssi_dbm: 0,
            snr_db_x100: 0,
            loss_pct_x100: 0,
        }
    }
}
