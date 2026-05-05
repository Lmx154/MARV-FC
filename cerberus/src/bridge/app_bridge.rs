use std::collections::VecDeque;
use std::fmt::Write as _;
use std::time::{Duration, Instant};

use crate::backend::{
    encode_hilink_command, hilink_msg_type_label, parse_hilink_frames_for_display,
    BenchEnableCommand, CvWaypointCommand, DshotCommand, GlobalWaypointCommand,
    HilSensorFrameCommand, HilinkCommand, HilinkEvent, HilinkParserService, MotorSweepCommand,
    MotorTestCommand, TimeService, TofWaypointCommand, UartPortInfo, UartService,
    DEFAULT_UART_BAUD_RATE,
};

use super::app_state::{
    HilinkCommandState, OperationMode, PendingCommandStatus, PendingHilinkCommand,
};
use super::gazebo_bridge_client::{GazeboBridgeClient, GazeboSensorFrame};
use super::AppState;

const SERIAL_MONITOR_MAX_LINES: usize = 600;
const SERIAL_MONITOR_BYTES_PER_LINE: usize = 24;
const SERIAL_PARSE_MAX_LINES: usize = 600;
const PENDING_COMMAND_MAX_ITEMS: usize = 64;

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
    operation_mode: OperationMode,
    pending_commands: VecDeque<PendingHilinkCommand>,
    command_state: HilinkCommandState,
    state: AppState,
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
        let operation_mode = OperationMode::Hil;

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
            operation_mode,
            pending_commands: VecDeque::new(),
            command_state: HilinkCommandState::default(),
            state: AppState {
                current_time,
                operation_mode,
                gazebo_bridge: gazebo_client.stats(),
                uart,
                hilink,
                serial_monitor: Default::default(),
                hilink_commands: Default::default(),
            },
            app_started_at: Instant::now(),
            last_refresh: Instant::now(),
            refresh_interval: Duration::from_millis(250),
            gazebo_client,
        }
    }

    pub fn tick(&mut self) {
        self.uart_service.poll_connection();
        self.poll_uart_hilink();
        self.gazebo_client.poll();
        if self.operation_mode.uses_gazebo_bridge() {
            self.forward_gazebo_sensors_to_fc();
        }

        if self.last_refresh.elapsed() >= self.refresh_interval {
            self.state.current_time = self.time_service.current_time();
            self.refresh_uart_state();
            self.refresh_gazebo_state();
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
        self.refresh_uart_state();
        result
    }

    pub fn close_uart(&mut self) {
        self.uart_service.close();
        self.refresh_uart_state();
    }

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
        if !self.operation_mode.uses_gazebo_bridge() {
            let error = "Gazebo bridge is only available in HIL operation".to_string();
            self.state.gazebo_bridge.last_error = Some(error.clone());
            return Err(error);
        }

        self.gazebo_client.set_endpoint(endpoint.to_string());
        let result = self.gazebo_client.connect();
        self.refresh_gazebo_state();
        result
    }

    pub fn set_operation_mode(&mut self, operation_mode: OperationMode) {
        if self.operation_mode == operation_mode {
            return;
        }

        self.operation_mode = operation_mode;
        self.state.operation_mode = operation_mode;

        if !operation_mode.uses_gazebo_bridge() && self.gazebo_client.stats().connected {
            self.gazebo_client.disconnect();
            self.refresh_gazebo_state();
        }
    }

    pub fn disconnect_gazebo_bridge(&mut self) {
        self.gazebo_client.disconnect();
        self.refresh_gazebo_state();
    }

    pub fn send_test_actuator_command(&mut self, motor_speed: f32) -> Result<(), String> {
        let result = self.gazebo_client.send_test_actuator_command(motor_speed);
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

    pub fn send_hilink_control_waypoint(
        &mut self,
        ref_sim_tick: u64,
        ref_sim_time_us: u64,
        lat_deg: f64,
        lon_deg: f64,
        alt_msl_m: f32,
        yaw_deg: f32,
    ) -> Result<(), String> {
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
        self.refresh_serial_monitor_state();
    }

    fn refresh_gazebo_state(&mut self) {
        self.state.gazebo_bridge = self.gazebo_client.stats();
        self.state.gazebo_bridge.connected_for_secs = self
            .gazebo_client
            .connected_for()
            .map(|duration| duration.as_secs());
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
                if self.operation_mode.uses_gazebo_bridge() {
                    self.forward_hilink_actuators_to_gazebo();
                }
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

    fn forward_hilink_actuators_to_gazebo(&mut self) {
        for command in self.hilink_parser.drain_actuator_commands() {
            if let Err(error) = self.gazebo_client.send_hil_actuator_command(
                command.sim_tick,
                command.sim_time_us,
                command.motor_cmd,
            ) {
                eprintln!("failed to forward HIL actuator command to Gazebo: {error}");
            }
        }
    }

    fn forward_gazebo_sensors_to_fc(&mut self) {
        let frames = self.gazebo_client.drain_sensor_frames();
        if frames.is_empty() {
            return;
        }

        if !self.uart_service.is_open() {
            self.refresh_gazebo_state();
            return;
        }

        for frame in frames {
            let send_time_ms = sim_time_us_to_send_time_ms(frame.sim_time_us);
            let command = HilinkCommand::HilSensorFrame(Self::hil_sensor_frame_from_gazebo(frame));
            if let Err(error) = self.send_hilink_command_at(command, send_time_ms) {
                eprintln!("failed to forward Gazebo sensor frame to FC: {error}");
            }
        }

        self.refresh_gazebo_state();
    }

    fn hil_sensor_frame_from_gazebo(frame: GazeboSensorFrame) -> HilSensorFrameCommand {
        let mut command = HilSensorFrameCommand::default();
        command.sim_tick = frame.sequence;
        command.sim_time_us = frame.sim_time_us;
        command.valid_flags = (1 << 0) | (1 << 1);
        command.accel_mps2 = frame.accel_mps2;
        command.gyro_rps = frame.gyro_rps;
        command
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

fn sim_time_us_to_send_time_ms(sim_time_us: u64) -> u32 {
    (sim_time_us / 1_000).min(u64::from(u32::MAX)) as u32
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
