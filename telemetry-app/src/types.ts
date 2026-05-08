export type ViewId = "dashboard" | "mission" | "hil" | "actuators" | "diagnostics";

export type TelemetryField = {
  group: string;
  parameter: string;
  value: string;
  unit: string;
};

export type AppState = {
  current_time: string;
  gazebo_bridge: {
    endpoint: string;
    connected: boolean;
    connected_for_secs: number | null;
    connection_attempts: number;
    actuator_frames_sent: number;
    sensor_frames_received: number;
    last_sensor_sequence: number | null;
    last_sensor_time_us: number | null;
    last_sensor_clock_source: string | null;
    last_error: string | null;
  };
  gazebo_bridge_process: {
    launch_path: string;
    running: boolean;
    pid: number | null;
    last_error: string | null;
    last_exit_status: string | null;
  };
  uart: {
    available_ports: { port_name: string; display_name: string }[];
    connected: boolean;
    selected_port: string | null;
    baud_rate: number;
    line_coding: string;
    last_error: string | null;
  };
  hil_comparison: {
    source_active: boolean;
    hil_ready: boolean;
    outstanding_source: {
      sequence: number;
      sim_time_us: number;
    } | null;
    latest_source: {
      sequence: number;
      sim_time_us: number;
      clock_source: string | null;
      accel_mps2: [number, number, number];
      gyro_rps: [number, number, number];
      orientation_quat: [number, number, number, number] | null;
      mag_ut: [number, number, number];
      pressure_pa: number;
      baro_altitude_m: number;
      temperature_c: number;
      lat_deg: number;
      lon_deg: number;
      alt_msl_m: number;
      vel_ned_mps: [number, number, number];
      sats: number;
      fix_type: number;
    } | null;
    latest_response: {
      sim_tick: number;
      sim_time_us: number;
      flags: number;
      motor_cmd: [number, number, number, number];
    } | null;
    latest_forwarded_actuator: {
      sequence: number;
      sim_time_us: number;
      motor_cmd: [number, number, number, number];
    } | null;
    matched: boolean;
    sim_time_delta_us: number | null;
    protocol_fault_count: number;
    latest_protocol_fault: string | null;
  };
  hilink: {
    fields: TelemetryField[];
    rx_frames: number;
    tx_frames: number;
    parse_errors: number;
    last_error: string | null;
    last_message: string | null;
  };
  serial_monitor: {
    lines: string[];
    dropped_lines: number;
    parse_enabled: boolean;
    parsed_lines: string[];
    dropped_parsed_lines: number;
  };
  hilink_commands: {
    pending: {
      seq: number;
      msg_type: number;
      label: string;
      sent_elapsed_ms: number;
      status: Record<string, unknown> | string;
    }[];
    last_event: string | null;
    pong_count: number;
    ack_count: number;
    nack_count: number;
    gps_count: number;
    telemetry_snapshot_count: number;
  };
};

export type BackendCommand = <TArgs extends Record<string, unknown> | undefined = undefined>(
  name: string,
  args?: TArgs,
) => Promise<void>;

export type TelemetryLookup = {
  field: (group: string, parameter: string, fallback?: string) => string;
  fieldUnit: (group: string, parameter: string) => string;
};
