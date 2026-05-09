import { useEffect, useState } from "react";

import type { AppState, BackendCommand, TelemetryLookup } from "../types";
import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import { Panel } from "../components/ui/Panel";
import { StatBox } from "../components/ui/StatBox";
import { formatSeconds } from "../utils/format";

export function Dashboard({
  state,
  lookup,
  command,
  commandStatus,
  onRefreshPorts,
}: {
  state: AppState | null;
  lookup: TelemetryLookup;
  command: BackendCommand;
  commandStatus: string;
  onRefreshPorts: () => Promise<void>;
}) {
  const { field, fieldUnit } = lookup;
  const protocol = state?.hilink.last_message ?? "HiLink standby";
  const gpsFix = field("Navigation", "GPS_FIX_TYPE", "No fix");
  const sats = field("Navigation", "GPS_SAT_COUNT", "--");

  return (
    <main className="page dashboard-grid">
      <ConnectionPanel state={state} command={command} onRefreshPorts={onRefreshPorts} />

      <section className="center-stack">
        <div className="split-panel">
          <Panel title="GPS Position" icon="satellite" accent={`${sats} SAT`}>
            <Metric label="Lat" value={`${field("Navigation", "GPS_LAT")} deg`} large />
            <Metric label="Lon" value={`${field("Navigation", "GPS_LON")} deg`} large />
            <Metric label="Alt" value={`${field("Navigation", "GPS_ALT_MSL")} ${fieldUnit("Navigation", "GPS_ALT_MSL") || "m"}`} large />
            <div className="mini-grid">
              <StatBox label="Vel N" value={`${field("Navigation", "GPS_VEL_NED_X")} m/s`} />
              <StatBox label="Vel E" value={`${field("Navigation", "GPS_VEL_NED_Y")} m/s`} />
              <StatBox label="Vel D" value={`${field("Navigation", "GPS_VEL_NED_Z")} m/s`} />
            </div>
            <small className="muted">{gpsFix}</small>
          </Panel>
          <Panel title="Attitude" icon="flight">
            <Metric label="QW" value={field("Navigation", "ATTITUDE_QUAT_W")} large />
            <Metric label="QX" value={field("Navigation", "ATTITUDE_QUAT_X")} large />
            <Metric label="QY" value={field("Navigation", "ATTITUDE_QUAT_Y")} large />
            <Metric label="QZ" value={field("Navigation", "ATTITUDE_QUAT_Z")} large />
          </Panel>
        </div>
        <div className="split-panel">
          <Panel title="IMU" icon="vibration">
            <Metric label="Accel X" value={`${field("Sensors", "IMU_ACCEL_X")} ${fieldUnit("Sensors", "IMU_ACCEL_X")}`} mono />
            <Metric label="Accel Y" value={`${field("Sensors", "IMU_ACCEL_Y")} ${fieldUnit("Sensors", "IMU_ACCEL_Y")}`} mono />
            <Metric label="Accel Z" value={`${field("Sensors", "IMU_ACCEL_Z")} ${fieldUnit("Sensors", "IMU_ACCEL_Z")}`} mono />
          </Panel>
          <Panel title="Radio Link" icon="cell_tower">
            <Metric label="RSSI" value={`${field("System", "RADIO_RSSI")} ${fieldUnit("System", "RADIO_RSSI") || "dBm"}`} mono />
            <Metric label="SNR" value={`${field("System", "RADIO_SNR")} ${fieldUnit("System", "RADIO_SNR") || "dB"}`} mono />
            <Metric label="Packet Loss" value={`${field("System", "RADIO_LOSS")} ${fieldUnit("System", "RADIO_LOSS") || "%"}`} mono />
          </Panel>
        </div>
        <div className="split-panel">
          <Panel title="Protocol Freshness" icon="sync">
            <Metric label="Last Message" value={protocol} mono />
            <Metric label="RX Frames" value={String(state?.hilink.rx_frames ?? 0)} mono />
            <Metric label="TX Frames" value={String(state?.hilink.tx_frames ?? 0)} mono />
            <Metric label="Parse Errors" value={String(state?.hilink.parse_errors ?? 0)} mono />
          </Panel>
          <Panel title="Estimator Biases" icon="tune">
            <Metric label="Gyro X" value={`${field("System", "GYRO_BIAS_X")} rad/s`} mono />
            <Metric label="Gyro Y" value={`${field("System", "GYRO_BIAS_Y")} rad/s`} mono />
            <Metric label="Gyro Z" value={`${field("System", "GYRO_BIAS_Z")} rad/s`} mono />
            <Metric label="Accel X" value={`${field("System", "ACCEL_BIAS_X")} m/s²`} mono />
            <Metric label="Accel Y" value={`${field("System", "ACCEL_BIAS_Y")} m/s²`} mono />
            <Metric label="Accel Z" value={`${field("System", "ACCEL_BIAS_Z")} m/s²`} mono />
          </Panel>
        </div>
      </section>

      <Panel title="Routine Commands" icon="radio_button_checked" className="span-3 danger-title static-panel">
        <div className="command-stack">
          <button className="command-button" disabled={!state?.uart.connected} onClick={() => command("send_hilink_ping")}>
            <span>Ping</span>
            <Icon name="graphic_eq" />
          </button>
          <button className="command-button danger" disabled={!state?.uart.connected} onClick={() => command("send_hilink_arm")}>
            <span>Arm Vehicle</span>
            <Icon name="lock_open" />
          </button>
          <button className="command-button" disabled={!state?.uart.connected} onClick={() => command("send_hilink_disarm")}>
            <span>Disarm Vehicle</span>
            <Icon name="lock" />
          </button>
          <button className="command-button primary" disabled={!state?.uart.connected} onClick={() => command("send_hilink_rtl")}>
            <span>RTL</span>
            <Icon name="flight_land" fill />
          </button>
          <button className="command-button" disabled={!state?.uart.connected} onClick={() => command("run_radio_link_smoke_test")}>
            <span>Radio Test</span>
            <Icon name="cell_tower" />
          </button>
        </div>
        <div className="panel-footer">
          <Icon name="check_circle" />
          <span>{state?.hilink_commands.last_event ?? commandStatus}</span>
        </div>
      </Panel>
    </main>
  );
}

function ConnectionPanel({
  state,
  command,
  onRefreshPorts,
}: {
  state: AppState | null;
  command: BackendCommand;
  onRefreshPorts: () => Promise<void>;
}) {
  const firstPort = state?.uart.available_ports[0]?.port_name ?? "";
  const [portName, setPortName] = useState(firstPort);
  const [baudRate, setBaudRate] = useState(state?.uart.baud_rate ?? 115200);
  const [gazeboEndpoint, setGazeboEndpoint] = useState(state?.gazebo_bridge.endpoint ?? "127.0.0.1:9000");

  useEffect(() => {
    if (state?.uart.selected_port) setPortName(state.uart.selected_port);
    else if (!portName && firstPort) setPortName(firstPort);
  }, [firstPort, portName, state?.uart.selected_port]);

  useEffect(() => {
    if (state?.uart.baud_rate) setBaudRate(state.uart.baud_rate);
  }, [state?.uart.baud_rate]);

  useEffect(() => {
    if (state?.gazebo_bridge.endpoint) setGazeboEndpoint(state.gazebo_bridge.endpoint);
  }, [state?.gazebo_bridge.endpoint]);

  return (
    <Panel title="Connections" icon="router" className="span-3 static-panel">
      <div className="connection-stack">
        <section>
          <div className="connection-heading">
            <span>UART Vehicle Link</span>
            <b>{state?.uart.connected ? "OPEN" : "CLOSED"}</b>
          </div>
          <label>
            <span>Port</span>
            <select
              value={portName}
              disabled={state?.uart.connected}
              onFocus={onRefreshPorts}
              onClick={onRefreshPorts}
              onChange={(event) => setPortName(event.currentTarget.value)}
            >
              {state?.uart.available_ports.length ? (
                state.uart.available_ports.map((port) => (
                  <option key={port.port_name} value={port.port_name}>
                    {port.display_name || port.port_name}
                  </option>
                ))
              ) : (
                <option value="">No ports detected</option>
              )}
            </select>
          </label>
          <label>
            <span>Baud Rate</span>
            <input
              type="number"
              value={baudRate}
              disabled={state?.uart.connected}
              onChange={(event) => setBaudRate(Number(event.currentTarget.value))}
            />
          </label>
          <div className="command-row compact">
            <button
              className={state?.uart.connected ? "" : "primary"}
              disabled={!state?.uart.connected && !portName}
              onClick={() => {
                if (state?.uart.connected) {
                  command("close_uart");
                } else {
                  command("open_uart", { portName, baudRate });
                }
              }}
            >
              {state?.uart.connected ? "Close" : "Open"}
            </button>
          </div>
          <Metric label="Line Coding" value={state?.uart.line_coding ?? "--"} mono />
          <Metric label="UART Error" value={state?.uart.last_error ?? "none"} mono />
        </section>

        <section>
          <div className="connection-heading">
            <span>Gazebo HIL Source</span>
            <b>{state?.gazebo_bridge_process.running ? "PROCESS RUNNING" : state?.hil_comparison.source_active ? "LIVE" : "IDLE"}</b>
          </div>
          <label>
            <span>Endpoint</span>
            <input value={gazeboEndpoint} onChange={(event) => setGazeboEndpoint(event.currentTarget.value)} />
          </label>
          <div className="command-row compact">
            <button
              onClick={() => {
                if (state?.gazebo_bridge_process.running) {
                  command("stop_gazebo_bridge_process");
                } else {
                  command("start_gazebo_bridge_process", { endpoint: gazeboEndpoint });
                }
              }}
            >
              {state?.gazebo_bridge_process.running ? "Stop Bridge" : "Run Bridge"}
            </button>
            <button
              className={state?.gazebo_bridge.connected ? "" : "primary"}
              disabled={!state?.gazebo_bridge.connected && !gazeboEndpoint}
              onClick={() => {
                if (state?.gazebo_bridge.connected) {
                  command("disconnect_gazebo_bridge");
                } else {
                  command("connect_gazebo_bridge", { endpoint: gazeboEndpoint });
                }
              }}
            >
              {state?.gazebo_bridge.connected ? "Disconnect" : "Connect"}
            </button>
          </div>
          <div className="mini-grid">
            <StatBox label="Uptime" value={formatSeconds(state?.gazebo_bridge.connected_for_secs)} />
            <StatBox label="Sensors" value={String(state?.gazebo_bridge.sensor_frames_received ?? 0)} />
          </div>
          <Metric label="Process" value={state?.gazebo_bridge_process.running ? `pid ${state.gazebo_bridge_process.pid}` : "stopped"} mono />
          <Metric label="Bridge Path" value={state?.gazebo_bridge_process.launch_path ?? "--"} mono />
          <Metric label="Actuator TX" value={String(state?.gazebo_bridge.actuator_frames_sent ?? 0)} mono />
          <Metric label="Gazebo Error" value={state?.gazebo_bridge_process.last_error ?? state?.gazebo_bridge.last_error ?? "none"} mono />
        </section>
      </div>
    </Panel>
  );
}
