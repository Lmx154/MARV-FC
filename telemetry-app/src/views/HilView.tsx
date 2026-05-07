import { useState } from "react";

import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import { MotorBar, MotorDiagram } from "../components/ui/MotorWidgets";
import { PageTitle } from "../components/ui/PageTitle";
import { Panel } from "../components/ui/Panel";
import { StatBox } from "../components/ui/StatBox";
import type { AppState, BackendCommand } from "../types";
import { fmtNum, formatSeconds } from "../utils/format";
import { findValue } from "../utils/telemetry";

export function HilView({ state, command }: { state: AppState | null; command: BackendCommand }) {
  const comparison = state?.hil_comparison;
  const motors = comparison?.latest_forwarded_actuator?.motor_cmd ?? comparison?.latest_response?.motor_cmd ?? [0, 0, 0, 0];
  const source = comparison?.latest_source;
  const response = comparison?.latest_response;
  const deltaMs = comparison?.sim_time_delta_us == null ? "--" : `${(comparison.sim_time_delta_us / 1000).toFixed(3)} ms`;
  const telemetry = (group: string, parameter: string, fallback = "--") => findValue(state, group, parameter, fallback);
  const gazeboReady = Boolean(state?.gazebo_bridge.connected);
  const [bridgeMotorSpeed, setBridgeMotorSpeed] = useState(0.12);
  const [bridgeTestRunning, setBridgeTestRunning] = useState(false);

  const sendBridgeMotorSpeed = async (motorSpeed: number) => {
    await command("send_test_actuator_command", { motorSpeed });
  };

  const runBridgeMotorPulse = async () => {
    setBridgeTestRunning(true);
    try {
      await sendBridgeMotorSpeed(bridgeMotorSpeed);
      window.setTimeout(() => {
        sendBridgeMotorSpeed(0).finally(() => setBridgeTestRunning(false));
      }, 1000);
    } catch {
      setBridgeTestRunning(false);
    }
  };

  const stopBridgeMotorTest = async () => {
    setBridgeTestRunning(false);
    await sendBridgeMotorSpeed(0);
  };

  return (
    <section className="page hil-page">
      <PageTitle
        title="HIL Comparison Lab"
        subtitle="Gazebo source frames are compared with MARV response frames only while the optional HIL source is active."
      />
      <div className="hil-grid">
        <Panel title="HIL Status" icon="router" className="hil-status">
          <div className="status-box">
            <span>Gazebo Bridge</span>
            <strong>
              {state?.gazebo_bridge.connected
                ? "CONNECTED"
                : state?.gazebo_bridge_process.running
                  ? "PROCESS RUNNING"
                  : "STANDBY"}
            </strong>
          </div>
          <div className="mini-grid">
            <StatBox label="Uptime" value={formatSeconds(state?.gazebo_bridge.connected_for_secs)} />
            <StatBox label="Source" value={comparison?.source_active ? "LIVE" : "IDLE"} />
            <StatBox label="MARV Ready" value={comparison?.hil_ready ? "READY" : "WAIT"} />
            <StatBox label="Faults" value={String(comparison?.protocol_fault_count ?? 0)} />
          </div>
          <Metric label="Process" value={state?.gazebo_bridge_process.running ? `pid ${state.gazebo_bridge_process.pid}` : "stopped"} mono />
          <Metric label="Endpoint" value={state?.gazebo_bridge.endpoint ?? "--"} mono />
          <Metric label="RX Sensors" value={String(state?.gazebo_bridge.sensor_frames_received ?? 0)} mono />
          <Metric label="TX Actuators" value={String(state?.gazebo_bridge.actuator_frames_sent ?? 0)} mono />
          <Metric
            label="Outstanding"
            value={
              comparison?.outstanding_source
                ? `${comparison.outstanding_source.sequence} @ ${comparison.outstanding_source.sim_time_us}us`
                : "none"
            }
            mono
          />
          <Metric label="Protocol Fault" value={comparison?.latest_protocol_fault ?? "none"} mono />
          <Metric label="Last Error" value={state?.gazebo_bridge.last_error ?? "none"} mono />
        </Panel>

        <Panel title="Gazebo Motor Link Test" icon="electric_bolt" className="hil-bridge-test" accent="BRIDGE ONLY">
          <div className="control-grid bridge-test-grid">
            <label>
              <span>Motor Speed</span>
              <input
                type="number"
                min={0}
                max={0.35}
                step={0.01}
                value={bridgeMotorSpeed}
                onChange={(event) => setBridgeMotorSpeed(clamp(Number(event.currentTarget.value), 0, 0.35))}
              />
            </label>
            <label>
              <span>Normalized</span>
              <input
                type="range"
                min={0}
                max={0.35}
                step={0.01}
                value={bridgeMotorSpeed}
                onChange={(event) => setBridgeMotorSpeed(clamp(Number(event.currentTarget.value), 0, 0.35))}
              />
            </label>
          </div>
          <div className="command-row compact">
            <button disabled={!gazeboReady} onClick={stopBridgeMotorTest}>
              Stop
            </button>
            <button className="primary" disabled={!gazeboReady || bridgeTestRunning} onClick={runBridgeMotorPulse}>
              <Icon name="play_arrow" />
              Pulse 1s
            </button>
          </div>
          <Metric label="Path" value="GUI -> backend -> bridge -> Gazebo Actuators" mono />
          <Metric label="Bridge TX" value={String(state?.gazebo_bridge.actuator_frames_sent ?? 0)} mono />
          <Metric label="State" value={gazeboReady ? (bridgeTestRunning ? "pulse active" : "ready") : "bridge disconnected"} mono />
        </Panel>

        <Panel title="Simulation Stamp Alignment" icon="compare" className="frame-compare" accent={`Offset ${deltaMs}`}>
          <div className="hil-frame-grid">
            <section className="hil-frame">
              <h4>Gazebo Source</h4>
              <Metric label="Seq" value={String(source?.sequence ?? "--")} mono />
              <Metric label="Sim Time" value={String(source?.sim_time_us ?? "--")} mono />
              <Metric label="Clock" value={source?.clock_source ?? "--"} mono />
            </section>
            <section className="hil-frame">
              <h4>MARV Response</h4>
              <Metric label="Tick" value={String(response?.sim_tick ?? "--")} mono />
              <Metric label="Sim Time" value={String(response?.sim_time_us ?? "--")} mono />
              <Metric label="Matched" value={comparison?.matched ? "yes" : "no"} mono />
            </section>
          </div>
          <div className="timing-bar">
            <span>Tick Alignment</span>
            <div>
              <b style={{ left: comparison?.matched ? "50%" : "35%" }} />
            </div>
            <code>GZ: {source?.sequence ?? "--"}</code>
            <code>MARV: {response?.sim_tick ?? "--"}</code>
          </div>
          {!comparison?.source_active && <p className="inactive-note">No live Gazebo source frame is available, so HIL comparison is idle.</p>}
        </Panel>

        <Panel title="Source Sensor Frame" icon="timeline" className="delta-table">
          <table>
            <thead>
              <tr>
                <th>Signal</th>
                <th>X</th>
                <th>Y</th>
                <th>Z</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Accel m/s2</td>
                <td>{fmtNum(source?.accel_mps2[0])}</td>
                <td>{fmtNum(source?.accel_mps2[1])}</td>
                <td>{fmtNum(source?.accel_mps2[2])}</td>
              </tr>
              <tr>
                <td>Gyro rad/s</td>
                <td>{fmtNum(source?.gyro_rps[0])}</td>
                <td>{fmtNum(source?.gyro_rps[1])}</td>
                <td>{fmtNum(source?.gyro_rps[2])}</td>
              </tr>
              <tr>
                <td>Orientation q</td>
                <td>{fmtNum(source?.orientation_quat?.[0])}</td>
                <td>{fmtNum(source?.orientation_quat?.[1])}</td>
                <td>{fmtNum(source?.orientation_quat?.[2])} / {fmtNum(source?.orientation_quat?.[3])}</td>
              </tr>
              <tr>
                <td>Mag uT</td>
                <td>{fmtNum(source?.mag_ut[0])}</td>
                <td>{fmtNum(source?.mag_ut[1])}</td>
                <td>{fmtNum(source?.mag_ut[2])}</td>
              </tr>
            </tbody>
          </table>
        </Panel>

        <Panel title="Source Navigation Frame" icon="public" className="delta-table">
          <div className="hil-frame-grid">
            <section className="hil-frame">
              <h4>Coordinates</h4>
              <Metric label="Latitude" value={`${fmtNum(source?.lat_deg, 7)} deg`} mono />
              <Metric label="Longitude" value={`${fmtNum(source?.lon_deg, 7)} deg`} mono />
              <Metric label="Alt MSL" value={`${fmtNum(source?.alt_msl_m)} m`} mono />
              <Metric label="GPS Fix" value={`${source?.fix_type ?? "--"} / ${source?.sats ?? "--"} sats`} mono />
            </section>
            <section className="hil-frame">
              <h4>Navigation Sensors</h4>
              <Metric label="Velocity N" value={`${fmtNum(source?.vel_ned_mps[0])} m/s`} mono />
              <Metric label="Velocity E" value={`${fmtNum(source?.vel_ned_mps[1])} m/s`} mono />
              <Metric label="Velocity D" value={`${fmtNum(source?.vel_ned_mps[2])} m/s`} mono />
              <Metric label="Baro Alt" value={`${fmtNum(source?.baro_altitude_m)} m`} mono />
              <Metric label="Pressure" value={`${fmtNum(source?.pressure_pa, 1)} Pa`} mono />
              <Metric label="Temp" value={`${fmtNum(source?.temperature_c, 1)} C`} mono />
            </section>
          </div>
        </Panel>

        <Panel title="MARV Navigation Output" icon="navigation" className="delta-table">
          <table>
            <thead>
              <tr>
                <th>Field</th>
                <th>Gazebo Source</th>
                <th>MARV Telemetry</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Latitude</td>
                <td>{fmtNum(source?.lat_deg, 7)}</td>
                <td>{telemetry("Navigation", "GPS_LAT")}</td>
              </tr>
              <tr>
                <td>Longitude</td>
                <td>{fmtNum(source?.lon_deg, 7)}</td>
                <td>{telemetry("Navigation", "GPS_LON")}</td>
              </tr>
              <tr>
                <td>Altitude MSL</td>
                <td>{fmtNum(source?.alt_msl_m)} m</td>
                <td>{telemetry("Navigation", "GPS_ALT_MSL")} m</td>
              </tr>
              <tr>
                <td>Velocity NED</td>
                <td>{fmtNum(source?.vel_ned_mps[0])}, {fmtNum(source?.vel_ned_mps[1])}, {fmtNum(source?.vel_ned_mps[2])}</td>
                <td>{telemetry("Navigation", "GPS_VEL_NED_X")}, {telemetry("Navigation", "GPS_VEL_NED_Y")}, {telemetry("Navigation", "GPS_VEL_NED_Z")}</td>
              </tr>
              <tr>
                <td>Barometer</td>
                <td>{fmtNum(source?.baro_altitude_m)} m / {fmtNum(source?.pressure_pa, 1)} Pa</td>
                <td>{telemetry("Sensors", "BARO_ALTITUDE")} m / {telemetry("Sensors", "BARO_PRESSURE")} Pa</td>
              </tr>
              <tr>
                <td>Magnetometer</td>
                <td>{fmtNum(source?.mag_ut[0])}, {fmtNum(source?.mag_ut[1])}, {fmtNum(source?.mag_ut[2])}</td>
                <td>{telemetry("Sensors", "MAG_FIELD_X")}, {telemetry("Sensors", "MAG_FIELD_Y")}, {telemetry("Sensors", "MAG_FIELD_Z")}</td>
              </tr>
            </tbody>
          </table>
        </Panel>

        <Panel title="Actuator Feedback" icon="settings_input_component" className="actuator-feedback" accent="FWD TO GZ">
          <MotorDiagram motors={motors as number[]} />
          <div className="motor-bars">
            {motors.map((motor, index) => (
              <MotorBar key={index} label={`M${index + 1}`} value={motor} />
            ))}
          </div>
          <Metric label="Forwarded Seq" value={String(comparison?.latest_forwarded_actuator?.sequence ?? "--")} mono />
          <Metric label="Forwarded Time" value={String(comparison?.latest_forwarded_actuator?.sim_time_us ?? "--")} mono />
        </Panel>
      </div>
    </section>
  );
}

function clamp(value: number, min: number, max: number) {
  if (!Number.isFinite(value)) {
    return min;
  }

  return Math.min(max, Math.max(min, value));
}
