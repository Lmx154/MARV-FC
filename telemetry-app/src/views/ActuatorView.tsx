import { useState } from "react";

import { HeaderLine } from "../components/ui/HeaderLine";
import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import { Panel } from "../components/ui/Panel";
import type { AppState, BackendCommand } from "../types";
import { findValue } from "../utils/telemetry";

export function ActuatorView({ state, command }: { state: AppState | null; command: BackendCommand }) {
  const [motorMask, setMotorMask] = useState(0x0f);
  const [motorValue, setMotorValue] = useState(1050);
  const [motorMode, setMotorMode] = useState(1);
  const [sweep, setSweep] = useState({ startValue: 1000, endValue: 1200, stepValue: 25, stepDurationMs: 250, zeroBetweenMs: 100, repeatCount: 1 });
  const [dshotCommand, setDshotCommand] = useState(0);
  const motorCommands = [1, 2, 3, 4].map((index) => findValue(state, "Actuators", `COMMAND_DSHOT_${index}`, "0"));
  const uartReady = Boolean(state?.uart.connected);

  return (
    <main className="page actuator-page">
      <section className="safety-banner">
        <div>
          <HeaderLine icon="warning" title="Bench Control & Actuator Override" danger />
          <p>Ensure propellers are removed and the vehicle is secured before executing motor sweeps or direct DShot commands.</p>
        </div>
        <div className="bench-actions">
          <span>Bench</span>
          <button disabled={!uartReady} onClick={() => command("send_hilink_bench_disable")}>
            Disable
          </button>
          <button className="danger" disabled={!uartReady} onClick={() => command("send_hilink_bench_enable", { timeoutMs: 30000 })}>
            Enable
          </button>
        </div>
      </section>

      <div className="actuator-layout">
        <Panel title="Actuator Status" icon="flag">
          <Metric label="Bench Armed" value={findValue(state, "Actuators", "BENCH_ARMED", "0")} mono />
          <Metric label="Bench Enabled" value={findValue(state, "Actuators", "BENCH_ENABLED", "0")} mono />
          <Metric label="Active Mask" value={findValue(state, "Actuators", "ACTIVE_MOTOR_MASK", "--")} mono />
          <Metric label="Command Age" value={`${findValue(state, "Actuators", "LAST_COMMAND_AGE_MS", "--")} ms`} mono />
          <Metric label="Timeout" value={`${findValue(state, "Actuators", "BENCH_TIMEOUT_MS", "--")} ms`} mono />
          <Metric label="Flags" value={findValue(state, "Actuators", "ACTUATOR_FLAGS", "none")} mono />
          <div className="command-row compact">
            <button disabled={!uartReady} onClick={() => command("send_hilink_actuator_status_request")}>
              Request Status
            </button>
          </div>
        </Panel>

        <div className="control-column">
          <Panel title="Motor Test" icon="terminal">
            <div className="control-grid">
              <label>
                <span>Target Mask</span>
                <input value={`0x${motorMask.toString(16).toUpperCase().padStart(2, "0")}`} readOnly />
              </label>
              <div className="mask-buttons">
                {[
                  ["ALL", 0x0f],
                  ["M1", 0x01],
                  ["M2", 0x02],
                  ["M3", 0x04],
                  ["M4", 0x08],
                ].map(([label, value]) => (
                  <button key={label} className={motorMask === value ? "active" : ""} onClick={() => setMotorMask(value as number)}>
                    {label}
                  </button>
                ))}
              </div>
              <label>
                <span>Cmd Mode</span>
                <select value={motorMode} onChange={(event) => setMotorMode(Number(event.currentTarget.value))}>
                  <option value={1}>DSHOT_DIRECT</option>
                  <option value={2}>NORMALIZED</option>
                  <option value={0}>STOP</option>
                </select>
              </label>
              <label>
                <span>Target Val</span>
                <input type="number" value={motorValue} onChange={(event) => setMotorValue(Number(event.currentTarget.value))} />
              </label>
            </div>
            <div className="command-row">
              <button disabled={!uartReady} onClick={() => command("send_hilink_motor_stop")}>
                Stop
              </button>
              <button
                className="primary"
                disabled={!uartReady}
                onClick={() => command("send_hilink_motor_test_values", { motorMask, mode: motorMode, value: motorValue, durationMs: 1000, rampMs: 0 })}
              >
                <Icon name="send" />
                Transmit Test
              </button>
            </div>
          </Panel>

          <Panel title="Sweep and DShot" icon="settings_input_component">
            <div className="control-grid">
              <NumberInput label="Start" value={sweep.startValue} onChange={(startValue) => setSweep({ ...sweep, startValue })} />
              <NumberInput label="End" value={sweep.endValue} onChange={(endValue) => setSweep({ ...sweep, endValue })} />
              <NumberInput label="Step" value={sweep.stepValue} onChange={(stepValue) => setSweep({ ...sweep, stepValue })} />
              <NumberInput label="Step Ms" value={sweep.stepDurationMs} onChange={(stepDurationMs) => setSweep({ ...sweep, stepDurationMs })} />
              <NumberInput label="Zero Ms" value={sweep.zeroBetweenMs} onChange={(zeroBetweenMs) => setSweep({ ...sweep, zeroBetweenMs })} />
              <NumberInput label="Repeat" value={sweep.repeatCount} onChange={(repeatCount) => setSweep({ ...sweep, repeatCount })} />
              <NumberInput label="DShot Cmd" value={dshotCommand} onChange={setDshotCommand} />
            </div>
            <div className="command-row">
              <button
                disabled={!uartReady}
                onClick={() => command("send_hilink_dshot_command", { motorMask, command: dshotCommand, repeatCount: sweep.repeatCount })}
              >
                DShot
              </button>
              <button
                className="primary"
                disabled={!uartReady}
                onClick={() => command("send_hilink_motor_sweep_values", { motorMask, mode: motorMode, ...sweep })}
              >
                Sweep
              </button>
            </div>
          </Panel>

          <Panel title="Actuator Telemetry Grid" icon="query_stats">
            <table className="status-table">
              <thead>
                <tr>
                  <th>ID</th>
                  <th>State</th>
                  <th>CMD TGT</th>
                  <th>Age</th>
                  <th>Error</th>
                </tr>
              </thead>
              <tbody>
                {motorCommands.map((motor, index) => (
                  <tr key={index}>
                    <td>MTR_0{index + 1}</td>
                    <td>
                      <span className="pill">IDLE</span>
                    </td>
                    <td>{motor}</td>
                    <td>{findValue(state, "Actuators", "LAST_COMMAND_AGE_MS", "--")} ms</td>
                    <td>-</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </Panel>
        </div>
      </div>
    </main>
  );
}

function NumberInput({ label, value, onChange }: { label: string; value: number; onChange: (value: number) => void }) {
  return (
    <label>
      <span>{label}</span>
      <input type="number" value={value} onChange={(event) => onChange(Number(event.currentTarget.value))} />
    </label>
  );
}
