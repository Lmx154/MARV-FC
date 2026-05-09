import { useState } from "react";

import { HeaderLine } from "../components/ui/HeaderLine";
import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import { Panel } from "../components/ui/Panel";
import type { AppState, BackendCommand } from "../types";
import { findValue } from "../utils/telemetry";

const MIXER_MOTORS = [
  { id: 1, roll: "+", pitch: "+", yaw: "-", equation: "T + R + P - Y" },
  { id: 2, roll: "-", pitch: "+", yaw: "+", equation: "T - R + P + Y" },
  { id: 3, roll: "-", pitch: "-", yaw: "-", equation: "T - R - P - Y" },
  { id: 4, roll: "+", pitch: "-", yaw: "+", equation: "T + R - P + Y" },
] as const;

export function ActuatorView({ state, command }: { state: AppState | null; command: BackendCommand }) {
  const [motorMask, setMotorMask] = useState(0x0f);
  const [motorValue, setMotorValue] = useState(1050);
  const [motorMode, setMotorMode] = useState(1);
  const [sweep, setSweep] = useState({ startValue: 1000, endValue: 1200, stepValue: 25, stepDurationMs: 250, zeroBetweenMs: 100, repeatCount: 1 });
  const [dshotCommand, setDshotCommand] = useState(0);
  const [motorOrder, setMotorOrder] = useState<[number, number, number, number]>([1, 2, 3, 4]);
  const motorCommands = [1, 2, 3, 4].map((index) => findValue(state, "Actuators", `COMMAND_DSHOT_${index}`, "0"));
  const uartReady = Boolean(state?.uart.connected);
  const benchEnabled = findValue(state, "Actuators", "BENCH_ENABLED", "0") === "1";
  const benchTimeout = findValue(state, "Actuators", "BENCH_TIMEOUT_MS", "--");
  const benchTimeoutLabel = benchEnabled && benchTimeout === "0" ? "indefinite" : `${benchTimeout} ms`;
  const orderValid = new Set(motorOrder).size === 4;
  const appliedMotorOrder = currentMotorOrder(state, motorOrder);
  const outputMask = (output: number) => 1 << (output - 1);
  const pulseOutput = (output: number) =>
    command("send_hilink_motor_test_values", { motorMask: outputMask(output), mode: motorMode, value: motorValue, durationMs: 750, rampMs: 0 });
  const pulseMixerMotor = (motorIndex: number) => pulseOutput(motorOrder[motorIndex]);
  const setMotorOutput = (motorIndex: number, output: number) => {
    const next = [...motorOrder] as [number, number, number, number];
    next[motorIndex] = output;
    setMotorOrder(next);
  };

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
          <button className="danger" disabled={!uartReady} onClick={() => command("send_hilink_bench_enable", { timeoutMs: 0 })}>
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
          <Metric label="Timeout" value={benchTimeoutLabel} mono />
          <Metric label="Mixer Order" value={findValue(state, "Actuators", "MIXER_MOTOR_ORDER", "M1->O1 M2->O2 M3->O3 M4->O4")} mono />
          <Metric label="Flags" value={findValue(state, "Actuators", "ACTUATOR_FLAGS", "none")} mono />
          <div className="command-row compact">
            <button disabled={!uartReady} onClick={() => command("send_hilink_actuator_status_request")}>
              Request Status
            </button>
          </div>
        </Panel>

        <div className="control-column">
          <Panel title="Motor Test" icon="terminal" className="static-panel">
            <div className="control-grid">
              <label>
                <span>Target Mask</span>
                <input value={`0x${motorMask.toString(16).toUpperCase().padStart(2, "0")}`} readOnly />
              </label>
              <div className="mask-buttons">
                {[
                  ["ALL", 0x0f],
                  ["O1", 0x01],
                  ["O2", 0x02],
                  ["O3", 0x04],
                  ["O4", 0x08],
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
            <div className="identify-grid">
              {[1, 2, 3, 4].map((output) => (
                <button key={output} disabled={!uartReady} onClick={() => pulseOutput(output)}>
                  Pulse O{output}
                </button>
              ))}
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

          <Panel title="Mixer Motor Order" icon="route" className="static-panel">
            <table className="mixer-reference-table">
              <thead>
                <tr>
                  <th>Logical</th>
                  <th>Roll</th>
                  <th>Pitch</th>
                  <th>Yaw</th>
                  <th>Equation</th>
                </tr>
              </thead>
              <tbody>
                {MIXER_MOTORS.map((motor) => (
                  <tr key={motor.id}>
                    <td>M{motor.id}</td>
                    <td>{motor.roll}</td>
                    <td>{motor.pitch}</td>
                    <td>{motor.yaw}</td>
                    <td>{motor.equation}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            <div className="motor-order-grid">
              {MIXER_MOTORS.map((motor, motorIndex) => (
                <label key={motor.id}>
                  <span>
                    M{motor.id} R{motor.roll} P{motor.pitch} Y{motor.yaw}
                  </span>
                  <select value={motorOrder[motorIndex]} onChange={(event) => setMotorOutput(motorIndex, Number(event.currentTarget.value))}>
                    {[1, 2, 3, 4].map((output) => (
                      <option key={output} value={output}>
                        Output {output}
                      </option>
                    ))}
                  </select>
                </label>
              ))}
            </div>
            <div className="identify-grid mixer-pulse-grid">
              {MIXER_MOTORS.map((motor, motorIndex) => (
                <button key={motor.id} disabled={!uartReady || !orderValid} onClick={() => pulseMixerMotor(motorIndex)}>
                  M{motor.id} -&gt; O{motorOrder[motorIndex]}
                </button>
              ))}
            </div>
            <div className="command-row">
              <button disabled={!uartReady} onClick={() => setMotorOrder([1, 2, 3, 4])}>
                Identity
              </button>
              <button disabled={!uartReady} onClick={() => setMotorOrder(appliedMotorOrder)}>
                Current
              </button>
              <button
                className="primary"
                disabled={!uartReady || !orderValid}
                onClick={() => command("send_hilink_mixer_motor_order", { outputForMotor: motorOrder })}
              >
                <Icon name="send" />
                Apply Order
              </button>
            </div>
          </Panel>

          <Panel title="Sweep and DShot" icon="settings_input_component" className="static-panel">
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

function currentMotorOrder(state: AppState | null, fallback: [number, number, number, number]): [number, number, number, number] {
  const order = [1, 2, 3, 4].map((index) => Number(findValue(state, "Actuators", `MIXER_MOTOR_${index}_OUTPUT`, "")));
  if (order.every((output) => Number.isInteger(output) && output >= 1 && output <= 4) && new Set(order).size === 4) {
    return order as [number, number, number, number];
  }

  return fallback;
}

function NumberInput({ label, value, onChange }: { label: string; value: number; onChange: (value: number) => void }) {
  return (
    <label>
      <span>{label}</span>
      <input type="number" value={value} onChange={(event) => onChange(Number(event.currentTarget.value))} />
    </label>
  );
}
