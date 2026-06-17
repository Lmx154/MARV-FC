import { useCallback, useEffect, useState } from "react";

import { MetricInline } from "../components/telemetry/MetricInline";
import { SettingsPanel } from "../components/telemetry/SettingsPanel";
import {
  connectSerial,
  disconnectSerial,
  isTauri,
  listSerialPorts,
  onLinkStatus,
  sendCommand,
  type RocketCommand,
} from "../lib/backend";
import type { DataSource, LinkStatus, SerialPortInfo, ThemeMode } from "../types";

const BAUD_RATES = [9600, 57600, 115200, 230400, 460800, 921600];

type SettingsViewProps = {
  theme: ThemeMode;
  onThemeChange: (theme: ThemeMode) => void;
  source: DataSource;
  onSourceChange: (source: DataSource) => void;
};

export function SettingsView({ theme, onThemeChange, source, onSourceChange }: SettingsViewProps) {
  const tauri = isTauri();
  const [ports, setPorts] = useState<SerialPortInfo[]>([]);
  const [selectedPort, setSelectedPort] = useState<string>("");
  // Default matches the MARV-RADIO ground-station host UART (HOST_UART_BAUD = 460_800).
  const [baud, setBaud] = useState<number>(460800);
  const [link, setLink] = useState<LinkStatus>({ connected: false, port: null, baud: 460800, lastError: null });

  const refreshPorts = useCallback(async () => {
    const found = await listSerialPorts();
    setPorts(found);
    setSelectedPort((current) => current || found[0]?.portName || "");
  }, []);

  useEffect(() => {
    if (!tauri) return;
    void refreshPorts();
    let unlisten: (() => void) | undefined;
    let active = true;
    onLinkStatus((status) => setLink(status)).then((fn) => {
      if (active) unlisten = fn;
      else fn();
    });
    return () => {
      active = false;
      unlisten?.();
    };
  }, [tauri, refreshPorts]);

  const connected = link.connected;

  return (
    <section className="settings-grid">
      <SettingsPanel title="Serial">
        <label>
          Port
          <select value={selectedPort} onChange={(event) => setSelectedPort(event.currentTarget.value)} disabled={!tauri || connected}>
            {ports.length === 0 && <option value="">No ports found</option>}
            {ports.map((port) => (
              <option key={port.portName} value={port.portName}>
                {port.displayName}
              </option>
            ))}
          </select>
        </label>
        <label>
          Baud
          <select value={baud} onChange={(event) => setBaud(Number(event.currentTarget.value))} disabled={!tauri || connected}>
            {BAUD_RATES.map((rate) => (
              <option key={rate} value={rate}>
                {rate}
              </option>
            ))}
          </select>
        </label>
        <div className="button-row">
          <button onClick={() => void connectSerial(selectedPort, baud)} disabled={!tauri || connected || !selectedPort}>
            Connect
          </button>
          <button onClick={() => void disconnectSerial()} disabled={!tauri || !connected}>
            Disconnect
          </button>
          <button onClick={() => void refreshPorts()} disabled={!tauri || connected}>
            Refresh
          </button>
        </div>
        <MetricInline label="Status" value={connected ? `Connected ${link.port ?? ""}` : "Disconnected"} />
        {link.lastError && <MetricInline label="Last error" value={link.lastError} />}
        {!tauri && <MetricInline label="Note" value="Run the desktop app to use serial" />}
      </SettingsPanel>

      <SettingsPanel title="Commands">
        <div className="button-row">
          <button onClick={() => void sendCommand("arm")} disabled={!tauri || !connected}>
            Arm
          </button>
          <button onClick={() => void sendCommand("disarm")} disabled={!tauri || !connected}>
            Disarm
          </button>
        </div>
        <div className="button-row">
          {(["ping", "motor_stop"] as RocketCommand[]).map((cmd) => (
            <button key={cmd} onClick={() => void sendCommand(cmd)} disabled={!tauri || !connected}>
              {cmd === "motor_stop" ? "Motor Stop" : "Ping"}
            </button>
          ))}
        </div>
        <MetricInline label="Link" value={connected ? "Commands enabled" : "Connect to send"} />
      </SettingsPanel>

      <SettingsPanel title="Display">
        <label>
          Data Source
          <select value={source} onChange={(event) => onSourceChange(event.currentTarget.value as DataSource)}>
            <option value="live">Live (serial)</option>
            <option value="demo">Demo (simulated)</option>
          </select>
        </label>
        <label>
          Theme
          <select value={theme} onChange={(event) => onThemeChange(event.currentTarget.value as ThemeMode)}>
            <option value="dark">High contrast dark</option>
            <option value="light">High contrast light</option>
          </select>
        </label>
        <label>
          Units
          <select defaultValue="Imperial">
            <option>Imperial</option>
            <option>Metric</option>
          </select>
        </label>
      </SettingsPanel>

      <SettingsPanel title="RF Profile">
        <MetricInline label="Frequency" value="915 MHz" />
        <MetricInline label="Bandwidth" value="125 kHz" />
        <MetricInline label="Spreading" value="SF7" />
        <MetricInline label="Coding rate" value="4/5" />
        <MetricInline label="Profile" value="Display only" />
      </SettingsPanel>
    </section>
  );
}
