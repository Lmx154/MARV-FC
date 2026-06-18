import { useCallback, useEffect, useState } from "react";

import { MetricInline } from "../components/telemetry/MetricInline";
import { SettingsPanel } from "../components/telemetry/SettingsPanel";
import {
  connectSerial,
  disconnectSerial,
  isTauri,
  listSerialPorts,
  onLinkStatus,
  onTelemetry,
  sendCommand,
  sendIdleFallback,
  sendRadioProfile,
  type RocketCommand,
} from "../lib/backend";
import {
  DEFAULT_FREQUENCY_HZ,
  IDLE_FALLBACK_DEFAULT_MS,
  IDLE_FALLBACK_MAX_MS,
  IDLE_FALLBACK_MIN_MS,
  RADIO_PRESETS,
  clampIdleFallbackMs,
  presetById,
  presetName,
  validateRadioProfile,
} from "../constants/radio";
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
  // RF profile control state.
  const [preset, setPreset] = useState<number>(RADIO_PRESETS[0].id);
  const [freqMhz, setFreqMhz] = useState<number>(DEFAULT_FREQUENCY_HZ / 1e6);
  const [txPower, setTxPower] = useState<number>(17);
  const [powerOverride, setPowerOverride] = useState<boolean>(false);
  const [rfStatus, setRfStatus] = useState<string | null>(null);
  // Radio's live active profile (from RadioStatus telemetry) for switch verification.
  const [radioActive, setRadioActive] = useState<{ preset: number; freqHz: number; power: number } | null>(null);
  // Idle-fallback window (operator-facing unit is minutes; sent in ms).
  const [idleFallbackMin, setIdleFallbackMin] = useState<number>(IDLE_FALLBACK_DEFAULT_MS / 60_000);
  const [idleStatus, setIdleStatus] = useState<string | null>(null);

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

  // Track the radio's reported active RF profile; only update state when it changes so telemetry-
  // rate packets don't re-render the settings page.
  useEffect(() => {
    if (!tauri) return;
    let unlisten: (() => void) | undefined;
    let active = true;
    onTelemetry((packet) => {
      if (!packet.radioActiveKnown) return;
      setRadioActive((current) =>
        current &&
        current.preset === packet.radioActivePreset &&
        current.freqHz === packet.radioActiveFrequencyHz &&
        current.power === packet.radioActiveTxPowerDbm
          ? current
          : {
              preset: packet.radioActivePreset,
              freqHz: packet.radioActiveFrequencyHz,
              power: packet.radioActiveTxPowerDbm,
            },
      );
    }).then((fn) => {
      if (active) unlisten = fn;
      else fn();
    });
    return () => {
      active = false;
      unlisten?.();
    };
  }, [tauri]);

  const connected = link.connected;

  const frequencyHz = Math.round(freqMhz * 1e6);
  const rfError = validateRadioProfile(preset, frequencyHz, txPower, powerOverride);

  const applyRadioProfile = () => {
    if (rfError) {
      setRfStatus(`Rejected locally: ${rfError}`);
      return;
    }
    void sendRadioProfile(preset, frequencyHz, txPower, powerOverride);
    setRfStatus(
      `Sent ${presetById(preset)?.name ?? preset} @ ${freqMhz.toFixed(3)} MHz, ${txPower} dBm — switching link-wide…`,
    );
  };

  const applyIdleFallback = () => {
    const ms = clampIdleFallbackMs(idleFallbackMin * 60_000);
    void sendIdleFallback(ms);
    setIdleStatus(`Sent ${(ms / 60_000).toFixed(0)} min — both ends re-home to setup freq when idle this long.`);
  };

  const radioMatchesSelection =
    radioActive !== null &&
    radioActive.preset === preset &&
    radioActive.freqHz === frequencyHz &&
    radioActive.power === txPower;

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

      <SettingsPanel title="RF Profile (link-wide)">
        <label>
          Preset
          <select
            value={preset}
            onChange={(event) => setPreset(Number(event.currentTarget.value))}
            disabled={!tauri || !connected}
          >
            {RADIO_PRESETS.map((option) => (
              <option key={option.id} value={option.id}>
                {option.label}
              </option>
            ))}
          </select>
        </label>
        <label>
          Frequency (MHz)
          <input
            type="number"
            step={0.025}
            min={902}
            max={909}
            value={freqMhz}
            onChange={(event) => setFreqMhz(Number(event.currentTarget.value))}
            disabled={!tauri || !connected}
          />
        </label>
        <label>
          TX power (dBm)
          <input
            type="number"
            step={1}
            min={-9}
            max={22}
            value={txPower}
            onChange={(event) => setTxPower(Number(event.currentTarget.value))}
            disabled={!tauri || !connected}
          />
        </label>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={powerOverride}
            onChange={(event) => setPowerOverride(event.currentTarget.checked)}
            disabled={!tauri || !connected}
          />
          Allow power above 17 dBm (up to 22)
        </label>
        <div className="button-row">
          <button onClick={applyRadioProfile} disabled={!tauri || !connected || rfError !== null}>
            Apply profile
          </button>
        </div>
        {rfError && <MetricInline label="Invalid" value={rfError} />}
        {rfStatus && <MetricInline label="Last action" value={rfStatus} />}
        <MetricInline
          label="Radio active"
          value={
            radioActive
              ? `${presetName(radioActive.preset)} @ ${(radioActive.freqHz / 1e6).toFixed(3)} MHz, ${radioActive.power} dBm`
              : connected
                ? "Awaiting radio status…"
                : "Connect to read"
          }
        />
        {radioActive && (
          <MetricInline
            label="Verification"
            value={radioMatchesSelection ? "Link is on the selected profile ✓" : "Differs from selection above"}
          />
        )}
        <MetricInline label="Safety" value="Both radios switch together; auto-reverts if unverified." />
        <label>
          Idle fallback to setup freq (min)
          <input
            type="number"
            step={1}
            min={IDLE_FALLBACK_MIN_MS / 60_000}
            max={IDLE_FALLBACK_MAX_MS / 60_000}
            value={idleFallbackMin}
            onChange={(event) => setIdleFallbackMin(Number(event.currentTarget.value))}
            disabled={!tauri || !connected}
          />
        </label>
        <div className="button-row">
          <button onClick={applyIdleFallback} disabled={!tauri || !connected}>
            Apply idle fallback
          </button>
        </div>
        {idleStatus && <MetricInline label="Idle fallback" value={idleStatus} />}
      </SettingsPanel>
    </section>
  );
}
