import { MetricInline } from "../components/telemetry/MetricInline";
import { SettingsPanel } from "../components/telemetry/SettingsPanel";
import type { ThemeMode } from "../types";

type SettingsViewProps = {
  theme: ThemeMode;
  onThemeChange: (theme: ThemeMode) => void;
};

export function SettingsView({ theme, onThemeChange }: SettingsViewProps) {
  return (
    <section className="settings-grid">
      <SettingsPanel title="Serial">
        <label>
          Port
          <select defaultValue="/dev/ttyUSB0">
            <option>/dev/ttyUSB0</option>
            <option>/dev/ttyACM0</option>
            <option>COM4</option>
          </select>
        </label>
        <label>
          Baud
          <select defaultValue="115200">
            <option>9600</option>
            <option>57600</option>
            <option>115200</option>
            <option>230400</option>
            <option>460800</option>
            <option>921600</option>
          </select>
        </label>
        <div className="button-row">
          <button>Connect</button>
          <button>Disconnect</button>
        </div>
      </SettingsPanel>

      <SettingsPanel title="Parser">
        <label>
          Packet Format
          <select defaultValue="CSV">
            <option>CSV</option>
            <option>JSON Lines</option>
          </select>
        </label>
        <label>
          Prefix
          <input defaultValue="TEL" />
        </label>
        <label className="switch">
          <input type="checkbox" defaultChecked />
          <span>Strict parsing</span>
        </label>
      </SettingsPanel>

      <SettingsPanel title="Display">
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
        <label>
          UI Refresh
          <select defaultValue="10 Hz">
            <option>5 Hz</option>
            <option>10 Hz</option>
            <option>20 Hz</option>
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
