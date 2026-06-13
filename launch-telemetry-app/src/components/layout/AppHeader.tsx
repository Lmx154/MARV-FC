import type { TelemetryPacket, ThemeMode } from "../../types";
import { formatTime, linkStatus, stageLabel } from "../../utils/format";
import { StatusChip } from "./StatusChip";

type AppHeaderProps = {
  packet: TelemetryPacket;
  theme: ThemeMode;
  onToggleTheme: () => void;
};

export function AppHeader({ packet, theme, onToggleTheme }: AppHeaderProps) {
  const themeLabel = theme === "dark" ? "Light HC" : "Dark HC";

  return (
    <header className="topbar">
      <div className="brand">
        <span className="brand-mark">LV</span>
        <div>
          <h1>MARV Launch Telemetry</h1>
          <p>Read-only event console</p>
        </div>
      </div>
      <div className="top-status">
        <StatusChip label="CP2102" value="Preview" tone="neutral" />
        <StatusChip label="Link" value={linkStatus(packet.packetAgeMs)} tone="good" />
        <StatusChip label="Stage" value={stageLabel(packet.stage)} tone="accent" />
        <StatusChip label="T+" value={formatTime(packet.missionTimeMs)} tone="neutral" />
        <button className="theme-toggle" type="button" onClick={onToggleTheme} aria-label={`Switch to ${themeLabel}`}>
          {themeLabel}
        </button>
      </div>
    </header>
  );
}
