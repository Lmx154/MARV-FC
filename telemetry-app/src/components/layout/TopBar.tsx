import type { AppState } from "../../types";
import { findValue } from "../../utils/telemetry";
import { Icon } from "../ui/Icon";

export function TopBar({ state, backendError }: { state: AppState | null; backendError: string | null }) {
  const uartStable = state?.uart.connected;
  const battery = findValue(state, "System", "BAT_VOLTAGE", "--");
  const linkText = backendError ? "BACKEND OFFLINE" : uartStable ? "UART: OPEN" : "UART: CLOSED";
  const isArmed = findValue(state, "System", "RESPONSE_FLAGS", "").includes("ARMED");

  return (
    <header className="topbar">
      <div className="brand-zone">
        <div className="brand-mark">
          <Icon name="rocket_launch" fill />
        </div>
        <h1>Cerberus Mission Control</h1>
        <div className="system-chip hide-md">
          <span className={`dot ${backendError ? "danger" : "ok"}`} />
          <span>SYSTEM STATE: {backendError ? "FAULT" : "NOMINAL"}</span>
        </div>
        <div className="system-chip hide-md">
          <span className={`dot ${isArmed ? "danger" : "ok"}`} />
          <span style={{ color: isArmed ? 'var(--error)' : 'inherit' }}>{isArmed ? "ARMED" : "DISARMED"}</span>
        </div>
      </div>
      <div className="topbar-right">
        <div className="signal-chip hide-sm">
          <div className="signal-bars">
            <i />
            <i />
            <i />
            <i />
            <i />
          </div>
          <span>{linkText}</span>
        </div>
        <div className="battery-chip hide-xs">
          <span>BATT</span>
          <div className="mini-meter">
            <b style={{ width: battery === "--" ? "0%" : "85%" }} />
          </div>
          <strong>
            {battery}
            {battery === "--" ? "" : "V"}
          </strong>
        </div>
        <button className="icon-button" aria-label="settings">
          <Icon name="settings" />
        </button>
      </div>
    </header>
  );
}
