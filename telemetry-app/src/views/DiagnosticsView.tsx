import { ConsoleLine } from "../components/ui/ConsoleLine";
import { HeaderLine } from "../components/ui/HeaderLine";
import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import { Panel } from "../components/ui/Panel";
import { StatBox } from "../components/ui/StatBox";
import type { AppState, BackendCommand } from "../types";
import { findValue } from "../utils/telemetry";

export function DiagnosticsView({ state, command }: { state: AppState | null; command: BackendCommand }) {
  const monitorLines = state?.serial_monitor.parse_enabled ? state.serial_monitor.parsed_lines : state?.serial_monitor.lines;
  const lines = monitorLines?.slice(-80).reverse() ?? [];
  const uartReady = Boolean(state?.uart.connected);

  return (
    <main className="page diagnostics-page">
      <section className="stream-pane">
        <header className="pane-header">
          <HeaderLine icon="monitor" title="TX/RX Stream" />
          <div className="toggle-group">
            <button className={!state?.serial_monitor.parse_enabled ? "active" : ""} onClick={() => command("set_serial_monitor_parse_enabled", { enabled: false })}>
              Raw Hex
            </button>
            <button className={state?.serial_monitor.parse_enabled ? "active" : ""} onClick={() => command("set_serial_monitor_parse_enabled", { enabled: true })}>
              Parsed HiLink
            </button>
            <button title="Clear stream" onClick={() => command("clear_serial_monitor")}>
              <Icon name="delete" />
            </button>
          </div>
        </header>
        <div className="console">
          {lines.length === 0 ? (
            <div className="console-row muted">
              <span>[--:--:--]</span>
              <b>--</b>
              <code>Waiting for UART traffic...</code>
            </div>
          ) : (
            lines.map((line, index) => <ConsoleLine key={`${index}-${line}`} line={line} />)
          )}
        </div>
      </section>

      <aside className="diagnostics-rail">
        <header className="pane-header sticky">
          <HeaderLine icon="query_stats" title="Parser Diagnostics" />
        </header>
        <div className="stats-grid">
          <StatBox label="Total Frames RX" value={String(state?.hilink.rx_frames ?? 0)} accent />
          <StatBox label="Total Frames TX" value={String(state?.hilink.tx_frames ?? 0)} />
        </div>
        <Panel title="System State" icon="monitor_heart">
          <Metric label="State Code" value={findValue(state, "System", "SYS_STATE", "--")} mono />
          <Metric label="Response Flags" value={findValue(state, "System", "RESPONSE_FLAGS", "none")} mono />
          <Metric label="Battery" value={`${findValue(state, "System", "BAT_VOLTAGE", "--")} V`} mono />
        </Panel>
        <Panel title="Radio Link" icon="cell_tower">
          <Metric label="RSSI" value={`${findValue(state, "System", "RADIO_RSSI", "--")} dBm`} mono />
          <Metric label="SNR" value={`${findValue(state, "System", "RADIO_SNR", "--")} dB`} mono />
          <Metric label="Packet Loss" value={`${findValue(state, "System", "RADIO_LOSS", "--")} %`} mono />
          <div className="command-row compact">
            <button disabled={!uartReady} onClick={() => command("run_radio_link_smoke_test")}>
              <Icon name="cell_tower" />
              Radio Smoke Test
            </button>
          </div>
        </Panel>
        <Panel title="Error Counters">
          <Metric label="Parse Errors" value={String(state?.hilink.parse_errors ?? 0)} mono />
          <Metric label="Dropped Raw" value={String(state?.serial_monitor.dropped_lines ?? 0)} mono />
          <Metric label="Dropped Parsed" value={String(state?.serial_monitor.dropped_parsed_lines ?? 0)} mono />
          <Metric label="Last Error" value={state?.hilink.last_error ?? "none"} mono />
        </Panel>
        <Panel title="Protocol State">
          <Metric label="Profile" value="HiLink v1" mono />
          <Metric label="UART Direction" value="RX and TX" mono />
          <Metric label="Baud Rate" value={`${state?.uart.baud_rate ?? "--"} bps`} mono />
          <Metric label="Last Message" value={state?.hilink.last_message ?? "--"} mono />
          <Metric label="Last Command" value={state?.hilink_commands.last_event ?? "--"} mono />
        </Panel>
        <Panel title="Acknowledgements">
          <Metric label="Pong" value={String(state?.hilink_commands.pong_count ?? 0)} mono />
          <Metric label="Ack" value={String(state?.hilink_commands.ack_count ?? 0)} mono />
          <Metric label="Nack" value={String(state?.hilink_commands.nack_count ?? 0)} mono />
          <Metric label="Pending" value={String(state?.hilink_commands.pending.length ?? 0)} mono />
        </Panel>
      </aside>
    </main>
  );
}
