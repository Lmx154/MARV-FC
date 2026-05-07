import type { AppState } from "../../types";
import { formatCommandStatus } from "../../utils/format";
import { Icon } from "./Icon";

export function CommandHistory({ state }: { state: AppState | null }) {
  const pending = state?.hilink_commands.pending.slice(-4).reverse() ?? [];
  return (
    <div className="rail-section">
      <div className="rail-header">
        <span>TX/RX Log</span>
        <Icon name="history" />
      </div>
      <div className="history-list">
        {state?.hilink_commands.last_event && <LogItem kind="ack" title="LATEST_EVENT" body={state.hilink_commands.last_event} />}
        {pending.map((command) => (
          <LogItem
            key={`${command.seq}-${command.label}`}
            kind="tx"
            title={`${command.label}_${command.seq}`}
            body={`msg=${command.msg_type} ${formatCommandStatus(command.status)}`}
          />
        ))}
        {pending.length === 0 && <LogItem kind="info" title="COMMAND_IDLE" body="No pending HiLink commands." />}
      </div>
    </div>
  );
}

function LogItem({ kind, title, body }: { kind: "ack" | "tx" | "info"; title: string; body: string }) {
  const icon = kind === "ack" ? "check_circle" : kind === "tx" ? "arrow_upward" : "info";
  return (
    <div className={`log-item ${kind}`}>
      <Icon name={icon} />
      <div>
        <strong>{title}</strong>
        <span>{body}</span>
      </div>
    </div>
  );
}
