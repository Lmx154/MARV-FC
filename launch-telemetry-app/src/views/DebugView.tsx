import type { DebugFilter, DebugKind, DebugLine } from "../types";

export function DebugView({
  autoscroll,
  debugFilter,
  lines,
  paused,
  recording,
  onClear,
  onFilterChange,
  onSave,
  onToggleAutoscroll,
  onTogglePaused,
  onToggleRecording,
}: {
  autoscroll: boolean;
  debugFilter: DebugFilter;
  lines: DebugLine[];
  paused: boolean;
  recording: boolean;
  onClear: () => void;
  onFilterChange: (filter: DebugFilter) => void;
  onSave: () => void;
  onToggleAutoscroll: () => void;
  onTogglePaused: () => void;
  onToggleRecording: () => void;
}) {
  const visibleLines = paused ? lines.slice(0, 22) : lines;

  return (
    <section className="debug-panel">
      <div className="toolbar">
        <button className={autoscroll ? "active" : ""} onClick={onToggleAutoscroll}>Autoscroll {autoscroll ? "ON" : "OFF"}</button>
        <button className={paused ? "active" : ""} onClick={onTogglePaused}>{paused ? "Resume" : "Pause"}</button>
        <button onClick={onClear}>Clear</button>
        <button className={recording ? "active" : ""} onClick={onToggleRecording}>Record {recording ? "ON" : "OFF"}</button>
        <button onClick={onSave}>Save Raw Log</button>
      </div>
      <div className="filter-row">
        {(["ALL", "RAW", "PARSED", "ERROR", "SYSTEM", "LINK"] as Array<DebugKind | "ALL">).map((filter) => (
          <button key={filter} className={debugFilter === filter ? "active" : ""} onClick={() => onFilterChange(filter)}>
            {filter}
          </button>
        ))}
      </div>
      <div className="terminal">
        {visibleLines.map((line, index) => (
          <p key={`${line.time}-${line.kind}-${index}`} className={line.kind.toLowerCase()}>
            <span>{line.time}</span>
            <b>{line.kind}</b>
            <code>{line.text}</code>
          </p>
        ))}
      </div>
    </section>
  );
}
