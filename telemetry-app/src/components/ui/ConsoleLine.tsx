export function ConsoleLine({ line }: { line: string }) {
  const direction = line.startsWith("TX") || line.includes(" TX ") ? "TX" : line.startsWith("RX") || line.includes(" RX ") ? "RX" : "--";
  const error = /error|fail|crc/i.test(line);
  return (
    <div className={`console-row ${error ? "error" : ""}`}>
      <span>[{new Date().toLocaleTimeString()}]</span>
      <b>{direction}</b>
      <code>{line}</code>
    </div>
  );
}
