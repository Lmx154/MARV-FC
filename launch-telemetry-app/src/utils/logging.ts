import type { DebugLine } from "../types";

export function saveRawLog(lines: DebugLine[]) {
  const rawLines = lines.filter((line) => line.kind === "RAW").map((line) => line.text).join("\n");
  const blob = new Blob([rawLines, "\n"], { type: "text/plain" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = "launch-telemetry-preview.log";
  anchor.click();
  URL.revokeObjectURL(url);
}
