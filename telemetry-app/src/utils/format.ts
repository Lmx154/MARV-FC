export function formatSeconds(seconds: number | null | undefined) {
  if (seconds == null) return "--";
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const secs = seconds % 60;
  return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${secs
    .toString()
    .padStart(2, "0")}`;
}

export function fmtNum(value: number | undefined, digits = 3) {
  return value == null ? "--" : value.toFixed(digits);
}

export function formatCommandStatus(status: Record<string, unknown> | string) {
  if (typeof status === "string") return status;
  const [key, value] = Object.entries(status)[0] ?? ["unknown", ""];
  if (value && typeof value === "object") {
    return `${key} ${Object.entries(value)
      .map(([itemKey, itemValue]) => `${itemKey}=${String(itemValue)}`)
      .join(" ")}`;
  }
  return key;
}
