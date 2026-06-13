import type { FlightStage } from "../types";

export function metersToMiles(meters: number) {
  return meters / 1609.344;
}

export function formatFeet(meters: number) {
  return Math.round(meters * 3.28084).toLocaleString("en-US");
}

export function formatMeters(meters: number) {
  return Math.round(meters).toLocaleString("en-US");
}

export function formatSpeed(mps: number) {
  return Math.round(mps * 3.28084).toLocaleString("en-US");
}

export function formatTime(milliseconds: number) {
  return `${(milliseconds / 1000).toFixed(1)}s`;
}

export function linkStatus(packetAgeMs: number) {
  if (packetAgeMs < 1000) return "GOOD";
  if (packetAgeMs < 3000) return "WEAK";
  if (packetAgeMs < 5000) return "STALE";
  return "LOST";
}

export function stageLabel(stage: FlightStage) {
  return stage.split("_").join(" ");
}

export function stageShortLabel(stage: FlightStage) {
  return stage.replace("_DESCENT", "").replace("_", " ");
}

export function bearingName(degrees: number) {
  const names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  return names[Math.round(degrees / 45) % names.length];
}
