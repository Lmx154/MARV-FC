// Thin wrappers around the Tauri backend (serial worker) — commands + event subscriptions.
// Centralizes the @tauri-apps/api usage and guards against running outside Tauri (plain `vite`).

import { invoke } from "@tauri-apps/api/core";
import { listen, type UnlistenFn } from "@tauri-apps/api/event";

import type { DebugLine, LinkStatus, SerialPortInfo, TelemetryPacket } from "../types";

/** True when running inside the Tauri shell (so backend IPC is available). */
export function isTauri(): boolean {
  return typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;
}

export async function listSerialPorts(): Promise<SerialPortInfo[]> {
  if (!isTauri()) return [];
  return invoke<SerialPortInfo[]>("list_serial_ports");
}

export async function connectSerial(port: string, baud: number): Promise<void> {
  await invoke("connect", { port, baud });
}

export async function disconnectSerial(): Promise<void> {
  await invoke("disconnect");
}

export type RocketCommand = "arm" | "disarm" | "ping" | "motor_stop";

export async function sendCommand(kind: RocketCommand): Promise<void> {
  await invoke("send_command", { kind });
}

export function onTelemetry(handler: (packet: TelemetryPacket) => void): Promise<UnlistenFn> {
  return listen<TelemetryPacket>("telemetry", (event) => handler(event.payload));
}

export function onDebug(handler: (line: DebugLine) => void): Promise<UnlistenFn> {
  return listen<DebugLine>("debug", (event) => handler(event.payload));
}

export function onLinkStatus(handler: (status: LinkStatus) => void): Promise<UnlistenFn> {
  return listen<LinkStatus>("link-status", (event) => handler(event.payload));
}
