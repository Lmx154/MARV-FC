import { invoke } from "@tauri-apps/api/core";

import type { AppState } from "../types";

export async function fetchBackendSnapshot() {
  return invoke<AppState>("backend_snapshot");
}

export async function invokeStateCommand<TArgs extends Record<string, unknown> | undefined = undefined>(
  name: string,
  args?: TArgs,
) {
  return invoke<AppState>(name, args);
}

export async function listUartPorts() {
  return invoke("list_uart_ports");
}
