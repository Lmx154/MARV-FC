import { useEffect, useState } from "react";

import { fetchBackendSnapshot, invokeStateCommand, listUartPorts } from "../backend/commands";
import type { AppState, BackendCommand } from "../types";

export function useBackendSnapshot() {
  const [state, setState] = useState<AppState | null>(null);
  const [backendError, setBackendError] = useState<string | null>(null);
  const [commandStatus, setCommandStatus] = useState("READY");

  async function refresh() {
    try {
      const snapshot = await fetchBackendSnapshot();
      setState(snapshot);
      setBackendError(null);
    } catch (error) {
      setBackendError(String(error));
    }
  }

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const snapshot = await fetchBackendSnapshot();
        if (mounted) {
          setState(snapshot);
          setBackendError(null);
        }
      } catch (error) {
        if (mounted) {
          setBackendError(String(error));
        }
      }
    }

    poll();
    const interval = window.setInterval(poll, 500);
    return () => {
      mounted = false;
      window.clearInterval(interval);
    };
  }, []);

  const command: BackendCommand = async (name, args) => {
    setCommandStatus(`TX ${name}`);
    try {
      const snapshot = await invokeStateCommand(name, args);
      setState(snapshot);
      setBackendError(null);
      setCommandStatus(`ACK ${name}`);
    } catch (error) {
      setCommandStatus(`ERR ${String(error)}`);
    }
  };

  async function refreshPorts() {
    setCommandStatus("TX list_uart_ports");
    try {
      await listUartPorts();
      await refresh();
      setCommandStatus("ACK list_uart_ports");
    } catch (error) {
      setCommandStatus(`ERR ${String(error)}`);
    }
  }

  return { state, backendError, commandStatus, command, refreshPorts };
}
