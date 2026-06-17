import { useEffect, useMemo, useRef, useState } from "react";
import type { UnlistenFn } from "@tauri-apps/api/event";

import { buildRecovery } from "../data/demoTelemetry";
import { isTauri, onDebug, onLinkStatus, onTelemetry } from "../lib/backend";
import type { DebugLine, GeoPoint, TelemetryPacket } from "../types";

const MAX_DEBUG_LINES = 500;

const defaultPacket: TelemetryPacket = {
  seq: 0,
  missionTimeMs: 0,
  stage: "PAD",
  altitudeM: 0,
  maxAltitudeM: 0,
  verticalSpeedMps: 0,
  accelG: 0,
  batteryV: 0,
  gpsLat: 0,
  gpsLon: 0,
  gpsAltM: 0,
  gpsValid: false,
  gpsSats: 0,
  rssiDbm: 0,
  snrDb: 0,
  packetAgeMs: 0,
  packetRateHz: 0,
  packetLossPct: 0,
};

function nowTime() {
  return new Date().toLocaleTimeString("en-US", { hour12: false });
}

/**
 * Live telemetry from the Rust serial backend. Mirrors the return shape of `useDemoTelemetry`
 * so views are interchangeable. Subscribes to the `telemetry`, `debug`, and `link-status`
 * Tauri events and tears the subscriptions down on unmount.
 */
export function useLiveTelemetry() {
  const [packet, setPacket] = useState<TelemetryPacket>(defaultPacket);
  const [debugLines, setDebugLines] = useState<DebugLine[]>([]);
  const [launchPoint, setLaunchPoint] = useState<GeoPoint | null>(null);
  const launchPointRef = useRef<GeoPoint | null>(null);

  const appendDebug = (line: DebugLine) =>
    setDebugLines((prev) => [...prev.slice(-(MAX_DEBUG_LINES - 1)), line]);

  useEffect(() => {
    if (!isTauri()) return;

    let active = true;
    let unlisteners: UnlistenFn[] = [];

    (async () => {
      const subs = await Promise.all([
        onTelemetry((next) => {
          setPacket(next);
          // Baseline the recovery launch point from the first valid GPS fix.
          if (!launchPointRef.current && next.gpsValid) {
            const point = { lat: next.gpsLat, lon: next.gpsLon };
            launchPointRef.current = point;
            setLaunchPoint(point);
          }
        }),
        onDebug(appendDebug),
        onLinkStatus((status) => {
          appendDebug({
            time: nowTime(),
            kind: "LINK",
            text: status.connected
              ? `link up: ${status.port ?? "?"} @ ${status.baud}`
              : `link down${status.lastError ? `: ${status.lastError}` : ""}`,
          });
          if (!status.connected) {
            launchPointRef.current = null;
            setLaunchPoint(null);
          }
        }),
      ]);

      if (active) {
        unlisteners = subs;
      } else {
        subs.forEach((unlisten) => unlisten());
      }
    })();

    return () => {
      active = false;
      unlisteners.forEach((unlisten) => unlisten());
    };
  }, []);

  const recovery = useMemo(() => {
    const point = launchPoint ?? { lat: packet.gpsLat, lon: packet.gpsLon };
    return buildRecovery(packet, point);
  }, [launchPoint, packet]);

  return { debugLines, packet, recovery };
}
