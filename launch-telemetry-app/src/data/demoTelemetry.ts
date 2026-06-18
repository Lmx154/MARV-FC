import { launchSite } from "../constants/telemetry";
import type { DebugLine, FlightStage, GeoPoint, RecoveryModel, TelemetryPacket } from "../types";
import { bearingDegrees, haversineMeters } from "../utils/geo";

export function buildDemoPacket(elapsedSeconds: number): TelemetryPacket {
  const t = elapsedSeconds % 128;
  const stage = stageFromTime(t);
  const altitudeM = altitudeFromTime(t);
  const maxAltitudeM = Math.max(altitudeM, altitudeFromTime(55));
  const verticalSpeedMps = verticalSpeedFromTime(t);
  const gpsLat = launchSite.lat + Math.min(0.055, t * 0.00042);
  const gpsLon = launchSite.lon + Math.min(0.04, t * 0.00031);

  return {
    seq: Math.floor(t * 18) + 1840,
    missionTimeMs: Math.floor(t * 1000),
    stage,
    altitudeM,
    maxAltitudeM,
    verticalSpeedMps,
    accelG: stage === "BOOST" ? 5.4 + Math.sin(t) * 0.4 : stage === "DROGUE_DESCENT" ? 1.8 : 1.1,
    batteryV: Math.max(7.18, 8.12 - t * 0.006),
    gpsLat,
    gpsLon,
    gpsAltM: Math.max(0, altitudeM + 1390),
    gpsValid: true,
    gpsSats: 10,
    rssiDbm: -74 - Math.min(25, t * 0.22),
    snrDb: Math.max(4.2, 11.5 - t * 0.04),
    packetAgeMs: 120 + Math.round(Math.sin(t / 3) * 30),
    packetRateHz: 8.8 + Math.sin(t / 5) * 0.4,
    packetLossPct: Math.max(0, Math.sin(t / 7) * 1.8 + 0.8),
    radioActiveKnown: true,
    radioActivePreset: 0,
    radioActiveTxPowerDbm: 17,
    radioActiveFrequencyHz: 902_080_000,
  };
}

export function buildRecovery(packet: TelemetryPacket, launchPoint: GeoPoint): RecoveryModel {
  const rocketPoint = { lat: packet.gpsLat, lon: packet.gpsLon };
  const distanceM = haversineMeters(launchPoint.lat, launchPoint.lon, rocketPoint.lat, rocketPoint.lon);
  const bearingDeg = bearingDegrees(launchPoint.lat, launchPoint.lon, rocketPoint.lat, rocketPoint.lon);

  return {
    distanceM,
    bearingDeg,
    launchPoint,
    rocketPoint,
    trail: Array.from({ length: 18 }, (_, index) => {
      const factor = index / 17;
      return {
        id: index,
        lat: launchPoint.lat + (rocketPoint.lat - launchPoint.lat) * factor,
        lon: launchPoint.lon + (rocketPoint.lon - launchPoint.lon) * factor,
      };
    }),
  };
}

export function buildLaunchPoint(): GeoPoint {
  const packet = buildDemoPacket(0);
  return { lat: packet.gpsLat, lon: packet.gpsLon };
}

export function buildDebugLines(packet: TelemetryPacket): DebugLine[] {
  const lines: DebugLine[] = [
    {
      time: lineTime(5200),
      kind: "SYSTEM",
      text: "frontend preview state initialized",
    },
  ];

  for (let offset = 8; offset >= 0; offset -= 1) {
    const seq = packet.seq - offset;
    const missionTimeMs = Math.max(0, packet.missionTimeMs - offset * 110);
    const altitudeM = Math.max(0, packet.altitudeM - offset * Math.max(4, Math.abs(packet.verticalSpeedMps) * 0.12));
    const timestampOffset = offset * 110;
    const raw = `TEL,1,${seq},${missionTimeMs},${packet.stage},${altitudeM.toFixed(1)},${altitudeM.toFixed(1)},${packet.gpsLat.toFixed(6)},${packet.gpsLon.toFixed(6)},${packet.gpsAltM.toFixed(1)},1,${packet.gpsSats},1.2,-0.4,${(packet.accelG * 9.80665).toFixed(1)},${packet.batteryV.toFixed(2)},${packet.rssiDbm.toFixed(0)},${packet.snrDb.toFixed(1)}`;

    lines.push({ time: lineTime(timestampOffset), kind: "RAW", text: raw });
    lines.push({
      time: lineTime(Math.max(0, timestampOffset - 4)),
      kind: "PARSED",
      text: `seq=${seq} stage=${packet.stage} alt=${altitudeM.toFixed(1)}m gps=valid sats=${packet.gpsSats}`,
    });

    if (offset % 3 === 0) {
      lines.push({
        time: lineTime(Math.max(0, timestampOffset - 7)),
        kind: "LINK",
        text: `packet_age=${packet.packetAgeMs}ms rssi=${packet.rssiDbm.toFixed(0)} snr=${packet.snrDb.toFixed(1)} loss=${packet.packetLossPct.toFixed(1)}%`,
      });
    }
  }

  lines.splice(8, 0, {
    time: lineTime(780),
    kind: "ERROR",
    text: "invalid packet: expected TEL prefix",
  });

  return lines;
}

function lineTime(offsetMs: number) {
  return new Date(Date.now() - offsetMs).toLocaleTimeString("en-US", { hour12: false });
}

function stageFromTime(t: number): FlightStage {
  if (t < 5) return "PAD";
  if (t < 18) return "BOOST";
  if (t < 22) return "BURNOUT";
  if (t < 52) return "COAST";
  if (t < 58) return "APOGEE";
  if (t < 96) return "DROGUE_DESCENT";
  if (t < 122) return "MAIN_DESCENT";
  return "LANDED";
}

function altitudeFromTime(t: number) {
  if (t < 5) return 0;
  if (t < 18) return Math.pow(t - 5, 2) * 32;
  if (t < 52) return 5408 + (t - 18) * 145 - Math.pow(t - 18, 2) * 2.4;
  if (t < 58) return 7580 - Math.pow(t - 55, 2) * 8;
  if (t < 96) return Math.max(2800, 7530 - (t - 58) * 120);
  if (t < 122) return Math.max(55, 2970 - (t - 96) * 112);
  return 0;
}

function verticalSpeedFromTime(t: number) {
  const dt = 0.5;
  return (altitudeFromTime(Math.min(127, t + dt)) - altitudeFromTime(Math.max(0, t - dt))) / (dt * 2);
}
