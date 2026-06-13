export type TabId = "dashboard" | "recovery" | "debug" | "settings";
export type ThemeMode = "dark" | "light";

export type FlightStage =
  | "PAD"
  | "BOOST"
  | "BURNOUT"
  | "COAST"
  | "APOGEE"
  | "DROGUE_DESCENT"
  | "MAIN_DESCENT"
  | "LANDED";

export type DebugKind = "RAW" | "PARSED" | "ERROR" | "SYSTEM" | "LINK";
export type DebugFilter = DebugKind | "ALL";

export type TelemetryPacket = {
  seq: number;
  missionTimeMs: number;
  stage: FlightStage;
  altitudeM: number;
  maxAltitudeM: number;
  verticalSpeedMps: number;
  accelG: number;
  batteryV: number;
  gpsLat: number;
  gpsLon: number;
  gpsAltM: number;
  gpsValid: boolean;
  gpsSats: number;
  rssiDbm: number;
  snrDb: number;
  packetAgeMs: number;
  packetRateHz: number;
  packetLossPct: number;
};

export type DebugLine = {
  time: string;
  kind: DebugKind;
  text: string;
};

export type TrailPoint = {
  id: number;
  lat: number;
  lon: number;
};

export type GeoPoint = {
  lat: number;
  lon: number;
};

export type RecoveryModel = {
  distanceM: number;
  bearingDeg: number;
  launchPoint: GeoPoint;
  rocketPoint: GeoPoint;
  trail: TrailPoint[];
};
