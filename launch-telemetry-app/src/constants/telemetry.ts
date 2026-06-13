import type { FlightStage, TabId } from "../types";

export const tabs: Array<{ id: TabId; label: string }> = [
  { id: "dashboard", label: "Dashboard" },
  { id: "recovery", label: "Recovery" },
  { id: "debug", label: "Debug" },
  { id: "settings", label: "Settings" },
];

export const stages: FlightStage[] = [
  "PAD",
  "BOOST",
  "BURNOUT",
  "COAST",
  "APOGEE",
  "DROGUE_DESCENT",
  "MAIN_DESCENT",
  "LANDED",
];

export const launchSite = {
  lat: 32.9903,
  lon: -106.975,
};
