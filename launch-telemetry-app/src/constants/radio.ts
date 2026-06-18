// Mirror of the firmware band plan (`marv-hilink::band_plan` + `rf::lora_profile`).
// Keep these values in sync with the Rust source of truth — the ground-station radio re-validates
// every request, so a drift here only means the UI lets through something the radio will reject.

export const SRAD_LOWER_HZ = 902_000_000;
export const SRAD_UPPER_HZ = 909_000_000;
export const MIN_TX_POWER_DBM = -9;
export const DEFAULT_MAX_TX_POWER_DBM = 17;
export const OVERRIDE_MAX_TX_POWER_DBM = 22;

/** Default boot channel (matches the firmware's unassigned Mode C bench default). */
export const DEFAULT_FREQUENCY_HZ = 902_080_000;

export type RadioPreset = {
  id: number;
  name: string;
  label: string;
  spreadingFactor: number;
  bandwidthHz: number;
  codingRateDenom: number;
};

export const RADIO_PRESETS: RadioPreset[] = [
  { id: 0, name: "FAST", label: "Fast — SF7 / 62.5 kHz (max telemetry)", spreadingFactor: 7, bandwidthHz: 62_500, codingRateDenom: 5 },
  { id: 1, name: "BALANCED", label: "Balanced — SF9 / 62.5 kHz", spreadingFactor: 9, bandwidthHz: 62_500, codingRateDenom: 5 },
  { id: 2, name: "LONG_RANGE", label: "Long range — SF10 / 62.5 kHz / CR4-8", spreadingFactor: 10, bandwidthHz: 62_500, codingRateDenom: 8 },
  { id: 3, name: "RECOVERY_BEACON", label: "Recovery beacon — SF12 / 20.8 kHz (beacon only)", spreadingFactor: 12, bandwidthHz: 20_830, codingRateDenom: 8 },
];

// Idle fallback ("return to setup frequency") — mirror of `hilink::idle_fallback`. The firmware
// re-clamps, so drift here only changes what the UI offers. Operator-facing unit is minutes.
export const IDLE_FALLBACK_MIN_MS = 60_000;
export const IDLE_FALLBACK_MAX_MS = 60 * 60_000;
export const IDLE_FALLBACK_DEFAULT_MS = 40 * 60_000;

export function clampIdleFallbackMs(ms: number): number {
  if (!Number.isFinite(ms)) return IDLE_FALLBACK_DEFAULT_MS;
  return Math.min(IDLE_FALLBACK_MAX_MS, Math.max(IDLE_FALLBACK_MIN_MS, Math.round(ms)));
}

export function presetById(id: number): RadioPreset | undefined {
  return RADIO_PRESETS.find((preset) => preset.id === id);
}

/** Short label for a preset id, e.g. for the active-profile readout. */
export function presetName(id: number): string {
  return presetById(id)?.name ?? `#${id}`;
}

/**
 * Client-side mirror of `band_plan::validate`: the whole occupied channel (center ± bandwidth/2)
 * must sit inside the SRAD band, and power must respect the policy ceiling. Returns an error
 * string, or null when the request is legal.
 */
export function validateRadioProfile(
  presetId: number,
  frequencyHz: number,
  txPowerDbm: number,
  powerOverride: boolean,
): string | null {
  const preset = presetById(presetId);
  if (!preset) return "Unknown preset";
  if (!Number.isFinite(frequencyHz)) return "Enter a frequency";

  const halfBw = Math.floor(preset.bandwidthHz / 2);
  const lower = frequencyHz - halfBw;
  const upper = frequencyHz + halfBw;
  if (lower < SRAD_LOWER_HZ || upper > SRAD_UPPER_HZ) {
    return `Channel ${(lower / 1e6).toFixed(3)}–${(upper / 1e6).toFixed(3)} MHz leaves the SRAD band (902.0–909.0 MHz)`;
  }

  const maxPower = powerOverride ? OVERRIDE_MAX_TX_POWER_DBM : DEFAULT_MAX_TX_POWER_DBM;
  if (txPowerDbm < MIN_TX_POWER_DBM) return `Power below hardware minimum (${MIN_TX_POWER_DBM} dBm)`;
  if (txPowerDbm > maxPower) {
    return powerOverride
      ? `Power above hardware maximum (${OVERRIDE_MAX_TX_POWER_DBM} dBm)`
      : `Power above the ${DEFAULT_MAX_TX_POWER_DBM} dBm default — enable override for up to ${OVERRIDE_MAX_TX_POWER_DBM} dBm`;
  }
  return null;
}
