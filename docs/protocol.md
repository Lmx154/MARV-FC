# Protocol

This document defines the common packet model and framing rules for the FC,
radio, and GS.

## Terms

- FC: Flight Controller
- GS: Ground Station
- Vehicles: rocket, drone

## Units

- Distances use meters unless noted otherwise.

## Lanes

- Control: `RcFrame`, low-latency RC control for drones/quadcopters.
  - Direction: GS -> radio (joysticks and buttons).
- Telemetry: `TelemetryFrame`, system telemetry.
  - Direction: FC (rp235x) -> radio via UART -> radio link -> GS.
- Command: `CommandFrame`, reliable commands with a robust ACK system.
  - Direction: GS -> radio (ground control station commands).

## Config implications (rocket vs drone)

- The packet mix and payload sizes differ by vehicle (rocket vs drone).
- Any RF/MAC setting that is derived from those packet requirements is per-vehicle
  and should not be shared, even if the numeric values happen to match.

## Data model

### Packet families

- `TelemetryFrame`
- `RcFrame`
- `CommandFrame`

### TelemetryFrame fields

- Dynamics
  - `accel`: `[i16; 3]`, scaled m/s^2 * 100
  - `gyro`: `[i16; 3]`, scaled deg/s * 10
- Location
  - `lat`: `i32`, deg * 1e7
  - `lon`: `i32`, deg * 1e7
  - `alt`: `i16`, meters * 10
  - `sats`: `u8`
- Status
  - `vbat`: `u16`, mV
  - `temp`: `i8`, deg C
  - `arm_status`: `u8`
  - `rssi_uplink`: `i8`
- DebugValues
  - `val1`: `f32`
  - `val2`: `f32`

## Serialization

- Frames are serialized with `serde` + `postcard`.
- The serialized bytes are the protocol payload for both UART framing and LoRa
  transport.

## UART framing (wire link)

- CRC is mandatory on UART and is always part of encoding and decoding.
- Wrap: `Data` -> postcard serialize -> CRC -> COBS encode -> UART bytes.
- Unwrap: UART bytes -> COBS decode -> CRC verify -> postcard deserialize.
- CRC failure: drop the frame.

## LoRa transport

- The LoRa payload carries a single postcard frame (no UART CRC or COBS).
- A radio only transmits frames that passed UART CRC validation.

## Directional flows

### Downlink (telemetry lane)

- FC -> UART framed telemetry -> air radio -> LoRa -> GS radio -> GS translator
  -> USB MAVLink.

### Uplink (control + command lanes)

- GS MAVLink -> `RcFrame`/`CommandFrame` -> UART framing -> air radio -> LoRa
  -> FC -> apply.
