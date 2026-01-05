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
- Telemetry: per-sensor packets with different rates (staggered).
  - Direction: FC (rp235x) -> radio via UART -> radio link -> GS.
  - Target rates:
    - IMU: 150 Hz
    - Barometer: 100 Hz
    - Compass: 50 Hz
    - GPS: 10 Hz
    - System: 1 Hz
- Command: `CommandFrame`, reliable commands with a robust ACK system.
  - Direction: GS -> radio (ground control station commands).

## Config implications (rocket vs drone)

- The packet mix and payload sizes differ by vehicle (rocket vs drone).
- Any RF/MAC setting that is derived from those packet requirements is per-vehicle
  and should not be shared, even if the numeric values happen to match.

## Data model

### Packet families

- `TelemetryImu`
- `TelemetryBaro`
- `TelemetryMag`
- `TelemetryGps`
- `TelemetrySystem`
- `RcFrame`
- `CommandFrame`

### Telemetry packets

- `TelemetryImu` (150 Hz)
  - `accel`: `[i16; 3]`, scaled m/s^2 * 100
  - `gyro`: `[i16; 3]`, scaled deg/s * 10
- `TelemetryBaro` (100 Hz)
  - `pressure_pa`: `i32`
  - `temp_c_x10`: `i16`
- `TelemetryMag` (50 Hz)
  - `mag`: `[i16; 3]`, scaled uT * 10 (or raw if preferred)
- `TelemetryGps` (10 Hz)
  - `lat`: `i32`, deg * 1e7
  - `lon`: `i32`, deg * 1e7
  - `alt_mm`: `i32`, mm
  - `sats`: `u8`
  - `fix`: `u8`
- `TelemetrySystem` (1 Hz)
  - `vbat_mv`: `u16`
  - `temp_c`: `i8`
  - `arm_status`: `u8`
  - `rssi_uplink`: `i8`

## Serialization

- Packets are encoded as 1 byte `PacketType` + payload bytes (little-endian
  fields as defined in the packet structs).
- The packet bytes are the protocol payload for UART, LoRa, and USB-CDC.

## UART framing (wire link)

- CRC is mandatory on UART and is always part of encoding and decoding.
- Wrap: packet bytes -> CRC -> COBS encode -> UART bytes.
- Unwrap: UART bytes -> COBS decode -> CRC verify -> packet bytes.
- CRC failure: drop the frame.

## LoRa transport

- The LoRa payload carries a single protocol packet (no UART CRC or COBS).
- A radio only transmits packets that passed UART CRC validation.

## Directional flows

### Downlink (telemetry lane)

- FC -> UART framed telemetry packets -> air radio -> LoRa -> GS radio -> GS
  translator -> USB MAVLink.

### Uplink (control + command lanes)

- GS MAVLink -> `RcFrame`/`CommandFrame` -> UART framing -> air radio -> LoRa
  -> FC -> apply.
