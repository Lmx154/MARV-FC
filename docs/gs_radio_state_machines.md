# GS and Radio State Machines

This document defines the formal state machines for the GS and the radio, then
summarizes how they interact with the ground control station software (GCS) and
the FC (rp235x). It is protocol-focused and does not use MAVLink in the data
path.

## Terms

- FC: Flight Controller (rp235x)
- GS: Ground Station device (radio + USB)
- GCS: Ground Control Station software on the PC

## Links and protocols

- GCS <-> GS: USB CDC carrying the custom protocol frames (no MAVLink layer).
- FC <-> Radio: UART using COBS framing with mandatory CRC on every frame.
- GS <-> Radio: custom LoRa TDMA link (see `docs/lora_tdma.md` for timing).

## Lanes and directions

- Control lane: `RcFrame`, low-latency RC control for drones/quadcopters.
  - Direction: GCS -> GS -> radio -> FC.
- Command lane: `CommandFrame`, reliable commands with robust ACK.
  - Direction: GCS -> GS -> radio -> FC.
- Telemetry lane: `TelemetryFrame`, system telemetry.
  - Direction: FC -> radio -> GS -> GCS.

## Units

- Distances are in meters unless noted otherwise.

## Vehicle modes

- Drone: uses control + command on uplink, telemetry on downlink.
- Rocket: uses command on uplink, telemetry on downlink.

## Config shareability rule

- Share only RF/PHY settings that are intrinsic and not derived from vehicle
  packetization or lane requirements.
- Treat all MAC schedule/payload values as per-vehicle; if derived from
  rocket/drone-specific parameters, they are not shareable even if equal.

## GS state machine (tick master)

### States

- BOOT: power-on entry point.
- RF_INIT: load configs, init SX1262, verify radio health.
- IDLE: link not active, low-duty behavior.
- SYNC: start TDMA tick schedule as master.
- RUN: normal TDMA operation (uplink TX on uplink ticks, downlink RX on downlink
  ticks).
- RECONNECT: link recovery mode after timeout or loss.
- ERROR: unrecoverable failure (no RF TX).

### Transitions

- [BOOT] -> [RF_INIT] on power_on
- [RF_INIT] -> [IDLE] on sx1262_ok
- [RF_INIT] -> [ERROR] on sx1262_fail
- [IDLE] -> [SYNC] on gcs_active or arm_request
- [SYNC] -> [RUN] on tick_started
- [RUN] -> [RECONNECT] on link_timeout
- [RECONNECT] -> [RUN] on sync_recovered
- [RUN] -> [IDLE] on disarm or gcs_idle
- [RECONNECT] -> [ERROR] on fatal_hw

### RUN behavior (per tick)

- Uplink tick: transmit control or command (priority driven).
- Downlink tick: arm RX, wait for telemetry or ACKs.
- USB CDC: receive control/command from GCS, deliver telemetry to GCS.

## Radio state machine (tick follower)

### States

- BOOT: power-on entry point.
- RF_INIT: load configs, init SX1262, verify radio health.
- LISTEN: idle RX, waiting for uplink frames.
- SYNC: align tick clock to received uplink frame.
- RUN: normal TDMA operation (uplink RX on uplink ticks, downlink TX on downlink
  ticks).
- RECONNECT: loss recovery, return to LISTEN with robust settings.
- ERROR: unrecoverable failure (no RF TX).

### Transitions

- [BOOT] -> [RF_INIT] on power_on
- [RF_INIT] -> [LISTEN] on sx1262_ok
- [RF_INIT] -> [ERROR] on sx1262_fail
- [LISTEN] -> [SYNC] on valid_uplink_rx
- [SYNC] -> [RUN] on tick_aligned
- [RUN] -> [RECONNECT] on link_timeout
- [RECONNECT] -> [LISTEN] on retry_window_expired
- [RUN] -> [LISTEN] on explicit_reset
- [RECONNECT] -> [ERROR] on fatal_hw

### RUN behavior (per tick)

- Uplink tick: receive control/command frames from GS.
- Downlink tick: transmit telemetry or ACK frames to GS.
- UART: decode/verify incoming telemetry from FC; encode/forward commands and
  control to FC.

## High-level interaction summary

### Control and command path (uplink)

GCS -> USB CDC -> GS -> LoRa TDMA -> radio -> UART -> FC

### Telemetry path (downlink)

FC -> UART -> radio -> LoRa TDMA -> GS -> USB CDC -> GCS

### Protocol rule

The custom protocol is end-to-end across USB CDC, UART, and LoRa, with CRC
required on all UART frames.
