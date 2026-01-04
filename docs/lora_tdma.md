# LoRa TDMA and Lane Behavior (Current Code, Post-Decoupling)

This document describes how the current tick-based LoRa TDMA link works after
the RF/MAC config split. It covers timing, payload sizing, and the implications
for sharing RF profiles across vehicles. A "desired behavior" section captures
the target firmware behavior (not fully implemented).

## Scope and file map

The behavior described here is implemented in these files:
- `common/src/coms/transport/lora/rf_config.rs`
- `common/src/coms/transport/lora/rf_presets.rs`
- `common/src/coms/transport/lora/mac_config.rs`
- `common/src/coms/transport/lora/link_config.rs`
- `common/src/coms/transport/lora/link_profile.rs`
- `common/src/coms/transport/lora/mac_codec.rs`
- `common/src/coms/transport/lora/mac_scheduler.rs`
- `common/src/coms/transport/lora/link_transport.rs`
- `common/src/coms/transport/lora/phy_service.rs`
- `device/gs/src/main.rs`
- `device/gs/src/mac_engine.rs`
- `device/radio/src/main.rs`
- `device/radio/src/mac_engine.rs`

Note: `device/rp235x/src/main.rs` references `common::coms::transport::lora::mac`
and a `LinkMacConfig`/`slot_mode`, but that module does not exist in this repo
snapshot. Those controls are not part of the TDMA path documented here.

## Configuration split (RF vs MAC)

The old monoconfig has been split into RF + MAC configs, with a pairing helper:

### RF config (RfConfig)

File: `common/src/coms/transport/lora/rf_config.rs`

- Alias of `LoRaConfig` (`type RfConfig = LoRaConfig`).
- Used directly by the SX1262 driver for modulation/packet params.
- Affects symbol time and time on air.
- Selected at build time via `link_config::ACTIVE` (which currently uses
  `rf_presets::ACTIVE`).

### MAC config (MacConfig)

File: `common/src/coms/transport/lora/mac_config.rs`

- Defines the TDMA schedule and payload sizing.
- Selected at build time via `link_config::ACTIVE` (which currently uses
  `mac_config::ACTIVE`).

Fields:
- `tick_hz`
- `slot_ratio_r`
- `tx_guard_us`
- `dl_tx_offset_us`
- `rx_ready_guard_us`
- `rx_timeout_symbols`
- `rx_timeout_auto`
- `uplink_payload_len`
- `downlink_payload_len`

### Link config (LinkConfig)

File: `common/src/coms/transport/lora/link_config.rs`

- Pairs `RfConfig` and `MacConfig` into a single selection point.
- Exposes `DRONE` and `ROCKET` presets for per-vehicle overrides.
- Provides helper methods:
  - `rx_timeout_symbols()` and `slot_rx_symbols()`
  - `profile()` for `LinkProfile::from_configs()`

### Derived/combined view

- `LinkProfile::from_configs(rf, mac)` computes:
  - `uplink_toa_us` and `downlink_toa_us`
  - schedule values copied from `MacConfig`
- `rx_timeout_symbols(rf, mac)` computes the default background RX timeout.
- `slot_rx_symbols(rf, mac)` computes full-slot RX timeout for GS downlink slots.

## Runtime wiring (GS vs Radio)

### GS

From `device/gs/src/main.rs`:
1) Load `link_config::ACTIVE`.
2) Initialize SX1262 with `link_cfg.rf`.
3) Derive `PhyServiceConfig` with `link_cfg.rx_timeout_symbols()`.
4) Compute `link_cfg.slot_rx_symbols()` for downlink slots.
5) Build `link_cfg.profile()`.
6) Run the MAC engine with that profile.

### Radio

From `device/radio/src/main.rs`:
1) Load `link_config::ACTIVE`.
2) Initialize SX1262 with `link_cfg.rf`.
3) Derive `PhyServiceConfig` with `link_cfg.slot_rx_symbols()` (full-slot RX).
4) Build `link_cfg.profile()`.
5) Run the MAC engine with that profile.

## TDMA schedule model

The link uses a fixed, tick-based TDMA schedule:
- A global tick counter `tick_seq` increments every tick.
- Tick period:
  ```
  tick_period_us = 1_000_000 / tick_hz
  ```
- Every `slot_ratio_r` ticks, there is a downlink slot. All other ticks are uplink.

Slot selection:
- Downlink slot: `tick_seq % slot_ratio_r == 0`
- Uplink slot: `tick_seq % slot_ratio_r != 0`
- Uplink:downlink ratio is `(slot_ratio_r - 1) : 1`

### GS role (ground station)

From `device/gs/src/mac_engine.rs`:
- On uplink ticks, GS sends `CONTROL_UP` immediately.
- On downlink ticks, GS arms RX for a full slot and waits for `CONTROL_DOWN`.

### Radio role

From `device/radio/src/mac_engine.rs`:
- The radio continuously listens for `CONTROL_UP`.
- On receive, it aligns its tick clock to:
  ```
  tick_start_us = rx_done_instant_us - uplink_toa_us
  ```
- On downlink ticks, it transmits `CONTROL_DOWN` at the configured offset.

## Frame and payload layout

Two layers are involved:

### 1) MAC frame (always on-air)

From `common/src/coms/transport/lora/mac_codec.rs`:
- Fixed header length: `HEADER_LEN = 8` bytes.
- Frame types: `CONTROL_UP` and `CONTROL_DOWN` (plus acquisition types).
- Header includes: magic, version, net_id, tick_seq, frame_type, flags.

### 2) Packet payload (inside the frame)

From `common/src/protocol/packet.rs` and `common/src/coms/transport/lora/link_transport.rs`:
- Packet type is the first byte of the payload.
- Payload is padded/truncated to a fixed size using `encode_packet_fixed()`.
- Payload length is fixed per direction (from `MacConfig`):
  - Uplink: `uplink_payload_len`
  - Downlink: `downlink_payload_len`

On-air frame length is fixed per direction:
```
total_len_bytes = HEADER_LEN + payload_len
```

As long as `payload_len` is fixed, on-air time does not change by packet type.

## Timing windows and constraints

Key timing values (from `MacConfig`):

- `tick_hz` sets `tick_period_us`.
- `tx_guard_us` reserves time at the end of each tick.
- `dl_tx_offset_us` is the planned downlink TX start.
- `rx_ready_guard_us` is how early GS tries to arm RX.

Derived windows:
```
slot_us = tick_period_us - tx_guard_us
```

Uplink fit check (GS):
- TX must fit inside the remaining slot window.
- If `uplink_toa_us` exceeds the budget, TX is refused.

Downlink fit check (Radio):
- Planned TX start:
  ```
  planned_tx_start = tick_start + dl_tx_offset_us
  ```
- Must still fit before `tick_end - tx_guard_us`.
- Actual TX start is `max(planned_tx_start, now_us)`.

RX readiness (GS):
```
rx_arm_deadline = tick_start + dl_tx_offset_us - rx_ready_guard_us
```
If RX arm happens after this deadline, a warning is logged.

## Derived RX timeouts and symbol math

Symbol time (from `RfConfig`):
```
symbol_us = (2^sf) / bw_hz
```

Auto RX timeout (`rx_timeout_symbols` in `mac_config.rs`):
```
uplink_len   = HEADER_LEN + uplink_payload_len
downlink_len = HEADER_LEN + downlink_payload_len
uplink_toa   = rf.toa_us(uplink_len)
downlink_toa = rf.toa_us(downlink_len)

max_tx_delay = tick_period_us - tx_guard_us - uplink_toa
target_rx    = dl_tx_offset_us + downlink_toa + rx_ready_guard_us
rx_us        = min(target_rx, max_tx_delay or tick_period_us)
rx_symbols   = ceil(rx_us / symbol_us) (min 4, and >= rx_timeout_symbols)
```

Full-slot RX (`slot_rx_symbols`):
```
slot_us = tick_period_us - tx_guard_us
symbols = ceil(slot_us / symbol_us)
```

## Packet priority and logical lanes

"Lanes" are logical traffic classes, not separate RF channels. The TDMA schedule
is still just uplink/downlink; packet selection is by priority.

### Uplink lanes (GS -> Radio)

From `common/src/coms/transport/lora/link_transport.rs`:
1) `Command` (reliable, with ARQ)
2) `RcData` (control)
3) `KeepAlive`

### Downlink lanes (Radio -> GS)

From `common/src/coms/transport/lora/link_transport.rs`:
1) `Ack`
2) `LinkStats`
3) `KeepAlive`

`Telemetry` exists as a packet type but is not queued or sent by `LoraTransport`
today. The intended direction for telemetry is Radio -> GS (from the FC over
UART), but that path is not implemented here.

## Vehicle differences (drone vs rocket)

Given the current code:
- Drone can use `RcData` and `Command` on uplink.
- Rocket effectively uses only `Command` on uplink.
- Telemetry is not implemented on downlink yet.

There is no lane-level TDMA; only uplink vs downlink slots.

## Desired firmware behavior (requested target)

This section describes the target behavior you want for both GS and radio
firmware. It is not implemented end-to-end in the current codebase.

### 1) Startup and SX1262 health check

- Reset SX1262, verify SPI, confirm response.
- If the check fails, remain in an error state (no RF TX) and retry or signal.

### 2) Mode selection (Rocket vs Drone)

- Rocket mode: uplink is commands (reliable), downlink is telemetry (best effort).
- Drone mode: uplink is control + commands, downlink is telemetry.

This is a logical lane decision; it does not change the physical RF config.

### 3) Operating states (Idle vs Armed)

- Idle: use a low-power, slow LoRa profile and low duty cycle.
- Armed: either fix the profile or adapt based on RSSI/SNR with hysteresis.

### 4) Rocket lane usage (armed)

- Downlink telemetry dominates airtime.
- Uplink commands use ARQ as needed.
- KeepAlive/heartbeat when idle.

### 5) Link loss and reconnect

- Enter search/reconnect on lock timeout.
- Fall back to the most robust profile while searching.
- Keep low-rate heartbeat attempts.

### 6) GS governs link setting changes

- GS is the authority for profile changes.
- GS signals the radio to switch at a known tick boundary.
- Radio applies changes only after command/ack.

### 7) Return to idle behavior

- Switch back to low-power profile and slow duty cycle.
- Keep heartbeat and link-quality updates active.

## Shared RF config vs different MAC schedules

What is now possible:
- You can share the exact same `RfConfig` between rocket and drone.
- You can select different `MacConfig` presets per vehicle at build time.

What is not yet implemented:
- Runtime negotiation or on-air switching of RF/MAC configs.
- Lane-level TDMA (control/command/telemetry ratios).
- Telemetry scheduling and payload sizing for that traffic.

## What changes when you tweak settings

Changing RF config affects:
- Symbol time and ToA
- RX timeout symbol counts
- Whether transmissions fit inside the tick budget

Changing payload lengths affects:
- On-air frame length
- ToA and RX timeout auto computations
- Whether packets fit (oversize packets are dropped)

Changing packet type does not affect:
- TDMA timing
- Frame length (still fixed per direction)
- Tick schedule

Packet type only changes the content of the fixed-size payload.

## Current coupling points

The monoconfig is gone, but there is still compile-time coupling:
- Both GS and Radio use `link_config::ACTIVE`.
- Pairing between an RF preset and a MAC preset is manual.

To get full decoupling at runtime, you still need a config negotiation path and
a paired profile selection mechanism.
