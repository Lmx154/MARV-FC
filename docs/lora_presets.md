# SX1262 LoRa Presets and Sizing Guide

This guide documents the telemetry-first LoRa presets added for SX1262 and the
math you need to keep them stable. The goal is to make ToA and payload limits
explicit so you can tune rates without guesswork.

## Quick sizing rules (what must fit)

- `tick_period_us = 1_000_000 / tick_hz`
- `slot_budget_us = tick_period_us - tx_guard_us`
- `on_air_len_bytes = HEADER_LEN (3) + payload_len`
- Requirement: `rf.toa_us(on_air_len_bytes) < slot_budget_us`

Notes:
- `payload_len` here is the fixed MAC payload length (`uplink_payload_len` or
  `downlink_payload_len` from `MacConfig`).
- `HEADER_LEN` is from `common/src/coms/transport/lora/mac_codec.rs`.

## Payload sizing cheat sheet

Packet payload sizes (not including the packet type byte):
- IMU: 12 bytes
- Baro: 6 bytes
- Mag: 6 bytes
- GPS: 14 bytes
- System: 5 bytes

MAC payloads include the packet type byte:
- Single IMU: `1 + 12 = 13`
- GPS: `1 + 14 = 15`

Telemetry bursts:
- Burst payload = `2 + N * sample_len`
- MAC payload for burst = `1 + 2 + N * sample_len`
- 2x IMU burst = `1 + 2 + 2*12 = 27`
- 2x Baro/Mag burst = `1 + 2 + 2*6 = 15`

## Slot ratio and effective telemetry rate

- Downlink slot: `tick_seq % slot_ratio_r == 0`
- Downlink rate = `tick_hz / slot_ratio_r`
- Bursts let you deliver multiple samples per downlink slot.
- The radio locks its tick clock from uplink, so `slot_ratio_r` must be >= 2
  to keep uplink slots available.

## ELRS notes (why 200 Hz works)

ELRS-style high rates come from:
- 500 kHz bandwidth
- low SF (5/6)
- short payloads
- tight guard times
- lower telemetry ratio

If you push payload size or SF up at 200 Hz, you will exceed the 5 ms budget.

## Telemetry-first presets (LoRa)

These presets live in:
- `common/src/coms/transport/lora/mac_presets.rs`
- `common/src/coms/transport/lora/rf_presets.rs`
- `common/src/coms/transport/lora/link_config.rs`

All ToA values below include the 3-byte MAC header (total_len = 3 + payload_len).

| Preset | RF | tick_hz / slot_ratio_r | DL payload | UL payload | DL ToA (ms) | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `LORA_TELEM_200` | SF5 / BW500 / CR4/5 | 200 / 2 | 14 | 8 | 4.112 | Downlink rate = 100 Hz; GPS does not fit. |
| `LORA_TELEM_100` | SF5 / BW500 / CR4/5 | 100 / 2 | 27 | 8 | 6.032 | 2x IMU burst per DL slot; uplink slots available. |
| `LORA_TELEM_50` | SF7 / BW500 / CR4/5 | 50 / 2 | 27 | 8 | 17.984 | 2x IMU burst per DL slot. |
| `LORA_TELEM_20` | SF8 / BW500 / CR4/5 | 20 / 2 | 27 | 8 | 30.848 | More margin for weaker links. |
| `LORA_TELEM_5` | SF9 / BW250 / CR4/6 | 5 / 2 | 27 | 8 | 127.488 | Long-range, low-rate. |
| `LORA_TELEM_1` | SF11 / BW125 / CR4/8 | 1 / 2 | 15 | 8 | 921.6 | Downlink rate = 0.5 Hz; GPS fits. |

Guard/offset values are set per preset in `common/src/coms/transport/lora/mac_presets.rs`.

Important constraints:
- With the current MAC sync (uplink-based), true downlink-only operation
  (`slot_ratio_r = 1`) will not lock. Reaching 200 Hz *telemetry* would require
  a different sync mechanism or an explicit acquisition phase.

## How to use

1) Pick a preset in `common/src/coms/transport/lora/link_config.rs` by changing:
   - `pub const ACTIVE: LinkConfig = ...`
2) Rebuild both GS and Radio firmware with the same `ACTIVE` value.

## Common tweaks

- Need RC data: set `uplink_payload_len >= 17` and re-check ToA.
- Need GPS at 200 Hz: increase `downlink_payload_len` and/or drop to 100 Hz.
- Want more uplink time: raise `slot_ratio_r` (downlink rate drops accordingly).
