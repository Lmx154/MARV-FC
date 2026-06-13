# MARV Radio Link Firmware Assessment

This assessment compares the current `MARV-RADIO-RP2354A` firmware against the intended LoRa radio-link plan. The original vision was written around a MAVLink bridge, but this repository is currently using HILink as the host protocol and a compact HILink-specific RF dialect over LoRa. That is a good architectural fit for MARV, but it means several MAVLink-specific tasks should be treated as replaced rather than merely incomplete.

## Current Architecture

The radio firmware already has the major vertical slice in place:

```text
Host UART
  -> HILink frame collection and priority classification
  -> priority host-to-radio channels
  -> compact HILink LoRa RF dialect
  -> single LoRa bridge task owning the SX1262
  -> lora-rs PHY APIs
  -> SX1262
```

Key implementation points:

- `device/MARV-RADIO-RP2354A/src/core0.rs` owns the host UART RX/TX tasks and queues complete HILink frames for LoRa.
- `device/MARV-RADIO-RP2354A/src/lora_bridge.rs` owns the radio object, TX/RX loop, keepalives, link-state transitions, and recovery.
- `common/src/drivers/sx1262.rs` wraps `lora-phy`/`lora-rs` for the Waveshare SX1262 board integration.
- `common/src/comms/links/lora/*` defines link profiles, LoRa frame wrapping, timing, stats, and health state.
- `common/src/protocol/hilink/rf.rs` defines the compact RF payload dialect.
- `device/MARV-RADIO-RP2354A/src/radio_dialect/*` translates between normal HILink frames and compact RF frames.

## Assessment Against Focus Areas

| Focus area from original plan | Current status | Assessment |
| --- | --- | --- |
| MAVLink UART bridge | Replaced by HILink UART bridge | The firmware reads delimiter-framed HILink packets from host UART, classifies them, queues them by priority, and writes received HILink packets back to UART. MAVLink parsing is not implemented because the design has moved to HILink. |
| MAVLink semantic compression | Replaced by HILink RF dialect | Compact LoRa payloads exist for flight snapshots, GPS, events, faults, link status, commands, command ACKs, profile changes, and snapshot requests. This covers the same architectural role as semantic MAVLink compression, but using MARV-native HILink semantics. |
| Custom RF frame format | Partially implemented | There is an outer LoRa frame with magic/version/source/kind/length plus an inner RF packet with message type and CRC. It is simple and useful, but it does not yet include sequence, ACK, ACK mask, or fragmentation fields. |
| Seq/ACK/ACK-mask link layer | Mostly missing | Command correlation and duplicate suppression exist for compact commands, but there is no generic RF sequence number, ACK bitmap, sliding receive window, packet-level dedup, or selective ACK mechanism. |
| Retry queues | Mostly missing | Priority queues exist between host and radio. Pending command ACK/event queues exist in `RadioStateCache`. There is not yet a durable reliable-send queue with retry timers, expiration handling, retransmit counters, or backoff. |
| Latest-value telemetry cache | Implemented for core snapshots | Flight snapshot, GPS snapshot, faults, link status, and event state are cached and scheduled. This is already aligned with the latest-value-wins idea for high-rate telemetry. Coverage is currently limited to selected HILink messages. |
| Airtime-aware scheduler | Partially implemented | The code uses `lora_modulation` airtime math through `LoraProfile::time_on_air_us()` and derives receive window timing in `LoraLinkTiming`. Packet selection is priority/rate based, but there is not yet per-slot airtime admission, fragmentation/defer decisions, duty-cycle accounting, or queue budget enforcement. |
| TDD timing | Basic alternating behavior, not full TDD MAC | Both roles have phased keepalives and the bridge alternates queued TX/keepalive with single RX windows. This gives a cooperative half-duplex rhythm, but it is not a formal slot table with guard times, role-owned slots, drift handling, or asymmetric slot scheduling. |
| Link-quality estimator | Basic link health implemented | The firmware tracks TX/RX counts, errors, malformed/unexpected frames, missed peer packets, last RSSI, and last SNR. It has `Acquiring`, `Linked`, `Degraded`, `Lost`, and `RadioFault` states. It does not yet compute rolling LQ/PER, retry rate, latency, queue depth, airtime utilization, or min/average RSSI/SNR. |
| Adaptive mode/power policy | Profile definitions exist, dynamic policy missing | `FAST_915`, `ROBUST_915`, `FALLBACK_915`, and `LONG_RANGE_915` profiles exist, and the SX1262 wrapper can apply profiles when the sync word is unchanged. No closed-loop mode manager currently switches profiles based on link quality or queue pressure. |
| Raw MAVLink tunnel fallback | Missing and probably should become raw HILink fallback | Oversized HILink frames are dropped, and unsupported host frames are dropped after classification/translation. There is no raw HILink byte/packet tunnel, no fragmentation, and no reassembly. If fallback is still desired, it should be specified as raw HILink fallback, not MAVLink fallback. |

## How We Match The Plan

The biggest architectural match is that the firmware correctly keeps the radio behind a single owner task. Random firmware modules do not call `lora.tx()` or `lora.rx()` directly; `lora_bridge_task` owns the SX1262 wrapper and serializes TX, RX, and recovery. This aligns with the original safety guidance around `lora-rs` IRQ processing.

The project also follows the intended split between PHY and link layer. `lora-rs` is used for SX1262 operation, LoRa parameters, TX/RX, and packet status, while MARV-specific framing, health, scheduling, and HILink translation live in repository code.

Where the implementation intentionally differs:

- The host protocol is HILink, not MAVLink.
- Semantic compression is implemented as HILink-to-LoRa RF translation, not MAVLink message compression.
- Command ACKs are mapped through HILink command correlation instead of a generic RF ACK bitmap.
- The first milestone is not raw MAVLink passthrough; it is compact HILink command/telemetry transport.

Where the implementation is still thinner than the plan:

- Packet-level reliability is not generalized.
- Scheduler decisions are priority/rate driven, but not truly airtime/slot-budget driven.
- There is no formal TDD frame schedule.
- Link metrics are useful for state indication, but not yet enough for adaptive rate/power control.
- Unsupported or oversized host traffic has no raw tunnel fallback.

## Notable Technical Gaps

The current `LoraProfile` carries a `sync_word`, but `common/src/drivers/sx1262.rs` uses it only to choose the public/private network setting passed into `LoRa::new()`. If the goal is a custom private P2P sync word, the driver should use the appropriate `lora-rs` custom sync-word constructor/API rather than treating `0x12` and `0x34` as a boolean public/private choice.

The outer LoRa frame is intentionally minimal:

```text
magic[2], version, source, kind, payload_len, payload
```

That is enough for basic validation and routing, but not enough for reliable packet transport. The inner HILink RF packet adds a CRC and message type, but still does not provide transport sequence numbers, ACK masks, retry identity, or fragmentation metadata.

The current scheduler has useful priorities:

```text
P0 critical
P1 command
P2 event
P3 snapshot
P4 background
```

It does not yet enforce an explicit slot budget or compare candidate airtime against a current TDD slot. This is the main place where the existing `LoraProfile::time_on_air_us()` and `LoraLinkTiming` foundation should be extended.

## Recommended Next Milestones

1. Add a real RF transport header revision with sequence number, ACK, ACK mask, flags, and payload length while keeping the current magic/version guard.
2. Implement a small reliable command queue with expiration, retry counters, duplicate suppression, and command ACK mapping.
3. Convert the bridge loop into an explicit two-role TDD schedule with guard time and bounded TX/RX windows.
4. Make scheduler admission airtime-aware: select, defer, or fragment based on candidate packet length and slot budget.
5. Add raw HILink fallback framing for unsupported or oversized packets, including fragmentation and reassembly.
6. Expand link metrics into rolling LQ/PER, retry rate, queue depth, RSSI/SNR averages, and airtime utilization.
7. Add a mode manager that can choose between the existing LoRa profiles and TX power settings using link metrics.
8. Audit the SX1262 wrapper's sync-word setup and switch to explicit custom P2P sync-word configuration if supported by the selected `lora-rs` API.

## Bottom Line

`MARV-RADIO-RP2354A` is no longer just a PHY demo. It already has a HILink-aware LoRa bridge with compact semantic payloads, priority queues, scheduled telemetry snapshots, link status, and health/recovery behavior.

The remaining work is concentrated in the true link-layer pieces: generalized reliability, airtime-aware scheduling, formal TDD timing, adaptive mode control, and raw HILink fallback. The original MAVLink-specific work should not be carried forward literally unless MARV decides to support MAVLink at the host boundary again.
