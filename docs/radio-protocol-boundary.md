# Radio Protocol Boundary

Phase 0 alignment note for the radio dialect scheduler refactor.

## Boundary Rule

FC, GCS, host tools, and simulators speak normal HILink. Vehicle radio and GS
radio firmware own the compact RF dialect, scheduling, rate limiting, retry
policy, telemetry coalescing, and link budget.

```text
GCS <-> GS radio module        normal HILink
GS radio <-> vehicle radio     compact RF dialect
vehicle radio <-> FC           normal HILink
```

FC and GCS endpoints must not emit or consume compact RF dialect messages
directly. If they need link health or radio configuration feedback, a radio
module exposes that information as a normal HILink status/reporting message.

## Normal HILink Surface

Normal HILink UART frames use:

```text
COBS(Header + Payload + CRC16) + 0x00 delimiter
```

The normal 12-byte header carries protocol version, normal `MsgType`, packet
flags, sender sequence, sender-local transport timestamp, and payload length.
`Header::send_time_ms` is transport-local only; estimator/control timing uses
`SimStamp`.

Public normal HILink messages are:

| Category | Messages | Boundary notes |
| --- | --- | --- |
| Liveness and correlation | `Ping`, `Pong`, `Ack`, `Nack`, `Heartbeat` | Public on FC/GCS/radio-module normal links. `Ack`/`Nack` are command acknowledgements, not streamed HIL frame acknowledgements. |
| Deterministic HIL | `HilSensorFrame`, `HilResponseFrame`, `HilReady` | Public normal HILink, but wired/local simulation traffic. Do not blindly forward high-rate HIL frames over RF. |
| Component sensor input | `Imu`, `Mag`, `Baro`, `Gps`, `Battery`, `RadioStatus` | Normal HILink IDs. In the current protocol notes, `Battery` and `RadioStatus` are reserved/incomplete. Component sensor traffic is high-bandwidth and should be local/wired unless a radio policy explicitly summarizes it. |
| FC state and telemetry | `SystemState`, `MotorState`, `EstimatorState`, `TelemetrySnapshot` | Normal FC-to-host telemetry. Radio firmware may compress selected fields into RF snapshots. |
| Safety and mission commands | `Arm`, `Disarm`, `ControlWaypoint`, `CvWaypoint`, `TofWaypoint`, `MissionWaypoint`, `Rtl` | Normal command surface for GCS/FC. Radio firmware translates supported commands to compact RF commands. |
| Bench commands and status | `BenchEnable`, `BenchDisable`, `MotorTest`, `MotorSweep`, `MotorStop`, `DshotCommand`, `ActuatorStatusRequest`, `ActuatorStatus` | Normal HILink bench surface. `MotorStop` is RF-suitable as urgent safety traffic. Other bench commands need explicit radio safety policy before RF forwarding. |

## RF Dialect Surface

Compact RF frames are radio-internal and do not use the normal HILink header,
COBS delimiter, or normal `MsgType` namespace.

```text
rf_msg_type  1 byte
payload      payload_len bytes
crc16        2 bytes, little-endian CRC16-CCITT-FALSE over rf_msg_type + payload
```

Active V1 RF messages are:

| RF message | Payload bytes | Owner and purpose |
| --- | ---: | --- |
| `LoRaFlightSnapshot` | 22 | Radio-generated compact flight/status snapshot from normal telemetry. |
| `LoRaGpsSnapshot` | 22 | Radio-generated lower-rate GPS snapshot from normal GPS telemetry. |
| `LoRaEvent` | 15 | Radio-scheduled event notification. |
| `LoRaFaults` | 14 | Radio-scheduled fault state notification. |
| `LoRaLinkStatus` | 18 | Radio bridge link-health report. FC must not fabricate this. |
| `LoRaCommand` | 16 | Compact safety/mission/radio command over RF. |
| `LoRaCommandAck` | 12 | Compact acknowledgement for RF commands. |
| `LoRaSetProfile` | 8 | Radio-management command over RF. |
| `LoRaRequestSnapshot` | 4 | Radio-management snapshot request over RF. |

All V1 RF payloads fit in 25 bytes including `rf_msg_type` and CRC. Odd payload
sizes are intentional; field-by-field encoding is the wire contract.

## Wired Or Simulation Only

The following messages are normal HILink but should not be default RF payloads:

| Message | Reason |
| --- | --- |
| `HilSensorFrame` / `HilResponseFrame` | 115-byte and 72-byte deterministic HIL loop traffic. HIL is lockstep and high-rate; RF may reject, throttle, summarize, or selectively forward only by explicit policy. |
| `HilReady` | HIL startup signal between FC and simulator/backend. Local/wired by default. |
| `Imu`, `Mag`, `Baro`, `EstimatorState`, `MotorState` | Component/high-rate detail. Radio firmware should coalesce into `TelemetrySnapshot`-derived RF snapshots or omit unless requested debug is budgeted. |
| `BenchEnable`, `MotorTest`, `MotorSweep`, `DshotCommand` | Direct actuator validation commands. Keep local unless a later safety policy explicitly allows RF use. |

`Gps` is a component sensor message but has a defined compact RF translation
because GPS is scheduled separately and lower-rate.

## Reliability And Freshness Classes

| Class | Messages | Policy |
| --- | --- | --- |
| Reliable urgent | `Disarm`, `MotorStop`, RF `ABORT` | P0, immediate/preemptive, never stale telemetry semantics, requires command result tracking. |
| Reliable command | `Arm`, supported mission/mode commands, RF profile/rate commands, `Ping` when used as command health | P1, queued, ACK/NACK tracked, retry by active profile. |
| Reliable acknowledgement | normal `Ack`/`Nack`, `LoRaCommandAck` | P1, must not be starved by telemetry. |
| On-change or latched | faults, events, state transitions | P2, queued or latched, repeated briefly until observed or aged out. |
| Best-effort latest-only | `TelemetrySnapshot` -> `LoRaFlightSnapshot` | P3, stale once a newer snapshot exists. |
| Best-effort latest-only slow | `Gps` -> `LoRaGpsSnapshot`, `LoRaLinkStatus` | P4, stale after one active-profile period. |
| Local/wired lockstep | HIL sensor/response frames | Not normal RF scheduler traffic. Matched by `SimStamp`, not transport sequence. |

## Fields Needed For Rejection And Correlation

| Need | Field or metadata |
| --- | --- |
| Normal command correlation | normal `Header::seq` plus normal `MsgType`. |
| RF command correlation | RF `command_seq`, `command_id`, and radio-maintained mapping to normal `Header::seq`/`MsgType`. |
| Command expiry | `LoRaCommandPayload.expires_ms`, active profile ACK timeout, enqueue time. |
| Duplicate command handling | per-sender duplicate cache keyed by `command_seq` and `command_id`, retaining 16 results or 30 seconds. |
| Telemetry freshness | sender-local `time_ms`, scheduler enqueue time, and latest-only replacement by message family. |
| HIL deterministic matching | `SimStamp { sim_tick, sim_time_us }`; do not use `Header::send_time_ms`. |
| Transport diagnostics | normal `Header::send_time_ms`, RF link RSSI/SNR, packet deltas. |

## Current Repository Alignment Notes

The protocol notes define RF dialect messages under `hilink::rf` with a separate
`RfMsgType` namespace. Current repository code still contains legacy
`LoRa*` variants in `common/src/protocol/hilink/mod.rs::MsgType` and
`device/MARV-RADIO-RP2354A/src/core0.rs` classifies those variants as if they
were normal HILink messages. Treat that as a migration deviation for Phase 1:
radio firmware may keep pass-through bring-up working, but FC/GCS-facing normal
links should converge on normal HILink only.

The current radio bridge also forwards complete normal HILink frames over LoRa
payloads. Phase 1 should introduce a semantic radio dialect layer before the
LoRa frame boundary.
