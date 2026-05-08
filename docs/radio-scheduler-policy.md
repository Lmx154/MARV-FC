# Radio Scheduler Policy

Phase 0 scheduler policy for the radio dialect scheduler refactor.

## Ownership

The scheduler runs inside the vehicle radio and GS radio modules. It schedules
compact RF dialect frames, not normal HILink UART frames.

The scheduler owns:

- priority selection
- command retry and expiry windows
- RF command duplicate handling
- telemetry coalescing
- event/fault queueing
- profile-based telemetry rates
- link-health publication
- rejection or summarization of traffic that does not fit the RF budget

FC and GCS remain normal HILink endpoints and stay unaware of LoRa payload
layout, airtime budget, RF profiles, and compact field scaling.

## Priority Classes

| Priority | Traffic | Policy |
| ---: | --- | --- |
| P0 | abort, motor stop, disarm | Send before all other traffic. Reliable command path. Never drop as stale telemetry. |
| P1 | command ACK/NACK, command responses, mode/profile/rate changes, explicit snapshot requests | Reliable command/control path. Must not be starved by telemetry. |
| P2 | faults, events, state transitions, warnings | Queue or latch. Repeat briefly until observed or aged out by policy. |
| P3 | flight snapshot | Latest-only. Replace older queued flight snapshot when a newer one exists. |
| P4 | GPS snapshot, link status | Latest-only and slower-rate. Stale after one active-profile period. |
| P5 | requested debug/detail | Disabled by default. Only send inside remaining budget or explicit debug windows. |

Initial scheduler loop:

```text
1. Send pending P0.
2. Send pending P1.
3. Send pending P2 if not over event budget.
4. Send P3 if due.
5. Send P4 if due.
6. Send P5 only when explicitly enabled and budget remains.
7. Otherwise listen.
```

Commands preempt telemetry. Telemetry cannot delay a command ACK.

## Profile Rates

| Profile | Flight snapshot | GPS snapshot | Link status | ACK timeout | Max retries |
| --- | ---: | ---: | ---: | ---: | ---: |
| SF7/500 kHz | 10 Hz | 1-2 Hz | 1 Hz | 250 ms | 3 |
| SF8/500 kHz | 5-10 Hz | 1 Hz | 1 Hz | 400 ms | 3 |
| SF8/250 kHz fallback | 5 Hz | 0.5-1 Hz | 1 Hz or folded heartbeat | 750 ms | 4 |
| Recovery beacon | 1 Hz or lower, alternating flight/GPS | alternating | disabled or folded | 2000 ms | 2 |

Recovery beacon disables requested debug and should favor survivable low-rate
state over detail.

## Freshness And Queueing

| Traffic | Queue policy | Stale rule |
| --- | --- | --- |
| P0/P1 commands | bounded reliable queue | Stale only when command expiry passes before send or response. |
| P1 ACK/NACK | bounded reliable queue | Do not expire before profile ACK handling completes unless the correlation entry expires. |
| P2 event/fault | bounded queue plus optional latch by event/fault key | Age out by policy after repeated opportunities or when superseded by clear/resolution event. |
| P3 flight snapshot | single latest slot | Older snapshot is stale immediately when replaced by a newer snapshot. |
| P4 GPS snapshot | single latest slot | Stale after one active-profile GPS period or when replaced. |
| P4 link status | local generated latest slot | Stale after one active-profile link-status period or when replaced. |
| P5 debug | bounded debug queue | Drop first when budget tightens. |

High-rate normal HILink frames are not queued as opaque RF payloads in the
refactored scheduler. They are translated, summarized, rejected, or handled only
under explicit debug policy.

## Command Policy

Each GS radio sender maintains a monotonic modulo-`u16` RF `command_seq`.
Radio bridges maintain correlation between normal HILink command sequence and
RF command sequence.

Command send policy:

- P0 commands are inserted at the head of the command path.
- P1 commands are sent before P2-P5 traffic.
- `expires_ms` limits command usefulness over RF.
- Retransmissions preserve command identity and mark normal retransmission when
  the command is represented on a normal link.
- Receivers keep duplicate command results per sender for 16 command results or
  30 seconds.
- Duplicate commands return `DUPLICATE_ACCEPTED` or `DUPLICATE_REJECTED` with
  the original reason and detail.

Default RF ACK result mapping:

| RF status | Normal result |
| --- | --- |
| `ACCEPTED`, `DUPLICATE_ACCEPTED` | normal `Ack` |
| `REJECTED`, `DENIED_STATE`, `DENIED_SAFETY`, `INVALID_ARG`, `EXPIRED`, `DUPLICATE_REJECTED`, `BUSY` | normal `Nack` |

Normal ACK/NACK reconstruction must use stored correlation metadata so the GCS
or FC sees the original normal `seq` and `MsgType`.

## Telemetry Policy

Flight snapshots are the primary telemetry budget item. The vehicle radio
derives them from normal `TelemetrySnapshot` and cached state. The GS radio
expands them into normal HILink telemetry/status for GCS consumers.

GPS is separate because it is lower-rate and has distinct validity semantics.
Invalid GPS uses RF invalid sentinels and clears `GPS_VALID`.

Link status is owned by radio bridge firmware:

- vehicle radio reports RF observations from the vehicle side
- GS radio reports RF observations from the ground side
- FC does not fabricate `LoRaLinkStatus`
- GCS sees link health only after GS radio translates it to a normal HILink
  status/reporting message

## Event And Fault Policy

Events and faults are P2. They are more important than periodic telemetry but
less urgent than commands and ACKs.

Use on-change emission for:

- state transitions
- mode transitions
- arm/disarm results
- abort triggered
- GPS fix changes
- radio profile changes
- fault asserted/cleared
- pyro fired
- recovery beacon entered

Fault snapshots may be latched so a temporary outage does not hide important
state from the peer. Fault/event repetition should be bounded to protect command
latency.

## HIL And Bench Policy

The first-version HIL loop is lockstep and matched by `SimStamp`, not RF
sequence. `HilSensorFrame` and `HilResponseFrame` are normal HILink messages but
are wired/local by default and are not retransmitted or ACKed in the runtime
loop.

Bench actuator commands remain local/wired by default. `MotorStop` is the
exception because it maps cleanly to urgent safety traffic. Any RF support for
`BenchEnable`, `MotorTest`, `MotorSweep`, or `DshotCommand` requires a separate
safety policy and explicit V1 mapping.

## Implementation Migration Notes

The current radio firmware has a priority queue over opaque HILink bridge
frames. Phase 1 should move that logic behind a `radio_dialect` boundary:

- decode normal HILink from UART
- classify semantic traffic
- translate supported traffic into RF dialect frames
- cache/latest-drop telemetry before RF enqueue
- decode RF dialect from LoRa
- translate back to normal HILink for FC/GCS UART

During migration, keep Ping/Pong pass-through available for bring-up, but do not
extend transparent forwarding as the long-term scheduler behavior.
