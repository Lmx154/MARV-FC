# Radio Dialect Scheduler Refactor Plan

This plan defines the boundary we want before the radio link grows more
coupled:

```text
FC + GCS speak normal HILink.
Radio + GS firmware own LoRa dialect, scheduling, rate limiting, and link budget.
RF link carries compact radio-internal messages.
```

The desired end state is:

```text
GCS
  normal HILink client/UI

GS radio module
  normal HILink <-> RF dialect
  scheduler
  link status owner

RF link
  compact radio-internal messages

Vehicle radio module
  normal HILink <-> RF dialect
  scheduler
  link budget owner

FC
  normal HILink endpoint
```

## Phase 0: Protocol Alignment Analysis

Goal: verify that the protocol update supports radio-owned scheduling and
dialect translation before changing firmware behavior.

Inputs to provide during this phase:

```text
- protocol README
- message ID table
- payload definitions
- encoding/decoding rules
- command/ack rules
- telemetry/event/fault definitions
- HIL/simulation message definitions
- notes about radio/link-budget constraints
```

Questions to answer:

```text
- Which messages are public normal HILink?
- Which messages are radio-internal RF dialect?
- Which messages are wired/simulation-only?
- Which messages are reliable, best-effort, latest-only, or on-change?
- Which normal messages translate into compact RF messages?
- Which compact RF messages translate back into normal messages?
- What fields are needed for stale rejection: seq, command_id, expires_ms, timestamps?
- Are compact RF payloads small enough for the LoRa frame budget?
```

Expected outputs:

```text
docs/radio-protocol-boundary.md
docs/radio-dialect-translation-matrix.md
docs/radio-scheduler-policy.md
```

This phase should stay open ended so updated protocol notes, a protocol README,
or external protocol analysis can be dropped in as source material.

## Phase 1: Radio Firmware Boundary Cleanup

Target: `device/MARV-RADIO-RP2354A`

Current problem: `core0.rs` and `lora_bridge.rs` mostly act like a transparent
HILink bridge. We need a semantic layer between UART HILink and RF frames.

Create modules like:

```text
src/radio_dialect/mod.rs
src/radio_dialect/normal.rs
src/radio_dialect/rf.rs
src/radio_dialect/translate.rs
src/radio_dialect/scheduler.rs
src/radio_dialect/state_cache.rs
src/radio_dialect/policy.rs
```

Responsibilities:

```text
normal.rs:
  Decode/encode normal HILink frames from/to UART.

rf.rs:
  Decode/encode compact RF dialect payloads.

translate.rs:
  Convert normal HILink <-> compact RF dialect.

state_cache.rs:
  Keep latest telemetry/state from FC or GCS.

scheduler.rs:
  Select next RF packet based on priority, rate, freshness, and link health.

policy.rs:
  Default rates, priority classes, stale timeouts, queue depths.
```

## Phase 2: GS Firmware Behavior

The GS-side radio firmware should expose a normal HILink interface to the GCS.

GCS to GS:

```text
GCS sends normal HILink command.
GS decodes it.
GS classifies priority.
GS translates command to RF dialect if needed.
GS scheduler sends it over RF.
```

GS from RF:

```text
GS receives compact RF telemetry/event/ack.
GS decodes RF dialect.
GS translates into normal HILink.
GS writes normal HILink to GCS serial.
```

GS should own:

```text
- GCS command validation at the transport/protocol boundary
- command expiry before RF send
- RF priority queueing
- link-status generation
- translation from compact telemetry to GCS-friendly HILink messages
```

GCS should not know:

```text
- LoRa frame format
- RF profiles
- airtime budget
- compact payload layout
- telemetry throttling rules
```

## Phase 3: Vehicle Radio Firmware Behavior

The vehicle-side radio firmware should expose a normal HILink interface to the
FC.

FC to radio:

```text
FC sends normal HILink telemetry/event/ack.
Radio decodes it.
Radio updates latest-state cache or event queue.
Radio scheduler chooses compact RF packets to send.
```

Radio from RF:

```text
Radio receives compact RF command.
Radio decodes RF dialect.
Radio translates into normal HILink command.
Radio writes normal HILink to FC UART.
```

Vehicle radio should own:

```text
- telemetry coalescing
- snapshot generation
- event/fault queueing
- command priority
- RF command duplicate/stale rejection where possible
- adaptive telemetry rates based on link health
```

FC should not know:

```text
- LoRa dialect
- radio profiles
- link budget
- telemetry downsampling
- RF packet size
```

Phase 3 implementation checkpoint:

```text
- Vehicle normal TelemetrySnapshot/Gps no longer transmit immediately over RF.
- Vehicle radio translates FC telemetry into compact RF snapshots and stores
  them as latest-only scheduler state.
- Vehicle scheduler emits compact flight snapshots at 5 Hz and GPS snapshots at
  1 Hz when fresh data has arrived.
- Vehicle command ACK/NACK remains P1/immediate and uses stored normal/RF
  correlation.
- Vehicle RF command duplicate detection prevents duplicate commands from being
  re-forwarded to the FC and queues compact duplicate ACKs back to RF.
- Unsupported vehicle RF commands are rejected at the radio boundary with a
  compact command ACK instead of leaking RF dialect details to the FC.
```

Still remaining for later scheduler phases:

```text
- event/fault queueing and latching
- profile-driven/adaptive telemetry rates
- command retry windows and expiry based on send-time metadata
- generated LoRaLinkStatus/normal link-health publication
```

## Phase 4: Scheduler Design

The scheduler should be the core of the radio firmware.

Priority model:

```text
P0 Critical:
  abort
  disarm
  motor stop
  safety actions

P1 Command/control:
  command acks
  command responses
  mode changes
  rate/profile changes
  explicit snapshot requests

P2 Events/faults:
  state transitions
  fault asserted
  fault cleared
  warnings
  important one-shot events

P3 Core telemetry:
  compact flight snapshot
  state/mode
  altitude
  vertical velocity
  battery
  estimator status

P4 Background:
  GPS detail
  link status
  diagnostics
```

Scheduler loop:

```text
1. Send pending P0.
2. Send pending P1.
3. Send pending P2 if not over event budget.
4. Send P3 if due.
5. Send P4 if due.
6. Otherwise listen.
```

Telemetry policy:

```text
- Commands are queued.
- Acks are queued.
- Events are queued or latched.
- Telemetry is latest-only.
- GPS is latest-only and slower.
- Link status is local/generated and slow.
- High-rate HIL traffic is not blindly forwarded over RF.
```

Example initial rates:

```text
Flight snapshot: 5-10 Hz
GPS snapshot: 1-2 Hz
Link status: 0.5-1 Hz
Fault/event: on change, repeated briefly until observed or aged out
Commands/acks: immediate/preemptive
```

Phase 4 implementation checkpoint:

```text
- Scheduler selection is now semantic RF traffic, not opaque HILink traffic:
  P1 command ACKs, P2 events/fault snapshots, P3 flight snapshots, then P4
  GPS/link status.
- FC SystemState/TelemetrySnapshot changes can queue compact state-change
  events and update a compact fault snapshot without exposing RF dialect to FC.
- Link status is generated locally from LoRa health counters and emitted as a
  compact RF status at the background rate.
- Flight/GPS telemetry remains latest-only and cannot starve command ACKs or
  event/fault traffic.
```

## Phase 5: RF Dialect Encoder/Decoder

The RF dialect should be internal to radio firmware.

It should support compact messages like:

```text
RfCommand
RfCommandAck
RfFlightSnapshot
RfGpsSnapshot
RfFaults
RfEvent
RfLinkStatus
RfRequestSnapshot
RfSetProfile
```

Design requirements:

```text
- bounded max encoded size
- scaled integers where possible
- no repeated static metadata
- explicit units
- sequence/command IDs
- expiry/deadline for commands
- rejection/status reasons
- enough timestamping for freshness
```

Translation examples:

```text
Normal Arm/Disarm/Rtl/MotorStop
  -> RfCommand
  -> Normal Arm/Disarm/Rtl/MotorStop

Normal TelemetrySnapshot/SystemState/Battery
  -> RfFlightSnapshot
  -> Normal TelemetrySnapshot/SystemState

Normal Gps
  -> RfGpsSnapshot
  -> Normal Gps

Normal Fault/Event
  -> RfFaults/RfEvent
  -> Normal Fault/Event

Normal Ack/Nack
  -> RfCommandAck
  -> Normal Ack/Nack or command response
```

Phase 5 implementation checkpoint:

```text
- RF packets are strictly `rf_msg_type + fixed payload + crc16`.
- Decoder rejects unknown RF message types and CRC-valid packets with the wrong
  payload length for their RF message type.
- Encoder exposes bounded max RF payload/packet sizes and asserts that the
  largest compact RF packet fits inside the LoRa frame payload budget.
- Payload-specific decoding rechecks the expected message type and exact payload
  length before producing a typed RF payload.
```

## Phase 6: FC Changes

Target: `device/MARV-FC-SP-RP2354B`

The FC radio endpoint should evolve from `Ping -> Pong` into a normal HILink
endpoint.

FC should handle normal messages only:

```text
Ping
Arm
Disarm
Rtl
MotorStop
ActuatorStatusRequest
normal command envelope if the protocol adds one
HilSensorFrame if local/wired HIL path supports it
```

FC should emit normal messages only:

```text
Pong
Ack/Nack
Heartbeat
TelemetrySnapshot
SystemState
Gps
Battery
Fault/Event
ActuatorStatus
HilResponseFrame where appropriate
```

Remove or avoid FC dependency on:

```text
LoRaCommand
LoRaFlightSnapshot
LoRaGpsSnapshot
LoRaLinkStatus
RF profile commands
RF scheduling concepts
```

Phase 6 implementation checkpoint:

```text
- FC radio UART endpoint no longer imports LoRa frame sizing or RF/link-budget
  constants; its buffering is sized from normal HILink payload contracts.
- FC radio endpoint accepts only normal HILink frames and explicitly handles
  Ping, Arm, Disarm, Rtl, MotorStop, ActuatorStatusRequest, and HilSensorFrame.
- Ping receives normal Pong. ActuatorStatusRequest receives normal
  ActuatorStatus plus Ack.
- Arm/Disarm/Rtl/MotorStop/HilSensorFrame are decoded as normal HILink and
  rejected with normal Nack until the FC command/HIL execution services exist.
- FC radio endpoint emits normal Heartbeat and SystemState periodically using
  FlightPhase wire codes; no RF dialect message is emitted by FC firmware.
```

## Phase 7: Ground Control Software Changes

The GCS should implement normal HILink only.

It should:

```text
- encode/decode HILink UART frames
- maintain command seq
- track pending command acks
- display telemetry/events/faults/link status
- tolerate unsolicited messages
- use Ping/Pong as an end-to-end health check
```

It should not:

```text
- build LoRa frames
- emit compact RF dialect messages
- know airtime/radio profile rules
- special-case FC behavior based on radio link
```

One exception: the GCS may display radio health if the GS exposes it as a
normal HILink link-status or health message.

## Phase 8: Migration Path

Keep the current bring-up path working while refactoring.

Step order:

```text
1. Keep normal HILink Ping/Pong pass-through working.
2. Add radio_dialect module with no-op pass-through mode.
3. Move current priority classification into scheduler policy.
4. Add state cache and latest-only telemetry handling.
5. Add RF dialect structs/codecs.
6. Translate one message first: Ping/Pong or LinkStatus.
7. Translate command path next.
8. Translate compact telemetry snapshots.
9. Disable transparent forwarding for messages that now have dialect translations.
10. Add rejection/throttle behavior for oversized/high-rate messages.
```

## Phase 9: Tests

Add protocol and firmware-level tests for:

```text
- normal HILink command -> RF command translation
- RF command -> normal HILink command translation
- telemetry coalescing keeps latest only
- P0 preempts P3/P4
- command ack is never starved by telemetry
- stale command is rejected
- oversized normal frame is rejected or summarized
- RF dialect messages never need to be constructed by FC/GCS
- Ping/Pong still works end-to-end
```

## Final Shape

The scheduler becomes the radio firmware's brain, and the dialect
encoder/decoder becomes its language layer. The FC and software stay
radio-agnostic.

The final ownership model should be:

```text
FC:
  normal HILink endpoint

Vehicle radio:
  normal HILink <-> RF dialect
  scheduler
  link budget owner

GS radio:
  RF dialect <-> normal HILink
  scheduler
  link status owner

GCS:
  normal HILink client/UI
```

## Bench Solidification Implementation Plan

This section captures the next implementation pass after Phase 6 bring-up. The
goal is to harden the radio link itself while accepting that GPS, estimator,
battery, and actuator/mixer data are not fully available yet.

The guiding rule remains:

```text
PC/GCS and FC speak normal HILink.
GS and vehicle radio own RF dialect, scheduling, retries, compact payloads,
and link-health policy.
```

### Milestone A: Command Correlation and Response Behavior

Goal: prove every public command sent by the PC is either matched to the
correct normal `Ack`/`Nack`/`Pong`, or times out cleanly in the PC tool.

Implementation tasks:

```text
1. Add explicit bench tests for normal Ping, Arm, Disarm, and MotorStop.
2. Verify GS stores normal seq/msg_type -> RF command_id/command_seq
   correlation before RF transmit.
3. Verify vehicle radio stores RF command_id/command_seq -> normal seq/msg_type
   correlation before forwarding to FC.
4. Verify FC unsupported commands return normal Nack with original seq/msg_type.
5. Verify vehicle radio translates normal Nack into LoRaCommandAck.
6. Verify GS translates LoRaCommandAck back into normal Nack with the original
   PC seq/msg_type.
7. Keep Ping special-cased as normal Ping -> RF command -> FC Pong -> RF ack
   -> normal Pong until the public protocol grows a correlated Pong.
```

Acceptance checks:

```text
- PC sends Arm seq=N and receives Nack(rejected_seq=N, rejected_msg_type=Arm).
- PC sends Disarm seq=N and receives Nack(rejected_seq=N, rejected_msg_type=Disarm).
- PC sends MotorStop seq=N and receives Nack(rejected_seq=N, rejected_msg_type=MotorStop).
- PC sends Ping and receives normal Pong.
- Unmatched RF command acks are dropped and counted, not emitted as bogus
  normal acks.
```

Later, when FC command execution exists:

```text
- Replace unsupported Nack behavior with real command dispatch.
- Preserve the same correlation path for Ack/Nack.
- Add command-result reasons for denied state, safety inhibit, busy, invalid
  argument, and execution failure.
```

### Milestone B: Duplicate and Stale Command Behavior

Goal: prevent RF retransmits or operator command spam from causing repeated FC
execution.

Implementation tasks:

```text
1. Make the vehicle radio duplicate cache explicit in policy:
   command_id + command_seq history depth, eviction behavior, and reason codes.
2. On duplicate RF command:
   - do not forward to FC again
   - queue LoRaCommandAck with DUPLICATE_ACCEPTED or DUPLICATE_REJECTED based
     on the original outcome policy
3. Add command expiry checks using LoRaCommand.expires_ms and receive time.
4. Reject expired commands at the vehicle radio boundary with LoRaCommandAck
   status=REJECTED reason=EXPIRED.
5. Add unit tests around wraparound command_seq and duplicate history eviction.
```

Acceptance checks:

```text
- Replaying the same RF command_id/command_seq does not produce a second FC
  normal command frame.
- Expired RF command produces a normal Nack at the PC after GS rehydration.
- New command_seq for the same command_id still forwards normally.
```

### Milestone C: Link Status Visibility

Goal: expose RF health to the PC/GCS as normal public telemetry, not RF dialect.

Implementation tasks:

```text
1. Decide the normal public message:
   - Prefer RadioStatus if its payload is sufficient.
   - Otherwise add a normal-facing link-health payload before exposing raw
     LoRaLinkStatus.
2. Continue generating LoRaLinkStatus inside radio firmware only.
3. On GS, translate LoRaLinkStatus into the selected normal public status
   message.
4. Include at least:
   - link state
   - latest RSSI/SNR
   - rx/tx packet deltas
   - missed/lost packet deltas
   - active profile
   - telemetry rate
5. Keep link status unsolicited and low rate.
```

Acceptance checks:

```text
- PC receives normal link-status telemetry without knowing LoRaLinkStatus.
- Link status continues when no commands are being sent.
- Link status does not starve command acks or flight snapshots.
```

### Milestone D: Queue and Backpressure Behavior

Goal: command acknowledgements and safety commands must not be starved by
telemetry, and command spam must not wedge telemetry forever.

Implementation tasks:

```text
1. Document queue depths for each priority class in policy.rs.
2. Add counters for each queue overflow/drop path.
3. Ensure scheduler order is:
   - pending command acks
   - P0/P1 host command traffic
   - event/fault traffic
   - flight snapshot
   - GPS snapshot
   - link status/background
4. Add an anti-wedge policy for sustained P0/P1 load:
   - command acks remain immediate
   - low-rate telemetry gets occasional slots once command queues drain or
     after a bounded starvation interval
5. Make latest-only telemetry overwrite older snapshots instead of queueing
   multiple stale copies.
```

Acceptance checks:

```text
- A command ack is sent before queued telemetry.
- Telemetry burst cannot delay Ack/Nack beyond one scheduler opportunity.
- Command spam increments drop/backpressure counters instead of blocking the
  bridge task.
- Latest telemetry after a burst is current, not a backlog of old snapshots.
```

### Milestone E: Bench Observability

Goal: make bench failures diagnosable from logs and counters without needing to
guess which boundary failed.

Implementation tasks:

```text
1. Add structured defmt logs for:
   - normal host frame accepted/dropped
   - normal msg_type -> RF msg_type translation
   - RF msg_type -> normal msg_type translation
   - scheduler-selected RF payload kind
   - command correlation stored/taken/missed
   - queue overflows
   - stale snapshot drops/overwrites
2. Add rate-limiting for noisy telemetry logs.
3. Add per-boundary counters in RadioStateCache or a small stats module:
   - host_rx_frames
   - host_rx_dropped
   - rf_tx_frames
   - rf_rx_frames
   - rf_rx_dropped
   - normal_to_rf_translated
   - rf_to_normal_translated
   - command_correlation_miss
   - duplicate_command_rejected_or_acked
   - expired_command_rejected
   - queue_overflow_by_priority
4. Expose a compact summary in logs every N seconds during bench builds.
```

Acceptance checks:

```text
- Given only GS, vehicle radio, and FC logs, the failed hop can be identified.
- Sending one Ping produces one clear trace through each relevant boundary.
- Telemetry logs are useful without flooding RTT.
```

### Milestone F: Rate Policy Cleanup

Goal: make telemetry and status rates explicit, tunable, and owned by radio
policy rather than scattered constants.

Implementation tasks:

```text
1. Keep default bench rates:
   - flight snapshot: 5 Hz
   - GPS snapshot: 1 Hz
   - link status: 0.5 Hz to 1 Hz
2. Move radio scheduler rates into policy.rs with names tied to behavior:
   VEHICLE_FLIGHT_SNAPSHOT_PERIOD_MS
   VEHICLE_GPS_SNAPSHOT_PERIOD_MS
   LINK_STATUS_PERIOD_MS
3. Keep FC normal telemetry emitter rates in FC config while estimator/GPS are
   not fully wired, but name them as FC normal endpoint rates.
4. Add comments stating that RF rates are controlled by radio scheduler, not FC
   or PC software.
5. Later, add profile-driven rates through LoRaSetProfile or a normal public
   profile command translated by GS.
```

Acceptance checks:

```text
- Rate changes require editing one obvious policy/config location.
- PC software does not need to request telemetry to receive it.
- FC can emit normal data faster than RF without increasing RF rate; vehicle
  radio coalesces latest-only snapshots.
```

### Milestone G: Temporary Debug Surfaces

Goal: support bench investigation without polluting the flight radio contract.

Implementation tasks:

```text
1. Decide if raw IMU/baro debug is needed over radio.
2. If needed, add an explicit bench/debug mode with low rates:
   - raw baro: 1 Hz
   - raw IMU summary, not full-rate stream: 1 Hz to 2 Hz
3. Keep debug messages normal HILink at PC/FC boundaries.
4. Translate debug traffic through either:
   - a compact RF debug payload, or
   - a deliberately low-priority normal debug message if the payload fits
5. Ensure debug traffic is P5/background and never competes with command acks.
6. Make debug mode compile-time or explicitly command-enabled so it cannot
   accidentally become the flight default.
```

Acceptance checks:

```text
- Debug mode off: no raw sensor traffic crosses RF.
- Debug mode on: low-rate raw/summarized debug appears at PC.
- Command/Pong/Nack behavior is unchanged with debug enabled.
```

## Suggested Execution Order

Use this order so each step improves bench confidence without depending on the
future estimator or motor stack:

```text
1. Milestone A: command correlation bench checks.
2. Milestone E: observability counters/logs for the existing path.
3. Milestone B: duplicate/stale command rejection.
4. Milestone D: queue/backpressure counters and scheduler fairness.
5. Milestone F: rate policy cleanup.
6. Milestone C: normal GCS-visible link status.
7. Milestone G: optional explicit debug surface.
```

The missing GPS, estimator, battery, and mixer systems should populate richer
normal payloads later. They should not change the radio boundary or require the
PC/FC to learn RF dialect details.
