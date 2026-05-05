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
