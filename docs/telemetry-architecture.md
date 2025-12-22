# Telemetry Architecture Plan (Rust + Embassy + LoRa + USB CDC)

This document plans a clean telemetry architecture for **MARV-FC** inspired by:

- **ArduPilot**: “single source of truth” subsystems + producers (“streams”) + per-link scheduling.
- **PX4**: pub/sub topics (uORB) + a MAVLink module that subscribes and rate-controls.
- **Betaflight**: minimal overhead + transport-specific constraints (especially for low-bandwidth links).

The goal is to keep the **data model**, **telemetry producers**, **MAVLink encoding**, and **transport/link** separated so we can:

- send the *same telemetry* over **LoRa** and **USB CDC** with different rates
- evolve telemetry without breaking transports
- keep bandwidth-aware scheduling centralized (LoRa needs it)

---

## 0) Current Codebase Snapshot (what exists today)

### Transports / Links

- `common/src/coms/uart_coms.rs`: async MAVLink framing over a generic async UART bus.
- `common/src/coms/transport/uart.rs`: canonical location for async UART MAVLink framing.
- `common/src/coms/usb_cdc.rs`: async write-only USB CDC abstraction.
- `common/src/coms/transport/lora/link.rs`: LoRa “datagrams” (`LoRaLink`) with MTU + basic stats (no link-level ARQ).
  - (A compatibility shim still exists at `common/src/lora/mod.rs` for older imports.)

### MAVLink

- `common/src/mavlink2/`:
  - `msg.rs`: MAVLink frame builders (heartbeat/statustext/params) **plus** a compact telemetry payload (`TelemetrySample`) encoded into `EncapsulatedData`.
  - `handlers.rs`, `param_handler.rs`: protocol dispatch for PARAM_*.
  - `mod.rs`: includes `send_frame_over_lora` + `send_frame_over_uart` (currently couples MAVLink ↔ transports).

### Truth / State

- Device firmware keeps state in `device/rp235x/src/main.rs`:
  - `STATE: Mutex<..., SensorsState>` and a `StateTelemetrySource` implementing `TelemetrySource`.
- Parameters are already treated as “single source of truth”:
  - `common/src/params.rs`: `ParamRegistry` + definitions.
- Commands are also “single source of truth”:
  - `common/src/commands.rs`: CLI command registry.

### SD Persistence (device-specific)

- `device/rp235x/src/hardware/sd_params.rs`: load/save params/config via SD.

---

## 1) Target Architecture (layered model)

We want this layering (same mental model as ArduPilot/PX4):

```
┌──────────────────────────────────────────────┐
│ Truth / State Layer                          │
│  - sensors state, estimator state, health    │
│  - ParamRegistry, Command registry           │
└──────────────────────────────────────────────┘
                    ↓
┌──────────────────────────────────────────────┐
│ Telemetry Producers (views over truth)       │
│  - AttitudeProducer, SystemHealthProducer…   │
│  - read-only access to truth                 │
└──────────────────────────────────────────────┘
                    ↓
┌──────────────────────────────────────────────┐
│ Protocol / Encoding Layer                    │
│  - MAVLink message builders + parsers        │
│  - param/command/telemetry mapping           │
└──────────────────────────────────────────────┘
                    ↓
┌──────────────────────────────────────────────┐
│ Transport / Link Layer                       │
│  - UART byte-stream, USB CDC byte-stream     │
│  - LoRa reliable datagrams                   │
└──────────────────────────────────────────────┘
                    ↓
┌──────────────────────────────────────────────┐
│ Scheduler / Routing (glue)                   │
│  - stream configs, rate limits, priorities   │
│  - per-link budgets                          │
└──────────────────────────────────────────────┘
```

Key rule (ArduPilot rule): **Telemetry never owns state.**

Key rule (PX4 rule): scheduling and rate limits are centralized.

Key rule (Betaflight constraint): keep overhead low; no heap required; no complex runtime reflection.

---

## 2) How to Leverage Rust/Async (Embassy-friendly)

Embassy gives you great building blocks to implement both “ArduPilot-style” and “PX4-style” patterns.

### Option A (ArduPilot-like): snapshot polling

- Control/sensor tasks update shared state in a `Mutex` (you already do this with `STATE`).
- Scheduler calls producers periodically; each producer takes a snapshot and encodes output.

Pros:
- Very simple to reason about
- No channel backpressure / queue tuning
- Works well with `no_std`

Cons:
- Some redundant reads at high rates

### Option B (PX4-like): topic pub/sub using Embassy channels

- Each subsystem publishes topic updates (e.g. `imu`, `baro`, `gps`, `health`).
- Scheduler subscribes; keeps the latest value per topic.

Embassy tools:
- `embassy_sync::channel::Channel` for queued messages (careful with backpressure)
- `embassy_sync::signal::Signal` (or a “latest value” pattern) for “always keep last state”

Pros:
- Clear data ownership and update flow
- Scheduler can react immediately

Cons:
- More runtime tuning (queue sizes)

Recommendation for MARV-FC right now:
- Keep **Option A** for bring-up (you already have it working).
- Introduce **Option B** selectively later for hot data paths (IMU) or event-like data (mode changes, failsafes).

---

## 3) Proposed Folder / Module Re-org (your notes applied)

You called out two excellent cleanup moves:

1) **All transports in one folder** (LoRa should live under comms).
2) **All MAVLink-encoded things colocated** (params/commands/telemetry are all carried over MAVLink).

### 3.1 Transport layer: consolidate under `common/src/coms/transport/`

Current:
- `common/src/coms/uart_coms.rs`
- `common/src/coms/usb_cdc.rs`
- `common/src/lora/link.rs`

Target:

```
common/src/coms/
  mod.rs
  transport/
    mod.rs
    uart.rs        (from uart_coms.rs)
    usb_cdc.rs     (keep)
    lora/
      mod.rs
      link.rs      (from lora/link.rs)
```

Notes:
- `LoRaLink` is a **link layer** (reliable datagrams). That’s still “transport” from the perspective of higher layers.
- Keep the `Sx1262Interface` trait and SX1262 driver in `drivers/`—those are hardware drivers, not comms policy.

### 3.2 Protocol layer: rename/reshape `mavlink2` into a protocol module

Current:
- `common/src/mavlink2/` mixes:
  - message builders (`msg.rs`)
  - param handlers
  - transport glue functions (`send_frame_over_lora`, `send_frame_over_uart`)

Target:

```
common/src/protocol/
  mod.rs
  mavlink/
    mod.rs
    encode.rs          (pure MAVLink message building)
    decode.rs          (parsing helpers if needed)
    params.rs          (PARAM_* mapping using ParamRegistry)
    commands.rs        (COMMAND_* mapping using Command registry)
    telemetry.rs       (mapping producer outputs → MAVLink)
    transport_uart.rs  (MAVLink-over-UART framing adapter)
    transport_lora.rs  (MAVLink-over-LoRa datagram adapter)
```

Why split adapters?
- The **protocol** layer should be able to build/parse frames without knowing about LoRa pacing or UART reads.
- The **transport adapters** are thin wrappers that bridge protocol ↔ transport traits.

### 3.3 Telemetry layer: create a dedicated “producers + scheduler” module

Add:

```
common/src/telemetry/
  mod.rs
  frame.rs        (internal logical telemetry frames, or typed outputs)
  producer.rs     (traits + producer IDs)
  scheduler.rs    (streams, rates, priorities, link budgets)
```

This is the layer that ArduPilot calls “streams” and PX4 calls “mavlink streams”.

---

## 4) Telemetry Producers (how to design them)

### 4.1 Design goals (ArduPilot-inspired)

Each producer:
- reads from truth layer (state + params)
- produces exactly one “logical message” (e.g. `Attitude`, `RawImu`, `GpsFix`, `Health`)
- knows **nothing** about LoRa/USB/UART

### 4.2 Trait proposal (no heap, no dynamic dispatch required)

A minimal, embedded-friendly interface:

```rust
pub enum ProducerId {
    Heartbeat,
    Statustext,
    RawImu,
    Gps,
    Health,
}

pub enum TelemetryFrame {
    // Internal, transport-agnostic representation.
    // You can later map these to MAVLink messages.
    RawImu { accel: [i16;3], gyro: [i16;3] },
    Gps { lat_e7: i32, lon_e7: i32, alt_mm: i32, sats: u8, fix: u8 },
    Health { flags: u32 },
}

pub trait TelemetryProducer {
    fn id(&self) -> ProducerId;

    // Called by scheduler. Return None if producer has nothing new.
    async fn produce(&mut self) -> Option<TelemetryFrame>;
}
```

A “truth handle” can be injected as references:
- `&Mutex<..., SystemState>` for snapshots
- `&Mutex<..., ParamRegistry>` for configuration-driven behavior

This matches how you already implemented `StateTelemetrySource`.

### 4.3 Mapping to existing code

You already have a proto-producer pattern:

- `TelemetrySource` (in `mavlink2/msg.rs`) returns `TelemetrySample`.
- `StateTelemetrySource` (in device main) snapshots `STATE`.

That’s a good *starting point*.

The next evolution is to split:
- telemetry producers → return logical `TelemetryFrame` variants (multiple message types)
- protocol layer → converts `TelemetryFrame` into MAVLink frames

---

## 5) Scheduler / Stream Manager (LoRa-first constraints)

This is the “glue” layer ArduPilot does with SRx_* streams and per-link rate control.

### 5.1 Core responsibilities

- owns per-link stream configs
- enforces per-message intervals
- enforces link budget (bytes/sec and/or “time gap”)
- prioritizes messages (e.g. heartbeat > health > GPS > raw IMU)

### 5.2 Link budgets (what we can use today)

For LoRa, you already have:
- `LoRaLink::MTU`
- `LoRaLink::recommended_tx_gap_ms()` driven by RSSI/SNR pacing

So scheduler can apply:
- hard cap: never send frames > MTU
- time-based budget: never send faster than `recommended_tx_gap_ms`

For USB CDC, budget is typically generous; use a simple maximum rate to avoid spamming.

### 5.3 Suggested stream groups (ArduPilot-inspired)

Define 3–4 stream groups:

- **FAST**: heartbeat + minimal attitude/imu (only if link allows)
- **NAV**: GPS + baro
- **HEALTH**: status flags + statustext
- **DEBUG**: verbose developer-only messages

Tie stream enable/rates to params (you already have `TEL_RATE_HZ`, `HB_EN`, `STATUSTXT_EN`).

---

## 6) MAVLink: keeping params + commands + telemetry together (without tangling truth)

You’re right that “params, commands, telemetry” are all encoded over MAVLink.

But keep the separation:

- Truth:
  - `common/src/params.rs` stays as authoritative registry
  - `common/src/commands.rs` stays as authoritative command registry

- MAVLink mapping:
  - lives under `common/src/protocol/mavlink/`
  - contains:
    - PARAM_* mapping (already exists in `mavlink2/msg.rs` + handlers)
    - command mapping (future: `COMMAND_LONG`, `COMMAND_INT`)
    - telemetry mapping (EncapsulatedData or typed MAVLink messages)

This prevents an architectural trap:
- If you move `params.rs` under “mavlink”, you’ll start coupling non-MAVLink consumers (USB CLI, SD persistence) to MAVLink.

Instead, colocate the *MAVLink adapters*.

---

## 7) Concrete Migration Plan (incremental, low-risk)

### Step 1 — consolidate transports (mechanical move)

- Move `common/src/lora/` to `common/src/coms/transport/lora/`
- Rename `uart_coms.rs` → `common/src/coms/transport/uart.rs`
- Update imports:
  - anything using `crate::lora::link::LoRaLink` becomes `crate::coms::transport::lora::link::LoRaLink` (or re-export it)

Definition of done:
- firmware builds unchanged
- no behavior change

### Step 2 — make MAVLink protocol module transport-agnostic

- Move `common/src/mavlink2/msg.rs` → `common/src/protocol/mavlink/encode.rs`
- Move handlers into `common/src/protocol/mavlink/params.rs` etc.
- Keep UART framing helper (currently in `uart_coms.rs`) under transport.
- Replace `send_frame_over_lora`/`send_frame_over_uart` with thin adapter modules:
  - `protocol/mavlink/transport_uart.rs`
  - `protocol/mavlink/transport_lora.rs`

Definition of done:
- call sites in `device/rp235x/src/main.rs` still send/receive MAVLink over LoRa/UART
- no new alloc, no new tasks required

### Step 3 — introduce `telemetry/` module with 1–2 producers

- Create `common/src/telemetry/producer.rs` and implement:
  - `RawImuProducer` reading from existing `STATE`
  - `GpsProducer` reading from existing `STATE`

Initially keep encoding as EncapsulatedData (your current compact payload) to avoid schema churn.

Definition of done:
- USB CDC and LoRa both reuse the same producer(s)
- LoRa stream sends at a lower configured rate

### Step 4 — add scheduler that is LoRa-aware

- Scheduler runs in the comms task loop:
  - uses `LoRaLink::recommended_tx_gap_ms()`
  - prioritizes heartbeat + health
  - rate-controls telemetry

Definition of done:
- LoRa no longer drops frames due to bursty sends
- “FAST” stream can be enabled/disabled without touching transport code

### Step 5 — (optional) PX4-like topics for hot paths

Only if/when needed:
- Publish IMU updates to a channel
- Scheduler consumes latest IMU sample without locking a big state mutex

---

## 8) Notes for Your Current Progress (what to keep, what to change)

Keep:
- `ParamRegistry` as the single source of truth.
- `CommandDef` table as the single source of truth.
- LoRa deterministic tick MAC + QoS queues (this is the right direction for low jitter).

Change next:
- Move LoRa under `coms/transport` so layering becomes obvious.
- Move MAVLink “transport glue” out of the core protocol module.
- Expand “telemetry = one `TelemetrySample`” into multiple producers.

---

## 9) Suggested “Definition of Done” for Telemetry Architecture v1

- One telemetry producer is reused by both USB and LoRa.
- LoRa and USB streams have separate rates (parameter-controlled).
- MAVLink encode/decode does not import SX1262/LoRaLink types directly.
- Transport code does not import params/commands/telemetry types directly.

---

## 10) Quick follow-ups (choose one direction)

If you want, I can implement the first mechanical refactor next:

- Move LoRa into `common/src/coms/transport/lora/`
- Adjust `common/src/mavlink2` imports accordingly
- Keep all public APIs stable via re-exports so device crates don’t churn
