# FC State + HEARTBEAT Proposal (Standard MAVLink)

Date: 2025-12-21

## Goal

Make the FC publish an unambiguous **state** over MAVLink so:

- GS + Radio can enforce “no modem changes while armed”.
- Bench UX is predictable:
  - **USB CDC connected → MAV_STATE_STANDBY + MODE_CONFIG**
  - **Power-on with no external connection → MAV_STATE_STANDBY + MODE_IDLE**
  - **ARMED** can be toggled by MAVLink command.

Constraint: **use standard MAVLink** (no custom messages). Custom *modes* are allowed via standard fields (`custom_mode`).

## TL;DR (What we will implement)

1. FC emits `HEARTBEAT` continuously.
2. FC encodes arming status in `HEARTBEAT.base_mode` (standard).
3. FC encodes “CONFIG vs IDLE” as a **mode** via `custom_mode` (standard field) + optional `STATUSTEXT` for human clarity.
4. FC accepts:
   - `MAV_CMD_COMPONENT_ARM_DISARM` to arm/disarm.
  - `MAV_CMD_DO_SET_MODE` to switch between MODE_IDLE/MODE_CONFIG (when allowed).
5. **USB CDC connected forces CONFIG** (cannot exit CONFIG while USB is connected).

Additionally:

- `HEARTBEAT` is treated as an **end-to-end “online” indicator** because it originates at FC and is forwarded FC → Radio → GS → USB CDC.

## Why this is valid (against common firmware patterns)

### What PX4 / ArduPilot do (high level)

- They **always** use `HEARTBEAT.base_mode` to indicate armed/disarmed (`MAV_MODE_FLAG_SAFETY_ARMED`).
- They use `HEARTBEAT.custom_mode` to encode flight/operation modes.
- They use `system_status` (`MAV_STATE_*`) for coarse system status (standby/active/calibrating/etc).
- They do **not** typically encode “GCS is connected” as a primary state in HEARTBEAT. GCS connection is usually inferred from traffic / timeouts.

### What we’re doing differently (and why it’s still OK)

- We *do* want a “CONFIG” concept because it changes what the system should allow (e.g., RF changes).
- MAVLink does not have a universal “config mode” flag.
- The most compatible standard way to represent a project-specific “mode” is:
  - use `custom_mode` (standard field) +
  - implement `MAV_CMD_DO_SET_MODE` +
  - optionally emit `STATUSTEXT`.

This mirrors how autopilots represent non-universal modes while remaining within standard MAVLink.

### Suggested adjustment (compatibility-minded)

To avoid surprising other MAVLink tooling:

- Keep **arming** strictly in `base_mode`.
- Treat CONFIG/IDLE primarily as a **custom mode** (via `custom_mode`), not as a new meaning for `system_status`.
- Use `system_status` conservatively:
  - MODE_IDLE (disarmed) → `MAV_STATE_STANDBY`
  - ARMED → `MAV_STATE_ACTIVE`
  - MODE_CONFIG (disarmed) → `MAV_STATE_STANDBY` (plus mode=MODE_CONFIG)

That keeps standard semantics intact while still giving GS/Radio what they need.

## State model

### States

We model **two orthogonal axes**:

- **Arming**: `DISARMED` vs `ARMED`
- **Operation mode** (project-local, encoded in `custom_mode`): `MODE_IDLE` vs `MODE_CONFIG`

Rules:

- USB CDC connected → mode is forced to `MODE_CONFIG`.
- Arming allowed only when mode is `MODE_IDLE` (recommended safety rule).
- RF/LoRa modem config changes are rejected while armed (already a project constraint).

### Transition rules

- Boot → `DISARMED + MODE_IDLE`
- USB connected → `DISARMED + MODE_CONFIG`
- USB disconnected → return to `DISARMED + MODE_IDLE`
- `MAV_CMD_DO_SET_MODE`:
  - If USB connected: reject (recommended) because CONFIG is forced.
  - If USB disconnected and disarmed: allow switching MODE_IDLE⇄MODE_CONFIG.
- `MAV_CMD_COMPONENT_ARM_DISARM`:
  - Only allowed when USB disconnected and currently in MODE_IDLE.

**Policy choice (recommended):**

- When USB is connected and a `DO_SET_MODE(MODE_IDLE)` arrives: reject with failure and send a `STATUSTEXT` like “Denied: USB connected → CONFIG forced”.

That makes behavior explicit and easier to debug.

## MAVLink encoding details

### HEARTBEAT

We will emit:

- `HEARTBEAT.type`: set to your vehicle type (rocket vs quad) if you have it; otherwise use a safe default and add later.
- `HEARTBEAT.autopilot`: `MAV_AUTOPILOT_GENERIC` (or the closest appropriate value).
- `HEARTBEAT.base_mode`:
  - Always include `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`.
  - Include `MAV_MODE_FLAG_SAFETY_ARMED` when armed.
- `HEARTBEAT.custom_mode`: encodes our project modes.
- `HEARTBEAT.system_status`:
  - `MAV_STATE_STANDBY` when disarmed
  - `MAV_STATE_ACTIVE` when armed

### custom_mode values

We define (project-local) numeric constants:

- `CUSTOM_MODE_MODE_IDLE = 0`
- `CUSTOM_MODE_MODE_CONFIG = 1`

(If you later add flight modes, this becomes a richer enum.)

### STATUSTEXT (optional but recommended)

On every transition, emit `STATUSTEXT`:

- “STATE: IDLE”
- “STATE: CONFIG (USB)”
- “STATE: ARMED”

If we switch to MAVLink naming, the human strings should match:

- “STATE: MAV_STATE_STANDBY / MODE_IDLE”
- “STATE: MAV_STATE_STANDBY / MODE_CONFIG (USB)”
- “STATE: MAV_STATE_ACTIVE (ARMED)”

This is standard MAVLink and makes Mission Planner/QGC logs readable.

## Commands (standard MAVLink)

### Arm/disarm

- Command: `MAV_CMD_COMPONENT_ARM_DISARM`
- Behavior:
  - If request arm while USB connected → `COMMAND_ACK` failure (“Denied: CONFIG/USB”).
  - If request arm while already armed → ACK success (idempotent).
  - If request disarm → ACK success.

### Set mode

- Command: `MAV_CMD_DO_SET_MODE`
- We accept mode changes only while disarmed.
- Allowed targets:
  - `CUSTOM_MODE_MODE_IDLE`
  - `CUSTOM_MODE_MODE_CONFIG`

## Interaction with GS/Radio link policy

- GS/Radio treat FC `HEARTBEAT.base_mode` as authoritative for armed.
- If heartbeat missing → assume disarmed (bench fallback).
- GS enforces:
  - LoRa modem config changes: allowed only when disarmed.
  - Telemetry rate changes: allowed while armed.

## HEARTBEAT as end-to-end “online” indicator

You’re right that in our system the heartbeat itself is a status signal because it traverses the whole chain.

Rule set:

- FC emits `HEARTBEAT` on a fixed schedule.
- Radio forwards FC-origin `HEARTBEAT` over LoRa to GS.
- GS forwards that `HEARTBEAT` to USB CDC.

Implications:

- If Mission Planner/QGC sees FC heartbeats: the path FC → Radio → GS → USB is working.
- If the GS sees heartbeats over LoRa but the FC UART is down: the Radio can emit its own heartbeat, but it must not be confused with FC heartbeat (different sysid/compid).

Recommendation (to keep semantics clean):

- Keep FC heartbeat as the authoritative “vehicle state” heartbeat.
- Let Radio/GS optionally emit their own heartbeats (as separate components) for debugging, but do not substitute for FC state.

## Telemetry selection (leave policy flexible)

We do **not** need to finalize the full message set now.

Recommended minimum to implement first:

- `HEARTBEAT`
- `SYS_STATUS`
- `RADIO_STATUS` (link quality reporting)
- A small core set you already need for control/monitoring

Later:

- FC parameters define “telemetry profiles” by:
  - vehicle type (rocket/quadcopter)
  - system mode (idle/config)
  - armed state

## Parameters (FC-owned; changeable from GCS)

Preference: many useful parameters is better than too few.

### State + safety parameters

- `FC_MODE_USB_FORCES_CONFIG` (bool, default true)
- `FC_ARM_ALLOWED_IN_CONFIG` (bool, default false)
- `FC_STATE_HEARTBEAT_HZ` (u8, default 1 or 2)
- `FC_STATE_HEARTBEAT_TIMEOUT_MS` (u16/u32, default e.g. 2000) – used by Radio/GS to declare FC heartbeat missing

### Telemetry enable + rate parameters (baseline)

- `TEL_EN` (bool)
- `TEL_PROFILE` (u8) – selects a profile table (e.g., 0=default, 1=rocket, 2=quad)

Note:

- `TEL_*` parameters are **FC-owned telemetry policy** knobs.
- Wireless/LoRa modem settings are separate and should use a dedicated `LORA_*` namespace (defined in the wireless telemetry proposal).

### Telemetry profile: per-state target rates

These govern the *scheduler intent*; actual transmission may be clamped by link quality later.

- `TEL_RATE_STANDBY_HZ` (u8, default 5)
- `TEL_RATE_ACTIVE_HZ` (u8, default 20)
- `TEL_RATE_CONFIG_HZ` (u8, default 2)

### Telemetry adaptivity (FC-controlled)

- `TEL_ADAPT_EN` (bool)
- `TEL_ADAPT_STEP_HZ` (u8)
- `TEL_ADAPT_MIN_HZ` (u8)
- `TEL_ADAPT_MAX_HZ` (u8)
- `TEL_ADAPT_SNR_GOOD_X4` (i16)
- `TEL_ADAPT_SNR_BAD_X4` (i16)
- `TEL_ADAPT_HOLD_MS` (u16/u32)

### Telemetry message enables (coarse, standard-only)

This keeps us flexible without creating a huge per-message parameter explosion on day 1.

- `TEL_EN_SYS` (bool) – system/health set (`SYS_STATUS`, etc.)
- `TEL_EN_ATT` (bool) – attitude set (`ATTITUDE`, etc.)
- `TEL_EN_POS` (bool) – position/GPS set (`GPS_RAW_INT`, etc.)
- `TEL_EN_IMU` (bool) – raw IMU set (`RAW_IMU`, etc.)
- `TEL_EN_RADIO` (bool) – link stats (`RADIO_STATUS`)

### Future (optional): per-message rate table

If you later want “tons of parameters” down to per-message rates:

- `TEL_MSG_<MSGID>_HZ` (u8) pattern, generated from a list.

We should only do this once we know which messages matter, otherwise it becomes noisy.

## Decisions locked in

- Use `HEARTBEAT.base_mode` to signal armed/disarmed.
- Use `system_status` as `MAV_STATE_STANDBY` (disarmed) vs `MAV_STATE_ACTIVE` (armed).
- Use `custom_mode` for MODE_IDLE vs MODE_CONFIG.
- Forbid arming while MODE_CONFIG (USB forces config).
- If USB is connected and `DO_SET_MODE(MODE_IDLE)` arrives: reject + `STATUSTEXT`.

---

## Implementation plan (do this before new wireless transport)

### Phase 0: Repo discovery + placement

1. Identify where FC UART/USB CDC endpoints live in firmware (likely `device/rp235x` or similar).
2. Identify the existing MAVLink encode/decode plumbing and where heartbeats are currently handled (if at all).

### Phase 1: FC state machine (authoritative)

1. Add `FcState` with:
  - `usb_connected: bool`
  - `armed: bool`
  - `custom_mode: u32` (MODE_IDLE / MODE_CONFIG)
  - derived `system_status: MAV_STATE_*`
2. Add transition functions that enforce:
  - usb_connected ⇒ MODE_CONFIG forced
  - MODE_CONFIG ⇒ cannot arm
  - only allow mode changes while disarmed

### Phase 2: HEARTBEAT emission

1. Add periodic task emitting `HEARTBEAT` at `FC_STATE_HEARTBEAT_HZ`.
2. Encode:
  - `base_mode`: includes `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` and armed flag
  - `custom_mode`: MODE_IDLE/MODE_CONFIG
  - `system_status`: STANDBY/ACTIVE
3. Emit `STATUSTEXT` on transitions (gated by a log parameter if needed).

### Phase 3: Command handling (standard)

1. Handle `COMMAND_LONG`:
  - `MAV_CMD_COMPONENT_ARM_DISARM`
  - `MAV_CMD_DO_SET_MODE`
2. Always reply `COMMAND_ACK`.
3. If rejected due to policy: also emit `STATUSTEXT`.

### Phase 4: Radio + GS forwarding semantics

1. Radio forwards FC heartbeats to GS.
2. GS forwards FC heartbeats to USB CDC.
3. Radio/GS optionally emit their own component heartbeats (separate sysid/compid) for debugging.

### Phase 5: Parameter wiring

1. Add parameters to FC ParamRegistry.
2. Ensure GCS can read/list/set them via standard MAVLink parameter protocol.
3. Use parameters to drive:
  - heartbeat rate
  - telemetry scheduling targets by state
  - adaptivity thresholds
