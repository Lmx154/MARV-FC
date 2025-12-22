# MARV-FC Wireless Link Spec (MAVLink-first, ELRS-like MAC)

Date: 2025-12-21 (edited)

## Scope (link-only)

This document defines **only** the wireless link behavior for GS ⇄ Radio ⇄ FC over SX1262 LoRa:

- Deterministic low-jitter transmission scheduling (ELRS-like feel)
- Traffic classes + queueing + drop policies
- Encapsulation/packetization over LoRa MTU
- Transactional radio reconfiguration (two-phase apply)
- Link-quality reporting upward
- Link-level classification rules for “FAST” traffic (message-ID based)

**Out of scope**:
- Telemetry message selection/rates and meaning (application policy)
- FC parameter ownership/namespaces/storage (system architecture)
- Vehicle arming logic and safety policy (except as an input gate)
- MAVLink dialect semantics beyond what is required for link classification/size constraints

---

## 1) Link responsibilities

The wireless link is responsible for:

1. Moving packets over SX1262 LoRa PHY (TX/RX, RSSI/SNR, timestamps).
2. Enforcing a **deterministic tick-based MAC** (fixed cadence) to minimize jitter.
3. Providing **FAST newest-wins** behavior for high-rate setpoint-style traffic.
4. Prioritizing **RELIABLE operations traffic** (acks/commands/params) without allowing it to steal FAST slots.
5. Supporting safe radio reconfiguration via **staged apply**.
6. Exporting **link metrics** upward: RSSI/SNR, PER estimates, queue depths, drop counters.

The link **does not infer** armed/disarmed. It consumes explicit state as an input.

---

## 2) Hard constraints

- **SX1262 max payload**: 255 bytes per LoRa packet.
- **LoRaLink payload MTU**: 240 bytes (4-byte LoRaLink header reserved).
- **MAC header**: 4 bytes (sync + flags).
- **Inner payload MTU**: 236 bytes (240 - 4).
- **No fragmentation initially**.
  - Any frame (or aggregate) that would exceed MTU is **dropped** and counted.
- MAVLink signing (optional) reduces usable payload budget; keep off unless required.

---

## 3) Layering (link view)

1. **PHY**: SX1262 packet TX/RX + RSSI/SNR.
2. **MAC**: tick scheduler + slot ratio + QoS queues (ELRS-like behavior).
3. **Encapsulation**: MAVLink frames and/or link-aggregate frames carried in LoRa packets.
4. **Protocol semantics (above link)**: MAVLink `COMMAND_ACK`, params, etc.

---

## 4) Deterministic tick MAC (ELRS-like behavior)

### 4.1 Tick cadence

The link runs a fixed **TX tick** (e.g., 50–200 Hz depending on modem settings and range needs).

At each tick, the link transmits **at most one** LoRa packet (or remains silent).

- Parameter: `LINK_TICK_HZ` (u16)

**Storage vs application**: the FC is the durable store for link configuration (parameters).

The GS is responsible for **pulling** the latest parameter values from the FC (via MAVLink `PARAM_*`),
then **applying** them locally and distributing the runtime MAC configuration to the Radio.

**Robustness requirement** (practical): the non-master node should inhibit TX unless it has
received a recent master sync packet (sync-age guard) to avoid transmitting out of phase.

### 4.2 Slot ratio (uplink/downlink schedule)

Define a repeating slot schedule (directional airtime reservation):

Examples:
- Control-centric: `U U U D` (3 uplink ticks, 1 downlink tick)
- Balanced: `U D U D`
- Bench/telemetry-centric: `U D D D`

- Parameter: `LINK_SLOT_MD` (enum)

Encoding (stable):
- `0` = `UuuD`
- `1` = `UdUd`
- `2` = `Uddd`

**Invariant**: A slot schedule must prevent downlink bursts (telemetry/params) from stealing uplink airtime and increasing jitter.

### 4.3 Traffic classes (QoS)

Maintain three queues per direction:

1. **FAST** (newest-wins, no retries)
2. **RELIABLE** (bounded, prioritized; retries occur above link)
3. **NORMAL** (best-effort, bulk/telemetry)

Recommended queue constraints:
- FAST: effectively size 1 (keep newest only)
- RELIABLE: small bounded FIFO (drop oldest on overflow)
- NORMAL: bounded FIFO (drop oldest on overflow)

### 4.4 Scheduler rules (per tick)

If current slot is Uplink:
1. Send newest FAST-uplink item if present (drop older)
2. Else send one RELIABLE-uplink item (FIFO)
3. Else send one NORMAL-uplink item (FIFO)

If current slot is Downlink:
1. Send newest FAST-downlink item if present
2. Else send one RELIABLE-downlink item (FIFO)
3. Else send one NORMAL-downlink item (FIFO)

**Goal**: FAST never waits behind RELIABLE/NORMAL, and slotting prevents cross-direction contention from creating jitter.

---

## 5) FAST payload rules (what makes it feel “RC-like”)

### 5.1 Deterministic byte budget

To achieve low jitter, FAST payloads must be:
- **Fixed-size**
- **Small enough for the tick airtime budget**
- **Deterministic airtime**

- Parameter: `LINK_FAST_MB` (u16)

Notes:
- This is a *hard* byte budget for a single FAST MAVLink2 frame on the inner MTU.
- Implementations should clamp to `INNER_MTU` and demote oversized FAST frames to RELIABLE.

Practical note: when aggregation is enabled, a FAST frame that fits within `INNER_MTU` may still fail to fit inside an aggregate packet due to aggregate wrapper overhead. In that case, the MAC may transmit FAST as a legacy single-frame inner payload for that tick (see §6.2).

---

## 5.4 Runtime config distribution (MAVLink carrier)

To avoid requiring a custom dialect for “link settings”, the GS distributes `LinkMacConfig` at runtime
to the Radio using a standard MAVLink2 `COMMAND_LONG` frame.

**On-wire carrier** (see `common/src/protocol/mavlink/link_mac_config.rs`):

- Message: `COMMAND_LONG` (msgid `76`)
- `command`: `MAV_CMD_USER_1` (31000)
- `target_system` / `target_component`: may be `0/0` (broadcast) or explicitly addressed.
- Params mapping:
  - `param1`: `tick_hz` (float, intended integer Hz)
  - `param2`: `slot_mode` code (0/1/2 per §4.2)
  - `param3`: `fast_max_bytes` (float, intended integer)
  - `param4`: magic discriminator (`424242.0`)
  - `param5`: encoding version (`1.0`)

**Receiver rules** (GS + Radio):

- Apply only when `target_system` matches our sysid (or is `0`) AND `target_component` matches our compid (or is `0`).
- Accept only when `param4` magic and `param5` version match expected constants.
- On apply, send `COMMAND_ACK` for `MAV_CMD_USER_1` back to the sender:
  - `ACCEPTED` when applied
  - `UNSUPPORTED` when command id matches but magic/version mismatch

**Recommendation**: Prefer a compact fixed payload (e.g., axes + flags + seq) for the highest-rate FAST stream.

### 5.2 Newest-wins semantics

FAST items represent *current setpoints/intent*:
- no retransmissions
- stale FAST items are discarded when newer arrives
- receiver uses `seq` to detect drops/replays

### 5.3 Tick alignment

FAST producers/bridges should align generation/forwarding to `LINK_TICK_HZ` so each transmitted FAST packet contains the freshest sample.

---

## 6) Encapsulation & packetization

### 6.1 Minimum viable: one MAVLink frame per LoRa packet

Allowed as baseline:
- LoRa packet carries exactly one MAVLink2 frame
- If MAVLink frame would exceed MTU: drop & count

Note: when the MAC header is enabled, the MAVLink frame must fit within the **inner payload MTU** (236 bytes).

This is simple, but wastes overhead.

### 6.2 Suggested upgrade: Link Aggregate Packet (aggregation, not fragmentation)

To reduce overhead and improve determinism, allow a link packet format that can carry:

- Optional FAST section (at most one)
- Plus 0–N MAVLink frames (length-delimited) up to MTU

This enables:
- FAST + small acknowledgements/status in the same LoRa packet
- fewer packets => less airtime overhead => tighter jitter bounds

Aggregate packet format (example):
- `hdr: {type, link_seq, flags}`
- `sections: [FAST?][MAV(len, bytes)]...[MAV(len, bytes)]`

Constraints:
- Total bytes ≤ MTU
- Never split a MAVLink frame across packets (no fragmentation)

Robustness requirement (no-stall): if a single MAVLink frame (FAST, RELIABLE, or NORMAL) fits within `INNER_MTU` but does not fit in the aggregate packet due to wrapper overhead, the transmitter may send it as a legacy single-frame inner payload for that tick (still MAC-wrapped) instead of stalling.

---

## 7) Reliability model (link-only)

### 7.1 No generic stop-and-wait ARQ for all packets

The link must **not** ACK/retry every LoRa datagram.

### 7.2 Reliability by class

- FAST: newest-wins, no retries, seq-based loss accounting
- RELIABLE: prioritized delivery + bounded buffering; retries happen above link (MAVLink/app)
- NORMAL: best-effort

### 7.3 Required counters/telemetry from the link

Expose:
- per-class enqueue/dequeue counts
- per-class drops (overflow)
- MTU drops
- RX/TX packet counts
- missed FAST seq counts

---

## 8) Transactional radio reconfiguration (safe staged apply)

Radio config changes that affect demodulation (freq/SF/BW/CR/sync word, etc.) must use a two-phase apply:

- `PROPOSE(txn_id, new_cfg, apply_after_ms)`
- `COMMIT(txn_id)`
- optional `ABORT(txn_id)`

Rules:
- `apply_after_ms` is relative to receipt time.
- For phase stability under jitter, implementations should **tick-align** application:
  - Convert `apply_after_ms` to an integer tick offset: `apply_after_ticks = ceil(apply_after_ms / tick_interval_ms)`.
  - Schedule the actual RF apply on the MAC tick boundary `target_tick = rx_master_tick_seq + apply_after_ticks`.
  - If COMMIT arrives after `target_tick`, apply on the next tick boundary (apply-ASAP but still tick-aligned).
- If COMMIT not received within a guard timeout, discard staged config.
- New PROPOSE supersedes any previous staged config (unless locked by commit-in-progress).

This is a link primitive; it may be triggered by MAVLink commands above the link, but the **state machine behavior** is link-defined.

### 8.1 MAVLink carrier (no-dialect scheme)

The repo carries staged RF reconfiguration using a standard MAVLink2 `COMMAND_LONG` frame.

**On-wire carrier** (see `common/src/protocol/mavlink/rf_reconfig.rs`):

- Message: `COMMAND_LONG` (msgid `76`)
- `command`: `MAV_CMD_USER_2` (31001)
- `target_system` / `target_component`: may be `0/0` (broadcast) or explicitly addressed.

**Params mapping**

- `param1`: op code
  - `1` = PROPOSE
  - `2` = COMMIT
  - `3` = ABORT
- `param2`: `txn_id` (float carrier, intended integer)
- `param3`: `apply_after_ms` (float carrier, intended integer; PROPOSE only)
- `param4`: packed0 (float carrier, intended u32 <= 2^24)
  - bits 0..19: `freq_khz`
  - bits 20..23: `sf`
- `param5`: packed1 (float carrier, intended u32 <= 2^24)
  - bits 0..3: `bw_code` (0=125k, 1=250k, 2=500k)
  - bits 4..7: `cr_code` (0=4/5, 1=4/6, 2=4/7, 3=4/8)
  - bits 8..23: `sync_word` (u16)
- `param6`: magic discriminator (`515151.0`)
- `param7`: encoding version (`1.0`)

**Receiver rules** (Radio):

- Accept only when magic/version match expected constants.
- Apply only after an explicit `COMMIT(txn_id)`.
- On accept/reject, send `COMMAND_ACK` for `MAV_CMD_USER_2` back to the sender.

**Coordinator behavior** (GS):

- GS acts as the *coordinator* for staged RF changes: it accepts `COMMAND_LONG` from the host, forwards the corresponding rf-reconfig command to the Radio over LoRa, and mirrors the same staging state locally.
- To avoid self-bricking the link, GS delays its own local apply until it has observed the Radio’s `COMMAND_ACK` for `MAV_CMD_USER_2` (i.e. Radio accepted the operation).
- Minimal observability: GS and Radio emit `STATUSTEXT` on stage/commit/abort/apply/deny/timeout to help debug timing and LAS lockouts in the field.

---

## 9) Authority state input (consume; never infer)

The link consumes an explicit **Link Authority State (LAS)** input:

LAS fields:
- `state`: `{DISARMED, ARMED, UNKNOWN}`
- `fresh_ms`: time since last valid LAS update

Parameters:
- `LAS_MAX_AGE_MS` (u16)

Rules:
- Dangerous RF changes (PROPOSE/COMMIT that alter demodulation) are allowed only if:
  - `state == DISARMED` **and**
  - `fresh_ms < LAS_MAX_AGE_MS`
- If `UNKNOWN` or stale: **deny RF changes** (lockout).
- Bench override must be **explicit** (implementation-defined), never inferred.

### 9.1 MAVLink carrier (dialect message)

- Message: `MARV_LINK_AUTH` (custom dialect)
- Fields: `state`, `max_age_ms`, `target_system`, `target_component`

Implementation note (repo): the Radio updates LAS from the explicit `MARV_LINK_AUTH` message on the UART side, and enforces the lockout when handling rf-reconfig commands.

---

## 10) Link-quality reporting (required)

The link must publish a compact status report upward (via MAVLink `RADIO_STATUS` or a custom status message) including:

- RSSI/SNR (last + running average)
- PER estimate (or proxy via missed seq / rx failure counts)
- per-class queue depths (FAST/RELIABLE/NORMAL)
- per-class drop counters
- current `LINK_TICK_HZ` and `LINK_SLOT_MD`
- last reconfig txn status (idle/staged/commit pending)

This enables adaptation in higher layers without baking application policy into the link.

### 10.1 MAVLink `RADIO_STATUS` packing (no-dialect scheme)

To avoid introducing a custom MAVLink dialect message, we pack additional MAC metrics into the
standard MAVLink `RADIO_STATUS` message fields. This mapping is **stable** and must remain
compatible across GS/Radio/FC components.

**Human-meaningful fields**

- `rssi`: last received RSSI (dBm), encoded as MAVLink `int8_t` (stored in code as raw `u8`).
- `noise`: **repurposed** to carry last received SNR (dB), because `RADIO_STATUS` has no SNR field.

**Packed fields (MARV-FC scheme)**

- `remrssi` (u8): queue depths + slot mode
  - bits 0..2: RELIABLE queue depth (0..7, saturating)
  - bits 3..5: NORMAL queue depth (0..7, saturating)
  - bits 6..7: `LINK_SLOT_MD` (0=`UuuD`, 1=`UdUd`, 2=`Uddd`)
- `txbuf` (u8): packed low-resolution always-on deltas
  - bits 0..3: total MAC outbound queue fill (0..15, derived from FAST+RELIABLE+NORMAL occupancy)
  - bits 4..7: delta of `missed_master_ticks` since last report (0..15, saturating)
- `remnoise` (u8): packed low-resolution always-on deltas
  - bits 0..3: delta of `fast_replaced` since last report (0..15, saturating)
  - bits 4..7: delta of `dropped_too_large` (inner MTU drops) since last report (0..15, saturating)
- `rxerrors` (u16): tick config + RF txn status
  - low byte: `LINK_TICK_HZ` (approx = `1000 / tick_interval_ms`, 0..255)
  - high byte:
    - bit 7: FAST present (1 if FAST queue occupied)
    - bits 4..6: `rf_last_result` (0=None, 1=Ok, 2=Denied, 3=Failed, 4=Timeout)
    - bits 0..3: `rf_txn_state` (0=Idle, 1=Staged, 2=Committed(waiting), 3=ApplyPending)
- `fixed` (u16): per-lane FIFO drops
  - high byte: delta of `dropped_reliable_overflow` since last report (0..255)
  - low byte:  delta of `dropped_normal_overflow` since last report (0..255)

**Notes**

- Deltas are reported over the status period (~1 Hz) to avoid rapid saturation of counters.
- Saturation means "at least" the max representable value.
- `missed_master_ticks` is a PER proxy based on gaps in received MAC `tick_seq` from the master.

### 10.2 MARV_LINK_METRICS (dialect message)

The link also emits a dedicated dialect message for full, machine-parsable counters:

- Message: `MARV_LINK_METRICS` (custom dialect)
- Rate: ~1 Hz on NORMAL
- Producer: Radio (forwarded unchanged by GS)
- Fields: full per-class enqueue/tx/drop counters, RX/TX packet counts, MTU drops, fast replaced,
  missed ticks, missed FAST seq, tx_inhibit_desync, and RF txn state/result (+ apply tick).

---

## 11) Summary: what makes this ELRS-like

This link spec includes the core ELRS-style ingredients:

1. **Fixed tick** transmission cadence (low jitter)
2. **Directional slot ratio** reserving airtime for control
3. **Strict priority** with newest-wins FAST queue
4. **Fixed-size FAST payloads with explicit byte budgets**
5. Optional **aggregation** to reduce overhead and jitter
6. Safe RF changes gated by **explicit** authority state input

---

## 12) FAST classification via MAVLink dialect (repo interface contract)

To avoid generic `ENCAPSULATED_DATA` for FAST traffic and to make FAST classification unambiguous, the repo defines a dedicated MAVLink dialect:

- Dialect file: `mavlink/marv_fc.xml`
- FAST message IDs:
  - `FAST_RC` (msgid `42000`)
  - `FAST_ATTITUDE_RATES` (msgid `42001`)

**Link contract for FAST dialect messages**:

1. The link/MAC treats these msgids as **FAST**:
   - newest-wins
   - best-effort
   - no retries
2. These messages must be **fixed-size** and must satisfy `encoded_len ≤ LINK_FAST_MB`.
3. Each FAST message includes a `seq` field to support loss accounting / replay rejection.
4. Any semantics beyond the above (field meanings, cadences, how they are produced/consumed) is defined outside this link spec (dialect docs / application policy).

Rationale (link-relevant):
- Fixed-size + bounded bytes => predictable airtime per tick.
- Scaled integers (vs floats) can reduce payload size and encode/decode cost.
- Explicit FAST msgids allow the MAC to classify traffic without heuristics.

Implementation note (temporary): while the dialect is being integrated end-to-end, the repo may use a compact standard message (e.g., `RAW_IMU`) as a transitional FAST telemetry carrier, since `ENCAPSULATED_DATA` cannot fit within the 236-byte inner MTU.
