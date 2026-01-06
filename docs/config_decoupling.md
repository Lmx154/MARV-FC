# LoRa Config Layers and Dependencies (Post-Decoupling)

This document reflects the current state after splitting the old monoconfig into
separate RF and MAC configs. It answers:
- Is there still a single "mega config"?
- What is the lowest-level configuration?
- What values are derived from what?
- Which RF fields affect MAC scheduling?
- What are the next steps?

## Short answer: is there one mega config?

No. The link now uses two independent configs:
- `RfConfig` (RF/PHY settings)
- `MacConfig` (TDMA schedule, payload sizes, RX timeout policy)

Both are still selected at build time via `link_config::ACTIVE` (which points to
an explicit RF+MAC pair in `link_config.rs`), but they can now be paired
independently.

## Config objects (lowest level first)

### 1) RfConfig (RF/PHY settings)

File: `common/src/coms/transport/lora/rf_config.rs`

Purpose:
- Alias of `LoRaConfig` (`type RfConfig = LoRaConfig`).
- Defines modulation, packet format, and radio hardware settings.
- Used directly by the SX1262 driver at init.
- Used for symbol time and time-on-air calculations.
- Preset RF profiles live in `common/src/coms/transport/lora/rf_presets.rs`.

### 2) MacConfig (TDMA schedule + payload + timeout policy)

File: `common/src/coms/transport/lora/mac_config.rs`

Purpose:
- Defines tick rate, slot ratio, guard timings, payload sizes, and RX timeout
  policy.
- Provides derived calculations that combine RF + MAC:
  - `rx_timeout_symbols(rf, mac)`
  - `slot_rx_symbols(rf, mac)`
- Preset MAC profiles live in `common/src/coms/transport/lora/mac_presets.rs`.

### 3) LinkConfig (RF + MAC pairing helper)

File: `common/src/coms/transport/lora/link_config.rs`

Purpose:
- Pairs an `RfConfig` and `MacConfig` into a single selection point.
- Provides helpers for derived values:
  - `rx_timeout_symbols()` and `slot_rx_symbols()`
  - `profile()` (builds a `LinkProfile`)
- Exposes `DRONE_*` and `ROCKET_*` presets for per-vehicle rate selection.

### 4) PhyServiceConfig (default RX timeout)

File: `common/src/coms/transport/lora/phy_service.rs`

Purpose:
- Defines default RX timeout symbols for background receive.
- Derived with `rx_timeout_symbols(rf, mac)`.

### 5) LinkProfile (MAC engine input)

File: `common/src/coms/transport/lora/link_profile.rs`

Purpose:
- Derived, MAC-only view of the link.
- Built with `LinkProfile::from_configs(rf, mac)`.
- Carries schedule values and precomputed ToA for uplink/downlink frames.

### 6) LoraTransport (packet selection)

File: `common/src/coms/transport/lora/link_transport.rs`

Purpose:
- Chooses which packet type to put in the fixed-size payload.
- Does not alter RF config or TDMA timing.

## Initialization timeline (actual code paths)

### GS path

Files: `device/gs/src/main.rs`, `device/gs/src/mac_engine.rs`

1) Select link config:
   - `link_config::ACTIVE` -> `link_cfg`
2) Initialize SX1262 with RF config:
   - `Sx1262::new(..., link_cfg.rf, ...)`
3) Derive default background RX timeout:
   - `link_cfg.rx_timeout_symbols()` -> `PhyServiceConfig`
4) Derive downlink-slot RX timeout:
   - `link_cfg.slot_rx_symbols()` -> used when arming RX on DL slots
5) Build MAC profile:
   - `link_cfg.profile()`
6) MAC engine uses LinkProfile for tick scheduling and TX fit checks.

### Radio path

Files: `device/radio/src/main.rs`, `device/radio/src/mac_engine.rs`

1) Select link config:
   - `link_config::ACTIVE` -> `link_cfg`
2) Initialize SX1262 with RF config:
   - `Sx1262::new(..., link_cfg.rf, ...)`
3) Derive default background RX timeout:
   - `link_cfg.rx_timeout_symbols()` -> `PhyServiceConfig`
4) Build MAC profile:
   - `link_cfg.profile()`
5) MAC engine uses LinkProfile for tick scheduling and TX fit checks.

## Derived values and cross-layer dependencies

### A) Symbol time (from RfConfig)

File: `common/src/coms/transport/lora/mac_config.rs`

```
symbol_us = (2^sf) / bw_hz
```

Used by:
- `rx_timeout_symbols(rf, mac)`
- `slot_rx_symbols(rf, mac)`

### B) Time on air (from RfConfig + payload length)

File: `common/src/coms/transport/lora/rf_config.rs`

```
toa_us = rf.toa_us(total_len_bytes)
total_len_bytes = HEADER_LEN + payload_len
```

Used by:
- `LinkProfile::from_configs` to compute `uplink_toa_us` / `downlink_toa_us`
- MAC TX fit checks
- Auto RX timeout calculation

### C) Auto RX timeout (from RfConfig + MacConfig)

File: `common/src/coms/transport/lora/mac_config.rs`

Inputs:
- RfConfig: symbol time, ToA
- MacConfig: `tick_hz`, `tx_guard_us`, `dl_tx_offset_us`, `rx_ready_guard_us`,
  `uplink_payload_len`, `downlink_payload_len`

Output:
- `rx_timeout_symbols` when `mac.rx_timeout_auto == true`

### D) Slot RX symbols (from RfConfig + MacConfig)

Inputs:
- RfConfig: symbol time
- MacConfig: `tick_hz`, `tx_guard_us`

Output:
- `slot_rx_symbols` used to arm RX for a full slot window (GS)

## Which RfConfig fields affect MAC scheduling?

MAC scheduling uses RfConfig only through:
- `lora_symbol_time_us()` (symbol time)
- `LoRaConfig::toa_us()` (time on air)

Therefore, the RfConfig fields that affect scheduling are:
- `sf` (spreading factor)
- `bw` (bandwidth code)
- `cr` (coding rate)
- `preamble_len`
- `explicit_header`
- `crc_on`

These fields do NOT affect scheduling timing in this code:
- `freq_hz`
- `tcxo_enable`, `tcxo_voltage`, `tcxo_delay_ms`
- `use_dcdc`
- `tx_power`
- `sync_word`
- `invert_iq`
- `rf_switch_swap`
- `ldro` (set on the radio, but not used in ToA or symbol time here)

## Which MAC values depend on RfConfig?

There is no direct dependency where MAC values are computed from RfConfig.
Instead, the relationship is constraint-based:

The MAC schedule must be chosen so that:
- `ToA(uplink_frame) <= tick_period_us - tx_guard_us`
- `ToA(downlink_frame)` fits after `dl_tx_offset_us` and before tick end

If these constraints are violated:
- TX will be refused by the MAC (`fits_tx` check fails).
- Auto RX timeouts may be too short or too long.

So the dependency is one-way:
- RfConfig (plus payload length) constrains valid tick rates and slot timing.
- MacConfig does not change RfConfig.

## Shareability rule (rocket vs drone)

When evaluating what can be shared across vehicles, use this rule:
- A value is *shareable* only if it is intrinsic to the RF/PHY hardware and is
  not derived from vehicle-specific packets, lanes, or traffic requirements.
- If a value must be derived from a rocket/drone-specific parameter (even if the
  resulting number matches on both vehicles), it is *not* shareable.

Implications:
- RF/PHY presets are shareable as long as they are not computed from vehicle
  traffic needs.
- All MAC schedule/payload values (and any derived timing) are per-vehicle
  because they are driven by packet sizes, lane priorities, and rate targets.

## What this means for decoupling (current state)

The split is now implemented:
- RF config lives in `rf_config.rs` and is passed only to the SX1262 driver.
- MAC schedule and payload sizing live in `mac_config.rs` and are passed only to
  the MAC engine.
- RX timeouts are derived from both and passed to `PhyServiceConfig`.

What remains coupled today:
- `link_config::ACTIVE` is still selected at build time.
- Pairing and validation between an RF preset and a MAC preset is manual.

## Next steps

1) Add a validation helper (startup check) that asserts the schedule constraints
   for a given RF/MAC pair so unsafe combos fail fast.
2) Wire runtime selection so GS can choose a pair (or separate RF + MAC configs)
   and command the radio to switch at a known tick boundary.
3) Keep RF presets shareable only when they are not derived from vehicle
   packetization; otherwise they are per-vehicle.
4) Treat all MAC presets as per-vehicle (rocket vs drone), even if the values
   happen to match, to avoid coupling through packet-specific derivations.
