# FC SITL ICD

Version: `v2`

This document freezes the FC SITL firmware contract currently implemented by the `simulator` crate.

## 1. MAVLink baseline

- Primary protocol: MAVLink 2
- Dialect: `common`
- Transport: UDP datagrams

Implementation compatibility notes:

- MAVLink 2 is the intended simulator -> FC contract
- the current FC SITL parser also accepts MAVLink 1 framing for compatibility/debugging
- MAVLink 2 payload truncation is accepted when trailing bytes are omitted; omitted tail bytes are treated as zero during decode

## 2. Inbound contract

Default simulator -> FC endpoint:

- `127.0.0.1:14560`

Accepted inbound messages:

1. `SYSTEM_TIME`
2. `HIL_SENSOR`
3. `HIL_GPS`

Inbound parser behavior:

- one replay tick is accepted only on a new `SYSTEM_TIME.time_boot_ms`
- duplicate or out-of-order `SYSTEM_TIME` ticks are ignored
- `HIL_SENSOR` may be absent on a tick
- `HIL_GPS` may be absent on a tick
- GPS fixes with `fix_type < 2` are ignored
- unsupported MAVLink messages are ignored
- valid MAVLink frames may arrive back-to-back in the receive buffer and are consumed sequentially

Field usage:

- `SYSTEM_TIME.time_boot_ms`
  - sim tick anchor
- `HIL_SENSOR.xacc/yacc/zacc`
  - IMU acceleration in `m/s^2`
- `HIL_SENSOR.xgyro/ygyro/zgyro`
  - IMU angular rate in `rad/s`
- `HIL_SENSOR.abs_pressure`
  - barometer pressure in `hPa`
- `HIL_SENSOR.temperature`
  - barometer temperature in `degC`
- `HIL_SENSOR.fields_updated`
  - freshness gating for IMU/barometer cache updates
- `HIL_GPS.lat/lon`
  - `degE7`
- `HIL_GPS.alt`
  - `mm`
- `HIL_GPS.vn/ve/vd`
  - `cm/s`
- `HIL_GPS.satellites_visible`
  - propagated into GPS sample state

Current freshness masks:

- IMU fresh when any of bits `0x003F` are set
- barometer fresh when any of bits `0x1A00` are set

## 3. Replay tick semantics

One FC iteration is defined as:

1. accept one strictly newer `SYSTEM_TIME.time_boot_ms`
2. publish any accepted truth into shared `common` sample channels
3. append one shared sensor snapshot CSV row using the latest published samples

No estimator, controller, or mixer is implemented in the runtime at this stage.
The replay tick does not advance from host wall-clock timers.

## 4. Outbound contract

Default FC -> simulator endpoint:

- `127.0.0.1:14561`

Reserved outbound message:

- `ACTUATOR_CONTROL_TARGET`

Current status:

- encoder is implemented in `common::protocol::mavlink`
- UDP transmit helper is implemented in `simulator/src/tx_actuator.rs`
- the main SITL runtime does not emit actuator traffic yet

When TX is enabled later, the frozen field contract is:

- `time_usec`
  - monotonic SITL loop time derived from `time_boot_ms`
- `controls[0..7]`
  - normalized actuator bus
- `group_mlx`
  - vehicle/profile-specific actuator group identifier

Binary actuator convention:

- `value >= 0.5` => ON / DEPLOY
- `value < 0.5` => OFF / RETRACT

Rocket channel mapping:

1. `ch0` TVC pitch
2. `ch1` TVC yaw
3. `ch2` fin set 1
4. `ch3` fin set 2
5. `ch4` airbrake
6. `ch5` ignition / engine enable
7. `ch6` reserved
8. `ch7` reserved

Drone channel mapping:

1. `ch0` roll
2. `ch1` pitch
3. `ch2` yaw
4. `ch3` throttle
5. `ch4..ch7` reserved

## 5. Timeout and disarm behavior

Host-side liveness timeout:

- default `500 ms`
- configurable via `FC_SITL_TICK_TIMEOUT_MS`

Timeout action:

1. emit a runtime warning to stderr
2. keep waiting for the next valid `SYSTEM_TIME`

No active disarm or zero-output behavior is implemented until TX is wired into the runtime.

## 6. Logging contract

Per accepted logging interval, append one row to the current flight log:

- `simulator/out/FLGT0001.CSV`
- subsequent runs increment the shared `FLGT####.CSV` sequence

Logged fields include:

- `log_us`
- sink state
- IMU/barometer/GPS freshness flags
- per-sensor sample timestamps
- sensor sample payloads formatted by `common::services::logging::SensorSnapshotLogger`
