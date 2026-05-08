# Simulator (FC SITL Runtime)

The `simulator` crate is currently the host-side FC SITL runtime.

Its job is narrow:

- receive simulator truth over UDP using MAVLink 2 `common`
- decode frames with the shared MAVLink parser in `common`
- publish accepted samples into shared acquisition channels in `common`
- write shared sensor snapshot CSV logs for replay auditability

The runtime is step-driven. It does not synthesize control, estimation, or other firmware behavior from host wall-clock timers.

## Run

From repo root:

```bash
cargo run -p simulator
```

Default endpoints:

- RX bind: `127.0.0.1:14560`
- TX actuator target: `127.0.0.1:14561`

## Environment variables

- `FC_SITL_BIND`
  - UDP socket the FC SITL listens on for simulator MAVLink traffic
  - Default: `127.0.0.1:14560`
- `FC_SITL_ACTUATOR_ADDR`
  - UDP target for outbound actuator MAVLink traffic
  - Currently reserved for the standalone TX helper; the main runtime does not transmit actuator commands yet
  - Default: `127.0.0.1:14561`
- `FC_SITL_TICK_TIMEOUT_MS`
  - Host-side liveness timeout for `SYSTEM_TIME`
  - Default: `500`
- `FC_SITL_LOG_DIR`
  - Directory for CSV replay logs
  - Default: `simulator/out`
- `FC_SITL_SYSTEM_ID`
  - Outbound MAVLink system id
  - Default: `42`
- `FC_SITL_COMPONENT_ID`
  - Outbound MAVLink component id
  - Default: `1`

Example:

```bash
FC_SITL_BIND=127.0.0.1:14560 \
FC_SITL_ACTUATOR_ADDR=127.0.0.1:14561 \
cargo run -p simulator
```

## Runtime path

Current active path:

1. bind UDP RX
2. decode MAVLink 2 `common` frames via `common::protocol::mavlink`
3. bridge `SYSTEM_TIME`, `HIL_SENSOR`, and `HIL_GPS` into shared sample channels
4. drain those shared channels into `common::services::logging::SensorSnapshotLogger`
5. append one portable sensor snapshot row on each accepted sim tick

No estimator, controller, mixer, or actuator solve is executed by the runtime today.

## Input contract

Inbound messages currently consumed:

- `SYSTEM_TIME`
- `HIL_SENSOR`
- `HIL_GPS`

Accepted `SYSTEM_TIME.time_boot_ms` values define the replay tick.

- missing `HIL_SENSOR` is allowed on a tick
- missing `HIL_GPS` is allowed on a tick
- stale or duplicate `SYSTEM_TIME` ticks are ignored
- GPS fixes with `fix_type < 2` are ignored

Published shared sample types:

- `TimeSample`
- `ImuSampleStamped`
- `BarometerSampleStamped`
- `GpsFixSampleStamped`

## Output contract

Outbound support present in the crate but not wired into the runtime:

- MAVLink `ACTUATOR_CONTROL_TARGET`
- encoder lives in `common::protocol::mavlink`
- UDP send helper lives in `simulator/src/tx_actuator.rs`
- the main runtime does not emit actuator traffic yet

## Logging

The host logger uses the shared `LoggerEngine` + `SensorSnapshotLogger` path.

Files are created with the shared flight-log naming convention:

- `simulator/out/FLGT0001.CSV`
- `simulator/out/FLGT0002.CSV`

The CSV schema is the portable sensor snapshot schema from `common`, including:

- `log_us`
- sink state
- per-sensor freshness/state markers
- sample timestamps
- IMU/barometer/GPS sample fields

## ICD

The frozen FC SITL interface contract lives in:

- [`docs/FC_SITL_ICD.md`](./FC_SITL_ICD.md)
