# src

Part of [MARV V2 firmware](../README.md)

The firmware binary (package `marv-v2-firmware`). `main.rs` starts both cores; each other file is one device, link or service.
Hardware-free logic is in the crates (see the [README](../README.md)), not here.

| File | What it is | Node |
|---|---|---|
| `main.rs` | Entry, core 1 spawn, USB MAVLink node, supervisor, start-up checks | [architecture](../docs/architecture.md) |
| `ram.rs` | Boot copy of the flight core's code to SRAM | [architecture](../docs/architecture.md) |
| `psram.rs` | PSRAM start, canary, log-ring storage | [architecture](../docs/architecture.md) |
| `flight.rs` | Flight step on the board (core 1), live-parameter hand-off | [flight-core](../docs/flight-core.md) |
| `flight_latency.rs` | Longest-poll figure of flight tasks | [health](../docs/health.md) |
| `imu1.rs`, `imu2.rs` | IMU1 (ICM-45686, SPI0) and IMU2 (LSM6DSK320X, SPI1) adapters on core 1 | [device-layer](../docs/device-layer.md) |
| `baro.rs`, `mag.rs`, `gnss.rs`, `batt.rs` | Barometer, compass, GNSS, battery adapters on core 0 | [device-layer](../docs/device-layer.md) |
| `batt_fs.rs` | Low-battery monitor | [flight-modes](../docs/flight-modes.md) |
| `device.rs` | Device rows, generic device loop over the health machine | [device-layer](../docs/device-layer.md) |
| `i2cbus.rs`, `busclear.rs` | I2C deadline wrapper and bus clear | [device-layer](../docs/device-layer.md) |
| `esc.rs` | Four ESCs: DShot300 on PIO2, eRPM replies | [esc](../docs/esc.md) |
| `rc.rs` | ELRS receiver port, RC row, radio MAVLink node | [radio](../docs/radio.md) |
| `station.rs` | Station flight commands, mode and heartbeat bridge | [flight-modes](../docs/flight-modes.md) |
| `mission.rs` | Waypoint list service for both links | [missions](../docs/missions.md) |
| `actions.rs` | Commands and the long actions they start | [ground-station](../docs/ground-station.md) |
| `calibration.rs` | Gyro, accelerometer calibration; offsets to the flight core | [calibration](../docs/calibration.md) |
| `store.rs`, `settings.rs` | Parameter store; flash records | [parameters](../docs/parameters.md) |
| `logger.rs`, `sd.rs`, `logdl.rs` | Log collector, SD writer, log download | [logging](../docs/logging.md) |
| `led.rs` | Status LED task | [health](../docs/health.md) |
| `watchdog.rs` | Hardware watchdog and its monitor | [health](../docs/health.md) |
| `inject.rs`, `imu_inject.rs` | Fault injection wrappers (features) | [build-and-test](../docs/build-and-test.md) |
| `xip_park.rs` | Board check (feature) | [build-and-test](../docs/build-and-test.md) |
| `bin/eskf_bench.rs` | ESKF timing bench binary | [build-and-test](../docs/build-and-test.md) |

Back to the [README](../README.md).
