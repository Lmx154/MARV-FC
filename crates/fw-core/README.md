# fw-core

Part of [MARV V2 firmware](../../README.md)

Pure `no_std` logic with no hardware dependency, plus the lock-free cells that cross between the two cores. Everything is tested
on the host (`cargo host-test -p fw-core`); the firmware binary uses every module. Generic logic lives here, device-specific
logic in [fw-settings](../fw-settings/README.md).

| Module | What it is | Node |
|---|---|---|
| `xcore` | `Seqlock`, `TripleBuffer`, `Kick`, `LogRing`: cross-core cells (the only `unsafe`) | [architecture](../../docs/architecture.md) |
| `health` | Per-device health state machine and recovery ladder | [device-layer](../../docs/device-layer.md) |
| `i2c` | I2C deadline rule and bus-after-timeout rule | [device-layer](../../docs/device-layer.md) |
| `sample` | Slow-sensor samples as the flight code takes them | [flight-core](../../docs/flight-core.md) |
| `rotation` | Sensor orientation matrices | [calibration](../../docs/calibration.md) |
| `calib`, `selftest` | Gyro, accelerometer, compass calibration; self-test results | [calibration](../../docs/calibration.md) |
| `params`, `record` | Parameter store; flash settings record | [parameters](../../docs/parameters.md) |
| `handshake`, `arming` | Arming versus saving; pre-arm reasons | [flight-modes](../../docs/flight-modes.md) |
| `rc`, `relay` | Radio row, stick report rules; flight log relay | [radio](../../docs/radio.md) |
| `flight_cmd` | The station's five flight commands and the hand-off slot | [flight-modes](../../docs/flight-modes.md) |
| `mission` | Waypoint list service (upload protocol, limits) | [missions](../../docs/missions.md) |
| `battery` | Low-battery state machine | [flight-modes](../../docs/flight-modes.md) |
| `admission` | MAVLink command classification and the action book | [ground-station](../../docs/ground-station.md) |
| `sys_status`, `text_ring` | HEARTBEAT/SYS_STATUS masks; STATUSTEXT ring | [health](../../docs/health.md) |
| `esc` | ESC telemetry messages and the ESC row | [esc](../../docs/esc.md) |
| `dflog` (+ `dflog/walk`) | DataFlash `.BIN` encoders, header, reader | [logging](../../docs/logging.md) |
| `logfile`, `mbr`, `extring` | Log naming and clean-up; MBR check; ring over PSRAM | [logging](../../docs/logging.md) |
| `led` | Status LED display function | [health](../../docs/health.md) |
| `watchdog`, `scratch` | Watchdog feed rule and boot report; scratch record | [health](../../docs/health.md) |

`examples/dflog_sample.rs` writes a small sample log for `mavlogdump.py`. Back to the [README](../../README.md).
