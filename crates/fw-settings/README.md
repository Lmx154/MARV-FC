# fw-settings

Part of [MARV V2 firmware](../../README.md)

Device-specific pure logic: for each device the parameter table and the rule that turns the stored settings into the driver's
configuration, with a validator where the parameters depend on each other. Depends on [fw-core](../fw-core/README.md) and the
driver crates; tested on the host (`cargo host-test -p fw-settings`) and builds for the board.

```mermaid
flowchart LR
  tbl["tables per device"] --> store["fw_core::params store"]
  store --> cfg["settings_to_config"] --> drv["driver Config"]
```

| Module | Parameters it defines | Node |
|---|---|---|
| `baro` | Barometer rate, oversampling, filter | [parameters](../../docs/parameters.md) |
| `mag` | Compass settings, soft-iron matrix, health test | [calibration](../../docs/calibration.md) |
| `batt` | Cells, thresholds, ADC scales | [parameters](../../docs/parameters.md) |
| `imu` | Calibration offsets, chip settings, orientation | [calibration](../../docs/calibration.md) |
| `gnss` | Fix interval and the NAV-PVT mapping | [device-layer](../../docs/device-layer.md) |
| `rc` | Stick calibration `RCn_*` and `RCMAP_*`, switch rules | [flight-modes](../../docs/flight-modes.md) |
| `flight` | Pilot speed, notch switch, mission and guidance, failsafe, attitude gains | [parameters](../../docs/parameters.md) |
| `esc` | Motor pole count | [esc](../../docs/esc.md) |
| `led` | LED brightness | [health](../../docs/health.md) |
| `sd` | Log backend | [logging](../../docs/logging.md) |

Back to the [README](../../README.md).
