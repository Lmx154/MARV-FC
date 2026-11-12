# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today this repository is a skeleton. Its single binary starts embassy, prints the name of one public
type from each device driver to prove the driver is linked, prints `alive`, and exits through
semihosting with a pass verdict. It touches no bus and no sensor and has no flight functionality.

## Device drivers

The device drivers are not part of this repository. Each one lives in its own GitHub repository and
`Cargo.toml` takes it from there, pinned to a commit (`rev`); `Cargo.lock` is committed. The first build
on a machine fetches them over https. To take a newer driver, change its `rev` and run
`cargo update -p <crate>`. To work on a driver and the firmware together, add a `[patch]` entry with a
local path (the comment in `Cargo.toml` shows it) instead of editing the dependency.

| Driver crate | Repository | Device |
|---|---|---|
| icm45686 | <https://github.com/Lmx154/icm45686-rs> | ICM-45686 6-axis IMU (SPI) |
| lsm6dsk320x | <https://github.com/Lmx154/lsm6dsk320x-rs> | LSM6DSK320X: high-g accelerometer plus low-g accelerometer and gyroscope (SPI) |
| bmp581 | <https://github.com/Lmx154/bmp581-rs> | BMP581 barometer (I2C) |
| bmm350 | <https://github.com/Lmx154/bmm350-rs> | BMM350 magnetometer (I2C) |
| lc76g | <https://github.com/Lmx154/lc76g-rs> | LC76G GNSS receiver (UART) |
| pio-sdmmc | <https://github.com/Lmx154/pio-sdmmc-rs> | SD card on the RP2350 PIO, 4-bit SD bus |
| qmi-psram | <https://github.com/Lmx154/qmi-psram-rs> | 16 MiB QSPI PSRAM on the RP2350 QMI |
| ws2812c | <https://github.com/Lmx154/ws2812c-rs> | WS2812C status LED (PIO) |

For what each driver offers and how to use it, read the README in its own repository. What was proven
on the board, what the firmware has to respect, and what is still open: `docs/handoff.md`.

## Build and run

Needs the Rust toolchain with the `thumbv8m.main-none-eabihf` target, `probe-rs`, and a debug probe
connected to the board.

```
cargo build --release     # builds for the board (target set in .cargo/config.toml)
tools/run.sh              # build, flash, run, print the log lines; exit status is the verdict
```

`tools/run.sh` takes the probe lock (`marv-v2-probe.lock` in `XDG_RUNTIME_DIR`), so it waits if another
test is using the probe. Its full log is kept under `logs/` (git-ignored). Expected output: one line
per driver, then `alive` and `RESULT: PASS`; exit status 0.

## Board documentation

- `docs/handoff.md`: the state of the drivers and the starting point for the firmware work (2026-11-11).
- `docs/pinout.md`: the board pin assignment.
- `docs/devices.md`: the devices on the board and what was measured on them.

## Licence

MIT, see `LICENSE`.
