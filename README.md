# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today this repository is a skeleton. Its single binary starts embassy, prints the name of one public
type from each device driver to prove the driver is linked, prints `alive`, and exits through
semihosting with a pass verdict. It touches no bus and no sensor and has no flight functionality.

## Device drivers

The device drivers are not part of this repository. Each one lives in its own repository and is used
by local path (`../MARV-V2-drivers/<crate>` in `Cargo.toml`). The two folders must sit side by side:

```
<parent folder>/
|-- MARV-V2-firmware/        this repository
|   |-- Cargo.toml  build.rs  memory.x  .cargo/config.toml
|   |-- src/main.rs
|   |-- tools/run.sh
|   `-- docs/pinout.md  docs/devices.md
`-- MARV-V2-drivers/
    |-- icm45686/   lsm6dsk320x/   bmp581/   bmm350/
    `-- lc76g/      pio-sdmmc/     qmi-psram/ ws2812c/
```

| Driver | Path | Device |
|---|---|---|
| icm45686 | `../MARV-V2-drivers/icm45686` | ICM-45686 6-axis IMU (SPI) |
| lsm6dsk320x | `../MARV-V2-drivers/lsm6dsk320x` | LSM6DSK320X IMU with high-g accelerometer (SPI) |
| bmp581 | `../MARV-V2-drivers/bmp581` | BMP581 barometer (I2C) |
| bmm350 | `../MARV-V2-drivers/bmm350` | BMM350 magnetometer (I2C) |
| lc76g | `../MARV-V2-drivers/lc76g` | LC76G GNSS receiver (UART) |
| pio-sdmmc | `../MARV-V2-drivers/pio-sdmmc` | SD card on the RP2350 PIO, 4-bit SD bus |
| qmi-psram | `../MARV-V2-drivers/qmi-psram` | 16 MiB QSPI PSRAM on the RP2350 QMI |
| ws2812c | `../MARV-V2-drivers/ws2812c` | WS2812C status LED (PIO) |

For what each driver offers and how to use it, read the README in its own repository.

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

- `docs/pinout.md`: the board pin assignment.
- `docs/devices.md`: the devices on the board and what was measured on them.

## Licence

MIT, see `LICENSE`.
