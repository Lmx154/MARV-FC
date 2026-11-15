# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today the binary is a skeleton and the foundation's hardware-free logic is in `crates/fw-core`
(see Workspace). The single binary starts embassy, prints the name of one public
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

## Workspace

The repository is a Cargo workspace with three members:

```
.                   the firmware binary (package marv-v2-firmware, the workspace root package)
crates/fw-core      pure no_std logic and the lock-free cross-core cells, tested on the host
crates/fw-flight    the flight-core (core 1) tasks; depends on fw-core only
```

`fw-core` holds the foundation layer's logic that needs no hardware. It is implemented and tested on
the host (126 tests); nothing in it has run on the board yet, and the firmware binary does not use
it yet. Design: `~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`.

| Module | What it is |
|---|---|
| `xcore` | Lock-free cells between the two cores: `Seqlock` (flight core to core 0, one writer that never waits), `TripleBuffer` (core 0 to the flight core, wait-free both ways), `Kick` (request bits plus a lock-free wake). The only module with `unsafe`. Tested with two host threads over 10^7 writes; statics using them must live in the internal SRAM, and core 1 must be started with embassy-rp's `spawn_core1` (it sets the bit that makes atomics work across the cores) |
| `handshake` | Arming and saving settings can never both win: two flags, each side stores its own, then reads the other's |
| `health` | The per-device health state machine: Starting, Ok, Degraded, Recovering, Failed, Absent; error score, start retries, 4 recovery attempts with back-off, the flap limit. Events and time in, the next action out |
| `params` | The parameter store over a static table of definitions: ranges enforced (a refused write keeps the old value), a validator per device for whole-configuration checks, per-device "apply" masks, and the `ParamStore` trait of `mavlink-device`. No real parameter table yet |
| `record` | The settings record for the two 4 KB flash sectors of the `SETTINGS` region: "MRVS", sequence, 24-byte entries, CRC-32; the choice of the current sector at boot |
| `scratch` | The watchdog's record in four scratch registers (layout version 2) |
| `admission` | What happens to a MAVLink command: classification of 241, 245, 246 and the project's 42430, the admission order, and the books of the one long action with its progress and its single final acknowledgement |
| `sys_status` | From the device rows to SYS_STATUS's three masks, the HEARTBEAT state and the `MARV_DEVICE_STATUS` row |

`fw-flight` is still an empty skeleton (module files with doc comments only). It must not depend on
`embassy-time`, `embassy-sync` or `defmt`; check it with
`cargo tree -p fw-flight -e normal --depth 1 | grep -cE 'embassy-time|embassy-sync|defmt'` (prints 0).

`.cargo/config.toml` sets the board target for the whole workspace, so the two commands are:

```
cargo host-test -p fw-core    # tests on the host (alias: `cargo test --target x86_64-unknown-linux-gnu`)
cargo build --release         # the firmware for the board (thumbv8m.main-none-eabihf)
```

`cargo host-clippy -p fw-core -p fw-flight --all-targets` is the matching host lint. The host alias
assumes an x86_64 Linux host; on another host, pass your own `--target`.

## Flash layout

`memory.x` gives the program `FLASH` 2040K (0x10000000 to 0x101FE000) and reserves the last 8 KB of the
2 MiB flash, 0x101FE000 to 0x10200000, as the `SETTINGS` region for the settings record.

## Board documentation

- `docs/handoff.md`: the state of the drivers and the starting point for the firmware work (2026-11-11).
- `docs/handoff-usb-mavlink.md`: the USB link, the MAVLink layer and the ground station, each in its own
  GitHub repository, and how this firmware takes them (2026-11-12).
- `docs/pinout.md`: the board pin assignment.
- `docs/devices.md`: the devices on the board and what was measured on them.

## Licence

MIT, see `LICENSE`.
