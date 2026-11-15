# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today the binary is the two-core skeleton of the firmware (design: the two-core amendment of
`~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`) with the device layer and its first
device, the BMP581 barometer; the foundation's hardware-free logic is in `crates/fw-core` (see Workspace).
It has no flight functionality.

- Core 0, the I/O core (`embassy_rp::init`, thread-mode executor): `usb_link`, a MAVLink node (system 1,
  component 1) on the USB port: HEARTBEAT at 1 Hz (status from the device rows through
  `fw_core::sys_status`: BOOT while a row is Starting in the first 3 s, then STANDBY), SYS_STATUS at 1 Hz
  (the three sensor masks from the device rows), `MARV_DEVICE_STATUS` (42421) for every device row once per
  second, the parameter protocol over an empty table (count 0),
  STATUSTEXTs from the supervisor; commands get the node's default answer, UNSUPPORTED. `supervisor`, a
  10 ms tick: at 10 Hz it bumps `PING` and kicks the flight core, it tracks the age of `PONG`, gives the
  start-up verdicts, runs the devices' stuck check and transition texts, and writes `[STAT]` lines every
  10 s. `beat0` bumps `BEAT0` every 50 ms (read by nobody yet). `baro`, the barometer's device task (see
  Device layer).
- Core 1, the flight core (started with `embassy_rp::multicore::spawn_core1`, 16 KB static stack, its own
  thread-mode executor): one task, `fw_flight::liveness`, which copies `PING` into `PONG` on each kick. No
  timer, no log, no critical-section primitive, no peripheral there.
- Between the cores only atomics and `fw_core::xcore::Kick`, in internal SRAM. `DMA_IRQ_0` is disabled on
  core 0 before core 1 starts (all DMA belongs to the flight core).

Not in it yet: the devices other than the barometer, parameters, settings on flash, the watchdog, command
handling (so no RESTART or other device request), the radio link (UART), the SD card, the design's text
ring of 8 (a fixed queue of 4 texts stands in).

## Device layer

`src/device.rs` (design item 1, on core 0): each device has a task running the one generic `run_device`
loop over a firmware-side `DeviceAdapter` (`data_ready`, `start`, `acquire`, `status`, `verify`, `apply`,
`recover`, `restart`, `counters`; no `self_test` or `settings_to_config` yet). The loop steps the device's
`fw_core::health::Health` and carries out its action. Sampling is `select(with_timeout(T, data ready),
wake)` with T = max(3 P, 10 ms); a timeout makes ONE status read: data waiting = a missed edge, read it;
nothing = a deadline miss. Every bus call is awaited to completion, never inside a `select` or a timeout.
`verify()` runs about once per second between reads. Each device has a static registry row (`DEVICES`): id,
name, nominal period P, the latest-sample `Watch` (Pa, degC, board time in us, `seq` +1 per publish,
source), a status cell its task alone writes, `progress` (bumped after every await) and `stuck` (written
by the supervisor when `progress` stands still for max(1 s, 20 P)); `RUNNING` holds the id of the device
task that ran last. The supervisor turns each health transition into one STATUSTEXT naming the device
(entered Recovering WARNING, recovered NOTICE, Failed or Absent ERROR; none for Ok/Degraded), at most one
per device per second (later ones wait their turn). Adding a device is a new adapter, a new row and a new
`Reading` variant.

| Row | Device | Bus | Configuration (fixed until parameters arrive) | P | T | Stuck after |
|---|---|---|---|---|---|---|
| BARO | BMP581 (`src/baro.rs`) | I2C1 at 400 kHz, SDA GPIO22, SCL GPIO23, data-ready GPIO21 (rising edge), no DMA | the driver's default: pressure 16x, temperature 2x, normal mode, 50.056 Hz output data rate, no filter, no FIFO, data-ready pulsed | 19 978 us | 59.9 ms | 1 s |

The barometer's `restart` first does a bus clear (up to 9 SCL pulses while SDA is low, then START and
STOP, as the driver's `bmp581_cancel` bench program; neither the driver nor embassy-rp 0.10 has one), then
rebuilds I2C1 and runs `init`.

How to see it: `tools/run.sh` prints one `[INFO]` line per health transition (with the board time in ms)
and, every 10 s, a `[STAT] BARO:` line: state, samples and samples per second over the last 10 s, the
latest `seq`, errors, timeouts, missed edges, recoveries, attempts, the largest gap between samples, the
adapter's bus-call counter and the latest sample. On USB, `marv-link-test` (MARV-GCS) counts one
`MARV_DEVICE_STATUS` per second; the ground station's Devices view shows the row.

Each sample read (`acquire`) is two I2C transactions: `INT_STATUS`, then the data registers.
`INT_STATUS` is clear-on-read, and the loop reads it (through the adapter's `status`) only after a
data-ready timeout; if `acquire` did not read it too, the data-ready flag would never be cleared in normal
running, the first timeout of an outage would find that stale flag, count a "missed edge" and publish old
data as a new sample, and only the second timeout would be a real deadline miss. With the read in every
`acquire` the flag the timeout path sees is fresh: it is set only by a sample that has not been read. About
one extra short transaction per sample (50 Hz). If that read shows the power-on-reset flag (also
clear-on-read) `acquire` returns the error `Por` and remembers the flag, and the next `status` answers
`PowerOnReset` from it (the loop turns a power-on reset into an `Event` only on the status path; the first
`Por` result counts as one error event).

Fault injection (cargo features, off by default; `src/inject.rs`, a wrapper around the adapter that calls
the real adapter and then replaces its result; each acts once, 5 s after the first good sample, and logs an
`[INFO] inject:` line when it starts and ends acting):

- `FEATURES=inject-baro-transient tools/run.sh 60`: after 5 s of good samples `acquire` fails 4 times in a
  row, once. Expected: Ok -> Degraded -> Recovering (WARNING) -> Ok (NOTICE, `recoveries` 1); the 4th
  failure comes after the recovery and gives Ok -> Degraded -> Ok, with no text.
- `FEATURES=inject-baro-persistent tools/run.sh 60`: after 5 s every `acquire`, `recover` and `restart`
  fails. Expected: Recovering, 4 attempts at 100, 300, 1000, 3000 ms after the previous failure (logged),
  then Failed (ERROR) and no bus traffic (the adapter's call counter stands still).
- `FEATURES=inject-baro-missed-edge tools/run.sh 60`: the data-ready edges are swallowed for 200 ms while
  the chip keeps producing data. Shows the missed-edge path: the wait times out, the status read finds data,
  `missed edges` goes up (at least 1) and the sample is read; no health event (state Ok, errors 0,
  timeouts 0, recoveries 0, no text).
- `FEATURES=inject-baro-silent tools/run.sh 60`: for 1 s the edges are swallowed and `status` answers
  "nothing pending". Shows the deadline-miss path: `timeouts` > 0, `last_error` TIMEOUT, Ok -> Degraded ->
  Recovering (WARNING; 3 misses, so about 3 T = 180 ms after the last good sample), then a recovery attempt
  that succeeds -> Ok (NOTICE, `recoveries` 1).
- `FEATURES=inject-baro-verify tools/run.sh 60`: one `verify` answers "mismatch". Shows the reconfigure
  path: one error event (`last_error` CONFIG_MISMATCH), the adapter's `apply` runs on the chip,
  `config_restored` 1, back to Ok without Recovering and without text.
- `FEATURES=inject-baro-stuck tools/run.sh 60`: `acquire` never returns. Shows the supervisor's stuck
  check: after max(1 s, 20 P) `stuck` is set, the device's `MARV_DEVICE_STATUS` row reports FAILED with
  STUCK, one ERROR text "BARO: stuck"; the board is not reset (no watchdog yet) and the rest of the
  firmware (HEARTBEAT, PING/PONG, `[STAT]` lines) keeps running.

Run `tools/run.sh` again afterwards to put the normal build back on the board.

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
tools/run.sh 90           # build, flash, run for 90 s, print the report lines
```

`tools/run.sh` takes the probe lock (`marv-v2-probe.lock` in `XDG_RUNTIME_DIR`), so it waits if another
test is using the probe. Its full log is kept under `logs/` (git-ignored). The firmware runs until the
script's timeout ends it, so exit status 124 is the normal end. Expected output: one `[INFO]` line per
driver (proof that it is linked), four start-up checks, each `[PASS]` or `[FAIL]`: EXTEXCLALL set on
core 0, `DMA_IRQ_0` not enabled on core 0, EXTEXCLALL set on core 1, `PONG` follows `PING` within
200 ms; then a `[STAT]` line every 10 s: uptime, the largest `PONG` age seen (healthy: far below
200 ms), `PING` and `PONG`, the bytes and frames the node wrote and the bytes it read on USB, the link
state; and a `[STAT]` line per device (see Device layer).

From the ground station: the flight controller's own USB port enumerates as a CDC device (VID 0xc0de,
PID 0xcafe, product "MARV V2 firmware"; on Linux the second `/dev/ttyACM*` besides the debug probe's).
The node writes nothing until the station opens the port (DTR held 100 ms, or its HEARTBEAT). Then for
example `marv-link-test --connect serial:/dev/ttyACM1 --count 0 --duration 60` (MARV-GCS) counts one
HEARTBEAT, one SYS_STATUS and one `MARV_DEVICE_STATUS` per second from system 1 component 1, and the STATUSTEXT
"MARV V2 fw 0.1.0: flight core up".

Negative control: `FEATURES=no-fbeat tools/run.sh 30` builds with the cargo feature `no-fbeat`: core 1
starts but its liveness task is not spawned. Expected: `[FAIL] PONG follows PING`, the log line and the
WARNING text "flight core not answering". Run `tools/run.sh` again afterwards to put the normal build
back on the board.

## Workspace

The repository is a Cargo workspace with three members:

```
.                   the firmware binary (package marv-v2-firmware, the workspace root package)
crates/fw-core      pure no_std logic and the lock-free cross-core cells, tested on the host
crates/fw-flight    the flight-core (core 1) tasks; depends on fw-core only
```

`fw-core` holds the foundation layer's logic that needs no hardware. It is implemented and tested on
the host (126 tests); the binary uses `xcore::Kick`, `health`, `sys_status` and `params` (over an empty
table) so far. Design: `~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`.

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

`fw-flight` holds the flight core's liveness answer (`liveness`: waits on a `Kick`, copies `PING` into
`PONG`); its `imu` module is still empty. It must not depend on
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
