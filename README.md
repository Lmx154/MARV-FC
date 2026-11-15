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
  second, the parameter protocol over the barometer's five parameters (see Parameters),
  STATUSTEXTs from the supervisor and from refused parameter writes, and the commands (see Commands).
  `supervisor`, a 10 ms tick: at 10 Hz it bumps `PING` and kicks the flight core, it tracks the age of
  `PONG`, gives the start-up verdicts, runs the devices' stuck check and transition texts, ends the running
  action (`actions::tick`), and writes `[STAT]` lines every 10 s. `beat0` bumps `BEAT0` every 50 ms (read by nobody yet). `baro`, the barometer's device task (see
  Device layer).
- Core 1, the flight core (started with `embassy_rp::multicore::spawn_core1`, 16 KB static stack, its own
  thread-mode executor): `fw_flight::liveness`, which copies `PING` into `PONG` on each kick, and IMU1
  (`src/imu1.rs`): the ICM-45686 on SPI0 with DMA channels 0 and 1 and its INT1 on GPIO15, all built inside
  core 1's entry closure so their interrupts land on core 1, running the flight-core device loop
  `fw_flight::imu` (see Flight-core device loop). No log, no critical-section primitive of its own, and no
  timer in the sample loop there.
- Between the cores only atomics and `fw_core::xcore` cells (`Kick`, and IMU1's `Seqlock` and `Kick` in
  `fw_flight::imu::ImuCells`), in internal SRAM. `DMA_IRQ_0` is disabled on core 0 before core 1 starts (all
  DMA belongs to the flight core); start-up checks: not enabled on core 0, enabled on core 1, still not
  enabled on core 0 after core 1 started.

Not in it yet: IMU1's parameters and calibration (a fixed configuration for now), IMU2 and the devices
other than IMU1 and the barometer, settings on flash (parameters live in RAM and are lost
at every reset; there is no save), the watchdog, reboot, the calibrations and the self-test, the radio link
(UART), the SD card, the design's text ring of 8 (a fixed queue of 4 texts stands in).

## Flight-core device loop

`crates/fw-flight/src/imu.rs` (design: two-core amendment): `ImuTask` runs one IMU over an `ImuAdapter`
(the core-0 adapter contract: `start`, `acquire`, `status`, `verify`, `apply`, `recover`, `restart`, success
only after a fresh sample). It waits on `select(data-ready edge, kick)` and never on a timer; core 0's
supervisor kicks CHECK every 10 ms tick while the row's last sample is older than T = max(3 P, 10 ms); the
woken task compares the clock with its last sample and, if late by more than T, makes the one status read
(sample waiting = missed edge, read it; nothing = deadline miss). RESTART comes with its request number
through the same kick; APPLY is reserved. Start and recovery wait through the `DelayNs` passed in, wrapped in
`CountingDelay` (count after the start: 0 in a clean run, invariant F2); `verify` between samples (about
once per second, about 1.0 to 1.4 ms) waits on a busy loop over the time driver's counter, no timer, and
is followed by one status read: a sample already waiting is read at once (a host test pins this rule).
The board time is `embassy_time::Instant::now()`, lock-free on core 1 (embassy-rp 0.10.0
`time_driver.rs:35-44` reads the timer's raw registers; only `schedule_wake`, `:46-47`, takes the critical
section, and the sample loop never schedules). Everything core 0 needs
(state, counters, the latest sample, the largest gap, missed edges, delay counts, restart outcome, the last
four transitions, attempt times, injection state) goes out through one `Seqlock`; the supervisor copies it
into IMU1's registry row each tick and logs and texts its events; while IMU1 is Failed or Absent it gets
no CHECK kick and is not flagged stuck (it is parked and waits only for RESTART). Host tests: `cargo host-test -p fw-flight`
(9 tests).

IMU1's fixed configuration: 4 g, 250 dps, 800 Hz on both sensors, internal clock (as `icm45686_smoke` ran
it on this board), data-ready on INT1 in latched mode (high until INT1_STATUS0 is read, as `icm45686_int1`
showed), waited for by level, so a sample that arrives while the task is busy is not lost; 800 Hz because
the ICM-45686 has no 1 kHz rate. Single-sample mode
(`read()` on every data-ready signal, the driver README's single-sample path), with INT1_STATUS0 (clear on
read) read with every sample so that the status read after a late CHECK sees a fresh flag; no FIFO, so
the overflow count stays 0. Its values go to the station as HIGHRES_IMU (id 0, acceleration, gyro,
temperature, the other fields NaN with their `fields_updated` bits clear) at 25 Hz; the barometer's as
SCALED_PRESSURE at 25 Hz.

Features: `FEATURES=inject-imu1-persistent tools/run.sh 120`: 5 s after the first good sample every
`acquire`, `recover` and `restart` fails until `marv-cli device restart IMU1`. `FEATURES=inject-imu1-missed-edge
tools/run.sh 60`: 5 s after the first good sample the data-ready edges are swallowed for 200 ms. The
injection state goes out through the cells and core 0 logs it. `FEATURES=stall-core0 tools/run.sh 60`: one
core-0 task spins 3 s without yielding 20 s after start and logs IMU1's samples before and after (F1).
`FEATURES=f4-negative tools/run.sh 20`: a DMA channel is built on core 0 before the start-up check
(negative control of F4: `[FAIL] DMA_IRQ_0 not enabled on core 0`), its line disabled again before core 1
starts.

## Device layer

`src/device.rs` (design item 1, on core 0): each device has a task running the one generic `run_device`
loop over a firmware-side `DeviceAdapter` (`data_ready`, `start`, `acquire`, `status`, `verify`, `apply`,
`recover`, `restart`, `counters`; no `self_test` yet). The loop steps the device's
`fw_core::health::Health` and carries out its action. Sampling is `select(with_timeout(T, data ready),
wake)` with T = max(3 P, 10 ms); a timeout makes ONE status read: data waiting = a missed edge, read it;
nothing = a deadline miss. Every bus call is awaited to completion, never inside a `select` or a timeout.
A start, `recover` or `restart` that returns `Ok` only says the bus and the device answer: the loop then
waits for ONE fresh sample (`with_timeout(T, data ready)`, then one `acquire` awaited to completion) before
it tells `Health` the step worked; a timeout or a failed `acquire` counts as a failure (a failed `recover`
goes on to `restart`, a failed `restart` ends the attempt, a failed start is a failed start), so "recovered"
and "started" mean data flows. `verify()` runs about once per second between reads. Each device has a static registry row (`DEVICES`): id,
name, nominal period P, the latest-sample `Watch` (Pa, degC, board time in us, `seq` +1 per publish,
source), a status cell its task alone writes, `progress` (bumped after every await) and `stuck` (written
by the supervisor when `progress` stands still for max(1 s, 20 P)); `RUNNING` holds the id of the device
task that ran last. The supervisor turns each health transition into one STATUSTEXT naming the device
(entered Recovering WARNING, recovered NOTICE, Failed or Absent ERROR; none for Ok/Degraded), at most one
per device per second (later ones wait their turn). Adding a device is a new adapter, a new row and a new
`Reading` variant.

| Row | Device | Bus | Configuration | P | T | Stuck after |
|---|---|---|---|---|---|---|
| BARO | BMP581 (`src/baro.rs`) | I2C1 at 400 kHz, SDA GPIO22, SCL GPIO23, data-ready GPIO21 (rising edge), no DMA | from the parameters (below); the defaults are the driver's default: pressure 16x, temperature 2x, normal mode, 50.056 Hz output data rate, no filter; always no FIFO, data-ready pulsed | 19 978 us at the default; 1 / rate afterwards | max(3 P, 10 ms): 59.9 ms at the default | max(1 s, 20 P) |

The row's period P is a value that changes: the adapter sets it from the configured rate (at every start
and every apply, before the chip is configured), so T = max(3 P, 10 ms), the stuck threshold and the
`[STAT]` rate follow the rate.

Requests to a device task: the row has an APPLY request and a RESTART request (a flag byte, plus the row's
`wake` signal). The task takes them at the top of its loop, never inside a bus call. RESTART is allowed in
every state: the loop feeds `Event::Restart` to `Health` (Starting, from Ok, Degraded, Recovering, Failed
and Absent alike) and starts the device again; a back-off wait is cut short by it. APPLY is carried out when
the device is Ok or Degraded: the loop calls the adapter's `apply` (the driver's `configure` on the running
chip), awaited to completion; if it fails that is an error event for `Health`. A start or restart reads the
stored settings itself, so it also carries out a waiting APPLY.

The barometer's `restart` first does a bus clear (up to 9 SCL pulses while SDA is low, then START and
STOP, as the driver's `bmp581_cancel` bench program; neither the driver nor embassy-rp 0.10 has one), then
rebuilds I2C1 and runs `init`.

How to see it: `tools/run.sh` prints one `[INFO]` line per health transition (with the board time in ms)
and, every 10 s, a `[STAT] BARO:` line: state, samples and samples per second over the last 10 s, the
latest `seq`, errors, timeouts, missed edges, recoveries, config restored, attempts, the largest gap between samples, the
adapter's bus-call counter and the latest sample. On USB, `marv-link-test` (MARV-GCS) counts one
`MARV_DEVICE_STATUS` per second; the ground station's Devices view shows the row.

Each sample read (`acquire`) is two I2C transactions: `INT_STATUS`, then the data registers.
`INT_STATUS` is clear-on-read, and the loop reads it (through the adapter's `status`) only after a
data-ready timeout; if `acquire` did not read it too, the data-ready flag would never be cleared in normal
running, the first timeout of an outage would find that stale flag, count a "missed edge" and publish old
data as a new sample, and only the second timeout would be a real deadline miss. With the read in every
`acquire` the flag the timeout path sees is fresh: it is set only by a sample that has not been read. About
one extra short transaction per sample (50 Hz). If that read shows the power-on-reset flag (also
clear-on-read) `acquire` returns `Err(Por)`, which the loop feeds to `Health` as a power-on-reset event
(Recovering at once, not an error event).

Fault injection (cargo features, off by default; `src/inject.rs`, a wrapper around the adapter that calls
the real adapter and then replaces its result; each acts once, 5 s after the first good sample, and logs an
`[INFO] inject:` line when it starts and ends acting):

- `FEATURES=inject-baro-transient tools/run.sh 60`: after 5 s of good samples `acquire` fails 4 times in a
  row, once. Expected: Ok -> Degraded -> Recovering (WARNING); the 4th
  failure lands in the confirming `acquire` of recovery attempt 1, so its `recover` counts as failed and
  the same attempt goes on to `restart`, which succeeds -> Ok (NOTICE, `recoveries` 1, `errors` 3).
- `FEATURES=inject-baro-persistent tools/run.sh 60`: after 5 s every `acquire`, `recover` and `restart`
  fails. Expected: Recovering, 4 attempts at 100, 300, 1000, 3000 ms after the previous failure (logged),
  then Failed (ERROR) and no bus traffic (the adapter's call counter stands still).
- `FEATURES=inject-baro-missed-edge tools/run.sh 60`: the data-ready edges are swallowed for 200 ms while
  the chip keeps producing data. Shows the missed-edge path: the wait times out, the status read finds data,
  `missed edges` goes up (at least 1) and the sample is read; no health event (state Ok, errors 0,
  timeouts 0, recoveries 0, no text).
- `FEATURES=inject-baro-silent tools/run.sh 60`: for 1 s the edges are swallowed and `status` answers
  "nothing pending". Shows the deadline-miss path: `timeouts` > 0, `last_error` TIMEOUT, Ok -> Degraded ->
  Recovering (WARNING; 3 misses, so about 3 T = 180 ms after the last good sample); recovery attempts fail
  while the window is open (the bus answers but no fresh sample comes), then one succeeds -> Ok (NOTICE,
  `recoveries` 1).
- `FEATURES=inject-baro-verify tools/run.sh 60`: one `verify` answers "mismatch". Shows the reconfigure
  path: one error event (`last_error` CONFIG_MISMATCH), the adapter's `apply` runs on the chip,
  `config_restored` 1, back to Ok without Recovering and without text.
- `FEATURES=inject-baro-fail-until-restart tools/run.sh 120`: as `persistent` (after 5 s every `acquire`,
  `recover` and `restart` fails; Recovering, 4 attempts, Failed), until a RESTART request has been raised
  (`device restart BARO` from the ground station); from then on everything works and the device is Ok again.
- `FEATURES=inject-baro-por tools/run.sh 60`: one `acquire` answers a power-on reset. Shows the
  power-on-reset path: Ok -> Recovering at once (no Degraded, `errors` not incremented, `last_error` POR),
  WARNING, a recovery, Ok, NOTICE.
- `FEATURES=inject-baro-stuck tools/run.sh 60`: `acquire` never returns. Shows the supervisor's stuck
  check: after max(1 s, 20 P) `stuck` is set, the device's `MARV_DEVICE_STATUS` row reports FAILED with
  STUCK, one ERROR text "BARO: stuck"; the board is not reset (no watchdog yet) and the rest of the
  firmware (HEARTBEAT, PING/PONG, `[STAT]` lines) keeps running.

Run `tools/run.sh` again afterwards to put the normal build back on the board.

## Parameters

The first real parameter table (`crates/fw-settings/src/baro.rs`, a `static [ParamDef; 5]`), all `I32`, device BARO,
apply class Configure. The values are plain integers; the defaults are what `bmp581::Config::new()` ran
before:

| Name | Default | Range | Means |
|---|---|---|---|
| `BARO_ODR_HZ` | 50 | 1 to 240 | output data rate in Hz, one of 1, 2, 3, 4, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 100, 120, 130, 140, 160, 180, 200, 220, 240; the printed rate is the chip's code (50 is 50.056 Hz, 25 is 25.005 Hz, 100 is 100.299 Hz, 130 is 129.855 Hz, 180 is 179.2 Hz, 200 is 199.111 Hz, 220 is 218.537 Hz); the chip's other codes (sub-hertz, 89.6, 110.164, 149.333 Hz) have no whole-hertz name and are not offered |
| `BARO_OSR_P` | 16 | 1 to 128 | pressure oversampling: 1, 2, 4, 8, 16, 32, 64 or 128 |
| `BARO_OSR_T` | 2 | 1 to 128 | temperature oversampling, the same values |
| `BARO_IIR_P` | 0 | 0 to 127 | pressure IIR filter coefficient: 0 (off), 1, 3, 7, 15, 31, 63 or 127 |
| `BARO_IIR_T` | 0 | 0 to 127 | temperature IIR filter coefficient, the same values |

A write is checked before it is stored. The range check is the store's (`fw_core::params`: out of
`[min, max]` is refused, not clamped). Then the validator (`fw_settings::baro::validate`, installed with
`Params::set_validator`) builds the device's whole candidate `Config` with the pure
`fw_settings::baro::settings_to_config` (the candidate values: the stored ones with the proposed one in place of
its own) and refuses the write if that fails: a value the chip does not offer (the rate not in the list,
an oversampling that is not a power of two, a coefficient not in the list), or a combination the driver's own
`Config::validate` refuses (the rate above what the oversampling allows in normal mode, the driver's
`calc::max_odr`; no limit of this firmware's own). A refused write is answered with the OLD value and ONE
STATUSTEXT (WARNING) per refused write, "<NAME>: <reason>", for example "BARO_OSR_P: rate too high for the
oversampling", "BARO_ODR_HZ: rate not offered by the chip", "BARO_ODR_HZ: out of range". The ground
station `marv-cli param set` repeats a write that is not echoed, so it makes three writes and the board
sends three texts.

How a write takes effect: an accepted write is stored in RAM (`src/store.rs`, one `Params` in a
critical-section mutex that the node's `ParamStore`, the actions and the device task all reach through short
accesses, never across an await), the store's `take_configure_mask` names the device, and the firmware raises
that device's APPLY request. The barometer task then reads the stored values, builds the `Config` with
`settings_to_config`, sets its period P, and calls the driver's `configure` on the running chip; it logs the
configuration and "applied" (`[INFO] BARO: configuration: ...`). About 5 ms from the write to the applied chip.
Every start and restart reads the stored values the same way. NOTHING IS SAVED TO FLASH: no automatic save,
no save command (it answers "save: not built yet"); a reset or power cycle returns every parameter to its
default. Every parameter write, accepted or refused, has an `[INFO] param <NAME> = <value>: ...` line.

## Commands

`Telemetry::command` and `Telemetry::acks` on the USB link, over ONE `fw_core::admission::ActionBook`
(`src/actions.rs`; link id 0 = USB; `armed` and `hil` are false until the arming code and the HIL source
exist; `ActionBook::tick` runs in the supervisor). The answer to an admitted command is IN_PROGRESS 0 %;
the final ACK follows through `acks` (about 100 ms later, the node asks at least every 100 ms). Denied
commands put the book's text out as a STATUSTEXT. Every decision logs
`[INFO] command <number> from <sys>/<comp>: <result>`, every action `action <name> starts` and
`action <name> ended: <result>`, then `final ACK of command <number>: <result>`.

| Command (`marv-cli`) | What it does today |
|---|---|
| `device restart BARO` | raises the barometer's RESTART request; the loop feeds `Event::Restart` (from any state) and starts the device again; ACCEPTED when the device reaches Ok, FAILED when it reaches Failed or Absent first (the book's own 5 s limit covers a hang); the action records the number of the restart request it raised and ends only on that request's outcome, so an older restart finishing late never ends a later action; progress stays 0 |
| `param reset-all --yes` | `Params::reset_all`; each device whose values changed gets its APPLY request; ACCEPTED as soon as the store is reset and the requests are raised, without waiting for the applies (as a parameter write does not wait; a device that is Failed or Recovering picks the values up when it is restarted) |
| `param reset-calibration --yes` | `Params::reset_calibration`, the same way; there are no calibration parameters yet, so it changes nothing and is ACCEPTED |
| `device restart <other device>`, `param save`, `param reload`, `reboot --yes`, `calibrate gyro`, `accel`, `compass`, `device selftest <NAME>` | admitted, then FAILED at once with ONE STATUSTEXT "<what>: not built yet" (for example "restart GNSS: not built yet", "save: not built yet"). No fake success |
| `device restart RC` (a device this build does not serve), and any command or parameter the admission does not accept | DENIED by the admission, its text as a STATUSTEXT |

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
driver (proof that it is linked), six start-up checks, each `[PASS]` or `[FAIL]`: EXTEXCLALL set on
core 0, `DMA_IRQ_0` not enabled on core 0, EXTEXCLALL set on core 1, `DMA_IRQ_0` enabled on core 1,
`DMA_IRQ_0` still not enabled on core 0, `PONG` follows `PING` within 200 ms; then a `[STAT]` line every 10 s: uptime, the largest `PONG` age seen (healthy: far below
200 ms), `PING` and `PONG`, the bytes and frames the node wrote and the bytes it read on USB, the link
state; and a `[STAT]` line per device (see Device layer).

From the ground station: the flight controller's own USB port enumerates as a CDC device (VID 0xc0de,
PID 0xcafe, product "MARV V2 firmware"; on Linux the second `/dev/ttyACM*` besides the debug probe's).
The node writes nothing until the station opens the port (DTR held 100 ms, or its HEARTBEAT). Then for
example `marv-link-test --connect serial:/dev/ttyACM1 --count 0 --duration 60` (MARV-GCS) counts one
HEARTBEAT, one SYS_STATUS and two `MARV_DEVICE_STATUS` (IMU1, BARO) per second, 25 HIGHRES_IMU and 25
SCALED_PRESSURE per second from system 1 component 1, and the STATUSTEXT
"MARV V2 fw 0.1.0: flight core up".

Negative control: `FEATURES=no-fbeat tools/run.sh 30` builds with the cargo feature `no-fbeat`: core 1
starts but its liveness task is not spawned. Expected: `[FAIL] PONG follows PING`, the log line and the
WARNING text "flight core not answering". Run `tools/run.sh` again afterwards to put the normal build
back on the board.

## Workspace

The repository is a Cargo workspace with four members:

```
.                   the firmware binary (package marv-v2-firmware, the workspace root package)
crates/fw-core      pure no_std logic and the lock-free cross-core cells, tested on the host
crates/fw-flight    the flight-core (core 1) tasks; depends on fw-core and embedded-hal-async only
crates/fw-settings  per device, the parameter table and the pure settings-to-configuration rule; depends on fw-core and the driver crates
```

`fw-core` holds the foundation layer's logic that needs no hardware. It is implemented and tested on
the host (127 tests); the binary uses `xcore::Kick`, `health`, `sys_status`, `params` and `admission` so far. Design: `~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`.

| Module | What it is |
|---|---|
| `xcore` | Lock-free cells between the two cores: `Seqlock` (flight core to core 0, one writer that never waits), `TripleBuffer` (core 0 to the flight core, wait-free both ways), `Kick` (request bits plus a lock-free wake). The only module with `unsafe`. Tested with two host threads over 10^7 writes; statics using them must live in the internal SRAM, and core 1 must be started with embassy-rp's `spawn_core1` (it sets the bit that makes atomics work across the cores) |
| `handshake` | Arming and saving settings can never both win: two flags, each side stores its own, then reads the other's |
| `health` | The per-device health state machine: Starting, Ok, Degraded, Recovering, Failed, Absent; error score, start retries, 4 recovery attempts with back-off, the flap limit. Events and time in, the next action out |
| `params` | The parameter store over a static table of definitions: ranges enforced (a refused write keeps the old value), a validator per device for whole-configuration checks, per-device "apply" masks, and the `ParamStore` trait of `mavlink-device`. The table is the firmware's (`crates/fw-settings/src/baro.rs`) |
| `record` | The settings record for the two 4 KB flash sectors of the `SETTINGS` region: "MRVS", sequence, 24-byte entries, CRC-32; the choice of the current sector at boot |
| `scratch` | The watchdog's record in four scratch registers (layout version 2) |
| `admission` | What happens to a MAVLink command: classification of 241, 245, 246 and the project's 42430, the admission order, and the books of the one long action with its progress and its single final acknowledgement |
| `sys_status` | From the device rows to SYS_STATUS's three masks, the HEARTBEAT state and the `MARV_DEVICE_STATUS` row |

`fw-settings` holds the device-specific pure logic: per device, the parameter table and the pure
`settings_to_config` with its validator (today `baro`, over the `bmp581` driver's `Config`, without the
driver's `defmt` feature). Device-specific pure logic goes here, generic logic stays in `fw-core`. Tested on
the host with `cargo host-test -p fw-settings` (6 tests: defaults equal `Config::new()` and P = 19 978 us,
every refusal text, every offered rate within 1 % of its name and all distinct, the table accepted by
`Params::new` with every default passing the validator, every offered value of every parameter accepted or
refused only by the rate-against-oversampling rule with the refused sets listed, and the real store
refusing before storing); it also builds for the board (`cargo build -p fw-settings --release`).

`fw-flight` holds the flight core's liveness answer (`liveness`: waits on a `Kick`, copies `PING` into
`PONG`) and the IMU device loop (`imu`, see Flight-core device loop). It must not depend on
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
