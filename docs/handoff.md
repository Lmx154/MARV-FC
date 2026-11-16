# MARV V2 firmware: hand-off

State on 2026-11-15, late evening; the flight core in SRAM added on 2026-11-16 (branch `wip-ram-code`, not
in `main`). Written for whoever picks the work up next, human or agent. It says what
is built and proven, what is in progress, and what is left. It is an overview: the details and the evidence
are in the documents it points to.

## Where everything is

| Thing | Place | State |
|---|---|---|
| Firmware | `~/Documents/MARV-V2-firmware` (GitHub `Lmx154/MARV-FC`) | `main` is current and pushed |
| Driver crates, one repo each | `~/Documents/MARV-V2-drivers/<crate>` | every repo on `main`, equal to its remote |
| Ground station and `marv-cli` | `~/Documents/MARV-GCS` | `main` = `mvp`, pushed |
| Working plan (the log of everything, with the owner's words verbatim) | `docs/plan.md` in the firmware repo (git-ignored, local) | current |
| Design decisions and measured facts | `~/Documents/MARV-V2-drivers/_plan/` | see below |

Read these before changing anything:

- `README.md` here: what the firmware does, every cargo feature, what was seen on the board.
- `docs/resources.md`: who owns which pin, PIO, DMA channel and interrupt, and the rules that follow.
- `_plan/decision-foundation-20261113.md`: the foundation design (two cores, device layer, parameters, actions,
  watchdog), with its amendments.
- `_plan/facts-foundation.md`: what was measured and read (cross-core hazards, flash, timing).
- `_plan/decision-sd-logging-20261115.md` and `_plan/facts-sd-logging.md`: the SD logging design and its facts.
- `_plan/WARNING-oom-tab-kills-20261115.md`: why sessions were killed and the rules that prevent it.
- `docs/handoff-esc-link.md`: the ESC work that is not in the firmware yet.

## What the owner asked for

The foundation, not the flight loop: every device always on at the lowest layer with health and diagnostics;
settings as MAVLink parameters; actions (restart, self-test, calibrations, save, reboot); watchdogs; the ground
station as display and controller on any link and as configurator on serial or TCP only; a companion computer
through a command-line tool; the two-core split. His standard for what counts as good enough is ArduPilot, PX4
and Betaflight, nothing stricter, except where the two-core design has no precedent.

## Done and proven on the board

- **Two cores.** Core 1 is the flight core: both IMUs at 800 Hz, nothing else. Core 0 does everything else.
  Data crosses only through lock-free cells.
- **Seven device rows:** IMU1, IMU2, BARO, MAG, GNSS, BATT (supervised: health states, recovery ladder,
  operator restart) and RC (report only: "the radio link is up").
- **Parameters:** 25, with ranges and per-device validation. A write takes effect in RAM; flash is written
  only on the user's Save. Reload, reset, load at boot. Survives re-flash and a real power cycle.
- **Actions:** device restart; self-test of MAG, IMU1, IMU2; gyro, level-accelerometer and compass
  calibration (results applied, never saved by themselves); storage save/reload/reset; reboot.
- **Watchdog:** resets when either core stops answering, keeps a record of why, reports it at the next boot.
- **Radio (ELRS):** both receiver modes. CRSF model: sticks at about 245 per second, battery telemetry shown
  on the transmitter. MAVLink model: telemetry over the air (about 134 B/s). Mode switches both ways while
  running; transmitter off and on in each mode. The owner uses the CRSF model for sticks and the MAVLink model
  for telemetry only.
- **I2C faults:** both buses are cleared at every boot; a hung transfer times out (ArduPilot's formula) and
  the bus is cleared and rebuilt by the normal recovery ladder.
- **Ground station:** Devices view, parameter editing with Save, command buttons, the configure/operate gate.
  `marv-cli` covers the same from a shell; a full companion session over USB passed on this desktop.

## In progress: SD card logging

Design decided (`_plan/decision-sd-logging-20261115.md`): the SD driver completes its DMA channel on
`DMA_IRQ_1` on core 0 (no HAL fork); FAT32 through `embedded-fatfs` (approved by the owner); logs in
ArduPilot's `.BIN` format, from boot, one file per boot, about 5.6 kB/s; a 32 KiB ring that drops and counts
when the card stalls; card reader first, then MAVLink log download over USB.

| Step | What | State |
|---|---|---|
| P2 | DMA on the second interrupt line, on the board | done (branch `wip-sd-p2`, pushed, not in `main`) |
| SD1 | The driver's new DMA back-end | done, reviewed, `pio-sdmmc` `main` = `424fd58` |
| P1 | The `.BIN` encoder (`fw_core::dflog`), read back by ArduPilot's `mavlogdump.py` | done, in `main` |
| P3 | Format and mount the card inside the firmware, 16 MiB write and read-back | done (branch `wip-sd-p3`, merged into `wip-sd2`; the probes stay as features) |
| SD2 | The SD card as a device row (mount, capacity, health) | done, merged into `main` (README "SD card") |
| SD3 | The log writer and what is logged | built, branch `wip-sd3` (README "Logging"): 30-minute run with 3 saves, an 8.5-hour soak, both read back on the board with 0 bad bytes; RESTART, the write-fail injection and the clean-up seen. Not yet: a PC-side read of the files (`mavlogdump.py`, UAV Log Viewer), a power cut, a card removal |
| SD4 | Log list and download over USB (`marv-cli logs list/get`) | after SD3 |

The card in the slot is a scratch card and may be erased (owner's word). There is no card reader on the
desk, so checks that compare a file on a PC are done by read-back on the board or wait for SD4.

## Left open

**Waiting on the owner**
- The magnetometer is not mounted (it hangs on its cable). The real compass calibration waits for the mount.
  The firmware has no compass-orientation setting: either mount it aligned with the board or add one then.

**Not built**
- ESC in the firmware: the DShot driver and the passthrough work as driver crates with bench programs, but
  the firmware has no ESC row, no passthrough through its USB port, no stop on signal loss; the board's
  current scale and ESC serial telemetry are not done. See `docs/handoff-esc-link.md`.
- Status LED. The 16 MiB PSRAM is fitted (hand-fitted in U24) and its driver was measured on the board, but
  the firmware does not use it yet. It can hold data (not code: it shares the flash cache).
- Full-rate IMU data from the flight core to core 0 (a ring is reserved in the design; needed for a control
  loop and for full-rate logging).

**Known and parked**
- Late IMU samples: with the flight core in SRAM (branch `wip-ram-code`) a 200 s run of the normal build shows
  0 samples later than 1.5 P per 10 s window for both IMUs (before: 6 to 20 and 23 to 53), largest gaps 1.6
  and 1.7 ms, rates within 1 %, no errors (2026-11-16, debugger attached; not repeated without one). The
  shared XIP cache was their main cause (README "Flight core in SRAM"; facts-sd-logging C3). Measurement
  builds of the earlier search are on branch `wip-csstat`.
- About 0.1 receive errors per second on the radio UART in normal running.

**Never exercised on real hardware**
- A failing self-test; a real I2C bus that sticks in the middle of a run (only an injected hang); a GNSS
  receiver without a fix; a command sent over the radio (the station refuses all commands on UDP by design);
  the SD driver with any card but one SanDisk 64 GB; `marv-cli` compiled for the Pi's ARM processor (the
  target is not installed on this desktop).

**Small things noticed, not changed**
- `marv-cli devices` prints no flags column (the "self-test running" flag is only in the window).
- `MARV-V2-drivers/CONTRACT.md` still says eight drivers and names a 1.0 gate the owner set aside.
- An intermittent failure of one property test in `rc-port` (input not captured).
- Core 0 has no working stack guard: `_stack_end` resolves to a flash address (`memory.x` inserts `.end_block`
  after `.uninit`, ahead of cortex-m-rt's `PROVIDE(_stack_end = .)`), and embassy-rp's
  `install_core0_stack_guard` is not called. The 32 KiB floor of the link ASSERT is checked on `__euninit`.
- `python3 -m venv` does not work on this desktop (no ensurepip); `uv` does. A nightly Rust toolchain was
  installed by accident earlier (`rustup toolchain uninstall nightly` removes it).

## Rules that are easy to break

- **Nothing on core 1 may use a timer, a log call or a lock.** A timer scheduled from core 1 can silently kill
  a core-0 interrupt-executor task (the watchdog monitor, the radio port). The IMU tasks wait through
  `SpinDelay`. This is by convention; nothing enforces it at compile time.
- **The flight core runs from SRAM.** Everything core 1 runs after `core1_main`, every constant and core 1's
  vector table are copied to SRAM at boot (`.ram_text` in `memory.x`). `tools/run.sh` runs
  `tools/check-ram-code.py` before flashing and refuses to flash on a wrong result; a binary flashed another
  way skips that check. A new core-1 interrupt or flight crate needs a pattern line in `memory.x` and a
  passing checker (`docs/resources.md` rule 10).
- **`DMA_IRQ_0` belongs to core 1.** Core 0 builds no embassy-rp DMA channel. The SD card uses channel 4 on
  `DMA_IRQ_1` through its own driver code. Core 0 never read-modify-writes a DMA register shared by channels.
- **Flash:** the firmware writes only its settings region, through one guarded function; a save stalls both
  cores for about 45 ms. Keep the number of saves low (about 225 so far).
- **Bench:** the board through `tools/run.sh` only, one session at a time; battery unplugged, USB power,
  motors attached without propellers; nothing may drive GPIO36 to 39 or PIO2; never write settings to the
  receiver, the ESCs or the GNSS receiver.
- **This machine:** every test or build under a memory cap, every board run in its own detached scope
  (`_plan/WARNING-oom-tab-kills-20261115.md` has the commands). A runaway test killed whole sessions before
  this rule existed.
- **With the debugger attached, the debug logger can block both cores for tens of milliseconds** when its
  buffer fills. Figures taken under `probe-rs` include that; with no debugger it does not happen.

## How to run things

```
cargo host-test -p fw-core -p fw-settings -p fw-flight   # host tests (200 + 2 doc, 20, 20 today)
cargo build --release                                     # the firmware for the board
tools/run.sh 120                                          # build, flash, run, log to logs/ for 120 s
FEATURES=<feature> tools/run.sh 120                       # a test or measurement build (README lists them)
~/Documents/MARV-GCS/target/release/marv-cli --connect serial:/dev/ttyACM1 status | devices | param list ...
```

After `tools/run.sh` ends, the firmware keeps running on the board; the station and `marv-cli` reach it on
`/dev/ttyACM1`, and over the radio on UDP 14550 when the desktop is on the transmitter backpack's network and
the transmitter is in its MAVLink model.
