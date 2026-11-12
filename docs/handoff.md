# Handoff: drivers finished, firmware next

Written 2026-11-11 at the end of the driver work, for whoever starts the firmware (a new session or a
person). It says what exists, where it is, what was proven on the real board, what the firmware has to
respect, and what is still open. Start the next session in this folder.

Since then (2026-11-12): the link to the ground station exists. `docs/handoff-usb-mavlink.md` covers it and
answers the ground-station items listed as not decided in section 8.

## 1. Where everything is

| What | Local path | GitHub | Branch |
|---|---|---|---|
| Firmware (this repo) | `~/Documents/MARV-V2-firmware` | `Lmx154/MARV-FC` | `main` |
| Eight driver repos | `~/Documents/MARV-V2-drivers/<crate>` | `Lmx154/<crate>-rs` | `main` (local `to-1.0` is the same commit) |
| Bring-up test suite and the IMU clock evaluation | `~/Projects/Rust/MARV-V2` | none (not a git repo) | |
| PCB design (never modified by this work) | `~/Documents/kicad/MARV-V2` | | |

Local working files that are NOT in git (they stay on this machine):

- `docs/plan.md` in this repo: the full working plan and log. Its top holds the owner's instructions,
  quoted with dates; its end holds the state at each stopping point. Long (about 1900 lines); read the
  top sections and the last log entries.
- `~/Documents/MARV-V2-drivers/CONTRACT.md`: the rules the drivers were built to, with the
  amendments of 2026-11-11 (how the long IMU test counts a read pair, and why).
- `~/Documents/MARV-V2-drivers/_plan/`: scope drafts, datasheet text
  (`_plan/datasheets-text/*.txt`, searchable), backups of the plan and the contract.

Noticed, not changed: the older project in `~/Projects/Rust/MARV-FC` has the same GitHub URL as
its `origin`. The GitHub repository was empty when this repo was pushed to it.

## 2. This repository today

A skeleton. `src/main.rs` (34 lines) starts embassy, prints one public type of each driver to prove
that all eight link, prints `alive` and exits with a pass verdict. It touches no bus. It ran on the
board on 2026-11-11 with the drivers fetched from GitHub (`RESULT: PASS`).

- `Cargo.toml` takes the eight drivers from GitHub, each pinned to a commit (`rev`). `Cargo.lock` is
  committed. To take a newer driver, change its `rev` and run `cargo update -p <crate>`. To work on a
  driver and the firmware together, add a `[patch."https://github.com/Lmx154/<crate>-rs.git"]` entry
  with a local path instead of editing the dependency (the comment in `Cargo.toml` shows it).
- `tools/run.sh [timeout_seconds]` builds, flashes through the debug probe, runs, prints the report
  lines and returns the firmware's verdict as its exit status. It takes a machine-wide lock
  (`marv-v2-probe.lock`), so only one program uses the probe at a time. Logs go to `logs/` (ignored).
- `docs/pinout.md` is the pin map; `docs/devices.md` holds register and timing facts per device.

Toolchain proven here: Rust 1.99.1, target `thumbv8m.main-none-eabihf`, probe-rs 0.31.0, a CMSIS-DAP
debug probe on SWD header J10, embassy-rp 0.10.0 (`rp235xb`), embassy-executor 0.10.0, embassy-time
0.5.1, defmt 1.0.

## 3. The board (MARV V2 Rev 1, RP2354B at 150 MHz)

| Device | Driver crate | Bus and pins (GPIO) |
|---|---|---|
| ICM-45686, main IMU | `icm45686` | SPI0: SCK 18, MOSI 19, MISO 16, CS 20; INT1 15. Pin 9 of the chip is fed by a 32.000 kHz oscillator |
| LSM6DSK320X, second IMU: high-g accelerometer plus low-g accelerometer + gyroscope | `lsm6dsk320x` | SPI1: SCK 10, MOSI 11, MISO 8, CS 9; INT1 7, INT2 6 |
| BMP581 barometer | `bmp581` | I2C1: SDA 22, SCL 23; INT 21 |
| BMM350 magnetometer (added by the owner) | `bmm350` | I2C0: SDA 12, SCL 13; INT 4 |
| LC76G GPS (added by the owner) | `lc76g` | UART1: TX 24, RX 25, 115200 baud; PPS 5 |
| microSD, native 4-bit | `pio-sdmmc` | PIO: DAT0-3 28-31, CLK 32, CMD 33; no card-detect |
| 16 MiB PSRAM (added by the owner) | `qmi-psram` | QMI chip-select 1 on GPIO 0, shared QSPI pins |
| WS2812C status LED | `ws2812c` | GPIO 43 |

The full table, the spare pins, the ADC inputs (VBUS, VBAT, current), the servo and ESC outputs and
the connector labels are in `docs/pinout.md`.

## 4. The drivers: what each one is and what was proven

All eight are `no_std`, written against `embedded-hal` / `embedded-hal-async` 1.0 (the GPS against
`embedded-io` 0.7), version 0.1.0, MIT. Each has an async API and a `blocking` module with the same
methods. A driver owns no pin, timer or task: it configures the device's interrupt output and the
firmware waits on the pin. Settings are built with `Config::new().with_x(..)`. Methods return
`NotConfigured` until `init` has completed. Each README has a quick start, the rules for correct
use, and a "Measured on the board" section; read it before using the driver.

"Proven" below means: run on this board, with the log kept under `hil/logs/` of that repo.

| Crate | Pinned commit | Proven on the board |
|---|---|---|
| `icm45686` | `867093e` | Identity, 6-axis data at 6.4 kHz, FIFO, offsets, self-test, cancel then `recover()`, 30-minute run (7.7 million double reads at 6.4 kHz, 0 mismatches, 0 missed), every INT1 pin option. One `read()` takes 16.3 us at 18.75 MHz in SPI mode 3 |
| `lsm6dsk320x` | `07a8889` | Both functions together: every high-g range and rate, FIFO with high-g words, offsets, self-test, cancel then `recover()`, the high-accuracy rate mode, 30-minute run (see 5), high-g wake-up and shock events on real strikes |
| `bmp581` | `5046a13` | Identity, rates (238.4 Hz at the 240 Hz setting, 496 Hz continuous), FIFO with threshold and full interrupts, 1553 cancelled reads followed by `recover()` with 0 wrong reads, bus clear for a stuck SDA line |
| `bmm350` | `d233caa` | Identity, 100 and 400 Hz, forced mode, self-test, resets, 1858 cancelled reads followed by `recover()` with 0 wrong reads, bus clear |
| `lc76g` | `2ee54f9` | Version, 10 Hz fixes (300 fixes for 300 GGA + RMC pairs), PPS, commands, abandoned commands, every sentence of the stream decoded (3231 of 3231), ZDA / GNS / GST / GRS cross-checked |
| `pio-sdmmc` | `d078048` | Identification, single and multi-block read and write at 25 MHz, the block-device trait, about 800 cancelled operations per run with the driver cleaning up by itself and 0 wrong bytes |
| `qmi-psram` | `591ca04` | Chip ID, all 16 MiB written and verified at 75 MHz, retention, re-map, DMA into and out of the memory, 60 s under load with 0 mismatches |
| `ws2812c` | `831c70b` | Waveform against the datasheet on both transports, a cancelled send, CPU time per poll, colours confirmed by eye |

Six defects were found by running on the hardware and fixed on 2026-11-11: GPS late acknowledgement
taken for the next command's answer; GPS GNS decoder rejecting the layout the module really sends;
barometer FIFO never reported full in streaming mode; main IMU's 100 us interrupt pulse setting writing
the wrong register; second IMU's self-test step list able to drop a step silently; second IMU reporting
the wrong rates in the high-accuracy mode.

## 5. What the firmware has to respect (all measured on this board)

Bus transfers
- **Never cancel an SPI or I2C transfer in flight** (no timeout or `select` around a driver call that
  is on the bus). On embassy-rp 0.10 a dropped transfer is not aborted and the next reads return wrong
  data with no error. Put the timeout around the wait for the data-ready pin. If one is cancelled
  anyway, call the driver's `recover()`; `verify()` is the in-flight health check (it reads back the
  registers the configuration owns). A stuck I2C data line needs the bus-clear routine shown in
  `bmp581/hil/src/bin/bmp581_cancel.rs`.
- The GPS is the exception: every call may be wrapped in a timeout. The driver resynchronises by
  itself after an abandoned command (about 10 ms). The module answers a command in 4 to 93 ms.
- The SD driver may also have its futures dropped; it cleans up at its next operation.

Second IMU (`lsm6dsk320x`)
- **Read timing.** The output registers change, axis by axis, from 22 us to 3 us BEFORE the data-ready
  edge. A single-sample read (`read`, `read_high_g`, `read_all`) must start on the edge and return at
  least 25 us before the next one (within about 105 us at 7.68 kHz), or it can return axes of two
  samples with no error. A task that can be delayed by more than that should read through the FIFO.
  `read_all` (all three channels) takes 32 to 44 us at 9.375 MHz in SPI mode 3.
- With high-g at the same rate, one data-ready edge times all three channels.
- The high-g channel reads about 0.05 / -0.15 / 0.30 g (X / Y / Z) off the low-g one at rest on this
  chip. Calibrate it with the high-g offset registers (`set_offsets`).
- In normal mode the chip runs 1.8 % slow (7540 Hz at the 7680 setting). The high-accuracy rate mode
  gives rates within 0.3 % of the datasheet's table; `Info` reports the real rates in both modes.
- High-g events: each strike gives a wake-up + shock event and a second shock event about 80 ms later.
  The shock flag lasts one sample (130 us): use the interrupt pin, not polling.
- The 30-minute run: one `read_all` per sample at 7.68 kHz, 4 524 084 reads, 0 missed; pairs at
  3.84 kHz, 2 262 056 pairs, 0 mismatches; FIFO with high-g words, 1 802 612 words, all intervals
  exactly 8 words per channel. Pairs of `read_all` do not fit at 7.68 kHz, so bit-exactness was
  checked at 3.84 kHz and through the FIFO.

Main IMU (`icm45686`)
- The internal oscillator gave 200.895 Hz for a nominal 200 Hz. On the external 32 kHz clock the rate
  is exact up to 3.2 kHz, and 6388.7 Hz at the 6.4 kHz setting. The owner decided, on those
  measurements, to replace the oscillator with an MCU pin on the next board revision.
- `PulseWidth::Us100` is only valid below 4 kHz (`Config::validate` rejects it at 6.4 kHz).
- Gyroscope ranges other than +-250 dps were checked against the datasheet only (they cannot be
  verified with the board at rest): sanity-check the range you choose by rotating the board.

Barometer (`bmp581`)
- In streaming mode the FIFO count stops at 15 frames (31 for single-quantity frames), not 16 / 32.
  `fifo_read` reports `full` there; the exact indicator is `interrupt_status().fifo_full`.

SD card (`pio-sdmmc`)
- Speeds on the one card tested (SanDisk 64 GB): reads 9.1 MB/s and writes about 7.5 MB/s in 256-block
  calls; 8-block writes 2.9 MB/s; single-block writes 0.4 MB/s. Write in large calls.
- **The card stalls a single write call for 30 to 132 ms now and then.** A logger needs a buffer that
  rides through that.
- The driver does not retry: after its one clean-up it returns the error. Retrying is the firmware's
  choice. There is no card-detect.
- The driver has no filesystem. Records can be written straight to blocks, or a FAT library can sit on
  the optional block-device trait (feature `block-device-driver-0-2`).
- The driver's own tests overwrite blocks 50000000 to 50002047, and `sd_bus` overwrites blocks at
  random addresses: the card in the socket is a scratch card. Do not keep data you need on it while
  running driver tests.

PSRAM (`qmi-psram`)
- 16 MiB at 0x15000000 (uncached; 21.0 MiB/s write, 23.7 MiB/s read; DMA 31 and 26 MiB/s).
- `init` and `remap` must not run while the second core executes from flash. Programming the flash
  resets the QMI set-up: call `remap()` afterwards. Build with optimisation, or the chip-select pulse
  exceeds the chip's limit (README).

Everything
- A `defmt` log line is written with interrupts masked. One line from another task delayed an
  interrupt-level sensor task by about 80 us. Keep logging off the sampling path.
- The same code ran at different speeds in different builds (a `read_all` of 37 us in one build, 53 to
  57 us in another), which points at where the code lands in flash. Do not design with tight timing
  margins.
- Sampling at 6.4 and 7.68 kHz was done from an interrupt executor and from the second core with no
  sample missed.

## 6. How to work on a driver

```
cd ~/Documents/MARV-V2-drivers/<crate>
cargo test --locked                    # host tests against a simulated bus
tools/run.sh <program> [timeout_s]     # one on-board program; the README lists them
```

- All driver repos under that folder build into one shared folder (`.cargo/config.toml` there). Before
  a run whose result counts: `touch src/lib.rs hil/src/lib.rs hil/src/bin/<program>.rs`.
- Never run `spi_cancel` in `icm45686`: it disabled the IMU once and only a power cycle brought it back.
- Long runs: `SOAK_SECONDS=1800 tools/run.sh <crate>_soak 2100` (31 minutes; nothing else may use the
  probe). Tests that need a person: `ws2812c`: `tools/run.sh ws2812c_colours 600`; `lsm6dsk320x`:
  `tools/shock.sh` (terminal banners and the LED say when to strike and when to stop).
- After a driver change: host tests, the on-board programs it touches, commit, push, then change the
  `rev` in this repo's `Cargo.toml`.
- Do not run a blanket `cargo fmt` in a driver's `hil/` folder: several committed programs are not
  rustfmt-clean and it rewrites them. Format only the files you changed.

## 7. Not done, and known limits

- No 1.0: every crate is 0.1.0, with no tags, no API-freeze tests, no changelog. The owner set that
  paperwork aside.
- One SD card tested; none without CMD23. A cancel inside an SD command frame is proven on the
  simulated card only. No automatic retry, no long SD run.
- Not run on the board: the second IMU's on-change wake-up mode and the wake-up as a FIFO trigger; the
  main IMU's filter bandwidths, other accelerometer ranges, RTC_ALIGN and SPI slew; the barometer's IIR
  options and out-of-range event; the magnetometer's rarely used settings (single axes, watchdog, rates
  of 25 Hz and below, 200 Hz). All are host-tested against the datasheet.
- No 30-minute runs for the barometer, magnetometer, SD card, GPS or PSRAM (the owner asked for a long
  run per IMU only).
- Battery input, current sense with a real source, ESC and servo outputs under load, the ELRS port:
  never exercised (nothing attached).
- This repo depends on public GitHub repositories over https; a build on a new machine needs network
  access once.

## 8. What comes next: the firmware

The owner's goal for this repository, in the owner's words (2026-11-09):

> I want to make a simple data acquisition system for phase one of the MARV V2 project. the goal is to
> make a flight control system, but i think the lowest hanging fruit is the ground control software and
> the parameter system and logging system.

and on the drivers (2026-11-10):

> the pcb is its own repo that will never change. what is being called into question is the drivers
> being combined with the firmware. i think this is bad. ideally the drivers do everything from day one,
> they are tested rigorously and then never touched unless absolutely necessary.

Already settled:
- Drivers stay out of this repo and are used at pinned commits.
- No logging layer inside the SD driver. For a minimal logger a filesystem is not required; both
  routes (raw records on blocks, or FAT on the block-device trait) are open and the choice is the
  firmware's.
- The owner's standing instruction on 2026-11-11: commit and push accepted work when it is ready; do not
  over-engineer.

Not decided yet (ask the owner at the start; none of these was asked):
- What phase one records: which sensors, at which rates, and whether the high-g channel is logged
  continuously or only around events.
- The log format and where it lives: raw blocks or a PC-readable filesystem; how logs are taken off
  the board (card reader, USB, a link to the ground software).
- What the ground control software and the parameter system have to do first, and over which link.
- How the two cores and the interrupt executor are used (a measured starting point is in section 5).
- Whether the card in the socket stays a scratch card.

A first step that needs no decision: replace the skeleton `main` with one task per device that
initialises its driver on the real pins and reports the data rate, reusing the set-up code of each
driver's `hil/src/bin/*_smoke.rs`.
