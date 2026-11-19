# MARV V2 firmware: hand-off

State on 2026-11-18; written first on 2026-11-15 and brought up to date since. Written for whoever picks the work up next, human or agent. It says what
is built and proven, what is in progress, and what is left. It is an overview: the details and the evidence
are in the documents it points to.

## Where everything is

| Thing | Place | State |
|---|---|---|
| Firmware | `~/Documents/MARV-V2-firmware` (GitHub `Lmx154/MARV-FC`) | `main` is current and pushed; the only branch is `main` (the orientation build is merged), and the tag `rollback-20261117` marks the state before it |
| Driver crates, one repo each | `~/Documents/MARV-V2-drivers/<crate>` | every repo on `main`, equal to its remote |
| Ground station and `marv-cli` | `~/Documents/MARV-GCS` | `main`, pushed; the station's log work (list and download) is merged there |
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
- `docs/handoff-esc-link.md`: the ESC link work; in the firmware so far only the DShot telemetry stream (README, ESC telemetry).

## What the owner asked for

The foundation, not the flight loop: every device always on at the lowest layer with health and diagnostics;
settings as MAVLink parameters; actions (restart, self-test, calibrations, save, reboot); watchdogs; the ground
station as display and controller on any link and as configurator on serial or TCP only; a companion computer
through a command-line tool; the two-core split. His standard for what counts as good enough is ArduPilot, PX4
and Betaflight, nothing stricter, except where the two-core design has no precedent.

## Done and proven on the board

- **Two cores.** Core 1 is the flight core: both IMUs (configured once at boot from `IMUn_ACC_FS`, `IMUn_GYR_FS`, `IMUn_ODR_HZ`; default 16 g, 2000 dps, 800 Hz) and the ESC task (DShot300 at 1 kHz), nothing else. Core 0 does everything else.
  Data crosses only through lock-free cells.
- **Seven device rows:** IMU1, IMU2, BARO, MAG, GNSS, BATT (supervised: health states, recovery ladder,
  operator restart) and RC (report only: "the radio link is up"). Also SD (supervised) and ESC (device 12, report
  only: how many of the four ESCs answer).
- **ESC telemetry:** run on the board with the battery (2026-11-18, 4cfb1f0 and 0d25909, logs `20261118-191955` to
  `20261118-194637`: all four online, EDT voltage and temperature, rpm 0, 1000 exchanges/s, IMU rates unchanged;
  README, ESC telemetry, "Measured on the board") (`src/esc.rs`): core 1 sends DShot300 Stop frames at 1 kHz on
  GPIO36-39 from every boot (command 13 in EDT bursts), the ESC row, ESC_STATUS and ESC_INFO on
  USB. The station's Motors tab shows it. The props-off motor test is built (command 209, one motor at up to 5 % for up
  to 3 s, core 1 ends the spin on its own clock; README, ESC telemetry, Motor test) and was run on the board from the
  station's Motors tab on 2026-11-18 (all four motors, stop, replace, cable pull).
- **Parameters:** 80 (the last ten: the IMUs' six chip settings, `Reboot` class: they apply after save and reboot; then `AHRS_ORIENTATION` (`Reboot`), `MAG_ORIENT` (`Live`), `MOT_POLES` (`Live`, the motors' poles for the ESC rpm) and `LED_BRIGHT` (`Live`, the status LED's level 0 to 3, default 1); range-checked when written, the chip's value set checked at boot), with ranges and per-device validation. A write takes effect in RAM; flash is written
  only on the user's Save. Reload, reset, load at boot. Survives re-flash and a real power cycle.
- **Status LED:** built and host-tested, NOT yet run on the board (`src/led.rs`, PIO1 SM1, GPIO43): white at boot, a nine-blink roll call at 6 s (IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, ESC: green OK, red FAILED, yellow other), then a one-second pulse whose colour is the state (red double blink: a vital sensor FAILED; purple: motor spin; blue/white: calibrating; yellow: degraded or no log; green: GNSS 3D fix; blue: otherwise); three green or red flashes when a save or calibration ends. README, Status LED, has the table.
- **Actions:** device restart; self-test of MAG, IMU1, IMU2; gyro, level-accelerometer and compass
  calibration (results applied, never saved by themselves); storage save/reload/reset; reboot.
- **Watchdog:** resets when either core stops answering, keeps a record of why, reports it at the next boot.
- **Radio (ELRS):** both receiver modes. CRSF model: sticks at about 245 per second, battery telemetry shown
  on the transmitter; also a GPS frame (position, speed, heading, altitude, satellites) per new GNSS sample, 1 Hz, in place of that call's battery frame (host-tested; the owner saw the GPS telemetry on the Pocket on 2026-11-18). MAVLink model: telemetry over the air (about 134 B/s; now 270 B/s with HIGHRES_IMU, SCALED_IMU2 and SCALED_PRESSURE at 1 Hz, counted at the UART, not yet seen on the air). Mode switches both ways while
  running; transmitter off and on in each mode. The owner uses the CRSF model for sticks and the MAVLink model
  for telemetry only.
- **I2C faults:** both buses are cleared at every boot; a hung transfer times out (ArduPilot's formula) and
  the bus is cleared and rebuilt by the normal recovery ladder.
- **Ground station:** Devices view, parameter editing with Save, command buttons, the configure/operate gate.
  `marv-cli` covers the same from a shell; a full companion session over USB passed on this desktop.
- **RC mapping, reported only:** 44 RC parameters (`RCn_MIN/TRIM/MAX/REVERSED/DZ` for
  channels 1 to 8, `RCMAP_ROLL/PITCH/THROTTLE/YAW`, all `Live`) and MANUAL_CONTROL (69) on USB at 5 Hz (fastest 50 Hz) from the
  CRSF sticks only, while the latest stick report is younger than 1 s (a MAVLink override never feeds it). Nothing reads the values
  except the encoder: no arming, failsafe, mode or output. Computed in the USB task (`rc::manual_control`), not in the RC hook and not
  on core 1. Seen on the board on 2026-11-16 (transmitter in its CRSF model): 300 MANUAL_CONTROL in 60 s, RC_CHANNELS still 5 Hz; each
  MANUAL_CONTROL equal to the formulas applied to the nearest RC_CHANNELS; `param set RCMAP_ROLL 3` and `RC1_TRIM 1400` took effect
  at once, with no reboot and no save; `RC1_MIN 700` refused with "RC1_MIN: out of range"; 0 RC UART overruns in 320 s (also 0 on
  `main` in 320 s). Not yet seen: the transmitter's MAVLink model giving no MANUAL_CONTROL (checked with the owner), and the
  station side (Radio tab, calibration).

### Sensor configurations on the board (B-a, B-b)

The station's sensor presets (a station feature: each preset is several of these parameters set at once, a verified combination; nothing of
them is in the firmware) use the combinations below. They were measured on 2026-11-17 with an earlier build that set them through preset selectors (removed
since; the chip settings, the driver configurations and the code on core 1 are the same as in `main`), debugger attached, USB power,
battery out, motors on, no propellers; each configuration is named here by its values. BARO and MAG fields were set with
`marv-cli`, no save. Rate: samples in the `[STAT]` lines over the window shown (the first, partial line after the change left out),
against the chip's rate. Errors, timeouts, missed edges, overflows, recoveries and config restored: 0 in every line. Noise (a measurement, not a
pass/fail): the bench at rest, from a 66 s `marv-cli record` of the same window (first 6 s dropped), SCALED_PRESSURE and HIGHRES_IMU
xmag/ymag/zmag, both at 25 messages per second (the stream's own rate, so BARO 50/100 Hz and MAG 100/200 Hz are sampled, not averaged). Pressure sd
is after a 1 s moving mean is removed (the raw sd over 60 s is mostly drift); field sd likewise, per axis.

| BARO configuration (ODR, pressure OSR code, temperature OSR code, IIR P/T) | Configuration line | Rate (window) | vs chip | Pressure sd (raw), model |
|---|---|---|---|---|
| 50 Hz, 4, 1, IIR 0/0 | 50.056 Hz, P code 4, T code 1, IIR 0/0 | 49.83/s (60 s) | -0.45 % | 0.179 Pa (1.04), 0.21 |
| 100 Hz, 3, 0, IIR 0/0 | 100.299 Hz, 3, 0, 0/0 | 99.78/s (50 s) | -0.52 % | 0.250 Pa (0.38), 0.30 |
| 25 Hz, 5, 1, IIR P code 2 (coefficient 3) | 25.005 Hz, 5, 1, IIR P code 2 | 24.88/s (50 s) | -0.50 % | 0.058 Pa (1.11), 0.057 |
| 10 Hz, 7, 3, IIR 0/0 | 10.0 Hz, 7, 3, 0/0 | 9.975/s (40 s) | -0.25 % | 0.089 Pa (0.68), 0.08 |

| MAG configuration (`MAG_ODR_HZ`, `MAG_AVG`) | Configuration line | Rate (window) | vs chip | Field sd x/y/z (raw), model x/y and z |
|---|---|---|---|---|
| 100 Hz, averaging 2 (code 1) | 100.0 Hz, Avg2 | 100.15/s (60 s) | +0.15 % | 148/150/338 nT (159/166/458), 190 and 450 |
| 200 Hz, averaging 2 | 200.0 Hz, Avg2 | 200.30/s (50 s) | +0.15 % | 154/154/367 nT (171/160/408), 190 and 450 |
| 100 Hz, averaging 4 (code 2) | 100.0 Hz, Avg4 | 100.14/s (50 s) | +0.14 % | 125/126/307 nT (144/146/387), 134 and 318 |
| 50 Hz, averaging 8 (code 3) | 50.0 Hz, Avg8 | 50.08/s (50 s) | +0.16 % | 104/104/292 nT (119/121/364), 95 and 225 |

BARO: the 50 Hz configuration over the 100 Hz one is 0.72 against the model's 0.70; the 25 Hz and 10 Hz ones are at the model. MAG: every configuration is at or
below the model, but the fall with the averaging is slower than 1/sqrt(N): averaging 4 over 2 is 0.84 (model 0.71), 8 over
2 is 0.70 (model 0.50); z is flatter still (0.91, 0.86). Not explained (a floor from the bench's field or the 25/s
sampling are candidates); the model holds as a bound, not as a prediction. The windows of the last three BARO rows are shorter than
60 s because the 10 s lines fall between the 69 s steps. The BARO and MAG fields are `Configure` class: a write applies at once (about 50 ms), as seen in these runs.

IMU configurations. Both IMUs get the same combination except where the chip differs (IMU2's accelerometer stops at 16 g and its gyroscope starts at 250 dps). Each was set with
`param set IMUn_ACC_FS`, `IMUn_GYR_FS` and `IMUn_ODR_HZ` (the firmware answers "<NAME>: applies after reboot"), `param save`, reboot. Four saves in all (b to e, on that earlier build);
`SAVE_EPOCH` is not a parameter on this firmware, the save line's "epoch 1" stayed 1. A write alone changes nothing on the chip: with the fields at 8 g / 1000 dps / 400 Hz in RAM (written through that
build's selectors), 120 to 150 s of the run read 803.5/800.7 per second and the gyro step was unchanged, also after `device restart IMU1`
(the line holding the restart read 798.6/s, the 60 ms of the restart): the restart does not apply a new setting, only the next boot does.

Reboot 246: `marv-cli reboot --yes` printed "reboot: accepted", exit 0; the firmware logged "final ACK ... handed to the link,
reset due at 223245 ms" (100 ms after the ACK). probe-rs then ended with "Firmware exited unexpectedly: Exception" (no RTT
after it). The board came back by itself: `/dev/ttyACM1` was there again 15 s after the command; the SD log of that boot had the new steps. The boot lines
themselves go to defmt only, so they were read from a new `tools/run.sh` started after it (it builds, flashes the same ELF and resets; the stored settings are what the 246 boot had loaded).
For the other runs the reboot was that new `tools/run.sh` (kill of the old probe-rs, new session), not 246.

Saves on that earlier build (the "settings save" line, then the next boot's "settings boot" line): b: sector B, sequence 238, 8 entries; c: sector A,
239, 6 entries; d: sector B, 240, 5 entries; e: sector A, 241, 0 entries (every field at its default, the record is empty).
The count is the entries that differ from the default (800 Hz, 16 g and 2000 dps are defaults, so they are not stored).

Per configuration, 120 s (the 10 s to 130 s lines of `[STAT]`, 120 s of samples; the first line, which holds the start, is in the
per-10 s minimum only: 798.8 and 795.4). Errors, timeouts, missed edges, overflows, recoveries, config restored: 0 in every line
of every run; gaps over 1.5 P: 0; delay count after the start 0; watchdog stalls begun 0. The boot line is the
`[INFO] IMUn: <g> g, <dps> dps, <hz> Hz (until the next reboot)` of the run (on that earlier build the same line also carried a preset id and name).

| Configuration (IMU1 / IMU2) | Rate IMU1 / IMU2 (vs chip) | \|a\| IMU1 / IMU2 (vs 9.81) | Gyro step IMU1 / IMU2 (LSB) |
|---|---|---|---|
| 8 g, 1000 dps, 400 Hz both | 401.78 / 400.35 per s (+0.45 % / +0.09 %) | 9.870 / 9.630 (+0.6 % / -1.8 %) | 0.0305 / 0.0350 dps (0.0305 / 0.035) |
| 4 g, 250 dps, 800 Hz / 4 g, 500 dps, 800 Hz | 803.56 / 800.71 (+0.45 % / +0.09 %) | 9.867 / 9.638 (+0.6 % / -1.8 %) | 0.00763 / 0.0175 dps (0.0076 / 0.0175) |
| 32 g, 4000 dps, 800 Hz / 16 g, 4000 dps, 800 Hz | 803.56 / 800.71 (same) | 9.859 / 9.641 (+0.5 % / -1.7 %) | 0.1221 / 0.1400 dps (0.122 / 0.14) |
| 16 g, 2000 dps, 800 Hz both (the defaults; 600 s, below) | 803.56 / 800.71 | 9.868 / 9.637 (+0.6 % / -1.8 %) | 0.0610 / 0.0700 dps (0.061 / 0.07) |

Step: the figures are the smallest non-zero step of consecutive IMU records (GyrX, GyrY, GyrZ
agree on all three axes), 2750 records per IMU per 110 s window (14750 in the 600 s). Rates are the firmware's `[STAT]`; |a| is
the firmware's `|accel|` from the flight core line, averaged over the window (the SD log's |a| agrees to 0.01). IMU2 reads
9.63 at every range, as before; the chip is not judged here.

Gyro at rest, mean of the firmware's `[STAT] IMUn mean of 1000 samples` lines (sd in mrad/s, per axis x/y/z, then in dps and
over the design's Low noise model in dps; mean in mrad/s, calibration offsets all 0, so these are the raw zero-rate readings):

| IMU, configuration | sd x/y/z (mrad/s) | sd x/y/z (dps) | model | sd / model x, y, z | mean x/y/z (mrad/s) |
|---|---|---|---|---|---|
| IMU1 16 g, 2000 dps, 800 Hz (600 s) | 1.382 / 1.396 / 2.057 | 0.079 / 0.080 / 0.118 | 0.0780 | 1.01, 1.03, 1.51 | 5.05 / -2.42 / 2.02 |
| IMU1 4 g, 250 dps, 800 Hz | 1.430 / 1.404 / 1.996 | 0.082 / 0.080 / 0.114 | 0.0760 | 1.08, 1.06, 1.50 | 4.98 / -2.37 / 2.00 |
| IMU1 32 g, 4000 dps, 800 Hz | 1.472 / 1.471 / 2.140 | 0.084 / 0.084 / 0.123 | 0.0838 | 1.01, 1.01, 1.46 | 5.05 / -2.43 / 1.98 |
| IMU1 8 g, 1000 dps, 400 Hz | 1.048 / 1.009 / 1.445 | 0.060 / 0.058 / 0.083 | 0.0545 | 1.10, 1.06, 1.52 | 5.02 / -2.45 / 2.01 |
| IMU2 16 g, 2000 dps, 800 Hz (600 s) | 1.014 / 0.978 / 1.932 | 0.058 / 0.056 / 0.111 | 0.0786 | 0.74, 0.71, 1.41 | 3.07 / -3.29 / -3.10 |
| IMU2 4 g, 500 dps, 800 Hz | 1.006 / 0.999 / 1.961 | 0.058 / 0.057 / 0.112 | 0.0762 | 0.76, 0.75, 1.48 | 3.07 / -3.28 / -3.12 |
| IMU2 16 g, 4000 dps, 800 Hz | 1.192 / 1.216 / 2.046 | 0.068 / 0.070 / 0.117 | 0.0861 | 0.79, 0.81, 1.36 | 3.02 / -3.24 / -3.10 |
| IMU2 8 g, 1000 dps, 400 Hz | 0.822 / 0.762 / 1.302 | 0.047 / 0.044 / 0.075 | 0.0547 | 0.86, 0.80, 1.36 | 3.06 / -3.28 / -3.13 |

Noise against the model (expected at or below): NOT met as a bound. IMU1's x and y are at the model (1.01
to 1.10 times it) in every configuration, z is about 1.5 times; IMU2's x and y are below it (0.71 to 0.86), z about 1.4 times. Both
chips have z 1.4 to 1.5 times x and y in every configuration, the same on two chips, so the excess in z is probably the bench or the
board (motors attached, not a per-chip noise) rather than the chips' noise; not isolated here. The fall from 800 to 400 Hz
is 0.76 (IMU1) and 0.81 (IMU2) on x, against the model's 0.70 and 0.70, so the dependence on bandwidth is as modelled; the
range changes the sd little (2000 dps over 4000 dps: 0.94 and 0.85, the model's 0.93 and 0.91). Offsets carry: the
means do not move with the configuration beyond the scatter between runs of one configuration: IMU1's spread over the four is 0.07 /
0.08 / 0.04 mrad/s, IMU2's 0.05 / 0.05 / 0.03 (the first boot at the defaults of that session, 10 min before the 600 s one, read IMU2 y
-3.37 against -3.29), against about 0.05 mrad/s for the mean of 1000 samples (sd 1.0 to 2.0 over the square root of 1000);
the 10 s means move by +-0.1 within a run. So no range-dependent offset is visible at this resolution, and no "recalibrate after a change of
range" note is needed.

Defaults (16 g, 2000 dps, 800 Hz), 600 s (log `marv-v2-firmware-20261117-141123.log`, SD log 53, the lines of 10 s to 610 s): rows Ok in all 61 lines,
0 errors, timeouts, missed edges, overflows, recoveries and config restored, delay count after the start 0, watchdog stalls
begun 0. IMU1 803.56 samples/s (482135 samples in 600 s; 803.48 mean of the 10 s lines), largest
gap 1697 us, 0 gaps over 1.5 P, verify steps 18 to 276 us (20422), 618 passes; IMU2 800.71 samples/s (480426), largest gap
1781 us, 0 gaps over 1.5 P, steps 13 to 138 us (19535), 610 passes. The 10-minute figures in the README before this were taken
on 2026-11-15 at 4 g / 250 dps on an older firmware (110 and 1066 gaps over 1.5 P, largest gaps 2442 and 3410 us); the
difference is not attributed here (the flight core has moved to SRAM since, the configuration is not the only change). Seen and left: one
`[WARN] Overrun error (embassy_rp src/uart/buffered.rs)` at about 0.06 s of every boot, before any device row.

On the IMU-settings build now in `main` (2026-11-17, three `tools/run.sh` sessions, logs `marv-v2-firmware-20261117-160121.log`, `-160447.log`, `-160812.log`; the record was empty before, sector A sequence 241): the boot line reads
`[INFO] IMU1: 16 g, 2000 dps, 800 Hz (until the next reboot)` and the same for IMU2; the RAM-code check passes; at the defaults IMU1 803.5 to 803.6, IMU2 800.7, BARO 49.8 and MAG 100.1 per second, 0 errors,
timeouts, missed edges, overflows and recoveries, no ERROR or panic. `marv-cli --debug param set IMU1_ODR_HZ 400` answered "IMU1_ODR_HZ: applies after reboot" and the rate stayed 803.5 (80 s to 100 s lines);
`param save` (settings save: sector B, sequence 242, 1 entry, 44 bytes, stall 44 164 us); after the restart the boot line read 400 Hz and IMU1 ran at 401.79 to 401.71 per second (IMU2 800.7, 0 errors, also at 170 s); set back to 800 and saved
(sector A, sequence 243, 0 entries, stall 40 772 us); after the restart the line read 800 Hz, IMU1 803.6, IMU2 800.7, and the record had 0 entries. The firmware's range check refuses nothing at these values;
a value in range that the chip does not offer (for example 300 Hz) is stored and saved, and puts that IMU back to its defaults at the next boot with a WARNING (host-tested, not seen on the board).

## In progress: SD card logging

Design decided (`_plan/decision-sd-logging-20261115.md`): the SD driver completes its DMA channel on
`DMA_IRQ_1` on core 0 (no HAL fork); FAT32 through `embedded-fatfs` (approved by the owner); logs in
ArduPilot's `.BIN` format, from boot, one file per boot, about 5.6 kB/s; a 32 KiB ring that drops and counts
when the card stalls; card reader first, then MAVLink log download over USB.

| Step | What | State |
|---|---|---|
| P2 | DMA on the second interrupt line, on the board | done: a probe build, measured on 2026-11-15 and then removed (DMA channel 4 on `DMA_IRQ_1` on core 0, yielding between copies, about 5.5 s from 30.5 s: no stall, no reset, stalls begun 0) |
| SD1 | The driver's new DMA back-end | done, reviewed, `pio-sdmmc` `main` = `424fd58` |
| P1 | The `.BIN` encoder (`fw_core::dflog`), read back by ArduPilot's `mavlogdump.py` | done, in `main` |
| P3 | Format and mount the card inside the firmware, 16 MiB write and read-back | done, in `main` (the probes stay as features) |
| SD2 | The SD card as a device row (mount, capacity, health) | done, merged into `main` (README "SD card") |
| SD3 | The log writer and what is logged | done, in `main` (README "Logging"): 30-minute run with 3 saves, an 8.5-hour soak, both read back on the board with 0 bad bytes; RESTART, the write-fail injection and the clean-up seen. A real card removal seen (Failed 5.5 s after the pull, restart Ok in 329 ms), and three logs read on a PC with pymavlink 2.4.50 `mavlogdump.py` (no bad bytes). A power cut passed (owner, 2026-11-18). Not yet: UAV Log Viewer |
| SD4 | Log list and download over USB (`marv-cli logs list/get`) | done, in `main` (README "Logging", "Logs over USB"): list, download (logs 8 and 13 byte-identical to the card-reader copies, twice), absent log, end marker, take-over, END and the 3 s HEARTBEAT rule seen on the board with a throw-away pymavlink client; the station's half is in MARV-GCS `main`; on 2026-11-18 the owner's USB drop in the middle of a download (the station's fix) passed |

The card in the slot is a scratch card and may be erased (owner's word). The owner has a USB card reader:
logs copied off the card with it were read with ArduPilot's reader (2026-11-16) and serve as reference
copies for the download checks.

## Left open

**Sensor orientation (done, in `main`)**
- The magnetometer is mounted +X right, +Y forward, +Z up; the IMUs +X forward, +Y left, +Z up. Design:
  `_plan/decision-orientation-20261117.md`. Every output is body FRD. Owner's hands-on results, 2026-11-18: the IMU and
  compass sign checks passed on both IMUs (moved by hand); gyroscope, accelerometer and compass were calibrated and the
  offsets SAVED to flash (kept after a reboot); the RC calibration was saved; the radio checks passed (Pocket GPS
  telemetry, transmitter off, the MAVLink model over UDP); the station's presets BARO and IMU1 were checked. The board
  therefore holds calibrated offsets now; the measurements below were taken before, with all offsets 0.
  - **Orientation on the board, still vehicle** (2026-11-17, the build that became `main`, then at `05ae38d`, tree clean, four `tools/run.sh` sessions,
    logs `marv-v2-firmware-20261117-222029.log`, `-222449.log`, `-222634.log`, `-222819.log`; vehicle still on the bench, USB
    power, battery out, motors on, no propellers; the stored offsets are all 0; no calibration). B-0: the RAM-code check
    printed `PASS: nothing the flight core can reach after core1_main lies in XIP (0x10000000-0x1FFFFFFF)`; boot
    `AHRS_ORIENTATION 0, MAG_ORIENT 10`, both IMU lines `16 g, 2000 dps, 800 Hz`, no "settings refused", no ERROR, no panic;
    `param get` gave 0 and 10; 20 s to 70 s of `[STAT]`: IMU1 803.54/s, IMU2 800.70/s, BARO 49.8/s, MAG 100.1/s, 0 errors,
    timeouts, missed edges, overflows and recoveries. B-1 (level, at rest; the bench is not exactly level): IMU1 accel mean
    (x, y, z) (-0.193, -0.028, -9.865) m/s^2, IMU2 (-0.182, +0.071, -9.632) (SCALED_IMU2 zacc -981.6 mG); gyro means IMU1
    (+4.9, +2.5, -2.1) mrad/s, IMU2 (+3.0, +3.3, +3.1) (chip bias: the offsets are 0). Magnetometer (HIGHRES_IMU, gauss)
    (+0.036, +0.180, +0.391), |B| 0.432 G (43.2 uT). B-M (no save; `MAG_OFF*` read 0; 15 s each, 375 messages; means in gauss;
    10, then 0, then 10 again): 10a (+0.0360, +0.1802, +0.3918), 0 (+0.1794, +0.0346, -0.3886), 10b (+0.0350, +0.1801,
    +0.3902); the mean of the two 10s minus (y_0, x_0, -z_0) = (+0.0009, +0.0008, +0.0024), 0.2 % to 0.6 % of |B|; the two 10s
    differ by (0.0010, 0.0000, 0.0016): the captures drift by about that much, more than the noise of their means. B-A (2 saves):
    `AHRS_ORIENTATION 8` answered "applies after reboot"; save 1 `settings save: sector B, sequence 244, 1 entries, 44 bytes,
    stall 42427 us, verify ok`; after the restart the boot line read `AHRS_ORIENTATION 8` and zacc was +9.866 (IMU1) and +9.632
    (IMU2), the accel and gyro y and z flipped, x unchanged, the magnetometer unchanged; back to 0, save 2 `sector A, sequence 245,
    0 entries, 20 bytes, stall 39917 us, verify ok`; after the restart `AHRS_ORIENTATION 0`, zacc -9.863 (IMU1) and -9.630 (IMU2),
    the record 0 entries (sector A sequence 245, sector B 244). One bump of the bench during the capture at 8 (one 25-message group,
    zacc up to 11.9) is in its means. Seen once, on the boot after save 1 only: about 200 `Framing error` and 10 `Break error` WARN
    lines from `embassy_rp src/uart/buffered.rs` between 5 ms and 260 ms, none in the other three boots (not attributed). The hands-on
    steps that followed are above.

**Not built**
- ESC, beyond the telemetry and the props-off motor test above: arming, any other throttle, passthrough through the USB port, the board's
  current scale, ESC serial telemetry on GPIO34. See `docs/handoff-esc-link.md`.
- The estimator (next).
- The PSRAM. The 16 MiB PSRAM is fitted (hand-fitted in U24) and its driver was measured on the board, but
  the firmware does not use it yet. It can hold data (not code: it shares the flash cache).
- Full-rate IMU data from the flight core to core 0 (a ring is reserved in the design; needed for a control
  loop and for full-rate logging).

**Known and parked**
- Late IMU samples: with the flight core in SRAM a 200 s run of the normal build shows
  0 samples later than 1.5 P per 10 s window for both IMUs (before: 6 to 20 and 23 to 53), largest gaps 1.6
  and 1.7 ms, rates within 1 %, no errors (2026-11-16, debugger attached; not repeated without one). The
  shared XIP cache was their main cause (README "Flight core in SRAM"; facts-sd-logging C3). Its measurement
  builds were removed once their results were recorded here.
- (Closed 2026-11-18) The ~0.1 receive errors/s on the radio UART (2026-11-15, older build) do not reproduce on
  today's build: 0 new RC errors in 320 s of CRSF lock (log 20261118-172835).

**UART errors at boot (explained 2026-11-18, logs 20261118-162319 to -174800)**
- The ~0.06 s RC `Overrun error` and the rare boots with ~200 framing / 10 break: a debugger artefact. probe-rs puts
  the defmt RTT channel in block-if-full mode; once boot logging filled the 1 KiB buffer, defmt-rtt spun in its
  critical section with core-0 interrupts off (6 to 117 ms) and UART0's 32-byte FIFO overflowed (and rc-port missed
  its MAVLink lock window). Fixed: `DEFMT_RTT_BUFFER_SIZE = 8192` (.cargo/config.toml; +7 KiB RAM). 0 overruns in
  9 boots on both models after; peak fill ~2 KiB. Without a debugger RTT never blocks.
- Framing errors left at boot are expected and counted: UART0's first 50 ms search window listens at 460800 to a
  420000 CRSF stream (~130 framing); UART1 joins the LC76G mid-way through its NMEA burst after a reset of the MCU
  alone (~12 % of resets), counted as "errors before the first sentence" (accepted, as ArduPilot/Betaflight do).
- Divisors are exact enough: 115207 (+0.006 %), 460829 (+0.006 %), 419874 (-0.030 %).

**Never exercised on real hardware**
- A failing self-test; a real I2C bus that sticks in the middle of a run (only an injected hang); a GNSS
  receiver without a fix; a command sent over the radio (the station refuses all commands on UDP by design);
  the SD driver with any card but one SanDisk 64 GB; `marv-cli` compiled for the Pi's ARM processor (the
  target is not installed on this desktop).

**Small things noticed, not changed**
- (Fixed on branch `stack-guard`.) Core 0's stack guard: `_stack_end` used to resolve to a flash address
  (`memory.x` inserts `.end_block` after `.uninit`, ahead of cortex-m-rt's `PROVIDE(_stack_end = .)`) and
  `install_core0_stack_guard` was not called. Now `memory.x` defines `_stack_end = ALIGN(__euninit, 8)` (the
  true bottom of core 0's stack, in RAM), the 32 KiB ASSERT is checked on it, and `main` calls
  `embassy_rp::install_core0_stack_guard()` first (it writes MSPLIM). An overflow is a HardFault (escalated
  UsageFault STKOF); there is no fault handler, so without the probe's catch the core locks up (the HardFault's stacking
  is below MSPLIM) and the watchdog resets it; the next boot report reads "TimedOut, record false, WDG reset: no
  record" (measured twice, `NO_CATCH_HARDFAULT=1` then `ATTACH=1`, see `tools/run.sh`); under `probe-rs run` the hard-fault vector catch stops the
  run ("Firmware exited unexpectedly: Exception"). Core 1 was already guarded: `spawn_core1` writes MSPLIM with
  the bottom of `CORE1_STACK`. Negative control: `FEATURES=stack-negative`.
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
- **Bench:** the board through `tools/run.sh` only, one session at a time; battery in, propellers off (the owner's
  word, 2026-11-18), motors attached; the firmware drives GPIO36 to 39 (DShot Stop frames, PIO2) from every boot;
  never write settings to the receiver, the ESCs or the GNSS receiver.
- **This machine:** every test or build under a memory cap, every board run in its own detached scope
  (`_plan/WARNING-oom-tab-kills-20261115.md` has the commands). A runaway test killed whole sessions before
  this rule existed.
- **With the debugger attached, the debug logger can block both cores for tens of milliseconds** when its
  buffer fills. Figures taken under `probe-rs` include that; with no debugger it does not happen.

## How to run things

```
cargo host-test -p fw-core -p fw-settings -p fw-flight   # host tests (254 + 4 doc, 46, 20 today)
cargo build --release                                     # the firmware for the board
tools/run.sh 120                                          # build, flash, run, log to logs/ for 120 s
FEATURES=<feature> tools/run.sh 120                       # a test or measurement build (README lists them)
~/Documents/MARV-GCS/target/release/marv-cli --connect serial:/dev/ttyACM1 status | devices | param list ...
```

After `tools/run.sh` ends, the firmware keeps running on the board; the station and `marv-cli` reach it on
`/dev/ttyACM1`, and over the radio on UDP 14550 when the desktop is on the transmitter backpack's network and
the transmitter is in its MAVLink model.
