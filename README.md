# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today the binary is the two-core skeleton of the firmware (design: the two-core amendment of
`~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`) with the device layer and its first
devices on core 0, the BMP581 barometer (BARO), the BMM350 magnetometer (MAG), the LC76G GNSS receiver (GNSS) and the battery monitor on the ADC (BATT), and IMU1 and IMU2 on core 1; the foundation's hardware-free logic is in `crates/fw-core` (see Workspace).
It has no flight functionality.

- Core 0, the I/O core (`embassy_rp::init`, thread-mode executor): `usb_link`, a MAVLink node (system 1,
  component 1) on the USB port: HEARTBEAT at 1 Hz (status from the device rows through
  `fw_core::sys_status`: BOOT while a row is Starting in the first 3 s, then STANDBY), SYS_STATUS at 1 Hz
  (the three sensor masks from the device rows), `MARV_DEVICE_STATUS` (42421) for every device row once per
  second, the parameter protocol over the 25 parameters of BARO, MAG, BATT and the two IMUs' calibration offsets (see Parameters),
  STATUSTEXTs from the supervisor and from refused parameter writes, and the commands (see Commands).
  `supervisor`, a 10 ms tick: at 10 Hz it bumps `PING` and kicks the flight core, it tracks the age of
  `PONG`, gives the start-up verdicts, runs the devices' stuck check and transition texts, ends the running
  action (`actions::tick`), and writes `[STAT]` lines every 10 s. `beat0` bumps `BEAT0` every 50 ms (read by nobody yet). `baro`, `mag`, `gnss` and `batt`, the four core-0 device tasks (see
  Device layer).
- Core 1, the flight core (started with `embassy_rp::multicore::spawn_core1`, 16 KB static stack, its own
  thread-mode executor): `fw_flight::liveness`, which copies `PING` into `PONG` on each kick, IMU1
  (`src/imu1.rs`): the ICM-45686 on SPI0 with DMA channels 0 and 1 and its INT1 on GPIO15, and IMU2
  (`src/imu2.rs`): the LSM6DSK320X on SPI1 (SCK GPIO10, MOSI 11, MISO 8, CS 9) with DMA channels 2 and 3 and
  its INT1 on GPIO7 (INT2, GPIO6, unused), all built inside core 1's entry closure so their interrupts land on
  core 1, each running the flight-core device loop `fw_flight::imu` in its own task (see Flight-core device
  loop). The program's one `DMA_IRQ_0` binding (`DmaIrqs` in `src/main.rs`) lists the four channels. No log, no critical-section primitive of its own, and no
  timer in the sample loop there.
- RULE of the flight core (lead's ruling, 2026-11-15): the flight core verifies INCREMENTALLY (as ArduPilot's
  `check_next_register`). The sample loop never runs a whole-configuration `verify`: after every K-th sample,
  and never while a sample is waiting (the latched data-ready level, `ImuAdapter::data_waiting`), it takes ONE
  `verify_step`, which checks one register (one or two SPI transactions; the ICM-45686's indirect registers
  add their two 10 us busy waits; a register that differs is read once more). K = samples per second /
  registers per pass, so one pass takes about 1 s: IMU1 33 registers, K 24 (792 samples, 0.99 s); IMU2 32
  registers, K 25 (800 samples, 1.0 s). A mismatch is the same health event as a failed `verify`
  (reconfigure); a bus error at a step is an error event. The steps come from the drivers' additive
  `verify_steps` / `verify_step` (branch `verify-step` of both driver repositories). The earlier rule (a
  yield after every transaction and busy wait) was withdrawn: it helped the other task but stretched a
  whole `verify` past one period, during which the task cannot read its own samples. Each IMU task's
  longest poll (the longest stretch it ran without yielding, since boot and over the last 10 s window) and
  its poll count are taken around every poll (`flight_latency::measured`), published through its status
  cell (`ImuAdapter::latency`) and logged in its `[STAT]` line: the flight core's latency figure.
- Between the cores only atomics and `fw_core::xcore` cells (`Kick`, and each IMU's `Seqlock` and `Kick` in
  `fw_flight::imu::ImuCells`), in internal SRAM. `DMA_IRQ_0` is disabled on core 0 before core 1 starts (all
  DMA belongs to the flight core); start-up checks: not enabled on core 0, enabled on core 1, still not
  enabled on core 0 after core 1 started.

Not in it yet: IMU configuration parameters and the self-test (fixed configurations for now; the IMU calibration
offsets and the gyro, accelerometer and compass calibrations exist, see Calibrations), IMU2's
high-g accelerometer (off; nothing of it on the link), the RC receiver row and the radio link (the ELRS receiver on UART0), GNSS's PPS and time synchronisation (the pin is not read), the six-position accelerometer calibration and the compass's soft-iron (matrix) fit, the radio link
(UART), the SD card, the design's text ring of 8 (a fixed queue of 8 texts stands in: texts wait there until
a ground station's node runs; when it is full, newer texts are dropped with a log line).

## Flight-core device loop

`crates/fw-flight/src/imu.rs` (design: two-core amendment): `ImuTask` runs one IMU over an `ImuAdapter`
(the core-0 adapter contract: `start`, `acquire`, `status`, `verify`, `apply`, `recover`, `restart`, success
only after a fresh sample). It waits on `select(data-ready edge, kick)` and never on a timer; core 0's
supervisor kicks CHECK every 10 ms tick while the row's last sample is older than T = max(3 P, 10 ms); the
woken task compares the clock with its last sample and, if late by more than T, makes the one status read
(sample waiting = missed edge, read it; nothing = deadline miss). RESTART comes with its request number
through the same kick; APPLY is reserved. Start and recovery wait through the `DelayNs` passed in, wrapped in
`CountingDelay` (count after the start: 0 in a clean run, invariant F2); the configuration is checked one
register at a time between samples (the rule below), with no status read after a step: with a latched
data-ready a sample that arrived during the step keeps the line high and is read next (host tests pin the
pass length, the mismatch and bus-error events and "never while a sample is waiting").
The board time is `embassy_time::Instant::now()`, lock-free on core 1 (embassy-rp 0.10.0
`time_driver.rs:35-44` reads the timer's raw registers; only `schedule_wake`, `:46-47`, takes the critical
section, and the sample loop never schedules). Everything core 0 needs
(state, counters, the latest sample, the largest gap, missed edges, delay counts, restart outcome, the last
four transitions, attempt times, injection state) goes out through one `Seqlock`; the supervisor copies it
into the IMU's registry row (IMU1, IMU2: one loop over the rows) each tick and logs and texts its events; while an IMU is Failed or Absent it gets
no CHECK kick and is not flagged stuck (it is parked and waits only for RESTART). Host tests: `cargo host-test -p fw-flight`
(16 tests).

IMU1's fixed configuration: 4 g, 250 dps, 800 Hz on both sensors, internal clock (as `icm45686_smoke` ran
it on this board), data-ready on INT1 in latched mode (high until INT1_STATUS0 is read, as `icm45686_int1`
showed), waited for by level, so a sample that arrives while the task is busy is not lost; 800 Hz because
the ICM-45686 has no 1 kHz rate. Single-sample mode
(`read()` on every data-ready signal, the driver README's single-sample path), with INT1_STATUS0 (clear on
read) read with every sample so that the status read after a late CHECK sees a fresh flag; no FIFO, so
the overflow count stays 0. Its values go to the station as HIGHRES_IMU (id 0, acceleration, gyro,
temperature, the other fields NaN with their `fields_updated` bits clear; the gyro and acceleration have the `IMU1_GYR_OFF*` and `IMU1_ACC_OFF*` offsets subtracted, which is nothing while they are 0) at 25 Hz; the magnetometer's field in gauss (1 G = 100 uT) as `xmag`, `ymag`, `zmag` with their `fields_updated` bits
set only while the latest MAG sample is younger than 1 s (the same rule for the IMU1 fields: NaN and clear bits when older; SCALED_PRESSURE is not sent when its sample is older than 1 s); the barometer's as SCALED_PRESSURE at 25 Hz; the receiver's as GPS_RAW_INT (id 24) at 1 Hz (see GNSS below). SYS_STATUS carries
`voltage_battery` (mV) and `current_battery` (cA) of the latest BATT sample while it is younger than 1 s (65535 and -1, MAVLink's "unknown", while there is none or it is older);
`battery_remaining` stays -1.

IMU2's fixed configuration (`src/imu2.rs`): 4 g, 500 dps, both sensors at 800 Hz: the high-accuracy ODR set
"800" with the ODR code that is 960 Hz in the normal mode, as `lsm6dsk320x_modes` (check 4) ran it on this
board (800.664 Hz there; the datasheet gives +-1 % over temperature and supply with the gyroscope on). The
normal mode offers 480 or 960 Hz, and its 960 Hz ran at 942.66 Hz on this unit. The driver's default filters
(gyroscope LPF1 and low-g LPF2 off, data-ready mask on), high-g accelerometer off, no FIFO, SPI mode 0 at
8 MHz. Data-ready: the low-g accelerometer's on INT1 only, LATCHED (high until the output registers are read,
as `lsm6dsk320x_modes` check 5 showed), waited for by level, so a sample that arrives while the task is busy
is not lost. One `read_all()` per sample (temperature, gyroscope, low-g; the high-g registers are read and
dropped). STATUS_REG is not clear-on-read (its flags clear when the data is read), so no status read goes
with the sample; the chip has no reset flag, so a power-on reset shows as a timeout or a `verify` mismatch.
Its values go to the station (with the `IMU2_GYR_OFF*` and `IMU2_ACC_OFF*` offsets subtracted, which is nothing while they are 0) as SCALED_IMU2 (116) at 25 Hz: milli-g, milli-rad/s, centi-degrees (0.00 C sent
as 1, since 0 means "no temperature"), the magnetometer fields 0 (the chip has none); not sent when IMU2's
latest sample is older than 1 s. SYS_STATUS: 3D_GYRO2 + 3D_ACCEL2 from IMU2's row.

Measured with both IMUs and the incremental verify (2026-11-15, 10-minute clean run, log
`marv-v2-firmware-20261115-114930.log`): five rows Ok throughout, 0 transitions after the start, 0 errors,
timeouts, missed edges and recoveries, delay count after the start 0, watchdog stalls begun 0. IMU1 803.52
samples/s (its own rate: nothing lost), largest gap 2442 us, 110 gaps over 1.5 P (10 across a step), steps 134
to 946 us, 608 passes; IMU2 800.48 samples/s (bench 800.664 Hz), largest gap 3410 us, 1066 gaps over 1.5 P (79
across a step), steps 84 to 489 us, 600 passes. Gaps and long polls are REPORTED, not a pass criterion: the
data-ready, one-read-per-sample scheme is Betaflight's, which loses a sample when the reader is late and does
not count it; ArduPilot and PX4 avoid the loss with the chip's FIFO (kept in reserve here). An IMU row is
accepted when it is Ok with 0 errors, timeouts and recoveries and its rate is within 1 % of the chip's own.
The longest poll is 1.06 to 2.56 ms (IMU1) and 0.63 to 1.36 ms (IMU2) in every 10 s window, with
single-register steps, so it is not `verify`; while IMU2 was parked IMU1 had no new gap but still polls of
about 1 ms. Three 620 s runs (logging as usual, `DEFMT_LOG=off`, `quiet-stat`) gave IMU1 63 / 37 / 42 gaps and
IMU2 379 / 279 / 199: logging on core 0 is NOT the main cause. The cause is not found; it belongs to the
measurement of what disturbs the flight core, to be done before any control loop runs there (the features
`latency-text` and `quiet-stat` and the per-poll latency figure exist for it). Facts and suspects:
`~/Documents/MARV-V2-drivers/_plan/facts-foundation.md`, "Two IMUs on the flight core".

Features: `FEATURES=inject-imu1-persistent tools/run.sh 120`: 5 s after the first good sample every
`acquire`, `recover` and `restart` fails until `marv-cli device restart IMU1`; `inject-imu2-persistent` the
same for IMU2 (`device restart IMU2`); `inject-imu2-verify`: one IMU2 verify step answers "mismatch" once
(reconfigure, `config restored` 1); one wrapper serves both IMUs (`src/imu_inject.rs`). `FEATURES=inject-imu1-missed-edge
tools/run.sh 60`: 5 s after the first good sample the data-ready edges are swallowed for 200 ms. The
injection state goes out through the cells and core 0 logs it. `FEATURES=stall-core0 tools/run.sh 60`: one
core-0 task spins 1.2 s without yielding 20 s after start and logs IMU1's and IMU2's samples before and after (F1).
The watchdog survives that spin (fed again before 1.6 s without a feed; a "core 0 stalled" text, no reset); the
feature does not act after a watchdog reset, so it does not loop.
`FEATURES=f4-negative tools/run.sh 20`: a DMA channel (6, since 2 and 3 are IMU2's) is built on core 0 before the start-up check
(negative control of F4: `[FAIL] DMA_IRQ_0 not enabled on core 0`), its line disabled again before core 1
starts.

## Device layer

`src/device.rs` (design item 1, on core 0): each device has a task running the one generic `run_device`
loop over a firmware-side `DeviceAdapter` (`data_ready`, `start`, `acquire`, `status`, `verify`, `apply`,
`recover`, `restart`, `counters`; no `self_test` yet). The loop steps the device's
`fw_core::health::Health` and carries out its action. Sampling is `select(with_timeout(T, data ready),
wake)` with T = max(3 P, 10 ms) (GNSS: its own 2.5 s); a timeout makes ONE status read: data waiting = a missed edge, read it;
nothing = a deadline miss. Every bus call is awaited to completion, never inside a `select` or a timeout.
A start, `recover` or `restart` that returns `Ok` only says the bus and the device answer: the loop then
waits for ONE fresh sample (`with_timeout(T, data ready)`, then one `acquire` awaited to completion) before
it tells `Health` the step worked; a timeout or a failed `acquire` counts as a failure (a failed `recover`
goes on to `restart`, a failed `restart` ends the attempt, a failed start is a failed start), so "recovered"
and "started" mean data flows. On core 0 `verify()` runs about once per second between reads; the flight-core IMUs verify one register per step instead (see the flight core's rules). Each device has a static registry row (`DEVICES`): id,
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
| MAG | BMM350 (`src/mag.rs`) | I2C0 at 400 kHz, SDA GPIO12, SCL GPIO13, data-ready GPIO4 (rising edge), no DMA | from `MAG_ODR_HZ` and `MAG_AVG`; defaults = the driver's bench config: 100 Hz, averaging 2, all axes, normal mode, data-ready pulsed, active high, push-pull | 10 000 us at the default; 1 / rate afterwards | max(3 P, 10 ms): 30 ms at the default | max(1 s, 20 P) |
| GNSS | LC76G (`src/gnss.rs`) | UART1 (`BufferedUart`, interrupt driven on core 0, no DMA) 115200 8N1, board TX GPIO24, RX GPIO25, RX buffer 4096 B; PPS GPIO5 not read | none: receive only, nothing is sent to the receiver; it is read as it comes (default stream, 1 fix per second) | 1 000 000 us (1 Hz) | 2.5 s (own deadline, not 3 P) | 20 s |
| BATT | the RP2350 ADC (`src/batt.rs`) | ADC on core 0 without DMA: GPIO42 (ADC2, battery voltage, 100k / 10k), GPIO41 (ADC1, ESC current output, no pad pull); `ADC_IRQ_FIFO`; no bus, no chip | `BATT_V_SCALE`, `BATT_A_SCALE`, `BATT_A_OFFSET`, read for every sample | 100 000 us (10 Hz) | 300 ms (3 P) | 2 s |

**MAG.** `acquire` reads INT_STATUS (clear-on-read; in the driver's pulsed mode it also clears itself 1.25 ms after the edge, so after a
data-ready timeout `status` finds "nothing pending" and a missed edge shows as a timeout, not as a missed edge: accepted, the bench-proven pulsed mode is kept and latched mode is not used), then `read_field`: the
compensated field in microtesla and the die temperature, minus `MAG_OFFX/Y/Z` in software (the chip's offset registers are not
used). `Reading::Mag { field_ut, temperature_c }`. The BMM350 has no power-on-reset flag: a reset shows as a `verify` mismatch.
`verify` and `recover` are the driver's, `apply` is `configure`, `restart` does the bus clear (`src/busclear.rs`, shared with BARO) and
rebuilds the I2C0 bus. Sample rate: the chip's own oscillator, measured 0.7 % fast on the bench unit (100.7 Hz at the 100 Hz setting).
The field at rest on the driver's bench was about 66 to 67 uT.

**GNSS.** A row for the receiver, `src/gnss.rs`, the `lc76g` driver as it is (rev 2ee54f9), UART1 read in thread mode through the driver's
`read`. **The firmware only listens**: no `init`, no setting, no query, no save, no baud change is ever sent (the TX pin is driven idle-high so the
receiver's input does not float), so the receiver's stored configuration cannot change; the row sees the receiver's default stream
(GGA, GLL, GSA, GSV, RMC, VTG, 19 sentences per second at 1 fix per second on the bench). PPS (GPIO5) is not configured or read: neither the driver
nor its bench program does anything with it beyond counting edges, and there is no time synchronisation here.
- **Cancel answer (the design's "GNSS only after the `lc76g` cancel check").** Dropping the driver's `read`, `read_fix` or `read_sentence` future is
  SAFE: nothing is lost, no sentence is corrupted, and `recover` is not needed afterwards. Evidence: the driver README, "Dropped reads"
  (`lc76g/README.md:111`, "dropping a `read*` or `command` future loses nothing the driver already read"), the host test
  `dropping_a_read_future_loses_nothing` (`src/tests.rs:1013`: a fix cut in the middle, the future dropped, the rest fed, the next `read_fix`
  returns the right fix and 6 sentences are counted), the code (`Lc76g::next_sentence`, `src/lib.rs`: the driver's own buffer, position and
  framer hold everything; the only await is one `uart.read`, and `BufferedUart::read` copies its bytes in the poll that completes it, so a drop
  can only come before bytes were taken), and the board (`lc76g_resync`, README line 172: 10 Hz fixes read under 20 ms timeouts, 100 of 100 fixes
  kept over 444 dropped calls). A half sentence stays in the framer and is finished by the next read. Only a `command` write is not droppable
  (a partial line on the wire; the module discards it by its checksum), and this firmware sends none. **Shape that follows**: the data-ready wait
  IS the read of the next fix (`data_ready` reads sentences, counts them, and returns when the driver has completed a fix, that is a GGA and an RMC
  of one epoch; the fix waits in the adapter), so the loop's `select(with_timeout(T, data_ready()), wake)` may drop it at any await; `acquire`
  turns the waiting fix into the reading and never blocks.
- **Ok means**: valid sentences keep arriving, WITH OR WITHOUT a position. One sample is one complete fix epoch (GGA + RMC of the same UTC time);
  the position fields can be empty. 2.5 s without one is ONE error event (TIMEOUT; the row's own deadline T = 2.5 s, Betaflight's figure, instead of
  3 P = 3 s): the score is 3 per event and 9 enters Recovering, so Degraded after 2.5 s, Recovering after 7.5 s of silence (measured from the last
  good sample, see the injection run), as the design says. A checksum error, a malformed sentence, a sentence over the length limit and a UART
  error (overrun, framing, break, parity) are counted (`[STAT] GNSS:`) and are NOT error events: the driver has no "stream broken" signal, and a
  stream that is broken shows as silence. `status` answers "nothing pending" (a deadline miss); `verify` and `apply` have nothing to do.
- **`start`** waits up to 5 s for the first valid sentence of any type (bench: 367 to 424 ms after the start of the task); no identity query
  (that would need a command). The README of the driver gives no start time of the module, so 5 s is this firmware's own bound. **How the long
  start fits the health machine**: the machine's start retries wait 100, 300 and 1000 ms, too short for a receiver that is still booting, so the
  waiting is INSIDE `start` (it is awaited to completion, and its 5 s are below the stuck threshold of 20 s); with three retries a receiver that
  never talks is Absent after 4 x 5 s + 1.4 s. `fw-core` is unchanged. The loop then waits for one fresh fix (at most T) as after any start.
- **`recover`** = flush and resynchronise: the UART's receive buffer is read empty (reads until one finds nothing for 5 ms, at most 128), and the driver
  is rebuilt over the same UART (`release` then `new`: no half sentence, no held fix). **`restart`** (the first attempt after an operator request, and
  the automatic recovery's second step) is the same: a UART has no bus clear, and nothing is configured, so there is nothing stronger to do
  (noticed: the UART block is not re-initialised). The driver's counts restart at 0 with the new driver; the row's totals add them up.
- `Reading::Gnss { fix_type, lat_e7, lon_e7, alt_mm, speed_mm_s, course_cdeg, sats_used, hdop_x100, vdop_x100, time, date }`, the driver's integer
  units (1e-7 degree, mm, mm/s, 0.01 degree, DOP x 100; no `f64` anywhere: the build has 0 `__aeabi_d` symbols). `fix_type` is MAVLink's
  `GPS_FIX_TYPE`: 1 (NO_FIX) unless the RMC status, the GGA quality and the GSA fix mode all say there is a position, then 2 (2D), 3 (3D) or 4
  (the GGA quality says differential or SBAS-aided, 3D). Position, altitude, speed and course are `None` while `fix_type` is 1.
- **GPS_RAW_INT (id 24), 1 Hz** (the fix rate; the slot's minimum interval is 100 ms). With a fix: `lat`, `lon` in 1e-7 degree, `alt` the altitude above
  mean sea level in mm, `eph`/`epv` HDOP/VDOP x 100, `vel` in cm/s, `cog` in 0.01 degree, `satellites_visible` the satellites USED (the GGA's count;
  the GSV's count of visible satellites is not read), `time_usec` the board time of the sample (not UTC; UTC is in the reading and the `[STAT]` line).
  Without a fix: `fix_type` NO_FIX, `lat`, `lon` and `alt` 0 (the dialect defines no other convention: it says nothing about a missing position),
  `vel` and `cog` UINT16_MAX, `eph`/`epv` whatever the receiver says about its DOP or UINT16_MAX, `satellites_visible` its count or 255. The
  1 s age rule of the other values is replaced by a 3 s rule: no message is sent while the latest sample is older than 3 s (3 fix periods; at
  1 Hz a sample is up to 1 s old when it is read, so 1 s would drop every message). The same sample can go out twice when the stream's tick comes
  before the next fix.
- **Parameters: none.** Nothing of the receiver is configured and there is nothing the driver's README shows as a proven run-time setting that
  the owner would plausibly change, so `fw-settings` has no GNSS module.
- A settings save (`param save`) runs with all interrupts off for about 40 ms (rule 1 of `docs/resources.md`); the UART's 32-byte FIFO holds 2.8 ms
  of line, so a save loses a piece of the stream (counted as a UART overrun and a checksum error, not an error event, and well under T). Not
  exercised on the board in this packet (no save was run).
- With nobody reading (the row Failed, or between recovery attempts) the 4096-byte buffer fills in about 3 s and the UART overruns; the first read
  afterwards reports it (counted) and `recover`/`restart` throw the old bytes away (4128 bytes in the injection run).
- `[STAT] GNSS:` (after the row's generic line): sentences and sentences per second, fixes, checksum errors, malformed, overflows, epochs dropped,
  the four UART error counters, resyncs and the bytes they threw away, the fix type, satellites used, HDOP x 100, the UTC time of the latest fix and
  the age of the last valid sentence.

**GNSS on the board** (2026-11-15). The receiver is not indoors-without-sky on this bench: it reported a fix at every sample (fix type 4, DGPS/SBAS,
28 to 29 satellites used, HDOP 0.5), so the NO_FIX branch of `fix_type` and of GPS_RAW_INT was NOT seen on the board. Clean run (330 s, log
`marv-v2-firmware-20261115-130145.log`): six rows Ok throughout, 0 transitions after the start, all 7 `[PASS]` start-up lines, no `[FAIL]`,
no reset, watchdog stalls begun 0. GNSS: the first valid sentence 367 ms after the start of its task (424 and 847 ms in two other runs), Ok at 467 ms;
1.0000 fixes/s (nominal 1 Hz) and 19.0 sentences/s, 300 fixes and 5698 sentences at 300 s, 0 errors, 0 timeouts, 0 recoveries, 0 checksum errors,
0 malformed, 0 overflows, 0 epochs dropped, UART1 errors 0 (overrun, framing, break, parity), largest gap between fixes 1.033 s. The other rows at 300 s:
IMU1 803.6/s, IMU2 800.7/s, BARO 49.80/s, MAG 100.1/s, BATT 10.00/s, all 0 errors (the earlier clean run: IMU1 803.52, IMU2 800.48). `marv-cli devices`:
six rows OK; `marv-cli status`: the health word lists GPS and `devices 6`; `marv-link-test` 20 s: GPS_RAW_INT 21 (both ends of the 20 s counted), SYS_STATUS 21,
SCALED_PRESSURE 501, HIGHRES_IMU 502, SCALED_IMU2 501, MARV_DEVICE_STATUS 164, 0 lost, 0 checksum failures. `device restart GNSS` on the healthy
row: ACCEPTED, `Ok -> Starting` at 40 710 ms and `Starting -> Ok` at 40 946 ms (the UART flushed, 0 bytes). Injection `inject-gnss-silent`
(log of the 100 s run): Ok at 522 ms, sentences swallowed from 10 524 ms (the last good sample was at about 9.5 s); error events (TIMEOUT) at 12.02 s
(Degraded), 14.5 s and 17.02 s (Degraded -> Recovering, WARNING text): 7.5 s after the last good sample, 6.5 s after the swallowing began; recovery
attempts 1 to 4 start at 17.12, 22.43, 28.53 and 36.54 s (100, 300, 1000 and 3000 ms after the previous failure), each takes 5.0 s (`recover` then
`restart`, each waiting 2.5 s for a fix); Failed at 41.55 s after exactly 4 attempts with ONE ERROR text; the other five rows undisturbed, no reset; at
79.0 s `device restart GNSS`: ACCEPTED, `Failed -> Starting`, Ok at 80.53 s (the swallowing ends with the request).

**BATT.** One sample every 100 ms from a period timer (the "data ready" of this device; a period wait is no bus call). `acquire`
converts the voltage and the current channel with interrupt-driven single conversions, each awaited (one discarded after the channel
switch, then 16 averaged), and `adc-battery` turns the sums into millivolts and milliamps; both conversions together must finish within
3 periods, else it is an error event (timeout). **"Ok" for BATT means only that the ADC converts**: there is no chip identity, `start`
succeeds when one conversion of each channel completes (after 150 ms for the pads to settle), `verify` and `apply` have nothing to do
(the parameters are read for every sample), `recover` and `restart` are a conversion of each channel. With the battery unplugged the
voltage reads about 0 V (a few counts, tens of millivolts); the current is NOT calibrated (the default scale, 0.020 V/A with its zero at
0 V, is the crate's bench PLACEHOLDER, so an unplugged board reports a small positive current). `Reading::Batt { voltage_v, current_a }`.

The row's period P is a value that changes: the adapter sets it from the configured rate (at every start
and every apply, before the chip is configured), so T = max(3 P, 10 ms), the stuck threshold and the
`[STAT]` rate follow the rate.

Requests to a device task: the row has an APPLY request and a RESTART request (a flag byte, plus the row's
`wake` signal). The task takes them at the top of its loop, never inside a bus call. RESTART is allowed in
every state: the loop feeds `Event::Restart` to `Health` (Starting, from Ok, Degraded, Recovering, Failed
and Absent alike) and starts the device again; a back-off wait is cut short by it. The first start attempt
after an operator RESTART request goes through the adapter's `restart` (for BARO and MAG: bus clear, rebuild
the bus, start), so a bus stuck low is cleared; if that attempt fails, the normal start retries follow (they use
`start`). A boot, and every start without a RESTART request, uses `start`. The loop then waits for one fresh
sample as after any start. The flight-core IMU loop does the same. APPLY is carried out when
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
- `FEATURES=inject-mag-fail-until-restart tools/run.sh 120`: as the barometer's, around the magnetometer's adapter (the barometer is
  not injected): 5 s after the first good sample every `acquire`, `recover` and `restart` fails until `device restart MAG`.
- `FEATURES=inject-baro-por tools/run.sh 60`: one `acquire` answers a power-on reset. Shows the
  power-on-reset path: Ok -> Recovering at once (no Degraded, `errors` not incremented, `last_error` POR),
  WARNING, a recovery, Ok, NOTICE.
- `FEATURES=inject-baro-stuck tools/run.sh 60`: `acquire` never returns. Shows the supervisor's stuck
  check: after max(1 s, 20 P) `stuck` is set, the device's `MARV_DEVICE_STATUS` row reports FAILED with
  STUCK, one ERROR text "BARO: stuck"; the board is not reset (a device stuck in an await does not stop
  core 0's executor, so the watchdog stays fed) and the rest of the
  firmware (HEARTBEAT, PING/PONG, `[STAT]` lines) keeps running.

- `FEATURES=inject-gnss-silent tools/run.sh 100`: 10 s after GNSS's first good sample the adapter stops seeing sentences (the wrapper swallows the
  fixes; the receiver keeps talking) until a RESTART request (`device restart GNSS`). Shows the long path of a device that stays silent:
  one TIMEOUT error event per 2.5 s (Degraded after the first, Recovering after the third), 4 recovery attempts (each a `recover` then a `restart`, each
  waiting one deadline for a fix), Failed, ONE ERROR text; then `device restart GNSS` -> Ok. Times in the "GNSS on the board" paragraph.

Run `tools/run.sh` again afterwards to put the normal build back on the board.

## Parameters

The parameter tables: BARO (below), MAG, BATT and the IMUs' offsets (after it); `fw_settings::TABLE` joins them (25 entries: 5 + 5 + 3 + 12). The barometer's (`crates/fw-settings/src/baro.rs`, a `static [ParamDef; 5]`), all `I32`, device BARO,
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
Every start and restart reads the stored values the same way. A write is NOT saved to flash: it is lost at
a reset unless the station's "Save to flash" (`marv-cli param save`) follows it (see Settings in flash).
Every parameter write, accepted or refused, has an `[INFO] param <NAME> = <value>: ...` line.

### MAG parameters

`crates/fw-settings/src/mag.rs`, device MAG. The two configuration parameters are the `Configure` class (the whole candidate goes
through the driver's `Config::validate` before the value is stored); the three offsets are `Live` and `calibration`
(`param reset-calibration` resets them); the compass calibration writes them (see Calibrations).

| Name | Type | Default | Range | Means |
|---|---|---|---|---|
| `MAG_ODR_HZ` | F32 | 100 | 1.5625 to 400 | output data rate in Hz: 400, 200, 100, 50, 25, 12.5, 6.25, 3.125 or 1.5625; refused otherwise ("rate not offered by the chip") |
| `MAG_AVG` | I32 | 2 | 1 to 8 | samples averaged: 1, 2, 4 or 8 ("not 1, 2, 4 or 8"); the rate allows at most 1 at 400 Hz, 2 at 200 Hz, 4 at 100 Hz, 8 below ("averaging too high for the rate") |
| `MAG_OFFX`, `MAG_OFFY`, `MAG_OFFZ` | F32 | 0 | -200 to 200 | offset in microtesla subtracted from the field of that axis in every sample (by the core-0 adapter, `fw_settings::mag::apply_offsets`) |

### BATT parameters

`crates/fw-settings/src/batt.rs`, device BATT, all `F32`, `Live`: applied to the next sample, no validator. The defaults are the
`adc-battery` bench program's configuration.

| Name | Default | Range | Means |
|---|---|---|---|
| `BATT_V_SCALE` | 1.0 | 0.1 to 10 | gain on the battery voltage (100k / 10k divider, 3.3 V reference): the crate's `gain_ppm` is `(scale - 1) * 1e6`; 2.0 doubles the voltage; the bench's one-point trim for this board would be 1.007603 (not applied) |
| `BATT_A_SCALE` | 0.02 | 0 to 10 | volts at the pin per ampere of the ESC's current output; the default is the crate's PLACEHOLDER, not calibrated; 0 reads 0 A |
| `BATT_A_OFFSET` | 0.0 | -3.3 to 3.3 | volts at the pin with no current (the crate's `zero_uv`) |

### IMU offset parameters

`crates/fw-settings/src/imu.rs`, twelve `F32` parameters, `Live` and `calibration` (`param reset-calibration`
resets them), default 0, devices IMU1 and IMU2. The flight core's adapter subtracts them from every sample
(see Calibrations); a change reaches it within one supervisor tick.

| Name | Range | Means |
|---|---|---|
| `IMU1_GYR_OFFX`, `IMU1_GYR_OFFY`, `IMU1_GYR_OFFZ`, `IMU2_GYR_OFFX`, `IMU2_GYR_OFFY`, `IMU2_GYR_OFFZ` | -34.9066 to 34.9066 | gyro offset in rad/s: the gyros' full scale, 2000 dps (Betaflight's `GYRO_SCALE_2000DPS`), so the store holds any rate a working gyro reports |
| `IMU1_ACC_OFFX`, `IMU1_ACC_OFFY`, `IMU1_ACC_OFFZ`, `IMU2_ACC_OFFX`, `IMU2_ACC_OFFY`, `IMU2_ACC_OFFZ` | -9.80665 to 9.80665 | accelerometer offset in m/s^2: ArduPilot's largest accepted offset, one gravity (`AccelCalibrator::accept_result`) |

## Settings in flash

`src/settings.rs` (design: decision-foundation item 3, the two-core amendment "Item 3, changes", and the
owner's answers of 2026-11-15). The parameters persist in the `SETTINGS` region, two 4096-byte sectors
A (0x101FE000) and B (0x101FF000), one `fw_core::record` record each: only the values that differ from
their default.

- NOTHING IS SAVED WITHOUT SAVE. A write takes effect in RAM only. The flash is written only by
  MAV_CMD_PREFLIGHT_STORAGE param1 = 1 (`param save`); no automatic save, no save before a reboot or after a
  reset or a calibration. `param reset-all` resets RAM only; `param save` afterwards stores a record with 0
  entries.
- Boot, before the devices start (so they start with the stored values): both sectors are read and
  `record::choose` decides. A current record (the valid one with the highest sequence): the store is reset
  to defaults and each entry loaded; an entry that fails (unknown name, wrong type, out of range, read
  only) is skipped with one WARNING text, at most 4 named, then one "settings: ... and N more skipped".
  Both sectors erased: defaults and one INFO text "settings: none stored, defaults". Otherwise no valid
  record: defaults and one WARNING "settings: stored record invalid, defaults"; NOTHING is erased. Then
  (at boot and after a reload) each device's stored values go through that device's validator (the
  barometer: `fw_settings::baro::settings_to_config`, what `validate` runs); a set it refuses (for
  example a rate in the table's range that the chip does not offer, from a record of another firmware
  version: `load` runs no validator) puts ALL of that device's parameters back to their defaults in RAM,
  the flash untouched, with ONE WARNING text "BARO: stored settings invalid, defaults"; the device then
  starts normally on its defaults (`param list` shows them; a save writes them). One log
  line `[INFO] settings boot: <class>, sector, sequence, loaded, skipped; sector A <state>, sector B <state>;
  next save to sector <X> with sequence <N>`. The texts are queued before anything else and wait in the
  text queue until a ground station connects (if nobody ever connects they stay queued; nothing is lost
  unless the queue of 8 fills first).
- Save (`param save`): FAILED with the text "save failed: flight core not answering" unless the flight
  core's `PONG` was seen fresh within 100 ms (the HAL pauses core 1 through its FIFO interrupt and waits for
  it without a time limit); FAILED "save failed: armed" when the arm/save handshake
  (`fw_core::handshake`, `begin_save`) refuses. Otherwise the record (`record::encode`, `non_default()`,
  the next sequence) is written to the sector that does NOT hold the current record (erase, then program
  only the record's bytes), read back and compared byte for byte and decoded; a mismatch ends FAILED
  "save failed: verify failed" and the previous record stays the current one. Then `end_save`, the
  store's dirty flag is cleared, ACCEPTED. The store's lock is never held across the flash operation.
  One log line per save: sector, sequence, entries, bytes, the stall (erase and program), verify.
- Reload (`param reload`, param1 = 0): the boot rule again; the devices whose values changed get their
  APPLY request; ACCEPTED (also with no valid record: defaults, with the boot's text).
- The range guard: every erase and program passes ONE function (`settings::guarded`) that refuses any
  range outside offsets 0x1FE000 to 0x200000 with a `[FAIL]` log line and an error, never a write.
- The stall: an erase or program runs with interrupts off on core 0 and core 1 paused. Measured on the
  board over 100 saves (feature `save-loop`): 30.6 ms min, 44.8 ms max, 40.2 ms mean (erase 30 to 44 ms,
  program of one page about 0.6 ms); the design's bound is 533 ms. Each stall is published (`SAVE_EPOCH`,
  the last window); a core-0 device's deadline miss or stuck window that a save overlapped is discarded
  (logged, `discarded, a flash save overlapped it`), never a health event. IMU1 (flight core): the samples
  due during the stall are not read; after it, one long gap (largest 47 ms) and no error, timeout or
  recovery. Over the 100 saves: 0 device transitions, 0 errors, 0 timeouts on IMU1 and BARO.
- `tools/run.sh` re-flashes only the program (`FLASH`, below 0x101FE000): the settings survive it
  (checked 3 of 3); the power-cycle half of I3.5 is not checked yet.

Features for the checks (off by default):

- `FEATURES=save-loop tools/run.sh 200`: 60 s after start, 100 saves one second apart, `BARO_IIR_T`
  alternating 1 and 3, each stall logged, then `[INFO] save-loop summary: ...` (count, verified,
  min/max/mean stall, device transitions, IMU1 missed edges, long gaps and largest gap, BARO timeouts,
  discarded windows, guard refusals, the last value saved). Flash wear: 100 erases per run. Runs once per
  power-up or flash: not after a reset the watchdog caused (so a fault can never make it loop). Not yet run
  under the watchdog (see Watchdog).
- `FEATURES=fake-armed tools/run.sh 60`: the handshake's armed flag is set at boot; `param save` ends
  FAILED "save failed: armed" and writes nothing.
- `FEATURES=corrupt-newest tools/run.sh 60`: at boot, before the settings are read, one byte of the newest
  valid record (the first entry's value byte, else the CRC's, else the magic's) is programmed to 0x00
  through the guard; the boot then chooses the other sector (invariant I3.2) and the next save goes to the
  corrupted sector's place.
- `FEATURES=guard-negative tools/run.sh 60` (NEGATIVE CONTROL of the range guard): at boot, before the
  settings are read, an erase of the sector just below the region (0x1FD000..0x1FE000), a program of one
  byte just above its end (0x200000) and an empty program; each is refused by the guard before any flash
  call (three `[FAIL] settings: ... refused` lines), then one `[PASS] guard-negative: 3 of 3 refused ...,
  GUARD_REFUSALS 3`; the firmware runs on normally.
- `FEATURES=save-foreign tools/run.sh 60`: the first `param save` after boot also stores `ZZZ_UNKNOWN` = 1
  and `BARO_ODR_HZ` = 33 through the normal guarded save. The next normal boot skips `ZZZ_UNKNOWN` with
  its text and falls back to the barometer's defaults (text "BARO: stored settings invalid, defaults").
  Put a clean record back afterwards (`param reset-all --yes`, `param save`).
- The existing `no-fbeat`: `param save` ends FAILED "save failed: flight core not answering".

## Calibrations

`marv-cli calibrate gyro`, `calibrate accel`, `calibrate compass` (MAV_CMD_PREFLIGHT_CALIBRATION 241) and
`calibrate cancel` (the all-zero 241). The decision is pure and tested on the host (`fw_core::calib`); the
firmware glue is `src/calibration.rs` on core 0, stepped from the supervisor tick (`actions::tick`). One
calibration at a time (the one `ActionBook`); progress 0 to 100 % goes out through the book, HEARTBEAT
`system_status` is CALIBRATING while one runs. The result is written into the parameters and used at once;
it is NOT saved to flash: only the user's Save (`param save`) stores it, and a reset or `param reload`
loses it. Nothing is written by a failed or cancelled run. Every calibration queues its result or failure
text BEFORE its action ends, and the final ACK is held until the text queue is empty (`actions::next_ack`,
because the node sends ACKs before it takes texts), but for at most 300 ms (then the ACK goes out even if
texts are still waiting, and a text may arrive after its ACK; one `[INFO]` log line says so), so normally the
text precedes the ACK on the link and the command-line tool prints it. The action's time limit is applied
every supervisor tick (`actions::tick`), whether or not an ACK is held.

**Cancel.** `calibrate cancel` drops the running calibration, nothing is written, the action ends CANCELLED
(its final ACK; no text).

**What the offsets are subtracted from, and where.** Every offset is subtracted from the sample in software
(the chips' own offset registers stay 0). IMU offsets: core 0 writes `IMUx_GYR_OFF*` and `IMUx_ACC_OFF*` into
one `fw_core::xcore::TripleBuffer<ImuOffsets>` per IMU (at boot before core 1 starts, and within one
supervisor tick of any change of the stored values: a parameter set, `param reset-calibration`, a reload,
a calibration result; pushes wait while a calibration runs, so the offsets stay constant during the
measurement); each IMU's adapter on the flight core reads it wait-free with every sample and publishes
`raw - offset`. MAG offsets: the core-0 adapter (`src/mag.rs`) reads `MAG_OFFX/Y/Z` from the store for every
sample and publishes `raw - offset` (`fw_settings::mag::apply_offsets`). So the samples on the link and in
the rows are corrected, and a calibration measures the RAW value: it adds the offsets in force back to each
sample it sees (`ImuOffsets::restore`, `fw_core::calib::mag_raw`); a second calibration with the first's
offsets in force gives the same result. Core 0 sees the rows' latest sample (the supervisor mirrors the
flight rows every 10 ms, about 100 per second per IMU); every sample is counted once by its `seq`.

### Gyro (`calibrate gyro`, both IMUs)

Method: Betaflight `performGyroCalibration` (`src/main/sensors/gyro.c`): the samples of a fixed duration,
`gyroCalibrationDuration` = 125 (1.25 s, `pgResetFn_gyroConfig`); their mean is the offset. Moving when the
standard deviation (`devStandardDeviation`, `common/maths.c`, n - 1) of any axis exceeds
`gyroMovementCalibrationThreshold` = 48 raw counts, at the 2000 dps scale Betaflight runs every gyro at
(`GYRO_SCALE_2000DPS` = 2000 / 2^15 dps per count, `drivers/accgyro/accgyro.h`): 2.93 dps. Betaflight starts
over when moving; here the run fails. Duration: 125 samples, about 1.25 s. There is no limit on the offset
(neither Betaflight nor ArduPilot has one) beyond the parameters' range, the gyros' full scale: a mean the
store refuses (a broken sensor) ends FAILED. Texts: "gyro calibration: moving", "gyro calibration: offset
out of range", "gyro calibration: no IMU Ok", "gyro calibration: IMUn not Ok, skipped" (that IMU is left
out); the result "gyro cal: |IMU1| N |IMU2| N urad/s" (the length of each IMU's offset vector). One IMU
moving fails the whole run.

### Accelerometer (`calibrate accel`, both IMUs, one position, +Z up)

Method: ArduPilot `AP_InertialSensor::simple_accel_cal`: averages of 50 samples; two successive averages
closer than `accel_convergence_limit` = 0.05 m/s^2 (length of the difference) converge, the offset is
their mean minus gravity (0, 0, 9.80665); not converged within 20 averages ("up to 10 seconds"): failed.
The largest offset accepted per axis is one gravity (`AccelCalibrator::accept_result`, `GRAVITY_MSS`;
`simple_accel_cal` itself has no limit). Duration: 1 s at the least, 10 s at the most. Texts: "accel
calibration: moving" (not converged), "accel calibration: IMUn offset too large", "accel calibration: no IMU
Ok", "accel calibration: IMUn not Ok, skipped"; the result "accel cal: |IMU1| N |IMU2| N mm/s^2".

### Compass (`calibrate compass`, MAG)

Method: Betaflight's compass calibration by minimum and maximum, `compassUpdate`
(`src/main/sensors/compass.c`, release tags up to 4.4.3): for 30 s (`(currentTimeUs - tCal) < 30000000`) the
user turns the vehicle through all orientations, the firmware tracks the minimum and maximum of each axis
(`magZeroTempMin`, `magZeroTempMax`), and the offset is `(min + max) / 2`. What master does now: 4.5 and
master (`CALIBRATION_TIME_US` = 30 s, `CALIBRATION_WAIT_US` = 15 s) start on movement and fit a recursive
bias estimator (`compassBiasEstimator`) instead; this firmware keeps the minimum and maximum method.
Duration: 30 s of the samples' own time stamps from the first sample (admission limit 40 s). The radius
seen on an axis is `(max - min) / 2`. Accepted only if EVERY axis' radius lies strictly between ArduPilot's
bounds (`libraries/AP_Compass/CompassCalibrator.cpp`: `#define FIELD_RADIUS_MIN 150`, `#define
FIELD_RADIUS_MAX 950`, milligauss; `fit_acceptable_status`: `radius <= FIELD_RADIUS_MIN || radius >=
FIELD_RADIUS_MAX` is `FAILED_RADIUS`), that is more than 15 and less than 95 microtesla (1 G = 100 uT); Betaflight
accepts whatever comes out. Texts: "compass cal: not rotated, radius X Y Z uT" (integers; a vehicle at rest
shows about 1 each), "compass cal: offset out of range X Y Z uT" (the offsets would fall outside
`MAG_OFF*`'s -200 to 200), "compass cal: MAG not ok" (the row is not Ok when the action starts; FAILED
at once); the result "compass cal: off X Y Z r X Y Z uT" (offsets and radii, integers, INFO). On success
`MAG_OFFX/Y/Z` hold the midpoints and the adapter uses them from the next sample. Not built: soft-iron
(matrix) fit, automatic or in-flight compass learning.

### Test feature

`inject-mag-rotation` (off by default; `FEATURES=inject-mag-rotation tools/run.sh`): while a compass
calibration runs, its input (not the published samples) is replaced by a synthetic rotation, the six points
(+-50, 0, 0), (0, +-50, 0), (0, 0, +-50) uT about the centre (10, -20, 30) uT, each held for 100 samples, so
the success path can be shown on a board that cannot be turned: offsets (10, -20, 30), radii 50.

### On the board (2026-11-15, MARV V2 Rev 1 at rest on the bench, nothing saved)

Normal build: `calibrate accel` ACCEPTED, "accel cal: |IMU1| 285 |IMU2| 294 mm/s^2" printed before the exit
(exit 0); `calibrate gyro` ACCEPTED, "gyro cal: |IMU1| 5417 |IMU2| 5412 urad/s"; `calibrate compass`: 0 to
98 % over 30 s, then FAILED "compass cal: not rotated, radius 1 1 1 uT" (exit 1), `MAG_OFF*` still 0; a run
interrupted at about 9 s and ended by `calibrate cancel`: "compass calibration stopped (cancelled): nothing
written", final ACK of command 241 CANCELLED, `MAG_OFF*` 0. Then `param reset-calibration --yes`. With
`inject-mag-rotation`: ACCEPTED, "compass cal: off 10 -20 30 r 50 50 50 uT", `MAG_OFFX/Y/Z` = 10, -20, 30;
the `[STAT] MAG:` field of the next line moved from (-32.93, 13.03, 37.86) to (-42.66, 32.93, 7.88) uT, the
offsets less the noise of a single sample (about 0.3 uT); after a restart `MAG_OFF*` read 0 (not saved).
Six rows Ok, 0 errors, watchdog stalls begun 0, no reset throughout.

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
| `device restart IMU1`, `IMU2`, `BARO`, `MAG`, `GNSS`, `BATT` (every row of the registry) | raises the barometer's RESTART request; the loop feeds `Event::Restart` (from any state) and starts the device again; ACCEPTED when the device reaches Ok, FAILED when it reaches Failed or Absent first (the book's own 5 s limit covers a hang); the action records the number of the restart request it raised and ends only on that request's outcome, so an older restart finishing late never ends a later action; progress stays 0 |
| `param reset-all --yes` | `Params::reset_all`; each device whose values changed gets its APPLY request; ACCEPTED as soon as the store is reset and the requests are raised, without waiting for the applies (as a parameter write does not wait; a device that is Failed or Recovering picks the values up when it is restarted) |
| `param reset-calibration --yes` | `Params::reset_calibration`, the same way: the twelve `IMUx_*_OFF*` and the three `MAG_OFF*` go to 0 and the IMU offsets are pushed to the flight core; ACCEPTED |
| `param save` | writes the parameters that differ from their defaults to the flash settings region (see Settings in flash); ACCEPTED when the record was read back and verified, FAILED with one text otherwise ("armed", "flight core not answering", "verify failed") |
| `param reload` | the boot rule again from flash; the devices whose values changed get their APPLY request; ACCEPTED |
| `reboot --yes` | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1 = 1: the action writes the REQUESTED scratch record and ends ACCEPTED; when the final ACK is handed to the link the watchdog monitor resets the board (`trigger_reset`) 100 to 200 ms later (1 s after the action when no ACK could be handed). Nothing is saved: an unsaved parameter change is gone after the reboot. The boot report then says INFO "reboot: requested" |
| `calibrate gyro`, `accel`, `compass`, `calibrate cancel` | see Calibrations |
| `device selftest <NAME>` | admitted, then FAILED at once with ONE STATUSTEXT "<what>: not built yet" (for example "self-test: not built yet"). No fake success |
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
| adc-battery | <https://github.com/Lmx154/adc-battery> | battery voltage and current from ADC sums (pinned at 3d8ad8e, its `main`) |
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
"MARV V2 fw 0.1.0: flight core up, reset: power-on" (the last reset's cause: `timeout` or `forced` when the
watchdog caused it, `power-on` otherwise, which includes a debugger reset by `tools/run.sh`).

Negative control: `FEATURES=no-fbeat tools/run.sh 30` builds with the cargo feature `no-fbeat`: core 1
starts but its liveness task is not spawned. Expected: `[FAIL] PONG follows PING`, the log line and the
WARNING text "flight core not answering", and `param save` ends FAILED "save failed: flight core not
answering" with nothing written. The hardware watchdog is NOT started in this build (`[INFO] watchdog NOT
started ...`): the flight core never answers, so it would reset the board in a loop. The monitor still runs
without effect (it logs the flight-core stall once) and carries out a reboot. Measured 2026-11-15: 60 s, no
reset, all three of the above. Run `tools/run.sh` again afterwards to put the normal build back on the
board.

## Watchdog

`src/watchdog.rs` with the pure rules in `fw_core::watchdog` (design: two-core amendment, item 6).

- Started first thing in `main` on core 0: 1600 ms, `pause_on_debug(true)` (paused while a core is halted by
  the debugger or the debugger is on the bus). Fed ONLY by the monitor, a task alone on an `InterruptExecutor`
  on `SWI_IRQ_1` (priority 1, above core 0's thread mode). Every 100 ms it reads `BEAT0` (bumped every 50 ms by
  `beat0` on core 0's thread-mode executor), `PONG` (the flight core's copy of the supervisor's 10 Hz `PING`)
  and `SAVE_EPOCH`, and feeds while both heartbeats changed less than 200 ms ago. No device task feeds it.
- At 200 ms of stall it writes the scratch record once (STALL_CORE0 or STALL_FLIGHT, the task that ran last
  on that core from `RUNNING0` / `RUNNING1`, the stall's age, the uptime) and stops feeding: the board resets
  1.65 to 1.8 s after the stall began. A stall that ends first clears the record (not while a reboot is pending: that record is the REQUESTED one); the supervisor logs it
  afterwards and, when it reached 500 ms, sends one WARNING text "core 0 stalled 1.0 s (BARO)". The check
  after a flash save (`SAVE_EPOCH` changed) never starts a stall.
- What resets the board: core 0's thread-mode executor held (a task that never yields), the flight core held,
  core 0 with interrupts off or a hang inside a critical section (reset with no record), a panic (it spins
  with the watchdog unfed: no record), `reboot --yes`. What does not: a device stuck in an await, a device
  that fails (its health goes to Failed, the executor runs on), a flash save (about 45 ms), a stall shorter
  than about 1.6 s.
- Task ids (one table, `fw_core::watchdog::task`, used by the record and the boot report): core 0 MAIN, USB,
  LINK, SUPERVISOR, BEAT, ACTION (a command's action inside the link task, so a settings save or reload runs under it), BARO, TEST (a check
  feature's task); flight core FBEAT, IMU1, IMU2. Each task stores its id when it resumes (`fw_flight::tagged`, a
  plain atomic store on every poll).
- The boot report, read once at start (then the record's magic is cleared), one `[INFO] boot report` log line
  and a text held until a ground station's node runs: time-out + a stall record = CRITICAL "WDG reset: BARO
  stalled (core 0)" or "WDG reset: IMU1 stalled (flight core)"; time-out + no record = CRITICAL "WDG reset: no
  record"; forced + REQUESTED = INFO "reboot: requested"; forced + no record = WARNING "reset: forced, no
  record"; power-on or a debugger reset = nothing. A station tells that a reset happened from the boot text
  line ("..., reset: timeout"), from `time_boot_ms` of SCALED_PRESSURE (and `time_usec` of HIGHRES_IMU)
  going back to near zero, and from the USB device re-enumerating. Nothing is kept in flash.
- Under `probe-rs run` a watchdog reset ends the probe session ("Firmware exited unexpectedly: Exception"): the
  firmware boots again from flash on its own and keeps running; its evidence is then read with `marv-cli`.
  A flash by `tools/run.sh` leaves the scratch registers at 0 and the reset reason empty.
- Every 10 s: `[STAT] watchdog: feeds N, stalls begun B, ended E, checks after a save S`.
- A save under the watchdog (2026-11-15, normal build, `param set BARO_IIR_T 3`, `param save`,
  `param reset-all --yes`, `param save`): both ACCEPTED, stalls 42.1 ms and 40.7 ms, `checks after a save
  2`, `stalls begun 0`, feeds 100 per 10 s (no check went unfed), no reset.
- 100 saves in a row under the watchdog (2026-11-15, `save-loop`, run by the owner): 100 saves, 100 verified,
  stall 38.7 to 45.5 ms (mean 41.8 ms), 0 device transitions, `feeds 1900` at 190 s (100 per 10 s throughout),
  `stalls begun 0`, `checks after a save 100`, no reset; IMU1 and BARO 0 errors (invariant I6.3 on the board).
- A power cycle (2026-11-15, the owner unplugged and replugged the board): the saved record was loaded
  (`BARO_ODR_HZ` 25 from the start), the boot text ends "reset: power-on" and there is no watchdog or reset text.
- A stuck device (2026-11-15, `inject-baro-stuck`, 75 s): "BARO: stuck" (ERROR), the row FAILED/STUCK, no
  stall, no reset; IMU1 Ok with 0 errors at 803.5 samples/s throughout.

What each cargo feature does under the watchdog:

| Feature | Under the watchdog |
|---|---|
| `no-fbeat` | the watchdog is not started (a pre-watchdog negative control); no reset |
| `inject-baro-*` (transient, persistent, missed-edge, silent, verify, por, fail-until-restart, stuck) | device faults inside awaits: the executor runs on, the watchdog stays fed, no reset (stuck: measured) |
| `inject-imu1-persistent`, `inject-imu1-missed-edge`, `inject-imu2-persistent`, `inject-imu2-verify` | IMU faults on the flight core; `fbeat` still answers, no reset |
| `stall-core0` | 1.2 s spin: a "core 0 stalled" text, no reset; once per power-up or flash |
| `f4-negative` | a DMA channel built on core 0 before the check: `[FAIL]` line only, no stall |
| `save-loop` | 100 saves; no stall, no reset (measured); once per power-up or flash |
| `fake-armed`, `corrupt-newest`, `guard-negative`, `save-foreign` | boot-time settings checks (corrupt-newest programs one byte at every boot); no stall |
| `hang-core0`, `hang-flight`, `spin-1s`, `hang-irqoff` | the watchdog checks below, each once per power-up or flash |
| `wdg-scratch-probe` | no monitor; at most two resets by design |
| `inject`, `inject-imu1`, `inject-imu2`, `inject-imu` | internal, switched on by the features above |

Check features (each acts once, 20 s after a start the watchdog did not cause, so not again after the reset
it provokes; `tools/run.sh` puts the normal build back):

- `FEATURES=wdg-scratch-probe`: at most two resets: writes a record, lets the watchdog time out, reads it
  back (text "probe: kept after TimedOut reset"), writes another, forces a reset 30 s later, reads it back
  ("probe: kept after Forced reset"). Result 2026-11-15: the scratch registers 0 to 3 survive both.
- `FEATURES=hang-core0`: the barometer's task spins forever: reset, "WDG reset: BARO stalled (core 0)".
- `FEATURES=hang-flight`: IMU1's task on the flight core spins forever: reset, "WDG reset: IMU1 stalled
  (flight core)".
- `FEATURES=spin-1s`: the barometer's task spins 1.0 s: no reset, one text "core 0 stalled 0.9 s (BARO)"
  (the age the monitor saw, in 100 ms steps).
- `FEATURES=hang-irqoff`: core 0 turns interrupts off and spins: reset, "WDG reset: no record".

## Workspace

The repository is a Cargo workspace with four members:

```
.                   the firmware binary (package marv-v2-firmware, the workspace root package)
crates/fw-core      pure no_std logic and the lock-free cross-core cells, tested on the host
crates/fw-flight    the flight-core (core 1) tasks; depends on fw-core and embedded-hal-async only
crates/fw-settings  per device, the parameter table and the pure settings-to-configuration rule; depends on fw-core and the driver crates
```

`fw-core` holds the foundation layer's logic that needs no hardware. It is implemented and tested on
the host (170 tests, and 2 doc tests); the binary uses `xcore::Kick`, `health`, `sys_status`, `params` and `admission` so far (`text_ring` is not wired into the binary yet). Design: `~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`.

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
| `text_ring` | The STATUSTEXT ring for the ground-station links: 8 entries, a read cursor per link (each text reaches each link once, in order), the oldest dropped and counted for a link more than 8 behind, so a slow or absent link holds back nobody; 2 sticky slots (the boot report) kept until every link took them, taken first |

`fw-settings` holds the device-specific pure logic: per device, the parameter table and the pure
`settings_to_config` with its validator (today `baro`, over the `bmp581` driver's `Config`, without the
driver's `defmt` feature). Device-specific pure logic goes here, generic logic stays in `fw-core`. Tested on
the host with `cargo host-test -p fw-settings` (20 tests, the first six: defaults equal `Config::new()` and P = 19 978 us,
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
2 MiB flash, 0x101FE000 to 0x10200000, as the `SETTINGS` region for the settings record (`src/settings.rs`;
the only range the firmware erases or programs). Use (normal build, 2026-11-15): program 171.7 KB of 2040 KB
(.text 154424 B, .rodata 15768 B, .data 1172 B); static RAM 41.3 KB of 512 KB (.data 1172 B, .bss 39080 B,
.uninit 1024 B), of which 12 KB are the settings module's three sector buffers.

## Board documentation

- `docs/handoff.md`: the state of the drivers and the starting point for the firmware work (2026-11-11).
- `docs/handoff-usb-mavlink.md`: the USB link, the MAVLink layer and the ground station, each in its own
  GitHub repository, and how this firmware takes them (2026-11-12).
- `docs/pinout.md`: the board pin assignment.
- `docs/devices.md`: the devices on the board and what was measured on them.

## Licence

MIT, see `LICENSE`.
