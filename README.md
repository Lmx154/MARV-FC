# MARV V2 firmware

Firmware for the MARV V2 flight controller (RP2350B). Version 0.1.0.

Today the binary is the two-core skeleton of the firmware (design: the two-core amendment of
`~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`) with the device layer and its first
devices on core 0, the BMP581 barometer (BARO), the BMM350 magnetometer (MAG), the LC76G GNSS receiver (GNSS) and the battery monitor on the ADC (BATT), the SD card (SD: an ArduPilot-format `.BIN` log from boot; see SD card and Logging) and the radio receiver (RC, report only; see Radio), and IMU1, IMU2 and the four ESCs' DShot stream (telemetry, and the props-off motor test; see ESC telemetry) on core 1; the foundation's hardware-free logic is in `crates/fw-core` (see Workspace).
It has no flight functionality.

- Core 0, the I/O core (`embassy_rp::init`, thread-mode executor): `usb_link`, a MAVLink node (system 1,
  component 1) on the USB port: HEARTBEAT at 1 Hz (status from the device rows through
  `fw_core::sys_status`: BOOT while a row is Starting in the first 3 s, then STANDBY), SYS_STATUS at 1 Hz
  (the three sensor masks from the device rows), `MARV_DEVICE_STATUS` (42421) for every device row once per
  second, ESC_STATUS and ESC_INFO (see ESC telemetry), the parameter protocol over the 92 parameters of BARO, MAG (with the compass calibration's soft-iron matrix and record), BATT, GNSS (its fix interval), the two IMUs' calibration offsets and chip settings, the SD card's `LOG_BACKEND_TYPE`, the RC stick mapping, the sensor orientation, the motors' pole count and the status LED's brightness (see Parameters),
  STATUSTEXTs from the supervisor and from refused parameter writes, and the commands (see Commands).
  `supervisor`, a 10 ms tick: at 10 Hz it bumps `PING` and kicks the flight core, it tracks the age of
  `PONG`, gives the start-up verdicts, runs the devices' stuck check and transition texts, ends the running
  action (`actions::tick`), and writes `[STAT]` lines every 10 s. `beat0` bumps `BEAT0` every 50 ms (read by nobody yet). `baro`, `mag`, `gnss` and `batt`, the four core-0 device tasks (see
  Device layer), and `sd`, the SD card's device task with its own loop (see SD card). `rc_link`, the radio's task (UART0, report only; see Radio). `led`, the status LED's task (PIO1, GPIO43; see Status LED).
- Core 1, the flight core (started with `embassy_rp::multicore::spawn_core1`, 16 KB static stack, its own
  thread-mode executor): `fw_flight::liveness`, which copies `PING` into `PONG` on each kick, IMU1
  (`src/imu1.rs`): the ICM-45686 on SPI0 with DMA channels 0 and 1 and its INT1 on GPIO15, and IMU2
  (`src/imu2.rs`): the LSM6DSK320X on SPI1 (SCK GPIO10, MOSI 11, MISO 8, CS 9) with DMA channels 2 and 3 and
  its INT1 on GPIO7 (INT2, GPIO6, unused), all built inside core 1's entry closure so their interrupts land on
  core 1, each running the flight-core device loop `fw_flight::imu` in its own task (see Flight-core device
  loop), and `esc` (`src/esc.rs`): DShot300 on PIO2 SM0, GPIO36-39, reply capture on DMA channel 5, also built
  inside core 1's entry (see ESC telemetry). The program's one `DMA_IRQ_0` binding (`DmaIrqs` in `src/main.rs`)
  lists the five channels (0-3 and 5). No log, no critical-section primitive of its own, and no
  timer in the sample loop there. Everything it runs, every constant and its vector table are in internal SRAM
  (see Flight core in SRAM).
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
- Between the cores only atomics and `fw_core::xcore` cells (`Kick`, each IMU's `Seqlock` and `Kick` in
  `fw_flight::imu::ImuCells`, and the ESC status `Seqlock`, `esc::ESC`), in internal SRAM. `DMA_IRQ_0` is disabled on core 0 before core 1 starts (all
  DMA belongs to the flight core); start-up checks: not enabled on core 0, enabled on core 1, still not
  enabled on core 0 after core 1 started; `PIO2_IRQ_0` enabled on core 1 after the ESC driver was built there
  (with `esc-off`: not enabled) and not enabled on core 0.

Not in it yet: IMU2's
high-g accelerometer (off; nothing of it on the link), any use of the receiver's sticks (no arming, no failsafe, no rule about which link has authority; see Radio), GNSS's PPS and time synchronisation (the pin is not read), the six-position accelerometer calibration and an on-board fit of the compass's soft-iron matrix (the station fits it and writes it as parameters; see Calibrations).

## Flight-core device loop

`crates/fw-flight/src/imu.rs` (design: two-core amendment): `ImuTask` runs one IMU over an `ImuAdapter`
(the core-0 adapter contract: `start`, `acquire`, `status`, `verify`, `apply`, `recover`, `restart`, success
only after a fresh sample). It waits on `select(data-ready edge, kick)` and never on a timer; core 0's
supervisor kicks CHECK every 10 ms tick while the row's last sample is older than T = max(3 P, 10 ms); the
woken task compares the clock with its last sample and, if late by more than T, makes the one status read
(sample waiting = missed edge, read it; nothing = deadline miss). RESTART comes with its request number
through the same kick; APPLY is reserved. Start and recovery wait through the `DelayNs` passed in, wrapped in
`CountingDelay` (count after the start: 0 in a clean run, invariant F2; a restart or a self-test adds its waits, thousands for an IMU1 self-test, and the count is only printed, never judged on the board); the configuration is checked one
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
(47 tests and 1 doc test, the flight step's included).

The flight step (build step 3 of the GNC plan; design: `~/Documents/MARV-V2-drivers/_plan/decision-sim-seams-20261119.md`).
`fw_flight::step::Flight::step` runs once per good IMU1 sample, on core 1, inside IMU1's task, right after the sample
is published: `ImuTask` hands each new sample to its `SampleSink` (IMU1's is `flight::Stepper`, which owns the
`Flight`; IMU2's is `()`, which does nothing). The stages run in the order of the decision's rule R1 (estimator,
controller reset while disarmed, stick shaper, validity check, controller, allocator, gate); the seams are in
`fw_flight::gnc`, the gate and `MotorCommand` (only the gate can make a spinning one) in `fw_flight::gate`, the
slow-sensor and stick sample types in `fw_core::sample` and `fw_core::rc`. Since step 5 (P4b, 2026-11-19) the step runs
the real flight code (`decision-first-pair-20261119.md`): `fw_flight::mahony::Mahony`, the Angle-mode stick shaper
(private, in `step.rs`; collective 0 -> 0, 0.5 -> m g, 1 -> 2 m g, one straight line: `pilot_thrust_max_g`, Amendment C), `fw_flight::pid::CascadedPid`, `fw_flight::alloc::CardAllocator` and the gate's arming and
RC-loss rules (section 5: an off-to-on switch cycle on fresh reports with throttle low, attitude valid, lean within
`ANGLE_MAX`, levels present and `HANDSHAKE.try_arm()` granted; Stop and disarm when the newest report is older than
`RC_FS_TIMEOUT`, on switch off, or on a refusal while armed). The board image names them in `src/flight.rs`
(`BoardFlight`). Since P6 the image spins the motors when armed (see Flying, props off, below). IMU1's sample, the
newest CRSF stick report and the newest barometer, compass and GNSS samples go in (IMU2 comes with a later step).
The slow sensors (`decision-nav-eskf-20261120.md` section 4, packet F1): core 0's device loop, when it publishes a good
sample to the row's `Watch` (`device::read`), hands it to the adapter's `DeviceAdapter::to_flight` hook; BARO, MAG and
GNSS turn the reading into its `fw_core::sample` sample (`BaroSample`, `MagSample` with the calibrated field m, `GnssSample`
with everything `Reading::Gnss` has, deviations included), each with the core-0 board time and `seq` of the publish, and
write it to the triple buffer `flight::FLIGHT_BARO`, `FLIGHT_MAG` or `FLIGHT_GNSS` (the stick path's primitive, wait-free
on both sides). `Stepper::sample` reads the three once per step and gives `Sensors.baro`, `.mag`, `.gnss` only the
sample that is new since the previous step (newest only; a gap shows in `seq`). The read copies the three slots every
step (24 + 32 + 104 bytes), no lock and no wait; Mahony ignores the three samples, so the step's outputs are the same
bit for bit with and without them (test `slow_sensor_samples_do_not_change_a_mahony_step`).
Ground rule (`decision-flight-ground-20261119.md`, section 1; approved by the owner on 2026-11-19): while the collective is below `ground_thrust_g` x m g (0.5/0.7, the vehicle on the ground) `fw_flight::pid::CascadedPid` sets the three rate integrators to 0 every step; above it nothing changes.
`Flight::new` also takes the gains and limits (`fw_flight::tuning::Tuning::DEFAULT`, the numbers of
`decision-first-pair-20261119.md` section 13) and `settings::HANDSHAKE`, where the gate stores the armed state for
core 0; before each step the estimator is given the previous step's armed state (`Estimator::set_armed`).
Attitude estimator (`fw_flight::mahony::Mahony`, `decision-estimator-accel-20261120.md`, approved 2026-11-20): Mahony's filter on IMU1, tilt only, Kp 0.25 armed and 2.5 disarmed. The accelerometer goes through a PT1 low-pass (`Tuning::ins_accel_lpf_hz`, default 10 Hz, seeded with the first sample) before anything is decided on it: the 0.9 g to 1.1 g window and the direction used for the correction both come from the filtered vector, never from a raw sample (selecting and normalising raw samples slid the tilt by up to 18 deg on the ground with the motors running). The gyro is integrated raw. Test `tilt_holds_on_the_ground_with_the_motors_running` replays `crates/fw-flight/testdata/ground-run-20261120.bin`, a recording of the first flights' ground run (log 182, IMU1, 23.0 s to 40.0 s, motors spinning up, vehicle not moving; 12 075 records of 33 bytes, little-endian: u64 t_us, f32 gyro x y z rad/s, f32 accel x y z m/s^2, u8 armed; IMU and armed flag only, one particular vehicle): from the first armed sample the tilt stays within 0.5 deg of the tilt of the mean of all accelerometer samples (about 0.08 deg roll and 0.11 deg pitch). The unit test holds 2 g and 0.5 g along the mirrored tilt, and 0 g, for 0.25 s from a converged estimator: the attitude stays within 0.1 deg only while the window rejects the filtered vector. After a zero first sample the tilt becomes valid only when the filtered |f| reaches 0.9 g (about 37 ms at 10 Hz); `step.rs`'s test pins that delay.
`fw_flight::step::encode_step` writes one step as the records NAV, SETP, DMD and MOT (`fw_core::dflog::FLIGHT_TYPES`,
ids 139 to 142; the board logs them, see Logging).
After each step the task writes the step time and the command to the seqlock `flight::OUT`. The ESC task reads it at
every exchange (`try_read(1)`; a failed read sends Stop and counts `flight_read_failures`) and passes it to `Esc::frames`,
which applies `fw_flight::esc::hold` (Stop when the command is older than 10 ms) and, while the held command spins, sends
its throttles on all four lines, no EDT burst, and ends a running motor test as `Replaced`. There is no status line for it. A core-0
`[STAT] FLT:` line was tried on 2026-11-19 (it showed 8035 steps per 10 s, the IMU1 rate, `command Stop`) and removed:
with the probe attached, that one extra log line raised the ESC task's longest spacing from 1.09 to 1.26 ms (six runs
without it) to 1.58 ms inside the 10 s `[STAT]` burst and 1.36 ms on a tick of its own; without the line it is 1.20 ms.

IMU1's configuration comes from `IMU1_ACC_FS`, `IMU1_GYR_FS` and `IMU1_ODR_HZ` at boot, fixed until the next reboot; the
defaults are 16 g, 2000 dps, 800 Hz on both sensors (`fw_settings::imu::imu1_setup`: read once in `main` after `settings::boot`, logged as
`[INFO] IMU1: 16 g, 2000 dps, 800 Hz (until the next reboot)`, and moved into core 1's entry by value; its sample period is the row's P and the task's),
internal clock (the base of `icm45686_smoke` on this board), data-ready on INT1 in latched mode (high until INT1_STATUS0 is read, as `icm45686_int1`
showed), waited for by level, so a sample that arrives while the task is busy is not lost; 800 Hz because
the ICM-45686 has no 1 kHz rate. Single-sample mode
(`read()` on every data-ready signal, the driver README's single-sample path), with INT1_STATUS0 (clear on
read) read with every sample so that the status read after a late CHECK sees a fresh flag; no FIFO, so
the overflow count stays 0. Its values go to the station as HIGHRES_IMU (id 0, acceleration, gyro,
temperature, the other fields NaN with their `fields_updated` bits clear; the gyro and acceleration have the `IMU1_GYR_OFF*` and `IMU1_ACC_OFF*` offsets subtracted, which is nothing while they are 0) at 25 Hz; the magnetometer's field in gauss (1 G = 100 uT) as `xmag`, `ymag`, `zmag` with their `fields_updated` bits
set only while the latest MAG sample is younger than 1 s (the same rule for the IMU1 fields: NaN and clear bits when older; SCALED_PRESSURE is not sent when its sample is older than 1 s); the barometer's as SCALED_PRESSURE at 25 Hz; the receiver's as GPS_RAW_INT (id 24) at 1 Hz (see GNSS below). SYS_STATUS carries
`voltage_battery` (mV) and `current_battery` (cA) of the latest BATT sample while it is younger than 1 s (65535 and -1, MAVLink's "unknown", while there is none or it is older);
`battery_remaining` stays -1.

IMU2's configuration (`src/imu2.rs`) comes from `IMU2_ACC_FS`, `IMU2_GYR_FS` and `IMU2_ODR_HZ` at boot, fixed until the next reboot; the defaults
are 16 g, 2000 dps, 800 Hz (`fw_settings::imu::imu2_setup`, handed over as IMU1's): both sensors at the chosen rate, all through the high-accuracy ODR set
"800" (800 Hz is the ODR code that is 960 Hz in the normal mode), as `lsm6dsk320x_modes` (check 4) ran it on this
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

Measured with both IMUs at the defaults (16 g, 2000 dps, 800 Hz) and the incremental verify (2026-11-17, 600 s clean run, log
`marv-v2-firmware-20261117-141123.log`, the 10 s to 610 s lines; taken with an earlier build whose chip settings for these values are the same): every device row Ok with no transition after the start, 0 errors, timeouts, missed edges, overflows and recoveries, delay count after the start 0,
watchdog stalls begun 0. IMU1 803.56 samples/s (its own rate: nothing lost), largest gap 1697 us, 0 gaps over 1.5 P, verify steps 18 to 276 us, 618 passes; IMU2
800.71 samples/s (bench 800.664 Hz), largest gap 1781 us, 0 gaps over 1.5 P, steps 13 to 138 us, 610 passes. Other configurations (4 g / 250 dps; 4 g / 500 dps; 400 Hz) are in
`docs/handoff.md`, "IMU configurations on the board". History, not today's figures: the 10-minute run of 2026-11-15 (log `marv-v2-firmware-20261115-114930.log`), taken on
an older firmware at 4 g / 250 dps, read IMU1 803.52 samples/s, largest gap 2442 us, 110 gaps over 1.5 P (10 across a step), steps 134 to 946 us, 608 passes, and IMU2 800.48 samples/s,
largest gap 3410 us, 1066 gaps over 1.5 P (79 across a step), steps 84 to 489 us, 600 passes; what changed the gap count is not isolated. Gaps and long polls are REPORTED, not a pass criterion: the
data-ready, one-read-per-sample scheme is Betaflight's, which loses a sample when the reader is late and does
not count it; ArduPilot and PX4 avoid the loss with the chip's FIFO (kept in reserve here). An IMU row is
accepted when it is Ok with 0 errors, timeouts and recoveries and its rate is within 1 % of the chip's own.
In those runs of 2026-11-15 the longest poll was 1.06 to 2.56 ms (IMU1) and 0.63 to 1.36 ms (IMU2) in every 10 s window, with
single-register steps, so it is not `verify`; while IMU2 was parked IMU1 had no new gap but still polls of
about 1 ms. Three 620 s runs of that time (logging as usual, `DEFMT_LOG=off`, `quiet-stat`) gave IMU1 63 / 37 / 42 gaps and
IMU2 379 / 279 / 199: logging on core 0 is NOT the main cause. The cause is not found; it belongs to the
measurement of what disturbs the flight core, to be done before any control loop runs there (the features
`latency-text` and `quiet-stat` and the per-poll latency figure exist for it). Facts and suspects:
`~/Documents/MARV-V2-drivers/_plan/facts-foundation.md`, "Two IMUs on the flight core".

Features: `FEATURES=inject-imu1-persistent tools/run.sh 120`: 5 s after the first good sample every
`acquire`, `recover` and `restart` fails until `marv-cli device restart IMU1`; `inject-imu2-persistent` the
same for IMU2 (`device restart IMU2`); `inject-imu2-verify`: one IMU2 verify step answers "mismatch" once
(reconfigure, `config restored` 1); one wrapper serves both IMUs (`src/imu_inject.rs`). `FEATURES=inject-imu1-missed-edge
tools/run.sh 60`: 5 s after the first good sample the data-ready edges are swallowed for 200 ms. With the feature
`inject-late` added (`FEATURES=inject-imu1-missed-edge,inject-late`) every IMU injection starts 60 s after the first
good sample instead of 5 s, so the vehicle can be armed first (props off). The injection state goes out through the cells and core 0 logs it. `FEATURES=stall-core0 tools/run.sh 60`: one
core-0 task spins 1.2 s without yielding 20 s after start and logs IMU1's and IMU2's samples before and after (F1).
The watchdog survives that spin (fed again before 1.6 s without a feed; a "core 0 stalled" text, no reset); the
feature does not act after a watchdog reset, so it does not loop.
`tools/run.sh` opt-ins (default off): `NO_CATCH_HARDFAULT=1` passes `--no-catch-hardfault` to `probe-rs run` (a fault does not halt the run; the watchdog can reset the board); `ATTACH=1` uses `probe-rs attach` (no flash, no reset: reads the RTT log of the image already running, built with the same `FEATURES`).
`FEATURES=f4-negative tools/run.sh 20`: a DMA channel (6, since 2 and 3 are IMU2's) is built on core 0 before the start-up check
(negative control of F4: `[FAIL] DMA_IRQ_0 not enabled on core 0`), its line disabled again before core 1
starts.

## Device layer

`src/device.rs` (design item 1, on core 0): each device has a task running the one generic `run_device`
loop over a firmware-side `DeviceAdapter` (`data_ready`, `start`, `acquire`, `status`, `verify`, `apply`,
`recover`, `restart`, `counters`, `self_test`: implemented for IMU1, IMU2 and MAG, "unsupported" for the rest). The loop steps the device's
`fw_core::health::Health` and carries out its action. Sampling is `select(with_timeout(T, data ready),
wake)` with T = max(3 P, 10 ms) (GNSS: its own 2.5 s); a timeout makes ONE status read: data waiting = a missed edge, read it;
nothing = a deadline miss. The device loop awaits every bus call to completion, never inside a `select` or a
timeout of its own; the one deadline is inside the I2C bus wrapper of BARO and MAG (see the paragraph "A hung I2C transfer" below).
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
`Reading` variant. `Reading::Baro(BaroReading)`, `Reading::Mag { reading: MagReading, raw_ut }` and
`Reading::Gnss(GnssReading)` carry the flight code's types from `fw_core::sample` (one field set for both cores);
`Batt` and `Imu` have their own fields. Every adapter has the `to_flight` hook, called once per published sample (a
wrapper such as fault injection passes it on; BATT does nothing).

| Row | Device | Bus | Configuration | P | T | Stuck after |
|---|---|---|---|---|---|---|
| BARO | BMP581 (`src/baro.rs`) | I2C1 at 400 kHz, SDA GPIO22, SCL GPIO23, data-ready GPIO21 (rising edge), no DMA | from the parameters (below); the defaults are the driver's default: pressure 16x, temperature 2x, normal mode, 50.056 Hz output data rate, no filter; always no FIFO, data-ready pulsed | 19 978 us at the default; 1 / rate afterwards | max(3 P, 10 ms): 59.9 ms at the default | max(1 s, 20 P) |
| MAG | BMM350 (`src/mag.rs`) | I2C0 at 400 kHz, SDA GPIO12, SCL GPIO13, data-ready GPIO4 (rising edge), no DMA | from `MAG_ODR_HZ` and `MAG_AVG`; defaults = the driver's bench config: 100 Hz, averaging 2, all axes, normal mode, data-ready pulsed, active high, push-pull | 10 000 us at the default; 1 / rate afterwards | max(3 P, 10 ms): 30 ms at the default | max(1 s, 20 P) |
| GNSS | LC76G (`src/gnss.rs`) | UART1 (`BufferedUart`, interrupt driven on core 0, no DMA) 115200 8N1, board TX GPIO24, RX GPIO25, RX buffer 4096 B; PPS GPIO5 not read | from `GPS_RATE_MS` (default 200 ms, 5 fixes per second): fix interval and message set (GGA, RMC and GST every fix, GSA at most once per second, the rest off, EPE off) sent at every start, restart and apply; volatile, never saved in the receiver | 1 000 000 us (1 Hz) whatever the rate | 2.5 s (own deadline, not 3 P) | 20 s |
| BATT | the RP2350 ADC (`src/batt.rs`) | ADC on core 0 without DMA: GPIO42 (ADC2, battery voltage, 100k / 10k), GPIO41 (ADC1, ESC current output, no pad pull); `ADC_IRQ_FIFO`; no bus, no chip | `BATT_V_SCALE`, `BATT_A_SCALE`, `BATT_A_OFFSET`, read for every sample | 100 000 us (10 Hz) | 300 ms (3 P) | 2 s |
| SD | microSD card (`src/sd.rs`, own loop over the same `Health` machine; see SD card) | PIO0 SM0 (`pio-sdmmc`), CLK GPIO32, CMD 33, DAT0-3 28-31, DMA channel 4 completing on `DMA_IRQ_1` on core 0, `PIO0_IRQ_0` | `LOG_BACKEND_TYPE` (read at boot) | 1 000 000 us (the writer's 1 s: flush, longest wait) | none (no data-ready) | 20 s |
| RC | ELRS receiver (`src/rc.rs`, task `rc_link`; see Radio) | UART0 (`BufferedUart`, interrupt driven, `UART0_IRQ`), TX GPIO14, RX GPIO17, no DMA | none: report only, the port follows CRSF and MAVLink by itself | none | none | none (not a supervised row: not in `DEVICES`) |

**MAG.** `acquire` reads INT_STATUS (clear-on-read; in the driver's pulsed mode it also clears itself 1.25 ms after the edge, so after a
data-ready timeout `status` finds "nothing pending" and a missed edge shows as a timeout, not as a missed edge: accepted, the bench-proven pulsed mode is kept and latched mode is not used), then `read_field`: the
compensated field in microtesla and the die temperature, rotated by `MAG_ORIENT` (r), then the compass calibration m = S (r - o) in software
(see Calibrations; the chip's offset registers are not used). `Reading::Mag { reading: MagReading { field_ut, temperature_c }, raw_ut }`: m and the
temperature (what the flight core gets, as `MagSample` through `flight::FLIGHT_MAG`), and r (core 0 only: the compass calibration reads it). The BMM350 has no power-on-reset flag: a reset shows as a `verify` mismatch.
`verify` and `recover` are the driver's, `apply` is `configure`, `restart` does the bus clear (`src/busclear.rs`, shared with BARO) and
rebuilds the I2C0 bus. Sample rate: the chip's own oscillator, measured 0.7 % fast on the bench unit (100.7 Hz at the 100 Hz setting).
The field at rest on the driver's bench was about 66 to 67 uT.

**GNSS.** A row for the receiver, `src/gnss.rs`, the `lc76g` driver as it is (rev 2ee54f9), UART1 read in thread mode through the driver's
`read`. **The firmware configures the receiver's rate and messages at every start and never saves in the receiver**: at every start (after the
first valid sentence), restart (after the flush) and apply (a `GPS_RATE_MS` write) it sends exactly `fw_settings::gnss::commands`: the version
query, fix interval 1000 ms, the ten sentence rates, EPE off, then the interval of `GPS_RATE_MS` last if it is not 1000 ms; each is acknowledged,
the list stops at the first failure and runs under one deadline T (2.5 s). The last interval is held until `FIX_INTERVAL_SETTLE_MS` (500 ms)
after the first interval's acknowledgement (`fw_settings::gnss::not_before_ms`, host-tested with a scripted receiver): on the bench
(2026-11-20) a `$PAIR050` sent 18 to 170 ms after one that changed the rate was answered `$PAIR001,050,1` (being processed) and the receiver
then restarted by itself about 1.8 s later at its stored settings (8 of 8: every change between two rates other than 1000 ms, and a warm
reset of the board while the receiver ran faster than 1 Hz); from 207 ms on it was accepted (7 of 7). A configuration now takes about 0.55 s. Never `init` (it writes the navigation mode), PPS, a baud change, a
restart or a save: the settings are volatile, the receiver's stored configuration does not change, and a receiver that resets on its own comes
back at its stored settings until the next start, restart or apply. The row then sees GGA, RMC and GST every fix and GSA at most once per second.
Success: `[INFO] GNSS: configured: <ms> ms, GSA every <n>, GST every fix, other sentences off, EPE off, volatile (not saved), <version>, <t> ms`.
A failure is a WARNING `GNSS: rate not applied (<reason>)` (STATUSTEXT and defmt), never a start or restart failure; `apply` returns one bus
error; no retry until the next start, restart or apply. PPS (GPIO5) is not configured or read: neither the driver
nor its bench program does anything with it beyond counting edges, and there is no time synchronisation here.
- **Cancel answer (the design's "GNSS only after the `lc76g` cancel check").** Dropping the driver's `read`, `read_fix` or `read_sentence` future is
  SAFE: nothing is lost, no sentence is corrupted, and `recover` is not needed afterwards. Evidence: the driver README, "Dropped reads"
  (`lc76g/README.md:111`, "dropping a `read*` or `command` future loses nothing the driver already read"), the host test
  `dropping_a_read_future_loses_nothing` (`src/tests.rs:1013`: a fix cut in the middle, the future dropped, the rest fed, the next `read_fix`
  returns the right fix and 6 sentences are counted), the code (`Lc76g::next_sentence`, `src/lib.rs`: the driver's own buffer, position and
  framer hold everything; the only await is one `uart.read`, and `BufferedUart::read` copies its bytes in the poll that completes it, so a drop
  can only come before bytes were taken), and the board (`lc76g_resync`, README line 172: 10 Hz fixes read under 20 ms timeouts, 100 of 100 fixes
  kept over 444 dropped calls). A half sentence stays in the framer and is finished by the next read. Only a `command` write is not droppable
  (a partial line on the wire; the module discards it by its checksum); only the configuration's deadline can drop one, and the driver
  resynchronises at its next command. **Shape that follows**: the data-ready wait
  IS the read of the next fix (`data_ready` reads sentences, counts them, and returns when the driver has completed a fix, that is a GGA and an RMC
  of one epoch; the fix waits in the adapter; it takes the deviations of its epoch's GST, see below), so the loop's `select(with_timeout(T, data_ready()), wake)` may drop it at any await; `acquire`
  turns the waiting fix into the reading and never blocks.
- **Ok means**: valid sentences keep arriving, WITH OR WITHOUT a position. One sample is one complete fix epoch (GGA + RMC of the same UTC time);
  the position fields can be empty. 2.5 s without one is ONE error event (TIMEOUT; the row's own deadline T = 2.5 s, Betaflight's figure, instead of
  3 P = 3 s): the score is 3 per event and 9 enters Recovering, so Degraded after 2.5 s, Recovering after 7.5 s of silence (measured from the last
  good sample, see the injection run), as the design says. A checksum error, a malformed sentence, a sentence over the length limit and a UART
  error (overrun, framing, break, parity) are counted (`[STAT] GNSS:`) and are NOT error events: the driver has no "stream broken" signal, and a
  stream that is broken shows as silence. `status` answers "nothing pending" (a deadline miss); `verify` reads nothing back (a query every
  second would discard fixes); `apply` sends the configuration.
- **`start`** waits up to 5 s for the first valid sentence of any type (bench: 367 to 424 ms after the start of the task), then sends the
  configuration (at most T more); no identity query before the first sentence. The README of the driver gives no start time of the module, so 5 s is this firmware's own bound. **How the long
  start fits the health machine**: the machine's start retries wait 100, 300 and 1000 ms, too short for a receiver that is still booting, so the
  waiting is INSIDE `start` (it is awaited to completion, and its 5 s are below the stuck threshold of 20 s); with three retries a receiver that
  never talks is Absent after 4 x 5 s + 1.4 s. `fw-core` is unchanged. The loop then waits for one fresh fix (at most T) as after any start.
- **`recover`** = flush and resynchronise: the UART's receive buffer is read empty (reads until one finds nothing for 5 ms, at most 128), and the driver
  is rebuilt over the same UART (`release` then `new`: no half sentence, no held fix). **`restart`** (the first attempt after an operator request, and
  the automatic recovery's second step) is the flush followed by the configuration: a UART has no bus clear
  (noticed: the UART block is not re-initialised). The driver's counts restart at 0 with the new driver; the row's totals add them up.
- `Reading::Gnss(GnssReading { fix_type, lat_e7, lon_e7, alt_mm, speed_mm_s, course_cdeg, sats_used, hdop_x100, vdop_x100, time, date, lat_std_mm,
  lon_std_mm, alt_std_mm })` (`fw_core::sample`; `time` and `date` are the driver's UTC time and date copied once into `UtcTime` and `UtcDate`), the driver's integer
  units (1e-7 degree, mm, mm/s, 0.01 degree, DOP x 100; no `f64` anywhere: the build has 0 `__aeabi_d` symbols). `fix_type` is MAVLink's
  `GPS_FIX_TYPE`: 1 (NO_FIX) unless the RMC status, the GGA quality and the GSA fix mode all say there is a position, then 2 (2D), 3 (3D) or 4
  (the GGA quality says differential or SBAS-aided, 3D). Position, altitude, speed and course are `None` while `fix_type` is 1.
  `lat_std_mm`, `lon_std_mm`, `alt_std_mm` (mm, 1 sigma) come from the GST of the same epoch only (its UTC time equals the fix's,
  `fw_core::sample::same_epoch_sigmas`): a GST that came before the GGA + RMC pair goes with the fix at once; otherwise the fix waits for the one
  sentence after the pair and goes without deviations (`None`) if that is not its GST. A deviation never goes to another epoch.
- **GPS_RAW_INT (id 24), default 1 Hz** (the slot's minimum interval is 100 ms). With a fix: `lat`, `lon` in 1e-7 degree, `alt` the altitude above
  mean sea level in mm, `eph`/`epv` HDOP/VDOP x 100, `vel` in cm/s, `cog` in 0.01 degree, `satellites_visible` the satellites USED (the GGA's count;
  the GSV's count of visible satellites is not read), `time_usec` the board time of the sample (not UTC; UTC is in the reading and the `[STAT]` line),
  `h_acc` sqrt(lat_std^2 + lon_std^2) and `v_acc` alt_std, mm, 0 when not available.
  Without a fix: `fix_type` NO_FIX, `lat`, `lon` and `alt` 0 (the dialect defines no other convention: it says nothing about a missing position),
  `vel` and `cog` UINT16_MAX, `eph`/`epv` whatever the receiver says about its DOP or UINT16_MAX, `satellites_visible` its count or 255. The
  1 s age rule of the other values is replaced by a 3 s rule: no message is sent while the latest sample is older than 3 s (3 fix periods; at
  1 Hz a sample is up to 1 s old when it is read, so 1 s would drop every message). The same sample can go out twice when the stream's tick comes
  before the next fix.
- **Parameter: `GPS_RATE_MS`** (index 80, see Parameters). P stays 1 s, T 2.5 s and the stuck window 20 s whatever the rate.
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
rebuilds I2C1 and runs `init`. Both I2C buses (I2C1 BARO, I2C0 MAG) are also cleared the same way at every boot, before the first transaction, and each logs one line with the SDA and SCL levels before and after and the pulses given: a reset of the MCU in the middle of a transfer can leave the sensor holding SDA low (seen on I2C1 on 2026-11-15, where the barometer's first start then never ended), and the MCU's reset does not release it.

A hung I2C transfer (owner's decision of 2026-11-15, "Yes just do what PX4 and ardupilot do"; it amends the
design's rule I1.1): every transaction of the BARO and MAG drivers runs under a deadline (`src/i2cbus.rs`,
`Timed`, a wrapper around the embassy-rp bus the driver is given), ArduPilot's formula from
`I2CDevice::_transfer`, 1 + 2 * (((8 * 10^6 / bus clock) * bytes) / 1000) ms with its 4 ms floor
(`fw_core::i2c::timeout_ms`; 4 ms for every transfer of these drivers at 400 kHz). A deadline that a flash save
overlapped is discarded. A transaction that misses it is abandoned and the driver gets a bus error (row error
kind BUS), logged as `[INFO] I2C1: transaction of N bytes timed out after 4 ms ...`; the bus is then refused
(every further transaction fails at once with the same error) until it has been cleared and rebuilt. The
health machine and its ladder are unchanged: the errors take the row Ok -> Degraded -> Recovering; `recover`
fails at once on a refused bus, so the attempt goes on to `restart` (bus clear, rebuild, start); a start retry
at boot on a refused bus does the same clear and rebuild first. There is no retry inside the wrapper. The stuck
check stays as the last resort for anything else that hangs (`inject-baro-stuck`).

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
- `FEATURES=inject-baro-hang-once tools/run.sh 60`, `inject-baro-hang-boot`, `inject-baro-hang-forever`
  (`src/i2cbus.rs` `hang`, BELOW the deadline wrapper, so the deadline is what is exercised; not part of the
  adapter wrapper above): the inner I2C1 transaction never completes, once at the first transaction from 15 s
  of uptime, once at the first transaction at boot, or every transaction from 15 s on (also on rebuilt buses).
  Logs `[INFO] inject: I2C1 transaction never completes from ... ms` for each hang. Seen on the board
  (2026-11-15): `once` (`logs/marv-v2-firmware-20261115-191628.log`): hang at 15007 ms, time-out at 15011 ms
  (4 ms, a 2-byte transaction), Degraded at once, Recovering at 15047 ms (two refused reads), attempt 1 at
  15147 ms: `recover` refused, `restart` = bus clear, rebuild, start, Ok at 15174 ms: 167 ms from the hang to
  Ok, BARO errors 3, recoveries 1, never STUCK; MAG and the other rows Ok, MAG errors 0, watchdog stalls begun
  0, 7 `[PASS]`. `boot` (`...-191738.log`): the first start timed out at 10 ms, the start retry at 111 ms did
  the clear and rebuild first, Ok at 131 ms (no STUCK, not Absent). `forever` (`...-191829.log`): Recovering,
  4 attempts (each a clear, rebuild and a timed-out start) at 100, 300, 1000, 3000 ms, Failed at 19487 ms;
  `device restart BARO` answered `failed` about 1.4 s after the request was taken (the restart and its 3 start
  retries, each a clear and rebuild, then Absent); 9 time-outs in all, never STUCK, the other rows Ok, stalls
  begun 0. Not seen: a real bus stuck in the middle of a run (only this injected hang, which never touches the
  bus, so the state an abandoned embassy-rp transfer leaves in the controller was not exercised on the board).
  The normal build with the deadline (`...-192012.log`, 310 s): BARO Ok 49.90/s and MAG Ok 100.10/s over the
  last 10 s, errors 0, timeouts 0, no `timed out` line (no spurious time-out from the 4 ms deadline), both
  `I2Cx bus at boot` lines, the other rows Ok, 7 `[PASS]`, watchdog stalls begun 0.
  A settings save during a run (`...-194641.log`, the reviewer's run): the save stalled core 0 for 43.5 ms,
  ten times the 4 ms deadline; no `timed out` line, BARO and MAG errors 0 and recoveries 0 before and after
  (the deadline a save overlaps is discarded and the wait goes on with the same transfer). Not seen: a real
  hang right after a save.

- `FEATURES=inject-gnss-silent tools/run.sh 100`: 10 s after GNSS's first good sample the adapter stops seeing sentences (the wrapper swallows the
  fixes; the receiver keeps talking) until a RESTART request (`device restart GNSS`). Shows the long path of a device that stays silent:
  one TIMEOUT error event per 2.5 s (Degraded after the first, Recovering after the third), 4 recovery attempts (each a `recover` then a `restart`, each
  waiting one deadline for a fix), Failed, ONE ERROR text; then `device restart GNSS` -> Ok. Times in the "GNSS on the board" paragraph.

Run `tools/run.sh` again afterwards to put the normal build back on the board.

## Parameters

The parameter tables: BARO (below), MAG, BATT, the IMUs' offsets, SD, RC and the IMU chip settings (after it); `fw_settings::TABLE` joins them in that order (92 entries: 5 + 5 + 3 + 12 + 1 + 44 + 6 + 1 + 1 + 1 + 1 + 1 + 11: `AHRS_ORIENTATION`, `MAG_ORIENT`, `MOT_POLES`, `LED_BRIGHT` and `GPS_RATE_MS` are appended at indices 76, 77, 78, 79 and 80, and the compass calibration's eleven `MAG_DIAG*` to `MAG_HLT_INC` last, indices 81 to 91). The barometer's (`crates/fw-settings/src/baro.rs`, a `static [ParamDef; 5]`), all `I32`, device BARO,
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

`crates/fw-settings/src/mag.rs`, device MAG (`MAG_ORIENT`, below the table, is appended at index 77). The two configuration parameters are the `Configure` class (the whole candidate goes
through the driver's `Config::validate` before the value is stored); the three offsets are `Live` and `calibration`
(`param reset-calibration` resets them); the compass calibration writes them (see Calibrations).

| Name | Type | Default | Range | Means |
|---|---|---|---|---|
| `MAG_ODR_HZ` | F32 | 100 | 1.5625 to 400 | output data rate in Hz: 400, 200, 100, 50, 25, 12.5, 6.25, 3.125 or 1.5625; refused otherwise ("rate not offered by the chip") |
| `MAG_AVG` | I32 | 2 | 1 to 8 | samples averaged: 1, 2, 4 or 8 ("not 1, 2, 4 or 8"); the rate allows at most 1 at 400 Hz, 2 at 200 Hz, 4 at 100 Hz, 8 below ("averaging too high for the rate") |
| `MAG_OFFX`, `MAG_OFFY`, `MAG_OFFZ` | F32 | 0 | -200 to 200 | offset in microtesla o of that axis, subtracted from the field in every sample before S is applied, in the body frame, after `MAG_ORIENT` (by the core-0 adapter, `fw_settings::mag::apply`, see Calibrations) |
| `MAG_ORIENT` | I32 | 10 | 0 to 15 | the external compass's orientation, chip to vehicle: ArduPilot's rotation numbers (`COMPASS_ORIENT`; 0 none, 1 to 7 yaw 45 to 315, 8 roll 180, 9 to 11 roll 180 and yaw 45 to 135, 12 pitch 180, 13 to 15 roll 180 and yaw 225 to 315, `fw_core::rotation`). 10 is `ROLL_180_YAW_90`: the BMM350 mounted +X right, +Y forward, +Z up. `Live`, applied to the next sample, before `MAG_OFF*`; the range is the whole check. Index 77 |
| `MAG_DIAGX`, `MAG_DIAGY`, `MAG_DIAGZ` | F32 | 1 | 0.2 to 5 | the soft-iron matrix S's diagonal (body frame), applied after the offsets: m = S (r - o) (see Calibrations). `Live`, calibration. Indices 81 to 83 |
| `MAG_ODIAGXY`, `MAG_ODIAGXZ`, `MAG_ODIAGYZ` | F32 | 0 | -1 to 1 | S's off-diagonal terms (S is symmetric). `Live`, calibration. Indices 84 to 86 |
| `MAG_CAL_TYPE` | I32 | 0 | 0 to 2 | how the calibration in force was made: 0 none or unknown, 1 quick (best effort), 2 guided (the station's fit). `Live`, calibration. Index 87 |
| `MAG_FLD_UT` | F32 | 0 | 0 to 95 | expected field strength in microtesla (World Magnetic Model at the calibration site); 0 = not set. `Live`, calibration. Index 88 |
| `MAG_FLD_INC` | F32 | 0 | -90 to 90 | expected inclination in degrees, positive down; meaningful only when `MAG_FLD_UT` > 0. `Live`, calibration. Index 89 |
| `MAG_HLT_STR` | F32 | 10 | 1 to 50 | flight health test's strength tolerance in % of `MAG_FLD_UT`; stored only, nothing reads it yet. `Live`, not a calibration. Index 90 |
| `MAG_HLT_INC` | F32 | 10 | 1 to 45 | flight health test's inclination tolerance in degrees; stored only, nothing reads it yet. `Live`, not a calibration. Index 91 |

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

### IMU orientation parameter

`AHRS_ORIENTATION` (`crates/fw-settings/src/imu.rs`, `BOARD`, index 76), `I32`, device 0 (the system; a refused IMU setup never resets it), default 0, range 0 to 15, `Reboot` class: the rotation from the board's frame to the vehicle's, ArduPilot's name and numbers (see `MAG_ORIENT`). Both IMU adapters apply it after the offsets. It is read once at boot and handed to core 1 by value (no cell), and the calibrations on core 0 use the same boot value, so a write applies after save and reboot. Each IMU also has a fixed chip-to-board rotation in code, `fw_settings::imu::IMU1_MOUNT` and `IMU2_MOUNT`, both `ROLL_180` (8): the IMUs' axes are +X forward, +Y left, +Z up, the body's FRD.

`MOT_POLES` (`crates/fw-settings/src/esc.rs`, index 78), `I32`, device ESC (12), default 14, range 2 to 64, `Live`: the magnets (poles, not pole pairs) of each motor, for the rpm of ESC_STATUS (eRPM / (`MOT_POLES` / 2), rounded down). Core 0 reads it for every message, so it never reaches the flight core. Design: `_plan/decision-esc-telemetry-20261118.md`.

`LED_BRIGHT` (`crates/fw-settings/src/led.rs`, index 79), `I32`, device 0 (the system: there is no LED device row), default 1, range 0 to 3, `Live`: the status LED's brightness, 0 off, 1 dim (20 of 255), 2 medium (80), 3 bright (255). The LED task reads it every 260 ms (13 ticks of 20 ms), so a write shows within about a quarter of a second; it is saved only on the user's Save, like every parameter. See Status LED.

`GPS_RATE_MS` (`crates/fw-settings/src/gnss.rs`, index 80), `I32`, ms, device GNSS (5), default 200, range 100 to 1000, `Configure`: the
receiver's fix interval. The validator accepts only 100, 200, 500 or 1000 (text `GPS_RATE_MS: not 100, 200, 500 or 1000`). The message set
follows from it (`fw_settings::gnss::config`): GGA, RMC and GST every fix, GSA every ceil(1000 / ms) fixes, GLL, VTG, GSV, ZDA, GRS and GNS off,
EPE off. A write is sent to the receiver at once (apply; also while armed, as BARO and MAG), and again at every start and restart; it is never
saved in the receiver. A stored record from before this parameter loads it at its default (the record holds values by name; host test
`a_record_of_the_first_80_parameters_loads_gps_rate_ms_at_its_default`). See GNSS.

### SD parameter

`crates/fw-settings/src/sd.rs`, one `I32` parameter, device SD, `Reboot` (read once at boot).

| Name | Default | Range | Means |
|---|---|---|---|
| `LOG_BACKEND_TYPE` | 1 | 0 to 1 | 0: no logging, the SD task never touches the card and the row is Absent with the text "SD: logging off"; 1: the SD card (ArduPilot's name; ArduPilot's other values are not offered) |

### RC parameters

`crates/fw-settings/src/rc.rs`, 44 `I32` parameters, device RC (7), all `Live` (the next computation reads them; no reboot, no
validator). ArduPilot's names for channels 1 to 8 (`RCn_*`) and for the function map (`RCMAP_*`); the table follows SD's, and the six IMU chip settings (below) are appended after it, so no
other parameter changed its index. `RCn_MIN`, `RCn_TRIM` and `RCn_MAX` are calibration results, but they are not `calibration` parameters:
`param reset-calibration` (command 245, param1 = 3) resets sensor calibrations only and leaves them; `param reset-all` (param1 = 2) or a
new radio calibration brings them back. There is no check between parameters (MIN below TRIM below MAX, a unique map): the normaliser gives a value for every in-range
combination, and 0 when MAX is not above MIN. The normalised sticks are REPORTED as MANUAL_CONTROL (69) on USB only (see Radio); nothing
else in the firmware reads them.

| Name (n = 1 to 8) | Default | Range | Means |
|---|---|---|---|
| `RCn_MIN` | 1100 | 800 to 2200 | channel n's value at the low end, microseconds |
| `RCn_TRIM` | 1500 | 800 to 2200 | channel n's centre, microseconds |
| `RCn_MAX` | 1900 | 800 to 2200 | channel n's value at the high end, microseconds |
| `RCn_REVERSED` | 0 | 0 to 1 | 1: the channel is reversed (angle negated; throttle mirrored between MIN and MAX) |
| `RCn_DZ` | 0 | 0 to 200 | dead zone in microseconds: around TRIM for roll, pitch and yaw, above MIN for throttle |
| `RCMAP_ROLL` | 1 | 1 to 8 | the channel that is roll |
| `RCMAP_PITCH` | 2 | 1 to 8 | the channel that is pitch |
| `RCMAP_THROTTLE` | 3 | 1 to 8 | the channel that is throttle |
| `RCMAP_YAW` | 4 | 1 to 8 | the channel that is yaw |

Order in the table: for n = 1 to 8 `RCn_MIN`, `RCn_TRIM`, `RCn_MAX`, `RCn_REVERSED`, `RCn_DZ`, then the four `RCMAP_*`.

### IMU parameters

`crates/fw-settings/src/imu.rs` (`SETTINGS`, indices 70 to 75, appended after RC; the first 70 names keep their indices; the calibration offsets are above). Six `I32`
parameters, all `Reboot` class: core 1 configures the IMUs once, at boot, so a write takes effect after `param save` and a reboot (the notice is
"<NAME>: applies after reboot"; a reboot without a save drops it). IMU rates stop at 800 Hz: one read per data-ready edge, no FIFO. Nothing is saved without `param save`.

| Name | Default | Range | Means |
|---|---|---|---|
| `IMU1_ACC_FS` | 16 | 2 to 32 | accelerometer full scale in g: 2, 4, 8, 16 or 32 |
| `IMU1_GYR_FS` | 2000 | 125 to 4000 | gyroscope full scale in dps: 125, 250, 500, 1000, 2000 or 4000 |
| `IMU1_ODR_HZ` | 800 | 25 to 800 | output data rate of both sensors in Hz: 25, 50, 100, 200, 400 or 800 |
| `IMU2_ACC_FS` | 16 | 2 to 16 | the low-g accelerometer's full scale in g: 2, 4, 8 or 16 |
| `IMU2_GYR_FS` | 2000 | 250 to 4000 | gyroscope full scale in dps: 250, 500, 1000, 2000 or 4000 |
| `IMU2_ODR_HZ` | 800 | 25 to 800 | output data rate of both sensors in Hz: 25, 50, 100, 200, 400 or 800 (IMU2 through the high-accuracy ODR set "800", codes 30 to 960) |

The store checks only the range of each value when it is written ("out of range" above 800 Hz or 16 g on IMU2). The chip's value set is checked at boot: `fw_settings::imu::imu1_setup` and
`imu2_setup` (pure) turn the three values into the chip's configuration, the full scales the adapter scales with and the nominal sample period, and refuse a value the chip does not offer
("not 2, 4, 8, 16 or 32", "not 125, 250, 500, 1000, 2000 or 4000", "not 2, 4, 8 or 16", "not 250, 500, 1000, 2000 or 4000", "rate not offered by the chip"), then the driver's own
`Config::validate` ("combination not allowed by the chip"). The store runs no device validator for a `Reboot`-class write, so such a value is stored and saved like any in-range one; at the next boot the
IMU is put back to its `Configure`- and `Reboot`-class defaults in RAM (its calibration offsets stay, the flash is untouched) with one WARNING text "IMUn: stored settings invalid, defaults" and the
log line carrying the setup's reason. The station offers only valid combinations (its sensor presets are several of these parameters set at once; they are a station feature, nothing of them is in the firmware); the combinations the station's presets use are
measured in `docs/handoff.md`.

Firmware glue: in `main`, after `settings::boot` and before core 1 starts, `store::imu1_setup` / `imu2_setup` give each IMU's setup, the rows' P is set from it, one line per IMU is logged
(`[INFO] IMU1: 16 g, 2000 dps, 800 Hz (until the next reboot)`) and the setup is moved into `imu1::Parts` / `imu2::Parts`; the adapters
configure, scale and time themselves with it (init, `apply`, RESTART, self-test, `verify_every`, the task's period). No cell carries a configuration into core 1, and
`REQ_APPLY` stays reserved for the flight rows, so a change takes effect after save AND reboot.

## SD card (device row 9)

`src/sd.rs` (decision-sd-logging Q1, Q2, Q4 first bullet, Q6; milestone SD2), task `sd` on core 0's
thread-mode executor, tagged `SD` for the watchdog. It owns PIO0 (SM0, the `pio-sdmmc` program), DMA channel 4
(programmed by the driver through the PAC, completing on `DMA_IRQ_1`, enabled only in core 0's NVIC; the waker
static `SD_WAKER` and the binding `DMA_IRQ_1 => DmaIrqHandler<SdWaker>` are in `src/sd.rs`), the card and the
mounted FAT32 volume (`embedded-fatfs` over `BufStream` + `StreamSlice`), and drives row 9 through the shared
`fw_core::health::Health` with its own loop (the row is in `DEVICES`, so the supervisor's stuck check with
P = 1 s, its texts and `[STAT] SD:` line apply). It is also the log writer (milestone SD3, see Logging): the
one consumer of the log ring.

- Start, recovery and restart are the same: a NEW bus (PIO0 and its pins re-borrowed, so a bus the driver marked
  dead after a failed cancel is replaced), `init`, block 0 through `fw_core::mbr::partition1` (MBR partition 1 of
  type 0x0B or 0x0C inside the card), the mount, `LOGS/` opened (created if missing), one bounded listing of
  `LOGS/` for the notice; a recovery or restart un-mounts first. Free space is never asked of the file system
  (the scan can take 68 s): used space is the sum of the files' sizes in `LOGS/`.
- While mounted, the log's writes are the liveness (SD2's once-per-second reads are gone): `samples` = 512-byte
  blocks written since boot, `errors` = writes that failed after their retry, `timeouts` = driver time-outs,
  `overflows` = records the log ring dropped (saturating), `last_sample_age_ms` = time since the last good write.
  A write or flush that fails after its retry is an error event (BUS, or TIMEOUT for the driver's time-outs; a
  time-out that overlapped a settings save is retried with no event).
- Every card and file-system call is awaited to its end; the waits between them (`select` of the 1 s timer and
  the row's wake) are no card calls, and a RESTART request is taken between calls (Q2-I1). RESTART (`device
  restart SD`) is how a changed card is taken up (there is no card detect). SELF_TEST on SD is DENIED by the
  admission ("not supported yet").
- Start-up checks, once, after the first bus is built (`[PASS]`/`[FAIL]`): INTE0 = exactly the flight core's
  channels 0-3, INTE1 = 1 << 4, INTE2 = INTE3 = 0 (Q1-I1); on core 0 `DMA_IRQ_1` enabled and `DMA_IRQ_0` not, and
  `DMA_IRQ_1` not enabled on core 1, which publishes its NVIC bit once through `CORE1_DMA_IRQ1` (Q1-I2).
- In a build with `sd-fat-probe` the module is not compiled, its row is not in `DEVICES` (6 rows) and the probe
  owns the card.

| Condition | State | Last error | Text |
|---|---|---|---|
| init, MBR check, mount | Starting | | |
| clean-up deleting an old log (before the log opens; a long deletion is one call of about 50 s for 161 MB) | Starting | | NOTICE "SD: clean-up, deleting log N", one per deletion |
| mounted, log open | Ok | | NOTICE "SD: 59.5 GiB FAT32, 17 MiB in 2 logs, log 3" (partition size, sum of the logs in `LOGS/`, their number, the new log's number), at every mount |
| card full (the clean-up planner) | Failed | unchanged | ERROR "SD: card full", no log, parked until RESTART |
| `NoCard` on every start try (100/300/1000 ms) | Absent | TIMEOUT | ERROR "SD: no card" (instead of the supervisor's "absent") |
| block 0 not an MBR with a FAT32 partition 1, or a volume that does not mount as FAT32, on every start try | Failed | ID_MISMATCH | ERROR "SD: no FAT32 volume, reformat as FAT32" |
| another start failure on every try | Absent | its BUS or TIMEOUT | the supervisor's "SD: absent (...)" |
| write or flush errors | per the machine and its ladder | BUS or TIMEOUT | the supervisor's texts |
| `LOG_BACKEND_TYPE` = 0 at boot | Absent | NONE | INFO "SD: logging off", once |

The machine itself parks a row whose start retries all failed as Absent with no last error; the SD task shows
the cause of the last failed start instead (Failed and ID_MISMATCH for the wrong format, TIMEOUT for no card),
as the stuck flag is shown as Failed/STUCK. SYS_STATUS: row 9 is `MAV_SYS_STATUS_LOGGING` (as ArduPilot).

On the board (2026-11-15, MARV V2 Rev 1 on USB, the scratch SanDisk 64 GB card, FAT32, 32 KiB clusters; logs
`marv-v2-firmware-20261115-*`): normal build (231006): the five SD checks PASS with the seven earlier ones,
0 `[FAIL]`; card init 47 ms, mount 4.8 ms, Ok at 572 ms after boot, the notice "SD: 59.5 GiB FAT32, 17 MiB in 2
logs" on USB; `status` shows LOGGING healthy; `devices` 8 rows, the six and SD OK, RC Starting (no radio
link on the desk); liveness 1.0 samples/s; after 41 s the driver counted 85 blocks read, 0 written (2 per
liveness plus the mount). `device restart SD`: accepted, un-mount 0.3 ms, a new bus, Ok again 66 ms later with a
new notice; `device selftest SD`: DENIED "not supported yet". `inject-sd-no-card` (231243): four start tries,
Absent at 2.03 s, last error timeout, "SD: no card". `inject-sd-wrong-format` (231330): Failed at 2.05 s, last
error id mismatch, "SD: no FAT32 volume, reformat as FAT32". `sd-irq-negative` (231415): INTE0 0x1f, the INTE0
check FAILs, the other four PASS. `LOG_BACKEND_TYPE` 0, saved, `reboot --yes` (231521, 231607): SD Absent NONE,
"SD: logging off", no card init line in the boot's log; 1, saved, reboot: Ok with the notice. NOT seen then (a real card
removal is below, in Logging): a real exFAT card, a second card type, errors in service (the ladder on a real
card fault); the 10-minute run against `main` (milestone check G) was started but not read when this was
written.

## Logging

`src/logger.rs` and `src/sd.rs` (decision-sd-logging Q3 to Q6, milestone SD3; the pure pieces are
`fw_core::dflog`, `fw_core::logfile` and `fw_core::xcore::LogRing`).

- Format: ArduPilot DataFlash `.BIN` (FMT records first, then `A3 95 <type> <payload>` records, little-endian,
  `TimeUS` board microseconds first). Readers, free and without conversion: `mavlogdump.py` and MAVExplorer
  (`pip install pymavlink MAVProxy`), UAV Log Viewer (plot.ardupilot.org), Mission Planner. Checked on a PC
  (2026-11-16): ArduPilot's reader, pymavlink 2.4.50 `mavlogdump.py`, read three logs copied off the card with a
  USB card reader: the 30-minute log 3 (397 226 records, no bad bytes, the same counts as the on-board walker,
  45 282 IMU records per instance = 25.0 per second each, wall-clock time from the GPS records correct), the
  8.5-hour log 5 (161 MB, no bad bytes, DSF 30 804, GPS 30 885) and log 13 (the one cut by the card pull below:
  reads clean). UAV Log Viewer and Mission Planner were not tried.
- What and how often (Q5 and decision-flight-logging-20261119 section 1; about 166 kB/s armed, 34 kB/s
  disarmed, of which core 0's own records are about 4.6 kB/s with fresh stick reports): from the flight core
  (IMU1's `flight::Stepper`, after the motor command is published) IMU instance 0 (IMU1, this step's sample) with
  NAV, SETP, DMD and MOT at every flight step while armed (803.6 Hz, 182 B per step) and every 8th step while
  disarmed (100.4 Hz), ESC (eRPM, volts, degC by DShot line, the fresh bits; `TimeUS` the exchange's) every 4th
  step armed (200.9 Hz) and every 8th disarmed, RCS (sticks and arm switch, the report's own time) for each stick
  report the step takes in (up to about 250 Hz); from the collector IMU instance 1 (IMU2) at 25 Hz (from its
  seqlock every 4th tick of the collector's 10 ms ticker, only a new sample, its own time), BARO (about 50 Hz) and MAG (about 100 Hz) at every new sample, GPS at each receiver sample (the fix rate:
  status, GPS week and ms from UTC, 0/0 before a time is known) and with each GPS record one GPA (below), BAT at each sample (10 Hz), RCIN at 10 Hz while
  stick reports are fresh, DEV for the 8 rows at 1 Hz and at every state change, MSG for every STATUSTEXT (the
  text ring's third cursor: the boot texts first), PARM for all 92 parameters when a log opens and after
  every change of a value, whatever made it (a set, a calibration result, a reset, a reload: the collector
  compares the store with what it logged, every tick), DSF once per second (Dp records dropped in this log, Blk and Bytes of this log, FMn/FMx/FAv the
  ring's free bytes over the second), and with it FDS (Dp flight-ring records dropped in this log, FMn the flight
  ring's smallest free space since the last FDS). Flight records reach the file up to about 10 ms after core-0
  records of the same time, so `TimeUS` is not in order across types (nor within IMU, whose two instances come from
  the two cores).
- GPA (id 146, format `QBcffH`, 24 B, ArduPilot's name; no coordinate), one with every GPS record: `TimeUS` the GNSS sample's `t_us`, the
  same as its GPS record (they join on `TimeUS`); `I` 0; `VDop` VDOP x 100, encoded as the GPS record's HDop (`u16::MAX`, which reads as
  32 767, when unknown); `HAcc` m, sqrt(lat_std^2 + lon_std^2) of the epoch's GST; `VAcc` m, its altitude deviation; HAcc and VAcc NaN when
  not available; `Delta` ms since the previous GNSS sample's `t_us` in this log, 0 for the first, saturating at 65 535.
- Path: `log_collect` (core 0, watchdog id LOG) is the one producer of a 64 KiB `LogRing`; it pushes only while
  a log is open (`logger::OPEN`) and never awaits the card. A record that does not fit is dropped and counted.
  The flight core's records go first into a 16 KiB `LogRing` (`flight::LOG`: producer IMU1's `Stepper` on core 1,
  consumer the collector on core 0); every 10 ms tick, log open or not, the collector moves its whole records into
  the main ring (`fw_core::relay::relay`; with no log open they are consumed and discarded). Core 1 never waits: a
  record that does not fit the flight ring is dropped and counted (FDS Dp); one that does not fit the main ring is
  dropped there and counted (DSF Dp) and still consumed, so a stalled card never backs up into core 1.
  The SD task is the one consumer: whole 4 KiB pieces are written in place from the ring (the file offset and
  the ring's read index are kept equal modulo 4096 by a filler the collector commits and the SD task discards
  when a log opens), after 1 s without a write the bytes that wait, then a piece up to the next 4 KiB boundary;
  `flush` once per second. The SD task computes the DSF figures (it owns the ring's statistics) and the
  collector writes the record.
- Files: `LOGS/NNNNNNNN.BIN`, 1 to 500 then 1 again; `LOGS/LASTLOG.TXT` holds the last number and is written
  before the log is opened; a missing or unreadable `LASTLOG.TXT` falls back to the largest number in `LOGS/`.
  Other files in `LOGS/` are left alone. FAT times stay 1980-01-01.
- When: from boot, one log per boot and a new number after every recovery or RESTART (`device restart SD`
  writes what waits in the ring, flushes, closes, un-mounts, mounts and opens the next number). Off with
  `LOG_BACKEND_TYPE` = 0 (nothing collected, the card untouched).
- Stalls and failures: the main ring absorbs about 0.4 s of a stalled card while armed (about 1.9 s disarmed);
  beyond that records are dropped (counted in `overflows` and DSF Dp). A failed write is retried once at once; a second failure is one error event and the piece
  is discarded (the file then has a gap); three such events make the row Recovering, and the recovery opens the
  next log. FAT is not power-safe: a power cut or reset loses what was not yet flushed (about 1 to 2 s).
- Clean-up, BEFORE the log is opened and never while a log is written (as ArduPilot's `Prep_MinSpace`, which
  runs at initialisation): after the mount, the listing and the choice of the new number, and before
  `LASTLOG.TXT` is written, `plan_cleanup` runs over the listing in a loop: while the logs together exceed the
  volume minus 500 MiB the oldest (ring order) is deleted: first one NOTICE "SD: clean-up, deleting log N", then
  the deletion, then the listing is updated and the planner asked again. "Card full" (under 8 MiB left even
  without the other logs) opens no log and parks the row Failed with "SD: card full". A deletion is one awaited
  file-system call that costs about 10 ms per 32 KiB cluster (a 161 MB log: about 50 s), so after a long deletion
  logging starts late by its duration (the ring is not yet in use, nothing is lost, and the log's first records
  are the ones after it opens); the row stays Starting (or Recovering, Restarting) throughout, and while a deletion
  runs the task marks progress every second (the deletion's future is polled by reference, never dropped), so the
  supervisor does not flag the row stuck. A log being written is never interrupted: the next clean-up is at the
  next open (boot, RESTART or recovery), so a log that grows past the reserve is not trimmed until then. A
  RESTART request that arrives during the clean-up waits for the log to open. The file system is never asked for
  free space.
- Logs over USB (SD4, the contract: decision-sd-logging "SD4" and MARV-GCS `docs/link-contract.md` section 11):
  ArduPilot's MAVLink log download on the USB link only: LOG_REQUEST_LIST 117 -> LOG_ENTRY 118, LOG_REQUEST_DATA
  119 -> LOG_DATA 120, LOG_REQUEST_END 122. Not offered: LOG_ERASE 121, the radio (its node has no accept list,
  so a request there is dropped and counted as an unknown message), MAVFTP. `id` is the log's number, `time_utc`
  0, `size` bytes (the open log: its size at the last flush, so it is listed and can be downloaded while it
  grows). A request for a log that does not exist gets nothing; `ofs` at or past the end one LOG_DATA with count
  0. A new LOG_REQUEST_DATA or LOG_REQUEST_LIST replaces the running transfer; LOG_REQUEST_END, the station's
  HEARTBEAT older than 3 s or the link going down stop it. Requests that arrive before the start gate opens
  are dropped (the station retries). While the SD row is not Ok (no card, starting, recovering, the clean-up
  before a log opens) a request gets no answer at all, not even the empty entry; a request that arrives while
  the row is only Degraded is held and answered when the row is Ok again.
  How (`src/logdl.rs`): the USB link runs its own copy of the node's driver (`drive` in `main.rs`, mirroring
  mavlink-device `run.rs` at dfdb11a) so that `receive_with` hands the three requests over; each request gets
  a new generation number and wakes the SD task. The SD task answers from its listing and reads the log
  through a second, read-only file handle, only when no log piece and no flush is due, one step at a time (a
  request taken, the handle opened, moved or closed, or at most 900 bytes read), and hands whole records
  tagged with the generation through a 4 KiB `LogRing`; the USB node's LOG_DATA stream slot (5 ms) sends at
  most 4 per 5 ms and drops records of an older generation, so a replaced transfer's bytes are never sent. A
  handle with nothing left to read is closed after 2 s. `[STAT]` of the USB link: transfers, bytes sent, the
  SD task's longest download step, and the longest time a piece or flush waited for one (an upper bound: the
  step's whole duration).
  On the board (2026-11-16, normal build, logging; `logs/marv-v2-firmware-20261116-113221.log`, `-114213.log`):
  the list equals the SD task's listing (19 logs, numbers 7 to 25 with sizes; the open log listed with its
  flushed size, growing); logs 8 and 13 downloaded twice with a throw-away pymavlink client (the station's
  client did not exist yet) and once with `marv-cli logs get`: all byte-identical (sha256) to the card-reader
  copies; 57 kB/s with the pymavlink script, about 70 kB/s with `marv-cli` (2 527 232 B in 36 s). Seen: a log
  that does not exist gets no LOG_DATA; `ofs` at and past the end one LOG_DATA with count 0; a new request in
  the middle of a transfer takes over (4 frames of the old log already on their way, none after the first frame
  of the new one); LOG_REQUEST_END stops the stream at once; with the station's HEARTBEAT stopped the last
  LOG_DATA came 3.01 s after the last HEARTBEAT. During a 109 s download of log 24 (6 258 688 B): SD Ok, 0
  errors, overflows 0, 9.6 to 10.4 blocks/s, every other row Ok with 0 errors, IMU 803.49 to 803.63 / 800.66
  to 800.76, watchdog stalls begun 0; one IMU2 gap over 1.5 P (largest gap 1 886 us against 1 874 us) in the
  window 260 to 270 s, IMU1 0. The SD task's longest download step 8.1 ms, the longest wait of a piece or
  flush for one 3.5 ms (upper bound). The open log, read through the second handle while being written, came
  down to its flushed size (1 449 984 B) and is the exact prefix of the same log downloaded after it was
  closed; `log-verify` read that log (2 527 232 B) with 0 bad bytes. NOT seen: a request while the row is not
  Ok (no card, recovery, clean-up), a link drop in the middle of a transfer, a listing of hundreds of logs.
- Test hooks (Cargo.toml): `inject-sd-write-fail`, `log-verify`, `inject-log-small-volume` (the first open's
  clean-up after boot is told a volume of the reserve plus the listed bytes minus one: the oldest log is deleted
  before the log opens), `inject-log-delete-big` (the first open's clean-up after boot is told a volume of the
  reserve plus the bytes of the logs after the largest one in ring order: every log up to and including the
  largest is deleted, one by one, so a long deletion is seen on the board).

On the board (2026-11-16, MARV V2 Rev 1 on USB, probe-rs attached, the scratch SanDisk 64 GB card, receiver
searching, so no RCIN):
- 30 minutes from boot, normal build, three `param save` at 307, 914 and 1367 s
  (`marv-v2-firmware-20261116-011541.log`): 13 `[PASS]`, 0 `[FAIL]`; every row Ok throughout with 0 errors, SD
  Ok with 0 errors, 0 recoveries, 0 records dropped, 10.2 blocks/s (18 440 at 1810 s), largest gap between
  writes 880 ms; IMU rates 803.5 to 803.6 / 800.6 to 800.7, and 800.0 / 797.2 in the three windows of a save;
  gaps over 1.5 P: one per IMU at each save (3 in 30 minutes, the saves' flash stalls of 40.9, 40.7 and 44.6 ms,
  the largest gap 45.0 / 45.3 ms), 0 in every other window (largest gap before the first save 1.66 / 1.54 ms);
  watchdog stalls begun 0.
- That log read back on the board (`log-verify`, `-014612.log`): 9 453 568 B, 397 226 records, 0 bad bytes, a
  17-byte cut last record, FMT matched 11, mismatched 0; IMU 50.0/s (both together), BARO 49.8, MAG 99.5, GPS
  1.0, BAT 10.0, DEV 8.0, DSF 0.997/s; PARM 26, MSG 2 (the boot text and the SD notice); 5.2 kB/s; the last
  record within about 1 s of the moment the board was stopped.
- An unplanned soak: log 5 ran 8 h 35 min (`-014815.log` to `-102320.log`): every row Ok with 0 errors at the
  end; read back: 161 218 560 B, 6 775 764 records, 0 bad bytes, tail 37 B, FMT matched 11, mismatched 0, the
  same rates; the SD row was not flagged stuck during the 68 s walk.
- `device restart SD` (`-102721.log`): log 7 written out and closed (212 503 B), log 8 opened 80 ms later with a
  new notice. In log 8 `param set BARO_ODR_HZ 25` and `50` and `calibrate gyro`: 34 PARM records read back
  (26 + 2 + the 6 gyro offsets).
- `inject-sd-write-fail` (`-103425.log`): at 60.7 s the next six write calls fail: Ok -> Degraded at 61.3 s,
  Recovering at 62.8 s, log 10 closed (315 392 B), Ok again at 63.0 s with log 11; the other rows unaffected.
- `inject-log-small-volume` (`-103223.log`, before the clean-up moved to before the log opens): 1 s after log 9 opened, `LOGS/00000001.BIN` (319 488 B) deleted in
  58 ms, once; the row stayed Ok.
- A real card removal (`logs/marv-v2-firmware-20261116-104041.log`, pulled at 225 s): the next flush failed, Ok
  -> Degraded, two pieces discarded after their retry, Recovering 0.9 s after the pull, the log closed and the
  volume un-mounted on the missing card in under 2 ms (ok false, no hang), four recovery attempts "no card",
  Failed 5.5 s after the pull with the text "SD: failed (bus)"; with the card back, `device restart SD` gave Ok
  in 329 ms and log 14.
- Clean-up before the log opens (2026-11-16, `-105045.log`, `-105233.log`): `inject-log-small-volume`: "SD: clean-up,
  deleting log 2" at 0.56 s, `LOGS/00000002.BIN` (1 294 336 B) deleted in 210 ms, then the log opened at 0.80 s,
  SD Ok. `inject-log-delete-big`: logs 3 (9 453 568 B, 1.35 s), 4 (512 000 B, 77 ms) and 5 (161 218 560 B,
  22.9 s) deleted one after the other, the log opened 24.9 s after boot; during the 22.9 s (the stuck limit is 20 s)
  the SD row was Starting, never flagged stuck, then Ok; the other rows Ok with 0 errors, IMU 803.5 / 800.7,
  watchdog stalls begun 0; the log then written at 10.4 blocks/s, overflows 0, in the 210 s run.
- NOT seen: a power cut, a real write error on a present card, card full, a second card type, RCIN records (no
  transmitter), the logger without the debugger attached.

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
  version: `load` runs no validator) puts that device's `Configure`- and `Reboot`-class parameters (not its `Live`
  calibration offsets) back to their defaults in RAM,
  the flash untouched, with ONE WARNING text "BARO: stored settings invalid, defaults"; the device then
  starts normally on its defaults (`param list` shows them; a save writes them). This runs for BARO, MAG, IMU1 and IMU2
  (the IMUs through `imu1_setup` / `imu2_setup`, whose refusal text is in the log line). One log
  line `[INFO] settings boot: <class>, sector, sequence, loaded, skipped; sector A <state>, sector B <state>;
  next save to sector <X> with sequence <N>`. The texts are queued before anything else and wait in the
  text ring until each link's station connects; the classification text ("none stored" / "stored record
  invalid") is sticky (kept until every link took it), the skip and "stored settings invalid" lines are
  ordinary texts (a link more than 8 texts behind loses the oldest).
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
- `FEATURES=fake-armed tools/run.sh 60`: the handshake's armed flag is set at boot; `param save` is
  TEMPORARILY_REJECTED (the admission reads the same flag as armed) and writes nothing. Before the flight
  wiring (step 5) it reached the save and ended FAILED "save failed: armed".
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

## Frames

Every IMU and compass value that leaves the board is in the body frame, FRD (x forward, y right, z down), as in MAVLink and ArduPilot: level and at rest the accelerometer reads z about -9.81 m/s^2 (specific force, up). The IMU adapters (core 1) apply the chip's fixed rotation (`ROLL_180`), then subtract the offsets (the board frame), then apply `AHRS_ORIENTATION`; the compass adapter (core 0) applies `MAG_ORIENT`, then the calibration m = S (r - o) with `MAG_OFF*` and the `MAG_DIAG*`/`MAG_ODIAG*` matrix (the body frame). The log's IMU and MAG records and the `[STAT]` lines carry the same values. Measurements made before this build (this README's, `docs/handoff.md`'s) and offsets and logs stored before it are in the chips' own axes, and the stored offsets must be redone. Design: `_plan/decision-orientation-20261117.md`.

On 2026-11-18 the owner checked the IMU and compass signs on both IMUs (moved by hand), calibrated the gyroscopes, accelerometers and compass in the body frame, and saved the offsets to flash; they were kept after a reboot.

## Calibrations

`marv-cli calibrate gyro`, `calibrate accel`, `calibrate compass` (MAV_CMD_PREFLIGHT_CALIBRATION 241) and
`calibrate cancel` (the all-zero 241). The decision is pure and tested on the host (`fw_core::calib`); the
firmware glue is `src/calibration.rs` on core 0, stepped from the supervisor tick (`actions::tick`). One
calibration at a time (the one `ActionBook`); progress 0 to 100 % goes out through the book, HEARTBEAT
`system_status` is CALIBRATING while one runs. The result is written into the parameters and used at once;
it is NOT saved to flash: only the user's Save (`param save`) stores it, and a reset or `param reload`
loses it. Nothing is written by a failed or cancelled run. Every calibration queues its result or failure
text BEFORE its action ends, and the final ACK is held until that link's texts are taken (`actions::next_ack`, per link,
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
`raw - offset`. Compass: the core-0 adapter (`src/mag.rs`) reads the compass calibration (below) from the
store for every sample and publishes m = S (r - o) (`fw_settings::mag::apply`); the sample also carries the
raw field r. So the samples on the link and in the rows are corrected, and a calibration measures the RAW
value: the IMU calibrations add the offsets in force back to each sample they see (`ImuOffsets::restore`), the
compass calibration takes r from the adapter; a second calibration with the first's result in force gives the
same result. Core 0 sees the rows' latest sample (the supervisor mirrors the
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

### Accelerometer (`calibrate accel`, both IMUs, one position, level)

Method: ArduPilot `AP_InertialSensor::simple_accel_cal`: averages of 50 samples; two successive averages
closer than `accel_convergence_limit` = 0.05 m/s^2 (length of the difference) converge, the offset is
their mean minus gravity, the specific force a level vehicle reads: (0, 0, -9.80665) in the body frame (FRD), and its rotation by `AHRS_ORIENTATION`'s transpose in the board frame the offsets are stored in; not converged within 20 averages ("up to 10 seconds"): failed.
The largest offset accepted per axis is one gravity (`AccelCalibrator::accept_result`, `GRAVITY_MSS`;
`simple_accel_cal` itself has no limit). Duration: 1 s at the least, 10 s at the most. Texts: "accel
calibration: moving" (not converged), "accel calibration: IMUn offset too large", "accel calibration: no IMU
Ok", "accel calibration: IMUn not Ok, skipped"; the result "accel cal: |IMU1| N |IMU2| N mm/s^2".

### Compass: model, parameters and state

Model (decision-compass-cal-20261120): m = S (r - o), microtesla. r is the raw field in the body frame,
after `MAG_ORIENT`; o is the hard-iron offsets `MAG_OFFX/Y/Z`; S is a symmetric 3x3 soft-iron matrix,
[[`MAG_DIAGX`, `MAG_ODIAGXY`, `MAG_ODIAGXZ`], [`MAG_ODIAGXY`, `MAG_DIAGY`, `MAG_ODIAGYZ`], [`MAG_ODIAGXZ`,
`MAG_ODIAGYZ`, `MAG_DIAGZ`]]. Single precision in this order: d = r - o, then m_i = (S_i0 d_0 + S_i1 d_1) +
S_i2 d_2, no fused multiply-add (`fw_settings::mag::apply`; the worked vector of the decision is a host test,
bit for bit). The offsets are subtracted (PX4's sign); an ArduPilot offset has the opposite sign. Every
compass value that leaves the board (the log's MAG record, HIGHRES_IMU's `x/y/zmag`, the `[STAT] MAG:` line)
is m. With S the identity (the defaults) m equals the old `r - o` (bit for bit, except that a difference of
-0.0 may come out as +0.0). The parameters (indices 81 to 91, see Parameters): S's six terms, `MAG_CAL_TYPE`
(0 none or unknown, 1 quick, 2 guided), the expected field `MAG_FLD_UT` and `MAG_FLD_INC` (from the World
Magnetic Model at the calibration site, 0 = not set), and the flight health test's tolerances `MAG_HLT_STR`
and `MAG_HLT_INC` (stored only: nothing on the board reads them yet). `param reset-calibration` returns
`MAG_OFF*` and indices 81 to 89 to their defaults: S the identity, offsets 0, type 0, no expected field. A
record saved before these parameters existed (offsets only) loads with S the identity and type 0. Changing
`MAG_ORIENT` invalidates the calibration (S and o are in the body frame after it); nothing on the board warns.

States, as a station shows them (the board stores the parameters and does not compute a state): **not
calibrated**: `MAG_CAL_TYPE` 0 and the offsets all 0; **best effort**: type 1, or type 0 with offsets (an
older calibration); **guided**: type 2 with `MAG_FLD_UT` > 0.

The guided calibration is done by the station and written as parameters: MARV-GCS collects HIGHRES_IMU and
GPS on USB, fits S and o and writes `MAG_CAL_TYPE` 0, then `MAG_OFF*`, `MAG_DIAG*`, `MAG_ODIAG*`,
`MAG_FLD_UT` and `MAG_FLD_INC`, then `MAG_CAL_TYPE` 2, and reads them back. The board has no fit, coverage or
pose logic. As with every calibration, nothing is saved until the user's Save.

### Compass quick calibration (`calibrate compass`, MAG, best effort)

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
at once); the result "compass cal: off X Y Z r X Y Z uT" (offsets and radii, integers, INFO). It measures
the raw field r the adapter hands beside m (nothing is undone), so its result does not depend on the
calibration in force. On success it writes, in one store access within one supervisor tick, one coherent set
(`fw_settings::mag::write_quick_result`): `MAG_OFFX/Y/Z` = the midpoints, S = the identity (`MAG_DIAG*` 1,
`MAG_ODIAG*` 0), `MAG_CAL_TYPE` = 1, `MAG_FLD_UT` = 0 (its offsets mean something only with S the identity,
and it has no field model); `MAG_FLD_INC` and `MAG_HLT_*` are left as they are. The adapter uses the set from
the next sample. Not built: automatic or in-flight compass learning.

### Test feature

`inject-mag-rotation` (off by default; `FEATURES=inject-mag-rotation tools/run.sh`): while a compass
calibration runs, its input (not the published samples) is replaced by a synthetic rotation, the six points
(+-50, 0, 0), (0, +-50, 0), (0, 0, +-50) uT about the centre (10, -20, 30) uT, each held for 100 samples, so
the success path can be shown on a board that cannot be turned: offsets (10, -20, 30), radii 50.

### On the board (2026-11-15, MARV V2 Rev 1 at rest on the bench, nothing saved)

(Later: on 2026-11-18 the owner calibrated this board by hand (gyro, accelerometer, compass) and saved the offsets to flash; they were kept after a reboot. The figures below are the 2026-11-15 ones.)

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
(`src/actions.rs`; link id 0 = USB; `armed` is the flight core's handshake, flight throttles sent or a motor-test
spin, and `hil` is false until the HIL source exists; `ActionBook::tick` runs in the supervisor). The answer to an admitted command is IN_PROGRESS 0 %;
the final ACK follows through `acks` (about 100 ms later, the node asks at least every 100 ms). Denied
commands put the book's text out as a STATUSTEXT. Every decision logs
`[INFO] command <number> from <sys>/<comp>: <result>`, every action `action <name> starts` and
`action <name> ended: <result>`, then `final ACK of command <number>: <result>`.

| Command (`marv-cli`) | What it does today |
|---|---|
| `device restart IMU1`, `IMU2`, `BARO`, `MAG`, `GNSS`, `BATT`, `SD` (every row of the registry) | raises that device's RESTART request; the loop feeds `Event::Restart` (from any state) and starts the device again; ACCEPTED when the device reaches Ok, FAILED when it reaches Failed or Absent first (the book's own 5 s limit covers a hang); the action records the number of the restart request it raised and ends only on that request's outcome, so an older restart finishing late never ends a later action; progress stays 0 |
| `param reset-all --yes` | `Params::reset_all`; each device whose values changed gets its APPLY request; ACCEPTED as soon as the store is reset and the requests are raised, without waiting for the applies (as a parameter write does not wait; a device that is Failed or Recovering picks the values up when it is restarted) |
| `param reset-calibration --yes` | `Params::reset_calibration`, the same way: the twelve `IMUx_*_OFF*` and the three `MAG_OFF*` go to 0, the compass's `MAG_DIAG*` (1), `MAG_ODIAG*`, `MAG_CAL_TYPE`, `MAG_FLD_UT` and `MAG_FLD_INC` (0) to their defaults, and the IMU offsets are pushed to the flight core; ACCEPTED |
| `param save` | writes the parameters that differ from their defaults to the flash settings region (see Settings in flash); ACCEPTED when the record was read back and verified, FAILED with one text otherwise ("armed", "flight core not answering", "verify failed") |
| `param reload` | the boot rule again from flash; the devices whose values changed get their APPLY request; ACCEPTED |
| `reboot --yes` | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1 = 1: the action writes the REQUESTED scratch record and ends ACCEPTED; when the final ACK is handed to the link the watchdog monitor resets the board (`trigger_reset`) 100 to 200 ms later (1 s after the action when no ACK could be handed). Nothing is saved: an unsaved parameter change is gone after the reboot. The boot report then says INFO "reboot: requested" |
| `calibrate gyro`, `accel`, `compass`, `calibrate cancel` | see Calibrations |
| `device selftest MAG`, `IMU1`, `IMU2` (others DENIED "not supported yet") | THE DEVICE MUST LIE STILL (no motion check: a moved device just reports FAIL). The device's task runs the driver's self-test with the driver's defaults (bmm350 X and Y both polarities; icm45686 the vendor's eDMP test, both sensors, 320 ms, 50 %; lsm6dsk320x low-g, gyroscope and high-g, positive stimulus), then the operator restart path under the same request number (the adapter's `restart`, then a fresh sample) whatever the test gave. MAG on core 0 (`REQ_SELF_TEST` in `run_device`), IMU1/IMU2 on the flight core (Kick bit `REQ_SELF_TEST` = 8, the test and the restart in the same loop step so no CHECK is taken between them). The outcome crosses as `fw_core::selftest::Outcome` (number, verdict Pass / Fail(failed-axis mask: IMU1 accel XYZ gyro XYZ, IMU2 low-g/gyro/high-g XYZ, MAG X+ X- Y+ Y-) / Error(kind), duration; IMU outcome in `ImuStatus`), the device's return through the restart outcome of the same number. ACCEPTED only on a pass with the device back Ok, otherwise FAILED; ONE STATUSTEXT before the final ACK: "MAG self-test: PASS", "IMU2 self-test FAIL: lowg X gyro YZ", "IMU1 self-test: error bus". The row's flag bit 1 (self-test) is set from the request until the action ends, and the supervisor's stuck check does not apply to the row meanwhile. The IMU tests wait through the flight core's `SpinDelay`: every wait of 50 us or longer polls the clock and yields (no timer queue), shorter ones busy-wait (so a verify step, whose waits are 10 us, never yields). On the board (2026-11-15, logs `marv-v2-firmware-20261115-152811.log`, `-153906.log`, `-155325.log`): MAG 8 runs PASS in 133-164 ms, IMU1 7 runs PASS in 749-751 ms, IMU2 7 runs PASS in 875-884 ms, each device back Ok with samples at its rate, the calibration offsets still applied afterwards; 1 to 2 status rows went out with the flag per IMU run, 0 or 1 per MAG run (the flag is not visible in `marv-cli devices`, which prints no flags). With the 50 us rule (`-155325.log`), during the three IMU1 tests: no "flight core not answering", watchdog stalls begun 0, IMU2 0 errors, 0 timeouts, 0 recoveries, IMU2's largest gap 3.4 ms (up to 10 s, covering the first IMU1 test and most of the second; a busy-waiting 100 us poll had starved IMU2 for 0.96 s and the watchdog had reported a 0.5-0.7 s stall per test). Not seen on the board: a FAIL or an error verdict (host tests only, `fw_core::selftest`, `fw_flight::imu`) |
| `device restart RC` (a device this build does not serve), and any command or parameter the admission does not accept | DENIED by the admission, its text as a STATUSTEXT |

## Radio (ELRS receiver, `rc-port`)

`src/rc.rs`, task `rc_link` (watchdog id RC = 13) alone on core 0's `SWI_IRQ_0` interrupt executor at
priority P3 (0xC0, embassy-rp's lowest level: below the hardware interrupts at P0 and the watchdog monitor's
`SWI_IRQ_1` at P1, above thread mode): UART0 as `BufferedUart`
(interrupt driven, `UART0_IRQ`, no DMA), 256-byte rings, TX GPIO14, RX GPIO17 with the pad's pull-up turned
on after the UART is built (embassy-rp 0.10 leaves it without a pull; the start line logs pull-up, pull-down
and the line level). `rc-port` (`fe55602`) follows the receiver between CRSF (420000) and MAVLink (460800)
by itself; it carries the radio's MAVLink node (system 1, component 1, link 1). Everything the task
shares with thread mode (the parameter store, the action book, the text ring, the rows, the settings
storage) is a critical-section mutex or an atomic, none held across an await. Its watchdog id is stored
at the start of each of its polls and the preempted task's id put back at the end (`rc::preempting`), so
a stall inside its poll is recorded as RC and the interrupted thread-mode task keeps its own id.
UART overruns (in `[STAT] RC:`): 5 in 300 s in thread mode, 7 in 300 s on `SWI_IRQ_0` (2026-11-15, radio
on, MAVLink): the executor did not remove them (cause not found yet).

The RC row (`MARV_DEVICE_STATUS` device 7, SYS_STATUS RC_RECEIVER) is REPORT ONLY: no health machine,
no recovery (`rc-port` recovers by itself), `device restart RC` is DENIED. The row means "the radio link
is up" (`fw_core::rc::verdict`, host-tested table): CRSF lock with a stick report younger than 1 s = OK;
MAVLink lock with a RADIO_STATUS younger than 1 s whose `rssi` is above 0 = OK (the receiver goes on
sending RADIO_STATUS with `rssi` 0 while the transmitter is off); either lock otherwise = DEGRADED,
last error TIMEOUT; searching = STARTING until the first lock since boot, DEGRADED after. `present` =
locked at least once since boot; `enabled` = the state mapping of `fw_core::sys_status`; `health` = the
radio link is up. `samples` = stick reports + RADIO_STATUS reports, `last_sample_age_ms` from the
newer of the two, `errors` = CRC + UART errors, `timeouts` = lock losses. The row is in both links'
device rotation; it is not in the supervised registry (`device::DEVICES`). The ESC row (see ESC telemetry) is
built the same way and is in both rotations too.

What each link carries:

| | USB (link 0) | Radio (link 1, only while the port is locked on MAVLink) |
|---|---|---|
| HEARTBEAT, SYS_STATUS | 1 Hz each | 1 Hz each |
| MARV_DEVICE_STATUS | 9 rows (the seven registry rows, RC, ESC), 4 per message batch every 333 ms | one row per second: a row whose state changed since it was last sent on the radio goes out of turn, at most every other message (the message after it is the in-turn one), else in turn |
| GPS_RAW_INT | 1 Hz | every 2 s |
| HIGHRES_IMU (`id` 0: IMU1 with the magnetometer), SCALED_IMU2, SCALED_PRESSURE | 25 Hz each | 1 Hz each, built by the same functions as on USB (`push_highres_imu`, `push_scaled_imu2`, `push_scaled_pressure` in `main.rs`), so the fields are the USB link's; the Sensors tab's messages |
| RC_CHANNELS (65) | 5 Hz while stick reports are younger than 1 s: 16 channels in us, `chancount` 16, `rssi` = CRSF uplink LQ x 254 / 100, or RADIO_STATUS `rssi` in MAVLink mode, 255 without a fresh figure | none |
| MANUAL_CONTROL (69) | 5 Hz (fastest 50 Hz, like RC_CHANNELS) while the latest stick report is from CRSF and younger than 1 s (`fw_core::rc::crsf_sticks`; a MAVLink RC_CHANNELS_OVERRIDE report never feeds it): `target` 1, `x` pitch, `y` roll, `z` throttle (0 to 1000), `r` yaw, each from the RC parameters (`fw_settings::rc::RcMap::axes`), `buttons` and the extension fields 0. Computed in the USB link task when the stream is due (`rc::manual_control`: one `SHARED` lock for the sticks, then one store access that copies the 44 values by index; not in the RC hook, not on core 1). Reported only; no value is held or invented when the sticks are stale | none |
| STATUSTEXT | the ring, link 0 | the ring, link 1 |
| Parameters, commands | on request; ACKs on the origin link | the same; no automatic push |

The radio's budget from the frame sizes (MAVLink 2 as our dialect crate encodes them, 12 bytes of framing; trailing zero bytes of a payload are not sent; sizes got by encoding one message of each type with `marv-dialect` at the pinned commit `af6baa1` in a scratch program, outside this repository): HEARTBEAT 21 B/s, SYS_STATUS up
to 43 B/s, one MARV_DEVICE_STATUS up to 42 B/s, GPS_RAW_INT up to 64 B per 2 s = 32 B/s, HIGHRES_IMU 74 B/s (the
`id` byte, 0, is not sent), SCALED_IMU2 36 B/s, SCALED_PRESSURE 26 B/s (28 with a non-zero `temperature_press_diff`, which
is always 0): at most about 274 B/s plus texts (up to 66 B each). Counted at the UART (`sent MAVLink` in `[STAT] RC:`, MAVLink 460800
mode, GPS fix present, 2026-11-16, transmitter on in its MAVLink model): 2 700 B in every 10 s from 10 s to 90 s, 270 B/s,
against about 134 B/s before the three streams (2026-11-15). The owner's ELRS settings are 250 Hz packet rate, telemetry
ratio 1:2, switch mode hybrid; ExpressLRS's documentation gives about 590 B/s of telemetry at 250 Hz (UNVERIFIED: read from a
summarised web page, not checked against the source or on the air), of which 270 to 274 B/s is 46 to 47 %. The air link's own
framing of MAVLink and its share left for the receiver's link statistics are not known here, so this is the rule's margin, not
a measured air figure. The streams' fastest interval is their default, so a station cannot raise it (it can slow or stop one
with SET_MESSAGE_INTERVAL); the node's own slowdown from RADIO_STATUS `txbuf` stays on. RC_CHANNELS and MANUAL_CONTROL are not
on the radio: in MAVLink mode there are no CRSF sticks. In CRSF mode the port calls the telemetry hook at most every 100 ms. On each call, if the GNSS row's
latest sample is newer than the one last sent as a GPS frame, the hook sends a GPS frame (CRSF type 0x02: latitude, longitude, groundspeed,
heading, altitude, satellites) built from it, in place of that call's battery frame (Betaflight's rule, "on each new solution"); otherwise it
sends the battery frame from the BATT row's latest sample while it is younger than 1 s (remaining percent and capacity 0: not
estimated). A GPS frame goes at most once per second (`GPS_FRAME_MIN_US`, the 1 Hz receiver's frame mix), whatever `GPS_RATE_MS`: 1 GPS frame and about 9 battery frames per second. Values (`fw_core::rc::crsf_gps`, host-tested): latitude and
longitude in 1e-7 degree; groundspeed in 0.1 km/h, `(mm/s * 36 + 500) / 1000` (Betaflight's `(cm/s * 36 + 50) / 100` from a finer unit),
at most 65 535; heading is the course over ground in 0.01 degree; altitude is the height above mean sea level in whole metres (floor),
clamped to -1000 to 4000 m (wire value 0 to 5000), Betaflight's rule (`crsfFrameGps`: `constrain(altCm / 100 + 1000, 0, 5000)`); satellites
are the satellites used. Before a fix, and after a loss, the frame carries zeros (as Betaflight and ArduPilot send their zeroed GPS state
before a first fix); unlike them it does not hold the last position after a loss. Every value the sample lacks is sent as 0 (altitude 0 m,
satellites 0). The frame comes from `crsf-codec` (`2d12a55`, through `rc-port` `fe55602`). The hook only copies: it reads the GNSS and BATT
rows' latest samples, encodes into the port's buffer and remembers the sent sample's time.

A parameter change on one link is answered (echoed) on that link only; both nodes use the one store
(`store::Store`), whose critical-section mutex serialises writes from the two links. Texts: the ring
`fw_core::text_ring` (8 entries, 32 with `latency-text`), a cursor per link, each text once per link;
the boot report and the settings classification in the 2 sticky slots (taken first, kept until every
link took them; the worst case of all boot texts is about 14, so the skip and per-device lines are
ordinary texts); a link that is down holds back nobody; the 300 ms "text before ACK" hold is per link.

The flight core gets the sticks: the port's `report` hook maps every CRSF stick report with its own copy of the RC
parameters (read from the store at the first report and then every 64th, about 4 times a second; no sample before the
store gave one) and writes `RcSample { sticks, arm, t_us }` to the triple buffer `rc::FLIGHT_RC`; `arm` is the raw
channel 5 above 1800 us (`fw_settings::rc::arm_switch`, ArduPilot's switch-high threshold, no calibration or reversal).
A MAVLink RC_CHANNELS_OVERRIDE report is never written, so the radio's MAVLink model cannot arm or fly. Arming and RC
loss are decided on core 1 (see Flying, props off). Not built: a rule about which link has authority or what a mode
change means.

Seen on 2026-11-15, transmitter on in its MAVLink model, ground station on the transmitter backpack's
WiFi (UDP 14550): the port locked on MAVLink at start and held 300 s (100 stick reports and 90
RADIO_STATUS per second, `rssi` 254, `txbuf` 100, slowdown 0, row OK, link up, health bit set);
`marv-link-test` over the air, 30 s: HEARTBEAT 30, SYS_STATUS 30, MARV_DEVICE_STATUS 30, GPS_RAW_INT 15,
nothing else, 0 lost, 133.9 B/s; `devices --for 10` showed all 7 rows; `param set` over UDP refused by
the station ("this link is operate-only", status 4, no write on the board); `param list` over UDP got all
25 within 60 s; a text reached both links once; a USB gyro calibration with the radio up kept its text
before its ACK. CRSF mode has run since (2026-11-16: RC_CHANNELS and MANUAL_CONTROL on USB, RCIN in the
SD log). Not yet run: the transmitter switched off in either mode, a command over the radio.

## Status LED

Run on the board on 2026-11-18 (commit 51651e2, flash log `logs/marv-v2-firmware-20261118-213111.log`: 15 PASS, 0 FAIL;
the ESC cadence unchanged: 1000 exchanges/s, longest spacing 1.24 ms). Seen by the owner, in his words: "bring up worked,
all 3 clicks the led functioned as expected". The bring-up was a reboot from the station (white, the nine-blink roll call,
then the pulse); the three clicks were a motor-test spin (expected: solid purple), `LED_BRIGHT` set to 3 and back to 1
(full brightness, then dim) and the battery unplugged and plugged back (a yellow pulse, then back). This is what the owner
observed, not a measurement. Not seen on the board: the red double blink (no vital sensor failed), the calibration pattern,
the save and calibration flashes, `LED_BRIGHT` 0 and 2, the green pulse. The display logic, `fw_core::led`, has host tests.
Design: `_plan/decision-status-led-20261118.md`.

The board's one status LED (D20, a WS2812C on GPIO43) shows the vehicle's state, decided by the firmware (nothing in the
station). `src/led.rs` is a core-0 task (watchdog id LED = 19) on PIO1 state machine 1 with `PIO1_IRQ_0`, using the
`ws2812c` driver; every 20 ms it reads the device rows, the motor test, the calibration flag, the open log and the GNSS
sample (and `LED_BRIGHT` every 260 ms: the name lookup holds the parameter store's lock, which both cores share), asks the pure `fw_core::led` what to show, and writes the LED only when the colour or the
brightness changed. If the PIO timing cannot be built the task logs one line and ends: nothing else depends on the LED.
It has no device row and no cross-core cell.

**Boot.** From the start: steady white. At 6.0 s after boot (the ESCs are stable from about 5.4 s) a roll call of nine
blinks, 200 ms on and 150 ms off, always in this order: IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, ESC. Each blink is
green when that device's row is OK, red when it is FAILED and yellow for anything else (ABSENT, STARTING, DEGRADED,
RECOVERING), read at the moment of the blink (with the radio off the RC blink is yellow). Then the running display.

**Running.** One soft pulse per second (the brightness rises and falls smoothly between 15 % and 100 % of the level);
the colour is the state, and the highest row that applies wins:

1. Red, double blink (two 100 ms blinks, then dark to the end of the second): IMU1, IMU2, BARO, MAG or BATT is FAILED.
2. Purple, solid: a motor is being commanded (the motor test's spin is active, or the ESC task is sending the flight step's throttles, which happens only while armed).
3. Blue and white alternating at 5 Hz: a calibration is running.
4. Yellow pulse: any row is DEGRADED or RECOVERING; GNSS, RC, SD or ESC is FAILED; or no log is open on the SD card.
   RC or ESC merely ABSENT or STARTING (radio off, no battery) is not yellow.
5. Green pulse: the latest GNSS sample is younger than 3 s and has a 3D fix or better.
6. Blue pulse: otherwise.

**Flashes.** Three quick flashes (100 ms on, 100 ms off) drawn over the running display, then back to it: green when a
settings save (command 245, param1 = 1) or a calibration (gyro, accelerometer, compass) ends ACCEPTED, red when one of
those ends FAILED (a calibration that runs out of time counts as FAILED), and red when core 0 refuses an arm (see Flying,
props off). A cancelled calibration shows nothing. During the boot (the white and the roll call) an event waits until
the roll call has finished; at most one event waits, and a newer one replaces an older one.

**Not shown, on purpose:** sensor reads, telemetry, radio or USB traffic, parameter writes, low battery (the firmware has no
low-battery threshold), the watchdog's reset cause.

**Brightness:** the parameter `LED_BRIGHT` (see Parameters), 0 off, 1 dim (the default), 2 medium, 3 bright. With the
feature `esc-off` the ESC row reads FAILED, so the running display is a yellow pulse. A build of `sd-fat-probe` has no SD
row (the SD blink is yellow) and no log.

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
| usb-cdc | <https://github.com/Lmx154/usb-cdc> | bytes over the board's USB port (CDC-ACM), the link to the ground station |
| mavlink-device | <https://github.com/Lmx154/mavlink-device> | the board's MAVLink layer on any byte stream (with the `marv` dialect) |
| rc-port | <https://github.com/Lmx154/rc-port> | the ELRS receiver's UART: CRSF and MAVLink, with its `crsf-codec` |
| ws2812c | <https://github.com/Lmx154/ws2812c-rs> | the WS2812C status LED on GPIO43, PIO transport (feature `embassy-rp`; pinned at faf1eb5) |

The `qmi-psram` (PSRAM) driver exists in its own repository but is not a dependency of the firmware: it does not use the PSRAM yet. The `ws2812c` driver (status LED) is a dependency: see Status LED.

For what each driver offers and how to use it, read the README in its own repository. What was proven
on the board, what the firmware has to respect, and what is still open: the Measured section of each driver's README.

## Build and run

Needs the Rust toolchain with the `thumbv8m.main-none-eabihf` target, `probe-rs`, python3 and
arm-none-eabi-binutils (`readelf` and `objdump`, for `tools/check-ram-code.py`), and a debug probe connected
to the board.

```
cargo build --release     # builds for the board (target set in .cargo/config.toml)
tools/run.sh 90           # build, flash, run for 90 s, print the report lines
```

Between the build and the flash, `tools/run.sh` runs `tools/check-ram-code.py` on the ELF (see Flight core in
SRAM) and writes its output at the top of the log: the check must pass (exit 0), or with the feature
`ram-negative` it must fail (exit 1); on any other result the script flashes nothing and exits 3. A binary
flashed some other way skips this check. `tools/run.sh` takes the probe lock (`marv-v2-probe.lock` in `XDG_RUNTIME_DIR`), so it waits if another
test is using the probe. Its full log is kept under `logs/` (git-ignored). The firmware runs until the
script's timeout ends it, so exit status 124 is the normal end. Expected output: one `[INFO]` line per
driver (proof that it is linked), thirteen start-up checks, each `[PASS]` or `[FAIL]` (eight with `LOG_BACKEND_TYPE` = 0): EXTEXCLALL set on
core 0, `DMA_IRQ_0` not enabled on core 0, EXTEXCLALL set on core 1, core 1's VTOR is its vector table in SRAM, `DMA_IRQ_0` enabled on core 1,
`DMA_IRQ_0` still not enabled on core 0, the watchdog fed by its monitor, `PONG` follows `PING` within 200 ms, and the five DMA checks once the SD bus is built (INTE0 = the flight core's channels, INTE1 = channel 4, INTE2 and INTE3 = 0, `DMA_IRQ_1` on and `DMA_IRQ_0` off on core 0, `DMA_IRQ_1` off on core 1); then a `[STAT]` line every 10 s: uptime, the largest `PONG` age seen (healthy: far below
200 ms), `PING` and `PONG`, the bytes and frames the node wrote and the bytes it read on USB, the link
state; and a `[STAT]` line per device (see Device layer).

From the ground station: the flight controller's own USB port enumerates as a CDC device (VID 0xc0de,
PID 0xcafe, product "MARV V2 firmware"; on Linux the second `/dev/ttyACM*` besides the debug probe's).
The node writes nothing until the station opens the port (DTR held 100 ms, or its HEARTBEAT). Then for
example `marv-link-test --connect serial:/dev/ttyACM1 --count 0 --duration 60` (MARV-GCS) counts one
HEARTBEAT, one SYS_STATUS and twelve `MARV_DEVICE_STATUS` (4 per batch every 333 ms, cycling the seven registry rows, the RC row and the ESC row) per second,
10 ESC_STATUS and 1 ESC_INFO per second while the flight core's ESC data is younger than 1 s,
25 HIGHRES_IMU, 25 SCALED_IMU2 and 25 SCALED_PRESSURE per second, one GPS_RAW_INT per second while the fix is younger than 3 s and 5 RC_CHANNELS and 5 MANUAL_CONTROL per second
while stick reports are fresh (MANUAL_CONTROL: CRSF sticks only), all from system 1 component 1, and the STATUSTEXT
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
  LINK, SUPERVISOR, BEAT, ACTION (a command's action inside the link task, so a settings save or reload runs under it), BARO, LED (19), TEST (a check
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
| `sd-fat-probe`, `sd-fat-format`, `sd-fat-paced` | probe P3 of the SD logging design (`src/sd_probe.rs`): from 20.5 s a core-0 task builds the SD bus (PIO0 SM0, DMA channel 4 on `DMA_IRQ_1`), mounts the FAT32 volume of MBR partition 1, writes, reads back and compares `LOGS/P3TEST.BIN` (16 MiB) and un-mounts, yielding at every card access; `sd-fat-format` first writes a new MBR and formats the card FAT32 (erases it). Measured 2026-11-15 (logs 222457 format, 223053 probe): every probe check PASS, stalls begun 0, no reset; but while the card was written the flight core's IMU rates fell by 1 to 6 % and its gaps over 1.5 P per 10 s rose 10 to 50 times (back to normal once the probe ended), so the SD path is not yet fit for a burst. `sd-fat-paced` (also: instead of the burst, 150 s at the logger's duty, one 4 KiB write every 700 ms and a flush each second): measured 2026-11-15 (logs 224159 with `xip-stat`, 224604 without), every check PASS, IMU rates within 0.2 % of the normal build, XIP misses within 3 % of idle; with `xip-stat` (223918) the burst doubled the XIP misses per second. With the flight core in SRAM (2026-11-16, log 20261116-000838) the burst leaves the IMUs unchanged: see Flight core in SRAM |
| `inject-sd-no-card`, `inject-sd-wrong-format`, `sd-irq-negative` | the SD task's test hooks (every card `init` answers `NoCard`; block 0's partition type handed to the MBR check as 0x07) and the negative control of Q1-I1 (INTE0 bit 4 set before the SD register check, cleared after it): rows and `[FAIL]` lines only, no stall |
| `save-loop` | 100 saves; no stall, no reset (measured); once per power-up or flash |
| `fake-armed`, `corrupt-newest`, `guard-negative`, `save-foreign` | boot-time settings checks (corrupt-newest programs one byte at every boot); no stall |
| `hang-core0`, `hang-flight`, `spin-1s`, `hang-irqoff` | the watchdog checks below, each once per power-up or flash |
| `wdg-scratch-probe` | no monitor; at most two resets by design |
| `inject`, `inject-imu1`, `inject-imu2`, `inject-imu` | internal, switched on by the features above |
| `inject-late` | with an IMU injection feature, the injection starts 60 s (not 5 s) after the first good sample; no other effect |
| `xip-stat` | measurement only: at start the QMI flash read mode (registers in hex and decoded) and the cost of one XIP cache miss (three passes over 32 kB of unused flash, one control pass), then an extra `[STAT]` line with the XIP cache accesses, hits and misses since the previous line; reads QMI and XIP_CTRL, writes only the two XIP counters; the passes block core 0 for a few ms at start |
| `stack-negative` | NEGATIVE CONTROL of core 0's stack guard (MSPLIM = `_stack_end`): 10 s after a start the watchdog did not cause, a core-0 task recurses to 2 KiB below `_stack_end`; with the guard it takes a HardFault (nothing is logged after its `[INFO] stack-negative:` line; under `probe-rs run`: "Firmware exited unexpectedly: Exception", HardFault escalated; with `NO_CATCH_HARDFAULT=1` the core locks up and the watchdog resets it: "TimedOut, record false", measured), without it `[FAIL] ... no fault` and the statics below are overwritten |
| `ram-negative` | NEGATIVE CONTROL of `tools/check-ram-code.py`: `now_us` is linked into flash (`.text.ram_negative`, never inlined), so the checker must FAIL naming it; `tools/run.sh` requires that failure before it flashes. On the board the build runs normally (core 1 then reads flash on every sample) |
| `esc-off` | the ESC task is not built or spawned: nothing touches PIO2, DMA channel 5 or GPIO36-39 (the IMU baseline of the ESC board checks); the ESC row reads FAILED, stuck; not for a release |
| `esc-no-edt` | no EDT burst starts (`edt_bursts` stays 0); Stop, the motor test and flight throttles go out as without it (the negative control of extended telemetry); not for a release |
| `xip-park` | the board check of the flight core in SRAM (`src/xip_park.rs`), once, 30 s after a start the watchdog did not cause: a 1 s window of XIP accesses with both cores running (over 100 000: the counter runs), then core 0 parks 1.0 s in SRAM with its interrupts off and the XIP accesses counted meanwhile, core 1's, must be exactly 0 while each IMU gains at least 790 samples and no error; with `xip-park,ram-negative` at least 1000. One "core 0 stalled 1.0 s" text, no reset. Not together with `xip-stat` |

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
the host (309 tests, and 4 doc tests); the binary uses every module listed below. Design: `~/Documents/MARV-V2-drivers/_plan/decision-foundation-20261113.md`.

| Module | What it is |
|---|---|
| `xcore` | Lock-free cells between the two cores: `Seqlock` (flight core to core 0, one writer that never waits), `TripleBuffer` (core 0 to the flight core, wait-free both ways), `Kick` (request bits plus a lock-free wake), `LogRing` (the SD logger's single-producer single-consumer byte ring for whole records: a record goes in whole or not at all, drops counted, wait-free both ways, usable from either core). The only module with `unsafe`. Tested with two host threads over 10^7 writes (`LogRing`: 10^6 random operations against a model, and 4 x 10^5 records between two threads); statics using them must live in the internal SRAM, and core 1 must be started with embassy-rp's `spawn_core1` (it sets the bit that makes atomics work across the cores) |
| `dflog` | The log format, pure: ArduPilot DataFlash `.BIN` records, one table of the 19 types (FMT, PARM, MSG, IMU, BARO, MAG, GPS, BAT, RCIN, DEV, DSF, NAV, SETP, DMD, MOT, ESC, RCS, FDS, GPA) from which both the FMT records and the encoders' lengths come, the 1691-byte header, GPS week and ms from UTC (integer); `dflog::walk::Walker`, an incremental reader of our own files (per-type counts and first/last `TimeUS`, bad bytes, a truncated tail, the FMT check) |
| `mbr` | Block 0 of the SD card: MBR partition 1 of type 0x0B or 0x0C inside the card, or why not (no 0x55AA, protective GPT, exFAT/NTFS, out of range) |
| `logfile` | The SD log files' rules, pure: log numbers 1 to 500 then 1 again, the 8.3 name `NNNNNNNN.BIN` and its parse, `LASTLOG.TXT` (format, parse, the last-number rule), and the clean-up planner (one deletion per call, the oldest log in ring order, from the sum of the log sizes and the volume size; "card full" under 8 MiB) |
| `handshake` | Arming and saving settings can never both win: two flags, each side stores its own, then reads the other's; arming also needs core 0's permit, a third flag cleared at every command admission before it reads the armed flag |
| `arming` | The pre-arm rule, pure: the reasons arming is not permitted (booting, IMU1 not OK, ESC not OK, busy) and the refusal text |
| `health` | The per-device health state machine: Starting, Ok, Degraded, Recovering, Failed, Absent; error score, start retries, 4 recovery attempts with back-off, the flap limit. Events and time in, the next action out |
| `params` | The parameter store over a static table of definitions: ranges enforced (a refused write keeps the old value), a validator per device for whole-configuration checks, per-device "apply" masks, and the `ParamStore` trait of `mavlink-device`. The tables are the firmware's (`crates/fw-settings/src/`, one file per device) |
| `record` | The settings record for the two 4 KB flash sectors of the `SETTINGS` region: "MRVS", sequence, 24-byte entries, CRC-32; the choice of the current sector at boot |
| `scratch` | The watchdog's record in four scratch registers (layout version 2) |
| `admission` | What happens to a MAVLink command: classification of 241, 245, 246 and the project's 42430, the admission order, and the books of the one long action with its progress and its single final acknowledgement |
| `sys_status` | From the device rows to SYS_STATUS's three masks, the HEARTBEAT state and the `MARV_DEVICE_STATUS` row |
| `calib` | Gyroscope, one-position accelerometer and compass calibration, pure: samples in (per IMU, a sample counted once), a per-IMU offset or the reason for the failure out; Betaflight's gyro rule (1.25 s mean, moving when the standard deviation exceeds the threshold) |
| `selftest` | A device self-test as its result crosses from the device's task to the action: the verdict and the failed axes as a bit mask (never figures), the action's final result and its one STATUSTEXT |
| `watchdog` | The watchdog's pure decisions: the monitor's feed rule, the task-id table and the boot report |
| `i2c` | The I2C transfer deadline (twice the time on the wire, at least 4 ms, as ArduPilot) and the rule for a bus after a timed-out transfer: refused until the bus is cleared and rebuilt |
| `rotation` | Sensor orientation: ArduPilot's rotations 0 to 15 (flat mountings, right side up or upside down, in 45-degree yaw steps) as plain 3x3 matrices; runs from SRAM because the flight core calls it for every IMU sample |
| `rc` | The radio receiver's row (RC): whether the radio link is up, from what the receiver port reports; report only, no health machine |
| `text_ring` | The STATUSTEXT ring for the ground-station links: 8 entries, a read cursor per link (each text reaches each link once, in order), the oldest dropped and counted for a link more than 8 behind, so a slow or absent link holds back nobody; 2 sticky slots (the boot report) kept until every link took them, taken first |

`fw-settings` holds the device-specific pure logic: per device, the parameter table and the pure
`settings_to_config` with its validator (today `baro`, `mag`, `batt`, `imu` and `rc`, the last with the RC stick normaliser, no validator; `baro` is over the `bmp581` driver's `Config`, without the
driver's `defmt` feature). Device-specific pure logic goes here, generic logic stays in `fw-core`. Tested on
the host with `cargo host-test -p fw-settings` (48 tests, the first six of the first 23: defaults equal `Config::new()` and P = 19 978 us,
every refusal text, every offered rate within 1 % of its name and all distinct, the table accepted by
`Params::new` with every default passing the validator, every offered value of every parameter accepted or
refused only by the rate-against-oversampling rule with the refused sets listed, and the real store
refusing before storing); it also builds for the board (`cargo build -p fw-settings --release`).

`fw-flight` holds the flight core's liveness answer (`liveness`: waits on a `Kick`, copies `PING` into
`PONG`), the IMU device loop (`imu`) and the flight step (`step`, `gnc`, `gate`), both under Flight-core device loop. It must not depend on
`embassy-time`, `embassy-sync` or `defmt`; check it with
`cargo tree -p fw-flight -e normal --depth 1 | grep -cE 'embassy-time|embassy-sync|defmt'` (prints 0).

`.cargo/config.toml` sets the board target for the whole workspace, so the two commands are:

```
cargo host-test -p fw-core    # tests on the host (alias: `cargo test --target x86_64-unknown-linux-gnu`)
cargo build --release         # the firmware for the board (thumbv8m.main-none-eabihf)
```

`cargo host-clippy -p fw-core -p fw-flight --all-targets` is the matching host lint. The host alias
assumes an x86_64 Linux host; on another host, pass your own `--target`.

## ESC telemetry

Design: `~/Documents/MARV-V2-drivers/_plan/decision-esc-telemetry-20261118.md`. Code: `src/esc.rs`, `fw_flight::esc`,
`fw_core::esc`.

**The firmware sends DShot Stop frames on GPIO36-39 from every boot, whether or not a battery is connected**
(not with the feature `esc-off`).

- Core 1's task `esc` drives the four ESCs with `dshot`'s `PioDshot` (PIO2 SM0, DMA channel 5, DShot300, bidirectional):
  `start()` once at boot (lines released 3 s, a 25 ms break; its waits through `SpinDelay`), then one exchange
  every 1 ms for ever (`fw_flight::esc::next_deadline`: a stall gives one late frame, never a burst). Driver line 0 is
  GPIO36 = M4 ... line 3 is GPIO39 = M1.
- Frames: Stop on every line, except command 13 (extended telemetry on) in bursts of 10 frames on one line once
  that line has answered for 1 s with no extended reply in the last 1 s, then a 1 s hold-off (`Esc::frames`). The
  only throttle is the motor test's (see Motor test below). Nothing stops on errors; there is no lapse rule.
- After every exchange the task publishes its counters and latest values to the seqlock `esc::ESC`; core 0 does all
  conversion. No timer, log or lock in the task.
- USB only: ESC_STATUS (291) at 10 Hz (fastest 50 Hz) and ESC_INFO (290) at 1 Hz (fastest 10 Hz), neither sent while
  the data is older than 1 s. Their fields: the station's `docs/link-contract.md` section 14 (MARV-GCS). The rpm uses
  `MOT_POLES` (see Parameters), read on core 0 for every message.
- The ESC row (`MARV_DEVICE_STATUS` device 12) is report only, like the RC row, recomputed for every message
  (`fw_core::esc::verdict`): STARTING (start sequence running, or ended less than 5 s ago and not all four
  answering), ABSENT (no ESC has answered this boot), OK (all four answering: a good reply within 100 ms), DEGRADED
  (1 to 3, error TIMEOUT), FAILED (none after some good reply, TIMEOUT; or the data older than 1 s, STUCK). It sets
  no SYS_STATUS bit.
- Core 0 prints `[STAT] ESC:` every 10 s: the row, exchanges, starts, longest spacing, longest exchange and shortest exchange (0 before the first), and per
  motor good replies, non-replies, dropouts, EDT bursts and the latest raw voltage, temperature and current, and at
  the end of the line `throttle frames N` (the throttle frames the flight core has sent since boot).
- Since P6 the frames carry the flight command's throttles on all four lines while it spins (see Flying, props off,
  below); `EscStatus` counts them (`flight_spin`, `flight_frames`, `flight_read_failures`; not in the `[STAT]` line).
- Not built: the ESCs' serial telemetry on GPIO34, the passthrough, the ESC row in the SD log, ESC messages on the radio.
- Measured on the board (2026-11-18, battery connected, motors fitted, no propellers, probe-rs attached). F3a at
  4cfb1f0: logs `20261118-191955` (default), `20261118-192214` (`esc-off`), `20261118-192403` (default). F3b at
  0d25909: `20261118-193433` (default, one settings save), `20261118-193820` (`esc-no-edt`), `20261118-194106`
  (`xip-park`), `20261118-194357` (`xip-park,ram-negative`), `20261118-194637` (default).
  - Start-up checks: 15 `[PASS]`, no `[FAIL]`; `PIO2_IRQ_0` enabled on core 1 and not on core 0; INTE0 0x2f.
  - Online (row OK, ESC_INFO `info` 15) about 4.1 s after boot; the row dips to STARTING for about 0.7 s during the
    ESCs' arming tune (one dropout per line) and is stable from about 5.4 s.
  - 1000 exchanges/s (10 000 ±1 per 10 s); longest spacing 1.1-1.3 ms.
  - Error rate 0 after 5.4 s on all four; about 940 non-replies per line before.
  - ESC voltage M1 16.0 V, M2 15.25-15.75 V, M3 15.5 V, M4 15.5 V beside the board's 15.0 V (SYS_STATUS 14.99-15.02 V);
    temperature M1 46-47, M2 32-33, M3 30-32, M4 42-43 degC. The ESCs' raw current reads 153-162 A at rest, hence NaN
    on the link. rpm 0 in every ESC_STATUS.
  - IMU rates unchanged against `esc-off` (192214): IMU1 803.450/s, largest gap 1416 us, longest run 616 us; IMU2
    800.600/s, 1450 us, 266 us. With the ESC task (191955, 192403): IMU1 803.433-803.450/s, 1411-1487 us, 711-737 us;
    IMU2 800.600/s, 1406-1847 us, 204-276 us.
  - One settings save (193433): 45 ms stall, 44 frames lost, longest spacing 44.3 ms, all four stayed online,
    `starts` 1.
  - `esc-no-edt` control (193820): `info` 15, voltage NaN and temperature no-data (32767) on all four for the whole run.
  - `xip-park` with the ESC task running (194106): 0 accesses with core 0 parked; 184 035 in the `ram-negative`
    control (194357).
  - Battery pulled and plugged back by the owner with the station (figures from the lead's reading of the station
    recordings, not from these logs): row FAILED/TIMEOUT and `info` 0 within the same second; back online within a
    second of power, voltage after about 2.3 s; a boot without battery reads ABSENT, then OK after plugging in. The
    ESCs' bootloader v1 starts the application with frames already running, so the decision's fallback restart was
    not built.
  - Exchange duration: shortest 156-157 us. With 144.6 us for the frame and capture, that leaves 11.4 us for decode,
    frame choice, wake and two clock reads (1 us resolution). The driver decision's 1500-cycle decode tripwire is
    10 us: decode alone was not isolated, it is bounded above by 11.4 us. Longest 255-1358 us, which includes other
    core-1 tasks' polls and, once, the save stall.
  - Not measured: serial telemetry. Throttle: see Motor test below.
- Known behaviour: when the battery is plugged in more than 5 s after boot, the ESC row can read FAILED/TIMEOUT for
  under a second during the arming tune.
- Noticed (2026-11-18, not analysed): an RC UART framing/break burst after `xip-park`'s core-0 park (33 warnings,
  logs `20261118-194106` / `20261118-194357`; 1 overrun on 2026-11-16); one RC UART overrun at a settings save
  (`20261118-193433`).

### Flying, props off

**This firmware spins all four motors when the pilot arms it on the radio. Keep the propellers off unless the owner
says otherwise.** Not yet checked on the board (`docs/handoff.md`).

- Arms: channel 5 goes from off to above 1800 us on a fresh CRSF report, with throttle low, the attitude valid, lean
  within 30 deg, no settings save running and core 0's permit set (`HANDSHAKE.try_arm()`); the motors then run at idle
  at zero throttle and the status LED is purple.
- Core 0's permit (decision-prearm-20261119; `fw_core::arming`, set or cleared every 10 ms by the supervisor): the
  roll call has ended (9.15 s after boot), IMU1's and the ESC row are OK (with no battery the ESC row is ABSENT), and
  no calibration, action or motor-test spin is running. A refused arm: the motors stay still, the LED does not turn
  purple, it shows three red flashes (a refusal during the boot shows its red flashes right after the roll call), and
  the station gets one WARNING text "Arming refused: booting" / "IMU1 not OK" / "ESC not OK" / "busy". Cycle the
  switch to try again: a switch held on while the permit comes does not arm. Clearing the permit while armed changes
  nothing.
- Disarms (Stop at once): the switch off; no new CRSF stick report for 1.0 s (`RC_FS_TIMEOUT`; the switch must then be
  cycled to re-arm); a refusal by the controller while armed; a gap of more than 100 ms between flight steps
  (`FLIGHT_GAP_DISARM_US`, the IMU loop's first recovery back-off; the gate disarms at its next step, and the ESC task
  clears `ARMED` meanwhile when the newest spinning command is that old, so core 0 sees not armed; the switch must be
  cycled to re-arm). The ESC task also sends Stop when the newest flight command is older than 10 ms
  (`MOTOR_HOLD_MAX_US`), and the motors resume with the next step if the gate is still armed (a gap of 10 to 100 ms is
  ridden through). A non-finite IMU1 value never reaches the estimator: that step gives Stop and disarm.
- While armed or while flight throttles go out, core 0 refuses every action (saves, calibrations, restarts) and a
  motor-test spin (TEMPORARILY_REJECTED); a motor-test stop is still accepted. The motor test runs only while disarmed.
- Gains and limits are constants in `fw_flight::tuning` (`Tuning::DEFAULT`) for now; no parameter changes them. That includes the estimator's accelerometer cut-off `ins_accel_lpf_hz` (10 Hz).

### Motor test

Design: `~/Documents/MARV-V2-drivers/_plan/decision-motor-test-20261118.md` (amendment A, "lean"). Code:
`fw_core::admission` (`classify_motor_test`, `MotorTest`), `src/actions.rs` (`motor_test`), `src/esc.rs` (`TEST`,
`report`), `fw_flight::esc` (`Esc::frames`, `capped`). The station's field table: `docs/link-contract.md` section 15
(MARV-GCS). **Propellers off only.** Run on the board on 2026-11-18 (the last item below).

- The command: `MAV_CMD_DO_MOTOR_TEST` (209), `COMMAND_LONG`, on every link the same way (the firmware does not check the
  link; link contract section 9). param1 motor 1-4 (M1-M4, the ESC_STATUS order); param2 0 (percent); param3 throttle
  0 to 5.0 % (0 = stop now); param4 duration above 0 up to 3.0 s (ignored for a stop); param5 0 or 1; param6 0 or 2;
  param7 0. Anything else, NaN included, is DENIED with a STATUSTEXT `motor test: <reason>`. The level sent to the ESC
  is round(param3 x 20) of 2000, at least 1 (the bench's mapping, `dshot_spin` `step_level`): 5 % is level 100.
- The answers: a stop is ACCEPTED in every state. A spin is ACCEPTED at once (no IN_PROGRESS) unless another long
  action is in the books, the vehicle is armed or flight throttles go out: TEMPORARILY_REJECTED. A spin while one runs
  replaces it. While a spin runs, every other command that starts an action is TEMPORARILY_REJECTED (core 0 passes
  "armed, or a spin is active" to the book; its own record: the last accepted spin's `until_us` not reached and no stop
  accepted since). There are no pre-checks of the ESCs or the battery and no automatic stops on telemetry. With the
  feature `esc-off` the command is UNSUPPORTED.
- The path: core 0 admits the command (`MotorTest`), gives it a new `seq` (from 1, 0 skipped on a wrap; every accepted
  spin or stop gets one), sets `until_us` = now + duration on the board timer, and writes the whole
  `TestCmd { seq, line, level, until_us }` to the cell `esc::TEST`, a `TripleBuffer<TestCmd>` (writer in `actions`,
  reader taken before core 1 starts and moved into the ESC task, which reads it wait-free at every exchange). Core 1
  ends the spin on its own clock: at `until_us`, at 3 s from the spin's first throttle frame, on a stop, on a command with a
  new `seq` (replaced), or if its own guard finds the level above the 5 % cap (`capped()` is the motor test's only constructor
  of a `Throttle`; the flight command's is the gate's). A hung core 0 cannot extend a spin. During a spin the tested line gets the throttle and the other
  three Stop; no command 13 is sent, and a running EDT burst is cut.
- The report (core 0, every 10 ms tick): when the status shows a new ending (its `seq` changed), one line
  `[INFO] motor test M<n>: level L, ended by <cause> after <ms> ms` (cause: expiry, 3 s limit, stop, replaced, cap; ms
  from the first throttle frame to the exchange that ended it; `never started` when no throttle frame was sent), and a
  WARNING STATUSTEXT for a `cap` ending only. The accepted commands print `[INFO] motor test M<n> spin accepted: ...`
  and `[INFO] motor test stop accepted: ...`. Core 1 never logs. `[STAT] ESC:` ends with `throttle frames N`.
- Limits and risks: propellers off only; never measured above 5 % or with two motors. If core 1 hangs, no frames are
  sent and the ESC's own cut-off (about 510 ms without frames) is the last resort. If the link drops during a spin it
  runs to its end (at most 3 s); nothing repeats or holds it. The ESC task must be running: if the driver was not
  built the row reads FAILED but a spin is still ACCEPTED and nothing turns.
- Tests: `cargo host-test -p fw-core` (admission: `motor_test_*`), `-p fw-flight` (the frames and endings, M2-F1).
- Measured on the board (2026-11-18, commit 7b3cdb6, flash log `20261118-204512`: 15 PASS, 0 FAIL; battery on, no
  propellers; the owner clicking Spin and Stop in the station's Motors tab; figures read from the station's own
  recordings `2026-11-19_02-48-25Z.tlog` and `2026-11-19_03-03-53Z.tlog`, ESC_STATUS at 10 Hz, so times resolve to 0.1 s):
  - Identity: each of M1-M4 at 5 % for 2 s turned one motor, and rpm appeared only on that motor's ESC_STATUS element.
    The owner named them: M1 rear left counter-clockwise, M2 front left clockwise, M3 rear right clockwise, M4 front
    right counter-clockwise (the same as `docs/pinout.md`, 2026-11-13).
  - rpm at 5 % (level 100), median over the last second: M1 1809, M2 1899, M3 1822, M4 1840 (the bench's
    2026-11-13 median for M1 was 12 755 eRPM, 1822 rpm). At 1.65 % (level 33) M1 started and ran at 898 rpm.
  - Duration: 17 spins, all ACCEPTED; a 2 s spin showed rpm for 2.0-2.1 s and a 3 s spin for 3.0 s, each ended by
    itself.
  - Stop: a stop 0.67 s into a 3 s spin; rpm was 0 in the next ESC_STATUS (0.70 s after the spin began).
  - Replace: M2 commanded 0.35 s into M1's spin; M1 went to 0 and M2 ran its own 3 s.
  - Link loss: the USB cable pulled 1.2 s into a 3 s spin and 0.4 s into a 2 s spin; the owner saw the motor stop by
    itself both times; after reconnecting, rpm 0, all four ESCs online, no reset.
  - All four ESCs stayed online (`info` 15 in every ESC_INFO) with 0 new reply errors over the 914 s recording.
  - Not run on the board: a refused command (the station cannot send one outside the table; host-tested), a save
    or calibration during a spin (host-tested), a spin with the battery unplugged, `xip-park` and the IMU rates
    during a spin (the RAM-code checker passed at the flash), the `[INFO] motor test` lines (the probe was detached).

## Flight core in SRAM

Design: `~/Documents/MARV-V2-drivers/_plan/decision-ram-code-20261115.md` (the owner: "run from PSRAM or RAM
not flash"; the PSRAM sits behind the same 16 kB XIP cache as the flash, so this is internal SRAM). Since
2026-11-16 the flight core executes only from internal SRAM once `core1_main` has started, so it no
longer shares the XIP cache with core 0, whose misses under card work slowed it (facts-sd-logging C2).

- In SRAM: everything core 1 can run after `core1_main` begins (the IMU tasks and the ESC task, `fw_flight`'s imu,
  liveness, esc, step, gnc, gate, mahony, pid, alloc and quat modules, `fw_core::handshake`, `libm` and the soft
  double-precision routines its `sinf`/`cosf` use, the binary's `flight::Stepper` and the seven `fw_core::dflog` encoders it calls, both IMU drivers, the `dshot` crate, embassy-rp's SPI, DMA, GPIO, PIO (with `pio_core`) and multicore code, the executor's poll and wake path,
  `AtomicWaker`, `fw_core`'s health and xcore modules, the memory functions and the 64-bit division, the handlers `DMA_IRQ_0`,
  `IO_IRQ_BANK0`, `SIO_IRQ_FIFO` and `PIO2_IRQ_0`, the critical-section and time functions), every constant (`.rodata`),
  and core 1's own vector table. Left in flash: core 1's panic paths and the launch trampoline that runs
  before `core1_main`.
- How: `memory.x` adds the output section `.ram_text` (`> RAM AT> FLASH`, inserted before `.text`), whose
  rules pick functions by their v0-mangled names (`.cargo/config.toml` pins `symbol-mangling-version=v0`),
  then all `.rodata`. `src/ram.rs` copies it from its load image in flash to 0x20000000, in assembly, before
  anything else runs: embassy-rp already defines `__pre_init`, so the link wraps it (`--wrap=__pre_init`,
  `build.rs`); the copy, `__wrap___pre_init`, runs first and then continues into embassy-rp's. Link ASSERTs:
  the three handlers lie in RAM, `.rodata` is empty, `.ram_text` is word-aligned, at least 32 KiB of RAM stay
  free above the statics for core 0's stack.
- Stack guards: `memory.x` defines `_stack_end = ALIGN(__euninit, 8)` (cortex-m-rt's own default would be a
  flash address, because `.end_block` is inserted after `.uninit`), and `main` calls
  `embassy_rp::install_core0_stack_guard()` before anything else: MSPLIM = `_stack_end`, so a push below the
  end of the statics is a HardFault (no handler: `probe-rs run` stops at its hard-fault catch). Without that catch
  (`NO_CATCH_HARDFAULT=1`) the core locks up (the HardFault's own stacking is below MSPLIM), `probe-rs run`
  exits "Run loop exited due to a locked up core", and the watchdog resets the board: the next boot reports
  `reset reason TimedOut, record false ... WDG reset: no record` (read with `ATTACH=1`). `spawn_core1` does the same for core 1 with the bottom of `CORE1_STACK`.
  `FEATURES=stack-negative tools/run.sh 45` is the check: 10 s after start a core-0 task recurses to 2 KiB
  below `_stack_end`; it must fault (the log ends after the `stack-negative:` line); a build without the
  guard logs `[FAIL] stack-negative: ... no fault`.
- Core 1's vector table: core 0 copies its own (69 words) into a 512-byte-aligned static before
  `spawn_core1`; the first statement of `core1_main` points core 1's VTOR at it. Start-up check:
  `[PASS] core 1 VTOR is its RAM vector table (0x2002b600)`.
- The guard: `tools/check-ram-code.py <elf>` (python3 standard library, `arm-none-eabi-readelf` and
  `-objdump`), run by `tools/run.sh` before every flash. From the roots `core1_main`, `DMA_IRQ_0`,
  `IO_IRQ_BANK0`, `SIO_IRQ_FIFO` (and `xip_park::park_core0` in that feature) it follows direct branches and
  long-branch thunks, addresses formed by movw/movt pairs, literal loads and `adr`, and the code and data
  pointers inside every constant it reaches. It fails (exit 1, printing each find with its call chain) on any
  function, data object or constant address in flash (0x10000000 to 0x1FFFFFFF) other than the panic
  machinery; a tool error exits 2. Blind spot: a function pointer formed only by core 0 and called on core 1
  (none today), which `xip-park` covers on the board. The patterns are the mechanism and the checker is the
  proof: a pattern change counts only with a passing checker (`docs/resources.md`, rule 10).
- Measured (2026-11-16, probe-rs attached in every run):
  - `xip-park` (log `20261116-001800`): unparked 1 s window 1 667 188 XIP accesses; core 0 parked 1 s in SRAM
    with interrupts off: 0 accesses (HIT 0) while IMU1 gained 803 and IMU2 801 samples, 0 errors. Negative
    control `xip-park,ram-negative` (`20261116-000720`): 159 605 accesses.
  - The checker passes on this build and fails on `main`'s ELF of 96b94b7 (127 functions in flash) and with
    `ram-negative` (naming `now_us`).
  - The 16 MiB SD burst (`sd-fat-probe,xip-stat`, log `20261116-000838` against `20261115-223918`): during
    the burst IMU1 803.49 to 803.61 and IMU2 800.61 to 800.70 samples/s (idle 803.5 and 800.7; before 790 to
    802 and 770 to 799), 0 samples later than 1.5 P per 10 s (before 63 to 541), 0 errors.
    Core 0 got faster too: the 16 MiB write took 16.7 s (1004 kB/s) against 35.5 s (472 kB/s), though its
    longest single write call was longer (130 ms against 72 ms).
  - The normal build, 200 s (log `20261116-001110` against `20261115-233145`): 0 samples later than 1.5 P per
    10 s window for both IMUs (before 6 to 20 and 23 to 53), largest gap 1.63 and 1.67 ms (before 3.0 and
    3.3 ms).
  - The XIP counters now count core 0 alone: about 1.5 million accesses and 0.15 million misses per second
    idle (both cores before: 94 million and 0.74 million).
- Not done or not seen: no run without the debugger attached; the boot-copy time is not measured (the first
  log line's timestamp starts after the copy); no breakpoint was tried in SRAM code; stage 2 (core 0's own
  hot paths, such as the SD write path or the UART and USB handlers, in SRAM) is not built; core 0 still
  competes with itself for the cache (its card work evicts its own code).

## Flash layout and memory map

`memory.x` gives the program `FLASH` 2040K (0x10000000 to 0x101FE000) and reserves the last 8 KB of the
2 MiB flash, 0x101FE000 to 0x10200000, as the `SETTINGS` region for the settings record (`src/settings.rs`;
the only range the firmware erases or programs). The map of the normal build (2026-11-16,
from `arm-none-eabi-size -A` and the linker symbols):

| Where | What | Bytes |
|---|---|---|
| flash 0x10000000 | `.vector_table` (core 0's) and `.start_block` (the RP2350 image definition) | 276 + 40 |
| flash 0x10000140 | the load image of `.ram_text`, copied to SRAM at boot | 111 260 |
| flash 0x1001b3e0 | `.text`: core 0's code | 389 312 |
| flash | `.rodata` | 0 (every constant is in SRAM) |
| flash, after `.text` | the load image of `.data` | 4 548 |
| SRAM 0x20000000 | `.ram_text`: the flight core's code (75 668), then all constants (35 592) | 111 260 |
| SRAM | `.data` | 4 548 |
| SRAM | `.bss` (core 1's 512-byte vector table, at 0x2002b600, among it) | 103 972 |
| SRAM | `.uninit` | 1 024 |
| SRAM, up to 0x20080000 | free; core 0's stack grows down from the top (link ASSERT: at least 32 KiB) | 303 068 |

Program in flash 505 436 B of 2040 KiB; static SRAM 221 220 B of 512 KiB.

## Board documentation

- `docs/handoff.md`: the overview for whoever picks the work up: what is done and proven, what is in progress
  (SD card logging), what is left, and the rules that are easy to break (2026-11-15).
- `docs/handoff-esc-link.md`: the ESC link (DShot, AM32 passthrough, ESC telemetry input) and battery sensing: what the drivers
  already do, the owner's decisions and what is still open for the firmware (DShot telemetry and the props-off motor test are built,
  see ESC telemetry; passthrough, serial telemetry and arming are not).
- `docs/pinout.md`: the board pin assignment.
- `docs/devices.md`: the devices on the board and what was measured on them.
- `docs/resources.md`: who owns which pin, PIO state machine, DMA channel and interrupt, and the rules that follow.

## Licence

MIT, see `LICENSE`.
