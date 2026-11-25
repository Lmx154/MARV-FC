# MARV V2 flight firmware: functional overview

State: branch `mission-switch`, head `d430a96` (the firmware on the vehicle as of 2026-11-24 evening). This file is a map, not
a spec: it says what the firmware does and where the code is. Details live in `README.md` (long, partly out of date, see
section 10), `docs/*.md` and the decision files in `~/Documents/MARV-V2-drivers/_plan/`. Every rule below cites code
(`path:line`, lines are of this head) or the handoff.

## 1. What this is

A quadcopter flight controller firmware in Rust (`no_std`, embassy, edition 2024) for the MARV V2 Rev 1 board.

| Item | Fact | Source |
|---|---|---|
| MCU | RP2350 family, two Cortex-M33 cores, 512 KB SRAM, 2 MiB in-package flash (`docs/pinout.md` calls the part RP2354B), 16 MiB PSRAM (socket U24) | `memory.x`, `docs/pinout.md` |
| Sensors | IMU1 ICM-45686 (SPI0), IMU2 LSM6DSK320X (SPI1), baro BMP581, compass BMM350, GNSS u-blox SAM-M10Q (UART1, UBX), battery on the ADC | `Cargo.toml`, `src/gnss.rs:1` |
| Other | 4 ESCs on bidirectional DShot300 (PIO2), microSD (PIO0, 4-bit), ELRS receiver (UART0), status LED (WS2812C, GPIO43) | `src/esc.rs:1`, `src/sd.rs:1`, `src/rc.rs`, `src/led.rs:1` |

Core split (`src/main.rs`; rule: only lock-free cells and atomics cross, `fw_core::xcore`):

| Core | Runs |
|---|---|
| Core 1, the flight core (16 KB stack, `CORE1_STACK_BYTES` `src/main.rs:185`; all its code and constants execute from SRAM) | IMU1 and IMU2 tasks, the flight step (once per IMU1 sample, default 800 Hz), the ESC task (DShot300 at 1 kHz). No log, no timer, no lock. |
| Core 0, the I/O core | USB MAVLink node (`usb_link`), radio task (`rc_link`), `supervisor` (10 ms tick), baro/mag/GNSS/battery tasks, SD writer and log collector, settings flash, status LED, watchdog monitor, mission service, station command bridge |

Repositories and how they fit (all under `~/Documents/`):

| Repo | Role |
|---|---|
| `MARV-V2-firmware-ms` (this, branch `mission-switch`) | The firmware. Workspace crates: `fw-core` (pure logic: health, parameters, record codecs, mission service, log encoders), `fw-flight` (estimator, controllers, gate, hands-off state machine, guidance), `fw-settings` (parameter tables) |
| `MARV-V2-drivers/<crate>` | One git repo per driver, pinned by commit in `Cargo.toml`: `icm45686`, `lsm6dsk320x`, `bmp581`, `bmm350`, `sam-m10q` (GNSS in use), `lc76g` (older GNSS, not used), `adc-battery`, `dshot` (bidirectional DShot, PIO), `esc-passthrough` (AM32 configurator codecs), `pio-sdmmc`, `pio-uart`, `qmi-psram`, `ws2812c`, `usb-cdc`, `mavlink-device` (MAVLink 2 device layer), `marv-dialect` (message set), `rc-port` (ELRS UART, follows CRSF and MAVLink), `crsf-codec`. Contract text in `MARV-V2-drivers/CONTRACT.md`; design record in `MARV-V2-drivers/_plan/` |
| `MARV-GCS` (branch `mission-switch`) | Ground station `marv-gcs` (egui window) and `marv-cli` (command line); also `gcs-sim`, a simulated vehicle |
| `MARV-sim` | Simulator that runs the firmware's real flight code (see section 8) |

Vehicle-side peripherals/pins/DMA/IRQ ownership: `docs/resources.md`, `docs/pinout.md`. Sensor register facts: `docs/devices.md`.

## 2. Build, flash, test

| Task | Command (from the firmware checkout) | Notes |
|---|---|---|
| Build + RAM guard + flash + run + log | `tools/run.sh <seconds>` | Builds release, runs `tools/check-ram-code.py`, flashes through the debug probe, logs to `logs/`. Exit 124 = normal end; exit 3 = guard failed, nothing flashed; exit 1 "Target device did not respond" = probe lost contact or vehicle power-cycled (`tools/run.sh:1-20`) |
| Re-attach without reset | `ATTACH=1 tools/run.sh <s>` | Only valid while the sources match the flashed image |
| Host tests | `cargo host-test -p fw-flight` / `-p fw-core` / `-p fw-settings` | Alias for `test --target x86_64-unknown-linux-gnu` (`.cargo/config.toml`). Plain `cargo test -p` picks the board target and fails |
| RAM-code guard alone | `python3 tools/check-ram-code.py <elf>` | Fails on any function/constant the flight core reaches that lies in flash (0x10000000..). Negative control: `FEATURES=ram-negative` must fail |
| Other negative controls | `FEATURES=no-fbeat`, `FEATURES=stack-negative` (README "Flight core in SRAM") | |

Rules from the handoff (section 11, 1): board access only through `tools/run.sh`, props off and the station closed before flashing; one
cargo job at a time on the dev machine (it kills builds for memory): wait for `pgrep -x rustc` to be empty, then run cargo under
`systemd-run --user --scope -q -p MemoryMax=4G -p MemorySwapMax=0 timeout <s> cargo ...` (6G for release builds, `-j 2` for
marv-gcs tests). Parameters and the uploaded mission are lost at every flash and power cycle unless saved.

Memory limits of the firmware itself:

| Limit | Value | Source |
|---|---|---|
| Flash for program | 2040 KB; the last 8 KB are the settings region (two 4 KB sectors) | `memory.x` |
| SRAM | 512 KB; flight-core code + constants (`.ram_text`) 273,676 B at d430a96; link assert keeps >= 28,252 B above statics for core 0's stack | handoff section 6; README "Flight core in SRAM" |
| Flight-core stack | 16 KB; used 13,824 B, free 2,560 B at d430a96 (never-run paths: Start/AUTO were run on the bench; BLIND, waypoint arrival, battery-critical exits were not). Raising it is an open owner decision | handoff sections 6, 8.2 |
| Core 0 stack | grows from top of SRAM; guarded by MSPLIM (`install_core0_stack_guard`) | README "Flight core in SRAM" |
| PSRAM | 16 MiB; holds the log ring (64 KB) and log-download handover; core 0 only, never touched by core 1 | `src/psram.rs`, `memory.x` |

## 3. Architecture

### 3.1 Tasks and devices (core 0)

`src/device.rs`: every sensor is a "device row" driven by one generic loop over a `DeviceAdapter` and the `fw_core::health::Health`
machine (states, recovery ladder, stuck check). Rows: IMU1, IMU2, BARO, MAG, GNSS, BATT, SD; RC and ESC are report-only rows. Each
row is published as `MARV_DEVICE_STATUS` (1 per second per row, plus on change). A start-up report prints PASS/FAIL lines
(16 PASS at the last flash). Adapters: `src/imu1.rs`, `imu2.rs`, `baro.rs`, `mag.rs`, `gnss.rs`, `batt.rs`, `sd.rs`, `esc.rs`, `rc.rs`.

### 3.2 Flight-core loop and rates

| What | Rate | Where |
|---|---|---|
| IMU sample (both IMUs, configured at boot from `IMUn_*` parameters) | default 800 Hz | `src/imu1.rs`, `crates/fw-flight/src/imu.rs` |
| Flight step: estimator, shaper, controller, gate, allocator, then the step's log records | once per good IMU1 sample (about 800 Hz) | `src/flight.rs:1`, `crates/fw-flight/src/step.rs:1` |
| ESC DShot output and eRPM telemetry | 1 kHz | `src/esc.rs:1` |
| Barometer / compass / GNSS | about 50 Hz / 100 Hz / `GPS_RATE_MS` (default 200 ms) | README "Device layer" |
| Stick reports (CRSF) / (MAVLink receiver reports) | about 245 Hz / 100 Hz | handoff section 7 |

The flight core verifies its IMUs incrementally (one register per K samples), never a whole `verify` in the sample loop.
Safety on the core itself: the gate disarms if the gap between steps exceeds 100 ms (`crates/fw-flight/src/gate.rs:15`).

### 3.3 Estimator

`crates/fw-flight/src/eskf.rs`, `eskf_out.rs`: `SteerMix` = an ESKF3-style full-aiding filter behind an output predictor (attitude, rates,
horizontal state) plus a zero-delay ESKF2 (`Aiding::ImuBaro`) for the vertical state. Inputs: both IMUs, baro, compass, GNSS. Validity
flags per quantity (`NavState.valid`) decide the "tier": T3 = position usable, T2 = height only, T1 = tilt and rates only
(`hands_off.rs:97-101`). Heading trust is the board's `[STAT] NAV` `valid 0x7f`; a compass health check (`MAG_HLT_STR`, `MAG_HLT_INC`)
can withdraw heading. `mahony.rs` / `mahony_baro.rs` are an older estimator kept in the crate, not run on the board (`src/flight.rs:21-24`).

### 3.4 Controllers

`crates/fw-flight/src/pid.rs` (`CascadedPid`): attitude P into PID rate loop per axis; vertical loop for Altitude; horizontal position P / velocity PI
for Position. Gains and limits are compiled constants in `tuning.rs`, exposed as the 14 `ATC_*` parameters. RPM notch on the gyro rates (`notch.rs`,
`RPM_NOTCH_EN`). Pilot mapping: `tuning.rs` (`ANGLE_MAX` 30 deg, yaw rate 150 deg/s, `PILOT_SPEED_XY`).

### 3.5 Mixer and ESC

`crates/fw-flight/src/alloc.rs` (`CardAllocator`): thrust and torques to four rotor thrusts using the vehicle card (`marv-card` from the MARV-sim repo: mixer matrix,
`K_F`, levels); yaw is given up first, no airmode. `gate.rs`: the only producer of a spinning `MotorCommand` (field private); arming, disarming and the RC-loss rule live
here. `src/esc.rs` sends DShot300 on PIO2 SM0 (GPIO36-39), captures eRPM replies (DMA channel 5); a command older than `MOTOR_HOLD_MAX_US` is Stop. Props-off motor test
(command 209, at most 5 %, 3 s, one line) goes through the same task. ESC telemetry: README "ESC telemetry".

### 3.6 Logging

ArduPilot DataFlash `.BIN` on the SD card (`LOGS/NNNNNNNN.BIN`), readable by mavlogdump / MAVExplorer / Mission Planner, and by `marv-cli log report|tune`.
`src/logger.rs` (core-0 collector, 10 ms ticker) is the one producer of a ring in PSRAM; `src/sd.rs` the one consumer. Flight core pushes IMU/NAV/SETP/DMD/MOT every
step while armed (182 B per step), ESC every 4th, RCS per stick report; core 0 adds IMU2 (25 Hz), BARO, MAG, GPS, BAT, RCIN (10 Hz), DEV, MSG, PARM.
About 166 kB/s armed. Encoders: `crates/fw-core/src/dflog.rs`. Hands-off events appear as "HO" messages (state changes, e.g. `HO 6 LAND`, `step.rs:1793`). Download
over MAVLink: `src/logdl.rs`, `marv-cli logs list|get`. README "SD card", "Logging".

### 3.7 Settings and parameters

All parameters are MAVLink parameters in one store (`src/store.rs`, tables in `crates/fw-settings`, 128 at this head). Changes are live and volatile; written to
flash (two 4 KB records A/B, `src/settings.rs`) only by an explicit save (`MAV_CMD_PREFLIGHT_STORAGE` param1 = 1; `marv-cli param save`). `Apply::Live` parameters reach core 1
through triple buffers (`src/flight.rs:454`); several are taken only at the arming step or on a disarmed step. Commands (restart a device, self-test, calibrations: gyro,
one-position accel, quick compass; reset, reboot) are in `src/actions.rs`, admitted by `fw_core::admission`.

### 3.8 Watchdog and status LED

Hardware watchdog 1600 ms, fed only by a monitor on a high-priority interrupt executor that requires both a core-0 beat (50 ms) and the flight core's answer to the
10 Hz ping (`src/watchdog.rs:1-15`). At 200 ms stall it writes a scratch record (core, task) and stops feeding; the board resets about 1.7 s later and the next boot reports why.
Status LED (`src/led.rs`, logic `fw_core::led`): white at start, a nine-blink roll call at 6 s (IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, ESC; green OK, red FAILED, yellow else), then
the running display; brightness `LED_BRIGHT`.

## 4. Flight modes and RC links

### 4.1 Modes

The mode is the hands-off state machine's state (`crates/fw-flight/src/hands_off.rs`), sent in HEARTBEAT `custom_mode` (`State::code`, `hands_off.rs:72`; `src/station.rs:159`).

| custom_mode | Mode | Meaning |
|---|---|---|
| 1 | PILOT | Pilot flies. Sub-mode from the mode switch: Angle / Altitude / Position |
| 2 | TAKEOVER | Vehicle holds position, sticks ignored, until the pilot's hand-back conditions are met |
| 3 | HOLD | Position hold with centred synthetic sticks, sticks ignored |
| 4 | AUTO | Flying the uploaded waypoint list |
| 5 | RETURN | Climb to the return height, straight leg home, then LAND |
| 6 | LAND | Controlled descent (1.5 m/s then slower near the ground, `step.rs:102`); touchdown stops the motors |
| 7 | STOP | Disarmed, motors stopped |
| 8 | BLIND | LAND on tilt and rates only (tier T1); reported only (`hands_off.rs:71`) |

Pilot sub-modes (`fw_core::rc::Switch3`, `crates/fw-settings/src/rc.rs:313`): mode switch Low = Angle, Middle = Altitude, High = Position. Position and Altitude fall back
to a lower mode when the estimate does not cover them (`StepOut.mode`, `step.rs:54`).

### 4.2 The two RC links

| | CRSF model | MAVLink model |
|---|---|---|
| Transmitter | ELRS model sending CRSF (420000 baud) | ELRS model sending MAVLink (460800 baud); the receiver must be powered up on this model, else it sends no sticks (ExpressLRS issue 3860; handoff 9.B1) |
| Stick source | `Source::Crsf` frames | `RC_CHANNELS_OVERRIDE` only from sender (255, 68), none of ch1-8 at 0 or 65535 (`fw_core::rc::receiver_report`, `crates/fw-core/src/rc.rs:224`); any other sender (the station is 255/190) is never a stick report |
| Rate | about 245 /s | 100 /s |
| Arms | always | only with `FS_HOLD_EN` 1 (`gate.rs` `accepts`, `gate.rs:88-91`) |
| Hands-off states | never HOLD / AUTO / RETURN: stick loss gives LAND (BLIND, or STOP on the ground) | HOLD, AUTO, RETURN, TAKEOVER, mission switch (section 5) |
| Station radio commands | refused, reason 3 | accepted |

The flight keeps the link type it armed on until disarm (`gate.rs:143` latches `source`; `hands_off.rs:728-735` `ho_allowed`, `mavlink_flight`). Reports from the other
source are ignored for the rest of the flight.

### 4.3 Channel map

Defaults from `fw-settings/src/rc.rs:93-96`; calibration `RCn_MIN/TRIM/MAX/REVERSED/DZ` for channels 1-8.

| Channel | Function | Rule |
|---|---|---|
| 1 / 2 / 3 / 4 | roll / pitch / throttle / yaw | `RCMAP_ROLL/PITCH/THROTTLE/YAW` (1,2,3,4) |
| 3 | throttle | arming needs throttle at or below `arm_thr_max` (0.05) |
| 5 | arm switch | raw above 1800 us = ON; no calibration; a missing channel (0) reads OFF (`fw-settings/src/rc.rs:296-301`) |
| 6 | mode switch | thirds: Low Angle, Middle Altitude, High Position (`rc.rs:313`) |
| 8 | mission switch | raw above 1800 us = ON, same rule as the arm switch (`src/rc.rs:375`). Only read on a MAVLink flight |

Also sent to the station over USB: RC_CHANNELS (65), MANUAL_CONTROL (69, CRSF only); `src/rc.rs:254`.

## 5. Mission switch and failsafes (Amendment N19)

Authoritative design: `decision-mavlink-mode-20261123.md` section 67. Code: `hands_off.rs:739` `mission_switch` (the table), `:940` `lost`, `:701` `update_home`, `:1028` `command`.
Terms: MS = mission switch (ch8) ON; A = arm ON; P = mode switch in Position; "fresh" = newest report younger than 1.0 s (`Tuning.rc_fs_timeout_us`, `tuning.rs:221`). All rows are MAVLink flights.

### 5.1 Transition table

| State | MS ON, A | MS OFF, A, P | MS OFF, A, not P | A OFF, MS ON | A OFF, MS OFF | No fresh report |
|---|---|---|---|---|---|---|
| PILOT | HOLD if P, airborne and `hold_entry_check` passes (tier T3, no track fault, battery OK, link, arm ON); else PILOT | PILOT | PILOT | on ground STOP; else LAND | STOP | `lost()`: HOLD (T3), LAND (T2), BLIND (T1), STOP (ground) |
| TAKEOVER | stays TAKEOVER | PILOT | stays | LAND | STOP | `lost()` |
| HOLD | stays (mode switch ignored; Start allowed) | PILOT, place dropped | TAKEOVER | LAND | STOP | stays; link age over 10 s gives RETURN (home + return built) else LAND |
| AUTO | stays | PILOT | TAKEOVER | LAND | STOP | stays; station silent 5 s gives HOLD with place kept |
| RETURN | stays | PILOT | TAKEOVER | LAND | STOP | stays |
| LAND / BLIND | unchanged | PILOT | TAKEOVER | stays | STOP | unchanged |

Priority (code order, `hands_off.rs:739-830`): arm OFF columns (they read the newest report even if stale), then the ground exit (hands-off state, not airborne, touchdown held: STOP),
then the MS OFF columns, then each state's own exits (tier lost, battery critical, track fault, link age, station loss, arrival), then MS ON entry to HOLD, then station commands.

### 5.2 The rules in plain terms

| Rule | Behaviour | Code |
|---|---|---|
| Arming | Needs: arm switch cycled OFF then ON, throttle low, tilt OK, a controller output, fresh report; refused with MS ON on a receiver report; refused on a receiver report at `FS_HOLD_EN` 0 | `gate.rs:134-143` |
| Mission switch ON in the air | Armed + Position + airborne + MS ON: PILOT goes to HOLD (sticks ignored). Start on the laptop then gives AUTO | `hands_off.rs:820-825` |
| Take control back | MS OFF with arm ON and Position: PILOT at once (also after a link gap). Mode switch not in Position: TAKEOVER (holds, sticks ignored) until it is | `hands_off.rs:760-770` |
| Arm OFF, MS ON | Controlled LAND (BLIND on T1); stops on the ground. Arm ON again changes nothing; MS OFF + arm ON + P returns control | `hands_off.rs:740-756` |
| Arm OFF, MS OFF | Motors stop at once, in the air too (owner's rule: arm OFF is a kill unless MS is ON) | `hands_off.rs:743-746` |
| Stick-link loss, 1 s | No fresh report for over 1.0 s: HOLD if position tier T3, LAND on T2, BLIND on T1, STOP on the ground or if never airborne | `hands_off.rs:940-955` |
| Stick-link loss, 10 s | In HOLD, link age over 10 s (`LINK_LOSS_US`, `hands_off.rs:12`; newer of stick report and station heartbeat): RETURN if home latched and RETURN built, else LAND | `hands_off.rs:784-791` |
| Reports return | The switch positions decide: MS OFF + arm ON + P gives PILOT; MS ON carries on whatever state the gap left (AUTO / HOLD / RETURN) | table above |
| Laptop heartbeat lost | In AUTO, no station HEARTBEAT (sender 255/190 on the radio) for over 5 s (`STATION_LOST_US`, `hands_off.rs:16`): HOLD, mission paused, place kept; press Start again. Live sticks mean no RETURN | `hands_off.rs:800-801`, `:969` |
| Home latch | Latched only between arming and first take-off, at the first valid position (height and NE each, both or nothing); cleared at disarm. Taking off within a few seconds of arming = no home: link loss then lands in place and Start is refused "no home point". Wait 15 s on the ground after arming (15 s at idle was enough on the bench; the minimum is not measured). The station cannot show "home set" | `hands_off.rs:701-724` |
| Hand-back into Position | Moves at up to `PILOT_SPEED_XY` horizontally and 3.0 / 1.5 m/s vertically if sticks are off centre | decision section 67.6 hazard 7 |
| Other exits (all MAVLink-flight states) | Tier debounce 1.0 s (`TIER_DEBOUNCE_US`); battery critical or a track fault in HOLD/AUTO/RETURN gives LAND; arrival at home in RETURN gives LAND; distance rule: D_max 10 m | `hands_off.rs:25-29`, `:772-815` |
| CRSF flights | No hands-off path: stick loss gives LAND on T3/T2, BLIND on T1, STOP on the ground; when reports return, TAKEOVER and the old lock (arm ON, mode switch out of Position, to give control back); arm OFF stops at once unless a RADIO_STATUS was seen | `hands_off.rs:834-917` |

Never in the air: HOLD, AUTO, RETURN, LAND's touchdown stop, TAKEOVER and the mission switch (section 9).

## 6. Waypoint missions end to end

### 6.1 Upload from marv-gcs

Station Map tab: "Waypoints" button opens the panel; click the map to add points (amber), set altitude, set "Return height (m)", then "Upload to vehicle" (confirm). The station
sends the MAVLink mission protocol (link contract section 17: MISSION_COUNT, REQUEST_INT, ITEM_INT, ACK, REQUEST_LIST, CLEAR_ALL; plan type 0, frame 3 relative altitude) over USB or the
radio (UDP 14550 through the transmitter's backpack), reads the list back and shows "Upload verified". Same from the command line: `marv-cli mission upload|download|clear FILE.waypoints`.
Vehicle side: `src/mission.rs` hands both links' frames to `fw_core::mission::MissionService` (`crates/fw-core/src/mission.rs:524`), one transaction at a time, retries 1500 ms x 5
(`mission.rs:53-55`), replies 40 ms apart on the radio. The list lives in RAM only: lost at power cycle. Uploads are accepted while armed (`src/mission.rs:115`, `let armed = false`).

### 6.2 Limits

| Limit | Value | Code |
|---|---|---|
| Waypoints | up to 16, plus a mandatory last "return item" (MAV_CMD_NAV_RETURN_TO_LAUNCH, x = y = 0) whose z is the return height; 17 items max | `fw-core/src/mission.rs:31-43`, `:326-348` |
| Item rules | waypoints are command 16, frame 3 (or 6); lat within +-90, lon within +-180 (1e-7 deg); z must be finite and within `GDN_ALT_MIN`..`GDN_ALT_MAX` (defaults 5..50 m, parameter range 3..120 m), checked at upload; refused with MAV_MISSION codes (INVALID_PARAM7 for height) | `mission.rs:326-348`, `fw-settings/src/flight.rs:98-99` |
| Distance | vehicle and every waypoint within 1 km of home (`HOME_RANGE_M`) at Start | `hands_off.rs:20`, `:1138-1160` |
| Return height | the return item's z, capped at `GDN_ALT_MAX`, never lower than the present height above home | `hands_off.rs:1008-1018` |
| Arrival radius | `GDN_RADIUS` (default 2.0 m, range 0.05..10) | `fw-settings/src/flight.rs:74` |
| Cruise speed | min(`PILOT_SPEED_XY`, 6.8 m/s); climb 3.0, descent 1.5 m/s; carrot acceleration 3 m/s^2 | `guidance.rs:266-275`, `tuning.rs:284` |
| Guidance source | only `GDN_SRC` 1 uses the uploaded list; 0 uses the `GDN_COUNT`/`GDN_LAT1..4`/`GDN_LON1..4` parameters | `fw-settings/src/flight.rs:29-32` |

### 6.3 Start preconditions and refusal codes

Start is a station command (MAV_CMD_MISSION_START) from state HOLD (so the pilot must first be in HOLD: armed, Position, airborne, MS ON). It must arrive on the radio link, and the first
failing check below answers COMMAND_ACK DENIED with the code in `result_param2` (`DenyReason::code`, `hands_off.rs:296`; order of `start_check`, `hands_off.rs:1125`):

| Order | Check | Code |
|---|---|---|
| a | command arrived on the radio (USB commands refused) | 1 |
| b | flight armed on CRSF / `FS_HOLD_EN` 0 | 3 |
| c | state is not HOLD (or AUTO already) | 4 |
| 1 | station heartbeat heard within 5 s | 5 |
| 2 | a list with a waypoint stored and `GDN_SRC` is 1 | 6 (none stored), 20 (stored but `GDN_SRC` not 1) |
| 3 | every z within `GDN_ALT_MIN`..`GDN_ALT_MAX` | 7 |
| 4 | home latched | 8 |
| 5 | vehicle position valid and within 1 km of home | 9 |
| 6 | waypoint distance known / all within 1 km | 10 / 11 |
| 7 | heading monitor OK and no track fault | 12 |
| 8 | position tier T3 | 13 |
| 9 | battery known non-critical | 14 |
| 10 | live receiver report younger than 1.0 s with MS ON / arm ON / mode switch in Position | 19 / 15 / 16 |

Other codes: 17 battery critical and 18 link lost (on entering HOLD or RETURN: Pause, Abort, Return). Codes 0 = unknown. The station shows one result line per command
("sent, waiting", "ACCEPTED by the vehicle at HH:MM:SS", "REFUSED at ...: reason", "NO ANSWER", "NOT SENT").

### 6.4 Guidance behaviour

`crates/fw-flight/src/guidance.rs` (`LegGuidance`, wired in `step.rs`): a "carrot" moves along each straight leg at the cruise speed, braking to stop at the waypoint; the controller
tracks the carrot's position with velocity feed-forward (`Setpoint::Position`). The carrot stops advancing when the vehicle lags more than 2 m (`LAG_E_MAX_M`). Expect corner
cutting of up to 2 m and a height sag while moving (the sag fix is pending). On reaching waypoint k (radius `GDN_RADIUS`) the next leg starts at it; at the last waypoint the vehicle goes to HOLD at its height
(`i.last_waypoint_reached`, `hands_off.rs:798`). Station commands once flying: Pause (AUTO to HOLD, place kept), Start again (continue from the kept place), Abort (to HOLD, place dropped),
Land, Return (`hands_off.rs:1050-1092`). Uploading a new list while holding drops the kept place. RETURN: climb in place to the return height, straight leg home at that height, within the radius LAND.

## 7. Key parameters

Defaults and ranges from `crates/fw-settings/src`. Values saved on the vehicle at the handoff: none of the first three (see section 9).

| Parameter | Default (range) | What it does |
|---|---|---|
| `FS_HOLD_EN` | 0 (0..1) | 1 lets the MAVLink model arm and fly (hands-off states). Latched while disarmed. Index 122 (`fw-settings/src/flight.rs:116`) |
| `GDN_SRC` | 0 (0..1) | 1 = guidance and Start use the uploaded mission; 0 = the `GDN_*` parameters. Start refused with code 20 if a list is stored and this is 0 |
| `GDN_ALT_MIN` / `GDN_ALT_MAX` | 5 / 50 m (3..120) | Height limits for uploaded waypoints and the return item, checked at upload and at Start |
| `GDN_RADIUS` | 2.0 m (0.05..10) | Waypoint arrival radius |
| `GDN_COUNT`, `GDN_LAT1-4`, `GDN_LON1-4` | 0 / 0 | Up to 4 targets set by parameters (used when `GDN_SRC` 0) |
| `PILOT_SPEED_XY` | 5.0 m/s (3..20) | Horizontal speed at full stick; also caps hands-off and mission cruise speed. Taken at arming |
| `MAG_HLT_STR` / `MAG_HLT_INC` | 10 % (1..50) / 10 deg (1..45) | Compass health test tolerance; loosening it (bench used 40) risks a bad heading. Never save 40 |
| `BATT_CELLS` | 4 (0..12, 0 = monitor off) | Cells in series |
| `BATT_LOW_CELL` / `BATT_CRT_CELL` | 3.6 / 3.5 V per cell | LOW and CRITICAL thresholds |
| `BATT_LOW_TIMER` | 10 s | Voltage must stay below a level this long |
| `BATT_FS_CRT_ACT` | 0 (0 warn, 1 descend) | What CRITICAL does |
| `BATT_V_SCALE`, `BATT_A_SCALE`, `BATT_A_OFFSET` | 1.0, 0.02, 0 | ADC calibration |
| `ATC_ANG_RP_P`, `ATC_ANG_YAW_P`, `ATC_RAT_{RLL,PIT,YAW}_{P,I,D,IMAX}`, `ATC_RAT_FLTD` (14 total) | compiled `Tuning::DEFAULT` | Attitude and rate controller gains, indices 105..118 (`fw-settings/src/flight.rs:274-289`, test table) |
| `RPM_NOTCH_EN` | 0 | Gyro RPM notch on/off |
| `RC1..8_MIN/TRIM/MAX/REVERSED/DZ`, `RCMAP_*` | per channel | Stick calibration and mapping |
| `IMUn_ACC_FS`, `IMUn_GYR_FS`, `IMUn_ODR_HZ` | 16 g, 2000 dps, 800 Hz | IMU config, applied once at boot |
| `GPS_RATE_MS` | 200 ms | GNSS fix interval |

## 8. Ground station, CLI, simulator

**marv-gcs** (`~/Documents/MARV-GCS`, egui): started by typing `marv-gcs` (launcher rebuilds and runs the release binary). Tabs for HUD, Sensors, Devices, Parameters, Map (position, track, waypoint editor and
mission upload, mission command buttons Start/Pause/Abort/Land/Return, mode text, take-over panel), Log (download, review), calibrations. Links: serial (USB), TCP, UDP 0.0.0.0:14550 through the
transmitter's WiFi backpack (role "Operate (read only)": no parameter writes, but mission upload and the five flight commands work). Records every session to `~/Documents/MARV-GCS/logs/*.tlog`.
Eight crates (`docs/architecture.md`): `gcs-transport`, `gcs-mavlink`, `gcs-link`, `gcs-dflog`, `gcs-magcal`, `gcs-sim` (simulated vehicle), `marv-gcs`, `marv-cli`. Wire rules between FC and station:
`MARV-GCS/docs/link-contract.md` (1811 lines; section 10 commands and mode codes, section 17 missions).

**marv-cli** (no display, one command per run, TAB-separated output): `status`, `devices`, `texts`, `watch`, `param list|get|set|save|reload|reset-all`, `calibrate gyro|accel|compass`,
`device restart|selftest`, `reboot --yes`, `logs list|get`, `mission upload|download|clear`, `record FILE --for s`, `session export|list`, `log report|tune|survey` (offline). Connect with
`--connect serial:/dev/serial/by-id/usb-MARV_MARV_V2_firmware_marv-v2-fw-if00` (or `udp:`, `tcp:`).

Watching a test (handoff section 11): `marv-cli --connect serial:... record x.tlog --for <s>` in the background (it holds the port), then
`marv-cli session export x.tlog --name x --out <dir>` and read the CSVs. One CSV per MAVLink message type; first column `t_s`, then the message's own fields:

| CSV | Useful fields |
|---|---|
| `HEARTBEAT.csv` | `custom_mode` (1 PILOT, 2 TAKEOVER, 3 HOLD, 4 AUTO, 5 RETURN, 6 LAND, 7 STOP, 8 BLIND), `base_mode`, `system_status` |
| `RC_CHANNELS.csv` | `chan3_raw` throttle, `chan5_raw` arm, `chan6_raw` mode, `chan8_raw` mission switch (the USB stream carries all 16 channels; `src/rc.rs:254`) |
| `ESC_STATUS.csv` | motor eRPM |
| others | `SYS_STATUS`, `HIGHRES_IMU`, `GPS_RAW_INT` (no lat/lon: privacy), `MARV_DEVICE_STATUS`, `STATUSTEXT` |

**Simulator** (`~/Documents/MARV-sim`, workspace `marv-card` + `marv-sim`): runs the firmware's real flight code (`fw-flight`, `fw-core` as path dependencies, so the sim sees the same flight step and mission code) on a
rigid-body plant at a 125 us tick (IMU models at 800 Hz, ESC command at 1 kHz, baro 50 Hz, compass 100 Hz, GNSS 5 Hz) with noise, wind and a scripted pilot; `marv-run` judges runs
against thresholds and writes DataFlash logs. The N19 work was run on firmware branch `mission-switch-sim` (e5aed63 + hooks; d430a96's refusal 20 is not in it).

## 9. Status: verified and not flown

Verified (handoff sections 6-7):

| Layer | Evidence |
|---|---|
| Host tests at d430a96 | fw-flight 523 + 2, fw-core 458 + 5, fw-settings 85; hand-flying reference outputs (`SCRIPT_GOLDEN`, `WALK_GOLDEN`) unchanged; six deliberate faults caught |
| Review | Three review passes on the N19 change and one on d430a96: PASS for a push and a props-off bench flash |
| Build guards | `check-ram-code` PASS; start-up report 16 PASS, 0 FAIL; 128 parameters (120 flown ones identical to the snapshot) |
| Simulator | N19 flight end to end in 126 of 126 runs over thrust gain 0.62-1.46; arm-OFF landings 126 of 126 with 0 motor stops above 0.3 m; hand-back brakes to under 0.3 m/s in about 1 s; transmitter-off 3 / 9.9 / 10.1 s as ruled; CRSF regression 540 cells identical |
| Bench (props off, with the owner) | Every cell of the switch table, HOLD 30 s, Start accepted and AUTO 30 s (twice), RETURN 15 s, LAND and touchdown stop, TAKEOVER lock and release, both arm-OFF rules, arming refusals, upload over the backpack verified, refusal 20 and ACCEPTED line seen |

Never flown (the handoff's own words, section 7): "HOLD, AUTO, RETURN, LAND's touchdown stop, TAKEOVER and the mission switch have never been in the air. Hand flying on the MAVLink model
(100 stick reports/s instead of about 245; stop delay after arm OFF 8 ms instead of 2 ms in the simulator) has never been flown. The real link's delay is not measured." Also never run on
the board: BLIND, a waypoint arrival or leg change, the battery-critical exits. The vehicle is on d430a96, not the flown firmware (`mission-upload` 72de0d4 in `~/Documents/MARV-V2-firmware`; restore with `tools/run.sh 120` there).

Open owner decisions (handoff section 8): save `FS_HOLD_EN` 1, `GDN_SRC` 1, `PILOT_SPEED_XY` 3.0 for the flights (not yet; never `MAG_HLT_STR` 40); raise the flight-core stack; remove outdated
firmware folders. Flight plan (handoff section 9): pre-flight on the MAVLink model, compass check `valid 0x7f`, upload, wait 15 s after arming; flight 1 hand flight with MS OFF; flight 2 the mission.

## 10. Where to look next

| Need | Place |
|---|---|
| Per-subsystem detail and bench evidence | `README.md` headings: Flight-core device loop (54), Device layer (183), Parameters (457), SD card (616), Logging (678), Settings in flash (844), Calibrations (929), Commands (1057), Radio (1079), Status LED (1169), Watchdog (1285), ESC telemetry (1430), Flight core in SRAM (1578), Flash layout (1642) |
| Pins, DMA, PIO, IRQ ownership | `docs/resources.md`, `docs/pinout.md`, `docs/devices.md` |
| ESC link history | `docs/handoff-esc-link.md`; project state at 2026-11-19: `docs/handoff.md` (dated) |
| Why the mission-switch behaviour is what it is | `~/Documents/MARV-V2-drivers/_plan/decision-mavlink-mode-20261123.md` sections 65-75 (67 = N19 table); latest handoff `_plan/handoff-20261124-evening.md` |
| Wire contract (commands, codes, missions) | `~/Documents/MARV-GCS/docs/link-contract.md` sections 10, 17; `docs/architecture.md`, `README.md` there |
| Driver contracts | `~/Documents/MARV-V2-drivers/CONTRACT.md`, each driver's README |
| Simulator | `~/Documents/MARV-sim/README.md`, `scenarios/` |

Known stale statements in other docs (this file follows the code): `README.md` Radio section says a MAVLink report is never written to the
flight core (`src/rc.rs:357-379` now writes it); `src/mission.rs:5` says "nothing here flies a list"; `docs/handoff.md` is dated 2026-11-19;
`MARV-GCS/README.md` Map section says the vehicle only stores the list and `link-contract.md:1264-1273` says Amendment A is "approved, not built".
