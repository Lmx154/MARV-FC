# Handoff: USB link, MAVLink layer and ground station finished; wiring them into the firmware is next

Written 2026-11-12 at the end of the ground-station work, for whoever continues the firmware (a new
session or a person). It adds to `docs/handoff.md` (the drivers, the board, the rules for the sensors and
the SD card), which still holds. It says what exists, where it is on GitHub, how this repo takes it, what
was proven on the real board, what the firmware has to respect, and what is still open.

## 1. Where everything is

| What | GitHub repository | Commit on `main` |
|---|---|---|
| Ground station (runs on the computer) | <https://github.com/Lmx154/MARV-GCS> | `3abc685` |
| `usb-cdc`: bytes over the board's USB port | <https://github.com/Lmx154/usb-cdc> | `0002fe9` |
| `marv-dialect`: the MAVLink message definitions, buildable for the board | <https://github.com/Lmx154/marv-dialect> | `61915e6` |
| `mavlink-device`: the MAVLink layer of the board, on any byte stream | <https://github.com/Lmx154/mavlink-device> | `a5e4f4f` |

All four are public, MIT, version 0.1.0 and not on crates.io. They refer to each other by GitHub URL
only: `mavlink-device` takes `marv-dialect`, and its on-board demonstration takes `usb-cdc`, as git
dependencies pinned to a commit, the way this repo takes the sensor drivers. No repo needs another one
checked out next to it to build.

On the owner's computer the checkouts are `~/Documents/MARV-GCS` and
`~/Documents/MARV-V2-drivers/{usb-cdc,marv-dialect,mavlink-device}`; in each, the local work branch
(`mvp`, `first-version`) is the same commit as `main`. Not in git: `docs/plan.md` of the MARV-GCS
checkout, the working plan and log of this work with the owner's instructions quoted.

The owner's request (2026-11-12):

> I want to create a ground control software for my future flight controller. It should work like common
> ground control software (mission planner, qground control) -logging -parameters -live sensor inputs. I
> want to use rust for the gui and backend. I want to use mavlink as the protocol. The software starts
> right now, so we can add parameters and settings as the firmware grows so that we don't introduce
> unnecessary bloat. we need an MVP not a fully featured ground control software from day 1. I believe we
> may need to create a driver for USB communication between the flight controller and the software's
> computer, if so, please do this in a separate repo/ crate just like how we have created the other
> drivers. just make sure it's expandable so we can add parameters and settings and other features later
> on without too much friction

and the rule every crate follows (2026-11-12):

> do not combine usb with mavlink. usb lives alone mavlink is a layer on top of it that utilizes usb

```
computer                                    flight controller
--------                                    -----------------
marv-gcs (window)                           firmware (owns identity, tasks, parameters, sensors)
gcs-mavlink   <---- MAVLink 2 frames ---->  mavlink-device   (any embedded-io-async Read / Write)
gcs-transport <---- bytes ---------------->  usb-cdc          (bytes only; never names MAVLink)
serial port of the operating system         embassy-usb CDC-ACM
```

## 2. What exists

- **The ground station** (`MARV-GCS`). A desktop program in Rust (egui). Tabs: Sensors (live plots),
  Messages (every message that arrives, any numeric field can be plotted), Parameters (list, search,
  edit, write with confirmation, save and load a file), Log (the vehicle's text messages; every session
  is recorded to a `.tlog` file that Mission Planner, QGroundControl and pymavlink read; a log can be
  replayed), Link (state and counters). It connects to a USB serial port or a TCP address. It knows no
  particular parameter and no particular message: what the firmware reports is what it shows. It has a
  simulated vehicle (`gcs-sim`) for work without a board. 388 host tests.
- **`usb-cdc`**. A thin layer on the CDC-ACM class of `embassy-usb` 0.6: the firmware's
  `embassy_usb::Builder` in, `Tx` (`embedded_io_async::Write`), `Rx` (`embedded_io_async::Read`) and
  `Link` (up or down, DTR, counters) out. It was not written from scratch; it exists because the bare
  class has three traps on embassy-rp 0.10 (a transfer ending on a full packet can be held back by the
  computer, an empty packet reads as end of file, and a write in flight does not end when the link goes
  down). `docs/device.md` of that repo and section 3 of the contract list them with file and line.
- **`marv-dialect`**. The MAVLink `common` message set with extension fields, generated at build time,
  `no_std`. It exists because the stock `mavlink` 0.21.0 crate does not build without `std` when
  extension fields are on, and without them `HIGHRES_IMU` has no `id` field.
- **`mavlink-device`**. `Node`: framing, the heartbeat, the parameter protocol, two commands (set a
  message rate, request a message), a scheduler for the telemetry streams and a queue of text messages.
  It does no I/O by itself (`receive`, `poll_transmit`, `next_deadline_us`), and `Node::run` drives it on
  any `embedded_io_async` reader and writer. The firmware passes in the parameters (`ParamStore`) and
  the values to send (`Telemetry`).
- **The contract** both ends were built to:
  [`docs/link-contract.md` of MARV-GCS](https://github.com/Lmx154/MARV-GCS/blob/main/docs/link-contract.md).
  Section 2 is what goes over the wire, section 5 the invariants. Read it before writing the link task.

## 3. How this repo takes the crates

Nothing was added to this repo: no code here uses the crates yet. The dependency lines, pinned like the
sensor drivers in `Cargo.toml`:

```toml
usb-cdc = { git = "https://github.com/Lmx154/usb-cdc.git", rev = "0002fe99a8d2dfed44917d44e26dbda6ee92427e", features = ["defmt"] }
mavlink-device = { git = "https://github.com/Lmx154/mavlink-device.git", rev = "a5e4f4fe69599fe1d6d72e947df5f786f56d4228", features = ["defmt"] }

embassy-usb = { version = "0.6.0", default-features = false, features = ["defmt"] }
embassy-sync = "0.8.0"
embassy-futures = "0.1.2"
embedded-io-async = "0.7"
static_cell = "2.1.1"
```

- Checked on 2026-11-12: a copy of this repo (commit `4aae70d`) with exactly these lines added, and the
  demonstration program of section 4 as its `main`, built for the board with every one of the crates
  fetched from GitHub. `cargo tree` showed the three crates coming from their GitHub URLs at the commits of
  section 1 and no path outside the copy. That build was not flashed.
- Do not add `marv-dialect` here. The message types are re-exported as `mavlink_device::dialect`. A
  second copy named directly, at another commit, would be a different crate to the compiler and its types
  would not fit `mavlink-device`'s.
- `Cargo.lock` gains the three git sources (`marv-dialect` comes in through `mavlink-device`); commit it.
- To take a newer version: change the `rev` and run `cargo update -p <crate>`, as for the drivers.
- To work on one of them and the firmware together, add a `[patch."https://github.com/Lmx154/<repo>.git"]`
  entry with a local path instead of editing the dependency (the comment in `Cargo.toml` shows the form).
- `embassy-sync` is not needed by the demonstration; it is in the list because the cells that carry sensor
  values to the link task (section 5) will need it, and 0.8.0 is the version already in `usb-cdc`'s tree.

## 4. Wiring it in: the shape that ran on the board

The reference is the demonstration program,
[`hil/src/bin/mavlink_demo.rs` of mavlink-device](https://github.com/Lmx154/mavlink-device/blob/a5e4f4fe69599fe1d6d72e947df5f786f56d4228/hil/src/bin/mavlink_demo.rs)
(463 lines; the USB set-up, the start gate and the link loop are about 140 of them). It ran on this board
against the ground station. Its
values are computed from the clock and its 13 parameters live in RAM; those two parts are what the
firmware replaces.

1. **The USB device.** `embassy_rp::usb::Driver::new(p.USB, Irqs)` with `USBCTRL_IRQ` bound to
   `embassy_rp::usb::InterruptHandler<USB>`; an `embassy_usb::Config::new(vid, pid)` with the firmware's
   own identity; an `embassy_usb::Builder`; then `usb_cdc::UsbCdc::new(&mut builder, state, usb_cdc::Config::new())`
   before `builder.build()`, and `split()` gives `(tx, rx, link)`. One task runs `device.run()` for ever
   and is never cancelled.
2. **The node.** A table of streams the firmware owns, and `Node::new(Config::new(1, 1), &mut slots)`
   (system 1, component 1). The streams of the contract, as `(message id, default interval, shortest
   interval)` in microseconds: HIGHRES_IMU `(105, 20_000, 5_000)`, SCALED_PRESSURE `(29, 100_000, 20_000)`,
   GPS_RAW_INT `(24, 200_000, 100_000)`.
3. **The link task**, one loop: `link.wait_up().await`, the start gate, `node.run(&mut rx, &mut tx, &mut
   delay, now_us, &mut params, &mut telemetry).await`, `node.on_link_down()`. `run` returns only when the
   link is gone.
4. **The start gate** (`start_gate`, lines 316 to 357 of the demonstration; copy it). After each link-up
   the task writes nothing until DTR has read true for 100 ms (checked at most every 5 ms) or a
   ground-station HEARTBEAT has arrived; meanwhile it reads and passes the bytes to `Node::receive`.
   Without it the first bytes after every plug-in are damaged on the computer: Linux hands a new serial
   port over in terminal mode for about a millisecond of the first open (contract, section 3).
5. **`Telemetry`**: `sample(message_id, now_us, out)` is called once per period of a stream and pushes
   up to 4 frames (the four HIGHRES_IMU instances: `id` 0 main IMU, 1 second IMU, 2 high-g accelerometer,
   3 magnetometer; a field the sensor does not measure is NaN with its `fields_updated` bit clear).
   `status(now_us)` gives the state for the heartbeat (BOOT, then STANDBY); `texts(now_us, out)` offers
   text messages. All three are called from the link task and must not block.
6. **`ParamStore`**: `count`, `name`, `get`, `find`, `set`. `set` returns the value now in effect (so it
   may clamp) or an error, which the ground station shows as a refused write. Storage, validation and
   persistence are the firmware's.

## 5. What the firmware has to respect

From the contract, the two READMEs and the runs on the board.

Tasks
- **Only the link task awaits a `usb-cdc` or `mavlink-device` future** (contract, invariant 3). Sensor
  and control tasks never call into the node. They leave their latest value in a cell that never blocks,
  and `Telemetry::sample` reads it when the stream is due. Values are sampled, never queued: a late link
  task skips periods, it does not send a burst.
- **A write waits while the computer is not reading**, and the node has no timeout for it. Nothing else
  may wait behind the link task. Measured with `usb-cdc`'s own test program (no MAVLink in it; a writer
  with a 100 ms timeout per write) while the computer held the port open and did not read for 10 s: a
  1 kHz task beside the writer had 0 periods over 1.5 ms. The same has not been measured with the node.
- **`Tx`, `Rx` and `Link` are not `Send`**: the link task and the task that runs the USB device stay on
  one core. Different executors on that core are fine. Values from the other core come through the cells.
- While `run` is awaited the node is borrowed. Status and text messages go through the `Telemetry` hooks,
  not through `set_status` and `queue_text`.
- After `run` returns the firmware calls `on_link_down()` (message rates return to their defaults) and
  waits for the link again.
- Logging: the demonstration logs nothing after its start-up lines, for the reason in `docs/handoff.md`
  section 5 (a `defmt` line masks interrupts for about 80 us).

USB
- Nothing is written between link-up and the start gate (contract, invariant 8).
- Leave `device_class`, `device_sub_class` and `device_protocol` at the defaults of
  `embassy_usb::Config::new`, and keep one handler slot free for `usb-cdc` (README, "What the application
  provides").
- **With no battery attached this board is powered through its USB cable.** Pulling the cable then
  switches the board off; it is not a link going down with the board running, and a debug session ends
  with it. With a battery attached the board keeps running and the pull is a bus suspend: the link goes
  down 3.2 ms after the last packet and a pending write ends with `NotConnected` (one pull, 2026-11-12;
  `usb-cdc` README, check F).
- The USB identity of the demonstration (vendor 0xc0de, product 0xcafe, "MARV V2 link demo") is a bench
  value (section 9).

Protocol
- Vehicle: system 1, component 1. Heartbeat at 1 Hz, not changeable. MAVLink 2 only, no signing.
- Parameters are INT32 or REAL32 only (a bool or an enum is an INT32). Names: at most 16 characters of
  `A-Z`, `0-9`, `_`, unique. The set, its order and its count are fixed for one boot. A write is always
  answered with the value now in effect.
- The device layer never arms, never changes a mode and does nothing when the ground station is lost. Any
  such behaviour is the firmware's to add, deliberately.
- Load on the link: about 15.6 kB/s at the default rates and 62 kB/s at the fastest ones the contract
  allows; the measured USB throughput is 757 kB/s from the board to the computer.
- A message outside the `common` set means a `marv.xml` in `marv-dialect` and a one-line change in the
  ground station (contract, end of section 2). A new parameter or a new standard message needs no change
  in the ground station.

Cost on the board (the demonstration, release build): `Node` 1952 bytes, the future of `run` 536 bytes.
The whole program (USB stack, embassy, the MAVLink layer and the demonstration's own code): `.text`
61.3 kB, `.rodata` 6.9 kB, `.bss` 5.5 kB. The dialect's `MavMessage` enum is not linked (0 symbols).

## 6. Proven on the board (MARV V2 Rev 1, one Linux computer, 2026-11-12)

The link, with the demonstration program and the ground station's board check (`tools/board_check.sh` of
MARV-GCS; three phases, the board restarted before each):

| Check | Result |
|---|---|
| First 20 s after a restart | 4343 frames, 0 checksum errors, 0 discarded bytes, 0 lost frames |
| Rates | HIGHRES_IMU 50.000 to 50.003 Hz for each of 4 instances (over all runs), SCALED_PRESSURE 10.000 Hz, GPS_RAW_INT 5.000 Hz, HEARTBEAT 1.000 Hz |
| Parameters | 13 listed in 20 ms; a write confirmed in 20 ms; a read-only one refused; an out-of-range one answered with the clamped value |
| Recording | about 1100 frames, every one decoded by pymavlink, 0 bad |
| Reconnect after a close | connected in 20 ms |
| A program that only listens | first byte about 105 ms after the open (the gate opened by DTR), frames consecutive from number 0 |
| A program that keeps DTR low | 0 bytes in the first second; first frame 0.6 ms after its HEARTBEAT |
| The gate switched off (control) | all three phases fail |

The last run of that check was made on the tree that was then committed as `11e2141` of mavlink-device
and `3abc685` of MARV-GCS, with the demonstration taking `usb-cdc` and `marv-dialect` from GitHub (the
commit of mavlink-device in section 1 is one later and changes only its README). It passed all three
phases: 4343 frames in the first 20 s with 0 errors, and a recording of 1103 frames, all decoded by
pymavlink.

`usb-cdc` alone (its own programs, README section "Measured on the MARV V2 Rev 1 board"): link up about
345 ms after the pull-up; echo of 1 MiB in awkward sizes with 0 errors; 757 kB/s board to computer,
639 kB/s computer to board; 20 close and reopen cycles clean; a write pending when the link goes down
(soft detach) ends with `NotConnected` in 161 to 234 us, 70 of 70; interrupts masked for at most 23.3 us
with the link flapping.

On the host: MARV-GCS 388 tests, `mavlink-device` 86 and 1 doc test, `usb-cdc` 34, and in `marv-dialect`
a test that all 234 messages have the same wire form as the std `mavlink` crate.

## 7. How to check the link

- Without a board, in a MARV-GCS checkout: `cargo run -p gcs-sim`, then
  `cargo run -p marv-gcs -- --connect tcp:127.0.0.1:5760`.
- With the board and the demonstration, in a `mavlink-device` checkout: `tools/run.sh mavlink_demo 300`
  (it ends after 300 s), then in MARV-GCS `cargo run -p marv-gcs -- --connect serial:/dev/ttyACM1` (on
  this computer `ttyACM0` is the debug probe). The Link tab must show 0 discarded bytes and 0 checksum
  errors.
- The whole check: `tools/board_check.sh <output directory>` in MARV-GCS, about 3 minutes. It needs a
  `mavlink-device` checkout (its path in `MAVLINK_DEVICE_REPO`), the debug probe and the board's USB
  cable. It flashes through that repo's `tools/run.sh`, which takes the same probe lock as this repo's.
- **The board tests look for the demonstration**, not for this firmware: its USB identity, its 13
  parameters and its text `MARV link demo ready` (`crates/marv-gcs/tests/board.rs`). They will not pass
  against the firmware as they are. Checking the firmware's own link the same way is work to plan.

## 8. Not done, and not verified

- **Real sensor data.** Every value sent so far was computed on the board. No driver of this repo has fed
  the link, so load, timing and jitter with the sensors running are not measured.
- **Parameters that survive a reboot.** The demonstration keeps them in RAM. Where the firmware stores
  them is not decided.
- A link taken away by the computer (needs root there). A cable pulled while the board keeps running was
  measured once on 2026-11-12 with a battery attached (section 5, USB); repeated pulls were not run.
- Windows and macOS. Sessions longer than about a minute. A USB-serial adapter or a radio in between.
- The fastest rates of the contract on the board (only the default rates ran).
- Not built in the ground station: missions, a map, joystick input, firmware upload, download of logs
  stored on the vehicle, changing message rates from the window.
- Not in the device layer, by design, each with its trigger in the contract (end of section 2): battery
  and system status, attitude, time synchronisation, parameter descriptions and ranges, signing, more
  parameter types.

## 9. The owner's to choose (none was decided)

- The USB vendor and product id before a board leaves the bench (contract, section 7, names the two
  usual routes).
- The vehicle type in the heartbeat (GENERIC today).
- Whether Mission Planner or QGroundControl must also be able to connect (default: no).

## 10. What comes next

`docs/handoff.md` section 8 lists, under "Not decided yet", what the ground control software and the
parameter system have to do first and over which link. That is now answered: the link is the board's USB
port carrying MAVLink 2, the ground station exists, and parameters travel by the MAVLink parameter
protocol with the firmware owning the storage. Still open from that list: what phase one records and at
which rates, the log format on the SD card, and how logs stored on the board are taken off it (the
ground station records what arrives over the link on the computer; it does not fetch files from the
board).

A first step that needs no decision: add the two dependency lines of section 3, move the USB set-up, the
start gate and the link loop of the demonstration into this repo as the link task, keep its computed
values, flash, and see the ground station connect. Then replace one computed value at a time with a real
sensor's latest value, starting with the barometer (one message, 10 Hz).
