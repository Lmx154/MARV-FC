# Handoff: ESC link (DShot, AM32 passthrough, telemetry input) and battery sensing

Written 2026-11-12, about 18:05, at a forced stop (usage limit). The controlling plan is `docs/plan.md`
in this repo (local, not in git): its log has every step with evidence. This file is what a fresh session
needs first.

## What the owner asked for (verbatim, 2026-11-12)

"we can finish off this section with the battery plugged in test because i know it was missing, then we
can move on to the next part: I was thinking we could start on drivers for ESC communication with AM32,
as well as other battery functionality like current detection, and voltage detection since we have pins
on the board dedicated to that."

Later the same day: "if you can make our board work as a passthrough so we can access the esc through
the configurator then that would be perfect. i don't want to have to manage AM32 through our firmware";
"one port is fine. i will never need to use the ground station when i'm configuring the ESC"; on the USB
id: "this is a repo for development not for production. none of this is for production."; on READMEs:
"not just that one update them all, then commit and push. don't let readmes get stale".

## State of each piece

All driver crates live under `~/Documents/MARV-V2-drivers/`, work branch `first-version`, accepted work
on `main`, pushed to `git@github.com:Lmx154/<name>.git`.

| Piece | State | Commit on `main` (pushed) |
|---|---|---|
| Cable pull on battery (usb-cdc check F) | DONE | usb-cdc 15c1611 |
| Battery voltage (`adc-battery`) | DONE: within 0.08 % of a meter at a second voltage with a one-point trim of +7603 ppm; README current | adc-battery c9cf726 |
| Battery current (`adc-battery`) | Driver done; the scale is UNKNOWN, needs a real load (motors) | same |
| AM32 passthrough (`esc-passthrough`, remote `esc-passthrough`) | DONE on the standalone demo: the real configurator read all four ESCs, changed a setting, flashed one ESC; verified byte for byte. README current | esc-passthrough d32c25c |
| One-wire serial + receive-only UART (`pio-uart`) | DONE: `OneWire` and, new today, `UartRx` (reviewed PASS, loop-back on GPIO34). No real ESC telemetry received yet | pio-uart 074527f |
| DShot pure core (`dshot`, remote `dshot-rs`) | DONE and pushed | dshot 43c212d |
| DShot back-end `PioDshot` | **IN PROGRESS, NOT ACCEPTED**: local branch `wip-pio-dshot`, commit b484b5a, not pushed, not reviewed, README not updated | none |
| ESC serial telemetry (KISS frames on GPIO34) | NOT STARTED beyond `UartRx` and the pure framer in `dshot` | |
| Firmware hookup of the passthrough (tap in the USB link task, port shared with MAVLink) | NOT STARTED | |
| Motor test | Waiting: the owner is soldering the motors on | |

## The open item: accepting `PioDshot`

Everything known is in `~/Documents/MARV-V2-drivers/_plan/m9-esc-gaps-and-bursts.md` (run table,
measurements M9 and M10). In short:

- `dshot/tools/run.sh dshot_smoke 600` passed in full three times (zero throttle only). Its very first
  run failed four checks; that was not reproduced in 14 later executions of the affected check.
- Our drive was never at fault in anything measured (pads released after every exchange, our frame whole
  in every window, lapse race whole-or-nothing, cut-off at 399.996 to 399.999 ms).
- Armed ESCs survive 505 ms without frames and restart at 510 to 520 ms: the 0.4 s lapse lock has margin.
- NEW ESC behaviour: every ESC sends one reply-shaped burst right after its arming silence, about 1.2 s
  after its first reply, at a free-running phase. On pad M2 it lands in the driver's capture window on
  almost every start and is reported as `Foreign` on a Live line, which design rule 3.8 treats as an
  alarm, and which check 2 of the bench program asserts never happens.
- Unexplained: two ESCs stop answering for long stretches when sent only one frame per 0.4 s; the first
  run's failure itself.

**Update, 20:20:** the consult's answer arrived and is recorded as the fifth amendment in
`~/Documents/MARV-V2-drivers/_plan/decision-esc-link-20261112.md` ("the logic stays in the firmware"): the
driver is unchanged; the firmware's alarm is two Foreign captures on a Live line with no reply between; arming
waits until every line has replied on every exchange for 1500 ms. A second session had meanwhile built a
driver-side version; the owner rejected it ("I want to keep logic in firmware."; "if it made things violating
the split delete them, do not archive trash.") and it was deleted. `wip-pio-dshot` is at b484b5a. Never have
two sessions work in `dshot/` or on the board at once. The paragraph below is history.

**An architect consult was dispatched at about 18:00 with five questions** (line states and rule 3.8
given that burst; what checks 2 and 6 must assert; the lapse constant; the overlap of the burst with our
frame). It was to append a "Fifth amendment" to
`~/Documents/MARV-V2-drivers/_plan/decision-esc-link-20261112.md`. **First thing to do: look whether that
amendment exists** (`grep -n "Fifth amendment"` in that file). If it does, it is binding. If not, the
consult was cut off: run it again with the evidence file above (the questions are in the plan's log).

Then, in order:
1. Implement what the amendment says (driver: `dshot/src/link.rs`, `src/pio_dshot.rs`; checks 2 and 6
   in `dshot/hil/src/bin/dshot_smoke.rs`), each changed check with a negative control.
2. Add the per-line test for "line still low at the end of the window" at all four positions
   (`src/capture.rs` tests; an audit found the code right and that test missing).
3. Full `dshot_smoke` twice in a row, exit 0.
4. Reviewer against the decision document (all of the back-end; nothing of it has been reviewed).
5. README of `dshot` (it still describes the pure core only and names `hil/` and `tools/`), in the same
   commit; then merge to `first-version`, move `main`, push to `dshot-rs`.
6. Remaining zero-throttle bench checks: cancel sweep, beep commands, extended telemetry on and off, the
   decode cost.
7. ESC serial telemetry: measurement M3 first (request bit on one line at a time; also measure the ESC's
   real baud rate: the receiver's limits are 4.6 % slow to 3.9 % fast), then the driver.
8. With motors, only on the owner's word: low-throttle spin test, RPM in the replies, the current scale.
9. Firmware: the passthrough tap in the link task, and M7 (the configurator on a port that has carried
   MAVLink).

## Rules that bind the next session

- **Zero throttle only** until the owner says to spin. The motors are being soldered on now: before any
  throttle, ask again whether the props are off. Expect the ESCs to beep at every restart from now on.
- **Never write to an ESC** without the owner's word. The permission given today covered one
  save-and-reapply test and is used up. Changing settings and flashing are the owner's, in the
  configurator.
- The PCB repo `~/Documents/kicad/MARV-V2` is read-only. Files under `_plan/ref/` are downloaded code:
  never execute them. Building or running the external crate `embassy-dshot` on the board was denied by
  the permission system: do not work around it.
- The board only through each crate's `tools/run.sh` (machine-wide probe lock).
- A README is part of the unit: updated in the same commit as the code or the bench result, nothing
  pushed without that check (owner's rule, in the plan).
- USB ids: closed. Development builds use 1209:0001.

## The bench as it was left

- Board: last program flashed is `pio-uart`'s `uart_rx_loopback` (finished). It leaves the four ESC
  lines as inputs with pull-ups, so the ESCs sit in their bootloaders: inert. Nothing is running on the
  probe.
- ESCs: Flywoo GOKU G55M, four AM32 2.20 (`FLYWOO_GOKU_F421`, AT32F421, bootloader version 1). Settings
  as the owner had them (ESC 1 and 4 reversed, 14 poles, KV 820, DShot). ESC 1 was re-flashed today with
  the same release 2.20 through the passthrough, verified byte for byte.
- Backups: `_plan/esc-settings-20261112/` (four 184-byte blocks), `_plan/esc-flash-backup-20261112/`
  (flash 0x1000 to 0x7FFF of each), evidence of today's tests in `_plan/esc-write-test-20261112/` and
  `_plan/esc-flash-test-20261112/`.

## How to do the common things

- Passthrough for the configurator: in `esc-passthrough/`,
  `DEMO_USB_VID=1209 DEMO_USB_PID=0001 tools/run.sh passthrough_demo 3600`, wait for
  `link up; silent until the trigger request` in `hil/logs/`, then am32.ca/configurator in Chrome, port
  `0x1209:0x0001`, 115200, Direct unticked. The demo's start-up self-check can print FAIL when the ESCs
  are restarting: known false alarm.
- Read-only check of all four ESCs: build `esc-passthrough/host` (`cargo build --release`), then
  `esc-passthrough-host --port /dev/serial/by-id/usb-MARV_MARV_V2_ESC_passthrough_demo_passthrough-demo-if00
  --compare <four saved blocks> [--dump 0x1000:0x7000 --out DIR]`. The write tool is a separate binary
  behind the cargo feature `write`. The binaries used today were in a session scratch folder that may be
  gone: rebuild them.
- DShot bench: in `dshot/` on branch `wip-pio-dshot`: `tools/run.sh dshot_smoke 600` (about 8 minutes),
  `dshot_m9_gaps`, `dshot_m10_arming`, `dshot_m1_probe`. Build switches of the smoke program:
  `SMOKE_SHORT=1`, `SMOKE_RACE_END=whole|refused|edge`.

## Noticed, not changed

- `esc-passthrough/host/README.md` names the build folder by its absolute path on the owner's machine.
- One framing error was counted on an ESC line during the flash session; not located.
- The `UartRx` fast-side tolerance (+3.9 %) is only 0.9 % above what was tested (+3 %).
- The READMEs of this repo and of MARV-GCS were not checked today.
