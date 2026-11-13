# Handoff: ESC link (DShot, AM32 passthrough, telemetry input) and battery sensing

Rewritten 2026-11-12, about 23:10, at the owner's request (the session's context had grown too large). The
controlling plan is `docs/plan.md` in this repo (local, not in git): its log has every step with evidence. The
design is `~/Documents/MARV-V2-drivers/_plan/decision-esc-link-20261112.md`: read the SIXTH and SEVENTH
amendments first, then the fifth (only the parts the sixth says are kept).

## The owner's standing words (verbatim; they bind)

- Goal: "drivers for ESC communication with AM32, as well as other battery functionality like current
  detection, and voltage detection"; "if you can make our board work as a passthrough so we can access the esc
  through the configurator then that would be perfect."
- "I want to keep logic in firmware." (the driver reports, the firmware decides)
- "remove things that don't align with what betaflight and ardupilot have in place ... i think adding in extra
  safety features could end up backfiring"
- "if it made things violating the split delete them, do not archive trash."
- "don't let readmes get stale" (README updated in the same commit as the code or bench result)
- "I need you to be more usage aware. delegate more to opus 5.5 and sonnett 5.5 whenever possible. i only want
  you to do the troubleshooting and architecture auditing"
- "are you sure we're testing this correctly? cross check with other repos. you need to look outside of the
  current scope" (check AM32's source, Betaflight, ArduPilot BEFORE a long bench investigation)
- "let me know when dshot is ready and working so we can move on to soldering the motors"
- Closed, do not raise again: crates.io ("leave ... alone it's not important right now"); renaming ws2812c's
  feature ("leave it as is"); the release rule after a non-Stop frame and the 3 s park in `start()` ("leave it
  as is"); cross-checking our codec against other DShot crates ("no").

## State

Driver crates: `~/Documents/MARV-V2-drivers/<crate>`, accepted work on `main`, pushed to
`github.com/Lmx154/<crate>` (`dshot` -> `dshot-rs`). Work branches: `first-version` (new crates), `to-1.0`
(the eight older ones).

| Piece | State |
|---|---|
| Cable pull on battery | DONE (usb-cdc) |
| Battery voltage | DONE (adc-battery); current scale UNKNOWN until there is a load |
| AM32 passthrough | DONE on the standalone demo: the real configurator read, changed a setting, flashed one ESC, verified byte for byte (esc-passthrough) |
| `pio-uart` `OneWire` and `UartRx` | DONE, reviewed; no real ESC telemetry received yet |
| DShot pure core | DONE, pushed (dshot-rs main 43c212d) |
| DShot back-end `PioDshot` | DONE at zero throttle, no motors: accepted, reviewed, README current, pushed (dshot-rs main b7c823a, 2026-11-12 night). Not yet run with motors |
| READMEs / "Works with" / package metadata | Done and pushed for the 14 other crates |
| crates.io | Six crates prepared (icm45686, lsm6dsk320x, lc76g, bmm350 as `bmm350-async`, bmp581 as `bmp581-async`, esc-passthrough); nothing published (the owner's e-mail was not verified); PARKED by the owner |
| ESC serial telemetry, firmware passthrough hookup, firmware arming rules | NOT STARTED |
| Motors | NOT soldered. The owner has the lead's word to solder (back-end accepted) |

## The DShot back-end: exactly where it stands

Design in force (sixth amendment): NO hardware lapse lock, no alarm rule, no arm gate. The driver is one PIO
state machine (15 instructions) driving four lines, bidirectional, a frame only when the application pushes
one, `start()` (park 3 s, 25 ms break, release), `exchange()` returning the four captures and line states
(`Starting`/`Live`, Live latched at the first good reply), counters, cancellation safety, `release()`, `Drop`.
The firmware's rules will be written Betaflight-style later (arm when every line has answered; count errors).

Done on the board (zero throttle, no motors): the removal passed two full runs; check 6 (release, rebuild,
Drop) restructured and passing; cancel sweep with a release attempt after each of 2156 cancels and a
double-cancel sweep (940): every failure counter 0; NO driver defect found by any of it.

State of the tree when the session stopped (23:15): the last commit on `wip-pio-dshot` is 4226808 ("WIP
checkpoint: lapse lock and bench reply rules removed"). Everything after it is UNCOMMITTED in the working tree
(`git -C ~/Documents/MARV-V2-drivers/dshot status`): check 6 restructured, the single and double cancel sweeps
(passing on the board in logs -221711, -222912, -223115), and the SEVENTH amendment fully coded in
`hil/src/bin/dshot_smoke.rs` (check 2 with the bounded transient: `classify`, `a_form`, `Zone`; addendum C's
exemption at our release; the 25 us pacing margin; the five negative controls) but NOT YET RUN on the board. It
builds (`cargo build --release --bin dshot_smoke` and clippy clean in `hil/`); `src/` is unchanged since the
host checks last passed. The engineer's own checkpoint commit was refused by the permission system, so the
first act of the next session (or the owner) is to commit that tree as a WIP checkpoint.

**UPDATE (next session, same night): steps 1 to 4 below are DONE** (plan log, last entry: six passing full
runs, two reviews PASS, squashed to dshot-rs main b7c823a; `wip-pio-dshot` is local and has the same tree). The
"State of the tree" paragraph above is history. START AT STEP 5, after the owner has soldered the motors. Before
step 5's runs, consider the bench-review follow-ups listed in the plan log.

Remaining, in order (all delegated; the lead only audits and troubleshoots):
1. Commit the working tree as a WIP checkpoint; ONE development run of the seventh-amendment code and fix
   only what it shows; then THREE CONSECUTIVE full runs, `tools/run.sh dshot_smoke 600` in `dshot/` (about 70 s
   each, exit 0, no source change between them). If one fails: look at the failing lines with their edges and
   check AM32's source before believing a bench theory.
2. Independent review (reviewer) of the whole back-end against the decision document. Nothing has been reviewed.
3. Mechanical and docs (coder): `cargo fmt` in `dshot/hil` (never formatted); README rewrite (back-end API,
   what was measured with log names, the controller's start sequence and late reply, the overlap sentence,
   tested cadence 1 kHz and "sparse frames make AM32 drop out", "Works with", what is not covered); Cargo
   description (still says "no I/O"); package metadata like the other crates. One commit; merge to
   `first-version`; move `main`; push to `dshot-rs`.
4. Tell the owner the back-end is accepted and that the motors can be soldered.
5. With motors, still zero throttle: the same three runs. Then, only on the owner's word and after asking
   whether the props are off: a low-throttle spin test, RPM in the replies, the current scale.
Later: zero-throttle extras (beep commands, extended telemetry, decode cost); ESC serial telemetry (measure
the ESC's real baud rate first; `UartRx` tolerates -4.6 % to +3.9 %); the firmware's passthrough tap on the
port shared with MAVLink; the firmware's arming and error counting.

## Facts worth not relearning

- AM32 2.20 (source in `_plan/ref/AM32-v2.20`): armed signal timeout about 510 ms (measured: survives 505,
  restarts at 510-520), 2 s when not armed; on arming it plays a tune with interrupts off for about 300 ms: no
  replies, then one late reply at a free phase, and the next exchange can be odd too (capture re-armed
  mid-frame). After a reset the bootloader parks on an idle-high line; only the 25 ms break starts the
  application. One lone frame every 0.4 s makes every ESC drop into its bootloader within minutes: never send
  sparse frames; keep 1 kHz or more.
- Betaflight and ArduPilot never stop on one or two bad telemetry packets, have no hardware stall cut-off, and
  never send sparse frames.
- Measurement notes: `_plan/m9-esc-gaps-and-bursts.md`, `_plan/m1-...`, `_plan/m8-...`; today's bench logs in
  `dshot/hil/logs/`.
- Never two sessions in `dshot/` or on the board at once (it happened today; the other session's work was
  deleted on the owner's order).

## Rules for the bench

- Zero throttle only until the owner says to spin. Never write to an ESC without the owner's word (settings
  and flashing are the owner's, in the configurator). The PCB repo `~/Documents/kicad/MARV-V2` is read-only.
  Files under `_plan/ref/` are never executed. The board only through each crate's `tools/run.sh`.
- Bench as left: battery plugged in (the bench runs need it), no motors, four AM32 2.20 ESCs with their
  original settings (ESC 1 re-flashed with the same 2.20 today, verified); the last program on the board is
  `dshot_smoke`, which ends with the lines released, so the ESCs sit in their bootloaders.
- Passthrough for the configurator: in `esc-passthrough/`,
  `DEMO_USB_VID=1209 DEMO_USB_PID=0001 tools/run.sh passthrough_demo 3600`, then am32.ca/configurator in Chrome.
