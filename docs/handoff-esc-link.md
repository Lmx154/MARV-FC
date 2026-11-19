# Handoff: ESC link (DShot, AM32 passthrough, telemetry input) and battery sensing

Rewritten 2026-11-12, about 23:10, at the owner's request (the session's context had grown too large). The
controlling plan is `docs/plan.md` in this repo (local, not in git): its log has every step with evidence. The
design is `~/Documents/MARV-V2-drivers/_plan/decision-esc-link-20261112.md`: read the SIXTH and SEVENTH
amendments first, then the fifth (only the parts the sixth says are kept).

Updated 2026-11-15: the motors are wired and were spun, the working branches are gone and the state paragraphs below are
brought to the present (what is still open is listed in "What is still open").

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
`github.com/Lmx154/<crate>` (`dshot` -> `dshot-rs`). The local work branches (`first-version`, `to-1.0`,
`wip-pio-dshot`, `wip-spin`) no longer exist: every driver repository has only `main`.

| Piece | State |
|---|---|
| Cable pull on battery | DONE (usb-cdc) |
| Battery voltage | DONE (adc-battery); current scale UNKNOWN until there is a load |
| AM32 passthrough | DONE on the standalone demo: the real configurator read, changed a setting, flashed one ESC, verified byte for byte (esc-passthrough) |
| `pio-uart` `OneWire` and `UartRx` | DONE, reviewed; no real ESC telemetry received yet |
| DShot pure core | DONE, pushed (dshot-rs main 43c212d) |
| DShot back-end `PioDshot` | DONE: accepted, reviewed, pushed (dshot-rs main b7c823a, 2026-11-12 night). With the four motors attached: three zero-throttle runs passed (`dshot_smoke-20261113-022722`, `-022838`, `-022952`) and the spin test `dshot_spin` ran all four at up to 5 % (`dshot_spin-20261113-030326`, PASS); dshot-rs main `3557eb1` |
| READMEs / "Works with" / package metadata | Done and pushed for the 14 other crates |
| crates.io | Six crates prepared (icm45686, lsm6dsk320x, lc76g, bmm350 as `bmm350-async`, bmp581 as `bmp581-async`, esc-passthrough); nothing published (the owner's e-mail was not verified); PARKED by the owner |
| Firmware DShot telemetry (`MARV-V2-firmware` `src/esc.rs`: Stop frames at 1 kHz from every boot, command 13 bursts, ESC_STATUS / ESC_INFO on USB, ESC row 12; design `_plan/decision-esc-telemetry-20261118.md`) | BUILT 2026-11-18; run on the board 2026-11-18 with the battery, Stop frames only (4cfb1f0 and 0d25909; logs `20261118-191955` to `20261118-194637`): all four online, EDT voltage and temperature, rpm 0, 1000 exchanges/s, error rate 0 after start-up; figures in the README, ESC telemetry |
| ESC serial telemetry, firmware passthrough hookup, firmware arming rules | NOT STARTED |
| Motors | Wired since 2026-11-13 ("motors are wired and ready to test"), no propellers; positions and directions: `docs/pinout.md`, bench configuration |

## The DShot back-end: exactly where it stands

Design in force (sixth amendment): NO hardware lapse lock, no alarm rule, no arm gate. The driver is one PIO
state machine (15 instructions) driving four lines, bidirectional, a frame only when the application pushes
one, `start()` (park 3 s, 25 ms break, release), `exchange()` returning the four captures and line states
(`Starting`/`Live`, Live latched at the first good reply), counters, cancellation safety, `release()`, `Drop`.
The firmware's rules will be written Betaflight-style later (arm when every line has answered; count errors).

Done on the board (zero throttle, no motors): the removal passed two full runs; check 6 (release, rebuild,
Drop) restructured and passing; cancel sweep with a release attempt after each of 2156 cancels and a
double-cancel sweep (940): every failure counter 0; NO driver defect found by any of it.

Present state (updated 2026-11-15): the work of that night is committed, reviewed and squashed into dshot-rs
`main` (`b7c823a`, then `a6fd136` and `3557eb1` for the spin test); the six passing full runs of `dshot_smoke`
and the review are in the plan log. The driver's README is current. The WIP branches are deleted.

**UPDATE 2 (2026-11-13, about 03:10): step 5 is DONE as far as the first spin.** Motors soldered (Emax ECO II 2807
1700KV, 14 magnets, 4S; M1 rear left CCW, M2 front left CW, M3 rear right CW, M4 front right CCW: `docs/pinout.md`).
Three zero-throttle runs with the motors attached passed; `dshot_spin` (feature `motor-test`, guards G2 to G6,
cap 5 %) spun all four, one at a time: `dshot_spin-20261113-030326`, RESULT: PASS; dshot-rs main `3557eb1`. AM32
reports a 2 us period ("not started yet") until its first commutation: never read an eRPM above what the motor
can do as a speed. STILL OPEN in the ESC program: the stop on signal loss; the board's current scale (the ESCs'
own current reading, about 155 A at rest, is unusable as a reference); ESC serial telemetry; the firmware's
passthrough hookup and arming rules.

## What is still open

Steps 1 to 5 of the earlier plan (checkpoint, three full runs, review, README, tell the owner, then the same three
runs with the motors and the first spin test at the 5 % cap) are DONE. Still open:
- The stop on signal loss (the driver does not stop the motors; the firmware's rule, Betaflight-style).
- The board's current scale: the ESCs' own current reading (153 to 160 A at rest) is unusable as a reference; a
  reference other than the ESC is needed.
- ESC serial telemetry (measure the ESC's real baud rate first; `UartRx` tolerates -4.6 % to +3.9 %).
- The firmware's hookup: the passthrough tap on the port shared with MAVLink and arming rules (arm when every line
  has answered). The DShot stream with error counting is BUILT in the firmware (README, ESC telemetry) and ran on
  the board on 2026-11-18 (4cfb1f0 and 0d25909, logs `20261118-191955` to `20261118-194637`; Stop frames only).
- The firmware's decode cost: not isolated; the shortest exchange (156-157 us, 0d25909, `20261118-193433`) bounds
  decode, frame choice, wake and two clock reads at 11.4 us, against the 10 us tripwire.
- Noticed (2026-11-18, not analysed): an RC UART framing/break burst after `xip-park`'s core-0 park (33 warnings,
  logs `20261118-194106` / `20261118-194357`; 1 overrun on 2026-11-16); one RC UART overrun at a settings save
  (`20261118-193433`).
- Zero-throttle extras (beep commands, extended telemetry, decode cost) and the bench-review follow-ups of the plan
  log (the self-test for a short pulse at our release, the 53 ns tolerance, the arming block's position, `shorts_lost`
  and `ring_over` asserted per rate only in check 3).

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

- Zero throttle only, except the capped spin test the owner allowed on 2026-11-13 (5 % cap, props off, `dshot_spin`, feature `motor-test`); anything beyond it needs his word again. Never write to an ESC without the owner's word (settings
  and flashing are the owner's, in the configurator). The PCB repo `~/Documents/kicad/MARV-V2` is read-only.
  Files under `_plan/ref/` are never executed. The board only through each crate's `tools/run.sh`.
- Bench: battery in, propellers off (the owner's word, 2026-11-18); the firmware itself now sends Stop frames on
  GPIO36-39 from every boot; the four motors are attached; four AM32
  2.20 ESCs with their original settings (ESC 1 was re-flashed with the same 2.20 on 2026-11-12, verified).
  `dshot_smoke` and `dshot_spin` end with the lines released, so the ESCs then sit in their bootloaders.
- Passthrough for the configurator: in `esc-passthrough/`,
  `DEMO_USB_VID=1209 DEMO_USB_PID=0001 tools/run.sh passthrough_demo 3600`, then am32.ca/configurator in Chrome.
