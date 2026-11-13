# Handoff: the radio link (ELRS receiver: CRSF and MAVLink on one UART)

Written 2026-11-13, early morning, after a night of host-only work while the owner slept. The controlling plan is
`docs/plan.md` in this repo (local, not in git): section "the radio link", with the owner's words verbatim and the
night's log. The design is `~/Documents/MARV-V2-drivers/_plan/decision-rc-link-20261113.md` (read its first section,
"the finding that gates the MAVLink half", and the amendments at the end). The facts and their confidence are in
`~/Documents/MARV-V2-drivers/_plan/facts-rc-link.md` (section 6: what only the bench can settle; section 7: a USB
transmitter module).

## What the owner wants (his words are in the plan; this is the short form)

Fly manually with the RadioMaster Pocket (ELRS "Normal" link mode, the receiver outputs CRSF), land, disarm, select
another model on the radio (ELRS "MAVLink" link mode) and work from the ground station (MARV-GCS) with rich
telemetry and his own dialect, the sticks still usable as an emergency override. The receiver (RadioMaster
RP4TD-M, ELRS 4.0.1) follows the model change by itself, powered: he confirmed that on the LED. Waypoint
navigation is out of scope. The receiver's settings are never touched by our code.
Standing words that bind: "I want to keep logic in firmware" (the driver reports, the firmware decides); follow
Betaflight and ArduPilot, no extra safety features; "do not over engineer"; "don't let readmes get stale";
"commit and push everything or just commit if local only".

## UPDATE (2026-11-13, about 11:00): R2a is done and the owner has decided how the two modes are used

R2a ran (two captures; facts file, section 8). On ELRS 4.0.1 the receiver sends stick messages in MAVLink mode
only until it has output CRSF once in the same power-up; RADIO_STATUS and the ground-station traffic are not
affected. The owner's decision, verbatim: "we use CRSF only for manual control and basic telemetry and mavlink
only for telemetry rich and autonomous control" / "ok let's assume this approach now. record it. the cost is worth
it for what we get in return." So: the link mode is the control authority; MAVLink mode has no stick source; a
change of mode, also in the air, is ridden out by the firmware like an RF link loss. No receiver power cycle, no
switched supply, no patched receiver firmware. The drivers do not change. Details: the decision document's last
two sections. The "Next step" below is DONE; the next step now is `rc_follow` (the real port on the board: R2, R3,
R4, R5 from one log), being written, then R6.

## State (2026-11-13 morning)

| Piece | Where | State |
|---|---|---|
| CRSF codec `crsf-codec` | `~/Documents/MARV-V2-drivers/crsf-codec`, local only (NO remote), `main` = `first-version` = `3cb6989` | DONE on the host: 32 tests, reviewed twice. Never run against a receiver |
| `mavlink-device`: accept list with a hand-over hook, RADIO_STATUS stream slowdown (ArduPilot's numbers), `AcceptList::iter` | mavlink-device-rs `main` `09dfd17`, pushed | DONE on the host: 115 tests (117 with the test feature), reviewed twice. Never run against a radio |
| Port crate `rc-port` (owns the UART, follows CRSF <-> MAVLink, reports sticks, link figures, counters; no policy) | `~/Documents/MARV-V2-drivers/rc-port`, local only (NO remote), `main` = `first-version` = `b6db329` | DONE on the host: 27 tests, reviewed twice ("no way to get an unguaranteed stick report"); the RP2350 back-end builds. Never run on a board |
| Listen-only capture program `rc_capture` (R2a) | `rc-port/hil/src/bin/rc_capture.rs` | BUILT, reviewed "safe to run with the motors attached", NEVER RUN |
| Ground station: UDP link | MARV-GCS `main` `367c3d3`, pushed | DONE on localhost: 408 tests, reviewed. Never run against a radio |
| R2a to R6 on the board; R7 firmware rules | | NOT STARTED: they need the owner, the receiver wired and powered, and a charged Pocket |

The two flagged design points were not struck by the owner and stand: our own CRSF codec (with `uf-crsf` 0.7.0
pinned as the independent check in the tests), and following the mode change at all (no precedent in Betaflight
or ArduPilot; the scenario needs it).

## Next step: R2a, the listen-only capture (needs the owner)

1. Wire the receiver: its TX to pad R0 (GPIO17), its RX to pad T0 (GPIO14, not driven by the capture), ground,
   and ITS OWN SUPPLY: the IO block's power pins are 3.3 V only; ask the owner what the receiver runs from
   (not recorded). The battery is not needed for the capture (the ESCs then stay unpowered); the program never
   touches the DShot pins either way.
2. In `~/Documents/MARV-V2-drivers/rc-port`: `tools/run.sh rc_capture 600`. The terminal shows only the banner;
   the data is in `hil/logs/`: `grep -E '\[(EVT|SUM|VAL|WIN)\]' <log>`.
3. The owner performs the sequences of `rc-port/hil/README.md` by hand (each three times, about 10 s between
   steps): Normal model with sticks moved, radio off 5 s, radio on; the same in the MAVLink model; receiver
   powered up in Normal, then the FIRST switch to MAVLink; MAVLink -> Normal; Normal -> MAVLink again; a receiver
   power cycle in each mode; receiver unpowered 10 s.
4. Read from the log, and write into the facts file: the CRSF baud really used; what arrives during a change and
   how long the gap is; the pad state the banner prints; the channel values and whether channels 15/16 carry LQ
   and RSSI; and THE GATE: in the MAVLink model, linked, in a 10 s `[WIN]` line, `mav_ovr` (stick messages) 0 with
   `mav_rst` above 500 = the ELRS problem is reproduced.
5. If reproduced: STOP before R4's stick checks and R5, and ask the owner: power-cycle the aircraft after each
   switch into MAVLink mode, a fixed receiver firmware he flashes, or MAVLink mode without the stick override.
   If not reproduced (3 of 3 with sticks resuming, both switch sequences): go on with R2, R3, R4, R5, R6 as the
   decision's table says (each has its "done when" and negative control).

## Open with the owner

- The receiver's supply (see step 1).
- A USB transmitter module: it carries MAVLink by cable and works stand-alone on a desk, but then the Pocket's
  sticks are not in the loop; the Pocket's internal module cannot use USB; an external module in the Pocket's bay
  cabled to the laptop is unverified (does the Pocket have a bay; does a module pass MAVLink on USB while a
  handset drives it). Facts file, section 7.
- R6 needs one message of his own dialect: `marv-dialect` has no `marv.xml` yet, and MARV-GCS still uses the
  `common` dialect directly. Which message does he want first?
- New crates have no GitHub repository (`crsf-codec`, `rc-port`): his to create; names are working names.

## Facts worth not relearning

- The stick frames simply STOP when the radio link is lost, in both modes; nothing is repeated. In MAVLink mode
  RADIO_STATUS keeps coming (about every 10 ms); in CRSF mode the line goes silent. Loss is "time since the last
  stick report": the firmware's decision, not the driver's (Betaflight: 150 ms, stage 2 at 1.5 s; ArduPilot: 1 s;
  overrides 3 s).
- The port reports nothing before the third valid frame of a lock, because CRSF's 8-bit checksum lets about 1 in
  256 false starts through. Lock after a change: about 1.02 s (CRSF -> MAVLink) and 0.21 s (MAVLink -> CRSF) on the
  model.
- embassy-rp 0.10 leaves the UART's RX pad with NO pull and the Schmitt trigger off (pad word 0x40): the line
  floats when the receiver is unpowered or re-opening its port. The capture prints the pad; the real back-end may
  need a pull-up set after UART init.
- The host tests rest on a UART MODEL (no noise, no baud error, no oversampling, no FIFO overrun): how much
  wrong-baud garbage a real line yields is a bench question (R2a, R5).
- `at_us` in the reports is the time the bytes were read: keep the UART receive buffer small (64 bytes is about 2
  RC frames at 250 Hz).
- ELRS: Link Mode is per model on the transmitter and cannot be changed in the Lua script while connected, but
  selecting another model works (owner's test). MAVLink mode forces telemetry ratio 1:2 and forbids the Wide switch
  mode. The laptop reaches MAVLink on the Pocket through the TX backpack's WiFi, UDP 14550 (MARV-GCS:
  `--connect udp:0.0.0.0:14550`).

## Rules for the bench

- The ESC program's rules hold whenever the board is powered: motors are attached (props off); nothing in the
  radio program commands a throttle; the board only through each crate's `tools/run.sh`; never two sessions on
  the board at once.
- No bind, flash or settings write to the receiver from our code or tests.
- New crates without a remote are committed locally only; repositories with a remote are committed and pushed
  after review (owner, 2026-11-13).
