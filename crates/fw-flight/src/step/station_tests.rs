//! Host tests of the station-command seam (`Flight::set_command`, `Flight::take_command_result`) with the real mailbox
//! (`fw_core::flight_cmd`, `crate::station`): a command reaches the state machine on the step that takes it, and its
//! answer comes back with the same sequence number; no command stops the motors.

extern crate std;

use super::althold_tests::{ALTITUDE, ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use crate::hands_off::{Command, CommandResult, DenyReason, Link, StationCommand};
use crate::station;
use core::cell::Cell;
use fw_core::flight_cmd::{ACCEPTED, FlightCommand, Mailbox, request_byte};

const POSITION: Valid = Valid {
    heading: true,
    velocity_ne: true,
    position_ne: true,
    ..ALTITUDE
};

fn handshake() -> &'static Handshake {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    h
}

fn rc(t: u64, arm: bool, throttle: f32) -> RcSample {
    RcSample {
        sticks: Sticks { roll: 0.0, pitch: 0.0, throttle, yaw: 0.0 },
        arm,
        switch6: Switch3::High,
        t_us: t,
    }
}

/// The board's step: the waiting request, the step, its answer (`crate::flight::Stepper::sample`'s order).
fn board_step(f: &mut TestFlight, m: &Mailbox, t: u64, report: Option<RcSample>) -> StepOut {
    let taken = station::take(m);
    if let Some((_, c)) = taken {
        f.set_command(c);
    }
    let o = f.step(&input(t, report));
    if let Some((seq, _)) = taken {
        station::answer(m, seq, f.take_command_result());
    }
    o
}

#[test]
fn a_command_through_the_mailbox_is_applied_on_the_step_that_takes_it_and_answered_with_its_sequence_number() {
    let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, POSITION))));
    let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
    f.set_fs_hold_en(true);
    let m = Mailbox::new();
    let mut t = 1_000_000u64;
    let mut go = |f: &mut TestFlight, report: Option<(bool, f32)>| {
        t += DT_US;
        f.set_last_radio_heartbeat_us(Some(t));
        board_step(f, &m, t, report.map(|(arm, throttle)| rc(t, arm, throttle)))
    };
    go(&mut f, Some((false, 0.0)));
    assert!(go(&mut f, Some((true, 0.0))).armed);
    for _ in 0..400 {
        go(&mut f, Some((true, 0.5)));
    }
    assert!(f.airborne);
    // PILOT with fresh sticks: DENIED, code 2, with this command's number; the state is unchanged.
    let s = m.post(request_byte(FlightCommand::Land, true)).unwrap();
    go(&mut f, Some((true, 0.5)));
    assert_eq!(m.poll(s), Some(DenyReason::CrsfFresh.code()));
    assert_eq!(f.hands_off_state(), State::Pilot);
    m.end(s, 0);
    // Stick loss: HOLD.
    for _ in 0..2_000 {
        go(&mut f, None);
        if f.hands_off_state() != State::Pilot {
            break;
        }
    }
    assert_eq!(f.hands_off_state(), State::Hold);
    // USB: DENIED, code 1.
    let s = m.post(request_byte(FlightCommand::Land, false)).unwrap();
    go(&mut f, None);
    assert_eq!(m.poll(s), Some(1));
    m.end(s, 0);
    // A step with no request leaves no answer.
    go(&mut f, None);
    assert_eq!(f.take_command_result(), None);
    // Land on the radio link: ACCEPTED, LAND at the step that took it, the answer tagged with its number only.
    let s = m.post(request_byte(FlightCommand::Land, true)).unwrap();
    assert_eq!(m.poll(s), None);
    let o = go(&mut f, None);
    assert_eq!(m.poll(s), Some(ACCEPTED));
    assert_eq!(f.hands_off_state(), State::Land);
    assert!(o.armed, "Land is a descent, not a stop (N)");
    m.end(s, 0);
    // Taken once: the next steps see no command.
    go(&mut f, None);
    assert_eq!(f.take_command_result(), None);
    // Start: DENIED NoWaypoints or NoHome on this branch (AUTO is not reachable), never ACCEPTED.
    let s = m.post(request_byte(FlightCommand::Start, true)).unwrap();
    go(&mut f, None);
    assert_ne!(m.poll(s), Some(ACCEPTED));
    assert!(m.poll(s).is_some());
    m.end(s, 0);
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 33
    }
    fn chance(&mut self, per_1000: u64) -> bool {
        self.next() % 1000 < per_1000
    }
    fn unit(&mut self) -> f32 {
        (self.next() % 10_001) as f32 / 10_000.0
    }
}

/// N: no command stops the motors. A random walk (reports, silences, coverage, motion, arming cycles; FS_HOLD_EN 1, the
/// station heard on the radio link) with one of the five commands on a random link at 5 % of the steps, in whatever
/// state the walk is in. Every disarm must have the pilot's switch (a report with arm OFF at that step), the touchdown
/// rule (TD held at the previous step) or a stick loss on the ground as its cause.
#[test]
fn n_no_command_in_any_state_disarms_or_stops_the_motors() {
    let mut accepted = std::collections::BTreeMap::<(u8, u8), u32>::new();
    let mut disarms = 0u32;
    for seed in [0x5eed_0001u64, 0x5eed_0002, 0x5eed_0003] {
        let cell = Cell::new(nav_at(0.0, ANGLE));
        let mut f = flight(&cell, handshake(), &Tuning::DEFAULT);
        f.set_fs_hold_en(true);
        let mut rng = Lcg(seed);
        let mut t = 1_000_000u64;
        let (mut quiet, mut arm) = (0u32, false);
        let mut valid = ANGLE;
        let (mut pos, mut vel) = ([0.0f32; 3], [0.0f32; 3]);
        let mut newest = None::<u64>;
        let all = [Command::Start, Command::Pause, Command::Abort, Command::Land, Command::Return];
        for _ in 0..40_000 {
            t += DT_US;
            if quiet == 0 && rng.chance(2) {
                quiet = 900 + (rng.next() % 3000) as u32;
            }
            quiet = quiet.saturating_sub(1);
            if rng.chance(4) {
                arm = !arm;
            }
            if rng.chance(3) {
                valid = [ANGLE, ALTITUDE, POSITION, POSITION, POSITION][(rng.next() % 5) as usize];
            }
            for k in 0..3 {
                vel[k] = (vel[k] + 0.02 * (rng.unit() - 0.5)).clamp(-3.0, 3.0);
                pos[k] += vel[k] * DT_US as f32 * 1e-6;
            }
            let mut n = nav_at(pos[2], valid);
            n.position_m = pos;
            n.velocity_ms = vel;
            cell.set(n);
            let report = (quiet == 0 && !rng.chance(100)).then(|| RcSample {
                sticks: Sticks {
                    roll: rng.unit() - 0.5,
                    pitch: rng.unit() - 0.5,
                    throttle: if f.gate.armed() { 0.3 + 0.6 * rng.unit() } else { 0.0 },
                    yaw: rng.unit() - 0.5,
                },
                arm,
                switch6: Switch3::High,
                t_us: t,
            });
            f.set_last_radio_heartbeat_us(Some(t));
            let command = rng.chance(50).then(|| StationCommand {
                command: all[(rng.next() % 5) as usize],
                link: if rng.chance(800) { Link::Radio } else { Link::Usb },
            });
            let state = f.hands_off_state();
            let (was_armed, air, td) = (f.gate.armed(), f.airborne, f.still_us >= f.land_detect_us);
            if report.is_some() {
                newest = Some(t);
            }
            let lost = newest.is_none_or(|n| t - n > Tuning::DEFAULT.rc_fs_timeout_us);
            if let Some(c) = command {
                f.set_command(c);
            }
            let o = f.step(&input(t, report));
            let result = f.take_command_result();
            assert_eq!(result.is_some(), command.is_some(), "an answer exactly for a step with a command");
            if let (Some(c), Some(CommandResult::Accepted)) = (command, result) {
                *accepted.entry((c.command as u8, state.code())).or_default() += 1;
            }
            if was_armed && !o.armed {
                disarms += 1;
                let pilot = report.is_some_and(|r| !r.arm);
                assert!(
                    pilot || td || (lost && !air),
                    "seed {seed:#x} t {t}: a disarm without the pilot, TD or a ground stick loss (command {command:?}, {state:?})"
                );
            }
            if command.is_some() && was_armed {
                assert!(o.armed || report.is_some_and(|r| !r.arm) || td || (lost && !air), "a command stopped the motors");
            }
        }
    }
    std::println!("accepted (command, state code): {accepted:?}; disarms {disarms}");
    assert!(accepted.keys().any(|&(_, s)| s == State::Hold.code()), "the walk took commands in HOLD");
    assert!(accepted.keys().any(|&(_, s)| s == State::Land.code() || s == State::Blind.code()), "and in LAND or BLIND");
    assert!(disarms > 0, "the walk disarms");
}
