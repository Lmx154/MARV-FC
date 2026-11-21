//! Host tests of the mode switch, the effective mode and (from the second stage) altitude hold
//! (decision-nav-eskf-20261120, Amendment F). The estimator is a fake that returns the state the test sets, so each
//! test chooses what the estimate covers. Every expected number comes from `marv_card` or `Tuning`.

extern crate std;

use super::*;
use crate::alloc::CardAllocator;
use crate::imu::{ImuReading, ImuSample};
use crate::pid::CascadedPid;
use core::cell::Cell;

/// The step period of these tests, us.
pub(super) const DT_US: u64 = 1_250;

/// An estimator that returns the state in its cell, stamped with the step time.
pub(super) struct Fake<'a>(pub &'a Cell<NavState>);

impl Estimator for Fake<'_> {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let mut nav = self.0.get();
        nav.t_us = sensors.imu1.t_us;
        nav
    }
}

pub(super) type TestFlight<'a> = Flight<Fake<'a>, CascadedPid, CardAllocator>;

/// What Angle needs.
pub(super) const ANGLE: Valid = Valid {
    tilt: true,
    rates: true,
    ..Valid::NONE
};

/// What Altitude needs.
pub(super) const ALTITUDE: Valid = Valid {
    velocity_d: true,
    position_d: true,
    ..ANGLE
};

/// Level, at rest, at height `down_m` (down positive), with `valid`.
pub(super) fn nav_at(down_m: f32, valid: Valid) -> NavState {
    NavState {
        t_us: 0,
        attitude: [1.0, 0.0, 0.0, 0.0],
        rates_rads: [0.0; 3],
        velocity_ms: [0.0; 3],
        position_m: [0.0, 0.0, down_m],
        valid,
        sigma: None,
    }
}

pub(super) fn flight<'a>(
    cell: &'a Cell<NavState>,
    handshake: &'static Handshake,
    tuning: &Tuning,
) -> TestFlight<'a> {
    Flight::new(
        Fake(cell),
        CascadedPid::new(tuning),
        CardAllocator::new(),
        tuning,
        handshake,
    )
}

/// A stick report with the arm switch, the throttle stick and the mode switch; the other sticks centred.
pub(super) fn report(t_us: u64, arm: bool, throttle: f32, switch6: Switch3) -> RcSample {
    RcSample {
        sticks: Sticks {
            roll: 0.0,
            pitch: 0.0,
            throttle,
            yaw: 0.0,
        },
        arm,
        switch6,
        t_us,
    }
}

pub(super) fn input(t_us: u64, rc: Option<RcSample>) -> FlightInputs {
    FlightInputs {
        sensors: Sensors {
            imu1: ImuSample {
                reading: ImuReading::default(),
                t_us,
                seq: 0,
            },
            imu2: None,
            baro: None,
            mag: None,
            gnss: None,
            external: None,
        },
        rc,
    }
}

/// One step at `t_us` with a fresh report (so RC loss never fires and the switch and sticks update).
pub(super) fn step_rc(
    f: &mut TestFlight,
    t_us: u64,
    arm: bool,
    throttle: f32,
    switch6: Switch3,
) -> StepOut {
    f.step(&input(t_us, Some(report(t_us, arm, throttle, switch6))))
}

/// The mode byte MOT carries in the flight records of `out`.
pub(super) fn mot_mode_byte(out: &StepOut) -> u8 {
    let mut buf = [0u8; 256];
    let len = encode_step(out, &mut buf);
    let at = usize::from(dflog::NAV.length)
        + usize::from(dflog::SETP.length)
        + usize::from(dflog::DMD.length);
    assert!(len >= at + usize::from(dflog::MOT.length));
    // MOT: header 3, TimeUS 8, Arm 1, then Mode.
    buf[at + 12]
}

#[test]
fn the_effective_mode_rule() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let not_covered = [
        Valid {
            position_d: false,
            ..ALTITUDE
        },
        Valid {
            velocity_d: false,
            ..ALTITUDE
        },
        Valid {
            tilt: false,
            ..ALTITUDE
        },
        Valid {
            rates: false,
            ..ALTITUDE
        },
        ANGLE,
        Valid::NONE,
    ];
    // (switch, selected, effective when Altitude is covered, effective when it is not)
    let table = [
        (Switch3::Low, Mode::Angle, Mode::Angle, Mode::Angle),
        (Switch3::Middle, Mode::Altitude, Mode::Altitude, Mode::Angle),
        // Position counts as Altitude until it is built.
        (Switch3::High, Mode::Position, Mode::Altitude, Mode::Angle),
    ];
    for (n, &(switch, selected, covered, uncovered)) in table.iter().enumerate() {
        let cell = Cell::new(nav_at(0.0, ALTITUDE));
        let mut f = flight(&cell, &H, &tuning);
        let mut t = 1_000;
        let out = step_rc(&mut f, t, false, 0.0, switch);
        assert_eq!((out.selected, out.mode), (selected, covered), "row {n}");
        assert_eq!(mot_mode_byte(&out), mode_code(covered));
        for (k, valid) in not_covered.iter().enumerate() {
            cell.set(nav_at(0.0, *valid));
            t += DT_US;
            let out = step_rc(&mut f, t, false, 0.0, switch);
            assert_eq!((out.selected, out.mode), (selected, uncovered), "row {n}/{k}");
            assert_eq!(mot_mode_byte(&out), mode_code(uncovered), "row {n}/{k}");
        }
    }
}

#[test]
fn before_the_first_report_the_mode_is_angle_and_the_last_switch_holds() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let out = f.step(&input(1_000, None));
    assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Angle));
    let out = step_rc(&mut f, 2_000, false, 0.0, Switch3::Middle);
    assert_eq!((out.selected, out.mode), (Mode::Altitude, Mode::Altitude));
    // No new report: the switch holds.
    let out = f.step(&input(3_000, None));
    assert_eq!((out.selected, out.mode), (Mode::Altitude, Mode::Altitude));
    let out = step_rc(&mut f, 4_000, false, 0.0, Switch3::Low);
    assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Angle));
}

/// Negative control: a Flight whose effective mode ignores coverage is Altitude where the rule says Angle.
#[test]
fn the_coverage_switch_changes_the_effective_mode() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, &H, &tuning);
    assert_eq!(step_rc(&mut f, 1_000, false, 0.0, Switch3::Middle).mode, Mode::Angle);
    f.ignore_coverage = true;
    assert_eq!(step_rc(&mut f, 2_000, false, 0.0, Switch3::Middle).mode, Mode::Altitude);
}

#[test]
fn mode_is_written_at_the_first_step_and_at_every_change_of_either() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let mut log = ModeLog::new();
    let mut t = 1_000;
    let mut written = std::vec::Vec::new();
    let mut run = |f: &mut TestFlight, switch: Switch3, t: &mut u64, written: &mut std::vec::Vec<[u8; 13]>| {
        *t += DT_US;
        let out = step_rc(f, *t, false, 0.0, switch);
        if log.pending(&out) {
            let mut b = [0u8; 13];
            assert_eq!(encode_mode(&out, &mut b), 13);
            written.push(b);
            log.done(&out);
        }
    };
    // Steps: Low x3, Middle x3 (Eff 1 -> 2), the estimate drops (Eff 2 -> 1, Sel stays 2), returns, High, Low.
    for _ in 0..3 {
        run(&mut f, Switch3::Low, &mut t, &mut written);
    }
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    cell.set(nav_at(0.0, ANGLE));
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    cell.set(nav_at(0.0, ALTITUDE));
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    run(&mut f, Switch3::High, &mut t, &mut written);
    run(&mut f, Switch3::Low, &mut t, &mut written);
    let codes: std::vec::Vec<(u8, u8)> = written.iter().map(|b| (b[11], b[12])).collect();
    // (Sel, Eff): first step, Middle, Altitude lost, Altitude back, High (Eff stays 2: Position acts as Altitude), Low.
    assert_eq!(codes, [(1, 1), (2, 2), (2, 1), (2, 2), (3, 2), (1, 1)]);
    for b in &written {
        assert_eq!(b[..3], [0xA3, 0x95, dflog::ID_MODE]);
    }
    // The first record is at the first step's time.
    assert_eq!(u64::from_le_bytes(written[0][3..11].try_into().unwrap()), 1_000 + DT_US);
    // A record the ring dropped is offered again: `pending` stays true until `done`.
    let out = step_rc(&mut f, t + DT_US, false, 0.0, Switch3::Middle);
    assert!(log.pending(&out));
    assert!(log.pending(&out));
    // Too small a buffer writes nothing.
    assert_eq!(encode_mode(&out, &mut [0u8; 12]), 0);
}
