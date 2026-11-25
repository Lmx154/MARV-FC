//! Host tests of the in-air restart at the step level (decision-inair-restart-20261125): after an arm-switch STOP in
//! the air, the arm switch back ON within `RESTART_WINDOW_US` while IMU1 reads free fall arms with any throttle and lean;
//! the arming flies Altitude for an Angle selection (cancelled by a mode-switch change) and latches no home. Every other
//! disarm, a STOP on the ground, an impact, a late edge, a non-finite sample, no tilt and the N19 rule are refused as
//! today. The estimator is the fake of `althold_tests`; IMU1's specific force is set per step.

extern crate std;

use super::althold_tests::{ALTITUDE, ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use crate::gate::{ArmNote, FallNote, FallOff, RESTART_WINDOW_US, Refusal};
use crate::hands_off::Reason;
use core::cell::Cell;
use fw_core::rc::RcSource;
use std::vec::Vec;

/// Level and at rest (or at hover): 1 g up in the body frame.
const REST: [f32; 3] = [0.0, 0.0, -GRAVITY_MSS];

/// Altitude covered (Position not: no NE velocity, no heading), the NE position valid so home can latch.
const NE_ALTITUDE: Valid = Valid { position_ne: true, ..ALTITUDE };

/// Log 346's STOP height, m above take-off (down negative).
const STOP_D: f32 = -7.3;

fn handshake() -> &'static Handshake {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    h
}

/// The free fall of log 346 after the STOP: |f| in its 0.3 to 1.6 m/s^2 band, mostly along body z.
fn band(k: u64) -> [f32; 3] {
    let frac = (k as f32 * 0.618_034) % 1.0;
    [0.05, 0.0, -(0.3 + 1.3 * frac)]
}

/// |f| a fixed fraction of g along body z.
fn g_frac(x: f32) -> [f32; 3] {
    [0.0, 0.0, -x * GRAVITY_MSS]
}

struct Rig {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
    source: RcSource,
    mission: bool,
    switch6: Switch3,
    valid: Valid,
}

impl Rig {
    /// Disarmed on the ground; a MAVLink flight (FS_HOLD_EN 1) for receiver reports.
    fn new(source: RcSource, switch6: Switch3, valid: Valid) -> Rig {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(0.0, valid))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_fs_hold_en(source == RcSource::Receiver);
        Rig { f, cell, t: 1_000_000, source, mission: false, switch6, valid }
    }

    /// The estimate: level at `down_m`, `v_d` m/s down, with the rig's coverage.
    fn nav(&self, down_m: f32, v_d: f32) {
        let mut n = nav_at(down_m, self.valid);
        n.velocity_ms[2] = v_d;
        self.cell.set(n);
    }

    /// One step at `t` with IMU1's specific force `f_ms2` and, when `arm` is `Some`, a report.
    fn step_at(&mut self, t: u64, arm: Option<bool>, throttle: f32, f_ms2: [f32; 3]) -> StepOut {
        self.t = t;
        let rc = arm.map(|arm| RcSample {
            sticks: Sticks { roll: 0.0, pitch: 0.0, throttle, yaw: 0.0 },
            arm,
            switch6: self.switch6,
            source: self.source,
            mission: self.mission,
            t_us: t,
        });
        let mut i = input(t, rc);
        i.sensors.imu1.reading.accel_ms2 = f_ms2;
        self.f.step(&i)
    }

    fn step(&mut self, arm: Option<bool>, throttle: f32, f_ms2: [f32; 3]) -> StepOut {
        self.step_at(self.t + DT_US, arm, throttle, f_ms2)
    }

    /// Armed on the ground with the switch cycle, then 2 s at hover at `STOP_D` (the airborne latch forced, released
    /// before the STOP step so that step reads it as the flight left it); a report every 8th step (10 ms).
    fn hover(&mut self) {
        self.step(Some(false), 0.0, REST);
        self.step(Some(true), 0.0, REST);
        assert!(self.f.gate.armed(), "armed on the ground");
        self.nav(STOP_D, 0.0);
        self.f.airborne_forced = true;
        for k in 0..1_600 {
            assert!(self.step((k % 8 == 0).then_some(true), 0.6, REST).armed);
        }
        self.f.airborne_forced = false;
        assert!(self.f.airborne && self.f.hands_off.ever_airborne());
        assert_eq!(self.f.hands_off_state(), State::Pilot);
    }

    /// The arm switch OFF at hover (IMU1 still reads hover): the STOP step's time.
    fn stop(&mut self) -> u64 {
        let o = self.step(Some(false), 0.6, REST);
        assert!(!o.armed && o.motors.throttles().is_none(), "the STOP stops the motors");
        assert_eq!(self.f.hands_off_state(), State::Stop);
        self.t
    }

    /// `n` steps falling from `stop` (the estimate's height and v_D in free fall), the switch OFF in a report every 10 ms
    /// at `throttle`, IMU1 reading `f(steps since the STOP)`; the free-fall notes with their step since the STOP.
    fn fall(&mut self, stop: u64, n: u64, throttle: f32, f: impl Fn(u64) -> [f32; 3]) -> Vec<(u64, FallNote)> {
        let mut notes = Vec::new();
        for _ in 0..n {
            let k = (self.t + DT_US - stop) / DT_US;
            self.fall_nav(stop, self.t + DT_US);
            let o = self.step((k % 8 == 0).then_some(false), throttle, f(k));
            assert!(!o.armed && o.motors.throttles().is_none());
            assert_eq!(self.f.arm_note(), None);
            if let Some(note) = self.f.fall_note() {
                notes.push((k, note));
            }
        }
        notes
    }

    fn fall_nav(&self, stop: u64, t: u64) {
        let s = (t - stop) as f32 * 1e-6;
        self.nav(STOP_D + 0.5 * GRAVITY_MSS * s * s, GRAVITY_MSS * s);
    }

    /// The switch ON at `t` with `throttle` and IMU1 reading `f_ms2`.
    fn on_at(&mut self, stop: u64, t: u64, throttle: f32, f_ms2: [f32; 3]) -> StepOut {
        self.fall_nav(stop, t);
        self.step_at(t, Some(true), throttle, f_ms2)
    }
}

/// The edge step's time at `ms` after the STOP.
fn after(stop: u64, ms: u64) -> u64 {
    stop + ms * 1_000
}

/// Steps between the STOP and an edge at `ms` (exclusive).
fn steps_to(ms: u64) -> u64 {
    ms * 1_000 / DT_US - 1
}

/// Log 346 (2026-11-25, 621.167 s): a MAVLink flight hovering at 7.3 m, the arm switch OFF with the mission switch OFF
/// (STOP "arm-ms-off"), free fall with |f| in the log's 0.3 to 1.6 m/s^2 band, the switch back ON 0.63 s later with the
/// throttle at 0.74 (refused "thr" in the log). Now: "FALL on" at the first sample after the STOP, the edge arms with
/// "ARM fall t=630 thr=74", the motors run from that step, the next step is PILOT by the normal armed edge, the effective
/// mode is Altitude (Angle raised, or the log's Position selection without Position coverage) with the collective
/// demand above m g while falling at 6 m/s, and no home is latched. The band omits the spin-down transient of the raw
/// log (see the handback of this change).
#[test]
fn log_346_trace_restarts_at_0_63_s_into_altitude_with_no_home() {
    for switch6 in [Switch3::Low, Switch3::High] {
        let mut r = Rig::new(RcSource::Receiver, switch6, NE_ALTITUDE);
        r.hover();
        let home_before = r.f.hands_off.home();
        assert!(home_before.is_some(), "the flight's own home");
        let stop = r.stop();
        assert_eq!(r.f.hands_off_reason(), Some(Reason::ArmMsOff));
        let notes = r.fall(stop, steps_to(630), 0.74, band);
        assert_eq!(notes, [(1, FallNote::On)], "{switch6:?}");
        let edge = after(stop, 630);
        let o = r.on_at(stop, edge, 0.74, band(504));
        assert!(o.armed && o.motors.throttles().is_some(), "{switch6:?}: the edge restarts");
        let [x, _, z] = band(504);
        let f_x100 = (libm::sqrtf(x * x + z * z) * 100.0 + 0.5) as u32;
        assert_eq!(r.f.arm_note(), Some(ArmNote::Restart { t_ms: 630, f_x100, thr_x100: 74 }));
        let mg = MASS * GRAVITY_MSS;
        for k in 1..=80u64 {
            r.fall_nav(stop, r.t + DT_US);
            let o = r.step((k % 8 == 0).then_some(true), 0.74, band(504 + k));
            assert!(o.armed && o.motors.throttles().is_some(), "{switch6:?} step {k}");
            assert_eq!(o.mode, Mode::Altitude, "{switch6:?} step {k}");
            assert!(o.demand.is_ok_and(|d| d.thrust_n > mg), "{switch6:?} step {k}: {:?}", o.demand);
            assert_eq!(r.f.hands_off_state(), State::Pilot);
            assert_eq!(r.f.hands_off.home(), None, "{switch6:?} step {k}: no home on a restart arming");
        }
        assert_eq!(r.f.hands_off_reason(), Some(Reason::Armed));
    }
}

/// The restart's Altitude needs the estimate to cover it (Angle otherwise), lasts for that arming, and a change of the
/// mode switch cancels it until disarm (the low-battery descent's pattern).
#[test]
fn the_restart_altitude_needs_coverage_and_a_mode_switch_change_cancels_it() {
    // Not covered: Angle.
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ANGLE);
    r.hover();
    let stop = r.stop();
    r.fall(stop, steps_to(400), 0.74, band);
    assert!(r.on_at(stop, after(stop, 400), 0.74, band(320)).armed);
    for _ in 0..20 {
        assert_eq!(r.step(Some(true), 0.74, band(0)).mode, Mode::Angle);
    }
    // Covered: Altitude, then Middle (Altitude, selected), then Low again: Angle, cancelled.
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
    r.hover();
    let stop = r.stop();
    assert_eq!(r.f.hands_off_reason(), Some(Reason::ArmOff));
    r.fall(stop, steps_to(400), 0.74, band);
    assert!(r.on_at(stop, after(stop, 400), 0.74, band(320)).armed);
    for _ in 0..20 {
        let o = r.step(Some(true), 0.74, band(0));
        assert_eq!((o.selected, o.mode), (Mode::Angle, Mode::Altitude));
    }
    r.switch6 = Switch3::Middle;
    assert_eq!(r.step(Some(true), 0.74, band(0)).mode, Mode::Altitude);
    r.switch6 = Switch3::Low;
    for _ in 0..20 {
        let o = r.step(Some(true), 0.74, band(0));
        assert!(o.armed);
        assert_eq!((o.selected, o.mode), (Mode::Angle, Mode::Angle), "cancelled");
    }
}

/// Negative control: a STOP on the ground (landed: the airborne latch clear, ever airborne, TD held) is "FALL off
/// ground"; free fall after it (the vehicle dropped) and the switch ON at 0.74 is refused "thr" as today.
#[test]
fn a_ground_stop_then_free_fall_is_refused() {
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
    r.step(Some(false), 0.0, REST);
    r.step(Some(true), 0.0, REST);
    for k in 0..400 {
        r.step((k % 8 == 0).then_some(true), 0.8, REST);
    }
    assert!(r.f.airborne && r.f.hands_off.ever_airborne(), "lifted");
    for k in 0..2_400 {
        r.step((k % 8 == 0).then_some(true), 0.0, REST);
    }
    assert!(!r.f.airborne && r.f.hands_off.ever_airborne() && r.f.still_us >= Tuning::DEFAULT.land_detect_us, "landed");
    let o = r.step(Some(false), 0.0, REST);
    assert!(!o.armed);
    assert_eq!(r.f.hands_off_reason(), Some(Reason::ArmOff));
    assert_eq!(r.f.fall_note(), Some(FallNote::Off(FallOff::Ground)));
    let stop = r.t;
    let notes = r.fall(stop, steps_to(630), 0.74, |_| g_frac(0.05));
    assert!(notes.is_empty(), "{notes:?}");
    let o = r.on_at(stop, after(stop, 630), 0.74, g_frac(0.05));
    assert!(!o.armed && o.motors.throttles().is_none(), "ground STOP: no restart");
    assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Thr)));
}

/// Negative control: after the latch is set, one sample at 0.26 g (an impact) clears it for good: low samples after it
/// and the switch ON are refused "thr". Twin: the same with that sample at 0.24 g restarts.
#[test]
fn one_sample_at_or_above_0_25_g_after_the_latch_clears_it_for_good() {
    for (bump, restarts) in [(0.26f32, false), (0.24, true)] {
        let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
        r.hover();
        let stop = r.stop();
        let notes = r.fall(stop, steps_to(630), 0.74, |k| if k == 100 { g_frac(bump) } else { g_frac(0.05) });
        let o = r.on_at(stop, after(stop, 630), 0.74, g_frac(0.05));
        assert_eq!(o.armed, restarts, "bump {bump}");
        if restarts {
            assert_eq!(notes, [(1, FallNote::On)]);
            assert!(matches!(r.f.arm_note(), Some(ArmNote::Restart { .. })));
        } else {
            assert_eq!(notes, [(1, FallNote::On), (100, FallNote::Off(FallOff::Impact))]);
            assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Thr)));
        }
    }
}

/// Negative control: the window is 5 s from the STOP: an edge at 5.000 s restarts, at 5.001 s it is refused "thr" (and
/// the first sample past 5 s logged "FALL off late").
#[test]
fn the_restart_window_ends_5_s_after_the_stop() {
    assert_eq!(RESTART_WINDOW_US, 5_000_000);
    for (edge_us, restarts) in [(5_000_000u64, true), (5_001_000, false)] {
        let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
        r.hover();
        let stop = r.stop();
        let notes = r.fall(stop, steps_to(5_000), 0.74, |_| g_frac(0.05));
        assert_eq!(notes, [(1, FallNote::On)]);
        if !restarts {
            // One more sample, past the window, before the edge.
            r.fall_nav(stop, stop + 5_000_500);
            r.step_at(stop + 5_000_500, None, 0.74, g_frac(0.05));
            assert_eq!(r.f.fall_note(), Some(FallNote::Off(FallOff::Late)));
        }
        let o = r.on_at(stop, stop + edge_us, 0.74, g_frac(0.05));
        assert_eq!(o.armed, restarts, "edge at {edge_us} us");
        if !restarts {
            assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Thr)));
        }
    }
}

/// Negative control: a non-finite IMU1 sample after the latch is set clears it for good ("FALL off f"); the edge later
/// is refused "thr".
#[test]
fn a_non_finite_sample_clears_the_latch_for_good() {
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
    r.hover();
    let stop = r.stop();
    let notes = r.fall(stop, steps_to(630), 0.74, |k| if k == 160 { [0.0, 0.0, f32::NAN] } else { g_frac(0.05) });
    assert_eq!(notes, [(1, FallNote::On), (160, FallNote::Off(FallOff::NotFinite))]);
    let o = r.on_at(stop, after(stop, 630), 0.74, g_frac(0.05));
    assert!(!o.armed);
    assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Thr)));
}

/// Negative control: with the latch true, no tilt on the edge step is refused "tilt"; the cycle is cleared, and a new
/// cycle with tilt back restarts (the window and the latch are still in force).
#[test]
fn no_tilt_on_the_edge_is_refused_and_a_new_cycle_restarts() {
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
    r.hover();
    let stop = r.stop();
    r.fall(stop, steps_to(630), 0.74, |_| g_frac(0.05));
    r.valid = Valid { tilt: false, ..ALTITUDE };
    let o = r.on_at(stop, after(stop, 630), 0.74, g_frac(0.05));
    assert!(!o.armed);
    assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Tilt)));
    r.valid = ALTITUDE;
    let o = r.on_at(stop, after(stop, 640), 0.74, g_frac(0.05));
    assert!(!o.armed, "switch still on: no cycle");
    assert_eq!(r.f.arm_note(), None);
    r.fall_nav(stop, after(stop, 650));
    r.step_at(after(stop, 650), Some(false), 0.74, g_frac(0.05));
    let o = r.on_at(stop, after(stop, 660), 0.74, g_frac(0.05));
    assert!(o.armed);
    assert!(matches!(r.f.arm_note(), Some(ArmNote::Restart { t_ms: 660, .. })));
}

/// Negative control (N19): a receiver report with the mission switch ON never arms, the restart included ("ms").
#[test]
fn a_receiver_report_with_the_mission_switch_on_is_refused() {
    let mut r = Rig::new(RcSource::Receiver, Switch3::High, ALTITUDE);
    r.hover();
    let stop = r.stop();
    assert_eq!(r.f.hands_off_reason(), Some(Reason::ArmMsOff));
    r.fall(stop, steps_to(630), 0.74, band);
    r.mission = true;
    let o = r.on_at(stop, after(stop, 630), 0.74, band(504));
    assert!(!o.armed);
    assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Ms)));
}

/// Negative control: throttle 0.74 with no in-air STOP (never armed, IMU1 reading free fall) is refused "thr" as today.
#[test]
fn throttle_high_without_an_in_air_stop_is_refused() {
    let mut r = Rig::new(RcSource::Crsf, Switch3::Low, ALTITUDE);
    for k in 0..800 {
        r.step((k % 8 == 0).then_some(false), 0.74, g_frac(0.05));
    }
    let o = r.step(Some(true), 0.74, g_frac(0.05));
    assert!(!o.armed && o.motors.throttles().is_none());
    assert_eq!(r.f.arm_note(), Some(ArmNote::Refused(Refusal::Thr)));
    assert_eq!(r.f.fall_note(), None);
}

/// The MSG texts: "ARM no <token>", "ARM fall t=<ms> f=<x100> thr=<x100>", "FALL on", "FALL off <token>", NUL padded.
#[test]
fn restart_msg_texts() {
    let text = |m: &[u8]| std::string::String::from_utf8(m.iter().copied().take_while(|&b| b != 0).collect()).unwrap();
    assert_eq!(text(&arm_msg(ArmNote::Refused(Refusal::Thr))), "ARM no thr");
    assert_eq!(text(&arm_msg(ArmNote::Refused(Refusal::Busy))), "ARM no busy");
    assert_eq!(text(&arm_msg(ArmNote::Restart { t_ms: 630, f_x100: 95, thr_x100: 74 })), "ARM fall t=630 f=95 thr=74");
    let max = arm_msg(ArmNote::Restart { t_ms: u32::MAX, f_x100: u32::MAX, thr_x100: u32::MAX });
    assert_eq!(text(&max), "ARM fall t=4294967295 f=4294967295 thr=4294967295");
    assert_eq!(text(&fall_msg(FallNote::On)), "FALL on");
    let offs = [(FallOff::Ground, "ground"), (FallOff::Late, "late"), (FallOff::Impact, "impact"), (FallOff::NotFinite, "f")];
    for (why, token) in offs {
        assert_eq!(text(&fall_msg(FallNote::Off(why))), std::format!("FALL off {token}"));
    }
}
