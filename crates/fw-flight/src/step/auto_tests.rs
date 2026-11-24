//! Host tests of AUTO on the real step (decision-mavlink-mode-20261123, Amendment A, A4-A6, A9, A10; section 36; H3b):
//! the mission facts from the pushed target set, Start, the leg law and its exits. The estimator is the fake of
//! `althold_tests` (origin `ORIGIN`); the plant, where one is needed, is a point mass: the horizontal velocity follows
//! the acceleration of the step's attitude setpoint (as `handsoff_tests::brake_run`), plus a steady push when a test
//! sets one; the height follows the setpoint's vertical target, v_D = its rate + 1.0/s x (target - D).

extern crate std;

use super::althold_tests::{ALTITUDE, DT_US, ORIGIN, TestFlight, flight, input};
use super::*;
use crate::hands_off::{Command, CommandResult, DenyReason, Link, StationCommand};
use core::cell::Cell;

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

/// Latitude and longitude (1e-7 degree) of the point `n`, `e` metres from the fake's origin (`ned_from_geo` is linear in
/// the differences).
fn geo(n: f32, e: f32) -> (i32, i32) {
    let probe = ned_from_geo(&ORIGIN, &GeoPoint { lat_e7: ORIGIN.lat_e7 + 100_000, lon_e7: ORIGIN.lon_e7 + 100_000, alt_mm: ORIGIN.alt_mm });
    let (kn, ke) = (probe[0] / 100_000.0, probe[1] / 100_000.0);
    (ORIGIN.lat_e7 + libm::roundf(n / kn) as i32, ORIGIN.lon_e7 + libm::roundf(e / ke) as i32)
}

/// The uploaded mission (`GDN_SRC` 1) of waypoints (north, east, z above home), GDN_RADIUS `radius_m`, return height
/// 20 m, heights checked by core 0.
fn mission(wps: &[(f32, f32, f32)], radius_m: f32, plan_id: u32) -> GdnTargets {
    let mut m = GdnTargets { from_mission: true, heights_ok: true, return_m: 20.0, plan_id, radius_m, count: wps.len() as u8, ..MISSION_OFF };
    for (k, &(n, e, z)) in wps.iter().enumerate() {
        m.targets[k] = geo(n, e);
        m.alts_m[k] = z;
    }
    m
}

/// The 30 m square at 10, 15, 10, 10 m.
fn square() -> GdnTargets {
    mission(&[(30.0, 0.0, 10.0), (30.0, 30.0, 15.0), (0.0, 30.0, 10.0), (0.0, 0.0, 10.0)], 2.0, 0x5157_0001)
}

/// The nose at 30 degrees.
const YAW_RAD: f32 = 0.523_598_8;

/// A flight on the fake estimator: armed on the ground at the origin (home N 0, E 0, D 0), lifted, put 10 m up, the
/// sticks lost on T3' with FS_HOLD_EN 1: HOLD. The station is heard on the radio link at every step while `station`.
struct Run {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
    station: bool,
    /// The plant's steady push, m/s^2 north and east.
    push: [f32; 2],
    last: Option<StepOut>,
}

impl Run {
    fn new(speed_ms: f32, set: Option<GdnTargets>) -> Run {
        Run::with(speed_ms, set, true, true)
    }

    /// As `new`; without `home` the NE position is not valid until after take-off (no home, A6); with `fs` false
    /// FS_HOLD_EN is 0 and the stick loss gives LAND.
    fn with(speed_ms: f32, set: Option<GdnTargets>, home: bool, fs: bool) -> Run {
        let valid = if home { POSITION } else { Valid { position_ne: false, velocity_ne: false, ..POSITION } };
        let mut nav = super::althold_tests::nav_at(0.0, valid);
        nav.attitude = [libm::cosf(YAW_RAD / 2.0), 0.0, 0.0, libm::sinf(YAW_RAD / 2.0)];
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav)));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        assert!(f.set_pilot_speed_xy(speed_ms));
        f.set_fs_hold_en(fs);
        f.set_heading_ok(true);
        f.set_battery_ok(true);
        if let Some(m) = set {
            f.set_mission(m);
        }
        let mut r = Run { f, cell, t: 1_000_000, station: true, push: [0.0; 2], last: None };
        r.rc(false, 0.0);
        assert!(r.rc(true, 0.0).armed);
        for _ in 0..400 {
            r.rc(true, 0.5);
        }
        assert!(r.f.airborne && r.f.hands_off.home().is_some() == home);
        let mut n = r.cell.get();
        n.position_m[2] = -10.0;
        n.valid = POSITION;
        r.cell.set(n);
        for _ in 0..2_000 {
            r.quiet();
            if r.state() != State::Pilot {
                break;
            }
        }
        assert_eq!(r.state(), if fs { State::Hold } else { State::Land });
        r
    }

    fn rc(&mut self, arm: bool, throttle: f32) -> StepOut {
        self.t += DT_US;
        let t = self.t;
        self.f.set_last_radio_heartbeat_us(Some(t));
        let report = RcSample { sticks: Sticks { roll: 0.0, pitch: 0.0, throttle, yaw: 0.0 }, arm, switch6: Switch3::High, t_us: t };
        self.f.step(&input(t, Some(report)))
    }

    /// One step without a report and without the plant.
    fn quiet(&mut self) -> StepOut {
        self.t += DT_US;
        if self.station {
            self.f.set_last_radio_heartbeat_us(Some(self.t));
        }
        let o = self.f.step(&input(self.t, None));
        self.last = Some(o);
        o
    }

    /// The plant moved by the previous step's output, then one step without a report.
    fn fly(&mut self) -> StepOut {
        let dt = DT_US as f32 * 1e-6;
        let mut n = self.cell.get();
        if let Some(o) = self.last {
            let a = match o.setpoint {
                Setpoint::Position { .. } if o.allocation.is_some() => accel_ne(o.attitude_sp),
                Setpoint::Altitude { attitude, .. } => accel_ne(attitude),
                _ => [0.0; 2],
            };
            let (down, rate) = match o.setpoint {
                Setpoint::Position { position_m, velocity_ms, .. } => (position_m[2], velocity_ms[2]),
                Setpoint::Altitude { down_m, velocity_d_ms, .. } => (down_m, velocity_d_ms),
                _ => (n.position_m[2], 0.0),
            };
            let vd = rate + (down - n.position_m[2]);
            for k in 0..2 {
                n.velocity_ms[k] += (a[k] + self.push[k]) * dt;
                n.position_m[k] += n.velocity_ms[k] * dt;
            }
            n.velocity_ms[2] = vd;
            n.position_m[2] += vd * dt;
        }
        self.cell.set(n);
        self.quiet()
    }

    fn command(&mut self, c: Command) -> CommandResult {
        self.f.set_command(StationCommand { command: c, link: Link::Radio });
        self.fly();
        self.f.take_command_result().expect("answered")
    }

    fn state(&self) -> State {
        self.f.hands_off_state()
    }
}

/// The horizontal acceleration (north, east) of a point mass held at the weight's vertical share by attitude `q`.
fn accel_ne(q: [f32; 4]) -> [f32; 2] {
    let [w, x, y, z] = q;
    let col = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
    [-GRAVITY_MSS * col[0] / col[2], -GRAVITY_MSS * col[1] / col[2]]
}

/// Item 1: the facts come from the pushed set. `GDN_SRC` 0 (the parameters, no heights) is no mission; a height outside
/// the limits, then a valid list, reach Start's checks in their order.
#[test]
fn mission_facts_from_the_pushed_set_reach_start() {
    let mut params = square();
    params.from_mission = false;
    params.heights_ok = false;
    let mut r = Run::new(3.0, Some(params));
    assert_eq!(r.command(Command::Start), CommandResult::Denied(DenyReason::NoWaypoints));
    let mut high = square();
    high.heights_ok = false;
    r.f.set_mission(high);
    assert_eq!(r.command(Command::Start), CommandResult::Denied(DenyReason::HeightOutsideLimits));
    r.f.set_mission(MISSION_OFF);
    assert_eq!(r.command(Command::Start), CommandResult::Denied(DenyReason::NoWaypoints));
}

/// Horizontal distance of the vehicle from the waypoint put at (n, e), m: from where it lands after `geo`'s rounding to
/// 1e-7 degree (up to about 1 cm), as the guidance converts it.
fn dist_from(r: &Run, n: f32, e: f32) -> f32 {
    let (lat_e7, lon_e7) = geo(n, e);
    let [wn, we, _] = ned_from_geo(&ORIGIN, &GeoPoint { lat_e7, lon_e7, alt_mm: ORIGIN.alt_mm });
    let p = r.cell.get().position_m;
    libm::hypotf(p[0] - wn, p[1] - we)
}

/// The heading of a Position setpoint.
fn heading(o: &StepOut) -> Option<f32> {
    match o.setpoint {
        Setpoint::Position { heading_rad, heading_rate_rads, .. } => {
            assert_eq!(heading_rate_rads, 0.0);
            Some(heading_rad)
        }
        _ => None,
    }
}

/// (a) The 30 m square at 3 m/s, heights 10, 15, 10, 10 m: Start in HOLD gives AUTO; each waypoint is reached within
/// GDN_RADIUS at its height; after the last, HOLD there with the place dropped; e stays below X; the heading is that of
/// Start on every AUTO step.
#[test]
fn a_square_at_3_m_s_flies_every_waypoint_and_holds_at_the_last() {
    let wps = [(30.0, 0.0, 10.0f32), (30.0, 30.0, 15.0), (0.0, 30.0, 10.0), (0.0, 0.0, 10.0)];
    let mut r = Run::new(3.0, Some(square()));
    for _ in 0..800 {
        r.fly();
    }
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.leg_active()), (State::Auto, Some(0)));
    let x = hands_off::track_fault_m(r.f.speed_xy_in_force());
    let (mut e_max, mut steps, mut reached) = (0.0f32, 0u32, std::vec::Vec::new());
    let mut active = Some(0u8);
    while r.state() == State::Auto && steps < 200_000 {
        let o = r.fly();
        steps += 1;
        if r.state() == State::Auto {
            if let Some(h) = heading(&o) {
                assert!(libm::fabsf(h - YAW_RAD) < 1e-5, "heading {h}");
            }
            e_max = e_max.max(r.f.track_error_m.expect("e"));
        }
        let now = r.f.leg_active();
        if now != active || r.state() != State::Auto {
            let k = usize::from(active.unwrap());
            let (n, e, z) = wps[k];
            let h = -r.cell.get().position_m[2];
            reached.push((k, dist_from(&r, n, e), h - z, steps as f32 * DT_US as f32 * 1e-6));
            active = now;
        }
    }
    std::println!("reached (waypoint, distance m, height error m, s): {reached:?}; e max {e_max:.3} m against X {x} m");
    assert_eq!(reached.iter().map(|r| r.0).collect::<std::vec::Vec<_>>(), [0, 1, 2, 3]);
    for &(k, d, dh, _) in &reached {
        assert!(d <= 2.0, "waypoint {k}: {d} m");
        assert!(libm::fabsf(dh) < 0.5, "waypoint {k}: height error {dh} m");
    }
    assert!(e_max < x && e_max <= guidance::LAG_E_MAX_M + 0.05, "e max {e_max}");
    assert_eq!(r.state(), State::Hold);
    for _ in 0..4_000 {
        r.fly();
        assert_eq!(r.state(), State::Hold);
    }
    let (d, h) = (dist_from(&r, 0.0, 0.0), -r.cell.get().position_m[2]);
    std::println!("5 s after: {d:.2} m from the last waypoint at {h:.2} m");
    assert!(d <= 2.0 && libm::fabsf(h - 10.0) < 0.5, "{d} {h}");
    // The place was dropped: Start begins at waypoint 0.
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!(r.f.leg_active(), Some(0));
}

/// (b) Pause mid-leg holds with the place kept; Start resumes the same waypoint on a new first leg from the vehicle;
/// Abort holds with the place dropped and Start begins at waypoint 0.
#[test]
fn b_pause_resumes_the_same_waypoint_from_the_vehicle_and_abort_starts_over() {
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    while r.f.leg_active() != Some(1) {
        r.fly();
    }
    for _ in 0..4_000 {
        r.fly();
    }
    assert_eq!(r.command(Command::Pause), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.leg_active()), (State::Hold, None));
    for _ in 0..4_000 {
        r.fly();
    }
    let at = r.cell.get().position_m;
    assert!(libm::hypotf(at[0] - 30.0, at[1] - 30.0) > 5.0, "mid-leg: {at:?}");
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.leg_active()), (State::Auto, Some(1)));
    // The first leg starts at the vehicle: the carrot of the Start step is where the vehicle was.
    match r.last.unwrap().setpoint {
        Setpoint::Position { position_m, .. } => {
            assert!(libm::hypotf(position_m[0] - at[0], position_m[1] - at[1]) < 0.05, "{position_m:?} {at:?}");
        }
        ref s => panic!("{s:?}"),
    }
    for _ in 0..800 {
        r.fly();
    }
    assert_eq!(r.command(Command::Abort), CommandResult::Accepted);
    assert_eq!(r.state(), State::Hold);
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.leg_active()), (State::Auto, Some(0)));
}

/// (g) Start's answers on the real step: each case the first check that fails, in the state machine's order.
#[test]
fn g_start_denied_with_its_reason() {
    let deny = |r: &mut Run| r.command(Command::Start);
    let mut gdn_src_0 = square();
    gdn_src_0.from_mission = false;
    gdn_src_0.heights_ok = false;
    let mut high = square();
    high.heights_ok = false;
    let far = mission(&[(30.0, 0.0, 10.0), (1_001.0, 0.0, 10.0)], 2.0, 0x5157_0002);
    let near = mission(&[(30.0, 0.0, 10.0), (999.0, 0.0, 10.0)], 2.0, 0x5157_0003);
    let cases: [(&str, Option<GdnTargets>, DenyReason); 4] = [
        ("no mission", None, DenyReason::NoWaypoints),
        ("GDN_SRC 0", Some(gdn_src_0), DenyReason::NoWaypoints),
        ("a waypoint beyond 1 km", Some(far), DenyReason::WaypointBeyondRange),
        ("a height outside the limits", Some(high), DenyReason::HeightOutsideLimits),
    ];
    for (name, set, want) in cases {
        let mut r = Run::new(3.0, set);
        assert_eq!(deny(&mut r), CommandResult::Denied(want), "{name}");
        assert_eq!(r.state(), State::Hold, "{name}");
    }
    let mut r = Run::new(3.0, Some(near));
    assert_eq!(deny(&mut r), CommandResult::Accepted, "999 m");
    // The board's battery input (none on the flight core).
    let mut r = Run::new(3.0, Some(square()));
    r.f.set_battery_ok(false);
    assert_eq!(deny(&mut r), CommandResult::Denied(DenyReason::BatteryNotOk), "no battery input");
    // Station absent (no radio heartbeat for 5 s).
    let mut r = Run::new(3.0, Some(square()));
    r.station = false;
    for _ in 0..4_100 {
        r.fly();
    }
    assert_eq!(deny(&mut r), CommandResult::Denied(DenyReason::StationAbsent), "station absent");
    // Not in HOLD: LAND.
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Land), CommandResult::Accepted);
    assert_eq!(deny(&mut r), CommandResult::Denied(DenyReason::NotInThisState), "LAND");
    // No home: the NE position became valid only after take-off.
    let mut r = Run::with(3.0, Some(square()), false, true);
    assert_eq!(r.f.hands_off.home(), None);
    assert_eq!(deny(&mut r), CommandResult::Denied(DenyReason::NoHome), "no home");
    // FS_HOLD_EN 0: the stick loss lands; Start is HandsOffDisabled.
    let mut r = Run::with(3.0, Some(square()), true, false);
    assert_eq!(r.state(), State::Land);
    assert_eq!(deny(&mut r), CommandResult::Denied(DenyReason::HandsOffDisabled), "FS_HOLD_EN 0");
}
