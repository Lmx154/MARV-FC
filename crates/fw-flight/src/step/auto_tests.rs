//! Host tests of AUTO on the real step (decision-mavlink-mode-20261123, Amendment A, A4-A6, A9, A10; section 36; H3b):
//! the mission facts from the pushed target set, Start, the leg law and its exits. The estimator is the fake of
//! `althold_tests` (origin `ORIGIN`); the plant, where one is needed, is a point mass: the horizontal velocity follows
//! the acceleration of the step's attitude setpoint (as `handsoff_tests::brake_run`), plus a steady push when a test
//! sets one; the height follows the setpoint's vertical target, v_D = its rate + 1.0/s x (target - D), down to the
//! ground at D 0, where it stops; IMU1 reads at rest (so TD can hold on the ground).

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
pub(super) fn geo(n: f32, e: f32) -> (i32, i32) {
    let probe = ned_from_geo(&ORIGIN, &GeoPoint { lat_e7: ORIGIN.lat_e7 + 100_000, lon_e7: ORIGIN.lon_e7 + 100_000, alt_mm: ORIGIN.alt_mm });
    let (kn, ke) = (probe[0] / 100_000.0, probe[1] / 100_000.0);
    (ORIGIN.lat_e7 + libm::roundf(n / kn) as i32, ORIGIN.lon_e7 + libm::roundf(e / ke) as i32)
}

/// The uploaded mission (`GDN_SRC` 1) of waypoints (north, east, z above home), GDN_RADIUS `radius_m`, return height
/// 20 m, heights checked by core 0.
pub(super) fn mission(wps: &[(f32, f32, f32)], radius_m: f32, plan_id: u32) -> GdnTargets {
    let mut m = GdnTargets { from_mission: true, heights_ok: true, return_m: 20.0, plan_id, radius_m, count: wps.len() as u8, ..MISSION_OFF };
    for (k, &(n, e, z)) in wps.iter().enumerate() {
        m.targets[k] = geo(n, e);
        m.alts_m[k] = z;
    }
    m
}

/// The 30 m square at 10, 15, 10, 10 m.
pub(super) fn square() -> GdnTargets {
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
        // The height target the shaper captured on the ground moves with the vehicle (the fake does not fly the climb).
        r.f.shaper.target_d = Some(-10.0);
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
        let mut i = input(self.t, None);
        i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, -GRAVITY_MSS];
        let o = self.f.step(&i);
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
            if n.position_m[2] >= 0.0 {
                n.position_m[2] = 0.0;
                n.velocity_ms = [0.0; 3];
            }
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

/// FX-2 item 1: the square with GDN_RADIUS 10 m at 3 m/s (X 7 m): the arrival radius in force is min(10, X - 3.0) = 4 m,
/// so after each arrival the carrot at the reached corner is within 4 m and e stays below X at the 90 degree corners;
/// every waypoint reached within 4 m, HOLD at the last, never LAND. With GDN_RADIUS itself the first corner tripped the
/// track fault (e = 10 m).
#[test]
fn fx2_a_square_with_gdn_radius_10_m_at_3_m_s_completes_with_e_below_x() {
    let wps = [(30.0, 0.0, 10.0f32), (30.0, 30.0, 15.0), (0.0, 30.0, 10.0), (0.0, 0.0, 10.0)];
    let mut m = square();
    m.radius_m = 10.0;
    let mut r = Run::new(3.0, Some(m));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    let x = hands_off::track_fault_m(r.f.speed_xy_in_force());
    let radius = x - guidance::ARRIVAL_MARGIN_M;
    let (mut e_max, mut active, mut reached) = (0.0f32, Some(0u8), std::vec::Vec::new());
    let mut steps = 0u32;
    while r.state() == State::Auto && steps < 200_000 {
        r.fly();
        steps += 1;
        if r.state() == State::Auto {
            e_max = e_max.max(r.f.track_error_m.expect("e"));
        }
        let now = r.f.leg_active();
        if now != active || r.state() != State::Auto {
            let k = usize::from(active.unwrap());
            reached.push((k, dist_from(&r, wps[k].0, wps[k].1)));
            active = now;
        }
    }
    std::println!("FX-2 (1) GDN_RADIUS 10 m at 3 m/s: radius in force {radius} m; reached {reached:?}; e max {e_max:.3} m against X {x} m; {:?}", r.state());
    assert_eq!(radius, 4.0);
    assert_eq!(reached.iter().map(|r| r.0).collect::<std::vec::Vec<_>>(), [0, 1, 2, 3]);
    assert!(reached.iter().all(|&(_, d)| d <= radius + 0.01), "{reached:?}");
    assert!(e_max < x && !r.f.hands_off.track_fault(), "e max {e_max}");
    assert_eq!(r.state(), State::Hold);
}

/// N18 (b), the guidance's vertical carrot: on the square's leg from 15 m down to 10 m the vehicle is put 2.5 m below the
/// descending carrot mid-descent. The carrot stops at the lead cap (|d - carrot| >= 1.5 m), and while it stands its rate,
/// the feed-forward, is 0, so the loop asks for P (carrot - d) only and the vehicle climbs back toward it: there is no
/// fixed point outside the cap (unlike LAND, whose feed-forward stays 1.5 m/s). The carrot moves again and the leg
/// completes. Nothing changed; the climb-back and the time the carrot stood are printed.
#[test]
fn n18_auto_vehicle_below_a_descending_carrot_climbs_back_and_the_leg_completes() {
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    while !(r.f.leg_active() == Some(2) && -r.f.legs.down_m() < 13.5) {
        r.fly();
        assert_eq!(r.state(), State::Auto);
    }
    let mut n = r.cell.get();
    n.position_m[2] = r.f.legs.down_m() + 2.5;
    r.cell.set(n);
    let h_push = height(&r);
    let (mut stood, mut prev, mut top, mut steps) = (0u32, r.f.legs.down_m(), h_push, 0u32);
    while r.f.leg_active() == Some(2) {
        r.fly();
        steps += 1;
        assert!(steps < 40_000 && r.state() == State::Auto, "the leg never completed");
        let d = r.f.legs.down_m();
        if d == prev && -d > 10.0 {
            stood += 1;
        }
        prev = d;
        top = top.max(height(&r));
    }
    std::println!("N18 AUTO: pushed to {h_push:.2} m (2.5 m below the carrot); carrot stood {:.3} s; climb-back to {top:.2} m; leg done after {:.1} s", stood as f32 * 1.25e-3, steps as f32 * 1.25e-3);
    assert!(stood > 0 && r.f.leg_active() == Some(3));
    while r.state() == State::Auto {
        r.fly();
    }
    assert_eq!(r.state(), State::Hold);
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
    // The battery not known OK.
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

/// A run in AUTO on the square, 3 s after Start.
fn in_auto() -> Run {
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    for _ in 0..2_400 {
        r.fly();
    }
    assert_eq!(r.state(), State::Auto);
    r
}

/// (c) A new plan id in AUTO gives HOLD at that step with the place dropped; Start then begins at waypoint 0 of the new
/// list.
#[test]
fn c_a_new_plan_in_auto_holds_with_the_place_dropped() {
    let mut r = in_auto();
    while r.f.leg_active() != Some(1) {
        r.fly();
    }
    let mut next = square();
    next.plan_id = 0x5157_00ff;
    r.f.set_mission(next);
    r.fly();
    assert_eq!((r.state(), r.f.leg_active()), (State::Hold, None));
    for _ in 0..2_400 {
        r.fly();
        assert_eq!(r.state(), State::Hold);
    }
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!(r.f.leg_active(), Some(0));
}

/// (d) No station heartbeat for 5 s in AUTO: HOLD with the place kept (Start resumes the same waypoint).
#[test]
fn d_station_lost_5_s_in_auto_holds_with_the_place_kept() {
    let mut r = in_auto();
    r.station = false;
    let mut steps = 0u64;
    while r.state() == State::Auto {
        r.fly();
        steps += 1;
    }
    let s = steps * DT_US;
    assert!(s > hands_off::STATION_LOST_US && s <= hands_off::STATION_LOST_US + 2 * DT_US, "{s} us");
    assert_eq!(r.state(), State::Hold);
    r.station = true;
    r.fly();
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    assert_eq!(r.f.leg_active(), Some(0), "the kept place: still waypoint 0");
}

/// (e) A plant that cannot follow (a steady push of 15 m/s^2 south, more than the tilt limit gives): e grows to E_MAX
/// and the carrot stops; the vehicle is pushed on, and at the step after e passes X the track fault lands with the
/// braking law, armed with the motors running; never STOP.
#[test]
fn e_a_vehicle_that_cannot_follow_stops_the_carrot_then_lands_braking() {
    let mut r = in_auto();
    let x = hands_off::track_fault_m(r.f.speed_xy_in_force());
    r.push = [-15.0, 0.0];
    let (mut frozen, mut prev, mut e_seen) = (0u32, None::<[f32; 3]>, 0.0f32);
    loop {
        let o = r.fly();
        assert!(o.armed && o.motors.throttles().is_some(), "never a stop");
        if r.state() != State::Auto {
            break;
        }
        let e = r.f.track_error_m.unwrap();
        e_seen = e_seen.max(e);
        if let Setpoint::Position { position_m, .. } = o.setpoint {
            if e >= guidance::LAG_E_MAX_M && prev.is_some_and(|p| p[0] == position_m[0] && p[1] == position_m[1]) {
                frozen += 1;
            }
            prev = Some(position_m);
        }
    }
    std::println!("carrot frozen on {frozen} steps with e >= E_MAX; e max in AUTO {e_seen:.3} m, X {x} m");
    assert!(frozen > 100 && e_seen > x, "{frozen} {e_seen}");
    assert_eq!(r.state(), State::Land);
    assert!(r.f.hands_off.track_fault());
    for _ in 0..800 {
        let o = r.fly();
        assert!(o.armed && o.motors.throttles().is_some());
        assert_eq!(r.state(), State::Land);
        match o.setpoint {
            Setpoint::Position { position_m, velocity_ms, .. } => {
                assert_eq!([position_m[0], position_m[1]], [o.nav.position_m[0], o.nav.position_m[1]], "braking");
                assert_eq!([velocity_ms[0], velocity_ms[1]], [0.0, 0.0]);
            }
            ref s => panic!("{s:?}"),
        }
    }
}

/// (f) T3' lost in AUTO gives LAND after the debounce; the height lost gives BLIND; a stick report gives TAKEOVER; the
/// motors run throughout.
#[test]
fn f_t3_lost_lands_height_lost_blind_and_sticks_take_over() {
    let mut r = in_auto();
    r.f.set_heading_ok(false);
    let mut steps = 0;
    while r.state() == State::Auto {
        assert!(r.fly().armed);
        steps += 1;
    }
    assert_eq!(r.state(), State::Land);
    assert!(steps as u64 * DT_US >= hands_off::TIER_DEBOUNCE_US, "{steps}");
    let mut r = in_auto();
    let mut n = r.cell.get();
    n.valid = super::althold_tests::ANGLE;
    r.cell.set(n);
    while r.state() == State::Auto {
        assert!(r.fly().armed);
    }
    assert_eq!(r.state(), State::Blind);
    let mut r = in_auto();
    let o = r.rc(true, 0.5);
    assert!(o.armed);
    assert_eq!((r.state(), r.f.leg_active()), (State::Takeover, None));
}

// ---- RETURN (A7, H3b-2) ----

/// Height above home (D 0), m.
fn height(r: &Run) -> f32 {
    -r.cell.get().position_m[2]
}

/// (a) Return in AUTO mid-leg at 10 m, return height 20 m: the climb in place to 20 m, the leg home, LAND at home
/// within GDN_RADIUS holding position, the descent at 1.5 m/s above 10 m and 0.5 m/s below, TD stops the motors on the
/// ground; no stop in the air.
#[test]
fn ra_return_in_auto_climbs_in_place_flies_home_lands_and_td_stops() {
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    while r.cell.get().position_m[0] < 12.0 {
        r.fly();
    }
    assert!(libm::fabsf(height(&r) - 10.0) < 0.1);
    let entry = r.cell.get().position_m;
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.legs.phase()), (State::Return, LegPhase::Climb));
    let mut steps = 0u32;
    let mut drift = 0.0f32;
    while r.f.legs.phase() == LegPhase::Climb {
        let o = r.fly();
        steps += 1;
        drift = drift.max(libm::hypotf(o.nav.position_m[0] - entry[0], o.nav.position_m[1] - entry[1]));
        if let Some(h) = heading(&o) {
            assert!(libm::fabsf(h - YAW_RAD) < 1e-5);
        }
    }
    let climb_s = steps as f32 * DT_US as f32 * 1e-6;
    let h_top = height(&r);
    std::println!("climb {climb_s:.2} s to {h_top:.2} m, horizontal drift {drift:.3} m");
    // The drift is the stop from 3 m/s: the carrot stands at the entry point.
    assert!(libm::fabsf(h_top - 20.0) < 0.1 && drift < 1.5, "{h_top} {drift}");
    let mut e_max = 0.0f32;
    while r.state() == State::Return {
        r.fly();
        steps += 1;
        e_max = e_max.max(r.f.track_error_m.unwrap_or(0.0));
        assert!(steps < 200_000);
    }
    let at_land = dist_from(&r, 0.0, 0.0);
    std::println!("LAND {at_land:.2} m from home at {:.2} m after {:.1} s; e max {e_max:.3} m", height(&r), steps as f32 * 1.25e-3);
    assert_eq!(r.state(), State::Land);
    assert!(at_land <= 2.0 && libm::fabsf(height(&r) - 20.0) < 0.2);
    let (mut v_high, mut v_low) = (None, None);
    while r.f.gate.armed() {
        let o = r.fly();
        steps += 1;
        assert!(steps < 400_000, "never stopped");
        let h = height(&r);
        if o.armed {
            assert!(o.motors.throttles().is_some());
        } else {
            assert_eq!(h, 0.0, "a stop in the air");
        }
        if h < 15.0 && v_high.is_none() {
            v_high = Some(r.cell.get().velocity_ms[2]);
        }
        if h < 5.0 && v_low.is_none() {
            v_low = Some(r.cell.get().velocity_ms[2]);
        }
        assert!(matches!(r.state(), State::Land | State::Stop), "{:?}", r.state());
    }
    let (v_high, v_low) = (v_high.unwrap(), v_low.unwrap());
    let total = steps as f32 * DT_US as f32 * 1e-6;
    let touch = dist_from(&r, 0.0, 0.0);
    std::println!("descent {v_high:.3} m/s at 15 m, {v_low:.3} m/s at 5 m; stopped after {total:.1} s, {touch:.2} m from home");
    assert!(libm::fabsf(v_high - LAND_SPEED_HIGH_MS) < 0.05 && libm::fabsf(v_low - LAND_SPEED_MS) < 0.05);
    assert!(touch <= 2.0 && r.state() == State::Stop);
}

/// (b) Already above the return height (return 12 m, the vehicle at 15 m): no climb, the height is kept on the leg home.
#[test]
fn rb_above_the_return_height_no_climb_and_the_height_is_kept() {
    let mut m = square();
    m.return_m = 12.0;
    let mut r = Run::new(3.0, Some(m));
    assert_eq!(r.command(Command::Start), CommandResult::Accepted);
    while r.f.leg_active() != Some(1) || height(&r) < 14.99 {
        r.fly();
    }
    let h0 = height(&r);
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    r.fly();
    assert_eq!(r.f.legs.phase(), LegPhase::HomeLeg, "no climb");
    let (mut lo, mut hi) = (h0, h0);
    while r.state() == State::Return {
        r.fly();
        lo = lo.min(height(&r));
        hi = hi.max(height(&r));
    }
    std::println!("RETURN from {h0:.2} m: height {lo:.3} to {hi:.3} m; LAND {:.2} m from home", dist_from(&r, 0.0, 0.0));
    assert_eq!(r.state(), State::Land);
    assert!(hi - lo < 0.05 && dist_from(&r, 0.0, 0.0) <= 2.0, "{lo} {hi}");
}

/// FX-3 item 1 on the real step: a stored return height of 60 m with GDN_ALT_MAX 40 m in the pushed set: Return at 10 m
/// climbs in place to 40 m, not 60 m.
#[test]
fn fx3_return_climbs_to_gdn_alt_max_not_the_stored_height() {
    let mut m = square();
    m.return_m = 60.0;
    m.alt_max_m = 40.0;
    let mut r = Run::new(3.0, Some(m));
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    assert_eq!(r.f.legs.phase(), LegPhase::Climb);
    let (mut steps, mut top) = (0u32, 0.0f32);
    while r.f.legs.phase() == LegPhase::Climb {
        r.fly();
        steps += 1;
        assert!(steps < 40_000);
        top = top.max(height(&r));
    }
    let carrot = -r.f.legs.down_m();
    // At home already: Home, then LAND; the highest point is the top of the climb.
    for _ in 0..4_000 {
        r.fly();
        top = top.max(height(&r));
    }
    std::println!("FX-3 (1) return 60 m, GDN_ALT_MAX 40 m: carrot {carrot:.2} m, highest {top:.2} m, then {:?}", r.state());
    assert!(libm::fabsf(carrot - 40.0) < 1e-3 && libm::fabsf(top - 40.0) < 0.1, "{carrot} {top}");
}

/// Steps flown until the state leaves `from` (at most `max_s` seconds); the time, s.
fn until_not(r: &mut Run, from: State, max_s: f32) -> f32 {
    let mut steps = 0u32;
    while r.state() == from {
        let o = r.fly();
        assert!(o.armed && o.motors.throttles().is_some(), "a stop in {from:?}");
        steps += 1;
        assert!((steps as f32) * 1.25e-3 < max_s, "still {from:?}");
    }
    steps as f32 * DT_US as f32 * 1e-6
}

/// (c) Link loss (A1, table): sticks and station heartbeat both gone in HOLD: RETURN just past 10 s with home, LAND
/// without. The station heartbeat alone stopping in AUTO: HOLD at 5 s (place kept), RETURN at 10 s (the sticks were
/// lost long before, so the link age is the heartbeat's).
#[test]
fn rc_link_loss_in_hold_returns_with_home_lands_without_and_auto_holds_at_5_s_then_returns_at_10_s() {
    let mut r = Run::new(3.0, Some(square()));
    r.station = false;
    let s = until_not(&mut r, State::Hold, 12.0);
    assert_eq!(r.state(), State::Return);
    assert!(s > 10.0 && s < 10.01, "{s}");
    let mut r = Run::with(3.0, Some(square()), false, true);
    r.station = false;
    let s = until_not(&mut r, State::Hold, 12.0);
    assert_eq!(r.state(), State::Land, "no home");
    assert!(s > 10.0 && s < 10.01, "{s}");
    let mut r = in_auto();
    r.station = false;
    let a = until_not(&mut r, State::Auto, 6.0);
    assert_eq!(r.state(), State::Hold);
    let b = until_not(&mut r, State::Hold, 6.0);
    std::println!("station lost in AUTO: HOLD at {a:.3} s, RETURN at {:.3} s", a + b);
    assert_eq!(r.state(), State::Return);
    assert!(a > 5.0 && a < 5.01 && a + b > 10.0 && a + b < 10.01, "{a} {b}");
}

/// (d) Fresh sticks in RETURN: TAKEOVER; the lock as built (a report with switch6 High opens it, the next not High
/// hands over to PILOT).
#[test]
fn rd_fresh_sticks_in_return_take_over_and_the_lock_hands_over() {
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    for _ in 0..800 {
        r.fly();
    }
    assert_eq!(r.state(), State::Return);
    assert!(r.rc(true, 0.5).armed);
    assert_eq!((r.state(), r.f.legs.phase()), (State::Takeover, LegPhase::Idle));
    r.t += DT_US;
    let t = r.t;
    let report = RcSample { sticks: Sticks { roll: 0.0, pitch: 0.0, throttle: 0.5, yaw: 0.0 }, arm: true, switch6: Switch3::Middle, t_us: t };
    assert!(r.f.step(&input(t, Some(report))).armed);
    assert_eq!(r.state(), State::Pilot);
}

/// (e) In RETURN: a push the plant cannot follow on the leg home, and a push during the climb, each trip the track
/// fault (e past X) and land braking; T3' lost lands; the height lost gives BLIND. Never a stop.
#[test]
fn re_track_fault_lands_braking_t3_lost_lands_height_lost_blind() {
    // Away from home: 15 m north on AUTO's first leg.
    let returning = |climb: bool| {
        let mut r = in_auto();
        while r.cell.get().position_m[0] < 15.0 {
            r.fly();
        }
        assert_eq!(r.command(Command::Return), CommandResult::Accepted);
        if !climb {
            while r.f.legs.phase() == LegPhase::Climb {
                r.fly();
            }
            for _ in 0..1_600 {
                r.fly();
            }
        }
        assert_eq!(r.f.legs.phase(), if climb { LegPhase::Climb } else { LegPhase::HomeLeg });
        r
    };
    for climb in [true, false] {
        let mut r = returning(climb);
        let x = hands_off::track_fault_m(r.f.speed_xy_in_force());
        // Away from home (home is south of the vehicle; north pushes it away).
        r.push = [15.0, 0.0];
        let mut e_max = 0.0f32;
        while r.state() == State::Return {
            assert!(r.fly().armed);
            e_max = e_max.max(r.f.track_error_m.unwrap_or(0.0));
        }
        std::println!("climb {climb}: e max {e_max:.3} m, X {x} m, {:?}", r.f.legs.phase());
        assert_eq!(r.state(), State::Land);
        assert!(r.f.hands_off.track_fault() && e_max > x);
        let o = r.fly();
        assert!(o.armed);
        match o.setpoint {
            Setpoint::Position { position_m, velocity_ms, .. } => {
                assert_eq!([position_m[0], position_m[1], velocity_ms[0], velocity_ms[1]], [o.nav.position_m[0], o.nav.position_m[1], 0.0, 0.0]);
            }
            ref s => panic!("{s:?}"),
        }
    }
    let mut r = returning(false);
    r.f.set_heading_ok(false);
    until_not(&mut r, State::Return, 1.1);
    assert_eq!(r.state(), State::Land);
    let mut r = returning(false);
    let mut n = r.cell.get();
    n.valid = super::althold_tests::ANGLE;
    r.cell.set(n);
    until_not(&mut r, State::Return, 1.1);
    assert_eq!(r.state(), State::Blind);
}

/// (f) Return's answers as built: on USB NotRadioLink; FS_HOLD_EN 0 HandsOffDisabled; no home NoHome; in LAND
/// NotInThisState; in RETURN ACCEPTED (no effect); Start in RETURN NotInThisState.
#[test]
fn rf_return_denied_with_its_reason() {
    let mut r = Run::new(3.0, Some(square()));
    r.f.set_command(StationCommand { command: Command::Return, link: Link::Usb });
    r.fly();
    assert_eq!(r.f.take_command_result(), Some(CommandResult::Denied(DenyReason::NotRadioLink)));
    let mut r = Run::with(3.0, Some(square()), true, false);
    assert_eq!(r.command(Command::Return), CommandResult::Denied(DenyReason::HandsOffDisabled));
    let mut r = Run::with(3.0, Some(square()), false, true);
    assert_eq!(r.command(Command::Return), CommandResult::Denied(DenyReason::NoHome));
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Land), CommandResult::Accepted);
    assert_eq!(r.command(Command::Return), CommandResult::Denied(DenyReason::NotInThisState));
    let mut r = Run::new(3.0, Some(square()));
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    assert_eq!(r.command(Command::Return), CommandResult::Accepted);
    assert_eq!((r.state(), r.f.legs.phase()), (State::Return, LegPhase::Climb));
    assert_eq!(r.command(Command::Start), CommandResult::Denied(DenyReason::NotInThisState));
    assert_eq!(r.state(), State::Return);
}

/// (h) N2: with FS_HOLD_EN 0 a stick loss in the air lands, and with neither sticks nor station for 15 s it never
/// becomes RETURN.
#[test]
fn rh_fs_hold_en_0_stick_loss_lands_never_returns() {
    let mut r = Run::with(3.0, Some(square()), true, false);
    r.station = false;
    for k in 0..12_000 {
        let o = r.fly();
        assert!(o.armed, "step {k}: {:?} {:?} still {} air {}", r.state(), r.cell.get().position_m, r.f.still_us, r.f.airborne);
        assert_eq!(r.state(), State::Land);
    }
}

// ---- BATT-M: the battery monitor's byte on the real step (`Flight::set_battery_cell`, as the board's Stepper) ----

/// The byte core 0 hands over (`batt_fs`): the monitor's `cell` and, when it knows the battery is not critical,
/// `CELL_KNOWN_OK`. One monitor step at `t_us` with a sample of `volts` taken `age_ms` before, armed.
fn monitor_cell(m: &mut fw_core::battery::Monitor, cfg: &fw_core::battery::Config, t_us: u64, volts: Option<f32>, age_ms: u64) -> u8 {
    let t_ms = t_us / 1000 + 10_000;
    let sample = volts.map(|v| fw_core::battery::Sample { volts: v, t_ms: t_ms - age_ms });
    let u = m.step(cfg, t_ms, true, sample);
    u.cell | if m.known_not_critical() { fw_core::battery::CELL_KNOWN_OK } else { 0 }
}

/// Start's battery check from the monitor (decision-mavlink-mode-20261123, A10 "battery not critical", section 50;
/// `battery_ok` = `Monitor::known_not_critical`): 12 s in HOLD with the monitor stepped at 10 Hz, then Start. ACCEPTED
/// with the pack fine or at LOW; BatteryNotOk with the monitor off (no cells, no timer, CRITICAL at or above LOW), with
/// no sample and with a stale one. CRITICAL lands (B1), so Start is not taken.
#[test]
fn batt_m_start_needs_the_battery_known_not_critical() {
    use fw_core::battery::{Config, Level, Monitor};
    let d = Config::DEFAULT;
    let cases: [(&str, Config, Option<f32>, u64, Option<CommandResult>); 9] = [
        ("fine", d, Some(16.0), 0, Some(CommandResult::Accepted)),
        ("LOW", d, Some(14.3), 0, Some(CommandResult::Accepted)),
        ("cells 0", Config { cells: 0, ..d }, Some(16.0), 0, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("timer 0", Config { timer_s: 0, ..d }, Some(16.0), 0, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("CRT = LOW", Config { crt_cell_v: 3.6, ..d }, Some(16.0), 0, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("CRT > LOW", Config { crt_cell_v: 3.7, ..d }, Some(16.0), 0, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("no sample", d, None, 0, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("stale", d, Some(16.0), 5_001, Some(CommandResult::Denied(DenyReason::BatteryNotOk))),
        ("CRITICAL", d, Some(13.9), 0, None),
    ];
    for (name, cfg, volts, age_ms, want) in cases {
        let mut r = Run::new(3.0, Some(square()));
        let mut m = Monitor::new();
        let mut cell = 0;
        for k in 0..9_600u32 {
            if k % 80 == 0 {
                cell = monitor_cell(&mut m, &cfg, r.t, volts, age_ms);
            }
            r.f.set_battery_cell(cell);
            r.fly();
        }
        let level = m.level();
        match want {
            Some(want) => {
                assert_eq!(r.state(), State::Hold, "{name}");
                assert_eq!(level, if name == "LOW" { Level::Low } else { Level::None }, "{name}");
                assert_eq!(r.command(Command::Start), want, "{name}");
            }
            None => {
                assert_eq!((level, r.state()), (Level::Critical, State::Land), "{name}");
                assert_eq!(r.command(Command::Start), CommandResult::Denied(DenyReason::NotInThisState), "{name}");
            }
        }
    }
}

/// B1 in AUTO: CRITICAL from the monitor's byte mid-leg gives LAND at the next step, holding position (the stop from
/// 3 m/s), E2's stages, TD stops the motors on the ground and nothing stops them in the air; with action 0 and 1 every
/// output is bit-identical to the raw input's (the descent request adds nothing, its state Off).
#[test]
fn batt_m_auto_critical_lands_holding_position_and_td_stops() {
    let run = |cell: Option<u8>| {
        let mut r = Run::new(3.0, Some(square()));
        r.f.set_battery_cell(fw_core::battery::CELL_KNOWN_OK);
        assert_eq!(r.command(Command::Start), CommandResult::Accepted);
        while r.cell.get().position_m[0] < 12.0 {
            r.fly();
        }
        assert_eq!(r.state(), State::Auto);
        let entry = r.cell.get().position_m;
        match cell {
            Some(c) => r.f.set_battery_cell(c),
            None => r.f.battery_critical = true,
        }
        let mut out = std::vec::Vec::new();
        let mut drift = 0.0f32;
        let mut steps = 0u32;
        while r.f.gate.armed() {
            let o = r.fly();
            steps += 1;
            assert!(steps < 400_000, "never stopped");
            assert!(matches!(r.state(), State::Land | State::Stop), "{:?}", r.state());
            assert_eq!(r.f.batt_descent(), BattDescent::Off);
            if o.armed {
                assert!(o.motors.throttles().is_some());
            } else {
                assert_eq!(height(&r), 0.0, "a stop in the air");
            }
            drift = drift.max(libm::hypotf(o.nav.position_m[0] - entry[0], o.nav.position_m[1] - entry[1]));
            out.push(std::format!("{o:?}"));
        }
        assert_eq!(r.state(), State::Stop);
        std::println!("BATT-M AUTO -> LAND: drift {drift:.3} m, stopped after {:.1} s", steps as f32 * DT_US as f32 * 1e-6);
        assert!(drift < 1.5, "{drift}");
        out
    };
    let raw = run(None);
    assert_eq!(run(Some((fw_core::battery::Level::Critical as u8))), raw, "action 0");
    assert_eq!(run(Some((fw_core::battery::Level::Critical as u8) | fw_core::battery::CELL_DESCEND)), raw, "action 1");
}
