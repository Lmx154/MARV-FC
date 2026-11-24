//! Host tests of `LegGuidance` (A6, A7; guided-legs L2-L4, L7).

extern crate std;

use super::*;
use std::vec::Vec;

const A: f32 = Tuning::GUIDANCE_ACC_MS2;
const DT: f32 = 0.01;
/// Invented, in an ocean: lat 2.5 deg, lon -140 deg, 40 m.
const ORIGIN: GeoPoint = GeoPoint { lat_e7: 25_000_000, lon_e7: -1_400_000_000, alt_mm: 40_000 };
const HOME: Home = Home { ne: [0.0, 0.0], d: 0.0 };
const R: f32 = 2.0;

/// The point `n_m` north and `e_m` east of `origin` (an f64 inverse written for the tests, not `ned_from_geo`).
fn geo_at(n_m: f64, e_m: f64) -> (i32, i32) {
    let f = f64::from(crate::eskf::WGS84_F);
    let a = f64::from(crate::eskf::WGS84_A_M);
    let e2 = f * (2.0 - f);
    let lat0 = (f64::from(ORIGIN.lat_e7) * 1e-7).to_radians();
    let w = 1.0 - e2 * lat0.sin().powi(2);
    let (rm, rn) = (a * (1.0 - e2) / (w * w.sqrt()), a / w.sqrt());
    (
        ORIGIN.lat_e7 + ((n_m / rm).to_degrees() * 1e7).round() as i32,
        ORIGIN.lon_e7 + ((e_m / (rn * lat0.cos())).to_degrees() * 1e7).round() as i32,
    )
}

/// A mission of (north, east, z above home) waypoints, return height 25 m.
fn mission(wps: &[(f64, f64, f32)], radius_m: f32) -> GdnTargets {
    let mut t = GdnTargets { count: wps.len() as u8, radius_m, targets: [(0, 0); GDN_TARGETS], alts_m: [0.0; GDN_TARGETS], return_m: 25.0, plan_id: 1 };
    for (k, &(n, e, z)) in wps.iter().enumerate() {
        t.targets[k] = geo_at(n, e);
        t.alts_m[k] = z;
    }
    t
}

fn finite(o: &LegOut) {
    let all = o.position_ne.iter().chain(&o.velocity_ne).chain([&o.down_m, &o.velocity_d_ms, &o.e_m]);
    assert!(all.into_iter().all(|x| x.is_finite()), "{o:?}");
}

fn wp(g: &LegGuidance, k: usize) -> [f32; 3] {
    g.wp[k]
}

/// How the test vehicle moves: (position, velocity) from the output.
#[derive(Clone, Copy)]
enum Vehicle {
    /// Exactly at the carrot (horizontal and vertical).
    Follow,
    /// Never moves.
    Still,
    /// First-order lag toward the carrot, time constant 1 s, horizontal; vertical at the carrot.
    Lag,
    /// Point mass: velocity command = carrot velocity + 0.95 /s position error, capped at `cap`, acceleration <= 3 m/s^2.
    PointMass(f32),
}

struct Run {
    outs: Vec<LegOut>,
    /// The vehicle's horizontal distance to each waypoint at its Arrived/Done event.
    arrivals: Vec<(LegEvent, f32)>,
    e_max: f32,
}

/// Steps `g` from `p0` until a terminal event or `max` steps. The lead cap uses `lead_s` 1.0.
fn fly(g: &mut LegGuidance, p0: [f32; 3], veh: Vehicle, max: usize) -> Run {
    let (mut p, mut v) = (p0, [0.0f32; 3]);
    let mut run = Run { outs: Vec::new(), arrivals: Vec::new(), e_max: 0.0 };
    for _ in 0..max {
        let target = if g.phase() == LegPhase::Leg { Some(wp(g, g.idx)) } else { None };
        let lead = g.lead_ok(p[2], 1.0);
        let o = g.step(DT, p, true, lead);
        finite(&o);
        assert!(!o.held);
        run.e_max = run.e_max.max(o.e_m);
        if let (LegEvent::Arrived(_) | LegEvent::Done, Some(t)) = (o.event, target)
            && run.arrivals.last().map(|a| a.0) != Some(LegEvent::Done)
        {
            run.arrivals.push((o.event, hypotf(p[0] - t[0], p[1] - t[1])));
        }
        run.outs.push(o);
        if matches!(o.event, LegEvent::Done | LegEvent::Home) {
            break;
        }
        match veh {
            Vehicle::Follow => p = [o.position_ne[0], o.position_ne[1], o.down_m],
            Vehicle::Still => {}
            Vehicle::Lag => {
                p[0] += (o.position_ne[0] - p[0]) * DT;
                p[1] += (o.position_ne[1] - p[1]) * DT;
                p[2] = o.down_m;
            }
            Vehicle::PointMass(cap) => {
                let mut cmd = [o.velocity_ne[0] + 0.95 * (o.position_ne[0] - p[0]), o.velocity_ne[1] + 0.95 * (o.position_ne[1] - p[1])];
                let sp = hypotf(cmd[0], cmd[1]);
                if sp > cap {
                    cmd = [cmd[0] * cap / sp, cmd[1] * cap / sp];
                }
                let dv = [cmd[0] - v[0], cmd[1] - v[1]];
                let dvn = hypotf(dv[0], dv[1]);
                let lim = A * DT;
                let s = if dvn > lim { lim / dvn } else { 1.0 };
                v = [v[0] + dv[0] * s, v[1] + dv[1] * s, 0.0];
                p = [p[0] + v[0] * DT, p[1] + v[1] * DT, o.down_m];
            }
        }
    }
    run
}

/// L2: the first leg starts at the vehicle, also when the start index is not 0 (not at the previous waypoint).
#[test]
fn first_leg_is_anchored_at_the_vehicle() {
    let m = mission(&[(30.0, 0.0, 10.0), (30.0, 30.0, 10.0), (0.0, 30.0, 10.0)], R);
    for start in [0u8, 1, 2] {
        let mut g = LegGuidance::new();
        assert!(g.enter([5.0, -7.0, -10.0], HOME, &ORIGIN, &m, start, 3.0));
        assert_eq!((g.line.start, g.line.end), ([5.0, -7.0], [wp(&g, start.into())[0], wp(&g, start.into())[1]]));
        let o = g.step(DT, [5.0, -7.0, -10.0], true, true);
        // At rest at the vehicle: the first step moves the carrot by at most a dt^2 / 2.
        assert!(hypotf(o.position_ne[0] - 5.0, o.position_ne[1] + 7.0) <= 0.5 * A * DT * DT + 1e-6, "{o:?}");
        assert_eq!((o.active, o.phase, o.event), (start, LegPhase::Leg, LegEvent::None));
    }
    // A start index that is not a waypoint is refused.
    let mut g = LegGuidance::new();
    assert!(!g.enter([0.0; 3], HOME, &ORIGIN, &m, 3, 3.0));
    assert_eq!(g.step(DT, [0.0; 3], true, true).phase, LegPhase::Idle);
}

/// The trapezoid: acceleration 3 m/s^2, cruise at the speed in force, a stop exactly at the waypoint.
#[test]
fn carrot_speed_profile_accelerates_cruises_and_stops_at_the_waypoint() {
    for cruise in [3.0f32, 5.0] {
        let m = mission(&[(50.0, 0.0, 10.0)], 0.0);
        let mut g = LegGuidance::new();
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, cruise));
        let end = wp(&g, 0);
        let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Follow, 10_000);
        let last = run.outs.last().unwrap();
        assert_eq!(last.event, LegEvent::Done, "{cruise}");
        let (mut prev_v, mut prev_s, mut top) = (0.0f32, 0.0f32, 0.0f32);
        let n = run.outs.len();
        for (i, o) in run.outs.iter().enumerate() {
            // The stop at the end is from within LEG_SNAP_M: at most sqrt(2 a LEG_SNAP_M) + a dt.
            let dv_max = if i + 2 >= n { sqrtf(2.0 * A * LEG_SNAP_M) + A * DT } else { A * DT };
            let along = o.velocity_ne[0] * g.line.dir[0] + o.velocity_ne[1] * g.line.dir[1];
            let s = (o.position_ne[0] - g.line.start[0]) * g.line.dir[0] + (o.position_ne[1] - g.line.start[1]) * g.line.dir[1];
            assert!(along >= 0.0 && along <= cruise + 1e-4, "{along}");
            assert!((along - prev_v).abs() <= dv_max + 1e-4, "step {i} of {n}: dv {}", along - prev_v);
            assert!(s >= prev_s - 1e-5 && s <= g.line.len + 1e-4, "s {s} prev {prev_s}");
            (prev_v, prev_s, top) = (along, s, top.max(along));
        }
        assert!((top - cruise).abs() < 1e-3, "{top}");
        let err = hypotf(last.position_ne[0] - end[0], last.position_ne[1] - end[1]);
        assert!(err < 1e-3, "{err}");
        assert_eq!(last.velocity_ne, [0.0; 2]);
        // Ideal trapezoid time: L / v + v / a.
        let t = run.outs.len() as f32 * DT;
        assert!((t - (50.0 / cruise + cruise / A)).abs() < 0.2, "{cruise}: {t}");
    }
}

/// L4 with the vehicle held still: the carrot stops at E_MAX ahead; e never exceeds E_MAX + v dt.
#[test]
fn lag_rule_stops_the_carrot_at_e_max_from_a_still_vehicle() {
    for cruise in [3.0f32, 5.0] {
        let m = mission(&[(50.0, 0.0, 10.0)], R);
        let mut g = LegGuidance::new();
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, cruise));
        let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Still, 3_000);
        assert!(run.e_max <= LAG_E_MAX_M + cruise * DT, "{}", run.e_max);
        let last = run.outs.last().unwrap();
        assert!(last.e_m > 1.9 && last.e_m <= LAG_E_MAX_M, "{}", last.e_m);
        assert!(hypotf(last.velocity_ne[0], last.velocity_ne[1]) < 0.05, "{last:?}");
    }
}

/// L4 with a first-order lag of 1 s: e stays below 2.0 m on a 50 m leg and around a 90 deg corner, at 3 and 5 m/s.
#[test]
fn lag_rule_keeps_a_lagging_vehicle_within_e_max() {
    for cruise in [3.0f32, 5.0] {
        for wps in [&[(50.0, 0.0, 10.0)][..], &[(50.0, 0.0, 10.0), (50.0, 50.0, 10.0)][..]] {
            let m = mission(wps, R);
            let mut g = LegGuidance::new();
            assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, cruise));
            let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Lag, 20_000);
            assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
            assert!(run.e_max < 2.0, "{cruise} {}: {}", wps.len(), run.e_max);
            std::eprintln!("lag 1 s, {cruise} m/s, {} legs: e max {:.3} m", wps.len(), run.e_max);
        }
    }
}

/// A6 vertical: climb at 3.0 m/s, descent at 1.5 m/s, horizontal and vertical together; arrival only when both are done.
#[test]
fn vertical_carrot_rates_and_arrival_needs_both_axes() {
    // Climb 5 -> 35 m (10 s) on a 10 m leg (about 4.3 s): horizontal first. Then descent 35 -> 32 m (2 s) on a 60 m
    // leg: vertical first.
    let m = mission(&[(10.0, 0.0, 35.0), (70.0, 0.0, 32.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -5.0], Vehicle::Follow, 20_000);
    let o = &run.outs;
    let k_arr = o.iter().position(|x| x.event == LegEvent::Arrived(0)).unwrap();
    // Leg 0: climbing at 3.0 m/s until 35 m; the horizontal carrot was at the waypoint long before.
    assert!((o[10].velocity_d_ms + LEG_CLIMB_MS).abs() < 1e-3, "{:?}", o[10]);
    let k_h = o.iter().position(|x| hypotf(x.position_ne[0] - 10.0, x.position_ne[1]) < 0.05).unwrap();
    assert!(k_h < 500 && k_arr > 990 && k_arr < 1010, "{k_h} {k_arr}");
    assert_eq!(o[k_arr - 1].down_m, -35.0);
    for x in &o[..k_arr] {
        assert!(x.down_m >= -35.0 && x.velocity_d_ms <= 0.0);
    }
    // Leg 1: descent at 1.5 m/s for 2 s, then the horizontal finishes alone; Done only then.
    assert!((o[k_arr + 10].velocity_d_ms - LEG_DESCENT_MS).abs() < 1e-3, "{:?}", o[k_arr + 10]);
    let k_v = k_arr + o[k_arr..].iter().position(|x| x.down_m == -32.0).unwrap();
    assert!((k_v - k_arr) as f32 * DT > 1.9 && ((k_v - k_arr) as f32) * DT < 2.1, "{}", k_v - k_arr);
    let last = o.last().unwrap();
    assert_eq!(last.event, LegEvent::Done);
    assert!((o.len() - k_v) as f32 * DT > 15.0);
    assert_eq!(run.arrivals.len(), 2);
}

/// A6: the vertical carrot stops while the vehicle is outside the lead cap; the horizontal goes on.
#[test]
fn vertical_carrot_stops_without_lead() {
    let m = mission(&[(50.0, 0.0, 30.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    for _ in 0..100 {
        let o = g.step(DT, [0.0, 0.0, -10.0], true, true);
        assert!(o.velocity_d_ms < 0.0);
    }
    let d = g.down_m();
    assert!((d + 13.0).abs() < 1e-3, "{d}");
    // The vehicle stayed at 10 m: 3 m behind is at the cap (3.0 m/s x 1.0 s).
    assert!(!g.lead_ok(-9.9, 1.0) && g.lead_ok(-10.1, 1.0));
    let mut p = [0.0, 0.0, -10.0];
    for _ in 0..100 {
        let o = g.step(DT, p, true, false);
        assert_eq!((o.down_m, o.velocity_d_ms), (d, 0.0));
        p = [o.position_ne[0], o.position_ne[1], -10.0];
    }
    assert!(g.carrot_ne()[0] > 1.0);
    // Descending, the cap is 1.5 m/s x lead.
    let m = mission(&[(50.0, 0.0, 5.0)], R);
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    for _ in 0..50 {
        g.step(DT, [0.0, 0.0, -10.0], true, true);
    }
    assert!(g.lead_ok(-10.0, 1.0) && !g.lead_ok(-10.0, 0.4));
    assert!(!g.lead_ok(f32::NAN, 1.0));
}

/// 16 waypoints run to Done, each Arrived once and in order; the carrot frozen at the last one and its height.
#[test]
fn sixteen_waypoints_run_to_done() {
    let wps: Vec<(f64, f64, f32)> = (0..16).map(|k| (f64::from(k / 4) * 20.0 + 5.0, f64::from(k % 4) * 15.0 - 20.0, 10.0 + (k % 3) as f32)).collect();
    let m = mission(&wps, R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 5.0));
    let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Follow, 100_000);
    let events: Vec<LegEvent> = run.arrivals.iter().map(|a| a.0).collect();
    let mut want: Vec<LegEvent> = (0..15).map(LegEvent::Arrived).collect();
    want.push(LegEvent::Done);
    assert_eq!(events, want);
    let last = *run.outs.last().unwrap();
    let w = wp(&g, 15);
    assert_eq!((last.position_ne, last.down_m, last.velocity_ne, last.velocity_d_ms, last.active), ([w[0], w[1]], w[2], [0.0; 2], 0.0, 15));
    // Done stays Done and frozen wherever the vehicle is.
    for p in [[0.0, 0.0, 0.0], [100.0, -40.0, -3.0]] {
        let o = g.step(DT, p, true, true);
        assert_eq!((o.event, o.phase, o.position_ne, o.down_m), (LegEvent::Done, LegPhase::Done, [w[0], w[1]], w[2]));
    }
}

/// L7: re-entry resumes the active index with a new first leg from the vehicle.
#[test]
fn resume_starts_a_new_first_leg_from_the_vehicle() {
    let m = mission(&[(30.0, 0.0, 10.0), (30.0, 30.0, 10.0), (0.0, 30.0, 10.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let mut p = [0.0, 0.0, -10.0];
    let mut k = 0;
    while g.active() < 1 || k < 200 {
        let o = g.step(DT, p, true, true);
        p = [o.position_ne[0], o.position_ne[1], o.down_m];
        if g.active() == 1 {
            k += 1;
        }
    }
    let kept = g.active();
    assert_eq!(kept, 1);
    // Paused, the vehicle drifted to (20, 5) at 12 m; Start resumes index 1 from there.
    assert!(g.enter([20.0, 5.0, -12.0], HOME, &ORIGIN, &m, kept, 3.0));
    let w1 = wp(&g, 1);
    assert_eq!((g.line.start, g.line.end, g.down_m()), ([20.0, 5.0], [w1[0], w1[1]], -12.0));
    let o = g.step(DT, [20.0, 5.0, -12.0], true, true);
    assert_eq!(o.active, 1);
    assert!(hypotf(o.position_ne[0] - 20.0, o.position_ne[1] - 5.0) < 1e-3);
    let run = fly(&mut g, [20.0, 5.0, -12.0], Vehicle::Follow, 20_000);
    let events: Vec<LegEvent> = run.arrivals.iter().map(|a| a.0).collect();
    assert_eq!(events, [LegEvent::Arrived(1), LegEvent::Done]);
}

/// A7: already above H_r there is no climb; the leg to home starts at once at the present height.
#[test]
fn return_from_above_the_return_height_does_not_climb() {
    let mut g = LegGuidance::new();
    assert!(g.enter_return([40.0, 30.0, -30.0], HOME, 20.0, R, 3.0));
    let o = g.step(DT, [40.0, 30.0, -30.0], true, true);
    assert_eq!((o.phase, o.down_m, o.velocity_d_ms), (LegPhase::HomeLeg, -30.0, 0.0));
    let run = fly(&mut g, [40.0, 30.0, -30.0], Vehicle::Follow, 20_000);
    assert!(run.outs.iter().all(|x| x.down_m == -30.0));
    let last = run.outs.last().unwrap();
    assert_eq!(last.event, LegEvent::Home);
    // Arrival at GDN_RADIUS: the vehicle (= the previous carrot) was within 2.0 m of home, not closer than 1.9 m.
    let prev = run.outs[run.outs.len() - 2].position_ne;
    let dist = hypotf(prev[0], prev[1]);
    assert!(dist <= R && dist > R - 3.0 * DT - 1e-3, "{dist}");
    assert_eq!(last.position_ne, HOME.ne);
    assert_eq!(g.step(DT, [0.0, 0.0, -30.0], true, true).event, LegEvent::Home);
}

/// A7: below H_r it climbs in place at 3.0 m/s first; the horizontal carrot does not move until the vertical is at H_r.
#[test]
fn return_from_below_climbs_in_place_first() {
    let home = Home { ne: [-10.0, 5.0], d: -2.0 };
    let mut g = LegGuidance::new();
    // 5 m above home; return height 20 m: H_r 20 m, D -22.
    assert!(g.enter_return([40.0, 30.0, -7.0], home, 20.0, R, 3.0));
    let run = fly(&mut g, [40.0, 30.0, -7.0], Vehicle::Follow, 20_000);
    let o = &run.outs;
    let k_top = o.iter().position(|x| x.down_m == -22.0).unwrap();
    assert!((k_top as f32 * DT - 5.0).abs() < 0.02, "{k_top}");
    for x in &o[..=k_top] {
        assert_eq!((x.position_ne, x.velocity_ne, x.phase), ([40.0, 30.0], [0.0; 2], LegPhase::Climb));
        assert!((x.velocity_d_ms + LEG_CLIMB_MS).abs() < 1e-3 || x.down_m == -22.0, "{x:?}");
    }
    assert_eq!(o[k_top + 1].phase, LegPhase::HomeLeg);
    assert!(o[k_top + 2].position_ne != [40.0, 30.0]);
    assert!(o[k_top..].iter().all(|x| x.down_m == -22.0));
    assert_eq!(o.last().unwrap().event, LegEvent::Home);
    // A non-finite or lower return height: the present height.
    for ret in [f32::NAN, 1.0] {
        assert!(g.enter_return([40.0, 30.0, -7.0], home, ret, R, 3.0));
        let x = g.step(DT, [40.0, 30.0, -7.0], true, true);
        assert_eq!((x.phase, x.down_m), (LegPhase::HomeLeg, -7.0));
    }
}

/// Two identical waypoints: no division by zero, the zero-length leg is arrived at on the next step.
#[test]
fn zero_length_leg_arrives_at_once() {
    let m = mission(&[(20.0, 0.0, 10.0), (20.0, 0.0, 10.0), (20.0, 20.0, 10.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Follow, 20_000);
    let k0 = run.outs.iter().position(|x| x.event == LegEvent::Arrived(0)).unwrap();
    assert_eq!(run.outs[k0 + 1].event, LegEvent::Arrived(1));
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
    // The vehicle at the waypoint at entry: a zero-length first leg.
    let mut g = LegGuidance::new();
    let w = mission(&[(20.0, 0.0, 10.0)], R);
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &w, 0, 3.0));
    let p = wp(&g, 0);
    assert!(g.enter(p, HOME, &ORIGIN, &w, 0, 3.0));
    assert_eq!(g.line.len, 0.0);
    finite(&g.step(DT, p, true, true));
    assert_eq!(g.step(DT, p, true, true).event, LegEvent::Done);
}

/// A non-finite or invalid position holds the carrot and is flagged; nothing non-finite comes out of any input.
#[test]
fn non_finite_input_holds_the_carrot() {
    let m = mission(&[(50.0, 0.0, 30.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let mut p = [0.0, 0.0, -10.0];
    let mut good = g.step(DT, p, true, true);
    for _ in 0..200 {
        good = g.step(DT, p, true, true);
        p = [good.position_ne[0], good.position_ne[1], good.down_m];
    }
    let bad = [[f32::NAN, 0.0, 0.0], [0.0, f32::INFINITY, 0.0], [0.0, 0.0, f32::NEG_INFINITY]];
    for q in bad {
        let o = g.step(DT, q, true, true);
        finite(&o);
        assert!(o.held);
        assert_eq!((o.position_ne, o.down_m, o.velocity_ne, o.velocity_d_ms, o.e_m), (good.position_ne, good.down_m, [0.0; 2], 0.0, good.e_m));
    }
    let o = g.step(DT, p, false, true);
    assert!(o.held && o.position_ne == good.position_ne);
    // Non-finite or odd dt and lead: finite, no motion for dt NaN.
    for dt in [f32::NAN, f32::INFINITY, -1.0, 0.0] {
        let o = g.step(dt, p, true, true);
        finite(&o);
        assert_eq!((o.position_ne, o.down_m), (good.position_ne, good.down_m));
    }
    let o = g.step(DT, p, true, true);
    assert!(!o.held && o.position_ne[0] > good.position_ne[0]);
    // Refused entries.
    assert!(!g.enter([f32::NAN, 0.0, 0.0], HOME, &ORIGIN, &m, 0, 3.0));
    assert_eq!(g.phase(), LegPhase::Idle);
    assert!(!g.enter_return([0.0, 0.0, 0.0], Home { ne: [0.0, f32::NAN], d: 0.0 }, 20.0, R, 3.0));
    finite(&g.step(DT, [0.0; 3], true, true));
    // Non-finite speed, radius and heights: no motion or a held height, never a NaN.
    let m2 = mission(&[(50.0, 0.0, f32::NAN), (50.0, 50.0, f32::INFINITY)], f32::NAN);
    for speed in [f32::NAN, f32::INFINITY, -2.0, 100.0] {
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m2, 0, speed));
        assert!(g.cruise_ms >= 0.0 && g.cruise_ms <= LEG_SPEED_MAX_MS);
        for _ in 0..500 {
            finite(&g.step(DT, [0.0, 0.0, -10.0], true, true));
        }
        assert_eq!(g.down_m(), -10.0);
    }
    assert!(g.enter_return([0.0, 0.0, -10.0], HOME, f32::INFINITY, f32::NAN, f32::NAN));
    for _ in 0..500 {
        finite(&g.step(DT, [0.0, 0.0, -10.0], true, true));
    }
}

/// Closed loop: a point mass (velocity = feed-forward + 0.95 /s x error, capped at 3 m/s, 3 m/s^2) flies a 30 m square
/// at 3 m/s and reaches every waypoint within GDN_RADIUS.
#[test]
fn closed_loop_square_reaches_every_waypoint() {
    let m = mission(&[(30.0, 0.0, 10.0), (30.0, 30.0, 10.0), (0.0, 30.0, 10.0), (0.0, 0.0, 10.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::PointMass(3.0), 50_000);
    let events: Vec<LegEvent> = run.arrivals.iter().map(|a| a.0).collect();
    assert_eq!(events, [LegEvent::Arrived(0), LegEvent::Arrived(1), LegEvent::Arrived(2), LegEvent::Done]);
    for (ev, d) in &run.arrivals {
        assert!(*d <= R, "{ev:?}: {d}");
    }
    std::eprintln!("closed-loop square 30 m at 3 m/s: e max {:.3} m, {:.1} s", run.e_max, run.outs.len() as f32 * DT);
    assert!(run.e_max <= LAG_E_MAX_M + 3.0 * DT, "{}", run.e_max);
}
