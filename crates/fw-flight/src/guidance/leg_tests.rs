//! Host tests of `LegGuidance`.

extern crate std;

use super::*;
use crate::gnc::{NavState, Setpoint};
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
    let mut t = GdnTargets { count: wps.len() as u8, radius_m, targets: [(0, 0); GDN_TARGETS], alts_m: [0.0; GDN_TARGETS], return_m: 25.0, plan_id: 1, heights_ok: true, alt_max_m: fw_core::mission::ALT_MAX_DEFAULT };
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

/// The first leg starts at the vehicle, also when the start index is not 0 (not at the previous waypoint).
#[test]
fn first_leg_is_anchored_at_the_vehicle() {
    let m = mission(&[(30.0, 0.0, 10.0), (30.0, 30.0, 10.0), (0.0, 30.0, 10.0)], R);
    for start in [0u8, 1, 2] {
        let mut g = LegGuidance::new();
        assert!(g.enter([5.0, -7.0, -10.0], HOME, &ORIGIN, &m, start, 3.0));
        assert_eq!((g.line.start, g.line.end), ([5.0, -7.0], [wp(&g, start.into())[0], wp(&g, start.into())[1]]));
        let o = g.step(DT, [5.0, -7.0, -10.0], true, true);
        assert!(hypotf(o.position_ne[0] - 5.0, o.position_ne[1] + 7.0) <= 0.5 * A * DT * DT + 1e-6, "{o:?}");
        assert_eq!((o.active, o.phase, o.event), (start, LegPhase::Leg, LegEvent::None));
    }
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
        let t = run.outs.len() as f32 * DT;
        assert!((t - (50.0 / cruise + cruise / A)).abs() < 0.2, "{cruise}: {t}");
    }
}

/// With the vehicle held still: the carrot stops at E_MAX ahead; e never exceeds E_MAX + v dt.
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

/// With a first-order lag of 1 s: e stays below 2.0 m on a 50 m leg and around a 90 deg corner, at 3 and 5 m/s.
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

/// Vertical: climb at up to 3.0 m/s, descent at up to 1.5 m/s (ramped at `LEG_ACC_Z_MSS`), horizontal and
/// vertical together; arrival only when both are done.
#[test]
fn vertical_carrot_rates_and_arrival_needs_both_axes() {
    let m = mission(&[(10.0, 0.0, 35.0), (70.0, 0.0, 32.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -5.0], Vehicle::Follow, 20_000);
    let o = &run.outs;
    let k_arr = o.iter().position(|x| x.event == LegEvent::Arrived(0)).unwrap();
    assert!((o[600].velocity_d_ms + LEG_CLIMB_MS).abs() < 1e-3, "{:?}", o[600]);
    let k_h = o.iter().position(|x| hypotf(x.position_ne[0] - 10.0, x.position_ne[1]) < 0.05).unwrap();
    assert!(k_h < 500 && k_arr > 1290 && k_arr < 1310, "{k_h} {k_arr}");
    assert_eq!(o[k_arr - 1].down_m, -35.0);
    for x in &o[..k_arr] {
        assert!(x.down_m >= -35.0 && x.velocity_d_ms <= 0.0);
    }
    assert!((o[k_arr + 170].velocity_d_ms - LEG_DESCENT_MS).abs() < 1e-3, "{:?}", o[k_arr + 170]);
    let k_v = k_arr + o[k_arr..].iter().position(|x| x.down_m == -32.0).unwrap();
    assert!((k_v - k_arr) as f32 * DT > 3.4 && ((k_v - k_arr) as f32) * DT < 3.6, "{}", k_v - k_arr);
    let last = o.last().unwrap();
    assert_eq!(last.event, LegEvent::Done);
    assert!((o.len() - k_v) as f32 * DT > 15.0);
    assert_eq!(run.arrivals.len(), 2);
}

/// The vertical carrot stops while the vehicle is outside the lead cap; the horizontal goes on.
#[test]
fn vertical_carrot_stops_without_lead() {
    let m = mission(&[(50.0, 0.0, 30.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let mut n = 0;
    while g.lead_ok(-10.0, 1.0) {
        let o = g.step(DT, [0.0, 0.0, -10.0], true, true);
        assert!(o.velocity_d_ms < 0.0);
        n += 1;
    }
    assert!((n as f32 * DT - 2.45).abs() < 0.02, "{n}");
    let d = g.down_m();
    assert!((d + 13.0).abs() < 0.05, "{d}");
    assert!(!g.lead_ok(-9.9, 1.0) && g.lead_ok(-10.1, 1.0));
    let mut p = [0.0, 0.0, -10.0];
    for _ in 0..100 {
        let o = g.step(DT, p, true, false);
        assert_eq!((o.down_m, o.velocity_d_ms), (d, 0.0));
        p = [o.position_ne[0], o.position_ne[1], -10.0];
    }
    assert!(g.carrot_ne()[0] > 1.0);
    let o = g.step(DT, p, true, true);
    assert!((o.velocity_d_ms + LEG_ACC_Z_MSS * DT).abs() < 1e-6, "{o:?}");
    let m = mission(&[(50.0, 0.0, 5.0)], R);
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    for _ in 0..150 {
        g.step(DT, [0.0, 0.0, -10.0], true, true);
    }
    assert!(g.lead_ok(-10.0, 1.0) && !g.lead_ok(-10.0, 0.4));
    assert!(!g.lead_ok(f32::NAN, 1.0));
}

/// The vertical carrot's rate per step of `run` against the goal heights `goals` (m down, in order): |d rate| <= a dt
/// (the snap step, from within `LEG_SNAP_M`, at most sqrt(2 a LEG_SNAP_M) + a dt), the climb and descent caps, rate <=
/// sqrt(2 a r) + a dt with r the distance left, every goal reached exactly with rate 0.
fn check_vertical(outs: &[LegOut], goals: &[f32]) -> f32 {
    let (a, eps) = (LEG_ACC_Z_MSS, 1e-4);
    let (mut prev, mut k, mut snap_dv) = (0.0f32, 0usize, 0.0f32);
    for (i, o) in outs.iter().enumerate() {
        let r = o.down_m - goals[k.min(goals.len() - 1)];
        let rate = o.velocity_d_ms;
        let dv = libm::fabsf(rate - prev);
        let at_goal = k < goals.len() && r == 0.0;
        if at_goal && prev != 0.0 {
            snap_dv = snap_dv.max(dv);
            assert!(dv <= sqrtf(2.0 * a * LEG_SNAP_M) + a * DT + eps, "step {i}: snap dv {dv}");
        } else {
            assert!(dv <= a * DT + eps, "step {i}: {prev} -> {rate}");
        }
        assert!(rate <= LEG_DESCENT_MS + eps && rate >= -LEG_CLIMB_MS - eps, "step {i}: {rate}");
        assert!(libm::fabsf(rate) <= sqrtf(2.0 * a * libm::fabsf(r)) + a * DT + eps, "step {i}: rate {rate} r {r}");
        if at_goal {
            assert_eq!(rate, 0.0, "step {i}");
            k += 1;
        }
        prev = rate;
    }
    assert_eq!(k, goals.len(), "goals reached");
    snap_dv
}

/// The vertical carrot is acceleration limited at `LEG_ACC_Z_MSS`, capped at 3.0 up and 1.5 down, brakes into D_k
/// and reaches it exactly at rest: a 30 m climb, a 3 m descent, a 1 m climb (never at the cap) and RETURN's climb.
#[test]
fn vertical_carrot_is_acceleration_limited() {
    let m = mission(&[(10.0, 0.0, 35.0), (70.0, 0.0, 32.0), (70.0, 10.0, 33.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -5.0], Vehicle::Follow, 50_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
    let snap_auto = check_vertical(&run.outs, &[-35.0, -32.0, -33.0]);
    let top = run.outs.iter().map(|o| o.velocity_d_ms).fold(0.0f32, f32::min);
    let bottom = run.outs.iter().map(|o| o.velocity_d_ms).fold(0.0f32, f32::max);
    assert!((top + LEG_CLIMB_MS).abs() < 1e-4 && (bottom - LEG_DESCENT_MS).abs() < 1e-4, "{top} {bottom}");
    let home = Home { ne: [-10.0, 5.0], d: -2.0 };
    assert!(g.enter_return([40.0, 30.0, -7.0], home, 20.0, R, 3.0));
    let run = fly(&mut g, [40.0, 30.0, -7.0], Vehicle::Follow, 20_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Home);
    let snap_ret = check_vertical(&run.outs, &[-22.0]);
    std::eprintln!("vertical carrot: largest snap-step d rate {snap_auto:.4} (AUTO), {snap_ret:.4} (RETURN) m/s");
}

/// The vertical rate is 0 again at `enter` and `enter_return`: the first step after either, entered mid-climb at
/// 3.0 m/s, moves the rate by a dt from rest.
#[test]
fn vertical_rate_restarts_from_rest_at_entry() {
    let m = mission(&[(10.0, 0.0, 35.0)], R);
    for ret in [false, true] {
        let mut g = LegGuidance::new();
        assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
        let mut p = [0.0, 0.0, -5.0];
        for _ in 0..500 {
            let o = g.step(DT, p, true, true);
            p = [o.position_ne[0], o.position_ne[1], o.down_m];
        }
        assert!((g.vd + LEG_CLIMB_MS).abs() < 1e-4, "{}", g.vd);
        if ret {
            assert!(g.enter_return(p, HOME, 40.0, R, 3.0));
        } else {
            assert!(g.enter(p, HOME, &ORIGIN, &m, 0, 3.0));
        }
        assert_eq!(g.vd, 0.0);
        let o = g.step(DT, p, true, true);
        assert!((o.velocity_d_ms + LEG_ACC_Z_MSS * DT).abs() < 1e-6, "{ret}: {o:?}");
    }
}

/// The vertical feed-forward over `outs` (one run from entry): within +-`LEG_ACC_Z_MSS`, the change of the published
/// rate over the step except at a snap to D_k (there the change before the snap, within the bound), 0 at Done and Home.
fn check_accel(outs: &[LegOut]) -> f32 {
    let (a, eps) = (LEG_ACC_Z_MSS, 1e-3);
    let (mut prev, mut naive_snap, mut snaps) = (0.0f32, 0.0f32, 0);
    for (i, o) in outs.iter().enumerate() {
        let naive = (o.velocity_d_ms - prev) / DT;
        assert!(libm::fabsf(o.accel_d_mss) <= a, "step {i}: {}", o.accel_d_mss);
        if matches!(o.event, LegEvent::Done | LegEvent::Home) {
            assert_eq!(o.accel_d_mss, 0.0, "step {i}");
        } else if o.velocity_d_ms == 0.0 && prev != 0.0 {
            snaps += 1;
            naive_snap = naive_snap.max(libm::fabsf(naive));
            assert!(o.accel_d_mss * naive > 0.0, "step {i}: {} {naive}", o.accel_d_mss);
        } else {
            assert!(libm::fabsf(o.accel_d_mss - naive) < eps, "step {i}: {} vs {naive}", o.accel_d_mss);
        }
        prev = o.velocity_d_ms;
    }
    assert!(snaps > 0);
    naive_snap
}

/// The vertical carrot's acceleration is its rate's change before the snap, within +-`LEG_ACC_Z_MSS`, and 0 at Done and
/// Home, over the climbs and the descent of an AUTO run and RETURN's climb.
#[test]
fn vertical_accel_is_the_rate_change_before_the_snap_within_the_bound() {
    let m = mission(&[(10.0, 0.0, 35.0), (70.0, 0.0, 32.0), (70.0, 10.0, 33.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
    let run = fly(&mut g, [0.0, 0.0, -5.0], Vehicle::Follow, 50_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
    let o = &run.outs;
    assert!((o[0].accel_d_mss + LEG_ACC_Z_MSS).abs() < 1e-3, "{:?}", o[0]);
    assert_eq!(o[600].accel_d_mss, 0.0);
    assert!(o.iter().any(|x| (x.accel_d_mss - LEG_ACC_Z_MSS).abs() < 1e-3 && x.velocity_d_ms < 0.0));
    let naive_auto = check_accel(o);
    let home = Home { ne: [-10.0, 5.0], d: -2.0 };
    assert!(g.enter_return([40.0, 30.0, -7.0], home, 20.0, R, 3.0));
    let run = fly(&mut g, [40.0, 30.0, -7.0], Vehicle::Follow, 20_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Home);
    let naive_ret = check_accel(&run.outs);
    std::eprintln!("snap steps: published rate change / dt up to {naive_auto:.2} (AUTO), {naive_ret:.2} (RETURN) m/s^2");
    assert!(naive_auto > 2.0 * LEG_ACC_Z_MSS && naive_ret > 2.0 * LEG_ACC_Z_MSS);
}

/// The acceleration is 0 while the carrot is held (invalid or non-finite position), outside the lead cap and for a dt of
/// 0 or not usable.
#[test]
fn vertical_accel_is_zero_when_held_and_restarts_at_entry() {
    let m = mission(&[(50.0, 0.0, 30.0)], R);
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let mut p = [0.0, 0.0, -10.0];
    let mut o = g.step(DT, p, true, true);
    for _ in 0..150 {
        o = g.step(DT, p, true, true);
        p = [o.position_ne[0], o.position_ne[1], o.down_m];
    }
    assert!((o.accel_d_mss + LEG_ACC_Z_MSS).abs() < 1e-3, "{o:?}");
    let rate = o.velocity_d_ms;
    let stop = g.step(DT, p, true, false);
    assert_eq!((stop.accel_d_mss, stop.velocity_d_ms), (0.0, 0.0));
    assert!(libm::fabsf((stop.velocity_d_ms - rate) / DT) > 100.0 * LEG_ACC_Z_MSS, "control: {rate}");
    for q in [[f32::NAN, 0.0, 0.0], [0.0, 0.0, f32::INFINITY]] {
        assert_eq!(g.step(DT, q, true, true).accel_d_mss, 0.0);
    }
    assert_eq!(g.step(DT, p, false, true).accel_d_mss, 0.0);
    for dt in [0.0, f32::NAN, f32::INFINITY, -1.0] {
        let x = g.step(dt, p, true, true);
        finite(&x);
        assert_eq!(x.accel_d_mss, 0.0, "dt {dt}");
    }
    let m = mission(&[(0.0, 0.0, 14.0)], R);
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 3.0));
    let mut p = [0.0, 0.0, -10.0];
    for _ in 0..300 {
        o = g.step(DT, p, true, true);
        p = [0.0, 0.0, o.down_m];
    }
    assert!((o.accel_d_mss - LEG_ACC_Z_MSS).abs() < 1e-3, "{o:?}");
    let m = mission(&[(0.0, 0.0, 30.0)], R);
    assert!(g.enter(p, HOME, &ORIGIN, &m, 0, 3.0));
    let x = g.step(DT, p, true, true);
    assert!((x.accel_d_mss + LEG_ACC_Z_MSS).abs() < 1e-3, "{x:?}");
    let level = mission(&[(0.0, 0.0, -p[2])], R);
    assert!(g.enter(p, HOME, &ORIGIN, &level, 0, 3.0));
    assert_eq!(g.step(DT, p, true, true).accel_d_mss, 0.0);
    assert!(g.enter_return(p, HOME, 1.0, R, 3.0));
    assert_eq!(g.step(DT, p, true, true).accel_d_mss, 0.0);
}

/// Closed loop: the 25 m Start climb (5 -> 30 m in place) flown by `CascadedPid`'s altitude loop on a level vertical
/// point mass (a = g - thrust / m, 800 Hz).
#[test]
fn closed_loop_vertical_lead_while_braking_shrinks_with_the_accel_feed_forward() {
    use crate::gnc::{Controller, Saturation, Valid};
    use crate::pid::CascadedPid;
    let t = Tuning::DEFAULT;
    let dt = 1.25e-3;
    let fly_climb = |ff: bool| {
        let m = mission(&[(0.0, 0.0, 30.0)], R);
        let mut g = LegGuidance::new();
        assert!(g.enter([0.0, 0.0, -5.0], HOME, &ORIGIN, &m, 0, 3.0));
        let mut pid = CascadedPid::new(&t, 800.0);
        let (mut d, mut vd) = (-5.0f32, 0.0f32);
        let (mut lead, mut over, mut thr) = (0.0f32, 0.0f32, (f32::MAX, 0.0f32));
        let all = Valid { tilt: true, heading: true, rates: true, velocity_ne: true, velocity_d: true, position_ne: true, position_d: true };
        let none = Saturation { roll: false, pitch: false, yaw: false, thrust_low: false, thrust_high: false };
        for k in 0..(25.0 / dt) as u64 {
            let o = g.step(dt, [0.0, 0.0, d], true, g.lead_ok(d, t.pilot_alt_lead_s));
            let sp = Setpoint::Position {
                position_m: [o.position_ne[0], o.position_ne[1], o.down_m],
                velocity_ms: [o.velocity_ne[0], o.velocity_ne[1], o.velocity_d_ms],
                heading_rad: 0.0,
                heading_rate_rads: 0.0,
                accel_d_mss: if ff { o.accel_d_mss } else { 0.0 },
                accel_ne_mss: [0.0; 2],
            };
            let nav = NavState { t_us: k * 1_250, attitude: [1.0, 0.0, 0.0, 0.0], rates_rads: [0.0; 3], velocity_ms: [0.0, 0.0, vd], position_m: [0.0, 0.0, d], valid: all, sigma: None };
            let thrust = pid.update(&nav, &sp, &none).unwrap().thrust_n;
            thr = (thr.0.min(thrust), thr.1.max(thrust));
            if o.accel_d_mss > 0.0 && o.velocity_d_ms < 0.0 {
                lead = lead.max(o.down_m - d);
            }
            over = over.max(-30.0 - d);
            vd += (fw_core::calib::GRAVITY_MSS - thrust / marv_card::MASS) * dt;
            d += vd * dt;
        }
        assert_eq!(g.phase(), LegPhase::Done);
        (lead, over, thr)
    };
    let (lead, over, thr) = fly_climb(true);
    let (lead0, over0, thr0) = fly_climb(false);
    let steady = LEG_ACC_Z_MSS / (t.psc_posz_p * t.psc_velz_p);
    std::eprintln!(
        "25 m climb: lead while braking {lead:.3} m, overshoot {over:.3} m, thrust {:.2}..{:.2} N with the feed-forward; \
         without {lead0:.3} m, {over0:.3} m, {:.2}..{:.2} N (a / (posz_p velz_p) = {steady:.3} m)",
        thr.0, thr.1, thr0.0, thr0.1
    );
    assert!(lead0 > 0.5 * steady && over0 > 0.05, "{lead0} {over0}");
    assert!(lead < 0.25 * lead0 && over < 0.25 * over0, "{lead} {over}");
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
    for p in [[0.0, 0.0, 0.0], [100.0, -40.0, -3.0]] {
        let o = g.step(DT, p, true, true);
        assert_eq!((o.event, o.phase, o.position_ne, o.down_m), (LegEvent::Done, LegPhase::Done, [w[0], w[1]], w[2]));
    }
}

/// Re-entry resumes the active index with a new first leg from the vehicle.
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

/// Already above H_r there is no climb; the leg to home starts at once at the present height.
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
    // Home once the carrot is at rest at home (the step after it got there).
    let prev = run.outs[run.outs.len() - 2];
    assert_eq!((prev.position_ne, prev.velocity_ne, prev.event), (HOME.ne, [0.0; 2], LegEvent::None));
    assert!(run.outs[run.outs.len() - 3].position_ne != HOME.ne);
    assert_eq!(last.position_ne, HOME.ne);
    assert_eq!(g.step(DT, [0.0, 0.0, -30.0], true, true).event, LegEvent::Home);
    // Control: arrival at the radius alone fires with the carrot still moving.
    g.arrive_at_radius = true;
    assert!(g.enter_return([40.0, 30.0, -30.0], HOME, 20.0, R, 3.0));
    let run = fly(&mut g, [40.0, 30.0, -30.0], Vehicle::Follow, 20_000);
    let prev = run.outs[run.outs.len() - 2].position_ne;
    let dist = hypotf(prev[0], prev[1]);
    assert!(dist <= R && dist > R - 3.0 * DT - 1e-3, "{dist}");
}

/// Below H_r it climbs in place at up to 3.0 m/s first (ramped at `LEG_ACC_Z_MSS`); the horizontal carrot
/// does not move until the vertical is at H_r.
#[test]
fn return_from_below_climbs_in_place_first() {
    let home = Home { ne: [-10.0, 5.0], d: -2.0 };
    let mut g = LegGuidance::new();
    assert!(g.enter_return([40.0, 30.0, -7.0], home, 20.0, R, 3.0));
    let run = fly(&mut g, [40.0, 30.0, -7.0], Vehicle::Follow, 20_000);
    let o = &run.outs;
    let k_top = o.iter().position(|x| x.down_m == -22.0).unwrap();
    assert!(((k_top + 1) as f32 * DT - 8.0).abs() < 0.05, "{k_top}");
    assert!((o[400].velocity_d_ms + LEG_CLIMB_MS).abs() < 1e-3, "{:?}", o[400]);
    for x in &o[..=k_top] {
        assert_eq!((x.position_ne, x.velocity_ne, x.phase), ([40.0, 30.0], [0.0; 2], LegPhase::Climb));
        assert!(x.velocity_d_ms <= 0.0 && x.velocity_d_ms >= -LEG_CLIMB_MS - 1e-4, "{x:?}");
    }
    assert_eq!(o[k_top + 1].phase, LegPhase::HomeLeg);
    assert!(o[k_top + 2].position_ne != [40.0, 30.0]);
    assert!(o[k_top..].iter().all(|x| x.down_m == -22.0));
    assert_eq!(o.last().unwrap().event, LegEvent::Home);
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
    for dt in [f32::NAN, f32::INFINITY, -1.0, 0.0] {
        let o = g.step(dt, p, true, true);
        finite(&o);
        assert_eq!((o.position_ne, o.down_m), (good.position_ne, good.down_m));
    }
    let o = g.step(DT, p, true, true);
    assert!(!o.held && o.position_ne[0] > good.position_ne[0]);
    assert!(!g.enter([f32::NAN, 0.0, 0.0], HOME, &ORIGIN, &m, 0, 3.0));
    assert_eq!(g.phase(), LegPhase::Idle);
    assert!(!g.enter_return([0.0, 0.0, 0.0], Home { ne: [0.0, f32::NAN], d: 0.0 }, 20.0, R, 3.0));
    finite(&g.step(DT, [0.0; 3], true, true));
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

/// Field log 346 (M3 480.3-482.7 s): the vehicle passed the carrot at 4.75 m/s; with the lag scale on the unsigned
/// distance the carrot stopped and the vehicle braked and reversed. Ahead of the carrot the scale is 1.
#[test]
fn vehicle_ahead_of_the_carrot_does_not_stall_it() {
    let run = |unsigned: bool| {
        let m = mission(&[(200.0, 0.0, 10.0)], R);
        let mut g = LegGuidance::new();
        g.lag_unsigned = unsigned;
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &m, 0, 5.0));
        let mut o = g.step(DT, [0.0, 0.0, -10.0], true, true);
        while o.velocity_ne[0] < 5.0 {
            o = g.step(DT, [o.position_ne[0], o.position_ne[1], -10.0], true, true);
        }
        // Point mass as in `fly`, 2.5 m ahead of the carrot and 0.5 m to the side, at 4.75 m/s.
        let (mut p, mut v) = ([o.position_ne[0] + 2.5, 0.5], [4.75f32, 0.0]);
        let (mut carrot_min, mut veh_min) = (f32::MAX, f32::MAX);
        for _ in 0..300 {
            let o = g.step(DT, [p[0], p[1], -10.0], true, true);
            if p[0] > o.position_ne[0] {
                carrot_min = carrot_min.min(o.velocity_ne[0]);
            }
            let cmd = [o.velocity_ne[0] + 0.95 * (o.position_ne[0] - p[0]), o.velocity_ne[1] + 0.95 * (o.position_ne[1] - p[1])];
            let dv = [cmd[0] - v[0], cmd[1] - v[1]];
            let s = (A * DT / hypotf(dv[0], dv[1])).min(1.0);
            v = [v[0] + dv[0] * s, v[1] + dv[1] * s];
            p = [p[0] + v[0] * DT, p[1] + v[1] * DT];
            veh_min = veh_min.min(v[0]);
        }
        (carrot_min, veh_min)
    };
    let (carrot, veh) = run(false);
    let (carrot0, veh0) = run(true);
    std::eprintln!("ahead 2.5 m at 4.75 m/s: carrot min {carrot:.2} m/s, vehicle min {veh:.2} m/s; unsigned rule {carrot0:.2}, {veh0:.2}");
    assert!((carrot - 5.0).abs() < 1e-4 && veh > 2.0, "{carrot} {veh}");
    assert!(carrot0 < 0.01 && veh0 <= 0.0, "{carrot0} {veh0}");
}

/// The field rectangle (29 x 19 m, back to the start), heights 10 m.
const RECT: [(f64, f64, f32); 4] = [(29.0, 0.0, 10.0), (29.0, 19.0, 10.0), (0.0, 19.0, 10.0), (0.0, 0.0, 10.0)];

fn dv(a: [f32; 2], b: [f32; 2]) -> [f32; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

/// The horizontal acceleration against the published velocity's own difference: equal off the snap steps, within the
/// bound everywhere, 0 at Done and Home. Returns the largest |difference| / dt at a snap step.
fn check_accel_ne(outs: &[LegOut]) -> f32 {
    let (mut prev, mut naive_snap, mut snaps) = ([0.0f32; 2], 0.0f32, 0);
    for (i, o) in outs.iter().enumerate() {
        let d = dv(o.velocity_ne, prev);
        let naive = [d[0] / DT, d[1] / DT];
        assert!(hypotf(o.accel_ne[0], o.accel_ne[1]) <= A + 1e-4, "step {i}: {:?}", o.accel_ne);
        if matches!(o.event, LegEvent::Done | LegEvent::Home) || o.phase == LegPhase::Climb {
            assert_eq!(o.accel_ne, [0.0; 2], "step {i}");
        } else if o.velocity_ne == [0.0; 2] && prev != [0.0; 2] {
            snaps += 1;
            naive_snap = naive_snap.max(hypotf(naive[0], naive[1]));
            assert!(o.accel_ne[0] * naive[0] + o.accel_ne[1] * naive[1] >= 0.0, "step {i}: {:?} {naive:?}", o.accel_ne);
        } else {
            let e = dv(o.accel_ne, naive);
            assert!(hypotf(e[0], e[1]) < 2e-2, "step {i}: {:?} vs {naive:?}", o.accel_ne);
        }
        prev = o.velocity_ne;
    }
    assert!(snaps > 0);
    naive_snap
}

/// M3 B2: the horizontal carrot's acceleration is its speed's change before the snap, within `GUIDANCE_ACC_MS2`, also
/// on the switch and snap steps, over the rectangle at 5 and 3 m/s and RETURN's leg.
#[test]
fn horizontal_accel_is_the_speed_change_before_the_snap_within_the_bound() {
    let mut naive = std::vec::Vec::new();
    for cruise in [5.0f32, 3.0] {
        let mut g = LegGuidance::new();
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &mission(&RECT, R), 0, cruise));
        let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::Follow, 50_000);
        assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
        assert!((run.outs[0].accel_ne[0] - A).abs() < 1e-3, "{:?}", run.outs[0]);
        naive.push(check_accel_ne(&run.outs));
    }
    let home = Home { ne: [-10.0, 5.0], d: -2.0 };
    let mut g = LegGuidance::new();
    assert!(g.enter_return([40.0, 30.0, -7.0], home, 20.0, R, 5.0));
    let run = fly(&mut g, [40.0, 30.0, -7.0], Vehicle::Follow, 20_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Home);
    naive.push(check_accel_ne(&run.outs));
    std::eprintln!("snap steps: published velocity change / dt up to {naive:.2?} m/s^2 (5, 3 m/s, RETURN)");
    assert!(naive.iter().all(|&x| x > A), "control: {naive:?}");
}

/// The horizontal acceleration is 0 while held, for a dt of 0 or not usable, at Done and in RETURN's climb.
#[test]
fn horizontal_accel_is_zero_when_held_done_climbing_or_dt_unusable() {
    let mut g = LegGuidance::new();
    assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &mission(&[(50.0, 0.0, 10.0)], R), 0, 5.0));
    let mut p = [0.0, 0.0, -10.0];
    let mut o = g.step(DT, p, true, true);
    for _ in 0..50 {
        o = g.step(DT, p, true, true);
        p = [o.position_ne[0], o.position_ne[1], o.down_m];
    }
    assert!((o.accel_ne[0] - A).abs() < 1e-3, "control: {o:?}");
    for q in [[f32::NAN, 0.0, 0.0], [0.0, f32::INFINITY, 0.0]] {
        assert_eq!(g.step(DT, q, true, true).accel_ne, [0.0; 2]);
    }
    assert_eq!(g.step(DT, p, false, true).accel_ne, [0.0; 2]);
    for dt in [0.0, f32::NAN, f32::INFINITY, -1.0] {
        let x = g.step(dt, p, true, true);
        finite(&x);
        assert_eq!(x.accel_ne, [0.0; 2], "dt {dt}");
    }
    assert!((g.step(DT, p, true, true).accel_ne[0] - A).abs() < 1e-3);
    let run = fly(&mut g, p, Vehicle::Follow, 20_000);
    assert_eq!(run.outs.last().unwrap().event, LegEvent::Done);
    assert_eq!(g.step(DT, [10.0, 3.0, -10.0], true, true).accel_ne, [0.0; 2]);
    assert!(g.enter_return([40.0, 30.0, -7.0], HOME, 20.0, R, 5.0));
    for _ in 0..100 {
        let x = g.step(DT, [40.0, 30.0, -7.0], true, true);
        assert_eq!((x.phase, x.accel_ne), (LegPhase::Climb, [0.0; 2]));
    }
}

/// M3 B1: Arrived, Done and Home only with the carrot at rest at the waypoint (the step before published it there with
/// velocity 0), so the switch changes the published velocity by at most a dt; the point mass of `fly` at 5 and 3 m/s
/// over the rectangle and RETURN. Control: arrival at the radius alone switches with the carrot moving.
#[test]
fn arrival_needs_the_carrot_at_rest_and_the_switch_keeps_the_velocity_continuous() {
    let switches = |at_radius: bool| {
        let mut out = std::vec::Vec::new();
        for cruise in [5.0f32, 3.0] {
            let mut g = LegGuidance::new();
            g.arrive_at_radius = at_radius;
            assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &mission(&RECT, R), 0, cruise));
            let ends: Vec<[f32; 2]> = (0..4).map(|k| [wp(&g, k)[0], wp(&g, k)[1]]).collect();
            let run = fly(&mut g, [0.0, 0.0, -10.0], Vehicle::PointMass(cruise), 50_000);
            let ev: Vec<LegEvent> = run.arrivals.iter().map(|a| a.0).collect();
            assert_eq!(ev, [LegEvent::Arrived(0), LegEvent::Arrived(1), LegEvent::Arrived(2), LegEvent::Done]);
            assert!(run.arrivals.iter().all(|a| a.1 <= R), "{cruise}: {:?}", run.arrivals);
            for (i, o) in run.outs.iter().enumerate().skip(1) {
                let k = match o.event {
                    LegEvent::Arrived(k) => Some(usize::from(k)),
                    LegEvent::Done => Some(3),
                    _ => None,
                };
                if let Some(k) = k {
                    let prev = run.outs[i - 1];
                    out.push((prev.position_ne == ends[k], hypotf(prev.velocity_ne[0], prev.velocity_ne[1]), hypotf(dv(o.velocity_ne, prev.velocity_ne)[0], dv(o.velocity_ne, prev.velocity_ne)[1])));
                }
            }
            let mut g2 = LegGuidance::new();
            g2.arrive_at_radius = at_radius;
            assert!(g2.enter_return([25.0, 15.0, -30.0], HOME, 20.0, R, cruise));
            let run = fly(&mut g2, [25.0, 15.0, -30.0], Vehicle::PointMass(cruise), 50_000);
            let i = run.outs.len() - 1;
            assert_eq!(run.outs[i].event, LegEvent::Home);
            let prev = run.outs[i - 1];
            out.push((prev.position_ne == HOME.ne, hypotf(prev.velocity_ne[0], prev.velocity_ne[1]), 0.0));
        }
        out
    };
    let gated = switches(false);
    for &(at, v, step) in &gated {
        assert!(at && v == 0.0 && step <= A * DT + 1e-5, "{gated:?}");
    }
    let radius = switches(true);
    std::eprintln!("switches (carrot at the waypoint, its speed, velocity step): gated {gated:.3?}; at the radius {radius:.3?}");
    assert!(radius.iter().all(|&(at, v, _)| !at && v > 0.5), "control: {radius:?}");
    assert!(radius.iter().any(|&(_, _, step)| step > 10.0 * A * DT), "control: {radius:?}");
}

/// Closed loop, M3: the rectangle flown by `CascadedPid`'s horizontal loop on a point mass that holds the attitude
/// setpoint exactly (a = g tan(tilt), 800 Hz), at 5 and 3 m/s. Per corner: overshoot past the waypoint along the
/// incoming leg (the last one after Done for 5 s) and the peak tilt. Controls: arrival at the radius without the
/// acceleration feed-forward (before M3), and the gate alone (B1).
#[test]
fn closed_loop_corner_overshoot_drops_with_the_arrival_gate_and_the_accel_feed_forward() {
    use crate::gnc::{Controller, Saturation, Valid};
    use crate::pid::CascadedPid;
    let t = Tuning::DEFAULT;
    let dt = 1.25e-3;
    let fly_rect = |cruise: f32, gate: bool, ff: bool| {
        let mut g = LegGuidance::new();
        g.arrive_at_radius = !gate;
        assert!(g.enter([0.0, 0.0, -10.0], HOME, &ORIGIN, &mission(&RECT, R), 0, cruise));
        let ends: Vec<[f32; 2]> = (0..4).map(|k| [wp(&g, k)[0], wp(&g, k)[1]]).collect();
        let dirs: Vec<[f32; 2]> = (0..4)
            .map(|k| {
                let a = if k == 0 { [0.0, 0.0] } else { ends[k - 1] };
                let l = hypotf(ends[k][0] - a[0], ends[k][1] - a[1]);
                [(ends[k][0] - a[0]) / l, (ends[k][1] - a[1]) / l]
            })
            .collect();
        let mut pid = CascadedPid::new(&t, 800.0);
        pid.set_speed_limit_xy(crate::hands_off::speed_limit_xy_ms(cruise));
        let all = Valid { tilt: true, heading: true, rates: true, velocity_ne: true, velocity_d: true, position_ne: true, position_d: true };
        let none = Saturation { roll: false, pitch: false, yaw: false, thrust_low: false, thrust_high: false };
        let (mut p, mut v) = ([0.0f32; 2], [0.0f32; 2]);
        let (mut over, mut tilt) = ([f32::MIN; 4], 0.0f32);
        let mut done_steps = 0;
        for k in 0..(120.0 / dt) as u64 {
            let o = g.step(dt, [p[0], p[1], -10.0], true, true);
            let sp = Setpoint::Position {
                position_m: [o.position_ne[0], o.position_ne[1], o.down_m],
                velocity_ms: [o.velocity_ne[0], o.velocity_ne[1], o.velocity_d_ms],
                heading_rad: 0.0,
                heading_rate_rads: 0.0,
                accel_d_mss: 0.0,
                accel_ne_mss: if ff { o.accel_ne } else { [0.0; 2] },
            };
            let nav = NavState { t_us: k * 1_250, attitude: [1.0, 0.0, 0.0, 0.0], rates_rads: [0.0; 3], velocity_ms: [v[0], v[1], 0.0], position_m: [p[0], p[1], -10.0], valid: all, sigma: None };
            pid.update(&nav, &sp, &none).unwrap();
            let [w, x, y, z] = pid.attitude_setpoint();
            let col = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
            tilt = tilt.max(libm::acosf(col[2].min(1.0)).to_degrees());
            let a = [-fw_core::calib::GRAVITY_MSS * col[0] / col[2], -fw_core::calib::GRAVITY_MSS * col[1] / col[2]];
            v = [v[0] + a[0] * dt, v[1] + a[1] * dt];
            p = [p[0] + v[0] * dt, p[1] + v[1] * dt];
            let active = usize::from(o.active);
            for c in 0..4 {
                if c == active || (c + 1 == active && o.phase == LegPhase::Leg) || (c == 3 && o.phase == LegPhase::Done) {
                    over[c] = over[c].max((p[0] - ends[c][0]) * dirs[c][0] + (p[1] - ends[c][1]) * dirs[c][1]);
                }
            }
            if o.phase == LegPhase::Done {
                done_steps += 1;
                if done_steps as f32 * dt > 5.0 {
                    break;
                }
            }
        }
        assert_eq!(g.phase(), LegPhase::Done, "{cruise} {gate} {ff}");
        (over, tilt)
    };
    for cruise in [5.0f32, 3.0] {
        let (o0, t0) = fly_rect(cruise, false, false);
        let (o1, t1) = fly_rect(cruise, true, false);
        let (o2, t2) = fly_rect(cruise, true, true);
        let worst = |o: [f32; 4]| o.iter().fold(f32::MIN, |m, &x| m.max(x));
        std::eprintln!(
            "rectangle at {cruise} m/s, overshoot per waypoint m / peak tilt deg: before M3 {o0:.2?} {t0:.1}; B1 {o1:.2?} {t1:.1}; B1+B2 {o2:.2?} {t2:.1}"
        );
        assert!(worst(o0) > 0.1 && worst(o1) > worst(o0), "control: {o0:?} {o1:?}");
        assert!(worst(o2) < 0.5 * worst(o0), "{o2:?} against {o0:?}");
        // The feed-forward alone tilts by atan(a / g); at 5 m/s the corners tilted more before M3.
        assert!(t2 < libm::atanf(A / fw_core::calib::GRAVITY_MSS).to_degrees() + 1.0 && (cruise < 5.0 || t2 < t0), "{t2} against {t0}");
    }
}
