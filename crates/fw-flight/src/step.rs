//! The flight step: one call per IMU1 sample, on core 1 (decision-sim-seams-20261119, sections 7 and 11).
//! [`Flight::step`] runs the stages in the order of rule R1, each exactly once. No clock: the step time is
//! `sensors.imu1.t_us`.

use crate::alloc::CardAllocator;
use crate::gate::{Gate, MotorCommand};
use crate::hands_off::{self, Demand, HandsOff, LandLaw, MissionFacts, State, Tier};
use crate::eskf::ned_from_geo;
use crate::guidance::{self, LegGuidance, LegPhase};
use crate::gnc::{
    Allocation, Allocator, Controller, Estimator, GeoPoint, Mode, NavState, RateLog, Refused, Saturation,
    Sensors, Setpoint, ThrustTorque, Valid,
};
use crate::mahony::DT_MAX_US;
use crate::quat;
use crate::tuning::{Gains, PILOT_SPEED_XY_MAX_MS, PILOT_SPEED_XY_MIN_MS, Tuning, full_thrust_n};
use fw_core::calib::GRAVITY_MSS;
use fw_core::dflog::{self, FLIGHT_TYPES};
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::mission::{GDN_TARGETS, GdnTargets};
use fw_core::rc::{RcSample, Sticks, Switch3};
use marv_card::MASS;
#[cfg(test)]
use marv_card::{K_F, MAX_ROTOR_SPEED};

/// One step's inputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightInputs {
    /// What is new since the previous step.
    pub sensors: Sensors,
    /// This step's new stick report, if one arrived.
    pub rc: Option<RcSample>,
    /// The latest eRPM of each ESC and the board time it was captured, us, by DShot line (`EscLine::erpm`); `None` for a
    /// line without one, and for every line when the ESC status is not available. A value that is `notch_hold_us` or more
    /// old at the step time counts as missing.
    pub erpm: [Option<(u32, u64)>; LINES],
}

/// Everything one step decided, by value: the ESC command plus what the log records (firmware change 5).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepOut {
    /// The step time, `sensors.imu1.t_us`.
    pub t_us: u64,
    /// The gate's command.
    pub motors: MotorCommand,
    /// The gate's state after this step.
    pub armed: bool,
    /// The mode the pilot selected with the switch on channel 6 (Angle until the first stick report).
    pub selected: Mode,
    /// The mode in effect this step (decision-nav-eskf-20261120, 5e, F1; Amendment P, P3): Position if `selected` is
    /// Position, the estimate covers what `Setpoint::Position` needs and the airborne latch is set; else Altitude if `selected` is Altitude or Position and the estimate covers what
    /// `Setpoint::Altitude` needs; otherwise Angle. MOT's `Mode`.
    pub mode: Mode,
    /// The estimator's state.
    pub nav: NavState,
    /// The stick shaper's setpoint.
    pub setpoint: Setpoint,
    /// The controller's demand, or `Err` when it refused or was not called.
    pub demand: Result<ThrustTorque, Refused>,
    /// `None` when the demand was refused.
    pub allocation: Option<Allocation>,
    /// The attitude setpoint [w, x, y, z] the controller made from a `Setpoint::Position` this step (SETP kind 3); [0; 4]
    /// in every other case.
    pub attitude_sp: [f32; 4],
    /// The rate loop of this step (PIDT and NTCH records): zero except the rotor frequencies and the notch switch when the
    /// demand was refused.
    pub rate: RateLog,
}

/// What the low-battery descent (Amendment BATT, `BATT_FS_CRT_ACT` 1) is doing, by value so core 0 can raise the texts
/// (the flight core has none): [`Flight::batt_descent`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BattDescent {
    /// Not asked for, or the arm state says nothing.
    Off = 0,
    /// Asked for, not cancelled, and the estimate covers Altitude: the throttle stick reads 0.0.
    Descending = 1,
    /// Asked for, and a change of the mode switch cancelled it until disarm.
    Cancelled = 2,
    /// Asked for, but the estimate does not cover Altitude: Angle, the pilot's throttle.
    NoHeight = 3,
}

/// The default of the parameter `MOT_POLES`: poles, not pole pairs (`fw_settings::esc`).
const NOTCH_POLES_DEFAULT: i32 = 14;

/// The range of the parameter `MOT_POLES` (`fw_settings::esc`; the flight code checks it too, as for `PILOT_SPEED_XY`).
const NOTCH_POLES_RANGE: core::ops::RangeInclusive<i32> = 2..=64;

/// The throttle stick's centre in altitude hold: the dead zone is this plus and minus `pilot_thr_dz`
/// (ArduPilot `Attitude.cpp:97-112`, the centre fixed at half travel).
const THROTTLE_CENTRE: f32 = 0.5;

/// LAND's descent, m/s down: the height target integrates at this rate (decision-mavlink-mode-20261123, A8 and Numbers
/// "land 0.5 m/s", ArduPilot LAND_SPD_MS_DEFAULT, config.h:335-336).
pub const LAND_SPEED_MS: f32 = 0.5;

/// E2's first stage, m/s down: LAND and the TAKEOVER descent above `LAND_ALT_LOW_M` over home (decision-mavlink-mode-20261123,
/// E2, approved by the owner in section 17: the approved descent `pilot_speed_dn_ms`, tuning.rs; ArduPilot
/// LAND_SPD_HIGH_MS, mode_land.cpp:22, mode.cpp:719-727; PX4 MPC_Z_V_AUTO_DN 1.5).
pub const LAND_SPEED_HIGH_MS: f32 = 1.5;

/// E2: the height above home from which down LAND descends at `LAND_SPEED_MS`, m (ArduPilot LAND_ALT_LOW_M 10,
/// mode_land.cpp:31; PX4 MPC_LAND_ALT1 10, multicopter_takeoff_land_params.yaml). With no home, `LAND_SPEED_MS` all the
/// way (E2).
pub const LAND_ALT_LOW_M: f32 = 10.0;

/// E2: how fast LAND's descent rate changes from one stage to the other, m/s^2, so the change steps neither the velocity
/// setpoint nor the thrust (ArduPilot WP_ACC_Z default 1.0, AC_WPNav.cpp:14, the vertical acceleration LAND sets,
/// mode_land.cpp:77).
pub const LAND_RATE_ACC_MSS: f32 = 1.0;

/// E2: the first stage ends this far above `LAND_ALT_LOW_M`, m: the height a descent that tracks the rate covers while
/// `LAND_RATE_ACC_MSS` takes it from `LAND_SPEED_HIGH_MS` to `LAND_SPEED_MS`, so it is at `LAND_SPEED_MS` from
/// `LAND_ALT_LOW_M` down (ArduPilot likewise slows above LAND_ALT_LOW_M, mode.cpp:732-737).
const LAND_SLOW_MARGIN_M: f32 =
    (LAND_SPEED_HIGH_MS * LAND_SPEED_HIGH_MS - LAND_SPEED_MS * LAND_SPEED_MS) / (2.0 * LAND_RATE_ACC_MSS);

/// FX-2 (3): RETURN reports Home only below this vertical speed, m/s, so LAND's first rate (N11 (2), v_D) is within
/// `LAND_SPEED_MS` of rest and E2's first stage is reached within about 2 s at `LAND_RATE_ACC_MSS`.
const HOME_VD_MAX_MS: f32 = LAND_SPEED_MS;

/// FX-4 (1): RETURN reports Home this long after the guidance first reported it in this RETURN, us, whatever v_D reads:
/// the time LAND's own rate ramp needs to absorb the largest climb rate (`LEG_CLIMB_MS` / `LAND_RATE_ACC_MSS`).
const HOME_WAIT_US: u64 = (guidance::LEG_CLIMB_MS / LAND_RATE_ACC_MSS * 1e6) as u64;

/// BLIND's collective below hover, m/s^2 (decision-mavlink-mode-20261123, Numbers "BLIND 0.3 m/s^2 below hover", PX4
/// FlightTaskDescend.cpp:55-58). UNVERIFIED: the decision marks it UNKNOWN (U-1) and for the simulator only.
pub const BLIND_BELOW_HOVER_MSS: f32 = 0.3;

/// BLIND's synthetic throttle: Angle's collective at it is m (g - `BLIND_BELOW_HOVER_MSS`), "hover" being the card's
/// nominal m g, the collective Angle gives at half throttle (there is no height estimate to find the real hover). Only
/// before H is valid: then H - m `BLIND_BELOW_HOVER_MSS` (N11 (1c); N16 (b) uses H for every synthetic Angle throttle).
const BLIND_THROTTLE: f32 = THROTTLE_CENTRE * (1.0 - BLIND_BELOW_HOVER_MSS / GRAVITY_MSS);

/// N11 (1a): the learned hover collective's low-pass time constant, s (ArduPilot AP_MOTORS_THST_HOVER_TC 10,
/// AP_MotorsMulticopter.h:14-17, called from ArduCopter Attitude.cpp:62).
const HOVER_TC_S: f32 = 10.0;

/// N11 (1a): the learner runs only below this vertical speed, m/s (ArduCopter Attitude.cpp:59, 60 cm/s).
const HOVER_VD_MAX_MS: f32 = 0.6;

/// N11 (1a): tan 5 deg and sin 5 deg, the roll and pitch bounds of the learner (ArduCopter Attitude.cpp:59-60, 5 deg each).
const HOVER_TAN_ROLL_MAX: f32 = 0.087_488_66;
const HOVER_SIN_PITCH_MAX: f32 = 0.087_155_74;

/// N11 (1a), N14 (2): the learned hover collective's range, x the card's m g: the hover collective the real thrust gain
/// range allows, [1 / `GAIN_MAX`, 1 / `GAIN_MIN`] = [0.685, 1.61].
const HOVER_MIN_G: f32 = 1.0 / GAIN_MAX;
const HOVER_MAX_G: f32 = 1.0 / GAIN_MIN;

/// N14 (1), (2): the strongest and the weakest real thrust gain against the card: the plant spread +-30 % (tuning.rs
/// `ground_thrust_g`) times the measured gain range +-12 % (tune1 section 12), 1.30 x 1.12 = 1.456 and 0.70 x 0.88 =
/// 0.616, as N14 rounds them.
const GAIN_MAX: f32 = 1.46;
const GAIN_MIN: f32 = 0.62;

/// N14 (1): "ever airborne" latches when the commanded collective first reaches this x the card's m g after arming:
/// ArduPilot's hover/2 (ArduCopter Attitude.cpp:120) at the strongest end of the spread, 0.5 / `GAIN_MAX` = 0.34.
pub const EVER_AIRBORNE_G: f32 = 0.5 / GAIN_MAX;

/// N14 (3): the card check reports H when it differs from the card's m g by more than this fraction: the measured thrust
/// gain spread, +-12 % (tune1 section 12).
pub const CARD_CHECK_FRAC: f32 = 0.12;

/// N11 (1a): the learned hover collective is used after this much qualifying time since arming, us.
const HOVER_VALID_US: u64 = 10_000_000;

/// The touchdown backstop's acceleration filter, Hz (decision-mavlink-mode-20261123, E3: ArduPilot
/// LAND_DETECTOR_ACCEL_LPF_CUTOFF 1.0, config.h:359-372).
const TOUCHDOWN_ACCEL_LPF_HZ: f32 = 1.0;

/// The touchdown backstop's largest filtered earth-frame acceleration, gravity removed, m/s^2 (E3: ArduPilot
/// LAND_DETECTOR_ACCEL_MAX 1.0, config.h:359-372, land_detector.cpp:127).
const TOUCHDOWN_ACCEL_MAX_MSS: f32 = 1.0;

/// The touchdown rule's largest roll rate and largest pitch rate (each axis, the estimate's body rates), rad/s
/// (decision-mavlink-mode-20261123, Amendment N, N4 (c): PX4 LNDMC_ROT_MAX 20 deg/s, land_detector_params_mc.yaml;
/// approved by the owner 2026-11-24).
const TOUCHDOWN_ROT_MAX_RADS: f32 = 20.0 * (core::f32::consts::PI / 180.0);

/// BLIND's probe (Amendment N, N4; N14 (1)): the time over which its collective ramps linearly from BLIND's down to the
/// allocator's idle floor, us. No outside source; approved by the owner 2026-11-24.
pub const BLIND_PROBE_RAMP_US: u64 = 1_000_000;

/// N13: BLIND's one-shot correction is made this long after BLIND's entry, us, from the mean net vertical acceleration
/// over that time. Approved by the owner 2026-11-24, no outside source.
pub const BLIND_CORRECT_US: u64 = 1_000_000;

/// Crash rule K (decision-mavlink-mode-20261123, Amendment N, N5, shadow): K would fire when its four terms have held
/// together this long, us (ArduPilot CRASH_CHECK_TRIGGER_SEC 2, ArduCopter crash_check.cpp:4).
pub const CRASH_TRIGGER_US: u64 = 2_000_000;

/// K's lean term: the tilt from vertical over 15 deg, as its cosine (ArduPilot CRASH_CHECK_ANGLE_MIN_DEG 15,
/// crash_check.cpp:6).
const CRASH_COS_LEAN_MIN: f32 = 0.965_925_8;

/// K's error term: the angle between the thrust axis of the attitude setpoint and that of the estimate over 30 deg, as its
/// cosine (ArduPilot CRASH_CHECK_ANGLE_DEVIATION_DEG 30, crash_check.cpp:5; `get_att_error_angle_deg` is the thrust
/// error angle, AC_AttitudeControl.h:345).
const CRASH_COS_ERROR_MIN: f32 = 0.866_025_4;

/// K's acceleration term: the 1 Hz filtered earth-frame acceleration (gravity removed) under this, m/s^2 (ArduPilot
/// CRASH_CHECK_ACCEL_MAX 3.0, crash_check.cpp:8).
const CRASH_ACCEL_MAX_MSS: f32 = 3.0;

/// K's speed term: |v| under this when the velocity is valid, m/s (ArduPilot CRASH_CHECK_SPEED_MAX 10, crash_check.cpp:7).
const CRASH_SPEED_MAX_MS: f32 = 10.0;

/// Crash rule K's state after a step (Amendment N, N5, shadow: it reaches no output, the gate, the latch or the state
/// machine). The four terms are false on a step K does not watch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrashStatus {
    /// The lean over 15 deg.
    pub lean: bool,
    /// The setpoint's thrust axis more than 30 deg from the estimate's.
    pub error: bool,
    /// The filtered acceleration under 3 m/s^2.
    pub accel: bool,
    /// |v| under 10 m/s, or the velocity not valid.
    pub speed: bool,
    /// How long the four have held together, us.
    pub held_us: u64,
}

impl CrashStatus {
    /// K would fire: the four terms have held together for `CRASH_TRIGGER_US`.
    pub fn would_fire(&self) -> bool {
        self.held_us >= CRASH_TRIGGER_US
    }
}

/// The mission facts of a set that is not the uploaded mission (`GDN_SRC` 0: the `GDN_*` parameters have no heights,
/// section 36): none, so Start is DENIED NoWaypoints.
const NO_MISSION: MissionFacts = MissionFacts {
    plan_id: None,
    has_waypoints: false,
    mission_stored: false,
    z_within_limits: true,
    max_waypoint_distance_m: None,
    return_height_m: None,
    alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
};

/// The target set before the first `set_mission`: off, not the mission; GDN_RADIUS's default (the board's `GDN_OFF`), so
/// a RETURN before any push still arrives.
const MISSION_OFF: GdnTargets = GdnTargets {
    count: 0,
    radius_m: 2.0,
    targets: [(0, 0); GDN_TARGETS],
    alts_m: [0.0; GDN_TARGETS],
    return_m: 0.0,
    plan_id: 0,
    from_mission: false,
    heights_ok: false,
    alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
    mission_stored: false,
};

/// The state machine's mission facts from `set`, the target set in force (Amendment A, A4: list, heights, return height
/// and plan id of one push). Not the mission (`GDN_SRC` 0): [`NO_MISSION`] with the plan id of the station feed
/// (`station_plan`). The mission: its plan id, waypoints and height check (core 0's, against `GDN_ALT_MIN`/`GDN_ALT_MAX`
/// when pushed), the return height, and, only when `start` (a Start command at this step: A9 is checked at every Start
/// and nowhere else), the largest horizontal distance of a waypoint from `home`, converted at `origin` as the guidance
/// converts them; `None` without home or origin.
fn mission_facts(
    set: &GdnTargets,
    station_plan: Option<u32>,
    home: Option<hands_off::Home>,
    origin: Option<GeoPoint>,
    start: bool,
) -> MissionFacts {
    if !set.from_mission {
        return MissionFacts { plan_id: station_plan, mission_stored: set.mission_stored, ..NO_MISSION };
    }
    let far = match (home, origin) {
        (Some(h), Some(o)) if start => {
            let mut far = 0.0f32;
            for &(lat_e7, lon_e7) in set.targets.iter().take(usize::from(set.count)) {
                let [n, e, _] = ned_from_geo(&o, &GeoPoint { lat_e7, lon_e7, alt_mm: o.alt_mm });
                let (dn, de) = (n - h.ne_m[0], e - h.ne_m[1]);
                // Not `max`: a NaN distance must win (Start then answers WaypointDistanceUnknown).
                let d = libm::sqrtf(dn * dn + de * de);
                if !(d <= far) {
                    far = d;
                }
            }
            Some(far)
        }
        _ => None,
    };
    let plan_id = crate::station::plan_id_of(set.plan_id);
    MissionFacts {
        plan_id,
        has_waypoints: set.count > 0,
        mission_stored: set.mission_stored,
        z_within_limits: set.heights_ok,
        max_waypoint_distance_m: far,
        return_height_m: plan_id.map(|_| set.return_m),
        alt_max_m: set.alt_max_m,
    }
}

/// Turns (mode, the newest sticks, nav) into a `Setpoint`: Angle mode (decision-first-pair-20261119, section 4), altitude
/// hold (decision-nav-eskf-20261120, F1, F2) and position hold (Amendment P, P1).
struct StickShaper {
    /// The newest sticks; before the first report, throttle 0 and centred (4e).
    sticks: Sticks,
    angle_max_rad: f32,
    pilot_y_rate_rads: f32,
    pilot_thrust_max_g: f32,
    pilot_speed_up_ms: f32,
    pilot_speed_dn_ms: f32,
    pilot_thr_dz: f32,
    /// How far ahead of the vehicle the altitude target may run: seconds of the limit speed of its direction
    /// (`pilot_alt_lead_s`).
    pilot_alt_lead_s: f32,
    /// Fastest horizontal speed at full stick, m/s: the runtime parameter `PILOT_SPEED_XY` as latched at the arming
    /// step (`Flight::step`); `Tuning::pilot_speed_xy_ms` until the first latch.
    pilot_speed_xy_ms: f32,
    /// Dead zone of the roll and pitch sticks in Position.
    pilot_xy_dz: f32,
    /// Dead zone of the yaw stick, every mode.
    pilot_y_dz: f32,
    /// Position hold captures the position below this ground speed with the sticks centred, m/s.
    pilot_hold_speed_ms: f32,
    /// The altitude target, m down (positive down); `None` outside Altitude and Position. Captured at the first step
    /// whose effective mode is Altitude or Position, and kept across a change between the two.
    target_d: Option<f32>,
    /// The position target, m north and east; `None` outside Position and while the sticks move the vehicle.
    target_ne: Option<[f32; 2]>,
    /// LAND's law (decision-mavlink-mode-20261123, A8, E2): the height target's rate, m/s down, instead of the throttle
    /// stick's; `None` outside LAND.
    land_rate_d_ms: Option<f32>,
    /// LAND's braking law (decision-heading-monitor-20261123, H14 (1)): the position target is this step's position at
    /// every step, never captured; false outside it.
    brake_only: bool,
    /// The low-battery descent is in force: the throttle stick reads 0.0 in the vertical part of Altitude and Position.
    descend: bool,
    /// The previous step's time, us; `None` before the first.
    t_prev_us: Option<u64>,
    /// Tests only: the lead cap is off.
    #[cfg(test)]
    lead_cap_off: bool,
    /// Tests only: N18's seat is off (the control).
    #[cfg(test)]
    seat_off: bool,
    /// Tests only: the position is captured at any speed.
    #[cfg(test)]
    capture_any_speed: bool,
}

impl StickShaper {
    /// The setpoint for the effective `mode` at step time `t_us`. Acro is declared only: Angle. The targets live only
    /// while their mode does: `target_ne` in Position, `target_d` in Altitude and Position.
    fn setpoint(&mut self, mode: Mode, nav: &NavState, t_us: u64) -> Setpoint {
        let dt_s = self
            .t_prev_us
            .map_or(0.0, |prev| t_us.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6);
        self.t_prev_us = Some(t_us);
        match mode {
            Mode::Position => self.position(nav, dt_s),
            Mode::Altitude => {
                self.target_ne = None;
                self.altitude(nav, dt_s)
            }
            Mode::Acro | Mode::Angle => {
                self.target_d = None;
                self.target_ne = None;
                self.angle(nav)
            }
        }
    }

    /// The throttle stick's climb rate, m/s, down positive (F2, ArduPilot's law): 0 within `pilot_thr_dz` of the
    /// centre; below it `pilot_speed_dn_ms` x (distance below the dead zone / the travel left below it), reaching
    /// `pilot_speed_dn_ms` at 0; above it minus `pilot_speed_up_ms` x the same, reaching it at 1.
    fn stick_rate_d(&self, throttle: f32) -> f32 {
        let thr = throttle.clamp(0.0, 1.0);
        let low = THROTTLE_CENTRE - self.pilot_thr_dz;
        let high = THROTTLE_CENTRE + self.pilot_thr_dz;
        // The travel on each side of the dead zone is `low` (= 1 - high).
        if thr < low {
            self.pilot_speed_dn_ms * (low - thr) / low
        } else if thr > high {
            -self.pilot_speed_up_ms * (thr - high) / low
        } else {
            0.0
        }
    }

    /// Altitude hold (F1, F2): roll, pitch and yaw as Angle; the target height captured at the first call after any
    /// other mode (no jump: it is the current height), then integrated at the stick's rate while the vehicle is
    /// within `pilot_alt_lead_s` of the limit speed from it. Down is positive.
    fn altitude(&mut self, nav: &NavState, dt_s: f32) -> Setpoint {
        let (attitude, heading_rate_rads, _) = self.attitude_setpoint(nav);
        let (down_m, velocity_d_ms) = self.height_target(nav, dt_s);
        Setpoint::Altitude {
            attitude,
            heading_rate_rads,
            down_m,
            velocity_d_ms,
        }
    }

    /// The vertical part of Altitude and of Position (P1: exactly the same): the target height, captured at the first
    /// call after any other mode, then integrated at the throttle stick's rate within the lead cap; and that rate.
    /// Down is positive.
    fn height_target(&mut self, nav: &NavState, dt_s: f32) -> (f32, f32) {
        let d = nav.position_m[2];
        let throttle = if self.descend { 0.0 } else { self.sticks.throttle };
        let v_ff = self.land_rate_d_ms.unwrap_or_else(|| self.stick_rate_d(throttle));
        let target = match self.target_d {
            None => d,
            Some(target) => {
                // N18 (b): in the descent states the target is seated at the vehicle when the vehicle is below it (down
                // positive), so the loop asks for v_ff from there and the lead cap cannot hold it (the fixed point
                // d - target = v_ff / posz_p sits on the cap in the 1.5 m/s stage).
                #[cfg(not(test))]
                let seat = self.land_rate_d_ms.is_some();
                #[cfg(test)]
                let seat = self.land_rate_d_ms.is_some() && !self.seat_off;
                let target = if seat { target.max(d) } else { target };
                // The limit speed of the direction the target moves in: down is the descent speed.
                let limit = if v_ff > 0.0 {
                    self.pilot_speed_dn_ms
                } else {
                    self.pilot_speed_up_ms
                };
                let within = libm::fabsf(d - target) < limit * self.pilot_alt_lead_s;
                #[cfg(test)]
                let within = within || self.lead_cap_off;
                if within { target + v_ff * dt_s } else { target }
            }
        };
        self.target_d = Some(target);
        (target, v_ff)
    }

    /// One roll or pitch stick through the dead zone: 0 within `pilot_xy_dz`, else (|s| - dz) / (1 - dz) with the sign.
    fn stick_xy(&self, s: f32) -> f32 {
        Self::dead_zone(s, self.pilot_xy_dz)
    }

    /// The yaw stick's heading rate, rad/s: the stick through the dead zone `pilot_y_dz`, times `pilot_y_rate_rads`.
    fn heading_rate(&self, yaw: f32) -> f32 {
        Self::dead_zone(yaw, self.pilot_y_dz) * self.pilot_y_rate_rads
    }

    /// 0 within `dz` of the centre, else (|s| - dz) / (1 - dz) with the sign.
    fn dead_zone(s: f32, dz: f32) -> f32 {
        if libm::fabsf(s) <= dz {
            0.0
        } else {
            libm::copysignf((libm::fabsf(s) - dz) / (1.0 - dz), s)
        }
    }

    /// Position hold (P1; PX4 FlightTaskManualPosition.cpp:65-123): the sticks ask for a horizontal velocity in the
    /// heading frame, the height as Altitude. With the sticks centred and the ground speed below `pilot_hold_speed_ms` the
    /// position is captured and held; moving the sticks releases it. Unlocked, the position target is this step's
    /// position, so only the velocity feed-forward acts. `psi` is the yaw of the estimate's twist, as Angle uses it.
    fn position(&mut self, nav: &NavState, dt_s: f32) -> Setpoint {
        let (down_m, velocity_d_ms) = self.height_target(nav, dt_s);
        let [w, _, _, z] = nav.attitude;
        let psi = 2.0 * libm::atan2f(z, w);
        // Forward is the stick that tilts the nose down in Angle (pitch > 0), right the one that lowers the right side.
        let (mut sx, mut sy) = (self.stick_xy(self.sticks.pitch), self.stick_xy(self.sticks.roll));
        let n = libm::sqrtf(sx * sx + sy * sy);
        if n > 1.0 {
            sx /= n;
            sy /= n;
        }
        let (s, c) = (libm::sinf(psi), libm::cosf(psi));
        let v = self.pilot_speed_xy_ms;
        let v_ff = [v * (c * sx - s * sy), v * (s * sx + c * sy)];
        let [pn, pe, _] = nav.position_m;
        let [vn, ve, _] = nav.velocity_ms;
        let brake = sx == 0.0 && sy == 0.0;
        let stopped = libm::sqrtf(vn * vn + ve * ve) < self.pilot_hold_speed_ms;
        #[cfg(test)]
        let stopped = stopped || self.capture_any_speed;
        self.target_ne = match self.target_ne {
            _ if self.brake_only => None,
            None if brake && stopped => Some([pn, pe]),
            Some(_) if !brake => None,
            other => other,
        };
        let [tn, te] = self.target_ne.unwrap_or([pn, pe]);
        Setpoint::Position {
            position_m: [tn, te, down_m],
            velocity_ms: [v_ff[0], v_ff[1], velocity_d_ms],
            heading_rad: psi,
            heading_rate_rads: self.heading_rate(self.sticks.yaw),
        }
    }

    /// q_sp = twist(q) (x) q_tilt and the heading rate from yaw (4a, 4b), and the stick collective, N.
    fn attitude_setpoint(&self, nav: &NavState) -> ([f32; 4], f32, f32) {
        let Sticks {
            roll,
            pitch,
            throttle,
            yaw,
        } = self.sticks;
        // Tilt vector v = ANGLE_MAX [roll, -pitch, 0], limited to |v| <= ANGLE_MAX, as a rotation about v.
        let (vx, vy) = (self.angle_max_rad * roll, -self.angle_max_rad * pitch);
        let n = libm::sqrtf(vx * vx + vy * vy);
        let half = n.min(self.angle_max_rad) / 2.0;
        let tilt = if n > 0.0 {
            let s = libm::sinf(half) / n;
            [libm::cosf(half), s * vx, s * vy, 0.0]
        } else {
            [1.0, 0.0, 0.0, 0.0]
        };
        // Today's heading: the estimate's twist about down (identity while tilt is not valid, rule R5).
        let [w, _, _, z] = nav.attitude;
        let heading = if nav.valid.tilt {
            quat::normalise([w, 0.0, 0.0, z])
        } else {
            [1.0, 0.0, 0.0, 0.0]
        };
        // Collective, piecewise linear: 0 -> 0, 0.5 -> hover m g, 1 -> min(pilot_thrust_max_g m g, 4 T_hi)
        // (Amendment C: 2.0 m g by default, one straight line; 4 T_hi is four rotors at full speed).
        let hover = MASS * GRAVITY_MSS;
        let full = full_thrust_n(self.pilot_thrust_max_g);
        let thrust_n = if throttle <= 0.5 {
            2.0 * throttle * hover
        } else {
            hover + (2.0 * throttle - 1.0) * (full - hover)
        };
        (quat::mul(heading, tilt), self.heading_rate(yaw), thrust_n)
    }

    /// The throttle at which [`StickShaper::attitude_setpoint`]'s collective is `thrust_n`: its piecewise-linear map
    /// inverted (N11 (1c)).
    fn throttle_for(&self, thrust_n: f32) -> f32 {
        let hover = MASS * GRAVITY_MSS;
        if thrust_n <= hover {
            thrust_n / (2.0 * hover)
        } else {
            0.5 + (thrust_n - hover) / (2.0 * (full_thrust_n(self.pilot_thrust_max_g) - hover))
        }
    }

    /// Angle: [`StickShaper::attitude_setpoint`] as a `Setpoint`.
    fn angle(&self, nav: &NavState) -> Setpoint {
        let (attitude, heading_rate_rads, thrust_n) = self.attitude_setpoint(nav);
        Setpoint::Attitude {
            attitude,
            heading_rate_rads,
            thrust_n,
        }
    }
}

/// The flight code: estimator, controller, allocator, gate. Only [`Flight::step`] reaches them.
#[cfg_attr(
    not(feature = "sim"),
    doc = "\n\nWithout the feature `sim` (the board image) there is no `set_heading_ok` (decision-mavlink-mode-20261123, E5):\n\n```compile_fail\nfn f<E, C, A>(flight: &mut fw_flight::step::Flight<E, C, A>)\nwhere\n    E: fw_flight::gnc::Estimator,\n    C: fw_flight::gnc::Controller,\n    A: fw_flight::gnc::Allocator,\n{\n    flight.set_heading_ok(true);\n}\n```"
)]
pub struct Flight<E: Estimator, C: Controller, A: Allocator> {
    estimator: E,
    controller: C,
    allocator: A,
    gate: Gate,
    shaper: StickShaper,
    /// The mode the switch selects; the newest report's switch holds until the next report.
    selected: Mode,
    limited: Saturation,
    /// The airborne latch (Amendment P, P3 second revision): false at construction and after any disarmed step; true once
    /// an armed step's collective is at or above the ground threshold; false again when the demand has been refused or the
    /// collective on the allocator's idle floor for `land_detect_us`. Position is in effect only while it is true.
    airborne: bool,
    /// How long the demand has been refused or on the idle floor in the armed steps since it was last neither, us.
    low_us: u64,
    /// The low mark (N14 (1)): an armed step's collective has reached `EVER_AIRBORNE_G` x m g since arming; false after
    /// any disarmed step. The state machine's "ever airborne" and the hover learner read it; Position and the integrator
    /// hold keep the airborne latch.
    lifted: bool,
    /// The time the collective must stay low before the latch clears, us (`land_detect_us`).
    land_detect_us: u64,
    /// The previous step's time, us; `None` before the first.
    last_us: Option<u64>,
    /// The collective below which the vehicle counts as on the ground, N (`ground_thrust_g` x m g).
    ground_thrust_n: f32,
    /// The allocator's idle floor, N (`CardAllocator::collective_range`; N14 (1)).
    floor_n: f32,
    /// The allocator's full thrust, N (`CardAllocator::collective_range`; N14 (2)).
    full_n: f32,
    /// The newest value of the parameter `PILOT_SPEED_XY` that `set_pilot_speed_xy` accepted; the shaper and the
    /// controller take it at every disarmed step, so the value at the arming step is the one in force until disarm.
    speed_pending: f32,
    /// The newest value of the parameter `RPM_NOTCH_EN` and of `MOT_POLES` that `set_notch` accepted; taken at every
    /// disarmed step like `speed_pending`.
    notch_pending: (bool, i32),
    /// The notch switch and the pole count in force: the values of the latest disarmed step.
    notch: (bool, i32),
    /// The newest set of the 14 gain parameters that `set_gains` accepted; taken at every disarmed step like
    /// `speed_pending`.
    gains_pending: Gains,
    /// The gains in force: the values of the latest disarmed step.
    gains: Gains,
    /// An eRPM this old or older is missing, us (`notch_hold_us`).
    notch_hold_us: u64,
    /// The newest value of the parameter `FS_HOLD_EN` (`set_fs_hold_en`); the hands-off state machine takes it into force
    /// on disarmed steps only (decision-mavlink-mode-20261123, B9).
    fs_hold_en: bool,
    /// The hands-off state machine (decision-mavlink-mode-20261123, Amendments A to D), stepped before the shaper and the
    /// gate (WR3).
    hands_off: HandsOff,
    /// The newest stick report, as the gate has seen it (not necessarily new this step); `None` before the first.
    rc_newest: Option<RcSample>,
    /// Host tests and MARV-sim only: an override ANDed with this step's `Valid.heading` (`set_heading_ok`); true until set,
    /// and on the board, where nothing sets it.
    heading_ok: bool,
    /// When the newest station HEARTBEAT arrived on the radio link, us (`set_last_radio_heartbeat_us`); `None` until set.
    last_radio_heartbeat_us: Option<u64>,
    /// When the newest RADIO_STATUS arrived from the receiver, us (`set_last_radio_status_us`); `None` until set.
    last_radio_status_us: Option<u64>,
    /// The station command for the next step (`set_command`); taken by that step.
    command: Option<hands_off::StationCommand>,
    /// The answer of the latest step to its command (`take_command_result`).
    command_result: Option<hands_off::CommandResult>,
    /// The plan id of the committed list (`set_plan_id`); `None` when none is stored. Read only while `mission` is not
    /// the uploaded mission (with it, the set's own plan id, pushed with its list, is the plan).
    plan_id: Option<u32>,
    /// The newest target set from core 0 (`set_mission`): the mission facts at every step, and AUTO's list at Start.
    mission: GdnTargets,
    /// AUTO's leg law (A6, section 36): entered at Start (the list copied from `mission` then), stepped on AUTO's steps,
    /// stopped on every other step; its index survives a stop (the kept place, A10).
    legs: LegGuidance,
    /// AUTO: the heading at Start, held for the whole of AUTO (section 30; the guidance commands no yaw), rad.
    leg_heading_rad: f32,
    /// The guidance was done (the last waypoint reached, or the entry refused) at the previous step: the state
    /// machine's `last_waypoint_reached` (it runs before the law, as the track distance).
    leg_done: bool,
    /// RETURN's guidance was within GDN_RADIUS of home with |v_D| below `HOME_VD_MAX_MS` or for `HOME_WAIT_US` (or its
    /// entry was refused) at the previous step: the state machine's `home_reached` (A7: LAND at once).
    leg_home: bool,
    /// FX-4 (1): the step time at which the guidance first reported Home in this RETURN, us; `None` before.
    home_since_us: Option<u64>,
    /// H12: the horizontal distance between this step's position and the Position target the controller got this step,
    /// m, for the state machine at the next step (it runs before the shaper); `None` when the setpoint was not Position.
    track_error_m: Option<f32>,
    /// E3: the earth-frame acceleration, gravity removed, low-passed at `TOUCHDOWN_ACCEL_LPF_HZ`, m/s^2 NED, from IMU1's
    /// specific force and the estimate's attitude; every step with a finite value.
    accel_ef_mss: [f32; 3],
    /// The touchdown rule TD (Amendment N, N4; replaces E3; N14 (1)): how long (a) the collective on the allocator's idle
    /// floor (its `thrust_low` flag; a refused demand counts), (b) `accel_ef_mss` at most `TOUCHDOWN_ACCEL_MAX_MSS` and (c) the roll and pitch
    /// rates each at most `TOUCHDOWN_ROT_MAX_RADS` have held together, in the armed steps of every state since one last
    /// failed, us (N10).
    still_us: u64,
    /// Tests only: TD's (b) at the latest step, from a finite IMU sample.
    #[cfg(test)]
    still: bool,
    /// N15 (a): the latest step flew LAND, TAKEOVER's descent or BLIND.
    descent_prev: bool,
    /// BLIND's probe: how long (b) and (c) have held in the armed BLIND steps with no probe running, since one last
    /// failed, us.
    quiet_us: u64,
    /// BLIND's probe: how long its ramp has run, us; `None` when no probe runs.
    probe_us: Option<u64>,
    /// E2: the descent rate LAND and the TAKEOVER descent flew at the latest step, m/s down; `None` outside them.
    land_rate_ms: Option<f32>,
    /// N11 (1a): the learned hover collective H, N: the card's m g at every disarmed step, then a low-pass of the
    /// commanded collective on the qualifying steps, within m g x [`HOVER_MIN_G`, `HOVER_MAX_G`].
    hover_n: f32,
    /// N11 (1a): the qualifying time since arming, us (H is used from `HOVER_VALID_US`); 0 at every disarmed step.
    hover_us: u64,
    /// N13: time in the present BLIND episode up to `BLIND_CORRECT_US`, us; the sum of its finite net vertical
    /// accelerations (m/s^2, down positive), their count and the count of its steps; 0 outside BLIND.
    blind_us: u64,
    blind_sum: f32,
    blind_finite: u32,
    blind_steps: u32,
    /// N13: the correction was made (or skipped) in this BLIND episode; BLIND's collective after it, N, when made.
    blind_done: bool,
    blind_trim_n: Option<f32>,
    /// Crash rule K, shadow (Amendment N, N5): its terms and timer after the latest step.
    crash: CrashStatus,
    /// The newest value of the low-battery monitor's descent request (core 0, `set_batt_descend`).
    batt_request: bool,
    /// The request at the previous step.
    batt_prev_request: bool,
    /// A false-to-true edge of the request was seen while armed and the request has not fallen since; cleared while
    /// disarmed, so a request already true at arming (disarm and re-arm within one supervisor tick) starts nothing.
    batt_started: bool,
    /// A change of the mode switch cancelled the descent; cleared at the first step the gate is disarmed.
    batt_cancelled: bool,
    /// The mode switch of the previous stick report; `None` before the first.
    prev_switch6: Option<Switch3>,
    /// What the descent did at the latest step.
    batt_state: BattDescent,
    /// Tests only: the effective mode ignores what the estimate covers.
    #[cfg(test)]
    ignore_coverage: bool,
    /// Tests only: the latch is set at the end of every step, armed or not.
    #[cfg(test)]
    airborne_forced: bool,
    /// Tests only: the latch never clears by the timer.
    #[cfg(test)]
    timer_off: bool,
    /// Tests only: FX-4 (1)'s bound on RETURN's home wait is off (the control).
    #[cfg(test)]
    home_wait_off: bool,
    /// Tests only: the old per-step rule: airborne is armed and the collective at or above the threshold, at every step.
    #[cfg(test)]
    per_step_rule: bool,
    /// Tests only: the previous latch rule: low is a collective below the ground threshold, not the idle floor.
    #[cfg(test)]
    threshold_rule: bool,
    /// Tests only: the speed is latched at every step, armed or not.
    #[cfg(test)]
    latch_off: bool,
    /// Tests only: TD's count does not restart on entry into a descent (N15 (a) removed).
    #[cfg(test)]
    td_restart_off: bool,
    /// Tests only: TD's (a) is the old rule, the collective below the ground threshold (0.714 x the card's m g).
    #[cfg(test)]
    td_card_rule: bool,
    /// The state machine's battery critical (B1): the monitor's CRITICAL level (`set_battery_cell`); false until set.
    battery_critical: bool,
    /// The state machine's `battery_ok` (Start): the monitor knows the battery is not critical (`set_battery_cell`, or
    /// `set_battery_ok` in host tests and MARV-sim); false until set (Start answers BatteryNotOk).
    battery_ok: bool,
}

impl<E: Estimator, C: Controller, A: Allocator> Flight<E, C, A> {
    /// `tuning`: the gains and limits; `handshake`: where the gate stores the armed state for core 0.
    /// The selected mode is `Mode::Angle` until the first stick report brings the switch.
    pub fn new(
        estimator: E,
        controller: C,
        allocator: A,
        tuning: &Tuning,
        handshake: &'static Handshake,
    ) -> Self {
        Self {
            estimator,
            controller,
            allocator,
            gate: Gate::new(tuning, handshake),
            shaper: StickShaper {
                sticks: Sticks {
                    roll: 0.0,
                    pitch: 0.0,
                    throttle: 0.0,
                    yaw: 0.0,
                },
                angle_max_rad: tuning.angle_max_rad,
                pilot_y_rate_rads: tuning.pilot_y_rate_rads,
                pilot_thrust_max_g: tuning.pilot_thrust_max_g,
                pilot_speed_up_ms: tuning.pilot_speed_up_ms,
                pilot_speed_dn_ms: tuning.pilot_speed_dn_ms,
                pilot_thr_dz: tuning.pilot_thr_dz,
                pilot_alt_lead_s: tuning.pilot_alt_lead_s,
                pilot_speed_xy_ms: tuning.pilot_speed_xy_ms,
                pilot_xy_dz: tuning.pilot_xy_dz,
                pilot_y_dz: tuning.pilot_y_dz,
                pilot_hold_speed_ms: tuning.pilot_hold_speed_ms,
                target_d: None,
                target_ne: None,
                land_rate_d_ms: None,
                brake_only: false,
                descend: false,
                t_prev_us: None,
                #[cfg(test)]
                lead_cap_off: false,
                #[cfg(test)]
                seat_off: false,
                #[cfg(test)]
                capture_any_speed: false,
            },
            selected: Mode::Angle,
            limited: Saturation {
                roll: false,
                pitch: false,
                yaw: false,
                thrust_low: false,
                thrust_high: false,
            },
            airborne: false,
            low_us: 0,
            lifted: false,
            land_detect_us: tuning.land_detect_us,
            last_us: None,
            ground_thrust_n: tuning.ground_thrust_g * MASS * GRAVITY_MSS,
            floor_n: CardAllocator::new().collective_range().0,
            full_n: CardAllocator::new().collective_range().1,
            speed_pending: tuning.pilot_speed_xy_ms,
            notch_pending: (false, NOTCH_POLES_DEFAULT),
            notch: (false, NOTCH_POLES_DEFAULT),
            gains_pending: Gains::of(tuning),
            gains: Gains::of(tuning),
            notch_hold_us: tuning.notch_hold_us,
            fs_hold_en: false,
            hands_off: HandsOff::new(tuning),
            rc_newest: None,
            heading_ok: true,
            last_radio_heartbeat_us: None,
            last_radio_status_us: None,
            command: None,
            command_result: None,
            plan_id: None,
            mission: MISSION_OFF,
            legs: LegGuidance::new(),
            leg_heading_rad: 0.0,
            leg_done: false,
            leg_home: false,
            home_since_us: None,
            track_error_m: None,
            accel_ef_mss: [0.0; 3],
            still_us: 0,
            descent_prev: false,
            #[cfg(test)]
            still: false,
            quiet_us: 0,
            probe_us: None,
            land_rate_ms: None,
            hover_n: MASS * GRAVITY_MSS,
            hover_us: 0,
            blind_us: 0,
            blind_sum: 0.0,
            blind_finite: 0,
            blind_steps: 0,
            blind_done: false,
            blind_trim_n: None,
            crash: CrashStatus {
                lean: false,
                error: false,
                accel: false,
                speed: false,
                held_us: 0,
            },
            batt_request: false,
            batt_prev_request: false,
            batt_started: false,
            batt_cancelled: false,
            prev_switch6: None,
            batt_state: BattDescent::Off,
            #[cfg(test)]
            ignore_coverage: false,
            #[cfg(test)]
            airborne_forced: false,
            #[cfg(test)]
            timer_off: false,
            #[cfg(test)]
            home_wait_off: false,
            #[cfg(test)]
            per_step_rule: false,
            #[cfg(test)]
            threshold_rule: false,
            #[cfg(test)]
            latch_off: false,
            #[cfg(test)]
            td_restart_off: false,
            #[cfg(test)]
            td_card_rule: false,
            battery_critical: false,
            battery_ok: false,
        }
    }

    /// The newest value of the runtime parameter `PILOT_SPEED_XY`, m/s (Amendment P, P9). Accepted only if finite and
    /// within `PILOT_SPEED_XY_MIN_MS` to `PILOT_SPEED_XY_MAX_MS` (the parameter table enforces the same range; this is the
    /// flight core's own check). It takes effect at the next arming: every disarmed step latches the newest value, no
    /// armed step does. Returns whether it was accepted.
    pub fn set_pilot_speed_xy(&mut self, ms: f32) -> bool {
        let ok = (PILOT_SPEED_XY_MIN_MS..=PILOT_SPEED_XY_MAX_MS).contains(&ms);
        if ok {
            self.speed_pending = ms;
        }
        ok
    }

    /// The newest value of the runtime parameter `RPM_NOTCH_EN` (`on`) and of `MOT_POLES` (`poles`, poles and not pole
    /// pairs). Accepted only if `poles` is within 2 to 64 (the parameter's range; this is the flight core's own check). It
    /// takes effect at the next arming: every disarmed step latches the newest values, no armed step does. Returns whether
    /// it was accepted.
    pub fn set_notch(&mut self, on: bool, poles: i32) -> bool {
        let ok = NOTCH_POLES_RANGE.contains(&poles);
        if ok {
            self.notch_pending = (on, poles);
        }
        ok
    }

    /// The newest set of the 14 gain parameters (decision-gain-params-20261123). Accepted only if every value is finite and
    /// within `Gains::MIN` to `Gains::MAX` (the parameter table enforces the same ranges; this is the flight core's own
    /// check); a refused set keeps the previous one. It takes effect at the next arming: every disarmed step latches the
    /// newest set, no armed step does. Returns whether it was accepted.
    pub fn set_gains(&mut self, gains: Gains) -> bool {
        let ok = gains.in_range();
        if ok {
            self.gains_pending = gains;
        }
        ok
    }

    /// The newest value of the runtime parameter `FS_HOLD_EN` (decision-mavlink-mode-20261123). Given to the hands-off state
    /// machine every step; it takes effect on a disarmed step (B9).
    pub fn set_fs_hold_en(&mut self, on: bool) {
        self.fs_hold_en = on;
    }

    /// An override of the hands-off state machine's heading verdict (B4: T3' = position covered AND heading OK). The
    /// verdict is this step's `Valid.heading` AND `ok` (H12 item 4 as amended by H14): false forces it false, true leaves
    /// the estimate's flag (never true against it). Only in host tests and with the feature `sim`, which the board image
    /// never enables; there the verdict is `Valid.heading` alone (E5).
    #[cfg(any(test, feature = "sim"))]
    pub fn set_heading_ok(&mut self, ok: bool) {
        self.heading_ok = ok;
    }

    /// The state machine's `battery_ok` (Start's battery check) alone: only in host tests and with the feature `sim`; the
    /// board sets it with [`Flight::set_battery_cell`].
    #[cfg(any(test, feature = "sim"))]
    pub fn set_battery_ok(&mut self, ok: bool) {
        self.battery_ok = ok;
    }

    /// When the newest station HEARTBEAT arrived on the radio link, us (A1's link age). `None` (the default, and the board's
    /// value until the station link is wired) makes the link age run from the newest stick report alone. A missing feed
    /// means "link lost" (E5): 10 s (`LINK_LOSS_US`) after the last stick report HOLD leaves for RETURN, or LAND without
    /// home (always on this branch), and Start is DENIED (no station); that fails safe.
    pub fn set_last_radio_heartbeat_us(&mut self, t_us: Option<u64>) {
        self.last_radio_heartbeat_us = t_us;
    }

    /// When the newest RADIO_STATUS arrived from the receiver, us (the RC port's time of it). A RADIO_STATUS newer than
    /// the entry into a hands-off state sets N12's M, which holds an arm-OFF report while K is clear; `None` (the
    /// default) never sets it, so an arm-OFF report stops at once (a plain dropout).
    pub fn set_last_radio_status_us(&mut self, t_us: Option<u64>) {
        self.last_radio_status_us = t_us;
    }

    /// A station command (link-contract section 10) for the next step, which gives it to the state machine and answers
    /// it ([`Flight::take_command_result`]); a later call before that step replaces it.
    pub fn set_command(&mut self, command: hands_off::StationCommand) {
        self.command = Some(command);
    }

    /// The plan id of the committed list (link-contract section 17, Amendment A: a new one is the state machine's "new
    /// plan"); `None` when none is stored. Given to the state machine every step.
    pub fn set_plan_id(&mut self, plan_id: Option<u32>) {
        self.plan_id = plan_id;
    }

    /// The newest target set core 0 pushed (`FLIGHT_GDN`, whole sets only), at any step, armed or not (Amendment A, A4):
    /// the mission facts from the next step on; AUTO takes its list from it at Start only, never mid-leg.
    pub fn set_mission(&mut self, set: GdnTargets) {
        self.mission = set;
    }

    /// AUTO's active waypoint index while its guidance runs (on a leg or done at the last waypoint), else `None`: the
    /// board logs [`leg_msg`] when it changes (an `Arrived` advances it).
    pub fn leg_active(&self) -> Option<u8> {
        matches!(self.legs.phase(), LegPhase::Leg | LegPhase::Done).then(|| self.legs.active())
    }

    /// The latest step's answer to its command; `None` when it had none (or the answer was taken).
    pub fn take_command_result(&mut self) -> Option<hands_off::CommandResult> {
        self.command_result.take()
    }

    /// N12's M in force after the latest step: a hands-off state and a RADIO_STATUS after its entry (the log's M bit).
    pub fn radio_status_seen(&self) -> bool {
        self.hands_off.radio_status_seen()
    }

    /// N14 (3), the card check: H / the card's m g in percent (rounded, at most 999) once H is valid and differs from the
    /// card's m g by more than `CARD_CHECK_FRAC`; `None` otherwise. No behaviour reads it; the board logs it once per arming.
    pub fn card_check_pct(&self) -> Option<u16> {
        let r = self.hover_n / (MASS * GRAVITY_MSS);
        (self.hover_us >= HOVER_VALID_US && libm::fabsf(r - 1.0) > CARD_CHECK_FRAC).then(|| (r * 100.0 + 0.5).min(999.0) as u16)
    }

    /// The hands-off state after the latest step (its HEARTBEAT and log code is `State::code`).
    pub fn hands_off_state(&self) -> State {
        self.hands_off.state()
    }

    /// Why the hands-off state after the latest step was entered (`None` before the first change): the board's HO MSG
    /// ([`hands_off_reason_msg`]).
    pub fn hands_off_reason(&self) -> Option<hands_off::Reason> {
        self.hands_off.reason()
    }

    /// When the newest station HEARTBEAT arrived on the radio link, as given (`set_last_radio_heartbeat_us`), us.
    pub fn last_radio_heartbeat_us(&self) -> Option<u64> {
        self.last_radio_heartbeat_us
    }

    /// The plan id the state machine is given at every step (from the set in force or from `set_plan_id`, as
    /// `mission_facts` chooses) and the waypoints it sees (0 unless the set is the mission): the board's PLAN MSG.
    pub fn plan_seen(&self) -> (Option<u32>, u8) {
        let facts = mission_facts(&self.mission, self.plan_id, None, None, false);
        (facts.plan_id, if self.mission.from_mission { self.mission.count } else { 0 })
    }

    /// FS_HOLD_EN in force after the latest step (B9: taken on disarmed steps). The board logs every change of the
    /// hands-off state while it is true, and only the entries into and exits from a hands-off state while it is false
    /// ([`hands_off_msg_due`], N10 (iv)), so the log of an image flown with 0 changes only after a stick loss.
    pub fn fs_hold_in_force(&self) -> bool {
        self.hands_off.fs_hold_in_force()
    }

    /// Crash rule K after the latest step (Amendment N, N5, shadow): the four terms and how long they have held.
    pub fn crash_status(&self) -> CrashStatus {
        self.crash
    }

    /// K would fire after the latest step (shadow: nothing acts on it but the board's log).
    pub fn crash_would_fire(&self) -> bool {
        self.crash.would_fire()
    }

    /// The low-battery monitor's descent request (Amendment BATT): true while CRITICAL is reached and `BATT_FS_CRT_ACT` is 1.
    /// Core 0 sets it from one byte; every step reads the newest value. While true, armed and not cancelled the effective
    /// mode is at least Altitude when the estimate covers it and the throttle stick reads 0.0 (F2's descent speed and lead
    /// cap); false changes nothing, bit for bit.
    pub fn set_batt_descend(&mut self, on: bool) {
        self.batt_request = on;
    }

    /// The low-battery monitor's byte from core 0 (`fw_core::battery`'s cell byte), every step: the descent request
    /// ([`Flight::set_batt_descend`]), the state machine's battery critical (CRITICAL reached, whatever `BATT_FS_CRT_ACT`)
    /// and its `battery_ok` (`CELL_KNOWN_OK`).
    pub fn set_battery_cell(&mut self, cell: u8) {
        self.set_batt_descend(fw_core::battery::cell_descends(cell));
        self.battery_critical = fw_core::battery::cell_critical(cell);
        self.battery_ok = fw_core::battery::cell_known_ok(cell);
    }

    /// What the descent did at the latest step, read only.
    pub fn batt_descent(&self) -> BattDescent {
        self.batt_state
    }

    /// The estimator, read only (the log of its own counters, packet E2).
    pub fn estimator(&self) -> &E {
        &self.estimator
    }

    /// The pilot's horizontal speed in force, m/s (`PILOT_SPEED_XY` as latched), read only.
    pub fn speed_xy_in_force(&self) -> f32 {
        self.shaper.pilot_speed_xy_ms
    }

    /// The 14 gains in force (as latched), read only.
    pub fn gains_in_force(&self) -> Gains {
        self.gains
    }

    /// [`Flight::step`], then `publish` with its output (the board sends the command there), then the shadow estimator
    /// (decision-nav-eskf-20261120, 8a): the gate's armed state after the step and the same sensors. The step's output
    /// is computed and published before the shadow runs and nothing of the shadow reaches the flight code; its state is
    /// returned for the log only. `None`: no shadow, the plain step.
    pub fn step_with_shadow<S: Estimator>(
        &mut self,
        inputs: &FlightInputs,
        shadow: Option<&mut S>,
        publish: impl FnOnce(&StepOut),
    ) -> (StepOut, Option<NavState>) {
        let out = self.step(inputs);
        publish(&out);
        let nav = shadow.map(|s| {
            s.set_armed(out.armed);
            s.update(&inputs.sensors)
        });
        (out, nav)
    }

    /// One call per IMU1 sample.
    pub fn step(&mut self, inputs: &FlightInputs) -> StepOut {
        let t_us = inputs.sensors.imu1.t_us;
        // The pilot's horizontal speed is latched while disarmed (the gate's state the previous step left): the step that
        // arms uses the newest value and no later step changes it until the next disarm.
        #[cfg(test)]
        let disarmed = !self.gate.armed() || self.latch_off;
        #[cfg(not(test))]
        let disarmed = !self.gate.armed();
        if disarmed {
            self.shaper.pilot_speed_xy_ms = self.speed_pending;
            self.controller.set_speed_limit_xy(self.speed_pending);
            self.notch = self.notch_pending;
            self.gains = self.gains_pending;
            self.controller.set_gains(&self.gains);
        }
        // The descent starts on a false-to-true edge of the request seen while armed, never on a level.
        if !self.gate.armed() || !self.batt_request {
            self.batt_started = false;
        } else if !self.batt_prev_request {
            self.batt_started = true;
        }
        self.batt_prev_request = self.batt_request;
        // N19 1(3): a report the gate does not accept (the other source, or a receiver report at FS_HOLD_EN 0 while
        // disarmed) reaches nothing in this step.
        let rc = inputs.rc.as_ref().filter(|r| self.gate.accepts(r, self.fs_hold_en));
        if let Some(&rc) = rc {
            self.rc_newest = Some(rc);
            self.shaper.sticks = rc.sticks;
            self.selected = match rc.switch6 {
                Switch3::Low => Mode::Angle,
                Switch3::Middle => Mode::Altitude,
                Switch3::High => Mode::Position,
            };
            // Any change of the mode switch while the descent is in force cancels it until disarm.
            if self.batt_started && self.gate.armed() && self.prev_switch6.is_some_and(|p| p != rc.switch6) {
                self.batt_cancelled = true;
            }
            self.prev_switch6 = Some(rc.switch6);
        }
        if !self.gate.armed() {
            self.batt_cancelled = false;
        }
        let batt_on = self.batt_started && !self.batt_cancelled && self.gate.armed();
        // The armed state the previous step left, then R1.1 to R1.7.
        self.estimator.set_armed(self.gate.armed());
        // A non-finite IMU value never reaches the estimator: no tilt, so the demand is refused and the gate stops.
        let reading = &inputs.sensors.imu1.reading;
        let finite = reading.gyro_rads.iter().all(|v| v.is_finite())
            && reading.accel_ms2.iter().all(|v| v.is_finite());
        let nav = if finite {
            self.estimator.update(&inputs.sensors)
        } else {
            NavState {
                t_us,
                attitude: [1.0, 0.0, 0.0, 0.0],
                rates_rads: [0.0; 3],
                velocity_ms: [0.0; 3],
                position_m: [0.0; 3],
                valid: Valid::NONE,
                sigma: None,
            }
        };
        if !self.gate.armed() {
            self.controller.reset();
        }
        // The effective mode (5e, F1; P3 replaced): Position if selected, the estimate covers it and the vehicle is
        // airborne (the latch); else Altitude if selected (Position or Altitude) and the estimate covers it; otherwise Angle. Evaluated
        // every step; Acro cannot be selected.
        #[cfg(test)]
        let (covered, position_ok) = (
            self.ignore_coverage || altitude_covered(&nav.valid),
            (self.ignore_coverage || position_covered(&nav.valid)) && self.airborne,
        );
        #[cfg(not(test))]
        let (covered, position_ok) = (
            altitude_covered(&nav.valid),
            position_covered(&nav.valid) && self.airborne,
        );
        // The hands-off state machine (WR3): the step time and the newest report as the gate sees them, the armed state and
        // the airborne latch the previous step left, this step's tier.
        let tier = if position_covered(&nav.valid) {
            Tier::T3
        } else if altitude_covered(&nav.valid) {
            Tier::T2
        } else {
            Tier::T1
        };
        let (battery_critical, battery_ok) = (self.battery_critical, self.battery_ok);
        let was = self.hands_off.state();
        // N11 (1a): the low mark (N14 (1)) and TD as the state machine reads them at this step.
        let (lifted_in, td_in) = (self.lifted, self.still_us >= self.land_detect_us);
        let command = self.command.take();
        let start = command.is_some_and(|c| c.command == hands_off::Command::Start);
        let mission = mission_facts(&self.mission, self.plan_id, self.hands_off.home(), self.estimator.origin(), start);
        let ho = self.hands_off.step(&hands_off::Inputs {
            now_us: t_us,
            fs_hold_en: self.fs_hold_en,
            crsf: self.rc_newest,
            last_radio_heartbeat_us: self.last_radio_heartbeat_us,
            last_radio_status_us: self.last_radio_status_us,
            armed: self.gate.armed(),
            airborne: self.airborne,
            lifted: lifted_in,
            tier,
            position_ne_m: [nav.position_m[0], nav.position_m[1]],
            position_ne_valid: nav.valid.position_ne,
            velocity_ne_ms: [nav.velocity_ms[0], nav.velocity_ms[1]],
            velocity_ne_valid: nav.valid.velocity_ne,
            position_d_m: nav.position_m[2],
            position_d_valid: nav.valid.position_d,
            command,
            mission,
            last_waypoint_reached: self.leg_done,
            home_reached: self.leg_home,
            heading_ok: nav.valid.heading && self.heading_ok,
            battery_ok,
            battery_critical,
            return_built: true,
            track_error_m: self.track_error_m,
            speed_xy_ms: self.shaper.pilot_speed_xy_ms,
            td_held: td_in,
        });
        self.command_result = ho.command;
        // H13 (2): v_ho in the hands-off states, PILOT_SPEED_XY in force otherwise.
        self.controller.set_speed_limit_xy(ho.speed_limit_xy_ms);
        // Outside PILOT the state machine is the only actor: the low-battery descent changes nothing there (its request,
        // edge and cancel still run, so it acts again in PILOT as before; STOP included, the touchdown step's disarm).
        let batt_on = batt_on && ho.state == State::Pilot;
        // Hands-off: the switch treated as High (Position), and the state machine's demand is the ceiling (I12).
        // The low-battery descent lifts Angle to Altitude (when covered); Position stays under its own rule.
        let selected = if ho.synthetic_sticks {
            Mode::Position
        } else if batt_on && self.selected == Mode::Angle {
            Mode::Altitude
        } else {
            self.selected
        };
        let mode = match selected {
            Mode::Position if position_ok => Mode::Position,
            Mode::Altitude | Mode::Position if covered => Mode::Altitude,
            _ => Mode::Angle,
        };
        let mode = match (ho.synthetic_sticks, ho.demand, mode) {
            (true, Demand::Angle, _) => Mode::Angle,
            (true, Demand::Altitude, Mode::Position) => Mode::Altitude,
            _ => mode,
        };
        // Hands-off laws (Amendment D): centred synthetic sticks, no real stick reaching the shaper (I3); LAND's descent in
        // LAND and in TAKEOVER's descent; BLIND's collective below hover, and its probe (N4, N14 (1)) linearly down to the
        // idle floor over `BLIND_PROBE_RAMP_US`, then throttle 0 (the allocator holds the floor and flags `thrust_low`). AUTO and RETURN fly HOLD's here; the guidance's setpoint replaces it below. N11 (1c): BLIND's
        // collective is H - m `BLIND_BELOW_HOVER_MSS` once H is valid (the throttle that gives it in Angle), else the card's.
        // N13: after the one-shot correction, the corrected collective (the probe ramps from it, an aborted probe returns
        // to it); N16 (a): the correction only cuts, so it never exceeds BLIND's entry collective.
        let hover_valid = self.hover_us >= HOVER_VALID_US;
        let blind_start = match self.blind_trim_n {
            Some(c) => self.shaper.throttle_for(c),
            None if hover_valid => self.shaper.throttle_for(self.hover_n - MASS * BLIND_BELOW_HOVER_MSS),
            None => BLIND_THROTTLE,
        };
        let blind_throttle = match self.probe_us {
            None => blind_start,
            Some(p) if p >= BLIND_PROBE_RAMP_US => 0.0,
            Some(p) => {
                let ramp = p as f32 / BLIND_PROBE_RAMP_US as f32;
                let floor = self.shaper.throttle_for(self.floor_n);
                blind_start - ramp * (blind_start - floor)
            }
        };
        let dt_us = self.last_us.map_or(0, |prev| t_us.saturating_sub(prev).min(DT_MAX_US));
        self.last_us = Some(t_us);
        // E2: `LAND_SPEED_HIGH_MS` while the height above home is above `LAND_ALT_LOW_M` plus the slowing margin, else (no
        // home, no valid height) `LAND_SPEED_MS`; later steps move towards it at `LAND_RATE_ACC_MSS`. N11 (2): the first
        // descent step starts from the estimated v_D when it is valid (within the pilot's climb and descent speeds, the
        // range of the altitude law's own rate; ArduPilot AC_PosControl.cpp:852), else from the stage rate.
        let high = ho
            .home
            .is_some_and(|h| nav.valid.position_d && h.d_m - nav.position_m[2] > LAND_ALT_LOW_M + LAND_SLOW_MARGIN_M);
        let rate = if high { LAND_SPEED_HIGH_MS } else { LAND_SPEED_MS };
        let v_d = nav.velocity_ms[2];
        let start = if nav.valid.velocity_d && v_d.is_finite() {
            // max/min, not clamp: clamp's panic on runtime bounds formats floats (XIP code on the flight core).
            v_d.max(-self.shaper.pilot_speed_up_ms).min(self.shaper.pilot_speed_dn_ms)
        } else {
            rate
        };
        self.land_rate_ms = (ho.landing || ho.takeover_descent).then(|| match self.land_rate_ms {
            None => start,
            Some(v) => {
                let dv = LAND_RATE_ACC_MSS * dt_us as f32 * 1e-6;
                if libm::fabsf(rate - v) <= dv { rate } else { v + libm::copysignf(dv, rate - v) }
            }
        });
        let real = self.shaper.sticks;
        if ho.synthetic_sticks {
            self.shaper.sticks = Sticks {
                roll: 0.0,
                pitch: 0.0,
                throttle: match ho.state {
                    State::Blind => blind_throttle,
                    // N16 (b): flown in Angle, the centred synthetic throttle is the hover collective H once valid.
                    _ if mode == Mode::Angle && hover_valid => self.shaper.throttle_for(self.hover_n),
                    _ => THROTTLE_CENTRE,
                },
                yaw: 0.0,
            };
            self.shaper.land_rate_d_ms = self.land_rate_ms;
            self.shaper.brake_only = ho.land_law == Some(LandLaw::Brake);
            // H13 (3): the first hands-off step drops the pilot's target; capture restarts under `pilot_hold_speed_ms`.
            if matches!(was, State::Pilot | State::Stop) {
                self.shaper.target_ne = None;
            }
        }
        self.batt_state = if batt_on {
            if covered { BattDescent::Descending } else { BattDescent::NoHeight }
        } else if self.batt_started && self.batt_cancelled && self.gate.armed() {
            BattDescent::Cancelled
        } else {
            BattDescent::Off
        };
        self.shaper.descend = batt_on;
        let mut setpoint = self.shaper.setpoint(mode, &nav, t_us);
        // AUTO (A6, H3b): entered at the step Start is ACCEPTED, from the vehicle to waypoint 0, or to the kept index after
        // a Pause (A10, L7); each AUTO step on Position flies the guidance's carrots (NE and D, with their rates as
        // feed-forward) at the heading of Start with no heading rate, the cruise speed the hands-off limit in force. The
        // shaper still runs (HOLD's law where the ladder is below Position) and is left with no NE target and the vertical
        // carrot as its height target, so the HOLD that follows brakes, captures and keeps that height. Leaving AUTO stops
        // the guidance. RETURN (A7, H3b-2) is flown the same way from its entry (from HOLD or AUTO): H_r as the state
        // machine latched it, the climb in place, then the leg home with GDN_RADIUS of the target set in force; the
        // heading of the entry; Home (or a refused entry) is the state machine's `home_reached` at the next step (LAND).
        let mut leg_e_m = None;
        if !matches!(ho.state, State::Auto | State::Return) {
            self.legs.stop();
            self.leg_done = false;
            self.leg_home = false;
            self.home_since_us = None;
        } else {
            if was != ho.state {
                self.home_since_us = None;
                let start = match ho.events.start {
                    Some(hands_off::StartAt::KeptPlace) => self.legs.active(),
                    _ => 0,
                };
                let [w, _, _, z] = nav.attitude;
                self.leg_heading_rad = 2.0 * libm::atan2f(z, w);
                let speed = ho.speed_limit_xy_ms;
                let entered = match (ho.home, self.estimator.origin()) {
                    (Some(h), _) if ho.state == State::Return => {
                        let home = guidance::Home { ne: h.ne_m, d: h.d_m };
                        // A non-finite H_r is the present height (`enter_return`); the machine always latches one.
                        let h_r = ho.return_height_m.unwrap_or(f32::NAN);
                        self.legs.enter_return(nav.position_m, home, h_r, self.mission.radius_m, speed)
                    }
                    (Some(h), Some(origin)) => {
                        let home = guidance::Home { ne: h.ne_m, d: h.d_m };
                        self.legs.enter(nav.position_m, home, &origin, &self.mission, start, speed)
                    }
                    _ => false,
                };
                if !entered {
                    self.legs.stop();
                }
            }
            let lead_ok = self.legs.lead_ok(nav.position_m[2], self.shaper.pilot_alt_lead_s);
            let leg = self.legs.step(dt_us as f32 * 1e-6, nav.position_m, nav.valid.position_ne, lead_ok);
            let auto = ho.state == State::Auto;
            self.leg_done = auto && matches!(leg.phase, LegPhase::Done | LegPhase::Idle);
            // FX-2 (3): Home is reported once the vertical speed is also below `HOME_VD_MAX_MS` (or not valid), so LAND does
            // not start from the climb's v_D (N11 (2)) when the climb in place ends at home.
            let v_d = nav.velocity_ms[2];
            // FX-4 (1): or `HOME_WAIT_US` after the guidance first reported Home in this RETURN.
            if !auto && leg.phase == LegPhase::Home {
                self.home_since_us.get_or_insert(t_us);
            }
            let waited = self.home_since_us.is_some_and(|t0| t_us.saturating_sub(t0) >= HOME_WAIT_US);
            #[cfg(test)]
            let waited = waited && !self.home_wait_off;
            let settled = !(nav.valid.velocity_d && v_d.is_finite()) || libm::fabsf(v_d) < HOME_VD_MAX_MS || waited;
            self.leg_home = !auto && (leg.phase == LegPhase::Idle || (leg.phase == LegPhase::Home && settled));
            if leg.phase != LegPhase::Idle && !leg.held && mode == Mode::Position {
                self.shaper.target_ne = None;
                self.shaper.target_d = Some(leg.down_m);
                leg_e_m = Some(leg.e_m);
                setpoint = Setpoint::Position {
                    position_m: [leg.position_ne[0], leg.position_ne[1], leg.down_m],
                    velocity_ms: [leg.velocity_ne[0], leg.velocity_ne[1], leg.velocity_d_ms],
                    heading_rad: self.leg_heading_rad,
                    heading_rate_rads: 0.0,
                };
            }
        }
        self.shaper.sticks = real;
        self.shaper.land_rate_d_ms = None;
        self.shaper.brake_only = false;
        // H12: |p_NE - r_NE| of this step, r the target the controller gets (while unlocked, this step's position); in
        // AUTO's law the guidance's e, vehicle to carrot (H13 (1) "moving target").
        self.track_error_m = match setpoint {
            _ if leg_e_m.is_some() => leg_e_m,
            Setpoint::Position { position_m: [tn, te, _], .. } => {
                let (dn, de) = (nav.position_m[0] - tn, nav.position_m[1] - te);
                Some(libm::sqrtf(dn * dn + de * de))
            }
            _ => None,
        };
        // The rotor frequencies, f1 = eRPM x 2 / (poles x 60) Hz by DShot line, for the controller's notch; an eRPM
        // missing or `notch_hold_us` or more old is `None`.
        let (notch_on, poles) = self.notch;
        let f1_hz = inputs.erpm.map(|e| {
            e.filter(|&(_, t)| t_us.saturating_sub(t) < self.notch_hold_us)
                .map(|(erpm, _)| erpm as f32 * 2.0 / (poles as f32 * 60.0))
        });
        self.controller.set_rotor_hz(f1_hz, notch_on);
        let demand = if nav.valid.covers(&setpoint.requires()) {
            self.controller.update(&nav, &setpoint, &self.limited)
        } else {
            Err(Refused)
        };
        let mut rate = self.controller.rate_log();
        if demand.is_err() {
            rate = RateLog {
                f1_hz: rate.f1_hz,
                on: rate.on,
                ..RateLog::NONE
            };
        }
        let attitude_sp = if matches!(setpoint, Setpoint::Position { .. }) && demand.is_ok() {
            self.controller.attitude_setpoint()
        } else {
            [0.0; 4]
        };
        let allocation = demand.ok().map(|d| self.allocator.allocate(&d));
        if let Some(a) = allocation {
            self.limited = a.saturation;
        }
        let levels = allocation.as_ref().map(|a| &a.levels);
        // WR2: STOP entered from an armed step disarms in this step, before the gate's own rules.
        if ho.disarm {
            self.gate.disarm();
        }
        let motors = self.gate.apply(t_us, rc, &nav, levels, ho.hands_off, ho.arm_off_holds);
        // The airborne latch (P3 replaced), after the gate: a disarmed step clears it, so Position is never in effect on the
        // arming step.
        // Low: the demand was refused, or the allocator put the collective on the idle floor (its `thrust_low` flag, the
        // value DMD logs as Sat bit 3). Start only above the ground threshold, end only after `land_detect_us` low.
        let thrust_ok = demand.is_ok_and(|d| d.thrust_n >= self.ground_thrust_n);
        let low = allocation.map_or(true, |a| a.saturation.thrust_low);
        let floor = low;
        #[cfg(test)]
        let floor = if self.td_card_rule { !thrust_ok } else { floor };
        #[cfg(test)]
        let low = if self.threshold_rule { !thrust_ok } else { low };
        // N14 (1): the low mark, set by an armed step's collective at `EVER_AIRBORNE_G` x m g, cleared on a disarmed step.
        self.lifted = self.gate.armed()
            && (self.lifted || demand.is_ok_and(|d| d.thrust_n >= EVER_AIRBORNE_G * MASS * GRAVITY_MSS));
        if !self.gate.armed() {
            self.airborne = false;
            self.low_us = 0;
        } else if !low {
            if thrust_ok {
                self.airborne = true;
            }
            self.low_us = 0;
        } else {
            self.low_us += dt_us;
            #[cfg(test)]
            let expired = self.low_us >= self.land_detect_us && !self.timer_off;
            #[cfg(not(test))]
            let expired = self.low_us >= self.land_detect_us;
            if expired {
                self.airborne = false;
            }
        }
        // TD, the touchdown rule (decision-mavlink-mode-20261123, Amendment N, N4 and N10; replaces E3): on every armed step,
        // (a) the collective on the idle floor, `floor` (N14 (1): ArduPilot's throttle_lower), (b) the filtered
        // acceleration at most `TOUCHDOWN_ACCEL_MAX_MSS` and (c) the roll and pitch rates at most `TOUCHDOWN_ROT_MAX_RADS`,
        // all for `land_detect_us`. Only the state machine's stops read it (WR1, stick loss with the latch false); in LAND,
        // TAKEOVER's descent and BLIND it also clears the latch, so the next step's ground exit (WR1) stops and disarms.
        // No vertical speed. A non-finite IMU sample is neither (b) nor (c).
        let mut a_down = None;
        if finite {
            let f = reading.accel_ms2;
            let q = nav.attitude;
            let [_, n, e, d] = quat::mul(quat::mul(q, [0.0, f[0], f[1], f[2]]), quat::conj(q));
            let a = [n, e, d + GRAVITY_MSS];
            if a.iter().all(|v| v.is_finite()) {
                a_down = Some(a[2]);
                let dt_s = dt_us as f32 * 1e-6;
                let lpf = |y: f32, x: f32| quat::pt1(y, x, dt_s, TOUCHDOWN_ACCEL_LPF_HZ);
                let [yn, ye, yd] = self.accel_ef_mss;
                self.accel_ef_mss = [lpf(yn, a[0]), lpf(ye, a[1]), lpf(yd, a[2])];
            }
        }
        let still = finite
            && self.accel_ef_mss.iter().map(|v| v * v).sum::<f32>() <= TOUCHDOWN_ACCEL_MAX_MSS * TOUCHDOWN_ACCEL_MAX_MSS;
        let [p_rads, q_rads, _] = nav.rates_rads;
        let level = finite
            && nav.valid.rates
            && libm::fabsf(p_rads) <= TOUCHDOWN_ROT_MAX_RADS
            && libm::fabsf(q_rads) <= TOUCHDOWN_ROT_MAX_RADS;
        let blind = self.gate.armed() && ho.state == State::Blind;
        // N15 (a): entering LAND, TAKEOVER's descent or BLIND from a step that flew none of them restarts TD's count, so a
        // count from PILOT or HOLD never clears the latch on the first descent step (`lost()` read the count the previous
        // step left before the entry, so a vehicle already resting still stops there, N10 (iii)).
        let descent = ho.landing || ho.takeover_descent || blind;
        #[cfg(not(test))]
        let entry = descent && !self.descent_prev;
        #[cfg(test)]
        let entry = descent && !self.descent_prev && !self.td_restart_off;
        self.descent_prev = descent;
        if entry {
            self.still_us = 0;
        }
        if self.gate.armed() && floor && still && level {
            self.still_us += dt_us;
            if self.still_us >= self.land_detect_us && descent {
                self.airborne = false;
            }
        } else {
            self.still_us = 0;
        }
        #[cfg(test)]
        {
            self.still = still;
        }
        // Crash rule K, shadow (Amendment N, N5; ArduPilot crash_check.cpp:60-95): on an armed step, ever airborne since
        // arming, TD not held, tilt and rates valid, the four terms; their time together. The setpoint's attitude is the
        // one the controller tracked this step: Angle's and Altitude's from the setpoint, Position's from the controller;
        // none when the demand was refused (Rates and Waypoint never fly here). Nothing reads it but `crash_status`.
        let q_sp = match setpoint {
            _ if demand.is_err() => None,
            Setpoint::Attitude { attitude, .. } | Setpoint::Altitude { attitude, .. } => Some(attitude),
            Setpoint::Position { .. } => Some(attitude_sp),
            Setpoint::Rates { .. } | Setpoint::Waypoint { .. } => None,
        };
        let watch = self.gate.armed()
            && self.hands_off.ever_airborne()
            && self.still_us < self.land_detect_us
            && nav.valid.tilt
            && nav.valid.rates;
        let (lean, error, accel, speed) = if watch {
            crash_terms(&nav, q_sp, self.accel_ef_mss)
        } else {
            (false, false, false, false)
        };
        let held_us = if lean && error && accel && speed { self.crash.held_us.saturating_add(dt_us) } else { 0 };
        self.crash = CrashStatus { lean, error, accel, speed, held_us };
        // N13: BLIND's one-shot correction, before the probe can start: the mean of the unfiltered earth-frame net
        // acceleration down (the TD filter's input, gravity removed) over the episode's first `BLIND_CORRECT_US`, non-finite
        // samples skipped; with at least half of them finite the collective becomes, once, the first second's collective
        // - m (`BLIND_BELOW_HOVER_MSS` - a_m), kept in the allocator's physical range, idle floor to full thrust (N14 (2): it
        // measures the real net force, so it bounds a poisoned or stale H; max/min, not clamp). A new BLIND entry starts
        // again.
        if !blind {
            (self.blind_us, self.blind_sum, self.blind_finite, self.blind_steps) = (0, 0.0, 0, 0);
            (self.blind_done, self.blind_trim_n) = (false, None);
        } else if !self.blind_done {
            self.blind_steps += 1;
            if let Some(a) = a_down {
                self.blind_sum += a;
                self.blind_finite += 1;
            }
            self.blind_us += dt_us;
            if self.blind_us >= BLIND_CORRECT_US {
                self.blind_done = true;
                if 2 * self.blind_finite >= self.blind_steps {
                    let a_m = self.blind_sum / self.blind_finite as f32;
                    let mg = MASS * GRAVITY_MSS;
                    let first = if hover_valid { self.hover_n } else { mg } - MASS * BLIND_BELOW_HOVER_MSS;
                    let c = first - MASS * (BLIND_BELOW_HOVER_MSS - a_m);
                    // N16 (a): one-sided, min(c, c - m (0.3 - a_m)): it can only cut the collective.
                    self.blind_trim_n = c.is_finite().then(|| c.min(first).max(self.floor_n).min(self.full_n));
                }
            }
        }
        // BLIND's probe (N4): after (b) and (c) have held for `land_detect_us` it starts; (b) failing while it runs ends it
        // at once (BLIND's collective at the next step) and a new `land_detect_us` of (b) and (c) is needed.
        match self.probe_us {
            _ if !blind => {
                self.probe_us = None;
                self.quiet_us = 0;
            }
            Some(p) if still => self.probe_us = Some(p + dt_us),
            Some(_) => {
                self.probe_us = None;
                self.quiet_us = 0;
            }
            None if still && level => {
                self.quiet_us += dt_us;
                if self.quiet_us >= self.land_detect_us {
                    self.probe_us = Some(0);
                }
            }
            None => self.quiet_us = 0,
        }
        #[cfg(test)]
        {
            if self.per_step_rule {
                self.airborne = self.gate.armed() && thrust_ok;
            }
            if self.airborne_forced {
                self.airborne = true;
            }
        }
        // N11 (1a): the hover learner (ArduCopter Attitude.cpp:32-67, also in Angle): on an armed step with a finite
        // commanded collective (review F2), the low mark set (N14 (1)) and TD not held (as the state machine read them), the allocation
        // neither `thrust_low` nor `thrust_high` (N14 (2)), not BLIND, v_D
        // valid below `HOVER_VD_MAX_MS`, roll and pitch
        // each under 5 deg, H follows the commanded collective with `HOVER_TC_S`, clamped; the card's m g when disarmed.
        if !self.gate.armed() {
            self.hover_n = MASS * GRAVITY_MSS;
            self.hover_us = 0;
        } else if let Ok(d) = demand
            && d.thrust_n.is_finite()
            && lifted_in
            && allocation.as_ref().is_some_and(|a| !a.saturation.thrust_low && !a.saturation.thrust_high)
            && !td_in
            && ho.state != State::Blind
            && nav.valid.velocity_d
            && libm::fabsf(nav.velocity_ms[2]) < HOVER_VD_MAX_MS
            && nav.valid.tilt
        {
            let [w, x, y, z] = nav.attitude;
            let cos_part = 1.0 - 2.0 * (x * x + y * y);
            let level = cos_part > 0.0
                && libm::fabsf(2.0 * (w * x + y * z)) < HOVER_TAN_ROLL_MAX * cos_part
                && libm::fabsf(2.0 * (w * y - x * z)) < HOVER_SIN_PITCH_MAX;
            if level {
                let dt_s = dt_us as f32 * 1e-6;
                let mg = MASS * GRAVITY_MSS;
                let h = self.hover_n + dt_s / (dt_s + HOVER_TC_S) * (d.thrust_n - self.hover_n);
                self.hover_n = h.clamp(HOVER_MIN_G * mg, HOVER_MAX_G * mg);
                self.hover_us = self.hover_us.saturating_add(dt_us);
            }
        }
        StepOut {
            t_us,
            motors,
            armed: self.gate.armed(),
            selected: self.selected,
            mode,
            nav,
            setpoint,
            demand,
            allocation,
            attitude_sp,
            rate,
        }
    }
}

/// True when `valid` covers what an Altitude setpoint needs (`Setpoint::requires`, the single definition).
fn altitude_covered(valid: &Valid) -> bool {
    let probe = Setpoint::Altitude {
        attitude: [1.0, 0.0, 0.0, 0.0],
        heading_rate_rads: 0.0,
        down_m: 0.0,
        velocity_d_ms: 0.0,
    };
    valid.covers(&probe.requires())
}

/// True when `valid` covers what a Position setpoint needs (`Setpoint::requires`, the single definition).
fn position_covered(valid: &Valid) -> bool {
    let probe = Setpoint::Position {
        position_m: [0.0; 3],
        velocity_ms: [0.0; 3],
        heading_rad: 0.0,
        heading_rate_rads: 0.0,
    };
    valid.covers(&probe.requires())
}

/// Crash rule K's terms (lean, error, accel, speed; Amendment N, N5) for the estimate `nav`, the attitude setpoint `q_sp`
/// the controller tracked (`None`: no error term) and the filtered earth-frame acceleration `accel_mss`. The angles are
/// those of the thrust axes, compared as cosines; a zero or non-finite quaternion, acceleration or valid velocity makes
/// the terms it enters false.
fn crash_terms(nav: &NavState, q_sp: Option<[f32; 4]>, accel_mss: [f32; 3]) -> (bool, bool, bool, bool) {
    // The thrust axis (body down) of [w, x, y, z] in NED, times |q|^2, and |q|^2.
    let axis = |[w, x, y, z]: [f32; 4]| {
        let a = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
        (a, w * w + x * x + y * y + z * z)
    };
    let (a, n) = axis(nav.attitude);
    let lean = a[2] < CRASH_COS_LEAN_MIN * n;
    let error = q_sp.is_some_and(|q| {
        let (b, m) = axis(q);
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < CRASH_COS_ERROR_MIN * n * m
    });
    let accel = accel_mss.iter().map(|v| v * v).sum::<f32>() < CRASH_ACCEL_MAX_MSS * CRASH_ACCEL_MAX_MSS;
    let speed = !(nav.valid.velocity_ne && nav.valid.velocity_d)
        || nav.velocity_ms.iter().map(|v| v * v).sum::<f32>() < CRASH_SPEED_MAX_MS * CRASH_SPEED_MAX_MS;
    (lean, error, accel, speed)
}

/// What the log writes at one flight step (decision-flight-logging-20261119, section 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogDue {
    /// Write the IMU1 sample this step (every step, armed or not).
    pub imu: bool,
    /// Write the six flight records (NAV, SETP, DMD, MOT, PIDT, NTCH) this step.
    pub block: bool,
    /// Write an ESC record this step.
    pub esc: bool,
}

/// The decimation rule. `step` counts flight steps (the caller adds 1 after each). The IMU1 sample is
/// logged every step. Armed: the six flight records and an ESC record every step; disarmed:
/// the six flight records and the ESC record every 8th step.
pub fn log_due(step: u32, armed: bool) -> LogDue {
    if armed {
        LogDue {
            imu: true,
            block: true,
            esc: true,
        }
    } else {
        let every_8th = step % 8 == 0;
        LogDue {
            imu: true,
            block: every_8th,
            esc: every_8th,
        }
    }
}

/// Writes the six flight records of one step (NAV, SETP, DMD, MOT; decision-first-pair-wiring-20261119,
/// A6; PIDT, NTCH: decision-rpm-notch-20261123) at the start of `buf`. Returns the bytes written, or 0 (nothing written) if `buf` is too small.
pub fn encode_step(out: &StepOut, buf: &mut [u8]) -> usize {
    let need: usize = FLIGHT_TYPES.iter().map(|d| usize::from(d.length)).sum();
    if buf.len() < need {
        return 0;
    }
    let t = out.t_us;
    let valid = valid_bits(&out.nav.valid);
    let (kind, attitude, sp_rates, thrust) = match out.setpoint {
        Setpoint::Rates {
            rates_rads,
            thrust_n,
        } => (0, [1.0, 0.0, 0.0, 0.0], rates_rads, thrust_n),
        Setpoint::Attitude {
            attitude,
            heading_rate_rads,
            thrust_n,
        } => (1, attitude, [0.0, 0.0, heading_rate_rads], thrust_n),
        Setpoint::Altitude { .. } => (2, [0.0; 4], [0.0; 3], 0.0),
        Setpoint::Position { .. } => (3, out.attitude_sp, [0.0; 3], 0.0),
        Setpoint::Waypoint { .. } => (4, [0.0; 4], [0.0; 3], 0.0),
    };
    let demand = out.demand.unwrap_or(ThrustTorque {
        thrust_n: 0.0,
        torque_nm: [0.0; 3],
    });
    let saturation = out.allocation.map_or(0, |a| {
        let s = a.saturation;
        u8::from(s.roll)
            | u8::from(s.pitch) << 1
            | u8::from(s.yaw) << 2
            | u8::from(s.thrust_low) << 3
            | u8::from(s.thrust_high) << 4
    });
    let mode = mode_code(out.mode);
    let levels = out.allocation.map_or([0; LINES], |a| a.levels.level);
    let command = out
        .motors
        .throttles()
        .map_or([0; LINES], |th| th.map(|x| x.level()));
    let mut n = 0;
    n += dflog::encode_nav(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.nav.attitude,
        out.nav.rates_rads,
        valid,
    );
    n += dflog::encode_setp(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        kind,
        attitude,
        sp_rates,
        thrust,
    );
    n += dflog::encode_dmd(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.demand.is_ok(),
        demand.thrust_n,
        demand.torque_nm,
        saturation,
    );
    n += dflog::encode_mot(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.armed,
        mode,
        levels,
        command,
    );
    n += dflog::encode_pidt(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.rate.target_rads,
        out.rate.p_nm,
        out.rate.i_nm,
        out.rate.d_nm,
    );
    n += dflog::encode_ntch(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.rate.rate_rads,
        out.rate.f1_hz,
        out.rate.on,
    );
    n
}

/// MOT's and MODE's code of a mode: 0 Acro, 1 Angle, 2 Altitude, 3 Position.
pub fn mode_code(mode: Mode) -> u8 {
    match mode {
        Mode::Acro => 0,
        Mode::Angle => 1,
        Mode::Altitude => 2,
        Mode::Position => 3,
    }
}

/// The MSG text logged at the first step and at each change of the hands-off state (decision-mavlink-mode-20261123,
/// Amendment D: "Every state's code is logged at each change"): "HO", the `State::code`, the name.
pub fn hands_off_msg(state: State) -> &'static str {
    match state {
        State::Pilot => "HO 1 PILOT",
        State::Takeover => "HO 2 TAKEOVER",
        State::Hold => "HO 3 HOLD",
        State::Auto => "HO 4 AUTO",
        State::Return => "HO 5 RETURN",
        State::Land => "HO 6 LAND",
        State::Stop => "HO 7 STOP",
        State::Blind => "HO 8 BLIND",
    }
}

/// Whether the board writes [`hands_off_msg`] of `state` at this step, `logged` the state of the last MSG written (`None`
/// before the first; a dropped MSG is not written, so it is offered again). With FS_HOLD_EN in force 1, at the first
/// step and at each change (Amendment D, E4). With 0, only at an entry into or an exit from a hands-off state (N10 (iv)):
/// arming and disarming alone (PILOT and STOP) write nothing, so the log of an image flown with 0 changes only after a
/// stick loss.
pub fn hands_off_msg_due(fs_hold_in_force: bool, logged: Option<State>, state: State) -> bool {
    let hands_off = |s: State| !matches!(s, State::Pilot | State::Stop);
    logged != Some(state) && (fs_hold_in_force || hands_off(state) || logged.is_some_and(hands_off))
}

/// The MSG text the board writes for crash rule K (shadow, Amendment N, N5), `logged` the would-fire of the last one
/// written (false before the first; a dropped MSG is not written, so it is offered again): "K 1 " and the term bits
/// lean, error, accel, speed when K would fire and the last said not; "K 0" when it no longer would and the last said it
/// would; `None` otherwise, in every configuration, so a flight in which K never fires logs nothing new. NUL padded.
pub fn crash_msg(logged: bool, s: &CrashStatus) -> Option<[u8; 8]> {
    let bit = |b: bool| if b { b'1' } else { b'0' };
    match (logged, s.would_fire()) {
        (false, true) => Some([b'K', b' ', b'1', b' ', bit(s.lean), bit(s.error), bit(s.accel), bit(s.speed)]),
        (true, false) => Some(*b"K 0\0\0\0\0\0"),
        _ => None,
    }
}

/// AUTO's waypoint MSG: "HO WP <k>", k the active index (0 to 99, two digits).
pub fn leg_msg(k: u8) -> [u8; 8] {
    let k = k.min(99);
    let mut m = *b"HO WP 00";
    (m[6], m[7]) = (b'0' + k / 10, b'0' + k % 10);
    m
}

/// N14 (3): the card check's MSG text, "HOVER <pct> % of card", `pct` right-aligned in three places (at most 999).
pub fn card_msg(pct: u16) -> [u8; 19] {
    let p = pct.min(999);
    let digit = |n: u16| b'0' + (n % 10) as u8;
    let h = if p >= 100 { digit(p / 100) } else { b' ' };
    let t = if p >= 10 { digit(p / 10) } else { b' ' };
    let mut m = *b"HOVER  00 % of card";
    (m[6], m[7], m[8]) = (h, t, digit(p));
    m
}

/// Copies `s` into `m` from `at` (cut at the end of `m`); the index after it.
fn put(m: &mut [u8], at: usize, s: &[u8]) -> usize {
    for (d, &c) in m.iter_mut().skip(at).zip(s) {
        *d = c;
    }
    (at + s.len()).min(m.len())
}

/// Writes `v` in decimal, no leading zeros, into `m` from `at`; the index after it.
fn put_dec(m: &mut [u8], at: usize, v: u32) -> usize {
    let mut digits = [0u8; 10];
    let (mut v, mut n) = (v, 0);
    loop {
        if let Some(d) = digits.get_mut(9 - n) {
            *d = b'0' + (v % 10) as u8;
        }
        n += 1;
        v /= 10;
        if v == 0 || n == 10 {
            break;
        }
    }
    put(m, at, digits.get(10 - n..).unwrap_or_default())
}

/// The HO MSG the board writes: [`hands_off_msg`] of `state`, then a space and the reason's token when there is one
/// ("HO 3 HOLD stn-lost"); NUL padded.
pub fn hands_off_reason_msg(state: State, reason: Option<hands_off::Reason>) -> [u8; 32] {
    let mut m = [0u8; 32];
    let n = put(&mut m, 0, hands_off_msg(state).as_bytes());
    if let Some(r) = reason {
        let n = put(&mut m, n, b" ");
        put(&mut m, n, r.token().as_bytes());
    }
    m
}

/// A station command and the step's answer: "CMD <NAME> ok", or "CMD <NAME> den <code>" with `DenyReason::code` (0 when
/// the step gave no answer, as the mailbox's answer byte); NUL padded.
pub fn command_msg(command: hands_off::Command, result: Option<hands_off::CommandResult>) -> [u8; 20] {
    use hands_off::{Command, CommandResult};
    let mut m = [0u8; 20];
    let name: &[u8] = match command {
        Command::Start => b"CMD START",
        Command::Pause => b"CMD PAUSE",
        Command::Abort => b"CMD ABORT",
        Command::Land => b"CMD LAND",
        Command::Return => b"CMD RETURN",
    };
    let n = put(&mut m, 0, name);
    match result {
        Some(CommandResult::Accepted) => {
            put(&mut m, n, b" ok");
        }
        Some(CommandResult::Denied(r)) => {
            let n = put(&mut m, n, b" den ");
            put_dec(&mut m, n, r.code().into());
        }
        None => {
            put(&mut m, n, b" den 0");
        }
    }
    m
}

/// The plan id the state machine sees and its waypoints ([`Flight::plan_seen`]): "PLAN 0x1862F4A2 n=4", no plan id as
/// 0x00000000 (the empty list's); NUL padded.
pub fn plan_msg(plan_id: Option<u32>, waypoints: u8) -> [u8; 24] {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut m = *b"PLAN 0x00000000 n=\0\0\0\0\0\0";
    let id = plan_id.unwrap_or(0);
    for k in 0..8 {
        if let (Some(d), Some(&h)) = (m.get_mut(7 + k), HEX.get(((id >> (28 - 4 * k)) & 0xF) as usize)) {
            *d = h;
        }
    }
    put_dec(&mut m, 18, waypoints.into());
    m
}

/// At AUTO's exit on station loss: the station HEARTBEAT's age, ms, and the sender of the RC port's latest
/// ground-station HEARTBEAT (any sender of type GCS): "STN age 5012 ms gcs 255,190", "none" for either when unknown; NUL
/// padded.
pub fn station_lost_msg(age_ms: Option<u32>, gcs: Option<(u8, u8)>) -> [u8; 40] {
    let mut m = [0u8; 40];
    let mut n = put(&mut m, 0, b"STN age ");
    n = match age_ms {
        Some(a) => {
            let n = put_dec(&mut m, n, a);
            put(&mut m, n, b" ms gcs ")
        }
        None => put(&mut m, n, b"none gcs "),
    };
    match gcs {
        Some((sys, comp)) => {
            let n = put_dec(&mut m, n, sys.into());
            let n = put(&mut m, n, b",");
            put_dec(&mut m, n, comp.into());
        }
        None => {
            put(&mut m, n, b"none");
        }
    }
    m
}

/// When to write MODE (decision-nav-eskf-20261120, 8c, F6): at the first step and at every change of the selected or
/// the effective mode. The caller asks [`ModeLog::pending`], writes [`encode_mode`] and only then calls
/// [`ModeLog::done`], so a record the ring dropped is offered again at the next step.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModeLog {
    /// (selected, effective) codes of the last record written; `None` before the first.
    last: Option<(u8, u8)>,
}

impl ModeLog {
    /// Nothing written yet.
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// This step's (selected, effective) differs from the last record, or none was written yet.
    pub fn pending(&self, out: &StepOut) -> bool {
        self.last != Some((mode_code(out.selected), mode_code(out.mode)))
    }

    /// The record for `out` was written.
    pub fn done(&mut self, out: &StepOut) {
        self.last = Some((mode_code(out.selected), mode_code(out.mode)));
    }
}

/// MODE of this step at the start of `buf`: `TimeUS` = `out.t_us`, the selected and the effective mode. Returns the
/// bytes written (13), or 0 (nothing written) if `buf` is too small.
pub fn encode_mode(out: &StepOut, buf: &mut [u8]) -> usize {
    dflog::encode_mode(buf, out.t_us, mode_code(out.selected), mode_code(out.mode))
}

/// SPN of this step at the start of `buf` (decision-nav-eskf-20261120, 8c, F1; Amendment P, P5): only when the setpoint is
/// Altitude or Position, `TimeUS` = `out.t_us`, PD the target height (down positive), VD the climb-rate feed-forward (down
/// positive). In Altitude PN, PE, VN, VE and Hdg are 0; in Position PN, PE are the position target (this step's position
/// while unlocked), VN, VE the velocity feed-forward and Hdg the heading of the setpoint. Returns the bytes written (39),
/// or 0 (nothing written) if the setpoint is neither or `buf` is too small.
pub fn encode_spn(out: &StepOut, buf: &mut [u8]) -> usize {
    match out.setpoint {
        Setpoint::Altitude {
            down_m,
            velocity_d_ms,
            ..
        } => dflog::encode_spn(
            buf,
            out.t_us,
            [0.0, 0.0, down_m],
            [0.0, 0.0, velocity_d_ms],
            0.0,
        ),
        Setpoint::Position {
            position_m,
            velocity_ms,
            heading_rad,
            ..
        } => dflog::encode_spn(buf, out.t_us, position_m, velocity_ms, heading_rad),
        _ => 0,
    }
}

/// NAV's and NKF1's valid bits: tilt 1, heading 2, rates 4, velocity NE 8, velocity D 16, position NE 32,
/// position D 64.
fn valid_bits(v: &Valid) -> u8 {
    let Valid {
        tilt,
        heading,
        rates,
        velocity_ne,
        velocity_d,
        position_ne,
        position_d,
    } = *v;
    u8::from(tilt)
        | u8::from(heading) << 1
        | u8::from(rates) << 2
        | u8::from(velocity_ne) << 3
        | u8::from(velocity_d) << 4
        | u8::from(position_ne) << 5
        | u8::from(position_d) << 6
}

/// NKF1 then NKF2 of one estimator's state (decision-nav-eskf-20261120, 8c, B5) at the start of `buf`: instance
/// `inst`, `TimeUS` = `nav.t_us`, the biases as given, the sigmas from `nav.sigma` (infinite when `None`). Returns the
/// bytes written (113), or 0 (nothing written) if `buf` is too small.
pub fn encode_nkf(
    buf: &mut [u8],
    inst: u8,
    nav: &NavState,
    gyro_bias_rads: [f32; 3],
    accel_bias_ms2: [f32; 3],
) -> usize {
    let need = usize::from(dflog::NKF1.length) + usize::from(dflog::NKF2.length);
    if buf.len() < need {
        return 0;
    }
    let inf = f32::INFINITY;
    let sigmas = nav.sigma.map_or([inf; 6], |s| {
        [
            s.tilt_rad,
            s.heading_rad,
            s.velocity_ne_ms,
            s.velocity_d_ms,
            s.position_ne_m,
            s.position_d_m,
        ]
    });
    let n = dflog::encode_nkf1(
        buf,
        nav.t_us,
        inst,
        nav.attitude,
        nav.velocity_ms,
        nav.position_m,
        valid_bits(&nav.valid),
    );
    n + dflog::encode_nkf2(
        buf.get_mut(n..).unwrap_or_default(),
        nav.t_us,
        inst,
        gyro_bias_rads,
        accel_bias_ms2,
        sigmas,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alloc::CardAllocator;
    use crate::gnc::MotorLevels;
    use crate::imu::{ImuReading, ImuSample};
    use crate::mahony::Mahony;
    use crate::pid::CascadedPid;
    use core::cell::Cell;

    static HANDSHAKE: Handshake = Handshake::new();

    fn inputs(t_us: u64) -> FlightInputs {
        FlightInputs {
            sensors: Sensors {
                imu1: ImuSample {
                    reading: ImuReading::default(),
                    t_us,
                    seq: 7,
                },
                imu2: None,
                baro: None,
                mag: None,
                gnss: None,
                external: None,
            },
            rc: None,
            erpm: [None; LINES],
        }
    }

    /// Amendment C: the stick-to-collective map, 0 -> 0, 0.5 -> m g, 1 -> min(cap m g, 4 K_F w_max^2).
    #[test]
    fn collective_follows_the_capped_map() {
        let hover = MASS * GRAVITY_MSS;
        let physical = 4.0 * K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED;
        let nav = NavState {
            t_us: 0,
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: [0.0; 3],
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid {
                tilt: true,
                ..Valid::NONE
            },
            sigma: None,
        };
        let thrust = |cap: f32, throttle: f32| {
            let mut tuning = Tuning::DEFAULT;
            tuning.pilot_thrust_max_g = cap;
            let mut flight = Flight::new(
                Mahony::new(&tuning),
                CascadedPid::new(&tuning, 800.0),
                CardAllocator::new(),
                &tuning,
                &HANDSHAKE,
            );
            flight.shaper.sticks.throttle = throttle;
            match flight.shaper.angle(&nav) {
                Setpoint::Attitude { thrust_n, .. } => thrust_n,
                _ => panic!("not an attitude setpoint"),
            }
        };
        let near = |a: f32, b: f32| assert!((a - b).abs() <= 1e-4 * b.max(1.0), "{a} vs {b}");
        near(thrust(2.0, 0.0), 0.0);
        near(thrust(2.0, 0.25), 0.5 * hover);
        near(thrust(2.0, 0.5), hover);
        near(thrust(2.0, 0.75), 1.5 * hover);
        near(thrust(2.0, 1.0), 2.0 * hover);
        near(thrust(100.0, 1.0), physical);
    }

    #[test]
    fn board_types_give_stop_without_rc() {
        let tuning = Tuning::DEFAULT;
        let mut flight = Flight::new(
            Mahony::new(&tuning),
            CascadedPid::new(&tuning, 800.0),
            CardAllocator::new(),
            &tuning,
            &HANDSHAKE,
        );
        let mut out = flight.step(&inputs(1234));
        assert_eq!(out.motors, MotorCommand::STOP);
        assert_eq!(out.motors.throttles(), None);
        assert_eq!(out.demand, Err(Refused));
        assert_eq!(out.allocation, None);
        assert_eq!(out.t_us, 1234);
        assert_eq!(out.nav.t_us, 1234);
        assert!(!out.armed);

        // I5: a NaN gyro sample while armed gives Stop and disarm without a call to the estimator; the next good
        // sample gives a finite nav.
        let level = |t_us: u64, gyro: [f32; 3]| {
            let mut i = inputs(t_us);
            i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, -9.81];
            i.sensors.imu1.reading.gyro_rads = gyro;
            i
        };
        let report = |t_us: u64, arm: bool| RcSample {
            sticks: Sticks {
                roll: 0.0,
                pitch: 0.0,
                throttle: 0.0,
                yaw: 0.0,
            },
            arm,
            switch6: Switch3::Low,
            source: fw_core::rc::RcSource::Crsf,
            mission: false,
            t_us,
        };
        let mut t = 2_000;
        // The first sample (zero accel) seeds the accelerometer filter: the tilt is valid only once the filtered |f|
        // reaches 0.9 g, tau x ln(10) after the first level step. Run the level steps that takes, plus two.
        const STEP_US: u64 = 1_000;
        let tau_s = 1.0 / (2.0 * core::f32::consts::PI * tuning.ins_accel_lpf_hz);
        let settle = (libm::logf(10.0) * tau_s / (STEP_US as f32 * 1e-6)).ceil() as u32 + 2;
        t += STEP_US;
        out = flight.step(&level(t, [0.0; 3]));
        assert!(!out.nav.valid.tilt);
        for _ in 1..settle {
            t += STEP_US;
            out = flight.step(&level(t, [0.0; 3]));
        }
        HANDSHAKE.set_permit(true);
        for arm in [false, true] {
            t += 1_000;
            let mut i = level(t, [0.0; 3]);
            i.rc = Some(report(t, arm));
            out = flight.step(&i);
        }
        assert!(out.armed && HANDSHAKE.armed());
        assert!(out.motors.throttles().is_some());
        t += 1_000;
        let out = flight.step(&level(t, [0.0, f32::NAN, 0.0]));
        assert_eq!(out.motors, MotorCommand::STOP);
        assert!(!out.armed && !HANDSHAKE.armed());
        assert_eq!(out.nav.valid, Valid::NONE);
        assert_eq!(out.demand, Err(Refused));
        t += 1_000;
        let out = flight.step(&level(t, [0.0; 3]));
        assert!(out.nav.valid.tilt);
        assert!(
            out.nav
                .attitude
                .iter()
                .chain(&out.nav.rates_rads)
                .all(|v| v.is_finite())
        );
    }

    /// Decision-nav-eskf-20261120, section 4 (packet F1): with Mahony the barometer, compass and GNSS samples change
    /// nothing. The same step sequence (level, arm, sticks moving, a moving IMU) with and without the three samples
    /// gives the same outputs, bit for bit (every field's exact `Debug` text and the step's log records).
    #[test]
    fn slow_sensor_samples_do_not_change_a_mahony_step() {
        extern crate std;
        use fw_core::sample::{
            BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample, UtcTime,
        };
        static WITHOUT: Handshake = Handshake::new();
        static WITH: Handshake = Handshake::new();
        let run = |handshake: &'static Handshake, slow: bool| {
            let tuning = Tuning::DEFAULT;
            let mut flight = Flight::new(
                Mahony::new(&tuning),
                CascadedPid::new(&tuning, 800.0),
                CardAllocator::new(),
                &tuning,
                handshake,
            );
            handshake.set_permit(true);
            let mut outs = std::vec::Vec::new();
            for n in 0..3000u32 {
                let t_us = 10_000 + u64::from(n) * 1_245;
                let x = n as f32 * 0.01;
                let mut i = inputs(t_us);
                i.sensors.imu1.seq = n;
                i.sensors.imu1.reading.gyro_rads = [
                    0.2 * libm::sinf(x),
                    0.1 * libm::cosf(1.3 * x),
                    0.05 * libm::sinf(0.7 * x),
                ];
                i.sensors.imu1.reading.accel_ms2 = [
                    0.3 * libm::sinf(0.9 * x),
                    -0.2 * libm::cosf(x),
                    -9.81 + 0.1 * libm::sinf(x),
                ];
                // Arm after 200 steps (throttle low), then sticks that move.
                let stick = |k: f32| {
                    if n < 400 {
                        0.0
                    } else {
                        0.3 * libm::sinf(k * x)
                    }
                };
                i.rc = (n % 10 == 0).then_some(RcSample {
                    sticks: Sticks {
                        roll: stick(1.1),
                        pitch: stick(0.8),
                        throttle: if n < 400 { 0.0 } else { 0.5 + stick(0.5) },
                        yaw: stick(1.7),
                    },
                    arm: n >= 200,
                    switch6: Switch3::Low,
                    source: fw_core::rc::RcSource::Crsf,
                    mission: false,
                    t_us,
                });
                if slow {
                    i.sensors.baro = (n % 16 == 3).then_some(BaroSample {
                        reading: BaroReading {
                            pressure_pa: 101_325.0 - x,
                            temperature_c: 25.0,
                        },
                        t_us: t_us - 500,
                        seq: n / 16,
                    });
                    i.sensors.mag = (n % 8 == 5).then_some(MagSample {
                        reading: MagReading {
                            field_ut: [20.0 * libm::cosf(x), 20.0 * libm::sinf(x), 40.0],
                            temperature_c: 30.0,
                        },
                        t_us: t_us - 300,
                        seq: n / 8,
                        expected: None,
                    });
                    i.sensors.gnss = (n % 160 == 7).then_some(GnssSample {
                        reading: GnssReading {
                            fix_type: 3,
                            lat_e7: Some(254_000_000 + n as i32),
                            lon_e7: Some(-801_000_000 - n as i32),
                            alt_mm: Some(12_000),
                            speed_mm_s: Some(n),
                            course_cdeg: Some(9_000),
                            sats_used: Some(12),
                            hdop_x100: Some(80),
                            vdop_x100: Some(120),
                            time: Some(UtcTime {
                                hour: 12,
                                minute: 0,
                                second: (n / 160) as u8,
                                millis: 0,
                            }),
                            date: None,
                            lat_std_mm: Some(1_200),
                            lon_std_mm: Some(900),
                            alt_std_mm: Some(2_500),
                            vel_ned_mm_s: None,
                        },
                        t_us: t_us - 430_000.min(t_us),
                        seq: n / 160,
                    });
                }
                let out = flight.step(&i);
                let mut records = [0u8; 256];
                let len = encode_step(&out, &mut records);
                outs.push((std::format!("{out:?}"), records[..len].to_vec(), out));
            }
            outs
        };
        let (without, with) = (run(&WITHOUT, false), run(&WITH, true));
        assert_eq!(without.len(), with.len());
        for (n, (a, b)) in without.iter().zip(&with).enumerate() {
            assert_eq!(a.0, b.0, "step {n}: outputs differ");
            assert_eq!(a.1, b.1, "step {n}: log records differ");
        }
        // The sequence is not trivial: it armed, and the motors ran with commands that change.
        let last = &with[with.len() - 1].2;
        assert!(last.armed && WITH.armed());
        let throttles: std::vec::Vec<_> =
            with.iter().filter_map(|o| o.2.motors.throttles()).collect();
        assert!(throttles.len() > 2000);
        assert!(throttles.windows(2).any(|w| w[0] != w[1]));
    }

    extern crate std;

    /// The sequence of `slow_sensor_samples_do_not_change_a_mahony_step` at step `n`, with the barometer, compass and
    /// GNSS samples (`mag_expected` gives the compass samples an expected field).
    fn slow_inputs(n: u32, mag_expected: Option<fw_core::sample::MagExpected>) -> FlightInputs {
        use fw_core::sample::{
            BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample, UtcTime,
        };
        let t_us = 10_000 + u64::from(n) * 1_245;
        let x = n as f32 * 0.01;
        let mut i = inputs(t_us);
        i.sensors.imu1.seq = n;
        i.sensors.imu1.reading.gyro_rads = [
            0.2 * libm::sinf(x),
            0.1 * libm::cosf(1.3 * x),
            0.05 * libm::sinf(0.7 * x),
        ];
        i.sensors.imu1.reading.accel_ms2 = [
            0.3 * libm::sinf(0.9 * x),
            -0.2 * libm::cosf(x),
            -9.81 + 0.1 * libm::sinf(x),
        ];
        let stick = |k: f32| {
            if n < 400 {
                0.0
            } else {
                0.3 * libm::sinf(k * x)
            }
        };
        i.rc = (n % 10 == 0).then_some(RcSample {
            sticks: Sticks {
                roll: stick(1.1),
                pitch: stick(0.8),
                throttle: if n < 400 { 0.0 } else { 0.5 + stick(0.5) },
                yaw: stick(1.7),
            },
            arm: n >= 200,
            switch6: Switch3::Low,
            source: fw_core::rc::RcSource::Crsf,
            mission: false,
            t_us,
        });
        i.sensors.baro = (n % 16 == 3).then_some(BaroSample {
            reading: BaroReading {
                pressure_pa: 101_325.0 - x,
                temperature_c: 25.0,
            },
            t_us: t_us - 500,
            seq: n / 16,
        });
        i.sensors.mag = (n % 8 == 5).then_some(MagSample {
            reading: MagReading {
                field_ut: [20.0 * libm::cosf(x), 20.0 * libm::sinf(x), 40.0],
                temperature_c: 30.0,
            },
            t_us: t_us - 300,
            seq: n / 8,
            expected: mag_expected,
        });
        i.sensors.gnss = (n % 160 == 7).then_some(GnssSample {
            reading: GnssReading {
                fix_type: 3,
                lat_e7: Some(254_000_000 + n as i32),
                lon_e7: Some(-801_000_000 - n as i32),
                alt_mm: Some(12_000),
                speed_mm_s: Some(n),
                course_cdeg: Some(9_000),
                sats_used: Some(12),
                hdop_x100: Some(80),
                vdop_x100: Some(120),
                time: Some(UtcTime {
                    hour: 12,
                    minute: 0,
                    second: (n / 160) as u8,
                    millis: 0,
                }),
                date: None,
                lat_std_mm: Some(1_200),
                lon_std_mm: Some(900),
                alt_std_mm: Some(2_500),
                vel_ned_mm_s: None,
            },
            t_us: t_us - 430_000.min(t_us),
            seq: n / 160,
        });
        i
    }

    /// The parts of a step output the flight uses and logs outside the vertical estimate: motors, armed, mode,
    /// setpoint, demand, allocation, attitude, rates, the tilt, heading, rates and NE flags, and the step's records
    /// with NAV's velocity-D and position-D bits cleared.
    fn flight_part(out: &StepOut) -> (std::string::String, std::vec::Vec<u8>) {
        let v = out.nav.valid;
        let text = std::format!(
            "{:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {} {} {} {} {}",
            out.t_us,
            out.motors,
            out.armed,
            out.mode,
            out.setpoint,
            out.demand,
            out.allocation,
            (
                out.nav.attitude.map(f32::to_bits),
                out.nav.rates_rads.map(f32::to_bits)
            ),
            out.nav.t_us,
            v.tilt,
            v.heading,
            v.rates,
            v.velocity_ne || v.position_ne,
        );
        let mut records = [0u8; 256];
        let len = encode_step(out, &mut records);
        records[39] &= !(16 | 64);
        (text, records[..len].to_vec())
    }

    /// Packet E2, item 1: the board's estimator `MahonyBaro` flies as `Mahony` did. The sequence of
    /// `slow_sensor_samples_do_not_change_a_mahony_step` (with the barometer, compass and GNSS samples) gives the same
    /// motors, setpoints, demands, allocations, attitude, rates and tilt flags, bit for bit, with either estimator; only
    /// the vertical estimate (velocity D, position D, their flags and sigmas) differs, and it does.
    #[test]
    fn mahony_baro_steps_as_mahony_bit_for_bit() {
        extern crate std;
        use crate::mahony_baro::MahonyBaro;
        static A: Handshake = Handshake::new();
        static B: Handshake = Handshake::new();
        let tuning = Tuning::DEFAULT;
        let mut mahony = Flight::new(
            Mahony::new(&tuning),
            CascadedPid::new(&tuning, 800.0),
            CardAllocator::new(),
            &tuning,
            &A,
        );
        let mut baro = Flight::new(
            MahonyBaro::new(&tuning),
            CascadedPid::new(&tuning, 800.0),
            CardAllocator::new(),
            &tuning,
            &B,
        );
        A.set_permit(true);
        B.set_permit(true);
        let (mut spins, mut vertical) = (0u32, 0u32);
        for n in 0..3000u32 {
            let i = slow_inputs(n, None);
            let (a, b) = (mahony.step(&i), baro.step(&i));
            assert_eq!(flight_part(&a), flight_part(&b), "step {n}");
            spins += u32::from(b.motors.throttles().is_some());
            vertical += u32::from(b.nav.valid.position_d && b.nav.sigma.is_some());
            assert!(a.nav.sigma.is_none());
        }
        assert!(A.armed() && B.armed());
        assert!(spins > 2000, "{spins}");
        assert!(vertical > 2000, "{vertical}");
        assert!(baro.estimator().vert().fused() > 100);
    }

    /// Packet E2, item 6: the shadow never changes the step. The board's flight code over the same sequence with no
    /// shadow, with the `Eskf` at the default tuning and with an `Eskf` of another tuning gives identical step outputs
    /// and records (every field's exact `Debug` text), while the two shadows' states differ.
    #[test]
    fn the_shadow_never_changes_the_step() {
        extern crate std;
        use crate::eskf::Eskf;
        use crate::mahony_baro::MahonyBaro;
        use fw_core::sample::MagExpected;
        static H: [Handshake; 3] = [Handshake::new(), Handshake::new(), Handshake::new()];
        let mut other = Tuning::DEFAULT;
        other.nav_gate_sigma *= 0.5;
        other.nav_baro_sigma_m *= 3.0;
        other.nav_predict_us *= 2;
        let expected = Some(MagExpected {
            strength_ut: libm::sqrtf(20.0 * 20.0 + 40.0 * 40.0),
            inclination_rad: libm::atanf(40.0 / 20.0),
            tol_strength_frac: 0.2,
            tol_inclination_rad: 0.2,
        });
        let run = |handshake: &'static Handshake, shadow: Option<Tuning>| {
            let tuning = Tuning::DEFAULT;
            let mut flight = Flight::new(
                MahonyBaro::new(&tuning),
                CascadedPid::new(&tuning, 800.0),
                CardAllocator::new(),
                &tuning,
                handshake,
            );
            let mut eskf = shadow.map(|t| std::boxed::Box::new(Eskf::new(&t).unwrap()));
            handshake.set_permit(true);
            let mut outs = std::vec::Vec::new();
            let mut navs = std::vec::Vec::new();
            for n in 0..3000u32 {
                let i = slow_inputs(n, expected);
                let (out, nav) = flight.step_with_shadow(&i, eskf.as_deref_mut(), |_| {});
                let mut records = [0u8; 256];
                let len = encode_step(&out, &mut records);
                outs.push((std::format!("{out:?}"), records[..len].to_vec()));
                navs.push(nav.map(|v| std::format!("{v:?}")));
            }
            (outs, navs)
        };
        let (plain, none) = run(&H[0], None);
        let (default, a) = run(&H[1], Some(Tuning::DEFAULT));
        let (changed, b) = run(&H[2], Some(other));
        assert!(none.iter().all(Option::is_none));
        for n in 0..plain.len() {
            assert_eq!(
                plain[n], default[n],
                "step {n}: the default shadow changed the step"
            );
            assert_eq!(
                plain[n], changed[n],
                "step {n}: the other shadow changed the step"
            );
        }
        assert!(a.iter().all(Option::is_some));
        assert!(a.iter().zip(&b).filter(|(x, y)| x != y).count() > 1000);
        assert!(H.iter().all(Handshake::armed));
    }

    /// NKF1 and NKF2 of one state: the instance, the horizon time, the valid bits, the sigmas, infinite without them.
    #[test]
    fn encode_nkf_writes_the_two_records() {
        use crate::gnc::NavSigma;
        let mut nav = NavState {
            t_us: 0x0102_0304_0506,
            attitude: [0.5, -0.5, 0.25, 0.75],
            rates_rads: [9.0; 3],
            velocity_ms: [0.1, -0.2, 0.3],
            position_m: [1.0, -2.0, 3.0],
            valid: Valid {
                tilt: true,
                velocity_d: true,
                position_d: true,
                ..Valid::NONE
            },
            sigma: Some(NavSigma {
                tilt_rad: 0.01,
                heading_rad: 0.02,
                velocity_ne_ms: 0.03,
                velocity_d_ms: 0.04,
                position_ne_m: 0.05,
                position_d_m: 0.06,
            }),
        };
        let f = |b: &[u8], at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        let mut buf = [0xEEu8; 120];
        assert_eq!(
            encode_nkf(&mut buf, 1, &nav, [0.001, 0.002, 0.003], [0.1, 0.2, 0.3]),
            113
        );
        assert_eq!(encode_nkf(&mut [0u8; 112], 1, &nav, [0.0; 3], [0.0; 3]), 0);
        let time = 0x0102_0304_0506u64.to_le_bytes();
        assert_eq!(buf[0..3], [0xA3, 0x95, 147]);
        assert_eq!(buf[3..11], time);
        assert_eq!(buf[11], 1);
        let q: [f32; 4] = core::array::from_fn(|k| f(&buf, 12 + 4 * k));
        assert_eq!(q, nav.attitude);
        let vp: [f32; 6] = core::array::from_fn(|k| f(&buf, 28 + 4 * k));
        assert_eq!(vp, [0.1, -0.2, 0.3, 1.0, -2.0, 3.0]);
        assert_eq!(buf[52], 1 | 16 | 64);
        assert_eq!(buf[53..56], [0xA3, 0x95, 148]);
        assert_eq!(buf[56..64], time);
        assert_eq!(buf[64], 1);
        let rest: [f32; 12] = core::array::from_fn(|k| f(&buf, 65 + 4 * k));
        assert_eq!(
            rest,
            [
                0.001, 0.002, 0.003, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06
            ]
        );
        assert_eq!(buf[113..], [0xEE; 7]);
        nav.sigma = None;
        encode_nkf(&mut buf, 0, &nav, [0.0; 3], [0.0; 3]);
        assert_eq!(buf[11], 0);
        assert!((0..6).all(|k| f(&buf, 65 + 24 + 4 * k) == f32::INFINITY));
    }

    /// Estimator, controller and allocator in one: each call appends its digit to `log`
    /// (1 estimate, 2 reset, 5 control, 6 allocate).
    #[derive(Clone, Copy)]
    struct Rec<'a> {
        log: &'a Cell<u32>,
        valid: Valid,
        accept: bool,
    }

    impl Rec<'_> {
        fn note(&self, digit: u32) {
            self.log.set(self.log.get() * 10 + digit);
        }
    }

    impl Estimator for Rec<'_> {
        fn update(&mut self, sensors: &Sensors) -> NavState {
            self.note(1);
            NavState {
                t_us: sensors.imu1.t_us,
                attitude: [1.0, 0.0, 0.0, 0.0],
                rates_rads: [0.0; 3],
                velocity_ms: [0.0; 3],
                position_m: [0.0; 3],
                valid: self.valid,
                sigma: None,
            }
        }
    }

    impl Controller for Rec<'_> {
        fn update(
            &mut self,
            _: &NavState,
            _: &Setpoint,
            _: &Saturation,
        ) -> Result<ThrustTorque, Refused> {
            self.note(5);
            if self.accept {
                Ok(ThrustTorque {
                    thrust_n: 1.0,
                    torque_nm: [0.0; 3],
                })
            } else {
                Err(Refused)
            }
        }

        fn reset(&mut self) {
            self.note(2);
        }
    }

    impl Allocator for Rec<'_> {
        fn allocate(&mut self, _: &ThrustTorque) -> Allocation {
            self.note(6);
            Allocation {
                levels: MotorLevels { level: [100; 4] },
                saturation: Saturation {
                    roll: false,
                    pitch: false,
                    yaw: false,
                    thrust_low: false,
                    thrust_high: false,
                },
            }
        }
    }

    #[test]
    fn stages_run_in_r1_order() {
        let attitude = Valid {
            tilt: true,
            rates: true,
            ..Valid::NONE
        };
        // (valid, controller accepts, calls, allocation made)
        let cases = [
            (Valid::NONE, true, 12, false), // covers fails: no controller, no allocator
            (attitude, false, 125, false),  // the controller refuses: no allocator
            (attitude, true, 1256, true),   // the full order
        ];
        for (valid, accept, calls, allocated) in cases {
            let log = Cell::new(0);
            let rec = Rec {
                log: &log,
                valid,
                accept,
            };
            let mut flight = Flight::new(rec, rec, rec, &Tuning::DEFAULT, &HANDSHAKE);
            let out = flight.step(&inputs(10));
            assert_eq!(log.get(), calls);
            assert_eq!(out.allocation.is_some(), allocated);
            assert_eq!(out.demand.is_ok(), allocated);
            // The gate comes last and has no stick report: Stop whatever the allocator said.
            assert_eq!(out.motors, MotorCommand::STOP);
        }
    }

    /// A6: the six records of one step, decoded by hand, field by field.
    #[test]
    fn encode_step_writes_the_six_records() {
        extern crate std;
        let out = StepOut {
            t_us: 0x0102_0304_0506,
            motors: MotorCommand::spinning([111, 222, 333, 444]),
            armed: true,
            selected: Mode::Angle,
            mode: Mode::Angle,
            nav: NavState {
                t_us: 0x0102_0304_0506,
                attitude: [0.5, -0.5, 0.25, 0.75],
                rates_rads: [0.1, -0.2, 0.3],
                velocity_ms: [9.0; 3],
                position_m: [9.0; 3],
                valid: Valid {
                    tilt: true,
                    rates: true,
                    position_d: true,
                    ..Valid::NONE
                },
                sigma: None,
            },
            setpoint: Setpoint::Attitude {
                attitude: [0.9, 0.1, -0.2, 0.3],
                heading_rate_rads: 1.5,
                thrust_n: 7.25,
            },
            demand: Ok(ThrustTorque {
                thrust_n: 7.5,
                torque_nm: [0.01, -0.02, 0.03],
            }),
            allocation: Some(Allocation {
                levels: MotorLevels {
                    level: [120, 130, 140, 150],
                },
                saturation: Saturation {
                    roll: false,
                    pitch: true,
                    yaw: true,
                    thrust_low: false,
                    thrust_high: true,
                },
            }),
            attitude_sp: [0.0; 4],
            rate: RateLog {
                target_rads: [1.0, 2.0, 3.0],
                p_nm: [0.5, 0.6, 0.7],
                i_nm: [-0.5, -0.6, -0.7],
                d_nm: [0.25, 0.125, 0.0625],
                rate_rads: [0.1, -0.25, 0.5],
                f1_hz: [100.0, 0.0, 120.0, 130.0],
                on: true,
            },
        };
        let mut buf = [0u8; 300];
        assert_eq!(encode_step(&out, &mut buf), 40 + 44 + 29 + 29 + 59 + 40);
        assert_eq!(encode_step(&out, &mut [0u8; 240]), 0);
        let f = |at: usize| f32::from_le_bytes(buf[at..at + 4].try_into().unwrap());
        let h = |at: usize| u16::from_le_bytes(buf[at..at + 2].try_into().unwrap());
        let fs = |at: usize, n: usize| (0..n).map(|i| f(at + 4 * i)).collect::<std::vec::Vec<_>>();
        let time = 0x0102_0304_0506u64.to_le_bytes();
        // NAV at 0: header, time, Q1..Q4, GX..GZ, Vld (tilt 1 + rates 4 + position_d 64).
        assert_eq!(buf[0..3], [0xA3, 0x95, 139]);
        assert_eq!(buf[3..11], time);
        assert_eq!(fs(11, 4), out.nav.attitude);
        assert_eq!(fs(27, 3), out.nav.rates_rads);
        assert_eq!(buf[39], 1 | 4 | 64);
        // SETP at 40: K 1, Q, RX RY 0, RZ the heading rate, Thr.
        assert_eq!(buf[40..43], [0xA3, 0x95, 140]);
        assert_eq!(buf[43..51], time);
        assert_eq!(buf[51], 1);
        assert_eq!(fs(52, 4), [0.9, 0.1, -0.2, 0.3]);
        assert_eq!(fs(68, 3), [0.0, 0.0, 1.5]);
        assert_eq!(f(80), 7.25);
        // DMD at 84: Ok, Thr, TX..TZ, Sat (pitch 2 + yaw 4 + thrust_high 16).
        assert_eq!(buf[84..87], [0xA3, 0x95, 141]);
        assert_eq!(buf[87..95], time);
        assert_eq!(buf[95], 1);
        assert_eq!(fs(96, 4), [7.5, 0.01, -0.02, 0.03]);
        assert_eq!(buf[112], 2 | 4 | 16);
        // MOT at 113: Arm, Mode (Angle 1), A0..A3, C0..C3.
        assert_eq!(buf[113..116], [0xA3, 0x95, 142]);
        assert_eq!(buf[116..124], time);
        assert_eq!(buf[124..126], [1, 1]);
        let levels: std::vec::Vec<u16> = (0..8).map(|i| h(126 + 2 * i)).collect();
        assert_eq!(levels, [120, 130, 140, 150, 111, 222, 333, 444]);
        // PIDT at 142: per axis Tar, P, I, D.
        assert_eq!(buf[142..145], [0xA3, 0x95, 154]);
        assert_eq!(buf[145..153], time);
        assert_eq!(
            fs(153, 12),
            [1.0, 0.5, -0.5, 0.25, 2.0, 0.6, -0.6, 0.125, 3.0, 0.7, -0.7, 0.0625]
        );
        // NTCH at 201: the rate the loop used, F0..F3, On.
        assert_eq!(buf[201..204], [0xA3, 0x95, 155]);
        assert_eq!(buf[204..212], time);
        assert_eq!(fs(212, 3), [0.1, -0.25, 0.5]);
        assert_eq!(fs(224, 4), [100.0, 0.0, 120.0, 130.0]);
        assert_eq!(buf[240], 1);
        assert!(buf[241..].iter().all(|&b| b == 0));
    }

    #[test]
    fn log_due_decimation_over_64_steps() {
        let count = |armed| {
            (0..64u32)
                .map(|n| log_due(n, armed))
                .fold((0, 0, 0), |(i, b, e), d| {
                    (
                        i + u32::from(d.imu),
                        b + u32::from(d.block),
                        e + u32::from(d.esc),
                    )
                })
        };
        assert_eq!(count(true), (64, 64, 64));
        assert_eq!(count(false), (64, 8, 8));
        assert_eq!(
            log_due(0, false),
            LogDue {
                imu: true,
                block: true,
                esc: true
            }
        );
        assert_eq!(
            log_due(1, false),
            LogDue {
                imu: true,
                block: false,
                esc: false
            }
        );
        assert_eq!(
            log_due(4, true),
            LogDue {
                imu: true,
                block: true,
                esc: true
            }
        );
        assert_eq!(
            log_due(5, true),
            LogDue {
                imu: true,
                block: true,
                esc: true
            }
        );
    }

    #[test]
    fn log_due_disarmed_imu_every_step_block_every_8th() {
        for k in 0..64u32 {
            let d = log_due(k, false);
            assert!(d.imu);
            assert_eq!(d.block, k % 8 == 0);
            assert_eq!(d.esc, k % 8 == 0);
            let a = log_due(k, true);
            assert!(a.imu && a.block);
            assert!(a.esc);
        }
    }

    /// P-G2 T6 (Amendment G, G1 "Resets and the controller"): a yaw reset of the core is invisible to the Angle
    /// controller. `Flight` with the steering `EskfSteer` over a resting vehicle with the sticks off centre, run three
    /// times: plain; with the compass turned 90 deg from 20 s, which makes the core reset its yaw about 5 s later; and
    /// (the control) plain with a 2 deg tilt offset put into the output at that same step.
    /// Returns every step's demand after 6 s and the step of the core's yaw reset.
    fn t6_run(
        handshake: &'static Handshake,
        mag_turn: bool,
        tilt_at: Option<usize>,
    ) -> (std::vec::Vec<ThrustTorque>, Option<usize>) {
        extern crate std;
        use crate::eskf::Eskf;
        use crate::eskf::tests::World;
        use crate::eskf_out::EskfSteer;
        let tuning = Tuning::DEFAULT;
        let mut core = Eskf::new(&tuning).unwrap();
        // This run must stay armed (the Angle demand is asserted), and G6-B resets no yaw while armed: the test switch
        // restores the compass reset after the timeout, through the same `mag_job` -> `yaw_set` path.
        core.armed_yaw_reset = true;
        let steer = EskfSteer::new(&mut core, &tuning).unwrap();
        let mut flight = Flight::new(steer, CascadedPid::new(&tuning, 800.0), CardAllocator::new(), &tuning, handshake);
        handshake.set_permit(true);
        let mut world = World::new(&tuning);
        world.yaw = 0.4;
        let (mut demands, mut reset_at) = (std::vec::Vec::new(), None);
        for k in 0..32_000_000 / 1_250usize {
            world.mag_rot = if mag_turn && world.t_us() >= 20_000_000 { core::f64::consts::FRAC_PI_2 } else { 0.0 };
            let sensors = world.step();
            let t_us = sensors.imu1.t_us;
            let rc = (k % 10 == 0).then_some(RcSample {
                sticks: Sticks {
                    roll: 0.3,
                    pitch: -0.2,
                    throttle: if t_us < 3_000_000 { 0.0 } else { 0.5 },
                    yaw: 0.1,
                },
                arm: t_us >= 2_000_000,
                switch6: Switch3::Low,
                source: fw_core::rc::RcSource::Crsf,
                mission: false,
                t_us,
            });
            if Some(k) == tilt_at {
                flight.estimator.inject_tilt(2.0_f32.to_radians());
            }
            let before = flight.estimator.core().stats().yaw_resets;
            let out = flight.step(&FlightInputs { sensors, rc, erpm: [None; LINES] });
            if flight.estimator.core().stats().yaw_resets > before && t_us > 20_000_000 {
                reset_at = Some(k);
            }
            if t_us > 6_000_000 {
                assert!(out.armed && out.mode == Mode::Angle, "step {k}");
                demands.push(out.demand.expect("a demand"));
            }
        }
        (demands, reset_at)
    }

    #[test]
    fn t6_a_core_yaw_reset_does_not_change_the_angle_demand_and_a_tilt_offset_does() {
        static H: [Handshake; 3] = [Handshake::new(), Handshake::new(), Handshake::new()];
        let first = 6_000_000 / 1_250 + 1;
        let (plain, none) = t6_run(&H[0], false, None);
        assert_eq!(none, None);
        let (turned, reset) = t6_run(&H[1], true, None);
        let k = reset.expect("the compass turn must make the core reset its yaw");
        let (offset, _) = t6_run(&H[2], false, Some(k));
        assert_eq!(plain.len(), turned.len());
        let diff = |a: &[ThrustTorque], b: &[ThrustTorque]| {
            a.iter().zip(b).fold((0.0f32, 0.0f32), |(t, q): (f32, f32), (x, y)| {
                let dq = (0..3).fold(0.0f32, |m, i| m.max((x.torque_nm[i] - y.torque_nm[i]).abs()));
                (t.max((x.thrust_n - y.thrust_n).abs()), q.max(dq))
            })
        };
        let scale = plain.iter().fold(0.0f32, |m, d| d.torque_nm.iter().fold(m, |m, v| m.max(v.abs())));
        let at = k - first;
        let (pre_t, pre_q) = diff(&plain[..at], &turned[..at]);
        let (post_t, post_q) = diff(&plain[at..], &turned[at..]);
        let (off_t, off_q) = diff(&plain[at..], &offset[at..]);
        std::println!(
            "t6: yaw reset at step {k}; largest torque in the run {scale:.4} N.m; before the reset: thrust {pre_t:.2e} N, torque {pre_q:.2e} N.m; reset step and after: thrust {post_t:.2e} N, torque {post_q:.2e} N.m; control (2 deg tilt offset): thrust {off_t:.2e} N, torque {off_q:.2e} N.m"
        );
        assert!(scale > 0.01, "the demand must be non-trivial: {scale}");
        // Before the reset the runs differ only by a few ulp (the turned compass is gated, which the core notes in its
        // timers); after it, by rounding: the heading twist is rebuilt from a quaternion the reset rotated by 90 deg,
        // 1e-4 of the demand's scale (80 ulp of it measured, the control is 600 times that).
        assert!(pre_t == 0.0 && pre_q <= 1e-6, "before the reset: thrust {pre_t}, torque {pre_q}");
        assert!(post_q <= 1e-4 * scale && post_t <= 1e-4, "thrust {post_t}, torque {post_q}");
        assert!(off_q > 100.0 * post_q.max(1e-9) && off_q > 0.01 * scale, "the control must change the demand: {off_q}");
    }

    /// One step of a `Flight` steered by `SteerMix` in Altitude, as the tests M2 and M3 record it.
    #[derive(Clone, Copy, Debug)]
    struct MixRec {
        t_us: u64,
        armed: bool,
        mode: Mode,
        tilt: bool,
        vd: bool,
        pd: bool,
        d: f32,
        target: Option<f32>,
        integrator: f32,
        /// ESKF2's reset of this step moved D or v_D.
        z_moved: bool,
        /// ESKF2's re-initialisations so far.
        z_reinits: u32,
    }

    /// `Flight` steered by `SteerMix` over a vehicle climbing at `climb_ms` (yaw 0.4, level), armed from 2 s, Altitude
    /// selected, the throttle stick at the centre from 3 s, 32 s. With `baro_off` the barometer reads 40 m too high
    /// from 20 s; `poison_at` is the step before which ESKF2's nominal velocity is made non-finite.
    fn mix_run(handshake: &'static Handshake, baro_off: bool, poison_at: Option<usize>, climb_ms: f64) -> std::vec::Vec<MixRec> {
        extern crate std;
        use crate::eskf::tests::{World, poison, pressure_at};
        use crate::eskf::{Aiding, Eskf, EskfZ};
        use crate::eskf_out::{EskfSteer, SteerMix};
        let tuning = Tuning::DEFAULT;
        let (mut core, mut vz) = (Eskf::new(&tuning).unwrap(), EskfZ::with_aiding(&tuning, Aiding::ImuBaro).unwrap());
        let mix = SteerMix::new(EskfSteer::new(&mut core, &tuning).unwrap(), &mut vz);
        let mut flight = Flight::new(mix, CascadedPid::new(&tuning, 800.0), CardAllocator::new(), &tuning, handshake);
        handshake.set_permit(true);
        let mut world = World::new(&tuning);
        world.yaw = 0.4;
        world.v[2] = -climb_ms;
        let mut recs = std::vec::Vec::new();
        for k in 0..32_000_000 / 1_250usize {
            let mut sensors = world.step();
            let t_us = sensors.imu1.t_us;
            if let Some(b) = sensors.baro.as_mut().filter(|_| baro_off && t_us >= 20_000_000) {
                b.reading.pressure_pa = pressure_at(40.0 - world.p[2]);
            }
            let rc = (k % 10 == 0).then_some(RcSample {
                sticks: Sticks { roll: 0.0, pitch: 0.0, throttle: if t_us < 3_000_000 { 0.0 } else { 0.5 }, yaw: 0.0 },
                arm: t_us >= 2_000_000,
                switch6: Switch3::Middle,
                source: fw_core::rc::RcSource::Crsf,
                mission: false,
                t_us,
            });
            if Some(k) == poison_at {
                poison(flight.estimator.vert_mut());
            }
            let out = flight.step(&FlightInputs { sensors, rc, erpm: [None; LINES] });
            let z = flight.estimator.vert();
            recs.push(MixRec {
                t_us,
                armed: out.armed,
                mode: out.mode,
                tilt: out.nav.valid.tilt,
                vd: out.nav.valid.velocity_d,
                pd: out.nav.valid.position_d,
                d: out.nav.position_m[2],
                target: flight.shaper.target_d,
                integrator: flight.controller.vertical_integrator(),
                z_moved: z.last_reset().is_some_and(|r| r.dp[2] != 0.0 || r.dv[2] != 0.0),
                z_reinits: z.stats().reinits,
            });
        }
        recs
    }

    /// P-G1m M2 (Amendment G-1, V3): the barometer 40 m too high from 20 s, armed. ESKF2's height reset moves D by about
    /// 40 m in one step: the vertical flags are false in exactly that step, the effective mode is Angle for it, and
    /// Altitude captures the new D (target = D, integrator 0) on the next step.
    #[test]
    fn m2_a_height_reset_of_eskf2_drops_the_vertical_flags_for_that_step_and_altitude_recaptures_the_new_height() {
        static H: Handshake = Handshake::new();
        let recs = mix_run(&H, true, None, 0.0);
        let from = recs.iter().position(|r| r.t_us >= 8_000_000).unwrap();
        let moved: std::vec::Vec<usize> = (from..recs.len()).filter(|&k| recs[k].z_moved).collect();
        assert_eq!(moved.len(), 1, "ESKF2's height reset, once: {moved:?}");
        let k = moved[0];
        assert!(recs[k].t_us > 20_000_000);
        std::println!(
            "m2: ESKF2 height reset at step {k} ({} us); D {:.3} -> {:.3} -> {:.3} m; target before {:?}, at {:?}, after {:?}; integrator before {:.5}, at {:.5}, after {:.5}",
            recs[k].t_us, recs[k - 1].d, recs[k].d, recs[k + 1].d, recs[k - 1].target, recs[k].target, recs[k + 1].target,
            recs[k - 1].integrator, recs[k].integrator, recs[k + 1].integrator
        );
        for (i, r) in recs.iter().enumerate().skip(from) {
            assert!(r.armed && r.tilt, "step {i}");
            // The flags are false in the reset's step and in no other.
            assert_eq!((r.vd, r.pd), (i != k, i != k), "step {i}: flags");
            // Altitude in every step but that one, which is Angle.
            assert_eq!(r.mode, if i == k { Mode::Angle } else { Mode::Altitude }, "step {i}: mode");
        }
        // The reset moved D by the barometer's 40 m; Angle holds no target; the next step captures the new D, integrator 0.
        assert!((recs[k].d - recs[k - 1].d).abs() > 30.0, "D moved by {}", recs[k].d - recs[k - 1].d);
        assert_eq!(recs[k].target, None);
        assert_eq!(recs[k + 1].target, Some(recs[k + 1].d));
        assert_eq!(recs[k + 1].integrator, 0.0);
        // Before it the target sat at the old height (within the lead cap of D), its integrator was wound, not 0.
        assert!(recs[k - 1].target.is_some_and(|t| (t - recs[k - 1].d).abs() < 2.0));
        assert!(recs[k - 1].integrator != 0.0);
    }

    /// P-G1m M3 (Amendment G-1, I6): ESKF2 forced non-finite at 15 s while armed in Altitude. It re-initialises that
    /// step (flags false) while the output predictor's tilt stays true: the effective mode goes 2 -> 1, and back to 2
    /// when ESKF2 has re-aligned and re-initialised its height, with a fresh capture.
    #[test]
    fn m3_a_non_finite_eskf2_drops_altitude_to_angle_while_tilt_stays_true_and_it_recaptures_after_the_realignment() {
        static H: Handshake = Handshake::new();
        let at = 15_000_000 / 1_250;
        let recs = mix_run(&H, false, Some(at), 0.5);
        let from = recs.iter().position(|r| r.t_us >= 8_000_000).unwrap();
        for (i, r) in recs.iter().enumerate().skip(from) {
            assert!(r.armed && r.tilt, "step {i}: armed {} tilt {}", r.armed, r.tilt);
        }
        assert!(recs[from..at].iter().all(|r| r.mode == Mode::Altitude && r.vd && r.pd));
        assert_eq!(recs[at - 1].z_reinits, 0);
        assert_eq!(recs[at].z_reinits, 1, "the poisoned step re-initialises ESKF2");
        assert!(recs[at].mode == Mode::Angle && recs[at].tilt && !recs[at].vd && !recs[at].pd, "the poisoned step: {:?}", recs[at]);
        let back = (at..recs.len()).find(|&k| recs[k].mode == Mode::Altitude).expect("Altitude must return");
        std::println!(
            "m3: poisoned at step {at} ({} us); Altitude back at step {back} ({} us), {:.3} s later; ESKF2 reinits {}; target at {:?}, D {:.3} m, integrator {:.5}; ESKF2 height reset in step {:?}",
            recs[at].t_us, recs[back].t_us, (recs[back].t_us - recs[at].t_us) as f64 * 1e-6, recs[back].z_reinits, recs[back].target, recs[back].d, recs[back].integrator,
            (at..recs.len()).find(|&k| recs[k].z_moved)
        );
        // ESKF2's height-init reset (D from its zeroed state to the barometer's height) is in the step before the return.
        assert_eq!((at..recs.len()).filter(|&k| recs[k].z_moved).collect::<std::vec::Vec<_>>(), [back - 1]);
        assert!(back > at + 1);
        for r in &recs[at..back] {
            assert!(r.mode == Mode::Angle && r.tilt && !(r.vd && r.pd) && r.target.is_none());
        }
        assert!(recs[back].vd && recs[back].pd);
        assert_eq!(recs[back].target, Some(recs[back].d), "a fresh capture");
        assert_eq!(recs[back].integrator, 0.0);
        assert!(recs[back..].iter().all(|r| r.mode == Mode::Altitude), "Altitude stays once it is back");
        assert_eq!(recs[recs.len() - 1].z_reinits, 1);
    }

    /// One step of the P-2 run.
    #[derive(Clone, Copy, Debug)]
    struct P2Rec {
        t_us: u64,
        mode: Mode,
        ne: bool,
        target_ne: Option<[f32; 2]>,
        xy_i: [f32; 2],
        resets: u32,
    }

    /// Addendum P-2: `Flight` with `EskfSteer` in Position (selected from 2 s, hover collective), the vehicle moving north at
    /// 2 m/s, no GNSS epoch from 20 s to 28 s (longer than the 5 s timeout). From 34 s it brakes to rest.
    /// `keep` is the rule before P-2 (the control).
    fn p2_run(handshake: &'static Handshake, keep: bool) -> std::vec::Vec<P2Rec> {
        extern crate std;
        use crate::eskf::Eskf;
        use crate::eskf::tests::{Fault, World};
        use crate::eskf_out::EskfSteer;
        let tuning = Tuning::DEFAULT;
        let mut core = Eskf::new(&tuning).unwrap();
        core.keep_started_at_timeout = keep;
        let steer = EskfSteer::new(&mut core, &tuning).unwrap();
        let mut flight = Flight::new(steer, CascadedPid::new(&tuning, 800.0), CardAllocator::new(), &tuning, handshake);
        handshake.set_permit(true);
        // The latch is not under test (as in position_tests): forced, the vehicle is taken to be in the air.
        flight.airborne_forced = true;
        let mut world = World::new(&tuning);
        world.yaw = 0.4;
        world.v = [2.0, 0.0, 0.0];
        world.gps_vel = true;
        world.fault = Fault::Gone(20_000_000, 28_000_000);
        let mut recs = std::vec::Vec::new();
        for k in 0..45_000_000 / 1_250usize {
            if world.t_us() >= 34_000_000 && world.v[0] > 0.0 {
                world.a = [-2.0, 0.0, 0.0];
            } else if world.t_us() >= 34_000_000 {
                world.a = [0.0; 3];
                world.v[0] = 0.0;
            }
            let sensors = world.step();
            let t_us = sensors.imu1.t_us;
            let rc = (k % 10 == 0).then_some(RcSample {
                sticks: Sticks { roll: 0.0, pitch: 0.0, throttle: if t_us < 3_000_000 { 0.0 } else { 0.5 }, yaw: 0.0 },
                arm: t_us >= 2_000_000,
                switch6: Switch3::High,
                source: fw_core::rc::RcSource::Crsf,
                mission: false,
                t_us,
            });
            let out = flight.step(&FlightInputs { sensors, rc, erpm: [None; LINES] });
            recs.push(P2Rec {
                t_us,
                mode: out.mode,
                ne: out.nav.valid.position_ne && out.nav.valid.velocity_ne,
                target_ne: flight.shaper.target_ne,
                xy_i: flight.controller.horizontal_integrator(),
                resets: flight.estimator.core().stats().gps_resets,
            });
        }
        recs
    }

    #[test]
    fn p2_a_gnss_outage_longer_than_the_timeout_takes_position_to_altitude_and_back_the_step_after_the_reset_with_a_fresh_capture() {
        static H: [Handshake; 2] = [Handshake::new(), Handshake::new()];
        let recs = p2_run(&H[0], false);
        let after = |t: u64| recs.iter().position(|r| r.t_us >= t).unwrap();
        // Position before the outage (moving: no target yet, the integrator wound).
        let (k0, k_end) = (after(15_000_000), after(28_000_000));
        assert!(recs[after(10_000_000)..after(20_000_000)].iter().all(|r| r.mode == Mode::Position && r.ne && r.target_ne.is_none()));
        let drop = (k0..k_end).find(|&k| recs[k].mode != Mode::Position).expect("Position ends at the timeout");
        assert_eq!(recs[drop - 1].mode, Mode::Position);
        assert_eq!(recs[drop].mode, Mode::Altitude, "Eff 3 -> 2 at the drop: {:?}", recs[drop]);
        assert!(!recs[drop].ne && recs[drop - 1].ne, "the NE flags drop in the same step");
        assert!(recs[drop - 1].xy_i != [0.0, 0.0], "the integrator was wound before the drop");
        let reset = (k_end..recs.len()).find(|&k| recs[k].resets > recs[k - 1].resets).expect("a Src 9 reset after the outage");
        // No flicker: Altitude from the drop to the reset step inclusive, Position the step after, NE flags false in the reset step only.
        assert!(recs[drop..=reset].iter().all(|r| r.mode == Mode::Altitude && !r.ne), "Altitude and NE false from the drop to the reset");
        assert!(recs[reset + 1].mode == Mode::Position && recs[reset + 1].ne, "Eff 2 -> 3 the step after the reset");
        assert!(recs[reset + 1].target_ne.is_none(), "no target while moving");
        assert_eq!(recs[reset + 1].xy_i, [0.0, 0.0], "a fresh Position entry: the horizontal integrator is 0");
        // Moving: no capture until the vehicle is stopped.
        let stop = recs.iter().position(|r| r.t_us > 34_000_000 && r.target_ne.is_some()).expect("captured once stopped");
        assert!(recs[reset + 1..stop].iter().all(|r| r.mode == Mode::Position && r.target_ne.is_none()));
        std::println!(
            "p2: drop at step {drop} ({} us), reset at step {reset} ({} us, {:.3} s after the outage ended), capture at step {stop} ({} us)",
            recs[drop].t_us, recs[reset].t_us, (recs[reset].t_us - 28_000_000) as f64 * 1e-6, recs[stop].t_us
        );
        // The control, the rule before P-2: no reset at the first epoch back.
        let old = p2_run(&H[1], true);
        let old_reset = (k_end..old.len()).find(|&k| old[k].resets > old[k - 1].resets);
        std::println!("p2 control (started kept): reset after the outage {:?}", old_reset.map(|k| old[k].t_us));
        assert!(old_reset.is_none_or(|k| old[k].t_us > recs[reset].t_us + 1_000_000), "the control resets later or never");
    }
}

#[cfg(test)]
mod althold_tests;

#[cfg(test)]
mod position_tests;

#[cfg(test)]
mod notch_tests;

#[cfg(test)]
mod gain_tests;

#[cfg(test)]
mod handsoff_tests;

#[cfg(test)]
mod crash_tests;

#[cfg(test)]
mod n14_tests;

#[cfg(test)]
mod station_tests;

#[cfg(test)]
mod auto_tests;

#[cfg(test)]
mod batt_tests;

#[cfg(test)]
mod descent_tests;
