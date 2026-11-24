//! The hands-off state machine (decision-mavlink-mode-20261123, section 9 Amendment A as amended by section 10: A1,
//! A2', A3, A6, A7, A9, A10, A10' and the state table; by section 11 Amendment B, B1-B9; and by section 13 Amendment C,
//! C1-C3; and by section 14 Amendment D, WR1, WR2 and `return_built`). Pure: no clock, no log, no statics, no allocation; every time
//! comes in as an argument. Each flight step the caller hands it the facts ([`Inputs`]) and gets the state and what the
//! flight code must do ([`Output`]). Not called by the flight step yet (packets H2b, H2c wire it).

use crate::tuning::Tuning;
use fw_core::rc::{RcSample, Switch3};

/// Link loss: link age above this in HOLD gives RETURN (home and T3) or LAND, us. Decision section 9, A1 (line 158):
/// "link age > 10 s in HOLD -> RETURN"; Numbers "link loss 10 s", owner.
pub const LINK_LOSS_US: u64 = 10_000_000;

/// Station lost: no station HEARTBEAT on the radio link for longer than this pauses AUTO into HOLD with the place kept,
/// us; Start needs one within it. Decision section 9, A1 (line 161) and Numbers "AUTO pause on station loss 5 s".
pub const STATION_LOST_US: u64 = 5_000_000;

/// The largest horizontal distance from home of the vehicle and of every waypoint at Start, m; more is DENIED.
/// Decision section 9, A9 (line 213): "within 1000 m horizontally of home"; owner, section 8 Q9 "1km".
pub const HOME_RANGE_M: f32 = 1000.0;

/// A tier drop (position, heading or height) takes effect on state exits and the stick-loss entry only after it has
/// lasted this long without a break, us; an upgrade takes effect at once. Decision section 11, B5: ArduPilot
/// EKF_CHECK_ITERATIONS_MAX, "1 second ... of bad variances", ekf_check.cpp:10-11 (owner's approval pending, Q-B1).
pub const TIER_DEBOUNCE_US: u64 = 1_000_000;

/// D_max, m: the largest horizontal excursion at which LAND must have started (decision-heading-monitor-20261123, Q-H1;
/// H12 item 4, H13 (2)).
pub const D_MAX_M: f32 = 10.0;

/// The cap on the horizontal speed limit of every hands-off state, m/s: the owner-approved default of `PILOT_SPEED_XY`
/// (`Tuning::pilot_speed_xy_ms`; H13 (2)).
pub const SPEED_XY_CAP_MS: f32 = Tuning::DEFAULT.pilot_speed_xy_ms;

/// The horizontal speed limit of the hands-off states for `PILOT_SPEED_XY` in force `in_force_ms`, m/s: v_ho =
/// min(`in_force_ms`, `SPEED_XY_CAP_MS`) (H13 (2)).
pub fn speed_limit_xy_ms(in_force_ms: f32) -> f32 {
    in_force_ms.min(SPEED_XY_CAP_MS)
}

/// The hold-integrity bound X for `PILOT_SPEED_XY` in force `in_force_ms`, m: a horizontal distance between the
/// navigation position and the target the position controller tracks above this sets the track fault, and so does a
/// distance from the anchor above X + b (H12 item 4, H13 (1), (2)). X = D_max - v_ho t_B5: `D_MAX_M`, v_ho
/// [`speed_limit_xy_ms`], t_B5 the 1.0 s debounce (`TIER_DEBOUNCE_US`); 7 m at 3 m/s, 5 m at 5 m/s and above.
pub fn track_fault_m(in_force_ms: f32) -> f32 {
    D_MAX_M - speed_limit_xy_ms(in_force_ms) * (TIER_DEBOUNCE_US as f32 / 1e6)
}

/// The effective mode (link-contract section 10 codes 1 to 7; BLIND 8, not yet in the contract).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// A fresh CRSF report and the pilot latch set: today's modes.
    Pilot,
    /// A fresh CRSF report in a hands-off state: the ladder at High with centred synthetic sticks until A2' hands over.
    Takeover,
    /// Hands-off Position hold, centred sticks.
    Hold,
    /// Flying the stored list.
    Auto,
    /// Climb to the latched return height, then a straight leg home.
    Return,
    /// Descend to touchdown; the existing landing latch disarms.
    Land,
    /// Disarmed, motors stopped (or sticks lost on the ground: the gate disarms).
    Stop,
    /// LAND flown on the debounced T1 (tilt and rates only; B6). Control law UNKNOWN (decision U-1): reported only.
    Blind,
}

impl State {
    /// The HEARTBEAT `custom_mode` code: link-contract section 10 (1 to 7); BLIND 8 (lead's ruling, contract later).
    pub fn code(self) -> u8 {
        match self {
            State::Pilot => 1,
            State::Takeover => 2,
            State::Hold => 3,
            State::Auto => 4,
            State::Return => 5,
            State::Land => 6,
            State::Stop => 7,
            State::Blind => 8,
        }
    }

    /// TAKEOVER, HOLD, AUTO, RETURN, LAND and BLIND: RC loss must not disarm, and no real stick reaches the shaper.
    fn is_hands_off(self) -> bool {
        !matches!(self, State::Pilot | State::Stop)
    }
}

/// The validity tier (A3), evaluated by the caller every step from the flags. Inside the module T3 is B4's T3':
/// position covered and `heading_ok`; T3 without `heading_ok` counts as T2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tier {
    /// The position is covered (`Setpoint::Position`'s needs; includes the height).
    T3,
    /// The height is covered, the position is not.
    T2,
    /// Tilt and rates only.
    T1,
}

/// A station flight command (link-contract section 10), its parameters already checked by the caller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// MAV_CMD_MISSION_START.
    Start,
    /// MAV_CMD_DO_PAUSE_CONTINUE, param1 0.
    Pause,
    /// MAV_CMD_DO_SET_MODE, custom mode HOLD.
    Abort,
    /// MAV_CMD_DO_SET_MODE, custom mode LAND.
    Land,
    /// MAV_CMD_NAV_RETURN_TO_LAUNCH (C3).
    Return,
}

/// The link a command arrived on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// The radio link (the transmitter's backpack).
    Radio,
    /// USB.
    Usb,
}

/// The highest setpoint level this step's flags cover (B5, I12): the flight code must never demand more.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Demand {
    /// T3': the position is covered and `heading_ok`.
    Position,
    /// The height is covered.
    Altitude,
    /// Tilt and rates only: Angle with centred synthetic throttle when hands-off.
    Angle,
}

/// LAND's horizontal law, chosen every step from this step's facts (decision-heading-monitor-20261123, H14 (1)).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LandLaw {
    /// No track fault and this step's T3' (so also the debounced one): hold the horizontal position (B4).
    Hold,
    /// The track fault and this step's T3': `Setpoint::Position` at this step's position with no velocity feed-forward,
    /// on every step, never captured.
    Brake,
    /// Otherwise: level (Altitude, or Angle when the height is not covered).
    Level,
}

/// A pending station command and the link it came on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StationCommand {
    /// The command.
    pub command: Command,
    /// Where it came from.
    pub link: Link,
}

/// What the caller knows of the stored mission (core 0's committed set as core 1 has it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MissionFacts {
    /// The plan id of the stored list; `None` when nothing is stored.
    pub plan_id: Option<u32>,
    /// The stored list has at least one waypoint (count 2 or more).
    pub has_waypoints: bool,
    /// Every stored z (waypoints and return) is inside the height limits in force now.
    pub z_within_limits: bool,
    /// The largest horizontal distance of any stored waypoint from home, m; `None` when unknown.
    pub max_waypoint_distance_m: Option<f32>,
    /// The stored return height above home, m; `None` when no list is stored.
    pub return_height_m: Option<f32>,
}

/// The facts of one flight step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inputs {
    /// Board time, us.
    pub now_us: u64,
    /// FS_HOLD_EN's present value. Taken into force on disarmed steps only (B9); in force false: only PILOT and STOP,
    /// `hands_off` always false.
    pub fs_hold_en: bool,
    /// The newest CRSF stick report (not necessarily new this step); `None` before the first. Its sticks are never
    /// read here.
    pub crsf: Option<RcSample>,
    /// When the newest station HEARTBEAT was received on the radio link, us. USB heartbeats are never given (A1).
    pub last_radio_heartbeat_us: Option<u64>,
    /// When the newest RADIO_STATUS was received, us. Display only: it feeds no rule here (A1).
    pub last_radio_status_us: Option<u64>,
    /// The gate's armed state.
    pub armed: bool,
    /// The airborne latch.
    pub airborne: bool,
    /// The validity tier.
    pub tier: Tier,
    /// Steering north and east from the origin, m, and whether they may be used (`position_ne`).
    pub position_ne_m: [f32; 2],
    /// `position_ne`.
    pub position_ne_valid: bool,
    /// Velocity north and east, m/s (the estimate's), and whether they may be used (`velocity_ne`).
    pub velocity_ne_ms: [f32; 2],
    /// `velocity_ne`.
    pub velocity_ne_valid: bool,
    /// Steering down from the origin, m (T3 implies it is valid).
    pub position_d_m: f32,
    /// `position_d`.
    pub position_d_valid: bool,
    /// A station command to answer this step.
    pub command: Option<StationCommand>,
    /// The stored mission.
    pub mission: MissionFacts,
    /// The guidance reached the last waypoint (AUTO).
    pub last_waypoint_reached: bool,
    /// The guidance is within GDN_RADIUS of home (RETURN arrival).
    pub home_reached: bool,
    /// The heading monitor reports OK.
    pub heading_ok: bool,
    /// The battery is known not to be critical.
    pub battery_ok: bool,
    /// The battery is critical.
    pub battery_critical: bool,
    /// The RETURN law exists (Amendment D). False: no home latches, so RETURN cannot be reached and link loss lands
    /// in place.
    pub return_built: bool,
    /// The horizontal distance between the navigation position and the Position target the controller tracked at the
    /// previous step, both of that step, m; `None` when the previous step tracked no Position target. Over X
    /// ([`track_fault_m`]), or not a number, it sets the track fault when that step's state was covered (H12).
    pub track_error_m: Option<f32>,
    /// `PILOT_SPEED_XY` in force (latched at arming), m/s: gives v_ho and X (H13 (2)).
    pub speed_xy_ms: f32,
}

/// Why a command was DENIED: the first condition that failed, in this order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DenyReason {
    /// The command did not arrive on the radio link.
    NotRadioLink,
    /// A CRSF stick report is fresh.
    CrsfFresh,
    /// FS_HOLD_EN is 0.
    HandsOffDisabled,
    /// The state does not take this command.
    NotInThisState,
    /// A command into HOLD or RETURN (and Abort in BLIND): the debounced tier is not T3' (B2, B6, C3). Start: this
    /// step's tier is not T3 (checked after `HeadingNotOk`, B4).
    NotT3,
    /// A command into HOLD or RETURN: the battery is critical, so the state would land at once (B2, C3).
    BatteryCritical,
    /// A command into HOLD: the link is lost (`LINK_LOSS_US`), so HOLD would leave at once (B2).
    LinkLost,
    /// Start: no station HEARTBEAT on the radio link within `STATION_LOST_US`.
    StationAbsent,
    /// Start: no list with a waypoint is stored.
    NoWaypoints,
    /// Start: a stored z is outside the height limits in force.
    HeightOutsideLimits,
    /// Start or Return: home is not set.
    NoHome,
    /// Start: the vehicle is more than `HOME_RANGE_M` from home, or its position is not valid.
    VehicleBeyondRange,
    /// Start: the waypoints' largest distance from home is unknown or not finite.
    WaypointDistanceUnknown,
    /// Start: a waypoint is more than `HOME_RANGE_M` from home.
    WaypointBeyondRange,
    /// Start: the heading monitor is not OK, or the track fault is latched (H14).
    HeadingNotOk,
    /// Start: the battery is not known to be non-critical.
    BatteryNotOk,
    /// Start: the last CRSF report before the sticks were lost did not have arm ON (C2).
    ArmSwitchOff,
    /// Start: the last CRSF report before the sticks were lost did not have switch6 High (A10').
    SwitchNotHigh,
}

impl DenyReason {
    /// The reason code for COMMAND_ACK.result_param2 (C3; link-contract section 10 reason table, MARV-GCS 58103a4);
    /// 0 is reserved for unknown.
    pub fn code(self) -> u8 {
        match self {
            DenyReason::NotRadioLink => 1,
            DenyReason::CrsfFresh => 2,
            DenyReason::HandsOffDisabled => 3,
            DenyReason::NotInThisState => 4,
            DenyReason::StationAbsent => 5,
            DenyReason::NoWaypoints => 6,
            DenyReason::HeightOutsideLimits => 7,
            DenyReason::NoHome => 8,
            DenyReason::VehicleBeyondRange => 9,
            DenyReason::WaypointDistanceUnknown => 10,
            DenyReason::WaypointBeyondRange => 11,
            DenyReason::HeadingNotOk => 12,
            DenyReason::NotT3 => 13,
            DenyReason::BatteryNotOk => 14,
            DenyReason::ArmSwitchOff => 15,
            DenyReason::SwitchNotHigh => 16,
            DenyReason::BatteryCritical => 17,
            DenyReason::LinkLost => 18,
        }
    }
}

/// The answer to a command (COMMAND_ACK).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandResult {
    /// ACCEPTED: taken at this step.
    Accepted,
    /// DENIED, with the first failed condition.
    Denied(DenyReason),
}

/// Where AUTO starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartAt {
    /// Resume the kept place.
    KeptPlace,
    /// Sequence 0.
    First,
}

/// What the caller must act on at this step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Events {
    /// Forget the mission place.
    pub place_dropped: bool,
    /// AUTO left with the place kept (Start resumes it).
    pub place_kept: bool,
    /// AUTO entered by Start.
    pub start: Option<StartAt>,
}

impl Events {
    /// Nothing.
    pub const NONE: Events = Events {
        place_dropped: false,
        place_kept: false,
        start: None,
    };
}

/// Home (A6): set when both its height and its position were latched this arming.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Home {
    /// North and east from the origin, m.
    pub ne_m: [f32; 2],
    /// Down from the origin, m.
    pub d_m: f32,
}

/// The state and what the flight code must do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Output {
    /// The effective mode.
    pub state: State,
    /// For the gate: RC loss must not disarm.
    pub hands_off: bool,
    /// The flight code uses centred synthetic sticks with switch6 treated as High.
    pub synthetic_sticks: bool,
    /// The answer to this step's command, if one was given.
    pub command: Option<CommandResult>,
    /// Home, when set.
    pub home: Option<Home>,
    /// The return height above home latched at RETURN entry, m; `None` outside RETURN.
    pub return_height_m: Option<f32>,
    /// Events.
    pub events: Events,
    /// LAND: selects the LAND law (A8) only; it triggers no disarm (WR1: the ground exit is the state machine's).
    pub landing: bool,
    /// LAND flying [`LandLaw::Hold`].
    pub land_holds_position: bool,
    /// LAND and TAKEOVER's descent: the horizontal law (H14 (1)); `None` otherwise.
    pub land_law: Option<LandLaw>,
    /// The highest level the flight code may demand this step, from this step's flags (B5, I12).
    pub demand: Demand,
    /// TAKEOVER after the battery was critical in it (C1), or with the track fault (H14): fly the LAND law (A8) with
    /// the synthetic sticks and `land_law`; hand-over stays possible.
    pub takeover_descent: bool,
    /// For the gate: a report with arm OFF must not disarm. Hands-off state, K clear and airborne (C1), after this
    /// step's report.
    pub arm_off_holds: bool,
    /// For the gate: disarm in this step. STOP entered this step with `armed` true (WR2).
    pub disarm: bool,
    /// The horizontal speed limit for the position controller, m/s: v_ho ([`speed_limit_xy_ms`]) in a hands-off state,
    /// `PILOT_SPEED_XY` in force otherwise (H13 (2)).
    pub speed_limit_xy_ms: f32,
}

/// The hands-off state machine.
#[derive(Clone, Debug)]
pub struct HandsOff {
    state: State,
    /// Stick loss, us: the gate's RC_FS_TIMEOUT (`Tuning::rc_fs_timeout_us`, decision Numbers "hands-off entry 1.0 s").
    stick_loss_us: u64,
    /// A2': set when the pilot flies; cleared on entry to any hands-off state.
    pilot_latch: bool,
    /// C1 K, "arm seen ON": a counting report had arm ON. Cleared on entry to any hands-off state. A counting report
    /// is a fresh one in TAKEOVER: a report fresh in a hands-off state arrived after its entry, because every entry
    /// other than TAKEOVER's leaves the newest report stale.
    arm_seen: bool,
    /// C1 P, "lock open": a counting report had arm ON and switch6 High. Cleared with K.
    lock_open: bool,
    /// C1: the battery was critical in this TAKEOVER; cleared on any state change.
    descent: bool,
    /// A mission place exists (AUTO, or HOLD after a pause that kept it).
    place: bool,
    /// The plan id at Start.
    flown_plan: Option<u32>,
    return_height_m: Option<f32>,
    was_armed: bool,
    home_d: Option<f32>,
    home_ne: Option<[f32; 2]>,
    /// This arming may still latch home (armed, before take-off; B8).
    home_window: bool,
    /// FS_HOLD_EN in force, taken on disarmed steps (B9).
    fs_in_force: bool,
    /// The tier for exits and the stick-loss entry (B5).
    debounced: Tier,
    /// The first step of the present run of steps below T3'.
    below_t3_since: Option<u64>,
    /// The first step of the present run of T1 steps.
    below_t2_since: Option<u64>,
    /// H12 track_fault, a landing cause of its own (H14 (1)); set by `track_error_m` or by the distance from the anchor
    /// (H13), cleared on a disarmed step only.
    track_fault: bool,
    /// The previous step's state is one the check covers (H12 item 4 with H13 (3)): HOLD, AUTO, RETURN, TAKEOVER, or
    /// LAND holding position.
    track_covered: bool,
    /// H13 (1): the anchor A (NE, m) and the braking allowance b (m) of the present fixed-target segment; `None` outside
    /// one and until a step of it has a valid, finite NE position and velocity.
    anchor: Option<([f32; 2], f32)>,
    /// A fixed-target segment has begun and its anchor is not set yet.
    anchor_pending: bool,
}

impl HandsOff {
    /// STOP, disarmed, no home. The stick-loss time is the gate's (`tuning.rc_fs_timeout_us`).
    pub fn new(tuning: &Tuning) -> Self {
        Self {
            state: State::Stop,
            stick_loss_us: tuning.rc_fs_timeout_us,
            pilot_latch: true,
            arm_seen: false,
            lock_open: false,
            descent: false,
            place: false,
            flown_plan: None,
            return_height_m: None,
            was_armed: false,
            home_d: None,
            home_ne: None,
            home_window: false,
            fs_in_force: false,
            debounced: Tier::T3,
            below_t3_since: None,
            below_t2_since: None,
            track_fault: false,
            track_covered: false,
            anchor: None,
            anchor_pending: false,
        }
    }

    /// The present state.
    pub fn state(&self) -> State {
        self.state
    }

    /// Home, when set.
    pub fn home(&self) -> Option<Home> {
        match (self.home_ne, self.home_d) {
            (Some(ne_m), Some(d_m)) => Some(Home { ne_m, d_m }),
            _ => None,
        }
    }

    /// The H12 track fault is latched (until a disarmed step).
    pub fn track_fault(&self) -> bool {
        self.track_fault
    }

    /// H13 (1): the anchor A (NE, m) and the braking allowance b (m) in force, when set.
    pub fn anchor(&self) -> Option<([f32; 2], f32)> {
        self.anchor
    }

    /// FS_HOLD_EN in force (B9): the value of the latest disarmed step.
    pub fn fs_hold_in_force(&self) -> bool {
        self.fs_in_force
    }

    /// One flight step.
    pub fn step(&mut self, i: &Inputs) -> Output {
        let fresh = i
            .crsf
            .is_some_and(|r| i.now_us.saturating_sub(r.t_us) <= self.stick_loss_us);
        let mut ev = Events::NONE;
        let was = self.state;
        self.update_home(i);
        if !i.armed {
            self.fs_in_force = i.fs_hold_en;
        }
        // H12: the previous step's distance against X, in the state that step flew; latched until disarmed.
        let x = track_fault_m(i.speed_xy_ms);
        if !i.armed {
            self.track_fault = false;
        } else if self.track_covered && i.airborne && i.track_error_m.is_some_and(|d| d.is_nan() || d > x) {
            self.track_fault = true;
        }
        // H13 (1): this step's distance from the anchor against X + b on every step with a valid NE velocity (and
        // position), whatever the setpoint; not a number trips.
        if let Some(([an, ae], b)) = self.anchor
            && i.armed
            && i.airborne
            && i.position_ne_valid
            && i.velocity_ne_valid
        {
            let [n, e] = i.position_ne_m;
            let d = libm::sqrtf((n - an) * (n - an) + (e - ae) * (e - ae));
            if d.is_nan() || d > x + b {
                self.track_fault = true;
            }
        }
        let instant = self.instant_tier(i);
        self.update_tier(i.now_us, instant);
        if self.fs_in_force {
            self.automatic(i, fresh, &mut ev);
        } else {
            let state = if i.armed && fresh && i.crsf.is_some_and(|r| r.arm) {
                State::Pilot
            } else {
                State::Stop
            };
            self.go(state, i, &mut ev);
        }
        let command = i.command.map(|c| self.command(c, i, fresh, &mut ev));
        let hands_off = self.state.is_hands_off();
        let demand = match instant {
            Tier::T3 => Demand::Position,
            Tier::T2 => Demand::Altitude,
            Tier::T1 => Demand::Angle,
        };
        let land_law = (self.state == State::Land || (self.state == State::Takeover && self.descent)).then_some(
            match demand {
                Demand::Position if self.track_fault => LandLaw::Brake,
                Demand::Position => LandLaw::Hold,
                Demand::Altitude | Demand::Angle => LandLaw::Level,
            },
        );
        let land_holds_position = self.state == State::Land && land_law == Some(LandLaw::Hold);
        self.update_anchor(was, demand, i);
        self.track_covered =
            matches!(self.state, State::Hold | State::Auto | State::Return | State::Takeover) || land_holds_position;
        Output {
            state: self.state,
            hands_off,
            synthetic_sticks: hands_off,
            command,
            home: self.home(),
            return_height_m: self.return_height_m,
            events: ev,
            landing: self.state == State::Land,
            land_holds_position,
            land_law,
            demand,
            takeover_descent: self.state == State::Takeover && self.descent,
            arm_off_holds: self.arm_off_holds(i),
            disarm: self.state == State::Stop && was != State::Stop && i.armed,
            speed_limit_xy_ms: if hands_off {
                speed_limit_xy_ms(i.speed_xy_ms)
            } else {
                i.speed_xy_ms
            },
        }
    }

    /// H13 (1): a fixed-target segment begins on entry to HOLD, TAKEOVER or LAND on T3' from PILOT, AUTO or RETURN; its
    /// anchor is set at its first step with a valid, finite NE position and velocity: A that position, b = |v_NE|^2 /
    /// (2 `GUIDANCE_ACC_MS2`). Kept through any other change inside the segment; cleared in PILOT, STOP, AUTO and RETURN
    /// (so at disarm, which is STOP).
    fn update_anchor(&mut self, was: State, demand: Demand, i: &Inputs) {
        let entry = was != self.state
            && matches!(was, State::Pilot | State::Auto | State::Return)
            && (matches!(self.state, State::Hold | State::Takeover)
                || (self.state == State::Land && demand == Demand::Position));
        if entry {
            self.anchor = None;
            self.anchor_pending = true;
        }
        if matches!(self.state, State::Pilot | State::Stop | State::Auto | State::Return) {
            self.anchor = None;
            self.anchor_pending = false;
        }
        let [vn, ve] = i.velocity_ne_ms;
        let finite = i.position_ne_m.iter().chain(i.velocity_ne_ms.iter()).all(|v| v.is_finite());
        if self.anchor_pending && i.position_ne_valid && i.velocity_ne_valid && finite {
            self.anchor = Some((i.position_ne_m, (vn * vn + ve * ve) / (2.0 * Tuning::GUIDANCE_ACC_MS2)));
            self.anchor_pending = false;
        }
    }

    /// H14 (1): heading_ok = the input only (the track fault is a landing cause of its own).
    fn heading_ok(&self, i: &Inputs) -> bool {
        i.heading_ok
    }

    /// This step's tier with B4's T3' (T3 needs `heading_ok`).
    fn instant_tier(&self, i: &Inputs) -> Tier {
        match i.tier {
            Tier::T3 if self.heading_ok(i) => Tier::T3,
            Tier::T3 | Tier::T2 => Tier::T2,
            Tier::T1 => Tier::T1,
        }
    }

    /// B5: a drop below a level counts once it has lasted `TIER_DEBOUNCE_US` without a break; a rise at once.
    fn update_tier(&mut self, now: u64, instant: Tier) {
        if instant == Tier::T3 {
            self.below_t3_since = None;
        } else if self.below_t3_since.is_none() {
            self.below_t3_since = Some(now);
        }
        if instant != Tier::T1 {
            self.below_t2_since = None;
        } else if self.below_t2_since.is_none() {
            self.below_t2_since = Some(now);
        }
        let held = |since: Option<u64>| since.is_some_and(|t| now.saturating_sub(t) >= TIER_DEBOUNCE_US);
        self.debounced = if !held(self.below_t3_since) {
            Tier::T3
        } else if !held(self.below_t2_since) {
            Tier::T2
        } else {
            Tier::T1
        };
    }

    /// LAND, or BLIND when the debounced tier is T1 (B6: BLIND is LAND flown on T1).
    fn land_state(&self) -> State {
        if self.debounced == Tier::T1 {
            State::Blind
        } else {
            State::Land
        }
    }

    /// A6 with B8: D and NE each at the first armed step where they are valid while not airborne; the window closes on
    /// airborne; home is both or nothing; cleared at disarm.
    fn update_home(&mut self, i: &Inputs) {
        if !i.armed {
            self.home_d = None;
            self.home_ne = None;
            self.home_window = false;
        } else {
            if !self.was_armed {
                self.home_window = true;
            }
            if self.home_window && i.return_built {
                if i.airborne {
                    self.home_window = false;
                } else {
                    if self.home_d.is_none() && i.position_d_valid && i.position_d_m.is_finite() {
                        self.home_d = Some(i.position_d_m);
                    }
                    if self.home_ne.is_none() && i.position_ne_valid && i.position_ne_m.iter().all(|v| v.is_finite()) {
                        self.home_ne = Some(i.position_ne_m);
                    }
                }
            }
        }
        self.was_armed = i.armed;
    }

    /// The state table's exits that are not commands (FS_HOLD_EN 1).
    fn automatic(&mut self, i: &Inputs, fresh: bool, ev: &mut Events) {
        if !i.armed || (i.crsf.is_some_and(|r| !r.arm) && !self.arm_off_holds(i)) {
            self.go(State::Stop, i, ev);
            return;
        }
        if self.state.is_hands_off() && !i.airborne {
            self.go(State::Stop, i, ev);
            return;
        }
        match self.state {
            State::Stop => {
                if fresh && self.pilot_latch {
                    self.go(State::Pilot, i, ev);
                }
            }
            State::Pilot => {
                if !fresh {
                    self.go(self.lost(i), i, ev);
                }
            }
            State::Takeover => {
                if !fresh {
                    self.go(self.lost(i), i, ev);
                } else {
                    self.lock(i);
                    if self.pilot_latch {
                        self.go(State::Pilot, i, ev);
                    }
                }
            }
            State::Hold | State::Auto | State::Return | State::Land | State::Blind if fresh => {
                self.go(State::Takeover, i, ev);
                self.lock(i);
            }
            State::Hold => {
                if self.debounced != Tier::T3 || i.battery_critical || self.track_fault {
                    self.go(self.land_state(), i, ev);
                } else if self.link_age(i) > LINK_LOSS_US {
                    let next = if self.home().is_some() {
                        State::Return
                    } else {
                        State::Land
                    };
                    self.go(next, i, ev);
                } else if self.place && i.mission.plan_id != self.flown_plan {
                    self.drop_place(ev);
                }
            }
            State::Auto => {
                if self.debounced != Tier::T3 || i.battery_critical || self.track_fault {
                    self.go(self.land_state(), i, ev);
                } else if i.mission.plan_id != self.flown_plan || i.last_waypoint_reached {
                    self.go(State::Hold, i, ev);
                } else if !Self::station_present(i) {
                    self.pause(i, ev);
                }
            }
            State::Return => {
                if self.debounced != Tier::T3 || i.battery_critical || self.track_fault || i.home_reached {
                    self.go(self.land_state(), i, ev);
                }
            }
            State::Land => {
                if self.debounced == Tier::T1 {
                    self.go(State::Blind, i, ev);
                }
            }
            State::Blind => {
                if self.debounced != Tier::T1 {
                    self.go(State::Land, i, ev);
                }
            }
        }
        if self.state == State::Takeover && (i.battery_critical || self.track_fault) {
            self.descent = true;
        }
    }

    /// C1: arm OFF withholds the disarm in a hands-off state while K is clear and the airborne latch is set.
    fn arm_off_holds(&self, i: &Inputs) -> bool {
        self.state.is_hands_off() && !self.arm_seen && i.airborne
    }

    /// C1 with A2': this step's counting report sets K and P; after P, arm ON and switch6 not High sets the pilot latch.
    fn lock(&mut self, i: &Inputs) {
        let Some(r) = i.crsf.filter(|r| r.arm) else {
            return;
        };
        let high = r.switch6 == Switch3::High;
        if self.lock_open && !high {
            self.pilot_latch = true;
        }
        self.arm_seen = true;
        self.lock_open |= high;
    }

    /// Sticks lost in PILOT or TAKEOVER: STOP on the ground; LAND (or BLIND) with the track fault (H14); else by the
    /// debounced tier.
    fn lost(&self, i: &Inputs) -> State {
        if !i.airborne {
            State::Stop
        } else if self.track_fault {
            self.land_state()
        } else {
            match self.debounced {
                Tier::T3 => State::Hold,
                Tier::T2 => State::Land,
                Tier::T1 => State::Blind,
            }
        }
    }

    /// A1: now minus the newer of the last CRSF report and the last radio HEARTBEAT.
    fn link_age(&self, i: &Inputs) -> u64 {
        let newest = match (i.crsf.map(|r| r.t_us), i.last_radio_heartbeat_us) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        };
        newest.map_or(u64::MAX, |t| i.now_us.saturating_sub(t))
    }

    /// A station HEARTBEAT on the radio link within `STATION_LOST_US`.
    fn station_present(i: &Inputs) -> bool {
        i.last_radio_heartbeat_us
            .is_some_and(|t| i.now_us.saturating_sub(t) <= STATION_LOST_US)
    }

    fn drop_place(&mut self, ev: &mut Events) {
        self.place = false;
        ev.place_dropped = true;
    }

    /// AUTO to HOLD with the place kept.
    fn pause(&mut self, i: &Inputs, ev: &mut Events) {
        self.go(State::Hold, i, ev);
        self.place = true;
        ev.place_dropped = false;
        ev.place_kept = true;
    }

    /// Enters `next` (no effect when it is the present state). Any change other than to AUTO drops a place; the
    /// callers that keep it (`pause`) restore it.
    fn go(&mut self, next: State, i: &Inputs, ev: &mut Events) {
        if next == self.state {
            return;
        }
        if self.place && next != State::Auto {
            self.drop_place(ev);
        }
        if next.is_hands_off() {
            self.pilot_latch = false;
            self.arm_seen = false;
            self.lock_open = false;
        }
        self.descent = false;
        self.return_height_m = None;
        if next == State::Return {
            let present = self.home_d.map_or(0.0, |d| d - i.position_d_m);
            self.return_height_m = Some(
                i.mission
                    .return_height_m
                    .map_or(present, |h| h.max(present)),
            );
        }
        if next == State::Stop {
            // Disarmed: only a CRSF report arms (A2), so the next arming is the pilot's.
            self.flown_plan = None;
            self.pilot_latch = true;
        }
        self.state = next;
    }

    /// A10, A10', A9, C2, C3 and the lead's rulings: the answer, and the change at this step when ACCEPTED.
    fn command(&mut self, c: StationCommand, i: &Inputs, fresh: bool, ev: &mut Events) -> CommandResult {
        use CommandResult::{Accepted, Denied};
        if c.link != Link::Radio {
            return Denied(DenyReason::NotRadioLink);
        }
        if fresh {
            return Denied(DenyReason::CrsfFresh);
        }
        if !self.fs_in_force {
            return Denied(DenyReason::HandsOffDisabled);
        }
        let into_hold = matches!(
            (c.command, self.state),
            (Command::Pause | Command::Abort, State::Auto | State::Return) | (Command::Abort, State::Land)
        );
        if into_hold && let Some(reason) = self.hold_entry_check(i) {
            return Denied(reason);
        }
        match (c.command, self.state) {
            (Command::Start, State::Auto)
            | (Command::Pause, State::Hold)
            | (Command::Land, State::Land | State::Blind) => Accepted,
            (Command::Start, State::Hold) => match self.start_check(i) {
                Some(reason) => Denied(reason),
                None => {
                    ev.start = Some(if self.place {
                        StartAt::KeptPlace
                    } else {
                        StartAt::First
                    });
                    self.go(State::Auto, i, ev);
                    self.place = true;
                    self.flown_plan = i.mission.plan_id;
                    Accepted
                }
            },
            (Command::Pause, State::Auto) => {
                self.pause(i, ev);
                Accepted
            }
            (Command::Pause, State::Return) | (Command::Abort, State::Auto | State::Return | State::Land) => {
                self.go(State::Hold, i, ev);
                Accepted
            }
            (Command::Abort, State::Hold) => {
                self.drop_place(ev);
                Accepted
            }
            (Command::Abort, State::Blind) => Denied(DenyReason::NotT3),
            (Command::Return, State::Return) => Accepted,
            (Command::Return, State::Hold | State::Auto) => match self.return_check(i) {
                Some(reason) => Denied(reason),
                None => {
                    self.go(State::Return, i, ev);
                    Accepted
                }
            },
            (Command::Land, State::Hold | State::Auto | State::Return) => {
                self.go(self.land_state(), i, ev);
                Accepted
            }
            _ => Denied(DenyReason::NotInThisState),
        }
    }

    /// B2: a command into HOLD is DENIED when one of HOLD's automatic exits would fire on this step's facts.
    fn hold_entry_check(&self, i: &Inputs) -> Option<DenyReason> {
        if self.debounced != Tier::T3 {
            Some(DenyReason::NotT3)
        } else if i.battery_critical {
            Some(DenyReason::BatteryCritical)
        } else if self.link_age(i) > LINK_LOSS_US {
            Some(DenyReason::LinkLost)
        } else {
            None
        }
    }

    /// C3: Return needs home; then B2's no-bounce rule for RETURN's automatic exits (arrival excepted).
    fn return_check(&self, i: &Inputs) -> Option<DenyReason> {
        if self.home().is_none() {
            Some(DenyReason::NoHome)
        } else if self.debounced != Tier::T3 {
            Some(DenyReason::NotT3)
        } else if i.battery_critical {
            Some(DenyReason::BatteryCritical)
        } else {
            None
        }
    }

    /// Start's conditions in the contract's order (section 10), the lead's height ruling after the list's;
    /// the first that fails.
    fn start_check(&self, i: &Inputs) -> Option<DenyReason> {
        if !Self::station_present(i) {
            return Some(DenyReason::StationAbsent);
        }
        if !i.mission.has_waypoints {
            return Some(DenyReason::NoWaypoints);
        }
        if !i.mission.z_within_limits {
            return Some(DenyReason::HeightOutsideLimits);
        }
        let Some(home) = self.home() else {
            return Some(DenyReason::NoHome);
        };
        let [n, e] = i.position_ne_m;
        let [hn, he] = home.ne_m;
        let d2 = (n - hn) * (n - hn) + (e - he) * (e - he);
        let vehicle_within = d2 <= HOME_RANGE_M * HOME_RANGE_M;
        if !i.position_ne_valid || !vehicle_within {
            return Some(DenyReason::VehicleBeyondRange);
        }
        let Some(far) = i.mission.max_waypoint_distance_m.filter(|d| d.is_finite()) else {
            return Some(DenyReason::WaypointDistanceUnknown);
        };
        let waypoints_within = far <= HOME_RANGE_M;
        if !waypoints_within {
            return Some(DenyReason::WaypointBeyondRange);
        }
        if !self.heading_ok(i) || self.track_fault {
            return Some(DenyReason::HeadingNotOk);
        }
        if i.tier != Tier::T3 {
            return Some(DenyReason::NotT3);
        }
        if !i.battery_ok || i.battery_critical {
            return Some(DenyReason::BatteryNotOk);
        }
        if !i.crsf.is_some_and(|r| r.arm) {
            return Some(DenyReason::ArmSwitchOff);
        }
        if !i.crsf.is_some_and(|r| r.switch6 == Switch3::High) {
            return Some(DenyReason::SwitchNotHigh);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::rc::Sticks;

    const T0: u64 = 1_000_000_000;
    const S: u64 = 1_000_000;
    /// `Tuning::DEFAULT.rc_fs_timeout_us`.
    const LOSS: u64 = 1_000_000;
    const PLAN: u32 = 7;
    /// Made-up local positions (m from the origin), not coordinates.
    const HOME_NE: [f32; 2] = [3.0, -4.0];
    const HOME_D: f32 = -0.5;

    const CENTRED: Sticks = Sticks {
        roll: 0.0,
        pitch: 0.0,
        throttle: 0.5,
        yaw: 0.0,
    };
    const FULL: Sticks = Sticks {
        roll: 1.0,
        pitch: 1.0,
        throttle: 1.0,
        yaw: 1.0,
    };

    fn mission() -> MissionFacts {
        MissionFacts {
            plan_id: Some(PLAN),
            has_waypoints: true,
            z_within_limits: true,
            max_waypoint_distance_m: Some(100.0),
            return_height_m: Some(20.0),
        }
    }

    struct Rig {
        h: HandsOff,
        i: Inputs,
    }

    impl Rig {
        /// Disarmed on the ground at home, a fresh report (High, arm on), the station present.
        fn new() -> Rig {
            let i = Inputs {
                now_us: T0,
                fs_hold_en: true,
                crsf: Some(RcSample {
                    sticks: CENTRED,
                    arm: true,
                    switch6: Switch3::High,
                    t_us: T0,
                }),
                last_radio_heartbeat_us: Some(T0),
                last_radio_status_us: None,
                armed: false,
                airborne: false,
                tier: Tier::T3,
                position_ne_m: HOME_NE,
                position_ne_valid: true,
                velocity_ne_ms: [0.0, 0.0],
                velocity_ne_valid: true,
                position_d_m: HOME_D,
                position_d_valid: true,
                command: None,
                mission: mission(),
                last_waypoint_reached: false,
                home_reached: false,
                heading_ok: true,
                battery_ok: true,
                battery_critical: false,
                return_built: true,
                track_error_m: None,
                speed_xy_ms: Tuning::DEFAULT.pilot_speed_xy_ms,
            };
            Rig {
                h: HandsOff::new(&Tuning::DEFAULT),
                i,
            }
        }

        fn step(&mut self) -> Output {
            self.h.step(&self.i)
        }

        fn at(&mut self, t: u64) -> Output {
            self.i.now_us = t;
            self.step()
        }

        fn now(&self) -> u64 {
            self.i.now_us
        }

        fn last_report(&self) -> u64 {
            self.i.crsf.map_or(0, |r| r.t_us)
        }

        /// A new report at `t` with these sticks and switches.
        fn report_with(&mut self, t: u64, sticks: Sticks, switch6: Switch3, arm: bool) -> Output {
            self.i.crsf = Some(RcSample {
                sticks,
                arm,
                switch6,
                t_us: t,
            });
            self.at(t)
        }

        fn report(&mut self, t: u64, switch6: Switch3) -> Output {
            self.report_with(t, FULL, switch6, true)
        }

        /// The station's heartbeat at `t`, then a step at `t`.
        fn heartbeat(&mut self, t: u64) -> Output {
            self.i.last_radio_heartbeat_us = Some(t);
            self.at(t)
        }

        fn cmd_on(&mut self, command: Command, link: Link) -> Output {
            self.i.command = Some(StationCommand { command, link });
            let out = self.step();
            self.i.command = None;
            out
        }

        fn cmd(&mut self, command: Command) -> Output {
            self.cmd_on(command, Link::Radio)
        }

        /// Armed (home latched), airborne, PILOT, 10 m above home.
        fn pilot() -> Rig {
            let mut r = Rig::new();
            assert_eq!(r.step().state, State::Stop);
            r.i.armed = true;
            assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
            r.i.airborne = true;
            r.i.position_ne_m = [10.0, 10.0];
            r.i.position_d_m = HOME_D - 10.0;
            let out = r.report(T0 + 2_000, Switch3::High);
            assert_eq!(out.state, State::Pilot);
            assert!(!out.hands_off && !out.synthetic_sticks);
            r
        }

        /// Sticks lost on T3 in the air: HOLD, with the station's heartbeat at that step.
        fn hold() -> Rig {
            let mut r = Rig::pilot();
            let t = r.last_report() + LOSS + 1;
            let out = r.heartbeat(t);
            assert_eq!(out.state, State::Hold);
            r
        }

        fn auto() -> Rig {
            let mut r = Rig::hold();
            let out = r.cmd(Command::Start);
            assert_eq!(out.command, Some(CommandResult::Accepted));
            assert_eq!(out.state, State::Auto);
            assert_eq!(out.events.start, Some(StartAt::First));
            r
        }

        /// Link age above 10 s in HOLD with home and T3.
        fn ret() -> Rig {
            let mut r = Rig::hold();
            let t = r.now() + LINK_LOSS_US + 1;
            let out = r.at(t);
            assert_eq!(out.state, State::Return);
            r
        }

        fn land() -> Rig {
            let mut r = Rig::hold();
            let out = r.cmd(Command::Land);
            assert_eq!(out.command, Some(CommandResult::Accepted));
            assert_eq!(out.state, State::Land);
            r
        }

        /// This step's tier becomes `tier` (the first step of the drop).
        fn drop_tier(&mut self, tier: Tier) -> Output {
            self.i.tier = tier;
            self.step()
        }

        /// The drop has lasted `TIER_DEBOUNCE_US`.
        fn hold_drop(&mut self) -> Output {
            let t = self.now() + TIER_DEBOUNCE_US;
            self.at(t)
        }

        /// Sticks lost in the air after T1 has lasted 1.0 s.
        fn blind() -> Rig {
            let mut r = Rig::pilot();
            assert_eq!(r.drop_tier(Tier::T1).state, State::Pilot);
            let t = r.last_report() + LOSS + 1;
            assert_eq!(r.at(t).state, State::Blind);
            r
        }

        /// HOLD, then a fresh report with switch6 Low.
        fn takeover() -> Rig {
            let mut r = Rig::hold();
            let t = r.now() + 1_000;
            let out = r.report(t, Switch3::Low);
            assert_eq!(out.state, State::Takeover);
            r
        }
    }

    fn denied(reason: DenyReason) -> Option<CommandResult> {
        Some(CommandResult::Denied(reason))
    }

    // ---- PILOT ----

    #[test]
    fn hands_off_pilot_entry_is_arming_by_crsf_from_stop() {
        let mut r = Rig::new();
        let out = r.step();
        assert_eq!((out.state, out.hands_off), (State::Stop, false));
        r.i.armed = true;
        let out = r.report(T0 + 1_000, Switch3::Low);
        assert_eq!(out.state, State::Pilot);
        assert!(!out.hands_off && !out.synthetic_sticks);
    }

    #[test]
    fn hands_off_pilot_sticks_lost_goes_by_tier_after_the_stick_loss_time() {
        for (tier, want) in [
            (Tier::T3, State::Hold),
            (Tier::T2, State::Land),
            (Tier::T1, State::Blind),
        ] {
            let mut r = Rig::pilot();
            r.drop_tier(tier);
            let last = r.last_report();
            assert_eq!(r.at(last + LOSS).state, State::Pilot, "{tier:?} at exactly 1.0 s");
            let out = r.at(last + LOSS + 1);
            assert_eq!(out.state, want, "{tier:?}");
            assert!(out.hands_off && out.synthetic_sticks);
            assert_eq!(out.landing, want == State::Land);
        }
    }

    #[test]
    fn hands_off_pilot_sticks_lost_on_the_ground_is_stop() {
        let mut r = Rig::pilot();
        r.i.airborne = false;
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.hands_off), (State::Stop, false));
    }

    // ---- TAKEOVER ----

    #[test]
    fn hands_off_takeover_entry_from_every_hands_off_state() {
        let rigs: [(&str, fn() -> Rig); 5] = [
            ("hold", Rig::hold),
            ("auto", Rig::auto),
            ("return", Rig::ret),
            ("land", Rig::land),
            ("blind", Rig::blind),
        ];
        for (name, make) in rigs {
            let mut r = make();
            let t = r.now() + 1_000;
            let out = r.report(t, Switch3::Middle);
            assert_eq!(out.state, State::Takeover, "{name}");
            assert!(out.hands_off && out.synthetic_sticks, "{name}");
            assert_eq!(out.return_height_m, None, "{name}");
            assert_eq!(out.events.place_dropped, name == "auto", "{name}: AUTO drops the place");
        }
    }

    #[test]
    fn hands_off_takeover_rc_lost_goes_back_by_tier() {
        for (tier, want) in [
            (Tier::T3, State::Hold),
            (Tier::T2, State::Land),
            (Tier::T1, State::Blind),
        ] {
            let mut r = Rig::takeover();
            r.drop_tier(tier);
            let last = r.last_report();
            assert_eq!(r.at(last + LOSS).state, State::Takeover);
            assert_eq!(r.at(last + LOSS + 1).state, want, "{tier:?}");
        }
    }

    #[test]
    fn hands_off_takeover_arm_off_is_stop() {
        let mut r = Rig::takeover();
        let t = r.now() + 1_000;
        let out = r.report_with(t, CENTRED, Switch3::Low, false);
        assert_eq!((out.state, out.hands_off), (State::Stop, false));
    }

    #[test]
    fn hands_off_takeover_rc_lost_on_the_ground_is_stop() {
        let mut r = Rig::takeover();
        r.i.airborne = false;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.at(t).state, State::Stop);
    }

    // ---- I3 (as amended by A2') ----

    #[test]
    fn hands_off_i3_full_sticks_low_or_middle_before_high_keep_synthetic_sticks() {
        let mut r = Rig::takeover();
        for k in 1..=50u64 {
            let sw = if k % 2 == 0 { Switch3::Low } else { Switch3::Middle };
            let t = r.now() + 20_000;
            let out = r.report(t, sw);
            assert_eq!(out.state, State::Takeover, "report {k}");
            assert!(out.synthetic_sticks && out.hands_off, "report {k}");
        }
    }

    #[test]
    fn hands_off_i3_full_sticks_with_high_keep_synthetic_sticks_then_first_non_high_hands_over() {
        let mut r = Rig::takeover();
        let t = r.now() + 20_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!(out.state, State::Takeover);
        for _ in 0..10 {
            let t = r.now() + 20_000;
            let out = r.report(t, Switch3::High);
            assert_eq!(out.state, State::Takeover);
            assert!(out.synthetic_sticks);
        }
        let t = r.now() + 20_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!(out.state, State::Pilot);
        assert!(!out.synthetic_sticks && !out.hands_off);
    }

    #[test]
    fn hands_off_i3_switch_already_high_in_the_first_report_waits_until_it_leaves_high() {
        let mut r = Rig::hold();
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::High);
        assert_eq!(out.state, State::Takeover);
        assert!(out.synthetic_sticks);
        let t = r.now() + 20_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.now() + 20_000;
        let out = r.report(t, Switch3::Low);
        assert_eq!(out.state, State::Pilot);
        assert!(!out.synthetic_sticks);
    }

    #[test]
    fn hands_off_i3_high_seen_before_rc_loss_does_not_count_after_it() {
        let mut r = Rig::takeover();
        let t = r.now() + 20_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.at(t).state, State::Hold);
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover);
    }

    // ---- HOLD ----

    #[test]
    fn hands_off_hold_entries_from_auto() {
        // Pause: kept.
        let mut r = Rig::auto();
        let out = r.cmd(Command::Pause);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        assert!(out.events.place_kept && !out.events.place_dropped);
        // Abort: dropped.
        let mut r = Rig::auto();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        assert!(out.events.place_dropped && !out.events.place_kept);
        // Last waypoint: dropped.
        let mut r = Rig::auto();
        r.i.last_waypoint_reached = true;
        let out = r.step();
        assert_eq!(out.state, State::Hold);
        assert!(out.events.place_dropped && !out.events.place_kept);
    }

    #[test]
    fn hands_off_hold_entries_from_return_and_land() {
        for c in [Command::Pause, Command::Abort] {
            let mut r = Rig::ret();
            r.heartbeat(r.now() + 1_000);
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)), "{c:?}");
            assert_eq!(out.return_height_m, None);
        }
        let mut r = Rig::land();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
    }

    #[test]
    fn hands_off_hold_exits() {
        // CRSF: TAKEOVER (covered above). Start: AUTO (Rig::auto). Land command:
        assert_eq!(Rig::land().h.state(), State::Land);
        // T3 lost for 1.0 s: LAND, or BLIND on T1 (B6); latched (T3 back gives no HOLD).
        for (tier, want) in [(Tier::T2, State::Land), (Tier::T1, State::Blind)] {
            let mut r = Rig::hold();
            assert_eq!(r.drop_tier(tier).state, State::Hold);
            assert_eq!(r.hold_drop().state, want, "{tier:?}");
            r.i.tier = Tier::T3;
            assert_eq!(r.step().state, State::Land, "latched");
        }
        // Battery critical:
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        assert_eq!(r.step().state, State::Land);
    }

    // ---- I4 ----

    #[test]
    fn hands_off_i4_return_at_10_1_s_not_at_9_9_s_with_radio_status_present() {
        let mut r = Rig::hold();
        let hb = r.now();
        for t in (hb..hb + 9_900_000).step_by(100_000) {
            r.i.last_radio_status_us = Some(t);
            assert_eq!(r.at(t).state, State::Hold, "at {}", t - hb);
        }
        r.i.last_radio_status_us = Some(hb + 9_900_000);
        assert_eq!(r.at(hb + 9_900_000).state, State::Hold);
        r.i.last_radio_status_us = Some(hb + 10_100_000);
        let out = r.at(hb + 10_100_000);
        assert_eq!(out.state, State::Return);
        assert!(out.hands_off);
    }

    #[test]
    fn hands_off_i4_link_age_runs_from_the_newer_of_crsf_and_the_radio_heartbeat() {
        // No heartbeat at all: the last CRSF report starts the count.
        let mut r = Rig::pilot();
        r.i.last_radio_heartbeat_us = None;
        let tc = r.last_report();
        assert_eq!(r.at(tc + LOSS + 1).state, State::Hold);
        assert_eq!(r.at(tc + 9_900_000).state, State::Hold);
        assert_eq!(r.at(tc + 10_100_000).state, State::Return);
        // A heartbeat each second holds for a minute: the station keeps the link.
        let mut r = Rig::hold();
        let t0 = r.now();
        for k in 1..=60 {
            assert_eq!(r.heartbeat(t0 + k * S).state, State::Hold);
        }
    }

    #[test]
    fn hands_off_i4_a_usb_heartbeat_cannot_prevent_it() {
        // `Inputs` has no USB heartbeat (A1): the struct literal in `Rig::new` names every field, so one added there
        // would have to be written into this test's rig. Commands over USB do not refresh the link either.
        let mut r = Rig::hold();
        let hb = r.now();
        for t in (hb + S..hb + 10 * S).step_by(S as usize) {
            r.i.now_us = t;
            assert_eq!(
                r.cmd_on(Command::Pause, Link::Usb).command,
                denied(DenyReason::NotRadioLink)
            );
        }
        assert_eq!(r.at(hb + 10 * S + 1).state, State::Return);
    }

    #[test]
    fn hands_off_i4_no_home_lands() {
        let mut r = Rig::hold();
        // A flight with take-off before any NE: no home.
        let mut r2 = Rig::new();
        r2.step();
        r2.i.armed = true;
        r2.i.position_ne_valid = false;
        r2.report(T0 + 1_000, Switch3::High);
        r2.i.airborne = true;
        r2.report(T0 + 2_000, Switch3::High);
        r2.i.position_ne_valid = true;
        let t = r2.last_report() + LOSS + 1;
        let out = r2.heartbeat(t);
        assert_eq!((out.state, out.home), (State::Hold, None));
        let out = r2.at(t + LINK_LOSS_US + 1);
        assert_eq!(out.state, State::Land);
        // With home the same flight returns.
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Return);
    }

    // ---- AUTO and I5 ----

    #[test]
    fn hands_off_i5_auto_pauses_at_5_s_without_the_radio_heartbeat_place_kept() {
        let mut r = Rig::auto();
        let hb = r.now();
        r.i.last_radio_status_us = Some(hb + STATION_LOST_US);
        assert_eq!(r.at(hb + STATION_LOST_US).state, State::Auto);
        let out = r.at(hb + STATION_LOST_US + 1);
        assert_eq!(out.state, State::Hold);
        assert!(out.events.place_kept && !out.events.place_dropped);
        // The station back: Start resumes the kept place.
        r.heartbeat(hb + STATION_LOST_US + 1_000);
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.events.start), (State::Auto, Some(StartAt::KeptPlace)));
    }

    #[test]
    fn hands_off_auto_exits_to_land() {
        let mut r = Rig::auto();
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.command), (State::Land, Some(CommandResult::Accepted)));
        let mut r = Rig::auto();
        assert_eq!(r.drop_tier(Tier::T2).state, State::Auto);
        assert_eq!(r.hold_drop().state, State::Land);
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        let out = r.step();
        assert_eq!(out.state, State::Land);
        assert!(out.landing);
    }

    // ---- I6 ----

    #[test]
    fn hands_off_i6_new_plan_in_auto_holds_with_the_place_dropped() {
        let mut r = Rig::auto();
        r.i.mission.plan_id = Some(PLAN + 1);
        let out = r.step();
        assert_eq!(out.state, State::Hold);
        assert!(out.events.place_dropped);
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.events.start), (State::Auto, Some(StartAt::First)));
        // A cleared list is a new plan too.
        r.i.mission.plan_id = None;
        assert_eq!(r.step().state, State::Hold);
    }

    #[test]
    fn hands_off_i6_new_plan_in_hold_drops_a_kept_place() {
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        r.i.mission.plan_id = Some(PLAN + 1);
        let out = r.step();
        assert_eq!(out.state, State::Hold);
        assert!(out.events.place_dropped);
        assert_eq!(r.cmd(Command::Start).events.start, Some(StartAt::First));
    }

    #[test]
    fn hands_off_i6_new_plan_in_return_and_land_changes_nothing() {
        let mut r = Rig::ret();
        let before = r.step();
        r.i.mission.plan_id = Some(PLAN + 1);
        r.i.mission.return_height_m = Some(45.0);
        let out = r.step();
        assert_eq!(out.state, State::Return);
        assert_eq!(out.return_height_m, before.return_height_m);
        assert_eq!(out.events, Events::NONE);
        let mut r = Rig::land();
        r.i.mission.plan_id = Some(PLAN + 1);
        assert_eq!(r.step().state, State::Land);
    }

    // ---- RETURN ----

    #[test]
    fn hands_off_return_height_is_latched_at_entry() {
        // Mission 20 m, present 10 m: 20.
        let mut r = Rig::ret();
        assert_eq!(r.step().return_height_m, Some(20.0));
        r.i.position_d_m = HOME_D - 30.0;
        assert_eq!(r.step().return_height_m, Some(20.0));
        // Mission 5 m, present 10 m: 10.
        let mut r = Rig::hold();
        r.i.mission.return_height_m = Some(5.0);
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).return_height_m, Some(10.0));
        // No mission: present height.
        let mut r = Rig::hold();
        r.i.mission = MissionFacts {
            plan_id: None,
            has_waypoints: false,
            z_within_limits: true,
            max_waypoint_distance_m: None,
            return_height_m: None,
        };
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).return_height_m, Some(10.0));
    }

    #[test]
    fn hands_off_return_exits_to_land_and_ignores_the_link() {
        let mut r = Rig::ret();
        let t = r.now() + 600 * S;
        assert_eq!(r.at(t).state, State::Return);
        for (name, set) in [
            ("arrival", (|i: &mut Inputs| i.home_reached = true) as fn(&mut Inputs)),
            ("t3 lost", |i: &mut Inputs| i.tier = Tier::T2),
            ("battery", |i: &mut Inputs| i.battery_critical = true),
        ] {
            let mut r = Rig::ret();
            set(&mut r.i);
            let mut out = r.step();
            if name == "t3 lost" {
                assert_eq!(out.state, State::Return, "debounced");
                out = r.hold_drop();
            }
            assert_eq!(out.state, State::Land, "{name}");
            assert_eq!(out.return_height_m, None);
        }
        let mut r = Rig::ret();
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.command), (State::Land, Some(CommandResult::Accepted)));
    }

    #[test]
    fn hands_off_return_forgets_a_kept_place() {
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        let t = r.now() + LINK_LOSS_US + 1;
        let out = r.at(t);
        assert_eq!(out.state, State::Return);
        assert!(out.events.place_dropped);
    }

    // ---- LAND ----

    #[test]
    fn hands_off_land_entries() {
        // Land command, T3 lost (HOLD, AUTO, RETURN), arrival, link without home, battery: covered above.
        assert!(Rig::land().step().landing);
    }

    #[test]
    fn hands_off_land_ignores_the_link_and_abort_needs_t3() {
        let mut r = Rig::land();
        let t = r.now() + 600 * S;
        assert_eq!(r.at(t).state, State::Land);
        r.heartbeat(r.now() + 1_000);
        r.drop_tier(Tier::T2);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        r.hold_drop();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotT3)));
    }

    #[test]
    fn hands_off_land_disarm_is_stop() {
        let mut r = Rig::land();
        r.i.armed = false;
        r.i.airborne = false;
        let out = r.step();
        assert_eq!((out.state, out.hands_off, out.landing), (State::Stop, false, false));
        assert_eq!(out.home, None);
    }

    // ---- STOP and BLIND ----

    #[test]
    fn hands_off_disarm_is_stop_from_every_state() {
        let rigs: [fn() -> Rig; 7] = [
            Rig::pilot,
            Rig::takeover,
            Rig::hold,
            Rig::auto,
            Rig::ret,
            Rig::land,
            Rig::blind,
        ];
        for make in rigs {
            let mut r = make();
            let from = r.h.state();
            r.i.armed = false;
            let out = r.step();
            assert_eq!((out.state, out.hands_off), (State::Stop, false), "{from:?}");
        }
    }

    #[test]
    fn hands_off_stop_after_a_hands_off_landing_rearms_into_pilot() {
        let mut r = Rig::land();
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!(r.step().state, State::Stop);
        let t = r.now() + 1_000;
        assert_eq!(r.report_with(t, CENTRED, Switch3::Low, false).state, State::Stop);
        r.i.armed = true;
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, CENTRED, Switch3::Low, true).state, State::Pilot);
    }

    #[test]
    fn hands_off_b6_blind_and_land_switch_by_debounced_tier() {
        // T1 -> T2 in BLIND: LAND at once (an upgrade).
        let mut r = Rig::blind();
        let t = r.now() + 600 * S;
        assert_eq!(r.at(t).state, State::Blind);
        r.i.battery_critical = true;
        assert_eq!(r.step().state, State::Blind, "battery critical: no change");
        let out = r.drop_tier(Tier::T2);
        assert_eq!((out.state, out.landing), (State::Land, true));
        // T2 -> T1 in LAND: BLIND only once held 1.0 s.
        assert_eq!(r.drop_tier(Tier::T1).state, State::Land);
        let t = r.now() + TIER_DEBOUNCE_US - 1;
        assert_eq!(r.at(t).state, State::Land);
        assert_eq!(r.at(t + 1).state, State::Blind);
    }

    #[test]
    fn hands_off_b6_blind_command_table() {
        let mut r = Rig::blind();
        r.heartbeat(r.now() + 1_000);
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.command, out.events), (State::Blind, Some(CommandResult::Accepted), Events::NONE));
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Blind, denied(DenyReason::NotT3)));
        for c in [Command::Start, Command::Pause] {
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Blind, denied(DenyReason::NotInThisState)), "{c:?}");
        }
    }

    // ---- I8 ----

    #[test]
    fn hands_off_i8_home_latches_once_per_arming_on_the_ground() {
        let mut r = Rig::new();
        assert_eq!(r.step().home, None);
        r.i.armed = true;
        r.i.position_ne_valid = false;
        r.i.position_d_m = HOME_D;
        let out = r.report(T0 + 1_000, Switch3::High);
        assert_eq!(out.home, None, "D only");
        r.i.position_d_m = HOME_D - 0.2;
        r.i.position_ne_valid = true;
        r.i.position_ne_m = HOME_NE;
        let out = r.at(T0 + 2_000);
        let home = Some(Home {
            ne_m: HOME_NE,
            d_m: HOME_D,
        });
        assert_eq!(out.home, home, "D from the arming step, NE from the first valid step");
        r.i.position_ne_m = [50.0, 60.0];
        assert_eq!(r.at(T0 + 3_000).home, home);
        r.i.airborne = true;
        assert_eq!(r.at(T0 + 4_000).home, home);
        // Cleared at disarm; a new arming latches anew.
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!(r.at(T0 + 5_000).home, None);
        r.i.armed = true;
        r.i.position_d_m = -2.0;
        assert_eq!(
            r.report(T0 + 6_000, Switch3::High).home,
            Some(Home {
                ne_m: [50.0, 60.0],
                d_m: -2.0
            })
        );
    }

    #[test]
    fn hands_off_i8_take_off_first_gives_no_home_no_start_no_return() {
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_ne_valid = false;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        r.i.position_ne_valid = true;
        let t = r.last_report() + LOSS + 1;
        let out = r.heartbeat(t);
        assert_eq!((out.state, out.home), (State::Hold, None));
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::NoHome));
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land);
    }

    /// Seeded walk: RETURN and Start never happen without home; disarmed is STOP; `hands_off` follows the state;
    /// I12: the level the flight code may demand never exceeds this step's flags.
    #[test]
    fn hands_off_i8_i12_walk_never_returns_or_starts_without_home_and_never_demands_uncovered() {
        let mut rng = Lcg(0x5eed_0001);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::new().i;
        let mut seen = [false; 8];
        let (mut prev, mut high_seen) = (State::Stop, false);
        // C1, tracked here from the reports: K (arm ON) and P (arm ON and High in one report) since TAKEOVER's entry.
        let (mut k, mut p) = (false, false);
        let (mut hand_overs, mut held) = (0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            let out = h.step(&i);
            // A2': TAKEOVER hands over only after a High report since its entry, on a report that is not High.
            let high_now = i.crsf.is_some_and(|r| r.switch6 == Switch3::High);
            let arm_now = i.crsf.is_some_and(|r| r.arm);
            if prev == State::Takeover && out.state == State::Pilot {
                assert!(high_seen && !high_now, "hand-over: seen {high_seen}, now High {high_now}");
                // C1: only after P, on a report with switch6 not High and arm ON.
                assert!(p && !high_now && arm_now, "hand-over: P {p}, now High {high_now}, arm {arm_now}");
                hand_overs += 1;
            }
            if out.state == State::Takeover {
                if prev != State::Takeover {
                    high_seen = false;
                    (k, p) = (false, false);
                }
                high_seen |= high_now;
                k |= arm_now;
                p |= arm_now && high_now;
            }
            let k_now = out.state == State::Takeover && k;
            assert_eq!(out.arm_off_holds, out.hands_off && !k_now && i.airborne, "{out:?}");
            if i.crsf.is_some_and(|r| !r.arm) && out.state != State::Stop {
                assert!(out.arm_off_holds, "arm OFF outside STOP only while held: {out:?}");
                held += 1;
            }
            assert_eq!(out.landing, out.state == State::Land);
            if out.takeover_descent {
                assert_eq!(out.state, State::Takeover);
            }
            prev = out.state;
            if let Some(slot) = seen.get_mut(usize::from(out.state.code() - 1)) {
                *slot = true;
            }
            assert_eq!(out.hands_off, !matches!(out.state, State::Pilot | State::Stop));
            assert_eq!(out.synthetic_sticks, out.hands_off);
            if !i.armed {
                assert_eq!(out.state, State::Stop);
            }
            if out.state == State::Return || out.events.start.is_some() {
                assert!(out.home.is_some(), "{out:?}");
            }
            let covered = match out.demand {
                Demand::Position => i.tier == Tier::T3 && i.heading_ok,
                Demand::Altitude => i.tier != Tier::T1,
                Demand::Angle => true,
            };
            assert!(covered, "{:?} {} {out:?}", i.tier, i.heading_ok);
            if out.land_holds_position {
                assert!(out.state == State::Land && out.demand == Demand::Position);
            }
        }
        assert!(seen.iter().all(|&s| s), "every state reached: {seen:?}");
        assert!(hand_overs > 50 && held > 50, "lock exercised: {hand_overs} hand-overs, {held} held");
    }

    #[test]
    fn hands_off_b8_home_d_waits_for_a_valid_height_before_take_off() {
        // D invalid at the arming step, valid at the next: home, with the next step's D.
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_d_valid = false;
        assert_eq!(r.report(T0 + 1_000, Switch3::High).home, None);
        r.i.position_d_valid = true;
        r.i.position_d_m = HOME_D - 0.3;
        assert_eq!(
            r.at(T0 + 2_000).home,
            Some(Home {
                ne_m: HOME_NE,
                d_m: HOME_D - 0.3
            })
        );
        // Airborne before D is valid: no home this arming, even when D comes later.
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_d_valid = false;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        r.i.position_d_valid = true;
        assert_eq!(r.at(T0 + 3_000).home, None);
        // Disarm clears home.
        let mut r = Rig::pilot();
        assert!(r.step().home.is_some());
        r.i.armed = false;
        assert_eq!(r.step().home, None);
    }

    // ---- B1: exit priority, pairs in the same step ----

    #[test]
    fn hands_off_b1_each_pair_of_exits_gives_the_higher_one() {
        // (name, rig, the second cause; the first cause is set at the maturing step) -> state, place dropped
        // HOLD: battery critical with link > 10 s and home: LAND.
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land, "hold: battery over link");
        // HOLD: T3' lost (matured) with link > 10 s and home: LAND.
        let mut r = Rig::hold();
        r.drop_tier(Tier::T2);
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land, "hold: T3' over link");
        // HOLD: link > 10 s with a kept place and a new plan: RETURN, place dropped.
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        r.i.mission.plan_id = Some(PLAN + 1);
        let t = r.now() + LINK_LOSS_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped), (State::Return, true), "hold: link over plan");
        // HOLD: CRSF with battery critical: TAKEOVER. Disarm with CRSF: STOP.
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "hold: CRSF over battery");
        let mut r = Rig::hold();
        r.i.armed = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Stop, "disarm over CRSF");
        // AUTO: T3' lost (matured) with a new plan: LAND.
        let mut r = Rig::auto();
        r.drop_tier(Tier::T2);
        r.i.mission.plan_id = Some(PLAN + 1);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        assert_eq!(r.hold_drop().state, State::Land, "auto: T3' over plan");
        // AUTO: battery critical with station lost: LAND.
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        let t = r.now() + STATION_LOST_US + 1;
        assert_eq!(r.at(t).state, State::Land, "auto: battery over station");
        // AUTO: new plan with station lost: HOLD, dropped.
        let mut r = Rig::auto();
        r.i.mission.plan_id = Some(PLAN + 1);
        let t = r.now() + STATION_LOST_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped, out.events.place_kept), (State::Hold, true, false));
        // AUTO: last waypoint with station lost: HOLD, dropped.
        let mut r = Rig::auto();
        r.i.last_waypoint_reached = true;
        let t = r.now() + STATION_LOST_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped, out.events.place_kept), (State::Hold, true, false));
        // AUTO: CRSF with battery critical: TAKEOVER.
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "auto: CRSF over battery");
        // RETURN: CRSF with arrival: TAKEOVER.
        let mut r = Rig::ret();
        r.i.home_reached = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "return: CRSF over arrival");
    }

    // ---- B2: no one-step HOLD ----

    #[test]
    fn hands_off_b2_commands_into_hold_denied_when_hold_would_leave_at_once() {
        // LAND + battery critical: Abort DENIED BatteryCritical.
        let mut r = Rig::land();
        r.i.battery_critical = true;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::BatteryCritical)));
        assert_eq!(r.step().state, State::Land);
        // LAND + link lost: Abort DENIED LinkLost.
        let mut r = Rig::land();
        let t = r.now() + LINK_LOSS_US + 1;
        r.i.now_us = t;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::LinkLost)));
        assert_eq!(r.step().state, State::Land);
        // LAND + debounced T2: Abort DENIED NotT3.
        let mut r = Rig::land();
        r.drop_tier(Tier::T2);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        r.hold_drop();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotT3)));
        assert_eq!(r.step().state, State::Land);
        // RETURN + link lost (its own entry cause): Pause and Abort DENIED LinkLost.
        for c in [Command::Pause, Command::Abort] {
            let mut r = Rig::ret();
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Return, denied(DenyReason::LinkLost)), "{c:?}");
            assert_eq!(out.events, Events::NONE);
            assert_eq!(r.step().state, State::Return);
        }
        // Reason order: NotT3, then BatteryCritical, then LinkLost.
        let mut r = Rig::land();
        r.drop_tier(Tier::T2);
        r.i.battery_critical = true;
        let t = r.now() + LINK_LOSS_US + 1;
        r.i.now_us = t;
        assert_eq!(r.cmd(Command::Abort).command, denied(DenyReason::NotT3));
        r.i.tier = Tier::T3;
        assert_eq!(r.cmd(Command::Abort).command, denied(DenyReason::BatteryCritical));
        r.i.battery_critical = false;
        assert_eq!(r.cmd(Command::Abort).command, denied(DenyReason::LinkLost));
    }

    // ---- B4: T3' ----

    #[test]
    fn hands_off_b4_heading_not_ok_at_stick_loss_on_t3_lands() {
        let mut r = Rig::pilot();
        r.i.heading_ok = false;
        assert_eq!(r.step().state, State::Pilot);
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.land_holds_position, out.demand), (State::Land, false, Demand::Altitude));
    }

    #[test]
    fn hands_off_b4_heading_drop_in_hold_auto_return_lands() {
        let rigs: [(&str, fn() -> Rig); 3] = [("hold", Rig::hold), ("auto", Rig::auto), ("return", Rig::ret)];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            r.i.heading_ok = false;
            assert_eq!(r.step().state, from, "{name}: debounced");
            r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
            assert_eq!(r.hold_drop().state, State::Land, "{name}");
        }
    }

    #[test]
    fn hands_off_b4_land_holds_position_only_on_t3_prime() {
        let mut r = Rig::land();
        let out = r.step();
        assert_eq!((out.land_holds_position, out.demand), (true, Demand::Position));
        r.i.heading_ok = false;
        let out = r.step();
        assert_eq!((out.state, out.land_holds_position, out.demand), (State::Land, false, Demand::Altitude));
    }

    // ---- B5: debounced downgrades ----

    #[test]
    fn hands_off_b5_one_step_vertical_flag_drop_changes_no_state() {
        let rigs: [(&str, fn() -> Rig); 3] = [("hold", Rig::hold), ("auto", Rig::auto), ("return", Rig::ret)];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            let out = r.drop_tier(Tier::T1);
            assert_eq!((out.state, out.demand), (from, Demand::Angle), "{name}: the law follows the flags");
            r.i.tier = Tier::T3;
            let t = r.now() + 1_000;
            let out = r.at(t);
            assert_eq!((out.state, out.demand), (from, Demand::Position), "{name}");
        }
        // At the stick-loss step.
        let mut r = Rig::pilot();
        let t = r.last_report() + LOSS + 1;
        r.i.now_us = t;
        let out = r.drop_tier(Tier::T1);
        assert_eq!((out.state, out.demand), (State::Hold, Demand::Angle));
        r.i.tier = Tier::T3;
        assert_eq!(r.at(t + 1_000).state, State::Hold);
    }

    #[test]
    fn hands_off_b5_a_1_0_s_drop_lands_a_0_99_s_drop_does_not() {
        let mut r = Rig::hold();
        let t0 = r.now() + 1_000;
        r.i.now_us = t0;
        r.drop_tier(Tier::T2);
        assert_eq!(r.at(t0 + 990_000).state, State::Hold);
        r.i.tier = Tier::T3;
        assert_eq!(r.at(t0 + 991_000).state, State::Hold);
        let t1 = t0 + 2 * S;
        r.i.now_us = t1;
        r.drop_tier(Tier::T2);
        assert_eq!(r.at(t1 + TIER_DEBOUNCE_US - 1).state, State::Hold);
        assert_eq!(r.at(t1 + TIER_DEBOUNCE_US).state, State::Land);
    }

    // ---- B7: a kept place survives only Start ----

    #[test]
    fn hands_off_b7_every_exit_from_hold_drops_a_kept_place() {
        let paused = || {
            let mut r = Rig::auto();
            assert!(r.cmd(Command::Pause).events.place_kept);
            r
        };
        // TAKEOVER.
        let mut r = paused();
        let t = r.now() + 1_000;
        assert!(r.report(t, Switch3::Low).events.place_dropped, "takeover");
        // LAND (command and battery).
        let mut r = paused();
        assert!(r.cmd(Command::Land).events.place_dropped, "land");
        let mut r = paused();
        r.i.battery_critical = true;
        assert!(r.step().events.place_dropped, "battery");
        // STOP.
        let mut r = paused();
        r.i.armed = false;
        assert!(r.step().events.place_dropped, "stop");
        // RETURN, Abort and a plan change: hands_off_return_forgets_a_kept_place, hands_off_command_already_in_its_
        // result_is_accepted, hands_off_i6_new_plan_in_hold_drops_a_kept_place. Start keeps it:
        let mut r = paused();
        assert_eq!(r.cmd(Command::Start).events.start, Some(StartAt::KeptPlace));
    }

    // ---- B9: FS_HOLD_EN in force from disarmed steps ----

    #[test]
    fn hands_off_b9_fs_hold_en_takes_effect_at_the_next_arming() {
        // Writing 0 in HOLD keeps HOLD with hands_off until disarm.
        let mut r = Rig::hold();
        r.i.fs_hold_en = false;
        for k in 1..=5 {
            let out = r.heartbeat(r.now() + k * S);
            assert_eq!((out.state, out.hands_off), (State::Hold, true));
        }
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!(r.step().state, State::Stop);
        // Re-armed with 0 in force: stick loss in the air is STOP, and commands answer HandsOffDisabled.
        r.i.armed = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        r.i.airborne = true;
        // Writing 1 while armed in PILOT gives no HOLD at stick loss.
        r.i.fs_hold_en = true;
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.hands_off), (State::Stop, false));
        assert_eq!(r.cmd(Command::Land).command, denied(DenyReason::HandsOffDisabled));
        // After disarm and re-arm the 1 applies.
        r.i.armed = false;
        r.i.airborne = false;
        r.step();
        r.i.armed = true;
        let t = r.now() + 1_000;
        r.report(t, Switch3::High);
        r.i.airborne = true;
        let t = r.now() + 1_000;
        r.report(t, Switch3::High);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.at(t).state, State::Hold);
    }

    // ---- I9 ----

    /// HOLD with the station present, entered with the vehicle `vehicle_m` north of home (there, not moved inside the
    /// hold: H13's anchor), the farthest waypoint at `waypoint_m`.
    fn start_at(vehicle_m: f32, waypoint_m: f32) -> Output {
        let mut r = Rig::pilot();
        r.i.position_ne_m = [HOME_NE[0] + vehicle_m, HOME_NE[1]];
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        r.i.mission.max_waypoint_distance_m = Some(waypoint_m);
        r.cmd(Command::Start)
    }

    #[test]
    fn hands_off_i9_one_km_at_start() {
        assert_eq!(start_at(10.0, 1000.5).command, denied(DenyReason::WaypointBeyondRange));
        assert_eq!(start_at(1001.0, 10.0).command, denied(DenyReason::VehicleBeyondRange));
        let out = start_at(999.0, 999.0);
        assert_eq!((out.state, out.command), (State::Auto, Some(CommandResult::Accepted)));
        assert_eq!(start_at(1000.0, 1000.0).command, Some(CommandResult::Accepted));
    }

    // ---- Start conditions ----

    #[test]
    fn hands_off_each_start_condition_alone_is_denied_with_its_reason() {
        let cases: [(DenyReason, fn(&mut Rig)); 15] = [
            (DenyReason::StationAbsent, |r| {
                r.i.last_radio_heartbeat_us = Some(r.now() - STATION_LOST_US - 1)
            }),
            (DenyReason::NoWaypoints, |r| r.i.mission.has_waypoints = false),
            (DenyReason::HeightOutsideLimits, |r| r.i.mission.z_within_limits = false),
            (DenyReason::NoHome, |r| r.h.home_ne = None),
            // A move inside HOLD with the NE velocity valid would trip H13's anchor and land at once (H14).
            (DenyReason::VehicleBeyondRange, |r| {
                r.i.position_ne_m = [2000.0, 0.0];
                r.i.velocity_ne_valid = false;
            }),
            (DenyReason::VehicleBeyondRange, |r| r.i.position_ne_valid = false),
            (DenyReason::WaypointDistanceUnknown, |r| r.i.mission.max_waypoint_distance_m = None),
            (DenyReason::WaypointDistanceUnknown, |r| r.i.mission.max_waypoint_distance_m = Some(f32::NAN)),
            (DenyReason::WaypointDistanceUnknown, |r| {
                r.i.mission.max_waypoint_distance_m = Some(f32::INFINITY)
            }),
            (DenyReason::WaypointBeyondRange, |r| r.i.mission.max_waypoint_distance_m = Some(1000.5)),
            (DenyReason::VehicleBeyondRange, |r| {
                r.i.position_ne_m = [f32::NAN, 0.0];
                r.i.velocity_ne_valid = false;
            }),
            (DenyReason::HeadingNotOk, |r| r.i.heading_ok = false),
            (DenyReason::NotT3, |r| r.i.tier = Tier::T2),
            (DenyReason::BatteryNotOk, |r| r.i.battery_ok = false),
            (DenyReason::SwitchNotHigh, |r| {
                r.i.crsf = r.i.crsf.map(|c| RcSample {
                    switch6: Switch3::Middle,
                    ..c
                })
            }),
        ];
        for (reason, set) in cases {
            let mut r = Rig::hold();
            set(&mut r);
            let out = r.cmd(Command::Start);
            assert_eq!((out.state, out.command), (State::Hold, denied(reason)), "{reason:?}");
            assert_eq!(out.events, Events::NONE);
        }
        // After T3 was lost for 1.0 s HOLD has left: Start answers NotInThisState (B2).
        let mut r = Rig::hold();
        r.drop_tier(Tier::T2);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        assert_eq!(r.hold_drop().state, State::Land);
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotInThisState)));
        // All met: accepted (Rig::auto).
        let r = Rig::hold();
        assert_eq!(r.h.start_check(&r.i), None);
    }

    #[test]
    fn hands_off_switch_not_high_when_hold_came_from_a_middle_report() {
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        r.report(t, Switch3::Middle);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::SwitchNotHigh));
    }

    // ---- Commands ----

    #[test]
    fn hands_off_commands_denied_while_crsf_is_fresh() {
        for c in [Command::Start, Command::Pause, Command::Abort, Command::Land] {
            let mut r = Rig::pilot();
            assert_eq!(r.cmd(c).command, denied(DenyReason::CrsfFresh), "pilot {c:?}");
            let mut r = Rig::takeover();
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::CrsfFresh)));
            // The report and the command at the same step in HOLD: TAKEOVER, DENIED.
            let mut r = Rig::hold();
            let t = r.now() + 1_000;
            r.i.crsf = r.i.crsf.map(|s| RcSample { t_us: t, ..s });
            r.i.now_us = t;
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::CrsfFresh)));
        }
    }

    #[test]
    fn hands_off_commands_off_the_radio_link_are_denied() {
        for c in [Command::Start, Command::Pause, Command::Abort, Command::Land] {
            let mut r = Rig::hold();
            let out = r.cmd_on(c, Link::Usb);
            assert_eq!((out.state, out.command), (State::Hold, denied(DenyReason::NotRadioLink)));
        }
    }

    #[test]
    fn hands_off_no_command_arms_or_leaves_stop() {
        let mut r = Rig::new();
        r.i.crsf = None;
        for c in [Command::Start, Command::Pause, Command::Abort, Command::Land] {
            let out = r.cmd(c);
            assert_eq!(
                (out.state, out.hands_off, out.command),
                (State::Stop, false, denied(DenyReason::NotInThisState))
            );
        }
    }

    #[test]
    fn hands_off_command_already_in_its_result_is_accepted() {
        let mut r = Rig::auto();
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.command, out.events), (State::Auto, Some(CommandResult::Accepted), Events::NONE));
        // Pause in HOLD keeps a kept place; Abort in HOLD drops it.
        r.cmd(Command::Pause);
        let out = r.cmd(Command::Pause);
        assert_eq!((out.state, out.command, out.events), (State::Hold, Some(CommandResult::Accepted), Events::NONE));
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        assert!(out.events.place_dropped);
        assert_eq!(r.cmd(Command::Start).events.start, Some(StartAt::First));
        let mut r = Rig::land();
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.command), (State::Land, Some(CommandResult::Accepted)));
    }

    #[test]
    fn hands_off_commands_the_table_does_not_give_are_denied() {
        let cases: [(fn() -> Rig, Command); 4] = [
            (Rig::ret, Command::Start),
            (Rig::land, Command::Start),
            (Rig::land, Command::Pause),
            (Rig::blind, Command::Start),
        ];
        for (make, c) in cases {
            let mut r = make();
            let from = r.h.state();
            r.heartbeat(r.now() + 1_000);
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (from, denied(DenyReason::NotInThisState)), "{from:?} {c:?}");
        }
    }

    // ---- FS_HOLD_EN 0 ----

    #[test]
    fn hands_off_fs_hold_en_0_walk_gives_only_pilot_and_stop() {
        let mut rng = Lcg(0x5eed_0000);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::new().i;
        let (mut pilot, mut stop) = (0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, false);
            let out = h.step(&i);
            match out.state {
                State::Pilot => pilot += 1,
                State::Stop => stop += 1,
                other => panic!("{other:?} with FS_HOLD_EN 0"),
            }
            assert!(!out.hands_off && !out.synthetic_sticks && !out.landing);
            assert!(!out.arm_off_holds && !out.takeover_descent);
            if i.crsf.is_some_and(|r| !r.arm) {
                assert_eq!(out.state, State::Stop, "FS_HOLD_EN 0: arm OFF is STOP");
            }
            assert_eq!(out.return_height_m, None);
            assert_eq!(out.events, Events::NONE);
            assert!(matches!(out.command, None | Some(CommandResult::Denied(_))));
        }
        assert!(pilot > 1_000 && stop > 1_000, "both reached: {pilot} {stop}");
    }

    // ---- Review follow-up ----

    /// Amended by C1: arm OFF in a hands-off state in the air holds (K is clear there); on the ground, and in PILOT,
    /// it is STOP in the same step.
    #[test]
    fn hands_off_arm_switch_off_in_the_same_step_from_every_state() {
        let rigs: [(&str, fn() -> Rig, State); 6] = [
            ("pilot", Rig::pilot, State::Pilot),
            ("hold", Rig::hold, State::Hold),
            ("auto", Rig::auto, State::Auto),
            ("return", Rig::ret, State::Return),
            ("land", Rig::land, State::Land),
            ("blind", Rig::blind, State::Blind),
        ];
        for (name, make, from) in rigs {
            for airborne in [true, false] {
                let mut r = make();
                assert_eq!(r.h.state(), from, "{name}");
                r.i.airborne = airborne;
                let t = r.now() + 1_000;
                let out = r.report_with(t, CENTRED, Switch3::High, false);
                if airborne && from != State::Pilot {
                    assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Takeover, true, true), "{name}");
                } else {
                    let want = (State::Stop, false, false, false);
                    let got = (out.state, out.hands_off, out.synthetic_sticks, out.arm_off_holds);
                    assert_eq!(got, want, "{name} airborne {airborne}");
                }
            }
        }
    }

    #[test]
    fn hands_off_link_age_is_from_the_newer_of_crsf_and_heartbeat() {
        // The heartbeat is 30 s old; the last CRSF report counts: no RETURN until more than 10 s after it.
        let mut r = Rig::hold();
        let t0 = r.last_report();
        r.i.last_radio_heartbeat_us = Some(r.now() - 30 * S);
        assert_eq!(r.at(t0 + 5 * S).state, State::Hold);
        assert_eq!(r.at(t0 + LINK_LOSS_US).state, State::Hold);
        assert_eq!(r.at(t0 + LINK_LOSS_US + 1).state, State::Return);
    }

    #[test]
    fn hands_off_link_loss_is_strictly_above_10_s() {
        let mut r = Rig::hold();
        let t_hb = r.now();
        assert_eq!(r.at(t_hb + LINK_LOSS_US).state, State::Hold, "exactly 10.0 s");
        assert_eq!(r.at(t_hb + LINK_LOSS_US + 1).state, State::Return, "one step later");
    }

    #[test]
    fn hands_off_b5_a_tier_upgrade_takes_effect_at_once() {
        // T2 for longer than the debounce with fresh sticks, then T3 at the stick-loss step: HOLD, not LAND.
        let mut r = Rig::pilot();
        r.drop_tier(Tier::T2);
        let t = r.now() + 2 * TIER_DEBOUNCE_US;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        assert_eq!(r.h.debounced, Tier::T2);
        r.i.tier = Tier::T3;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        // In LAND on a matured T2: Abort is denied; at the step T3 is back it is accepted.
        let mut r = Rig::land();
        r.drop_tier(Tier::T2);
        assert_eq!(r.hold_drop().state, State::Land);
        assert_eq!(r.cmd(Command::Abort).command, denied(DenyReason::NotT3));
        r.i.tier = Tier::T3;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
    }

    #[test]
    fn hands_off_start_deny_order_is_the_contracts() {
        let mut r = Rig::hold();
        r.i.last_radio_heartbeat_us = Some(r.now() - STATION_LOST_US - 1);
        r.i.mission.has_waypoints = false;
        r.i.mission.z_within_limits = false;
        r.h.home_ne = None;
        r.i.position_ne_valid = false;
        r.i.mission.max_waypoint_distance_m = None;
        r.i.heading_ok = false;
        r.i.tier = Tier::T2;
        r.i.battery_ok = false;
        r.i.crsf = r.i.crsf.map(|c| RcSample {
            switch6: Switch3::Middle,
            ..c
        });
        let steps: [(DenyReason, fn(&mut Rig)); 11] = [
            (DenyReason::StationAbsent, |r| r.i.last_radio_heartbeat_us = Some(r.now())),
            (DenyReason::NoWaypoints, |r| r.i.mission.has_waypoints = true),
            (DenyReason::HeightOutsideLimits, |r| r.i.mission.z_within_limits = true),
            (DenyReason::NoHome, |r| r.h.home_ne = Some(HOME_NE)),
            (DenyReason::VehicleBeyondRange, |r| r.i.position_ne_valid = true),
            (DenyReason::WaypointDistanceUnknown, |r| {
                r.i.mission.max_waypoint_distance_m = Some(1500.0)
            }),
            (DenyReason::WaypointBeyondRange, |r| {
                r.i.mission.max_waypoint_distance_m = Some(100.0)
            }),
            (DenyReason::HeadingNotOk, |r| r.i.heading_ok = true),
            (DenyReason::NotT3, |r| r.i.tier = Tier::T3),
            (DenyReason::BatteryNotOk, |r| r.i.battery_ok = true),
            (DenyReason::SwitchNotHigh, |r| {
                r.i.crsf = r.i.crsf.map(|c| RcSample {
                    switch6: Switch3::High,
                    ..c
                })
            }),
        ];
        for (reason, fix) in steps {
            assert_eq!(r.h.start_check(&r.i), Some(reason));
            fix(&mut r);
        }
        assert_eq!(r.h.start_check(&r.i), None);
    }

    #[test]
    fn hands_off_start_is_denied_when_the_battery_is_critical_even_if_ok_is_set() {
        let mut r = Rig::hold();
        assert_eq!(r.h.start_check(&r.i), None);
        r.i.battery_ok = true;
        r.i.battery_critical = true;
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::BatteryNotOk));
    }

    #[test]
    fn hands_off_home_latches_only_finite_values() {
        let bads = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];
        for bad in bads {
            for which in 0..3 {
                let mut r = Rig::new();
                r.step();
                r.i.armed = true;
                match which {
                    0 => r.i.position_d_m = bad,
                    1 => r.i.position_ne_m = [bad, 0.0],
                    _ => r.i.position_ne_m = [0.0, bad],
                }
                assert_eq!(r.report(T0 + 1_000, Switch3::High).home, None, "{bad} {which}");
                assert_eq!(r.at(T0 + 2_000).home, None, "{bad} {which}");
                // Finite values later in the same window latch.
                r.i.position_d_m = HOME_D;
                r.i.position_ne_m = HOME_NE;
                assert_eq!(
                    r.at(T0 + 3_000).home,
                    Some(Home {
                        ne_m: HOME_NE,
                        d_m: HOME_D
                    })
                );
            }
        }
    }

    // ---- C1: the take-over lock ----

    /// HOLD, then CRSF returns with this arm switch and switch6.
    fn returns_with(switch6: Switch3, arm: bool) -> (Rig, Output) {
        let mut r = Rig::hold();
        let t = r.now() + 1_000;
        let out = r.report_with(t, CENTRED, switch6, arm);
        (r, out)
    }

    #[test]
    fn hands_off_c1_a1_arm_off_return_holds_then_arm_on_high_opens_then_middle_hands_over() {
        let (mut r, out) = returns_with(Switch3::High, false);
        assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Takeover, true, true));
        assert!(out.synthetic_sticks);
        // More arm-OFF reports: holding, latches unchanged.
        for _ in 0..5 {
            let t = r.now() + 20_000;
            let out = r.report_with(t, FULL, Switch3::High, false);
            assert_eq!((out.state, out.arm_off_holds), (State::Takeover, true));
            assert!(!r.h.arm_seen && !r.h.lock_open);
        }
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::High, true);
        assert_eq!((out.state, out.arm_off_holds), (State::Takeover, false), "lock open");
        assert!(r.h.arm_seen && r.h.lock_open);
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Middle, true);
        assert_eq!((out.state, out.hands_off, out.synthetic_sticks), (State::Pilot, false, false));
    }

    #[test]
    fn hands_off_c1_lock_needs_arm_on_and_high_in_the_same_report() {
        // High with arm OFF, then Middle with arm ON: K but no P, no hand-over.
        let (mut r, _) = returns_with(Switch3::High, false);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Middle, true).state, State::Takeover);
        assert!(r.h.arm_seen && !r.h.lock_open);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Low, true).state, State::Takeover);
        // Then arm ON and High: P; then Low hands over.
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::High, true).state, State::Takeover);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Low, true).state, State::Pilot);
    }

    #[test]
    fn hands_off_c1_a2_after_k_arm_off_is_stop_at_that_report() {
        // K from a report with arm ON and Low (no P).
        let (mut r, out) = returns_with(Switch3::Low, true);
        assert_eq!((out.state, out.arm_off_holds), (State::Takeover, false));
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Low, false);
        assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Stop, false, false));
        // After P too: arm OFF with Middle is STOP, not a hand-over.
        let (mut r, _) = returns_with(Switch3::High, true);
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Middle, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
    }

    #[test]
    fn hands_off_c1_a3_arm_off_with_the_airborne_latch_false_is_stop_at_once() {
        // CRSF returns on the ground with arm OFF.
        let mut r = Rig::hold();
        r.i.airborne = false;
        let t = r.now() + 1_000;
        let out = r.report_with(t, CENTRED, Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
        // Held in TAKEOVER in the air, then the latch clears with the arm-OFF report still the newest: STOP.
        let (mut r, _) = returns_with(Switch3::High, false);
        r.i.airborne = false;
        let t = r.now() + 20_000;
        let out = r.at(t);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
    }

    #[test]
    fn hands_off_c1_a4_rc_lost_in_takeover_clears_k_and_p_and_a_second_arm_off_return_holds() {
        let (mut r, _) = returns_with(Switch3::High, true);
        assert!(r.h.arm_seen && r.h.lock_open);
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.arm_off_holds), (State::Hold, true));
        assert!(!r.h.arm_seen && !r.h.lock_open);
        let t = r.now() + 1_000;
        let out = r.report_with(t, CENTRED, Switch3::Middle, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Takeover, true), "second return holds");
        // P was cleared: arm ON with Middle is not a hand-over.
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, CENTRED, Switch3::Middle, true).state, State::Takeover);
        // RC lost after an arm-OFF return: the stale arm-OFF report does not stop the motors in the air.
        let (mut r, _) = returns_with(Switch3::High, false);
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Hold, true, true));
        let t = r.now() + 600 * S;
        assert_ne!(r.at(t).state, State::Stop);
    }

    #[test]
    fn hands_off_c1_a6_pilot_arm_off_is_stop_at_once() {
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        let out = r.report_with(t, FULL, Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
        // FS_HOLD_EN 0 in force: the same.
        let mut r = Rig::new();
        r.i.fs_hold_en = false;
        r.step();
        r.i.armed = true;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        let out = r.report_with(T0 + 2_000, FULL, Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
    }

    #[test]
    fn hands_off_c1_a7_takeover_with_battery_critical_descends_hands_over_and_touchdown_disarms() {
        let mut r = Rig::takeover();
        let out = r.step();
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Takeover, false, false));
        r.i.battery_critical = true;
        let out = r.step();
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Takeover, false, true));
        // Latched for this TAKEOVER, as LAND is.
        r.i.battery_critical = false;
        assert!(r.step().takeover_descent);
        // Hand-over stays possible.
        let t = r.now() + 20_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.now() + 20_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Pilot, false, false));
        // Touchdown: the latch clears, STOP and disarm in that step (WR1, WR2).
        let mut r = Rig::takeover();
        r.i.battery_critical = true;
        assert!(r.step().takeover_descent);
        r.i.airborne = false;
        let out = r.step();
        assert_eq!((out.state, out.disarm, out.arm_off_holds), (State::Stop, true, false));
        r.i.armed = false;
        assert_eq!((r.step().state, r.step().disarm), (State::Stop, false));
        // Battery critical at the step CRSF returns: the descent starts at that step.
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Low);
        assert_eq!((out.state, out.takeover_descent), (State::Takeover, true));
        // RC lost: by tier, the descent flag gone (HOLD's own battery exit lands at the next step).
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.takeover_descent), (State::Hold, false));
        assert_eq!(r.step().state, State::Land);
    }

    // ---- C2 ----

    #[test]
    fn hands_off_c2_start_denied_arm_switch_off_when_the_last_report_had_arm_off() {
        let (mut r, _) = returns_with(Switch3::High, false);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.command), (State::Hold, denied(DenyReason::ArmSwitchOff)));
    }

    #[test]
    fn hands_off_c2_arm_switch_off_comes_after_battery_and_before_switch_not_high() {
        let mut r = Rig::hold();
        r.i.battery_ok = false;
        r.i.crsf = r.i.crsf.map(|c| RcSample {
            arm: false,
            switch6: Switch3::Middle,
            ..c
        });
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::BatteryNotOk));
        r.i.battery_ok = true;
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::ArmSwitchOff));
        r.i.crsf = r.i.crsf.map(|c| RcSample { arm: true, ..c });
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::SwitchNotHigh));
    }

    // ---- C3: Return ----

    #[test]
    fn hands_off_c3_return_from_hold_and_auto_drops_the_place_and_latches_the_height() {
        // HOLD with a kept place, 10 m up, mission return height 20 m.
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Return, Some(CommandResult::Accepted)));
        assert_eq!((out.events.place_dropped, out.return_height_m), (true, Some(20.0)));
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::NotInThisState));
        // AUTO: mission 5 m, present 10 m: 10.
        let mut r = Rig::auto();
        r.i.mission.return_height_m = Some(5.0);
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Return, Some(CommandResult::Accepted)));
        assert_eq!((out.events.place_dropped, out.return_height_m), (true, Some(10.0)));
    }

    #[test]
    fn hands_off_c3_return_in_return_is_accepted_with_no_effect() {
        let mut r = Rig::ret();
        let before = r.step();
        r.i.position_d_m = HOME_D - 40.0;
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command, out.events), (State::Return, Some(CommandResult::Accepted), Events::NONE));
        assert_eq!(out.return_height_m, before.return_height_m);
    }

    #[test]
    fn hands_off_c3_return_elsewhere_is_not_in_this_state() {
        let rigs: [(&str, fn() -> Rig); 2] = [("land", Rig::land), ("blind", Rig::blind)];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            r.heartbeat(r.now() + 1_000);
            let out = r.cmd(Command::Return);
            assert_eq!((out.state, out.command), (from, denied(DenyReason::NotInThisState)), "{name}");
        }
        let mut r = Rig::new();
        r.i.crsf = None;
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Stop, denied(DenyReason::NotInThisState)));
        // PILOT and TAKEOVER have a fresh report: CrsfFresh comes first.
        let mut r = Rig::pilot();
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::CrsfFresh));
        let mut r = Rig::takeover();
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::CrsfFresh)));
    }

    #[test]
    fn hands_off_c3_return_deny_reasons_and_their_order() {
        // The common four.
        let mut r = Rig::hold();
        assert_eq!(r.cmd_on(Command::Return, Link::Usb).command, denied(DenyReason::NotRadioLink));
        let mut r = Rig::hold();
        let t = r.now() + 1_000;
        r.i.crsf = r.i.crsf.map(|s| RcSample { t_us: t, ..s });
        r.i.now_us = t;
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::CrsfFresh));
        let mut r = Rig::hold();
        r.h.fs_in_force = false;
        r.i.fs_hold_en = false;
        r.i.armed = true;
        let mut ev = Events::NONE;
        let ret = StationCommand {
            command: Command::Return,
            link: Link::Radio,
        };
        assert_eq!(r.h.command(ret, &r.i, false, &mut ev), CommandResult::Denied(DenyReason::HandsOffDisabled));
        // NoHome: a flight with take-off before any NE.
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_ne_valid = false;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        r.i.position_ne_valid = true;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Hold, denied(DenyReason::NoHome)));
        // NotT3 and BatteryCritical (B2): HOLD's and AUTO's own exits take these facts first at step(), so the check
        // is driven directly; then NoHome before NotT3 before BatteryCritical.
        let rigs: [(&str, fn() -> Rig); 2] = [("hold", Rig::hold), ("auto", Rig::auto)];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            r.h.debounced = Tier::T2;
            r.i.battery_critical = true;
            r.h.home_ne = None;
            let mut ev = Events::NONE;
            for want in [DenyReason::NoHome, DenyReason::NotT3, DenyReason::BatteryCritical] {
                assert_eq!(r.h.command(ret, &r.i, false, &mut ev), CommandResult::Denied(want), "{name}");
                assert_eq!((r.h.state(), ev), (from, Events::NONE), "{name} {want:?}");
                match want {
                    DenyReason::NoHome => r.h.home_ne = Some(HOME_NE),
                    DenyReason::NotT3 => r.h.debounced = Tier::T3,
                    _ => r.i.battery_critical = false,
                }
            }
            assert_eq!(r.h.command(ret, &r.i, false, &mut ev), CommandResult::Accepted, "{name}");
        }
    }

    /// B2 for Return: DENIED leaves the state, at this step and the next. Control: without the check, RETURN appears
    /// for one step and then lands.
    #[test]
    fn hands_off_c3_return_no_bounce() {
        let ret = StationCommand {
            command: Command::Return,
            link: Link::Radio,
        };
        for battery in [true, false] {
            let mut r = Rig::hold();
            if battery {
                r.i.battery_critical = true;
            } else {
                assert_eq!(r.drop_tier(Tier::T2).state, State::Hold);
                r.h.debounced = Tier::T2;
            }
            let mut ev = Events::NONE;
            let want = if battery {
                DenyReason::BatteryCritical
            } else {
                DenyReason::NotT3
            };
            assert_eq!(r.h.command(ret, &r.i, false, &mut ev), CommandResult::Denied(want));
            assert_eq!(r.h.state(), State::Hold, "{want:?}: this step");
            r.i.now_us += TIER_DEBOUNCE_US;
            assert_eq!(r.step().state, State::Land, "{want:?}: the next step lands, never RETURN");
        }
    }

    #[test]
    fn hands_off_c3_return_already_at_home_is_accepted_and_lands_at_the_next_step() {
        let mut r = Rig::hold();
        r.i.home_reached = true;
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Return, Some(CommandResult::Accepted)));
        assert_eq!(r.step().state, State::Land);
    }

    #[test]
    fn hands_off_c3_deny_reason_codes_are_the_contracts() {
        let table = [
            (DenyReason::NotRadioLink, 1),
            (DenyReason::CrsfFresh, 2),
            (DenyReason::HandsOffDisabled, 3),
            (DenyReason::NotInThisState, 4),
            (DenyReason::StationAbsent, 5),
            (DenyReason::NoWaypoints, 6),
            (DenyReason::HeightOutsideLimits, 7),
            (DenyReason::NoHome, 8),
            (DenyReason::VehicleBeyondRange, 9),
            (DenyReason::WaypointDistanceUnknown, 10),
            (DenyReason::WaypointBeyondRange, 11),
            (DenyReason::HeadingNotOk, 12),
            (DenyReason::NotT3, 13),
            (DenyReason::BatteryNotOk, 14),
            (DenyReason::ArmSwitchOff, 15),
            (DenyReason::SwitchNotHigh, 16),
            (DenyReason::BatteryCritical, 17),
            (DenyReason::LinkLost, 18),
        ];
        for (k, (reason, code)) in table.iter().enumerate() {
            assert_eq!(reason.code(), *code, "{reason:?}");
            assert_eq!(usize::from(reason.code()), k + 1, "{reason:?}: 1..=18, 0 reserved");
        }
    }

    // ---- Amendment D: WR1 ground exit, WR2 disarm, return_built ----

    #[test]
    fn hands_off_wr1_every_hands_off_state_with_the_latch_false_is_stop_in_that_step() {
        let rigs: [(&str, fn() -> Rig); 6] = [
            ("takeover", Rig::takeover),
            ("hold", Rig::hold),
            ("auto", Rig::auto),
            ("return", Rig::ret),
            ("land", Rig::land),
            ("blind", Rig::blind),
        ];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            let out = r.step();
            assert_eq!((out.state, out.disarm), (from, false), "{name}: airborne, nothing changes");
            r.i.airborne = false;
            let out = r.step();
            assert_eq!((out.state, out.hands_off, out.disarm), (State::Stop, false, true), "{name}");
        }
    }

    #[test]
    fn hands_off_wr1_a_takeover_or_land_entered_with_the_latch_false_stops_at_once() {
        // CRSF returns in HOLD with the latch already false: STOP, not TAKEOVER.
        let mut r = Rig::hold();
        r.i.airborne = false;
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!((out.state, out.disarm), (State::Stop, true));
        // Land in HOLD with the latch false: STOP; the command finds STOP.
        let mut r = Rig::hold();
        r.i.airborne = false;
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.disarm, out.command), (State::Stop, true, denied(DenyReason::NotInThisState)));
        // Stick loss on T2 or T1 with the latch false: STOP, not LAND or BLIND.
        for tier in [Tier::T2, Tier::T1] {
            let mut r = Rig::pilot();
            r.drop_tier(tier);
            r.i.airborne = false;
            let t = r.last_report() + LOSS + 1;
            let out = r.at(t);
            assert_eq!((out.state, out.disarm), (State::Stop, true), "{tier:?}");
        }
    }

    #[test]
    fn hands_off_wr1_touchdown_paths_end_in_stop() {
        // LAND: the latch clears, STOP and disarm in that step; the next (disarmed) step stays STOP.
        let mut r = Rig::land();
        r.i.airborne = false;
        let out = r.step();
        assert_eq!((out.state, out.landing, out.disarm), (State::Stop, false, true));
        r.i.armed = false;
        let out = r.step();
        assert_eq!((out.state, out.disarm), (State::Stop, false));
        // Disarm reported by the gate first (old path): STOP, no second disarm.
        let mut r = Rig::land();
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!((r.step().state, r.h.state()), (State::Stop, State::Stop));
    }

    #[test]
    fn hands_off_wr2_disarm_is_stop_entered_while_armed() {
        // The review's case: a held arm-OFF report still the newest when the latch clears.
        let (mut r, out) = returns_with(Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds, out.disarm), (State::Takeover, true, false));
        r.i.airborne = false;
        let t = r.now() + 5_000;
        let out = r.at(t);
        assert_eq!((out.state, out.arm_off_holds, out.disarm), (State::Stop, false, true));
        // Arm OFF in PILOT; after K in TAKEOVER; sticks lost on the ground; FS_HOLD_EN 0.
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        assert!(r.report_with(t, FULL, Switch3::High, false).disarm, "pilot arm off");
        let (mut r, _) = returns_with(Switch3::Low, true);
        let t = r.now() + 20_000;
        assert!(r.report_with(t, FULL, Switch3::Low, false).disarm, "after K");
        let mut r = Rig::pilot();
        r.i.airborne = false;
        let t = r.last_report() + LOSS + 1;
        assert!(r.at(t).disarm, "sticks lost on the ground");
        let mut r = Rig::new();
        r.i.fs_hold_en = false;
        r.step();
        r.i.armed = true;
        assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
        let out = r.at(T0 + 1_000 + LOSS + 1);
        assert_eq!((out.state, out.disarm), (State::Stop, true), "FS_HOLD_EN 0: redundant with the gate's RC loss");
        // Not on the arming step, not while disarmed, not when STOP persists.
        let mut r = Rig::new();
        let out = r.step();
        assert_eq!((out.state, out.disarm), (State::Stop, false), "disarmed");
        r.i.armed = true;
        r.i.crsf = None;
        let out = r.at(T0 + 1_000);
        assert_eq!((out.state, out.disarm), (State::Stop, false), "armed, already STOP");
        let out = r.report(T0 + 2_000, Switch3::High);
        assert_eq!((out.state, out.disarm), (State::Pilot, false), "arming");
    }

    fn no_mission() -> MissionFacts {
        MissionFacts {
            plan_id: None,
            has_waypoints: false,
            z_within_limits: true,
            max_waypoint_distance_m: None,
            return_height_m: None,
        }
    }

    #[test]
    fn hands_off_return_built_false_latches_no_home_and_link_loss_lands() {
        let mut r = Rig::new();
        r.i.return_built = false;
        r.i.mission = no_mission();
        r.step();
        r.i.armed = true;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        let out = r.report(T0 + 2_000, Switch3::High);
        assert_eq!((out.state, out.home), (State::Pilot, None));
        let t = r.last_report() + LOSS + 1;
        let out = r.heartbeat(t);
        assert_eq!((out.state, out.home), (State::Hold, None));
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::NoWaypoints));
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::NoHome));
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land);
    }

    #[test]
    fn hands_off_return_built_false_walk_never_reaches_return_or_auto() {
        let mut rng = Lcg(0x5eed_00d3);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::new().i;
        let (mut hold, mut start_denied) = (0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            i.return_built = false;
            i.mission = no_mission();
            let out = h.step(&i);
            assert!(!matches!(out.state, State::Return | State::Auto), "{out:?}");
            assert_eq!(out.home, None);
            assert!(out.events.start.is_none());
            if let Some(c) = i.command
                && c.command == Command::Start
            {
                assert_ne!(out.command, Some(CommandResult::Accepted));
                start_denied += u32::from(out.command == denied(DenyReason::NoWaypoints));
            }
            hold += u32::from(out.state == State::Hold);
        }
        // H14: a track fault (the random positions trip the anchor) leaves HOLD at that step, not 1.0 s later.
        assert!(hold > 500 && start_denied > 10, "{hold} {start_denied}");
    }

    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            self.0 >> 33
        }

        fn chance(&mut self, per_1000: u64) -> bool {
            self.next() % 1000 < per_1000
        }

        fn pick<T: Copy>(&mut self, of: &[T]) -> T {
            of[(self.next() as usize) % of.len()]
        }
    }

    /// One random step: time moves 0 to 2 s (now and then 10 to 12 s with no report and no heartbeat), every input
    /// may change; armed and airborne change rarely.
    fn random_inputs(rng: &mut Lcg, i: &mut Inputs, fs_hold_en: bool) {
        i.fs_hold_en = fs_hold_en;
        let silent = rng.chance(5);
        i.now_us += if silent {
            LINK_LOSS_US + rng.next() % 2_000_000
        } else {
            rng.next() % if rng.chance(900) { 50_000 } else { 2_000_000 }
        };
        let now = i.now_us;
        if !silent && rng.chance(300) {
            let switch6 = rng.pick(&[Switch3::Low, Switch3::Middle, Switch3::High]);
            i.crsf = Some(RcSample {
                sticks: FULL,
                arm: !rng.chance(20),
                switch6,
                t_us: now,
            });
        }
        if !silent && rng.chance(200) {
            i.last_radio_heartbeat_us = Some(now);
        }
        if rng.chance(200) {
            i.last_radio_status_us = Some(now);
        }
        if rng.chance(10) {
            i.armed = !i.armed;
        }
        if rng.chance(10) {
            i.airborne = !i.airborne;
        }
        if rng.chance(50) {
            i.tier = rng.pick(&[Tier::T3, Tier::T3, Tier::T3, Tier::T2, Tier::T1]);
        }
        i.position_ne_valid = !rng.chance(100);
        i.position_d_valid = !rng.chance(50);
        i.position_ne_m = [(rng.next() % 2400) as f32 - 1200.0, (rng.next() % 2400) as f32 - 1200.0];
        i.position_d_m = -((rng.next() % 60) as f32);
        i.command = if rng.chance(300) {
            Some(StationCommand {
                command: rng.pick(&[Command::Start, Command::Pause, Command::Abort, Command::Land, Command::Return]),
                link: if rng.chance(900) { Link::Radio } else { Link::Usb },
            })
        } else {
            None
        };
        if rng.chance(20) {
            i.mission = MissionFacts {
                plan_id: rng.pick(&[None, Some(PLAN), Some(PLAN + 1)]),
                has_waypoints: !rng.chance(100),
                z_within_limits: !rng.chance(100),
                max_waypoint_distance_m: rng.pick(&[None, Some(100.0), Some(999.0), Some(1500.0)]),
                return_height_m: rng.pick(&[None, Some(5.0), Some(30.0)]),
            };
        }
        i.last_waypoint_reached = rng.chance(5);
        i.home_reached = rng.chance(5);
        i.heading_ok = !rng.chance(50);
        i.battery_ok = !rng.chance(50);
        i.battery_critical = rng.chance(3);
    }

    // ---- H12: the hold-integrity distance check ----

    /// One step over X, then back below: set at the step that receives the distance, kept until the disarm edge; LAND
    /// at that step (a landing cause of its own, H14 (1): no debounce), T3' kept (the demand stays Position), the braking
    /// law at that step and every later one (I-H2).
    #[test]
    fn h12_one_step_over_x_latches_until_the_disarm_edge_and_lands_braking_at_that_step() {
        let mut r = Rig::hold();
        r.i.track_error_m = Some(5.5);
        let out = r.step();
        assert!(r.h.track_fault());
        assert_eq!(
            (out.state, out.demand, out.land_law, out.land_holds_position),
            (State::Land, Demand::Position, Some(LandLaw::Brake), false)
        );
        r.i.track_error_m = Some(0.0);
        let t0 = r.now();
        for k in 1..2_000 {
            let out = r.at(t0 + k * 1_000);
            assert_eq!((out.state, out.land_law, out.land_holds_position), (State::Land, Some(LandLaw::Brake), false), "step {k}");
            assert!(r.h.track_fault(), "step {k}");
        }
        // The disarm edge clears it; the next arming starts clear.
        r.i.armed = false;
        assert_eq!(r.step().state, State::Stop);
        assert!(!r.h.track_fault());
        r.i.armed = true;
        r.i.airborne = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        assert!(!r.h.track_fault());
    }

    #[test]
    fn h12_exactly_x_does_not_trip_a_hair_above_and_not_a_number_do() {
        let x = track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms);
        let above = f32::from_bits(x.to_bits() + 1);
        for (d, trips) in [
            (0.0, false),
            (x, false),
            (above, true),
            (f32::NAN, true),
            (f32::INFINITY, true),
        ] {
            let mut r = Rig::hold();
            r.i.track_error_m = Some(d);
            let out = r.step();
            assert_eq!(r.h.track_fault(), trips, "{d}");
            assert_eq!((out.state == State::Land, out.demand), (trips, Demand::Position), "{d}");
        }
    }

    /// The states the check covers (HOLD, AUTO, RETURN, LAND on T3', and TAKEOVER: H13 (3), it flies HOLD's law);
    /// never LAND without T3', BLIND, PILOT, STOP, a disarmed step or a step with the airborne latch false.
    #[test]
    fn h12_only_the_covered_states_armed_and_airborne_set_it() {
        let land_t2 = || {
            let mut r = Rig::land();
            r.i.tier = Tier::T2;
            assert!(!r.step().land_holds_position);
            r
        };
        let cases: [(&str, fn() -> Rig, bool); 8] = [
            ("hold", Rig::hold, true),
            ("auto", Rig::auto, true),
            ("return", Rig::ret, true),
            ("takeover", Rig::takeover, true),
            ("land on T3'", Rig::land, true),
            ("land without T3'", land_t2, false),
            ("blind", Rig::blind, false),
            ("pilot", Rig::pilot, false),
        ];
        for (name, rig, covered) in cases {
            for d in [100.0, f32::NAN] {
                let mut r = rig();
                r.i.track_error_m = Some(d);
                r.step();
                assert_eq!(r.h.track_fault(), covered, "{name} {d}");
            }
        }
        // The airborne latch false: the ground exit, never the fault.
        let mut r = Rig::hold();
        r.i.airborne = false;
        r.i.track_error_m = Some(100.0);
        assert_eq!(r.step().state, State::Stop);
        assert!(!r.h.track_fault());
        // Disarmed.
        let mut r = Rig::new();
        r.i.track_error_m = Some(f32::NAN);
        for _ in 0..10 {
            r.step();
            assert!(!r.h.track_fault());
        }
    }

    /// PILOT: any distance changes nothing, and stick loss later still gives HOLD.
    #[test]
    fn h12_in_pilot_the_distance_never_sets_it_and_never_changes_an_output() {
        let (mut a, mut b) = (Rig::pilot(), Rig::pilot());
        b.i.track_error_m = Some(f32::NAN);
        for k in 0..500u64 {
            let t = a.now() + 1_000;
            b.i.track_error_m = Some(if k % 2 == 0 { f32::NAN } else { 50.0 });
            assert_eq!(a.report(t, Switch3::High), b.report(t, Switch3::High), "step {k}");
            assert!(!b.h.track_fault());
        }
        let t = b.last_report() + LOSS + 1;
        assert_eq!(b.heartbeat(t).state, State::Hold);
    }

    /// The distance belongs to the step it was measured in: the input of the step that enters HOLD is PILOT's and is
    /// not checked; the next step's is HOLD's.
    #[test]
    fn h12_the_entry_step_checks_the_previous_state_s_distance() {
        let mut r = Rig::pilot();
        r.i.track_error_m = Some(100.0);
        let t = r.last_report() + LOSS + 1;
        let out = r.heartbeat(t);
        assert_eq!((out.state, out.demand), (State::Hold, Demand::Position));
        assert!(!r.h.track_fault());
        r.step();
        assert!(r.h.track_fault());
    }

    /// H14 (1): heading_ok = the input only: T3' (the demand) follows the input whatever the fault; the fault lands at
    /// once, so Start meets LAND (NotInThisState); Start's own check denies a latched fault as HeadingNotOk.
    #[test]
    fn h14_heading_ok_is_the_input_only_and_start_is_denied_with_the_fault() {
        for (heading, fault) in [(true, false), (true, true), (false, false), (false, true)] {
            let mut r = Rig::hold();
            r.i.heading_ok = heading;
            if fault {
                r.i.track_error_m = Some(6.0);
            }
            let out = r.step();
            r.i.track_error_m = None;
            assert_eq!(out.demand == Demand::Position, heading, "{heading} {fault}");
            assert_eq!(out.state, if fault { State::Land } else { State::Hold }, "{heading} {fault}");
            let want = match (heading, fault) {
                (_, true) => denied(DenyReason::NotInThisState),
                (true, false) => Some(CommandResult::Accepted),
                (false, false) => denied(DenyReason::HeadingNotOk),
            };
            assert_eq!(r.cmd(Command::Start).command, want, "{heading} {fault}");
        }
        let mut r = Rig::hold();
        assert_eq!(r.h.start_check(&r.i), None);
        r.h.track_fault = true;
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::HeadingNotOk));
    }

    // ---- H13: the anchor, the braking allowance, the speed cap and X ----

    /// PILOT, then sticks lost at `ne` moving `v`: HOLD.
    fn hold_from(ne: [f32; 2], v: [f32; 2]) -> Rig {
        let mut r = Rig::pilot();
        r.i.position_ne_m = ne;
        r.i.velocity_ne_ms = v;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        r
    }

    /// H13 (1): none in PILOT; at the entry step A = the position and b = |v|^2 / (2 x 3.0 m/s^2).
    #[test]
    fn h13_the_anchor_and_the_braking_allowance_are_set_at_the_entry_step() {
        assert_eq!(Rig::pilot().h.anchor(), None);
        let r = hold_from([20.0, -7.0], [3.0, -4.0]);
        assert_eq!(r.h.anchor(), Some(([20.0, -7.0], 25.0 / 6.0)));
        let r = hold_from([20.0, -7.0], [20.0, 0.0]);
        assert_eq!(r.h.anchor(), Some(([20.0, -7.0], 400.0 / 6.0)), "67 m at 20 m/s");
    }

    /// AUTO and RETURN clear the anchor; entry from them to HOLD (Pause), TAKEOVER (a report) or LAND on T3' (the Land
    /// command, RETURN's arrival) sets a new one; LAND entered without T3' sets none.
    #[test]
    fn h13_auto_and_return_clear_the_anchor_and_entry_from_them_sets_a_new_one() {
        let mut r = Rig::auto();
        assert_eq!(r.h.anchor(), None);
        r.i.position_ne_m = [40.0, 10.0];
        r.i.velocity_ne_ms = [0.0, 3.0];
        assert_eq!(r.cmd(Command::Pause).state, State::Hold);
        assert_eq!(r.h.anchor(), Some(([40.0, 10.0], 1.5)));
        let mut r = Rig::auto();
        r.i.position_ne_m = [41.0, 10.0];
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover);
        assert_eq!(r.h.anchor(), Some(([41.0, 10.0], 0.0)));
        let mut r = Rig::auto();
        r.i.position_ne_m = [42.0, 10.0];
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.land_holds_position), (State::Land, true));
        assert_eq!(r.h.anchor(), Some(([42.0, 10.0], 0.0)));
        let mut r = Rig::ret();
        assert_eq!(r.h.anchor(), None);
        r.i.position_ne_m = HOME_NE;
        r.i.home_reached = true;
        let out = r.step();
        assert_eq!((out.state, out.land_holds_position), (State::Land, true));
        assert_eq!(r.h.anchor(), Some((HOME_NE, 0.0)));
        let mut r = Rig::auto();
        r.drop_tier(Tier::T2);
        let out = r.hold_drop();
        assert_eq!((out.state, out.land_holds_position), (State::Land, false));
        assert_eq!(r.h.anchor(), None);
    }

    /// H13 (1): a 0.5 s tier dip (Altitude) with the vehicle 2 m away, then HOLD -> TAKEOVER -> HOLD moved and moving:
    /// A and b kept. A take-over and hand-back: PILOT clears it, and the next entry sets it at the new place.
    #[test]
    fn h13_the_anchor_is_kept_through_a_tier_dip_and_hold_takeover_hold_and_re_set_after_a_hand_back() {
        let mut r = hold_from([10.0, 10.0], [0.0, 0.0]);
        let a = Some(([10.0, 10.0], 0.0));
        assert_eq!(r.h.anchor(), a);
        r.i.position_ne_m = [12.0, 10.0];
        r.i.tier = Tier::T2;
        for k in 0..500 {
            let t = r.now() + 1_000;
            let out = r.at(t);
            assert_eq!((out.state, out.demand), (State::Hold, Demand::Altitude), "step {k}");
        }
        r.i.tier = Tier::T3;
        assert_eq!(r.step().demand, Demand::Position);
        assert_eq!(r.h.anchor(), a);
        r.i.position_ne_m = [13.0, 10.0];
        r.i.velocity_ne_ms = [2.0, 0.0];
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover);
        assert_eq!(r.h.anchor(), a);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        assert_eq!(r.h.anchor(), a);
        assert!(!r.h.track_fault());
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Middle).state, State::Pilot, "hand-back");
        assert_eq!(r.h.anchor(), None);
        r.i.position_ne_m = [30.0, 10.0];
        r.i.velocity_ne_ms = [0.0, 0.0];
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        assert_eq!(r.h.anchor(), Some(([30.0, 10.0], 0.0)));
    }

    /// H13 (1), v0 3 m/s (b 1.5 m): exactly X + b does not trip; one ULP over, infinity and not a number do.
    #[test]
    fn h13_exactly_x_plus_b_does_not_trip_one_ulp_over_and_not_a_number_do() {
        let bound = track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms) + 1.5;
        assert_eq!(bound, 6.5);
        let over = f32::from_bits(bound.to_bits() + 1);
        for (n, trips) in [(bound, false), (-bound, false), (over, true), (-over, true), (f32::INFINITY, true), (f32::NAN, true)] {
            let mut r = hold_from([0.0, 0.0], [3.0, 0.0]);
            assert_eq!(r.h.anchor(), Some(([0.0, 0.0], 1.5)));
            r.i.position_ne_m = [n, 0.0];
            let out = r.step();
            assert_eq!(r.h.track_fault(), trips, "{n}");
            assert_eq!(out.state == State::Land, trips, "{n}");
        }
    }

    /// H13 (1): the anchor distance is read on a T2 step (Altitude: no target distance); not on a step with the NE
    /// velocity or position not valid, nor with the airborne latch false.
    #[test]
    fn h13_the_anchor_distance_is_read_on_a_tier_dip_step_and_only_with_the_ne_valid() {
        let cases: [(bool, fn(&mut Rig)); 4] = [
            (true, |_| {}),
            (false, |r| r.i.velocity_ne_valid = false),
            (false, |r| r.i.position_ne_valid = false),
            (false, |r| r.i.airborne = false),
        ];
        for (k, (trips, change)) in cases.into_iter().enumerate() {
            let mut r = hold_from([0.0, 0.0], [0.0, 0.0]);
            r.i.tier = Tier::T2;
            r.i.position_ne_m = [6.0, 0.0];
            change(&mut r);
            r.step();
            assert_eq!(r.h.track_fault(), trips, "case {k}");
        }
    }

    /// H13 (1): entry with the NE position or velocity not valid or not finite (the caller's tier then T2; the drop
    /// younger than B5's debounce, so HOLD): no anchor, stale or zero; the first later step of the segment with both
    /// valid and finite sets it, with b from that step's speed.
    #[test]
    fn h13_an_entry_without_a_valid_ne_sets_the_anchor_at_the_first_valid_step() {
        let cases: [fn(&mut Rig); 4] = [
            |r| r.i.position_ne_valid = false,
            |r| r.i.velocity_ne_valid = false,
            |r| r.i.position_ne_m = [f32::NAN, 0.0],
            |r| r.i.velocity_ne_ms = [f32::INFINITY, 0.0],
        ];
        for (k, bad) in cases.into_iter().enumerate() {
            let mut r = Rig::pilot();
            r.i.position_ne_m = [20.0, 0.0];
            r.i.tier = Tier::T2;
            bad(&mut r);
            let t = r.last_report() + LOSS + 1;
            assert_eq!(r.heartbeat(t).state, State::Hold, "case {k}");
            assert_eq!(r.h.anchor(), None, "case {k}");
            for _ in 0..500 {
                let t = r.now() + 1_000;
                r.at(t);
                assert_eq!(r.h.anchor(), None, "case {k}");
            }
            r.i.position_ne_valid = true;
            r.i.velocity_ne_valid = true;
            r.i.position_ne_m = [26.0, 0.0];
            r.i.velocity_ne_ms = [1.2, 0.0];
            r.i.tier = Tier::T3;
            let out = r.step();
            assert_eq!((out.state, out.demand), (State::Hold, Demand::Position), "case {k}");
            assert_eq!(r.h.anchor(), Some(([26.0, 0.0], 1.2 * 1.2 / 6.0)), "case {k}");
            assert!(!r.h.track_fault(), "case {k}");
        }
    }

    /// H13 (2): X for PILOT_SPEED_XY 3, 5 and 20 (capped to 5) is 7, 5 and 5 m; the speed limit is v_ho in every
    /// hands-off state and the speed in force in PILOT and STOP; the target distance and the anchor distance trip one
    /// ULP over that X and not at it.
    #[test]
    fn h13_x_and_the_speed_limit_follow_the_speed_in_force_capped_at_5() {
        for (v, x, v_ho) in [(3.0f32, 7.0f32, 3.0f32), (5.0, 5.0, 5.0), (20.0, 5.0, 5.0)] {
            assert_eq!((track_fault_m(v), speed_limit_xy_ms(v)), (x, v_ho), "{v}");
            let rigs: [(&str, fn() -> Rig, f32); 7] = [
                ("pilot", Rig::pilot, v),
                ("hold", Rig::hold, v_ho),
                ("takeover", Rig::takeover, v_ho),
                ("auto", Rig::auto, v_ho),
                ("return", Rig::ret, v_ho),
                ("land", Rig::land, v_ho),
                ("blind", Rig::blind, v_ho),
            ];
            for (name, rig, want) in rigs {
                let mut r = rig();
                r.i.speed_xy_ms = v;
                assert_eq!(r.step().speed_limit_xy_ms, want, "{name} {v}");
            }
            let mut r = Rig::new();
            r.i.speed_xy_ms = v;
            assert_eq!(r.step().speed_limit_xy_ms, v, "stop {v}");
            for (d, trips) in [(x, false), (f32::from_bits(x.to_bits() + 1), true)] {
                let mut r = Rig::hold();
                r.i.speed_xy_ms = v;
                r.i.track_error_m = Some(d);
                r.step();
                assert_eq!(r.h.track_fault(), trips, "target distance {v} {d}");
                let mut r = hold_from([0.0, 0.0], [0.0, 0.0]);
                r.i.speed_xy_ms = v;
                r.i.position_ne_m = [d, 0.0];
                r.step();
                assert_eq!(r.h.track_fault(), trips, "anchor distance {v} {d}");
            }
        }
    }
}
