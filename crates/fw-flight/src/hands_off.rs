//! The hands-off state machine. Pure: no clock, no log, no statics, no allocation; every time is an argument.

use crate::tuning::Tuning;
use fw_core::rc::{RcSample, RcSource, Switch3};

/// Link loss: link age above this in HOLD gives RETURN (home and T3) or LAND, us.
pub const LINK_LOSS_US: u64 = 10_000_000;

/// Station lost: no station HEARTBEAT on the radio link for longer than this pauses AUTO into HOLD with the place
/// kept, us; Start needs one within it. 10 s.
pub const STATION_LOST_US: u64 = 10_000_000;

/// The largest horizontal distance from home of the vehicle and of every waypoint at Start, m; more is DENIED.
/// Decision section 9: "within 1000 m horizontally of home"; owner, section 8 Q9 "1km".
pub const HOME_RANGE_M: f32 = 1000.0;

/// A tier drop (position, heading or height) takes effect on state exits and the stick-loss entry only after it has
/// lasted this long without a break, us; an upgrade takes effect at once.
pub const TIER_DEBOUNCE_US: u64 = 1_000_000;

/// D_max, m: the largest horizontal excursion at which LAND must have started (Q-H1; H12 item 4)
pub const D_MAX_M: f32 = 10.0;

/// The cap on the horizontal speed limit of every hands-off state, m/s: the owner-approved default of `PILOT_SPEED_XY`
/// (`Tuning::pilot_speed_xy_ms`;)
pub const SPEED_XY_CAP_MS: f32 = Tuning::DEFAULT.pilot_speed_xy_ms;

/// The horizontal speed limit of the hands-off states for `PILOT_SPEED_XY` in force `in_force_ms`, m/s: v_ho =
/// min(`in_force_ms`, `SPEED_XY_CAP_MS`)
pub fn speed_limit_xy_ms(in_force_ms: f32) -> f32 {
    in_force_ms.min(SPEED_XY_CAP_MS)
}

/// The hold-integrity bound X for `PILOT_SPEED_XY` in force `in_force_ms`, m: a horizontal distance between the
/// navigation position and the target the position controller tracks above this sets the track fault.
pub fn track_fault_m(in_force_ms: f32) -> f32 {
    D_MAX_M - speed_limit_xy_ms(in_force_ms) * (TIER_DEBOUNCE_US as f32 / 1e6)
}

/// The effective mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// A fresh CRSF report and the pilot latch set: today's modes.
    Pilot,
    /// A fresh CRSF report in a hands-off state: the ladder at High with centred synthetic sticks until A2' hands
    /// over.
    Takeover,
    /// Hands-off Position hold, centred sticks.
    Hold,
    /// Flying the stored list.
    Auto,
    /// Climb to the latched return height, then a straight leg home.
    Return,
    /// Descend to touchdown; the existing landing latch disarms.
    Land,
    /// Disarmed, motors stopped (or sticks lost on the ground: the gate disarms)
    Stop,
    /// LAND flown on the debounced T1 (tilt and rates only;)
    Blind,
}

impl State {
    /// The HEARTBEAT `custom_mode` code: link-contract section 10; BLIND 8 (lead's ruling, contract later)
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

/// The validity tier, evaluated by the caller every step from the flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tier {
    /// The position is covered (`Setpoint::Position`'s needs; includes the height)
    T3,
    /// The height is covered, the position is not.
    T2,
    /// Tilt and rates only.
    T1,
}

/// A station flight command, its parameters already checked by the caller.
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
    /// MAV_CMD_NAV_RETURN_TO_LAUNCH.
    Return,
}

/// The link a command arrived on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// The radio link (the transmitter's backpack)
    Radio,
    /// USB.
    Usb,
}

/// The highest setpoint level this step's flags cover: the flight code must never demand more.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Demand {
    /// T3': the position is covered and `heading_ok`.
    Position,
    /// The height is covered.
    Altitude,
    /// Tilt and rates only: Angle with centred synthetic throttle when hands-off.
    Angle,
}

/// LAND's horizontal law, chosen every step from this step's facts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LandLaw {
    /// No track fault and this step's T3' (so also the debounced one): hold the horizontal position.
    Hold,
    /// The track fault and this step's T3': `Setpoint::Position` at this step's position with no velocity feed-
    /// forward, on every step, never captured.
    Brake,
    /// Otherwise: level (Altitude, or Angle when the height is not covered)
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

/// What the caller knows of the stored mission (core 0's committed set as core 1 has it)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MissionFacts {
    /// The plan id of the stored list; `None` when nothing is stored.
    pub plan_id: Option<u32>,
    /// The stored list has at least one waypoint (count 2 or more)
    pub has_waypoints: bool,
    /// Every stored z (waypoints and return) is inside the height limits in force now.
    pub z_within_limits: bool,
    /// The largest horizontal distance of any stored waypoint from home, m; `None` when unknown.
    pub max_waypoint_distance_m: Option<f32>,
    /// The stored return height above home, m; `None` when no list is stored.
    pub return_height_m: Option<f32>,
    /// `GDN_ALT_MAX` in force when the list was pushed, m: RETURN's height is at most this; a non-finite
    /// or non-positive value is the default `ALT_MAX_DEFAULT`, as the mission service's limits default.
    pub alt_max_m: f32,
}

/// The facts of one flight step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inputs {
    /// Board time, us.
    pub now_us: u64,
    /// The newest stick report of the flight's source.
    /// accepts), not necessarily new this step; `None` before the first.
    pub crsf: Option<RcSample>,
    /// When the newest station HEARTBEAT was received on the radio link, us.
    pub last_radio_heartbeat_us: Option<u64>,
    /// The gate's armed state.
    pub armed: bool,
    /// The airborne latch.
    pub airborne: bool,
    /// The low mark: the collective has reached `EVER_AIRBORNE_G` x m g at an armed step since arming.
    pub lifted: bool,
    /// The validity tier.
    pub tier: Tier,
    /// Steering north and east from the origin, m, and whether they may be used (`position_ne`)
    pub position_ne_m: [f32; 2],
    /// `position_ne`.
    pub position_ne_valid: bool,
    /// Velocity north and east, m/s (the estimate's), and whether they may be used (`velocity_ne`)
    pub velocity_ne_ms: [f32; 2],
    /// `velocity_ne`.
    pub velocity_ne_valid: bool,
    /// Steering down from the origin, m (T3 implies it is valid)
    pub position_d_m: f32,
    /// `position_d`.
    pub position_d_valid: bool,
    /// A station command to answer this step.
    pub command: Option<StationCommand>,
    /// The stored mission.
    pub mission: MissionFacts,
    /// The guidance reached the last waypoint (AUTO)
    pub last_waypoint_reached: bool,
    /// The guidance is within GDN_RADIUS of home (RETURN arrival)
    pub home_reached: bool,
    /// The heading is OK: the estimate's `Valid.heading`.
    pub heading_ok: bool,
    /// The battery is known not to be critical.
    pub battery_ok: bool,
    /// The battery is critical.
    pub battery_critical: bool,
    /// The battery's LOW level (or CRITICAL) is reached.
    pub battery_low: bool,
    /// The horizontal distance between the navigation position and the Position target the controller tracked at the
    /// previous step, both of that step, m; `None` when the previous step tracked no Position target.
    pub track_error_m: Option<f32>,
    /// `PILOT_SPEED_XY` in force (latched at arming), m/s: gives v_ho and X.
    pub speed_xy_ms: f32,
    /// The touchdown rule TD had held at the previous step: the three conditions together for 1.0 s, counted on every
    /// armed step in every state.
    pub td_held: bool,
    /// The gate armed this arming through the in-air restart: home never latches.
    pub restart: bool,
}

/// Why a command was DENIED: the first condition that failed, in this order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DenyReason {
    /// The command did not arrive on the radio link.
    NotRadioLink,
    /// Armed on a CRSF report: not a MAVLink flight.
    HandsOffDisabled,
    /// The state does not take this command.
    NotInThisState,
    /// A command into HOLD or RETURN (and Abort in BLIND): the debounced tier is not T3', or the track fault is
    /// latched.
    NotT3,
    /// A command into HOLD or RETURN: the battery is critical, so the state would land at once.
    BatteryCritical,
    /// A command into HOLD: the link is lost (`LINK_LOSS_US`), so HOLD would leave at once.
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
    /// Start: the heading monitor is not OK, or the track fault is latched.
    HeadingNotOk,
    /// Start: the battery is not known to be non-critical, or its LOW level is reached.
    BatteryNotOk,
    /// Start: no live report with arm ON; a command into HOLD: the newest report has arm OFF.
    ArmSwitchOff,
    /// Start: no live report with switch6 High.
    SwitchNotHigh,
    /// Start on a MAVLink flight: no receiver report younger than 1.0 s with the mission switch ON.
    MissionSwitchOff,
}

impl DenyReason {
    /// The reason code for COMMAND_ACK.result_param2; 0 is reserved for unknown, 2 and 20 are retired (never reused).
    pub fn code(self) -> u8 {
        match self {
            DenyReason::NotRadioLink => 1,
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
            DenyReason::MissionSwitchOff => 19,
        }
    }
}

/// The answer to a command (COMMAND_ACK)
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
    /// AUTO left with the place kept (Start resumes it)
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

/// Why the present state was entered: one per rule of the state table that changes the state.
/// MAVLink-flight table and of the other has the same reason)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// An arm-OFF report with the mission switch OFF: STOP.
    ArmMsOff,
    /// An arm-OFF report in PILOT on the ground: STOP.
    ArmOffGround,
    /// An arm-OFF report: STOP or, on a MAVLink flight in the air, LAND.
    ArmOff,
    /// The gate is disarmed: STOP.
    Disarmed,
    /// A hands-off state on the ground with TD held: STOP.
    Landed,
    /// A fresh report in STOP with the pilot latch set (arming): PILOT.
    Armed,
    /// Sticks lost in PILOT or TAKEOVER (`lost`)
    RcLost,
    /// A fresh report in a hands-off state (not a MAVLink flight): TAKEOVER.
    RcBack,
    /// TAKEOVER's hand-over (the pilot latch): PILOT.
    Handover,
    /// A live report with the mission switch OFF: PILOT (switch6 High) or TAKEOVER.
    MsOff,
    /// The mission switch ON with switch6 High in PILOT in the air: HOLD.
    MsOn,
    /// The debounced tier: a drop below T3' (LAND or BLIND), or BLIND's tier change (LAND, BLIND)
    Tier,
    /// The battery is critical: LAND or BLIND.
    BattCrit,
    /// HOLD or AUTO: the battery's LOW level, once per arming: RETURN or LAND.
    BattLow,
    /// The H12 track fault: LAND or BLIND.
    Track,
    /// Link age above `LINK_LOSS_US` in HOLD: RETURN or LAND.
    LinkLost,
    /// AUTO: the plan id is not the one flown: HOLD.
    Plan,
    /// AUTO: the last waypoint reached: HOLD.
    LastWp,
    /// AUTO: no station HEARTBEAT within `STATION_LOST_US`: HOLD with the place kept.
    StationLost,
    /// RETURN: home reached: LAND or BLIND.
    Home,
    /// Command Start: AUTO.
    Start,
    /// Command Pause: HOLD.
    Pause,
    /// Command Abort: HOLD.
    Abort,
    /// Command Return: RETURN.
    Return,
    /// Command Land: LAND or BLIND.
    Land,
}

impl Reason {
    /// The short token the HO MSG appends to "HO <code> <NAME>".
    pub fn token(self) -> &'static str {
        match self {
            Reason::ArmMsOff => "arm-ms-off",
            Reason::ArmOffGround => "arm-gnd",
            Reason::ArmOff => "arm-off",
            Reason::Disarmed => "disarmed",
            Reason::Landed => "landed",
            Reason::Armed => "armed",
            Reason::RcLost => "rc-lost",
            Reason::RcBack => "rc-back",
            Reason::Handover => "handover",
            Reason::MsOff => "ms-off",
            Reason::MsOn => "ms-on",
            Reason::Tier => "tier",
            Reason::BattCrit => "batt-crit",
            Reason::BattLow => "batt-low",
            Reason::Track => "track",
            Reason::LinkLost => "link10s",
            Reason::Plan => "plan",
            Reason::LastWp => "last-wp",
            Reason::StationLost => "stn-lost",
            Reason::Home => "home",
            Reason::Start => "start",
            Reason::Pause => "pause",
            Reason::Abort => "abort",
            Reason::Return => "rtl",
            Reason::Land => "cmd-land",
        }
    }
}

/// Home: set when both its height and its position were latched this arming.
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
    /// LAND: selects the LAND law only; it triggers no disarm (WR1: the ground exit is the state machine's)
    pub landing: bool,
    /// LAND flying [`LandLaw::Hold`].
    pub land_holds_position: bool,
    /// LAND and TAKEOVER's descent: the horizontal law; `None` otherwise.
    pub land_law: Option<LandLaw>,
    /// The highest level the flight code may demand this step, from this step's flags.
    pub demand: Demand,
    /// TAKEOVER after the battery was critical in it, or with the track fault: fly the LAND law with the synthetic
    /// sticks and `land_law`; hand-over stays possible.
    pub takeover_descent: bool,
    /// For the gate: a report with arm OFF must not disarm.
    pub arm_off_holds: bool,
    /// For the gate: disarm in this step. STOP entered this step with `armed` true.
    pub disarm: bool,
    /// The horizontal speed limit for the position controller, m/s: v_ho ([`speed_limit_xy_ms`]) in a hands-off state,
    /// `PILOT_SPEED_XY` in force otherwise.
    pub speed_limit_xy_ms: f32,
    /// Why the present state was entered; `None` before the first change (the initial STOP)
    pub reason: Option<Reason>,
    /// STOP entered this step from armed by the arm switch (`disarm` with `ArmMsOff`, `ArmOff` or `ArmOffGround`): the
    /// ground predicate of that step (`ground`); `None` otherwise.
    pub arm_stop_ground: Option<bool>,
}

/// The hands-off state machine.
#[derive(Clone, Debug)]
pub struct HandsOff {
    state: State,
    /// Stick loss, us: the gate's RC_FS_TIMEOUT (`Tuning::rc_fs_timeout_us`, decision Numbers "hands-off entry 1.0 s")
    stick_loss_us: u64,
    /// Set when the pilot flies; cleared on entry to any hands-off state.
    pilot_latch: bool,
    /// P, "lock open": a counting report had arm ON and switch6 High.
    lock_open: bool,
    /// The battery was critical in this TAKEOVER; cleared on any state change.
    descent: bool,
    /// A mission place exists (AUTO, or HOLD after a pause that kept it)
    place: bool,
    /// The plan id at Start.
    flown_plan: Option<u32>,
    return_height_m: Option<f32>,
    was_armed: bool,
    home_d: Option<f32>,
    home_ne: Option<[f32; 2]>,
    /// This arming may still latch home (armed, before take-off;)
    home_window: bool,
    /// The flight's source: the newest report's at the first armed step, which is the gate's arming report's source;
    /// `None` on disarmed steps.
    source: Option<RcSource>,
    /// "Ever airborne": the low mark (`Inputs::lifted`) or the airborne latch has been set at an armed step since
    /// arming; cleared on a disarmed step.
    ever_airborne: bool,
    /// The tier for exits and the stick-loss entry.
    debounced: Tier,
    /// The first step of the present run of steps below T3'.
    below_t3_since: Option<u64>,
    /// The first step of the present run of T1 steps.
    below_t2_since: Option<u64>,
    /// Track_fault, a landing cause of its own; set by `track_error_m` or by the distance from the anchor,
    /// cleared on a disarmed step only.
    track_fault: bool,
    /// The previous step's state is one the check covers: HOLD, AUTO, RETURN, TAKEOVER, or LAND holding position.
    track_covered: bool,
    /// The anchor A (NE, m) and the braking allowance b (m) of the present fixed-target segment; `None` outside one
    /// and until a step of it has a valid, finite NE position and velocity.
    anchor: Option<([f32; 2], f32)>,
    /// A fixed-target segment has begun and its anchor is not set yet.
    anchor_pending: bool,
    /// Why the present state was entered (`None` before the first change)
    reason: Option<Reason>,
    /// The LOW battery return is used for this arming: it fired, or a step began in RETURN, LAND or BLIND with LOW;
    /// cleared on a disarmed step.
    low_done: bool,
}

impl HandsOff {
    /// STOP, disarmed, no home. The stick-loss time is the gate's (`tuning.rc_fs_timeout_us`)
    pub fn new(tuning: &Tuning) -> Self {
        Self {
            state: State::Stop,
            stick_loss_us: tuning.rc_fs_timeout_us,
            pilot_latch: true,
            lock_open: false,
            descent: false,
            place: false,
            flown_plan: None,
            return_height_m: None,
            was_armed: false,
            home_d: None,
            home_ne: None,
            home_window: false,
            source: None,
            ever_airborne: false,
            debounced: Tier::T3,
            below_t3_since: None,
            below_t2_since: None,
            track_fault: false,
            track_covered: false,
            anchor: None,
            anchor_pending: false,
            reason: None,
            low_done: false,
        }
    }

    /// Why the present state was entered; `None` before the first change.
    pub fn reason(&self) -> Option<Reason> {
        self.reason
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

    /// The H12 track fault is latched (until a disarmed step)
    pub fn track_fault(&self) -> bool {
        self.track_fault
    }

    /// The anchor A (NE, m) and the braking allowance b (m) in force, when set.
    pub fn anchor(&self) -> Option<([f32; 2], f32)> {
        self.anchor
    }

    /// "Ever airborne" in force: the latch set at an armed step since arming, as of the latest step.
    pub fn ever_airborne(&self) -> bool {
        self.ever_airborne
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
            self.source = None;
        } else if self.source.is_none() {
            self.source = i.crsf.map(|r| r.source);
        }
        self.ever_airborne = i.armed && (self.ever_airborne || i.airborne || i.lifted);
        // The previous step's distance against X, in the state that step flew; latched until disarmed.
        let x = track_fault_m(i.speed_xy_ms);
        if !i.armed {
            self.track_fault = false;
        } else if self.track_covered && i.airborne && i.track_error_m.is_some_and(|d| d.is_nan() || d > x) {
            self.track_fault = true;
        }
        // This step's distance from the anchor against X + b on every step with a valid NE velocity (and position);
        // not a number trips.
        let instant = self.instant_tier(i);
        let anchor_read = match self.state {
            State::Hold | State::Takeover => true,
            State::Land => !self.track_fault && instant == Tier::T3,
            _ => false,
        };
        if let Some(([an, ae], b)) = self.anchor
            && anchor_read
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
        self.update_tier(i.now_us, instant);
        self.automatic(i, fresh, &mut ev);
        let command = i.command.map(|c| self.command(c, i, &mut ev));
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
        self.update_anchor(was, land_holds_position, i);
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
            arm_off_holds: self.mavlink_flight() && hands_off && i.crsf.is_some_and(|r| !r.arm && r.mission),
            disarm: self.state == State::Stop && was != State::Stop && i.armed,
            speed_limit_xy_ms: if hands_off {
                speed_limit_xy_ms(i.speed_xy_ms)
            } else {
                i.speed_xy_ms
            },
            reason: self.reason,
            arm_stop_ground: (self.state == State::Stop
                && was != State::Stop
                && i.armed
                && matches!(self.reason, Some(Reason::ArmMsOff | Reason::ArmOff | Reason::ArmOffGround)))
            .then(|| self.ground(i)),
        }
    }

    /// A fixed-target segment begins on entry to HOLD, TAKEOVER or LAND holding position (`land_hold`) from PILOT,
    /// AUTO or RETURN; its anchor is set at its first step with a valid, finite NE position and velocity.
    fn update_anchor(&mut self, was: State, land_hold: bool, i: &Inputs) {
        let entry = was != self.state
            && matches!(was, State::Pilot | State::Auto | State::Return)
            && (matches!(self.state, State::Hold | State::Takeover) || land_hold);
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

    /// Heading_ok = the input only (the track fault is a landing cause of its own)
    fn heading_ok(&self, i: &Inputs) -> bool {
        i.heading_ok
    }

    /// This step's tier with T3' (T3 needs `heading_ok`)
    fn instant_tier(&self, i: &Inputs) -> Tier {
        match i.tier {
            Tier::T3 if self.heading_ok(i) => Tier::T3,
            Tier::T3 | Tier::T2 => Tier::T2,
            Tier::T1 => Tier::T1,
        }
    }

    /// A drop below a level counts once it has lasted `TIER_DEBOUNCE_US` without a break; a rise at once.
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

    /// LAND, or BLIND when the debounced tier is T1.
    fn land_state(&self) -> State {
        if self.debounced == Tier::T1 {
            State::Blind
        } else {
            State::Land
        }
    }

    /// D and NE each at the first armed step where they are valid while not airborne; the window closes on airborne;
    /// home is both or nothing; cleared at disarm.
    fn update_home(&mut self, i: &Inputs) {
        if !i.armed {
            self.home_d = None;
            self.home_ne = None;
            self.home_window = false;
            self.low_done = false;
        } else {
            if !self.was_armed {
                self.home_window = !i.restart;
            }
            if self.home_window {
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

    /// On the ground: neither the airborne latch nor (since arming) ever airborne, or TD held.
    fn ground(&self, i: &Inputs) -> bool {
        !i.airborne && (!self.ever_airborne || i.td_held)
    }

    /// Not a CRSF flight: a MAVLink flight, or disarmed.
    fn ho_allowed(&self) -> bool {
        self.source != Some(RcSource::Crsf)
    }

    /// Armed on a receiver report.
    fn mavlink_flight(&self) -> bool {
        self.source == Some(RcSource::Receiver)
    }

    /// Section 2, the transition table of a MAVLink flight, in its priority order: the arm columns 4 and 5.
    /// Newest report, stale or not), columns 2 and 3, each state's own exits (column 6 among them)
    fn mission_switch(&mut self, i: &Inputs, fresh: bool, ev: &mut Events) {
        if i.battery_low && matches!(self.state, State::Return | State::Land | State::Blind) {
            self.low_done = true;
        }
        if let Some(r) = i.crsf.filter(|r| !r.arm) {
            let ground = self.ground(i);
            match self.state {
                _ if !r.mission => {
                    self.go(State::Stop, Reason::ArmMsOff, i, ev);
                    return;
                }
                State::Stop => return,
                State::Pilot if ground => {
                    self.go(State::Stop, Reason::ArmOffGround, i, ev);
                    return;
                }
                State::Land | State::Blind => {}
                _ => self.go(self.land_state(), Reason::ArmOff, i, ev),
            }
        }
        if self.state.is_hands_off() && !i.airborne && i.td_held {
            self.go(State::Stop, Reason::Landed, i, ev);
            return;
        }
        let live = i.crsf.filter(|r| fresh && r.arm);
        if let Some(r) = live.filter(|r| !r.mission)
            && self.state != State::Stop
        {
            if r.switch6 == Switch3::High {
                self.go(State::Pilot, Reason::MsOff, i, ev);
            } else if self.state != State::Pilot {
                self.go(State::Takeover, Reason::MsOff, i, ev);
            }
        } else {
            match self.state {
                State::Stop => {
                    if fresh && self.pilot_latch {
                        self.go(State::Pilot, Reason::Armed, i, ev);
                    }
                }
                State::Pilot | State::Takeover => {
                    if !fresh {
                        self.go(self.lost(i), Reason::RcLost, i, ev);
                    }
                }
                State::Hold => {
                    if self.debounced != Tier::T3 || i.battery_critical || self.track_fault {
                        self.go(self.land_state(), self.land_reason(i), i, ev);
                    } else if i.battery_low && !self.low_done {
                        self.low_done = true;
                        self.go(if self.home().is_some() { State::Return } else { State::Land }, Reason::BattLow, i, ev);
                    } else if self.link_age(i) > LINK_LOSS_US {
                        let next = if self.home().is_some() {
                            State::Return
                        } else {
                            State::Land
                        };
                        self.go(next, Reason::LinkLost, i, ev);
                    } else if self.place && i.mission.plan_id != self.flown_plan {
                        self.drop_place(ev);
                    }
                }
                State::Auto => {
                    if self.debounced != Tier::T3 || i.battery_critical || self.track_fault {
                        self.go(self.land_state(), self.land_reason(i), i, ev);
                    } else if i.battery_low && !self.low_done {
                        self.low_done = true;
                        self.go(if self.home().is_some() { State::Return } else { State::Land }, Reason::BattLow, i, ev);
                    } else if i.mission.plan_id != self.flown_plan || i.last_waypoint_reached {
                        self.go(State::Hold, if i.last_waypoint_reached { Reason::LastWp } else { Reason::Plan }, i, ev);
                    } else if !Self::station_present(i) {
                        self.pause(Reason::StationLost, i, ev);
                    }
                }
                State::Return => {
                    if self.debounced != Tier::T3 || i.battery_critical || self.track_fault || i.home_reached {
                        self.go(self.land_state(), self.return_reason(i), i, ev);
                    }
                }
                State::Land => {
                    if self.debounced == Tier::T1 {
                        self.go(State::Blind, Reason::Tier, i, ev);
                    }
                }
                State::Blind => {
                    if self.debounced != Tier::T1 {
                        self.go(State::Land, Reason::Tier, i, ev);
                    }
                }
            }
            if self.state == State::Pilot
                && live.is_some_and(|r| r.mission && r.switch6 == Switch3::High)
                && i.airborne
                && self.hold_entry_check(i).is_none()
            {
                self.go(State::Hold, Reason::MsOn, i, ev);
            }
        }
        if self.state == State::Takeover && (i.battery_critical || self.track_fault) {
            self.descent = true;
        }
    }

    /// The state table's exits that are not commands.
    fn automatic(&mut self, i: &Inputs, fresh: bool, ev: &mut Events) {
        if self.mavlink_flight() {
            self.mission_switch(i, fresh, ev);
            return;
        }
        if !i.armed || i.crsf.is_some_and(|r| !r.arm) {
            self.go(State::Stop, if i.armed { Reason::ArmOff } else { Reason::Disarmed }, i, ev);
            return;
        }
        // Guarded by the touchdown rule TD (N10 (i); ArduPilot land_complete)
        if self.state.is_hands_off() && !i.airborne && i.td_held {
            self.go(State::Stop, Reason::Landed, i, ev);
            return;
        }
        match self.state {
            State::Stop => {
                if fresh && self.pilot_latch {
                    self.go(State::Pilot, Reason::Armed, i, ev);
                }
            }
            State::Pilot => {
                if !fresh {
                    self.go(self.lost(i), Reason::RcLost, i, ev);
                }
            }
            State::Takeover => {
                if !fresh {
                    self.go(self.lost(i), Reason::RcLost, i, ev);
                } else {
                    self.lock(i);
                    if self.pilot_latch {
                        self.go(State::Pilot, Reason::Handover, i, ev);
                    }
                }
            }
            State::Land | State::Blind if fresh => {
                self.go(State::Takeover, Reason::RcBack, i, ev);
                self.lock(i);
            }
            State::Land => {
                if self.debounced == Tier::T1 {
                    self.go(State::Blind, Reason::Tier, i, ev);
                }
            }
            State::Blind => {
                if self.debounced != Tier::T1 {
                    self.go(State::Land, Reason::Tier, i, ev);
                }
            }
            // Not reachable on a CRSF flight.
            State::Hold | State::Auto | State::Return => {}
        }
        if self.state == State::Takeover && (i.battery_critical || self.track_fault) {
            self.descent = true;
        }
    }

    /// This step's counting report sets P; after P, arm ON and switch6 not High sets the pilot latch.
    fn lock(&mut self, i: &Inputs) {
        let Some(r) = i.crsf.filter(|r| r.arm) else {
            return;
        };
        let high = r.switch6 == Switch3::High;
        if self.lock_open && !high {
            self.pilot_latch = true;
        }
        self.lock_open |= high;
    }

    /// Sticks lost in PILOT or TAKEOVER: with the latch false, STOP if never airborne since arming.
    /// or the latch) or TD has held, else LAND (or BLIND); LAND (or BLIND) with the track fault.
    fn lost(&self, i: &Inputs) -> State {
        if !i.airborne {
            if !self.ever_airborne || i.td_held {
                State::Stop
            } else {
                self.land_state()
            }
        } else if self.track_fault {
            self.land_state()
        } else {
            match self.debounced {
                Tier::T3 if !self.ho_allowed() => State::Land,
                Tier::T3 => State::Hold,
                Tier::T2 => State::Land,
                Tier::T1 => State::Blind,
            }
        }
    }

    /// Now minus the newer of the last CRSF report and the last radio HEARTBEAT.
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

    /// The reason of the landing exits of HOLD and AUTO: the first of their three conditions that holds.
    fn land_reason(&self, i: &Inputs) -> Reason {
        if self.debounced != Tier::T3 {
            Reason::Tier
        } else if i.battery_critical {
            Reason::BattCrit
        } else {
            Reason::Track
        }
    }

    /// The reason of RETURN's exit: the landing conditions first, then home reached.
    fn return_reason(&self, i: &Inputs) -> Reason {
        if self.debounced != Tier::T3 || i.battery_critical || self.track_fault {
            self.land_reason(i)
        } else {
            Reason::Home
        }
    }

    fn drop_place(&mut self, ev: &mut Events) {
        self.place = false;
        ev.place_dropped = true;
    }

    /// AUTO to HOLD with the place kept.
    fn pause(&mut self, reason: Reason, i: &Inputs, ev: &mut Events) {
        self.go(State::Hold, reason, i, ev);
        self.place = true;
        ev.place_dropped = false;
        ev.place_kept = true;
    }

    /// Enters `next` (no effect when it is the present state)
    fn go(&mut self, next: State, reason: Reason, i: &Inputs, ev: &mut Events) {
        if next == self.state {
            return;
        }
        if self.place && next != State::Auto {
            self.drop_place(ev);
        }
        if next.is_hands_off() {
            self.pilot_latch = false;
            self.lock_open = false;
        }
        self.descent = false;
        self.return_height_m = None;
        if next == State::Return {
            let present = self.home_d.map_or(0.0, |d| d - i.position_d_m);
            // H_r = max(present, min(return height, GDN_ALT_MAX in force)); a non-finite return height is
            // none.
            let alt_max = i.mission.alt_max_m;
            let cap = if alt_max.is_finite() && alt_max > 0.0 { alt_max } else { fw_core::mission::ALT_MAX_DEFAULT };
            self.return_height_m = Some(
                i.mission
                    .return_height_m
                    .filter(|h| h.is_finite())
                    .map_or(present, |h| h.min(cap).max(present)),
            );
        }
        if next == State::Stop {
            // Disarmed: only a CRSF report arms, so the next arming is the pilot's.
            self.flown_plan = None;
            self.pilot_latch = true;
        }
        self.state = next;
        self.reason = Some(reason);
    }

    /// And the lead's rulings: the answer, and the change at this step when ACCEPTED.
    fn command(&mut self, c: StationCommand, i: &Inputs, ev: &mut Events) -> CommandResult {
        use CommandResult::{Accepted, Denied};
        if c.link != Link::Radio {
            return Denied(DenyReason::NotRadioLink);
        }
        if !self.ho_allowed() {
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
                    self.go(State::Auto, Reason::Start, i, ev);
                    self.place = true;
                    self.flown_plan = i.mission.plan_id;
                    Accepted
                }
            },
            (Command::Pause, State::Auto) => {
                self.pause(Reason::Pause, i, ev);
                Accepted
            }
            (Command::Pause, State::Return) | (Command::Abort, State::Auto | State::Return | State::Land) => {
                self.go(State::Hold, if c.command == Command::Pause { Reason::Pause } else { Reason::Abort }, i, ev);
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
                    self.go(State::Return, Reason::Return, i, ev);
                    Accepted
                }
            },
            (Command::Land, State::Hold | State::Auto | State::Return) => {
                self.go(self.land_state(), Reason::Land, i, ev);
                Accepted
            }
            _ => Denied(DenyReason::NotInThisState),
        }
    }

    /// A command into HOLD is DENIED when one of HOLD's automatic exits would fire on this step's facts; the track
    /// fault answers NotT3.
    fn hold_entry_check(&self, i: &Inputs) -> Option<DenyReason> {
        if self.debounced != Tier::T3 || self.track_fault {
            Some(DenyReason::NotT3)
        } else if i.battery_critical {
            Some(DenyReason::BatteryCritical)
        } else if self.link_age(i) > LINK_LOSS_US {
            Some(DenyReason::LinkLost)
        } else if self.mavlink_flight() && i.crsf.is_some_and(|r| !r.arm) {
            Some(DenyReason::ArmSwitchOff)
        } else {
            None
        }
    }

    /// Return needs home; then the no-bounce rule for RETURN's automatic exits (arrival excepted);
    /// The track fault answers NotT3.
    fn return_check(&self, i: &Inputs) -> Option<DenyReason> {
        if self.home().is_none() {
            Some(DenyReason::NoHome)
        } else if self.debounced != Tier::T3 || self.track_fault {
            Some(DenyReason::NotT3)
        } else if i.battery_critical {
            Some(DenyReason::BatteryCritical)
        } else {
            None
        }
    }

    /// Start's conditions in the contract's order, the lead's height ruling after the list's; the first that fails.
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
        if !i.battery_ok || i.battery_critical || i.battery_low {
            return Some(DenyReason::BatteryNotOk);
        }
        let live = i.crsf.filter(|r| i.now_us.saturating_sub(r.t_us) <= self.stick_loss_us);
        if !live.is_some_and(|r| r.mission) {
            Some(DenyReason::MissionSwitchOff)
        } else if !live.is_some_and(|r| r.arm) {
            Some(DenyReason::ArmSwitchOff)
        } else if !live.is_some_and(|r| r.switch6 == Switch3::High) {
            Some(DenyReason::SwitchNotHigh)
        } else {
            None
        }
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
            alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
        }
    }

    struct Rig {
        h: HandsOff,
        i: Inputs,
    }

    impl Rig {
        /// Disarmed on the ground at home, a fresh receiver report (High, arm on, mission switch OFF), the station
        /// present: the rigs below fly MAVLink flights.
        fn new() -> Rig {
            let i = Inputs {
                now_us: T0,
                crsf: Some(RcSample {
                    sticks: CENTRED,
                    arm: true,
                    switch6: Switch3::High,
                    source: RcSource::Receiver,
                    mission: false,
                    t_us: T0,
                }),
                last_radio_heartbeat_us: Some(T0),
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
                battery_low: false,
                track_error_m: None,
                speed_xy_ms: Tuning::DEFAULT.pilot_speed_xy_ms,
                td_held: true,
                lifted: false,
                restart: false,
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

        /// A new report at `t` with these sticks and switches, the mission switch OFF, from the source of the previous
        /// one.
        fn report_with(&mut self, t: u64, sticks: Sticks, switch6: Switch3, arm: bool) -> Output {
            self.report_full(t, sticks, switch6, arm, false)
        }

        /// A new report at `t`, from the source of the previous one.
        fn report_full(&mut self, t: u64, sticks: Sticks, switch6: Switch3, arm: bool, mission: bool) -> Output {
            let source = self.i.crsf.map_or(RcSource::Receiver, |r| r.source);
            self.i.crsf = Some(RcSample {
                sticks,
                arm,
                switch6,
                source,
                mission,
                t_us: t,
            });
            self.at(t)
        }

        /// A new report at `t` with full sticks and the mission switch ON.
        fn report_ms(&mut self, t: u64, switch6: Switch3, arm: bool) -> Output {
            self.report_full(t, FULL, switch6, arm, true)
        }

        /// As `new`, the report from CRSF: the rigs built on it fly CRSF flights.
        fn crsf() -> Rig {
            let mut r = Rig::new();
            r.i.crsf = r.i.crsf.map(|s| RcSample { source: RcSource::Crsf, ..s });
            r
        }

        /// `pilot` on a CRSF flight.
        fn crsf_pilot() -> Rig {
            let mut r = Rig::crsf();
            assert_eq!(r.step().state, State::Stop);
            r.i.armed = true;
            assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
            r.i.airborne = true;
            r.i.position_ne_m = [10.0, 10.0];
            r.i.position_d_m = HOME_D - 10.0;
            assert_eq!(r.report(T0 + 2_000, Switch3::High).state, State::Pilot);
            r
        }

        /// Sticks lost on T3 in the air on a CRSF flight: LAND, with the station's heartbeat.
        fn crsf_land() -> Rig {
            let mut r = Rig::crsf_pilot();
            let t = r.last_report() + LOSS + 1;
            assert_eq!(r.heartbeat(t).state, State::Land);
            r
        }

        /// Sticks lost in the air after T1 has lasted 1.0 s, on a CRSF flight: BLIND.
        fn crsf_blind() -> Rig {
            let mut r = Rig::crsf_pilot();
            assert_eq!(r.drop_tier(Tier::T1).state, State::Pilot);
            let t = r.last_report() + LOSS + 1;
            assert_eq!(r.at(t).state, State::Blind);
            r
        }

        /// `crsf_land`, then a fresh CRSF report with switch6 Low: TAKEOVER.
        /// MAVLink-flight rule)
        fn crsf_takeover() -> Rig {
            let mut r = Rig::crsf_land();
            let t = r.now() + 1_000;
            assert_eq!(r.report(t, Switch3::Low).state, State::Takeover);
            r
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

        /// HOLD, then the station's heartbeat and a report with the mission switch ON, arm ON and High: HOLD stays
        /// (N19 column 1) and Start may be given.
        fn ms_hold() -> Rig {
            let mut r = Rig::hold();
            let t = r.now() + 1_000;
            r.i.last_radio_heartbeat_us = Some(t);
            assert_eq!(r.report_ms(t, Switch3::High, true).state, State::Hold);
            r
        }

        /// `ms_hold`, then Start.
        fn auto() -> Rig {
            let mut r = Rig::ms_hold();
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

        /// This step's tier becomes `tier` (the first step of the drop)
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

    // ---- (as amended by) ----

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
        let mut r = Rig::crsf_takeover();
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
        let mut r = Rig::crsf_land();
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
        let mut r = Rig::crsf_takeover();
        let t = r.now() + 20_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.at(t).state, State::Land);
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover);
    }

    // ---- HOLD ----

    #[test]
    fn hands_off_hold_entries_from_auto() {
        let mut r = Rig::auto();
        let out = r.cmd(Command::Pause);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        assert!(out.events.place_kept && !out.events.place_dropped);
        let mut r = Rig::auto();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        assert!(out.events.place_dropped && !out.events.place_kept);
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
        assert_eq!(Rig::land().h.state(), State::Land);
        for (tier, want) in [(Tier::T2, State::Land), (Tier::T1, State::Blind)] {
            let mut r = Rig::hold();
            assert_eq!(r.drop_tier(tier).state, State::Hold);
            assert_eq!(r.hold_drop().state, want, "{tier:?}");
            r.i.tier = Tier::T3;
            assert_eq!(r.step().state, State::Land, "latched");
        }
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        assert_eq!(r.step().state, State::Land);
    }

    // ---- I4 ----

    #[test]
    fn hands_off_i4_link_age_runs_from_the_newer_of_crsf_and_the_radio_heartbeat() {
        let mut r = Rig::pilot();
        r.i.last_radio_heartbeat_us = None;
        let tc = r.last_report();
        assert_eq!(r.at(tc + LOSS + 1).state, State::Hold);
        assert_eq!(r.at(tc + 9_900_000).state, State::Hold);
        assert_eq!(r.at(tc + 10_100_000).state, State::Return);
        let mut r = Rig::hold();
        let t0 = r.now();
        for k in 1..=60 {
            assert_eq!(r.heartbeat(t0 + k * S).state, State::Hold);
        }
    }

    #[test]
    fn hands_off_i4_a_usb_heartbeat_cannot_prevent_it() {
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
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Return);
    }

    // ---- AUTO ----

    #[test]
    fn hands_off_i5_auto_pauses_at_5_s_without_the_radio_heartbeat_place_kept() {
        let mut r = Rig::auto();
        let hb = r.now();
        assert_eq!(r.at(hb + STATION_LOST_US).state, State::Auto);
        let out = r.at(hb + STATION_LOST_US + 1);
        assert_eq!(out.state, State::Hold);
        assert!(out.events.place_kept && !out.events.place_dropped);
        r.i.last_radio_heartbeat_us = Some(hb + STATION_LOST_US + 1_000);
        r.report_ms(hb + STATION_LOST_US + 1_000, Switch3::High, true);
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
        let mut r = Rig::ret();
        assert_eq!(r.step().return_height_m, Some(20.0));
        r.i.position_d_m = HOME_D - 30.0;
        assert_eq!(r.step().return_height_m, Some(20.0));
        let mut r = Rig::hold();
        r.i.mission.return_height_m = Some(5.0);
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).return_height_m, Some(10.0));
        let mut r = Rig::hold();
        r.i.mission = MissionFacts {
            plan_id: None,
            has_waypoints: false,
            z_within_limits: true,
            max_waypoint_distance_m: None,
            return_height_m: None,
            alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
        };
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).return_height_m, Some(10.0));
    }

    /// H_r = max(present, min(stored return height, GDN_ALT_MAX in force)); a non-finite or non-positive
    /// limit is the default 50 m; no mission: the present height.
    #[test]
    fn fx3_return_height_is_capped_at_gdn_alt_max() {
        let enter = |ret: Option<f32>, alt_max: f32, present: f32| {
            let mut r = Rig::hold();
            r.i.mission.return_height_m = ret;
            r.i.mission.alt_max_m = alt_max;
            r.i.position_d_m = HOME_D - present;
            let t = r.now() + LINK_LOSS_US + 1;
            let out = r.at(t);
            assert_eq!(out.state, State::Return);
            out.return_height_m
        };
        assert_eq!(enter(Some(60.0), 50.0, 10.0), Some(50.0), "stored above the limit: capped");
        assert_eq!(enter(Some(45.0), 40.0, 10.0), Some(40.0), "the limit in force, not the default");
        assert_eq!(enter(Some(60.0), 50.0, 55.0), Some(55.0), "present above both: kept");
        assert_eq!(enter(Some(20.0), 50.0, 10.0), Some(20.0), "within the limit: unchanged");
        for bad in [f32::NAN, 0.0, -5.0, f32::INFINITY] {
            assert_eq!(enter(Some(60.0), bad, 10.0), Some(fw_core::mission::ALT_MAX_DEFAULT), "limit {bad}: the default");
        }
        assert_eq!(enter(None, 50.0, 10.0), Some(10.0), "no mission: present");
        assert_eq!(enter(Some(f32::NAN), 50.0, 10.0), Some(10.0), "a non-finite return height: present");
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

    /// The ground exit needs TD held (`td_held`): a hands-off state that reads the latch false
    /// stays, hands-off and not disarming, while `td_held` is false.
    #[test]
    fn hands_off_wr1_ground_exit_is_guarded_by_td() {
        let rigs: [(&str, fn() -> Rig); 4] = [("hold", Rig::hold), ("land", Rig::land), ("blind", Rig::blind), ("takeover", Rig::takeover)];
        for (name, make) in rigs {
            let mut r = make();
            let want = r.h.state();
            r.i.airborne = false;
            r.i.td_held = false;
            for k in 1..=5u64 {
                let t = r.now() + 1_000;
                let out = r.at(t);
                assert_eq!((out.state, out.hands_off, out.disarm), (want, true, false), "{name} step {k}");
            }
            r.i.td_held = true;
            let t = r.now() + 1_000;
            let out = r.at(t);
            assert_eq!((out.state, out.disarm), (State::Stop, true), "{name}");
        }
    }

    /// Sticks lost in PILOT or TAKEOVER with the latch false after take-off give LAND.
    /// on T1, by the debounced tier) while TD has not held.
    #[test]
    fn hands_off_n10_stick_loss_with_the_latch_false_after_take_off_lands_until_td() {
        for (tier, want) in [(Tier::T3, State::Land), (Tier::T2, State::Land), (Tier::T1, State::Blind)] {
            for takeover in [false, true] {
                let mut r = if takeover { Rig::takeover() } else { Rig::pilot() };
                r.drop_tier(tier);
                r.i.airborne = false;
                r.i.td_held = false;
                let t = r.last_report() + LOSS + 1;
                let out = r.at(t);
                let name = (tier, takeover);
                assert_eq!((out.state, out.hands_off, out.disarm), (want, true, false), "{name:?}");
                let t = r.now() + 1_000;
                assert_eq!(r.at(t).state, want, "{name:?}");
                r.i.td_held = true;
                let t = r.now() + 1_000;
                let out = r.at(t);
                assert_eq!((out.state, out.disarm), (State::Stop, true), "{name:?}: TD held");
                let mut r = if takeover { Rig::takeover() } else { Rig::pilot() };
                r.drop_tier(tier);
                r.i.airborne = false;
                let t = r.last_report() + LOSS + 1;
                assert_eq!(r.at(t).state, State::Stop, "{name:?}: TD held at the loss");
            }
        }
        let mut r = Rig::new();
        r.i.td_held = false;
        r.step();
        r.i.armed = true;
        assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
        assert_eq!(r.at(T0 + 1_000 + LOSS + 1).state, State::Stop, "never airborne");
        let mut r = Rig::pilot();
        r.i.armed = false;
        r.i.airborne = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Stop);
        r.i.armed = true;
        r.i.td_held = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.at(t).state, State::Stop, "re-armed, never airborne since");
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
        let mut r = Rig::blind();
        let t = r.now() + 600 * S;
        assert_eq!(r.at(t).state, State::Blind);
        r.i.battery_critical = true;
        assert_eq!(r.step().state, State::Blind, "battery critical: no change");
        let out = r.drop_tier(Tier::T2);
        assert_eq!((out.state, out.landing), (State::Land, true));
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

    /// Seeded walk: RETURN and Start never happen without home; disarmed is STOP; `hands_off` follows the state; The
    /// level the flight code may demand never exceeds this step's flags.
    #[test]
    fn hands_off_i8_i12_walk_never_returns_or_starts_without_home_and_never_demands_uncovered() {
        let mut rng = Lcg(0x5eed_0001);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::new().i;
        let mut seen = [false; 8];
        let (mut prev, mut high_seen) = (State::Stop, false);
        let mut p = false;
        let (mut hand_overs, mut held) = (0u32, 0u32);
        let (mut lows_this_arming, mut lows) = (0u32, 0u32);
        let mut low_rng = Lcg(0x5eed_0002);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            i.battery_low = i.battery_critical || low_rng.chance(100);
            let out = h.step(&i);
            if !i.armed {
                lows_this_arming = 0;
            }
            if out.reason == Some(Reason::BattLow) && out.state != prev {
                assert!(matches!(prev, State::Hold | State::Auto), "batt-low from {prev:?}");
                assert!(i.battery_low && !i.battery_critical, "batt-low: {i:?}");
                assert!(matches!(out.state, State::Return | State::Land), "{out:?}");
                lows_this_arming += 1;
                lows += 1;
                assert_eq!(lows_this_arming, 1, "batt-low once per arming");
            }
            let high_now = i.crsf.is_some_and(|r| r.switch6 == Switch3::High);
            let arm_now = i.crsf.is_some_and(|r| r.arm);
            let mav = h.mavlink_flight();
            let ms_now = i.crsf.is_some_and(|r| r.mission);
            let live = i.crsf.is_some_and(|r| i.now_us.saturating_sub(r.t_us) <= LOSS);
            if mav && prev == State::Takeover && out.state == State::Pilot {
                assert!(live && arm_now && !ms_now && high_now, "column 2 hand-over: {out:?}");
                hand_overs += 1;
            } else if prev == State::Takeover && out.state == State::Pilot {
                assert!(high_seen && !high_now, "hand-over: seen {high_seen}, now High {high_now}");
                assert!(p && !high_now && arm_now, "hand-over: P {p}, now High {high_now}, arm {arm_now}");
                hand_overs += 1;
            }
            if out.state == State::Takeover {
                if prev != State::Takeover {
                    high_seen = false;
                    p = false;
                }
                high_seen |= high_now;
                p |= arm_now && high_now;
            }
            let want = mav && out.hands_off && !arm_now && ms_now;
            assert_eq!(out.arm_off_holds, want, "{out:?}");
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
        assert!(lows > 50, "batt-low exercised: {lows}");
    }

    #[test]
    fn hands_off_b8_home_d_waits_for_a_valid_height_before_take_off() {
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
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_d_valid = false;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        r.i.position_d_valid = true;
        assert_eq!(r.at(T0 + 3_000).home, None);
        let mut r = Rig::pilot();
        assert!(r.step().home.is_some());
        r.i.armed = false;
        assert_eq!(r.step().home, None);
    }

    // ---- Exit priority, pairs in the same step ----

    #[test]
    fn hands_off_b1_each_pair_of_exits_gives_the_higher_one() {
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land, "hold: battery over link");
        let mut r = Rig::hold();
        r.drop_tier(Tier::T2);
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!(r.at(t).state, State::Land, "hold: T3' over link");
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        r.i.mission.plan_id = Some(PLAN + 1);
        let t = r.now() + LINK_LOSS_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped), (State::Return, true), "hold: link over plan");
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "hold: CRSF over battery");
        let mut r = Rig::hold();
        r.i.armed = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Stop, "disarm over CRSF");
        let mut r = Rig::auto();
        r.drop_tier(Tier::T2);
        r.i.mission.plan_id = Some(PLAN + 1);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        assert_eq!(r.hold_drop().state, State::Land, "auto: T3' over plan");
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        let t = r.now() + STATION_LOST_US + 1;
        assert_eq!(r.at(t).state, State::Land, "auto: battery over station");
        let mut r = Rig::auto();
        r.i.mission.plan_id = Some(PLAN + 1);
        let t = r.now() + STATION_LOST_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped, out.events.place_kept), (State::Hold, true, false));
        let mut r = Rig::auto();
        r.i.last_waypoint_reached = true;
        let t = r.now() + STATION_LOST_US + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.events.place_dropped, out.events.place_kept), (State::Hold, true, false));
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "auto: CRSF over battery");
        let mut r = Rig::ret();
        r.i.home_reached = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::Low).state, State::Takeover, "return: CRSF over arrival");
    }

    // ---- No one-step HOLD ----

    #[test]
    fn hands_off_b2_commands_into_hold_denied_when_hold_would_leave_at_once() {
        let mut r = Rig::land();
        r.i.battery_critical = true;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::BatteryCritical)));
        assert_eq!(r.step().state, State::Land);
        let mut r = Rig::land();
        let t = r.now() + LINK_LOSS_US + 1;
        r.i.now_us = t;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::LinkLost)));
        assert_eq!(r.step().state, State::Land);
        let mut r = Rig::land();
        r.drop_tier(Tier::T2);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        r.hold_drop();
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotT3)));
        assert_eq!(r.step().state, State::Land);
        for c in [Command::Pause, Command::Abort] {
            let mut r = Rig::ret();
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Return, denied(DenyReason::LinkLost)), "{c:?}");
            assert_eq!(out.events, Events::NONE);
            assert_eq!(r.step().state, State::Return);
        }
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

    // ---- T3' ----

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

    // ---- Debounced downgrades ----

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

    // ---- A kept place survives only Start ----

    #[test]
    fn hands_off_b7_every_exit_from_hold_drops_a_kept_place() {
        let paused = || {
            let mut r = Rig::auto();
            assert!(r.cmd(Command::Pause).events.place_kept);
            r
        };
        let mut r = paused();
        let t = r.now() + 1_000;
        assert!(r.report(t, Switch3::Low).events.place_dropped, "takeover");
        let mut r = paused();
        assert!(r.cmd(Command::Land).events.place_dropped, "land");
        let mut r = paused();
        r.i.battery_critical = true;
        assert!(r.step().events.place_dropped, "battery");
        let mut r = paused();
        r.i.armed = false;
        assert!(r.step().events.place_dropped, "stop");
        let mut r = paused();
        assert_eq!(r.cmd(Command::Start).events.start, Some(StartAt::KeptPlace));
    }

    // ---- I9 ----

    /// HOLD with the station present, entered with the vehicle `vehicle_m` north of home.
    /// hold: the anchor), the farthest waypoint at `waypoint_m`.
    fn start_at(vehicle_m: f32, waypoint_m: f32) -> Output {
        let mut r = Rig::pilot();
        r.i.position_ne_m = [HOME_NE[0] + vehicle_m, HOME_NE[1]];
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        let t = r.now() + 1_000;
        r.i.last_radio_heartbeat_us = Some(t);
        assert_eq!(r.report_ms(t, Switch3::High, true).state, State::Hold);
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
        let cases: [(DenyReason, fn(&mut Rig)); 16] = [
            (DenyReason::StationAbsent, |r| {
                r.i.last_radio_heartbeat_us = Some(r.now() - STATION_LOST_US - 1)
            }),
            (DenyReason::NoWaypoints, |r| r.i.mission.has_waypoints = false),
            (DenyReason::HeightOutsideLimits, |r| r.i.mission.z_within_limits = false),
            (DenyReason::NoHome, |r| r.h.home_ne = None),
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
                r.i.crsf = r.i.crsf.map(|c| RcSample { switch6: Switch3::Middle, ..c })
            }),
            (DenyReason::MissionSwitchOff, |r| {
                r.i.crsf = r.i.crsf.map(|c| RcSample { t_us: r.i.now_us - LOSS - 1, ..c })
            }),
        ];
        for (reason, set) in cases {
            let mut r = Rig::ms_hold();
            set(&mut r);
            let out = r.cmd(Command::Start);
            assert_eq!((out.state, out.command), (State::Hold, denied(reason)), "{reason:?}");
            assert_eq!(out.events, Events::NONE);
        }
        let mut r = Rig::hold();
        r.drop_tier(Tier::T2);
        r.i.last_radio_heartbeat_us = Some(r.now() + TIER_DEBOUNCE_US);
        assert_eq!(r.hold_drop().state, State::Land);
        let out = r.cmd(Command::Start);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotInThisState)));
        let mut r = Rig::ms_hold();
        assert_eq!(r.h.start_check(&r.i), None);
        r.i.crsf = r.i.crsf.map(|c| RcSample { arm: false, ..c });
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::ArmSwitchOff));
    }

    #[test]
    fn hands_off_switch_not_high_when_hold_came_from_a_middle_report() {
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        r.report(t, Switch3::Middle);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::MissionSwitchOff));
        let t = r.now() + 1_000;
        assert_eq!(r.report_ms(t, Switch3::Middle, true).state, State::Hold);
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::SwitchNotHigh));
    }

    // ---- Commands ----

    #[test]
    fn hands_off_commands_on_a_crsf_flight_are_refused_with_code_3() {
        for c in [Command::Start, Command::Pause, Command::Abort, Command::Land] {
            let mut r = Rig::crsf_pilot();
            assert_eq!(r.cmd(c).command, denied(DenyReason::HandsOffDisabled), "pilot {c:?}");
            let mut r = Rig::crsf_takeover();
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::HandsOffDisabled)));
            let mut r = Rig::crsf_land();
            let t = r.now() + 1_000;
            r.i.crsf = r.i.crsf.map(|s| RcSample { t_us: t, ..s });
            r.i.now_us = t;
            let out = r.cmd(c);
            assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::HandsOffDisabled)));
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

    // ---- CRSF flights ----

    /// On a CRSF flight a stick loss on T3' gives LAND: never HOLD, AUTO or RETURN, no mission event, no command
    /// accepted, and never STOP from a stick loss in the air.
    #[test]
    fn hands_off_crsf_walk_gives_no_hold_and_no_mission() {
        let mut rng = Lcg(0x5eed_0000);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::crsf().i;
        let mut seen = [0u32; 8];
        let mut low_rng = Lcg(0x5eed_0002);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, false);
            i.battery_low = i.battery_critical || low_rng.chance(100);
            let was = h.state();
            let out = h.step(&i);
            seen[usize::from(out.state.code() - 1)] += 1;
            assert!(!matches!(out.state, State::Hold | State::Auto | State::Return), "{:?} on CRSF", out.state);
            assert_ne!(out.reason, Some(Reason::BattLow));
            if matches!(was, State::Pilot | State::Stop) && i.crsf.is_some_and(|r| !r.arm) {
                assert_eq!(out.state, State::Stop, "CRSF: arm OFF in PILOT or STOP is STOP");
            }
            assert_eq!(out.return_height_m, None);
            assert_eq!(out.events, Events::NONE);
            assert!(matches!(out.command, None | Some(CommandResult::Denied(_))));
            assert!(!(out.state == State::Stop && was != State::Stop && i.armed && i.airborne && i.crsf.is_some_and(|r| r.arm)), "never STOP from the stick loss in the air");
        }
        for s in [State::Pilot, State::Stop, State::Takeover, State::Land, State::Blind] {
            assert!(seen[usize::from(s.code() - 1)] > 100, "{s:?} reached: {seen:?}");
        }
    }

    // ---- Review follow-up ----

    /// Arm OFF with the mission switch OFF is STOP in the same step, in every state, in the air and on the ground.
    #[test]
    fn hands_off_arm_switch_off_in_the_same_step_from_every_state() {
        let rigs: [(&str, fn() -> Rig, State); 9] = [
            ("pilot", Rig::pilot, State::Pilot),
            ("hold", Rig::hold, State::Hold),
            ("auto", Rig::auto, State::Auto),
            ("return", Rig::ret, State::Return),
            ("land", Rig::land, State::Land),
            ("blind", Rig::blind, State::Blind),
            ("crsf pilot", Rig::crsf_pilot, State::Pilot),
            ("crsf land", Rig::crsf_land, State::Land),
            ("crsf blind", Rig::crsf_blind, State::Blind),
        ];
        for (name, make, from) in rigs {
            for airborne in [true, false] {
                let mut r = make();
                assert_eq!(r.h.state(), from, "{name}");
                r.i.airborne = airborne;
                r.i.td_held = false;
                let t = r.now() + 1_000;
                let out = r.report_with(t, CENTRED, Switch3::High, false);
                let want = (State::Stop, false, false, false);
                let got = (out.state, out.hands_off, out.synthetic_sticks, out.arm_off_holds);
                assert_eq!(got, want, "{name} airborne {airborne}");
            }
        }
    }

    #[test]
    fn hands_off_link_age_is_from_the_newer_of_crsf_and_heartbeat() {
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
        let mut r = Rig::pilot();
        r.drop_tier(Tier::T2);
        let t = r.now() + 2 * TIER_DEBOUNCE_US;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        assert_eq!(r.h.debounced, Tier::T2);
        r.i.tier = Tier::T3;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
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
        let mut r = Rig::ms_hold();
        r.i.last_radio_heartbeat_us = Some(r.now() - STATION_LOST_US - 1);
        r.i.mission.has_waypoints = false;
        r.i.mission.z_within_limits = false;
        r.h.home_ne = None;
        r.i.position_ne_valid = false;
        r.i.mission.max_waypoint_distance_m = None;
        r.i.heading_ok = false;
        r.i.tier = Tier::T2;
        r.i.battery_ok = false;
        r.i.crsf = r.i.crsf.map(|c| RcSample { switch6: Switch3::Middle, arm: false, mission: false, ..c });
        let steps: [(DenyReason, fn(&mut Rig)); 13] = [
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
            (DenyReason::MissionSwitchOff, |r| r.i.crsf = r.i.crsf.map(|c| RcSample { mission: true, ..c })),
            (DenyReason::ArmSwitchOff, |r| r.i.crsf = r.i.crsf.map(|c| RcSample { arm: true, ..c })),
            (DenyReason::SwitchNotHigh, |r| r.i.crsf = r.i.crsf.map(|c| RcSample { switch6: Switch3::High, ..c })),
        ];
        for (reason, fix) in steps {
            assert_eq!(r.h.start_check(&r.i), Some(reason));
            fix(&mut r);
        }
        assert_eq!(r.h.start_check(&r.i), None);
    }

    #[test]
    fn hands_off_start_is_denied_when_the_battery_is_critical_even_if_ok_is_set() {
        let mut r = Rig::ms_hold();
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

    // ---- The take-over lock ----

    /// LAND on a CRSF flight, then CRSF returns with this arm switch and switch6.
    fn returns_with(switch6: Switch3, arm: bool) -> (Rig, Output) {
        let mut r = Rig::crsf_land();
        let t = r.now() + 1_000;
        let out = r.report_with(t, CENTRED, switch6, arm);
        (r, out)
    }

    #[test]
    fn hands_off_c1_a1_arm_on_high_opens_the_lock_then_middle_hands_over() {
        let (mut r, out) = returns_with(Switch3::Middle, true);
        assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Takeover, true, false));
        assert!(out.synthetic_sticks && !r.h.lock_open);
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::High, true);
        assert_eq!((out.state, out.arm_off_holds), (State::Takeover, false), "lock open");
        assert!(r.h.lock_open);
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Middle, true);
        assert_eq!((out.state, out.hands_off, out.synthetic_sticks), (State::Pilot, false, false));
    }

    #[test]
    fn hands_off_c1_lock_needs_arm_on_and_high_in_the_same_report() {
        let (mut r, out) = returns_with(Switch3::Low, true);
        assert_eq!(out.state, State::Takeover);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Middle, true).state, State::Takeover);
        assert!(!r.h.lock_open);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Low, true).state, State::Takeover);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::High, true).state, State::Takeover);
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, FULL, Switch3::Low, true).state, State::Pilot);
    }

    #[test]
    fn hands_off_c1_a2_after_k_arm_off_is_stop_at_that_report() {
        let (mut r, out) = returns_with(Switch3::Low, true);
        assert_eq!((out.state, out.arm_off_holds), (State::Takeover, false));
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Low, false);
        assert_eq!((out.state, out.hands_off, out.arm_off_holds), (State::Stop, false, false));
        let (mut r, _) = returns_with(Switch3::High, true);
        let t = r.now() + 20_000;
        let out = r.report_with(t, FULL, Switch3::Middle, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
    }

    /// Armed and airborne in PILOT on a CRSF flight, then sticks lost on T3: LAND.
    fn lost_with() -> Rig {
        let mut r = Rig::crsf();
        r.step();
        r.i.armed = true;
        assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        let t = r.last_report() + LOSS + 1;
        let out = r.heartbeat(t);
        assert_eq!(out.state, State::Land);
        r
    }

    /// An arm-OFF report on return stops in that step, in the air and on the ground.
    #[test]
    fn hands_off_c1_a3_arm_off_on_return_is_stop_at_once() {
        for airborne in [true, false] {
            let mut r = lost_with();
            r.i.airborne = airborne;
            r.i.td_held = false;
            let t = r.now() + 1_000;
            let out = r.report_with(t, CENTRED, Switch3::High, false);
            assert_eq!((out.state, out.arm_off_holds, out.disarm), (State::Stop, false, true), "air {airborne}");
        }
    }

    #[test]
    fn hands_off_c1_a4_rc_lost_in_takeover_clears_p_and_a_second_return_needs_the_lock_again() {
        let (mut r, _) = returns_with(Switch3::High, true);
        assert!(r.h.lock_open);
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.arm_off_holds), (State::Land, false));
        assert!(!r.h.lock_open);
        let t = r.now() + 1_000;
        assert_eq!(r.report_with(t, CENTRED, Switch3::Middle, true).state, State::Takeover, "second return");
        let t = r.now() + 20_000;
        assert_eq!(r.report_with(t, CENTRED, Switch3::Middle, true).state, State::Takeover);
    }

    #[test]
    fn hands_off_c1_a6_pilot_arm_off_is_stop_at_once() {
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        let out = r.report_with(t, FULL, Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
        let mut r = Rig::crsf();
        r.step();
        r.i.armed = true;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        let out = r.report_with(T0 + 2_000, FULL, Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds), (State::Stop, false));
    }

    #[test]
    fn hands_off_c1_a7_takeover_with_battery_critical_descends_hands_over_and_touchdown_disarms() {
        let mut r = Rig::crsf_takeover();
        let out = r.step();
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Takeover, false, false));
        r.i.battery_critical = true;
        let out = r.step();
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Takeover, false, true));
        r.i.battery_critical = false;
        assert!(r.step().takeover_descent);
        let t = r.now() + 20_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Takeover);
        let t = r.now() + 20_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!((out.state, out.landing, out.takeover_descent), (State::Pilot, false, false));
        let mut r = Rig::crsf_takeover();
        r.i.battery_critical = true;
        assert!(r.step().takeover_descent);
        r.i.airborne = false;
        let out = r.step();
        assert_eq!((out.state, out.disarm, out.arm_off_holds), (State::Stop, true, false));
        r.i.armed = false;
        assert_eq!((r.step().state, r.step().disarm), (State::Stop, false));
        let mut r = Rig::crsf_land();
        r.i.battery_critical = true;
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Low);
        assert_eq!((out.state, out.takeover_descent), (State::Takeover, true));
        let t = r.last_report() + LOSS + 1;
        let out = r.at(t);
        assert_eq!((out.state, out.takeover_descent), (State::Land, false));
        assert_eq!(r.step().state, State::Land);
    }

    // ---- Return ----

    #[test]
    fn hands_off_c3_return_from_hold_and_auto_drops_the_place_and_latches_the_height() {
        let mut r = Rig::auto();
        r.cmd(Command::Pause);
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Return, Some(CommandResult::Accepted)));
        assert_eq!((out.events.place_dropped, out.return_height_m), (true, Some(20.0)));
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::NotInThisState));
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
        let mut r = Rig::pilot();
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::NotInThisState));
        let mut r = Rig::crsf_pilot();
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::HandsOffDisabled));
        let mut r = Rig::crsf_takeover();
        let out = r.cmd(Command::Return);
        assert_eq!((out.state, out.command), (State::Takeover, denied(DenyReason::HandsOffDisabled)));
    }

    #[test]
    fn hands_off_c3_return_deny_reasons_and_their_order() {
        let mut r = Rig::hold();
        assert_eq!(r.cmd_on(Command::Return, Link::Usb).command, denied(DenyReason::NotRadioLink));
        let mut r = Rig::crsf_land();
        let t = r.now() + 1_000;
        r.i.crsf = r.i.crsf.map(|s| RcSample { t_us: t, ..s });
        r.i.now_us = t;
        assert_eq!(r.cmd(Command::Return).command, denied(DenyReason::HandsOffDisabled));
        let mut r = Rig::crsf_land();
        let mut ev = Events::NONE;
        let ret = StationCommand {
            command: Command::Return,
            link: Link::Radio,
        };
        assert_eq!(r.h.command(ret, &r.i, &mut ev), CommandResult::Denied(DenyReason::HandsOffDisabled));
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
        let rigs: [(&str, fn() -> Rig); 2] = [("hold", Rig::hold), ("auto", Rig::auto)];
        for (name, make) in rigs {
            let mut r = make();
            let from = r.h.state();
            r.h.debounced = Tier::T2;
            r.i.battery_critical = true;
            r.h.home_ne = None;
            let mut ev = Events::NONE;
            for want in [DenyReason::NoHome, DenyReason::NotT3, DenyReason::BatteryCritical] {
                assert_eq!(r.h.command(ret, &r.i, &mut ev), CommandResult::Denied(want), "{name}");
                assert_eq!((r.h.state(), ev), (from, Events::NONE), "{name} {want:?}");
                match want {
                    DenyReason::NoHome => r.h.home_ne = Some(HOME_NE),
                    DenyReason::NotT3 => r.h.debounced = Tier::T3,
                    _ => r.i.battery_critical = false,
                }
            }
            assert_eq!(r.h.command(ret, &r.i, &mut ev), CommandResult::Accepted, "{name}");
        }
    }

    /// For Return: DENIED leaves the state, at this step and the next.
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
            assert_eq!(r.h.command(ret, &r.i, &mut ev), CommandResult::Denied(want));
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
            (DenyReason::MissionSwitchOff, 19),
        ];
        for (k, (reason, code)) in table.iter().enumerate() {
            assert_eq!(reason.code(), *code, "{reason:?}");
            assert_eq!(usize::from(reason.code()), k + 1 + usize::from(k > 0), "{reason:?}: 1, 3..=19; 0 reserved, 2 and 20 retired");
        }
    }

    // ---- Ground exit, WR2 disarm ----

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
        let mut r = Rig::hold();
        r.i.airborne = false;
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!((out.state, out.disarm), (State::Stop, true));
        let mut r = Rig::hold();
        r.i.airborne = false;
        let out = r.cmd(Command::Land);
        assert_eq!((out.state, out.disarm, out.command), (State::Stop, true, denied(DenyReason::NotInThisState)));
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
        let mut r = Rig::land();
        r.i.airborne = false;
        let out = r.step();
        assert_eq!((out.state, out.landing, out.disarm), (State::Stop, false, true));
        r.i.armed = false;
        let out = r.step();
        assert_eq!((out.state, out.disarm), (State::Stop, false));
        let mut r = Rig::land();
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!((r.step().state, r.h.state()), (State::Stop, State::Stop));
    }

    #[test]
    fn hands_off_wr2_disarm_is_stop_entered_while_armed() {
        let (mut r, out) = returns_with(Switch3::High, false);
        assert_eq!((out.state, out.arm_off_holds, out.disarm), (State::Stop, false, true));
        assert_eq!((r.step().state, r.step().disarm), (State::Stop, false));
        let mut r = Rig::pilot();
        let t = r.now() + 1_000;
        assert!(r.report_with(t, FULL, Switch3::High, false).disarm, "pilot arm off");
        let (mut r, _) = returns_with(Switch3::Low, true);
        let t = r.now() + 20_000;
        assert!(r.report_with(t, FULL, Switch3::Low, false).disarm, "takeover arm off");
        let mut r = Rig::pilot();
        r.i.airborne = false;
        let t = r.last_report() + LOSS + 1;
        assert!(r.at(t).disarm, "sticks lost on the ground");
        let mut r = Rig::crsf();
        r.step();
        r.i.armed = true;
        assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
        let out = r.at(T0 + 1_000 + LOSS + 1);
        assert_eq!((out.state, out.disarm), (State::Stop, true), "CRSF, never airborne");
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

    /// One random step: time moves 0 to 2 s (now and then 10 to 12 s with no report and no heartbeat), every input may
    /// change; armed and airborne change rarely.
    fn random_inputs(rng: &mut Lcg, i: &mut Inputs, receiver: bool) {
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
                source: if receiver { RcSource::Receiver } else { RcSource::Crsf },
                mission: receiver && if i.crsf.is_some_and(|c| c.mission) { !rng.chance(20) } else { rng.chance(100) },
                t_us: now,
            });
        }
        if !silent && rng.chance(200) {
            i.last_radio_heartbeat_us = Some(now);
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
                alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
            };
        }
        i.last_waypoint_reached = rng.chance(5);
        i.home_reached = rng.chance(5);
        i.heading_ok = !rng.chance(50);
        i.battery_ok = !rng.chance(50);
        i.battery_critical = rng.chance(3);
    }

    // ---- The hold-integrity distance check ----

    /// One step over X, then back below: set at the step that receives the distance, kept until the disarm edge; LAND
    /// At that step (a landing cause of its own: no debounce), T3' kept (the demand stays Position)
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

    /// The states the check covers (HOLD, AUTO, RETURN, LAND on T3', and TAKEOVER:, it flies HOLD's law); never
    /// LAND without T3', BLIND, PILOT, STOP, a disarmed step or a step with the airborne latch false.
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
        let mut r = Rig::hold();
        r.i.airborne = false;
        r.i.track_error_m = Some(100.0);
        assert_eq!(r.step().state, State::Stop);
        assert!(!r.h.track_fault());
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

    /// Heading_ok = the input only: T3' (the demand) follows the input whatever the fault; the fault lands at once, so
    /// Start meets LAND (NotInThisState); Start's own check denies a latched fault as HeadingNotOk.
    #[test]
    fn h14_heading_ok_is_the_input_only_and_start_is_denied_with_the_fault() {
        for (heading, fault) in [(true, false), (true, true), (false, false), (false, true)] {
            let mut r = Rig::ms_hold();
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
        let mut r = Rig::ms_hold();
        assert_eq!(r.h.start_check(&r.i), None);
        r.h.track_fault = true;
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::HeadingNotOk));
    }

    // ---- The anchor, the braking allowance, the speed cap and X ----

    /// PILOT, then sticks lost at `ne` moving `v`: HOLD.
    fn hold_from(ne: [f32; 2], v: [f32; 2]) -> Rig {
        let mut r = Rig::pilot();
        r.i.position_ne_m = ne;
        r.i.velocity_ne_ms = v;
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        r
    }

    /// None in PILOT; at the entry step A = the position and b = |v|^2 / (2 x 3.0 m/s^2)
    #[test]
    fn h13_the_anchor_and_the_braking_allowance_are_set_at_the_entry_step() {
        assert_eq!(Rig::pilot().h.anchor(), None);
        let r = hold_from([20.0, -7.0], [3.0, -4.0]);
        assert_eq!(r.h.anchor(), Some(([20.0, -7.0], 25.0 / 6.0)));
        let r = hold_from([20.0, -7.0], [20.0, 0.0]);
        assert_eq!(r.h.anchor(), Some(([20.0, -7.0], 400.0 / 6.0)), "67 m at 20 m/s");
    }

    /// AUTO and RETURN clear the anchor; entry from them to HOLD (Pause), TAKEOVER (a report) or LAND on T3'.
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

    /// A 0.5 s tier dip (Altitude) with the vehicle 2 m away, then HOLD -> TAKEOVER -> HOLD moved and moving: A and b
    /// kept.
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
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot, "hand-back");
        assert_eq!(r.h.anchor(), None);
        r.i.position_ne_m = [30.0, 10.0];
        r.i.velocity_ne_ms = [0.0, 0.0];
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        assert_eq!(r.h.anchor(), Some(([30.0, 10.0], 0.0)));
    }

    /// V0 3 m/s (b 1.5 m): exactly X + b does not trip; one ULP over, infinity and not a number do.
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

    /// The anchor distance is read on a T2 step (Altitude: no target distance); not on a step with the NE velocity or
    /// position not valid, nor with the airborne latch false.
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

    /// Entry with the NE position or velocity not valid or not finite.
    /// Than the debounce, so HOLD): no anchor, stale or zero.
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

    /// X for PILOT_SPEED_XY 3, 5 and 20 (capped to 5) is 7, 5 and 5 m; the speed limit is v_ho in every hands-off
    /// state and the speed in force in PILOT and STOP.
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

    // ---- What happens after a track fault ----

    /// The track fault is a landing cause ranked with battery critical: with each other automatic exit of HOLD, AUTO
    /// And RETURN in the same step it gives LAND, as battery critical does.
    #[test]
    fn h14_the_track_fault_lands_like_battery_critical_over_every_other_exit() {
        let cases: [(&str, fn() -> Rig, fn(&mut Rig) -> u64); 5] = [
            ("hold: link > 10 s with home", Rig::hold, |r| r.now() + LINK_LOSS_US + 1),
            ("auto: new plan", Rig::auto, |r| {
                r.i.mission.plan_id = Some(PLAN + 1);
                r.now() + 1_000
            }),
            ("auto: last waypoint", Rig::auto, |r| {
                r.i.last_waypoint_reached = true;
                r.now() + 1_000
            }),
            ("auto: station lost", Rig::auto, |r| r.now() + STATION_LOST_US + 1),
            ("return: arrival", Rig::ret, |r| {
                r.i.home_reached = true;
                r.now() + 1_000
            }),
        ];
        for (name, rig, other) in cases {
            for (cause, law) in [("fault", LandLaw::Brake), ("battery", LandLaw::Hold)] {
                let mut r = rig();
                if cause == "fault" {
                    r.i.track_error_m = Some(6.0);
                } else {
                    r.i.battery_critical = true;
                }
                let t = other(&mut r);
                let out = r.at(t);
                assert_eq!((out.state, out.land_law, out.events.place_kept), (State::Land, Some(law), false), "{name}, {cause}");
                assert_eq!(r.h.track_fault(), cause == "fault", "{name}, {cause}");
            }
        }
        let mut r = Rig::hold();
        r.i.track_error_m = Some(6.0);
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Low);
        assert_eq!((out.state, out.takeover_descent, out.land_law), (State::Takeover, true, Some(LandLaw::Brake)), "CRSF over fault");
        let mut r = Rig::hold();
        r.i.track_error_m = Some(6.0);
        r.i.armed = false;
        assert_eq!(r.step().state, State::Stop, "disarm over fault");
        assert!(!r.h.track_fault());
    }

    /// D_A is read in TAKEOVER on a T2 step (an Altitude setpoint; TAKEOVER then flies its descent, level); in LAND
    /// only on a step that holds (no fault, this step's T3'), where a trip gives braking at that step.
    #[test]
    fn h14_the_anchor_is_read_in_takeover_on_a_t2_step_and_in_land_only_while_it_holds() {
        let a = Some(([10.0, 10.0], 0.0));
        let mut r = Rig::takeover();
        assert_eq!(r.h.anchor(), a);
        r.i.tier = Tier::T2;
        r.i.position_ne_m = [16.0, 10.0];
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Low);
        assert!(r.h.track_fault());
        assert_eq!(
            (out.state, out.takeover_descent, out.land_law, out.demand),
            (State::Takeover, true, Some(LandLaw::Level), Demand::Altitude)
        );
        let mut r = Rig::land();
        assert_eq!(r.h.anchor(), a);
        r.i.tier = Tier::T2;
        r.i.position_ne_m = [16.0, 10.0];
        for k in 0..3_000 {
            let t = r.now() + 1_000;
            let out = r.at(t);
            assert_eq!((out.state, out.land_law), (State::Land, Some(LandLaw::Level)), "step {k}");
            assert!(!r.h.track_fault(), "step {k}");
        }
        r.i.tier = Tier::T3;
        let out = r.step();
        assert!(r.h.track_fault());
        assert_eq!((out.state, out.land_law, out.land_holds_position), (State::Land, Some(LandLaw::Brake), false));
        assert_eq!(r.h.anchor(), a);
        let mut r = Rig::land();
        r.i.position_ne_m = [15.0, 10.0];
        let out = r.step();
        assert_eq!((out.land_law, r.h.track_fault()), (Some(LandLaw::Hold), false), "exactly X + b");
        r.i.position_ne_m = [16.0, 10.0];
        let out = r.step();
        assert_eq!((out.land_law, r.h.track_fault()), (Some(LandLaw::Brake), true), "6 m");
        r.i.position_ne_m = [100.0, 10.0];
        for k in 0..100 {
            let t = r.now() + 1_000;
            let out = r.at(t);
            assert_eq!((out.state, out.land_law, r.h.anchor()), (State::Land, Some(LandLaw::Brake), a), "step {k}");
        }
    }

    /// Once latched, no hands-off state holds a point again until disarm.
    #[test]
    fn h14_a_latched_fault_resolves_every_hands_off_entry_to_land_until_disarm() {
        let mut r = Rig::hold();
        r.i.track_error_m = Some(6.0);
        assert_eq!(r.step().state, State::Land);
        r.i.track_error_m = None;
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::NotT3)));
        assert_eq!(r.step().state, State::Land);
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::NotInThisState));
        assert_eq!(r.h.hold_entry_check(&r.i), Some(DenyReason::NotT3));
        assert_eq!(r.h.return_check(&r.i), Some(DenyReason::NotT3));
        assert_eq!(r.h.start_check(&r.i), Some(DenyReason::HeadingNotOk));
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Low);
        assert_eq!((out.state, out.takeover_descent, out.land_law), (State::Takeover, true, Some(LandLaw::Brake)));
        let t = r.now() + 1_000;
        r.report(t, Switch3::High);
        let t = r.now() + 1_000;
        let out = r.report(t, Switch3::Middle);
        assert_eq!(
            (out.state, out.hands_off, out.landing, out.takeover_descent, out.land_law),
            (State::Pilot, false, false, false, None)
        );
        assert!(r.h.track_fault());
        let t = r.last_report() + LOSS + 1;
        let out = r.heartbeat(t);
        assert_eq!((out.state, out.land_law, out.demand), (State::Land, Some(LandLaw::Brake), Demand::Position));
        assert_eq!(r.h.anchor(), None);
        r.i.armed = false;
        assert_eq!(r.step().state, State::Stop);
        assert!(!r.h.track_fault());
        assert_eq!(r.h.hold_entry_check(&r.i), None);
        r.i.armed = true;
        r.i.airborne = false;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        r.i.airborne = true;
        let t = r.now() + 1_000;
        r.report(t, Switch3::High);
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
    }

    /// Item 4 over random inputs (the random positions trip the anchor): with the fault latched the state
    /// is never HOLD, AUTO or RETURN, TAKEOVER flies its descent, LAND never holds.
    #[test]
    fn h14_walk_a_latched_fault_never_holds_a_point() {
        let mut rng = Lcg(0x5eed_0014);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::new().i;
        let (mut was, mut faulted, mut land, mut brake, mut takeover) = (false, 0u32, 0u32, 0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            let out = h.step(&i);
            let fault = h.track_fault();
            assert!(!was || fault || !i.armed, "cleared while armed: {out:?}");
            was = fault;
            if !fault {
                continue;
            }
            faulted += 1;
            assert!(!matches!(out.state, State::Hold | State::Auto | State::Return), "{out:?}");
            assert!(!out.land_holds_position && out.land_law != Some(LandLaw::Hold), "{out:?}");
            if out.state == State::Takeover {
                assert!(out.takeover_descent, "{out:?}");
                takeover += 1;
            }
            if out.land_law.is_some() {
                assert_eq!(out.land_law == Some(LandLaw::Brake), out.demand == Demand::Position, "{out:?}");
                brake += u32::from(out.land_law == Some(LandLaw::Brake));
            }
            land += u32::from(out.state == State::Land);
        }
        assert!(faulted > 1_000 && land > 100 && brake > 100 && takeover > 10, "{faulted} {land} {brake} {takeover}");
    }

    // ---- The mission switch ----

    /// One report at `t` of the column `col` (1 to 5; 6: no report, the step past the stick-loss time)
    fn column(r: &mut Rig, col: u8) -> Output {
        let t = r.now() + 1_000;
        match col {
            1 => r.report_full(t, CENTRED, Switch3::High, true, true),
            2 => r.report_full(t, CENTRED, Switch3::High, true, false),
            3 => r.report_full(t, CENTRED, Switch3::Middle, true, false),
            4 => r.report_full(t, CENTRED, Switch3::High, false, true),
            5 => r.report_full(t, CENTRED, Switch3::High, false, false),
            _ => {
                let t = (r.last_report() + LOSS + 1).max(t);
                r.at(t)
            }
        }
    }

    /// Each of the six columns in each state of a MAVLink flight (airborne, T3', home set, the station heard): the
    /// state after that step.
    #[test]
    fn n19_each_column_in_each_state() {
        use State::{Auto, Blind, Hold, Land, Pilot, Return, Stop, Takeover};
        let rows: [(&str, fn() -> Rig, [State; 6]); 7] = [
            ("pilot", Rig::pilot, [Hold, Pilot, Pilot, Land, Stop, Hold]),
            ("takeover", Rig::takeover, [Takeover, Pilot, Takeover, Land, Stop, Hold]),
            ("hold", Rig::hold, [Hold, Pilot, Takeover, Land, Stop, Hold]),
            ("auto", Rig::auto, [Auto, Pilot, Takeover, Land, Stop, Auto]),
            ("return", Rig::ret, [Return, Pilot, Takeover, Land, Stop, Return]),
            ("land", Rig::land, [Land, Pilot, Takeover, Land, Stop, Land]),
            ("blind", Rig::blind, [Blind, Pilot, Takeover, Blind, Stop, Blind]),
        ];
        for (name, make, want) in rows {
            for col in 1..=6u8 {
                let mut r = make();
                assert!(r.h.mavlink_flight(), "{name}");
                let out = column(&mut r, col);
                assert_eq!(out.state, want[usize::from(col - 1)], "{name}, column {col}");
                assert_eq!(out.disarm, out.state == Stop, "{name}, column {col}");
                assert_eq!(out.hands_off, !matches!(out.state, Pilot | Stop), "{name}, column {col}");
                assert_eq!(out.arm_off_holds, col == 4 && out.hands_off, "{name}, column {col}");
            }
        }
    }

    /// Section 2, the priority order: disarmed, the arm columns, columns 2 and 3, each state's own exits,
    /// column 1, commands.
    #[test]
    fn n19_the_priority_order() {
        let mut r = Rig::hold();
        r.i.armed = false;
        assert_eq!(column(&mut r, 1).state, State::Stop);
        for make in [Rig::pilot as fn() -> Rig, Rig::hold] {
            let mut r = make();
            r.i.airborne = false;
            let out = column(&mut r, 4);
            assert_eq!((out.state, out.disarm), (State::Stop, true));
        }
        for col in [2, 3] {
            let mut r = Rig::hold();
            r.i.airborne = false;
            assert_eq!(column(&mut r, col).state, State::Stop, "column {col}");
        }
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        assert_eq!(column(&mut r, 2).state, State::Pilot);
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        let out = column(&mut r, 3);
        assert_eq!((out.state, out.takeover_descent), (State::Takeover, true));
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        assert_eq!(column(&mut r, 1).state, State::Land);
        let mut r = Rig::auto();
        r.i.last_waypoint_reached = true;
        assert_eq!(column(&mut r, 1).state, State::Hold);
        let mut r = Rig::pilot();
        r.i.last_radio_heartbeat_us = Some(r.now());
        r.i.command = Some(StationCommand { command: Command::Start, link: Link::Radio });
        let out = column(&mut r, 1);
        assert_eq!((out.state, out.command), (State::Auto, Some(CommandResult::Accepted)));
    }

    /// Every conjunct of column 1's entry from PILOT false gives PILOT.
    /// hold_entry_check's tier, track fault and battery; and the mission switch itself)
    #[test]
    fn n19_every_conjunct_of_the_column_1_entry_false_gives_pilot() {
        let cases: [(&str, fn(&mut Rig)); 5] = [
            ("not Position", |_| {}),
            ("latch false", |r| {
                r.i.airborne = false;
                r.i.td_held = false;
            }),
            ("tier T2 for 1 s", |r| {
                r.i.tier = Tier::T2;
                r.step();
                let t = r.now() + TIER_DEBOUNCE_US;
                assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
            }),
            ("track fault", |r| r.h.track_fault = true),
            ("battery critical", |r| r.i.battery_critical = true),
        ];
        for (name, set) in cases {
            let mut r = Rig::pilot();
            set(&mut r);
            let t = r.now() + 1_000;
            let switch6 = if name == "not Position" { Switch3::Middle } else { Switch3::High };
            let out = r.report_full(t, CENTRED, switch6, true, true);
            assert_eq!((out.state, out.hands_off), (State::Pilot, false), "{name}");
        }
        let mut r = Rig::pilot();
        assert_eq!(column(&mut r, 2).state, State::Pilot, "the mission switch OFF");
        let mut r = Rig::pilot();
        assert_eq!(column(&mut r, 1).state, State::Hold, "control: all met");
    }

    /// Columns 4 and 5 read the newest report even when it is stale.
    #[test]
    fn n19_the_arm_columns_read_a_stale_report() {
        for (make, name) in [(Rig::pilot as fn() -> Rig, "pilot"), (Rig::hold, "hold"), (Rig::auto, "auto"), (Rig::ret, "return")] {
            for (mission, want) in [(true, State::Land), (false, State::Stop)] {
                let mut r = make();
                let old = r.now().saturating_sub(5 * S);
                r.i.crsf = r.i.crsf.map(|c| RcSample { arm: false, mission, t_us: old, ..c });
                let t = r.now() + 1_000;
                let out = r.at(t);
                assert_eq!(out.state, want, "{name}, mission switch {mission}");
                assert_eq!(out.arm_off_holds, mission, "{name}");
            }
        }
    }

    /// Section 2 (LAND from arm OFF with the mission switch ON) and section 4: Abort is refused with code 15 while
    /// the newest report has arm OFF; arm ON again changes nothing and Abort is taken.
    #[test]
    fn n19_abort_from_an_arm_off_land_is_refused_with_code_15() {
        let mut r = Rig::hold();
        assert_eq!(column(&mut r, 4).state, State::Land);
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Land, denied(DenyReason::ArmSwitchOff)));
        assert_eq!(DenyReason::ArmSwitchOff.code(), 15);
        assert_eq!(column(&mut r, 1).state, State::Land, "Q1: the landing continues");
        let out = r.cmd(Command::Abort);
        assert_eq!((out.state, out.command), (State::Hold, Some(CommandResult::Accepted)));
        let mut r = Rig::hold();
        column(&mut r, 4);
        assert_eq!(column(&mut r, 5).state, State::Stop);
        let mut r = Rig::hold();
        column(&mut r, 4);
        assert_eq!(column(&mut r, 2).state, State::Pilot);
    }

    /// The MAVLink-flight rule and section 3: a CRSF flight never enters HOLD, AUTO or RETURN: a stick loss on T3'
    /// lands, a radio command is refused with code 3.
    #[test]
    fn n19_a_crsf_flight_has_no_hands_off_path() {
        let mut r = Rig::crsf_pilot();
        assert!(!r.h.mavlink_flight());
        assert_eq!(r.cmd(Command::Land).command, denied(DenyReason::HandsOffDisabled));
        let mut r = Rig::crsf_land();
        for c in [Command::Start, Command::Abort, Command::Return] {
            assert_eq!(r.cmd(c).command, denied(DenyReason::HandsOffDisabled), "{c:?}");
            assert_eq!(r.cmd_on(c, Link::Usb).command, denied(DenyReason::NotRadioLink), "{c:?}");
        }
        assert_eq!(r.h.state(), State::Land);
    }

    /// The walk of `hands_off_i8_i12_walk_...` on a CRSF flight.
    /// from CRSF, the mission switch never read): never a MAVLink flight, never HOLD, AUTO or RETURN.
    #[test]
    fn n19_crsf_walk_keeps_the_c1_lock() {
        let mut rng = Lcg(0x5eed_0001);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::crsf().i;
        let mut seen = [0u32; 8];
        let (mut prev, mut high_seen) = (State::Stop, false);
        let mut p = false;
        let (mut hand_overs, mut stops, mut armed_steps) = (0u32, 0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            i.crsf = i.crsf.map(|c| RcSample { source: RcSource::Crsf, mission: false, ..c });
            let out = h.step(&i);
            assert!(!h.mavlink_flight());
            assert!(!matches!(out.state, State::Hold | State::Auto | State::Return), "{out:?}");
            let high_now = i.crsf.is_some_and(|r| r.switch6 == Switch3::High);
            let arm_now = i.crsf.is_some_and(|r| r.arm);
            if prev == State::Takeover && out.state == State::Pilot {
                assert!(high_seen && !high_now, "hand-over: seen {high_seen}, now High {high_now}");
                assert!(p && !high_now && arm_now, "hand-over: P {p}, now High {high_now}, arm {arm_now}");
                hand_overs += 1;
            }
            if out.state == State::Takeover {
                if prev != State::Takeover {
                    high_seen = false;
                    p = false;
                }
                high_seen |= high_now;
                p |= arm_now && high_now;
            }
            assert!(!out.arm_off_holds, "{out:?}");
            if i.crsf.is_some_and(|r| !r.arm) {
                assert_eq!(out.state, State::Stop, "arm OFF is STOP: {out:?}");
                stops += u32::from(prev != State::Stop);
            }
            assert_eq!(out.hands_off, !matches!(out.state, State::Pilot | State::Stop));
            if !i.armed {
                assert_eq!(out.state, State::Stop);
            }
            armed_steps += u32::from(i.armed);
            seen[usize::from(out.state.code() - 1)] += 1;
            prev = out.state;
        }
        for s in [State::Pilot, State::Takeover, State::Land, State::Blind, State::Stop] {
            assert!(seen[usize::from(s.code() - 1)] > 100, "{s:?} reached: {seen:?}");
        }
        assert!(hand_overs > 20 && stops > 20 && armed_steps > 10_000, "lock exercised: {hand_overs} hand-overs, {stops} arm-OFF stops, {armed_steps} armed steps");
    }

    /// On a CRSF flight an arm-OFF report stops at once in LAND, BLIND and TAKEOVER in the air, also with ch8 (the
    /// mission switch bit) ON: `arm_off_holds` is a MAVLink-flight output only.
    #[test]
    fn crsf_arm_off_with_ch8_on_stops_at_once_in_land_blind_and_takeover() {
        let rigs: [(&str, fn() -> Rig, State); 3] = [
            ("land", Rig::crsf_land, State::Land),
            ("blind", Rig::crsf_blind, State::Blind),
            ("takeover", Rig::crsf_takeover, State::Takeover),
        ];
        for (name, make, from) in rigs {
            let mut r = make();
            assert_eq!(r.h.state(), from, "{name}");
            r.i.td_held = false;
            let t = r.now() + 1_000;
            let out = r.report_full(t, CENTRED, Switch3::High, false, true);
            let got = (out.state, out.disarm, out.arm_off_holds, out.reason);
            assert_eq!(got, (State::Stop, true, false, Some(Reason::ArmOff)), "{name}");
        }
    }

    /// A CRSF flight with ch8 ON in random reports, commands included: never HOLD, AUTO or RETURN, never a held
    /// arm-OFF report.
    #[test]
    fn crsf_with_ch8_on_never_reaches_hold_auto_or_return() {
        let mut rng = Lcg(0x5eed_0002);
        let mut h = HandsOff::new(&Tuning::DEFAULT);
        let mut i = Rig::crsf().i;
        let (mut ch8_hands_off, mut ch8_stops) = (0u32, 0u32);
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            i.crsf = i.crsf.map(|c| RcSample { source: RcSource::Crsf, ..c });
            let was = h.state();
            let out = h.step(&i);
            assert!(!h.mavlink_flight());
            assert!(!matches!(out.state, State::Hold | State::Auto | State::Return), "{out:?}");
            assert!(!out.arm_off_holds && matches!(out.command, None | Some(CommandResult::Denied(_))), "{out:?}");
            let ch8 = i.crsf.is_some_and(|r| r.mission);
            ch8_hands_off += u32::from(ch8 && out.hands_off);
            if i.crsf.is_some_and(|r| !r.arm) {
                assert_eq!(out.state, State::Stop, "{out:?}");
                ch8_stops += u32::from(ch8 && was != State::Stop && i.armed);
            }
        }
        assert!(ch8_hands_off > 1_000 && ch8_stops > 10, "ch8 ON: {ch8_hands_off} hands-off steps, {ch8_stops} arm-OFF stops");
    }

    /// On a MAVLink flight never airborne since arming, PILOT with arm OFF and the
    /// mission switch ON is STOP (the "never airborne" half of column 4's ground)
    #[test]
    fn n19_pilot_arm_off_with_the_mission_switch_on_on_the_ground() {
        for loss in [false, true] {
            let mut r = Rig::new();
            r.i.td_held = false;
            r.step();
            r.i.armed = true;
            assert_eq!(r.report(T0 + 1_000, Switch3::High).state, State::Pilot);
            assert!(r.h.mavlink_flight() && !r.h.ever_airborne());
            let out = if loss { r.at(T0 + 1_000 + LOSS + 1) } else { column(&mut r, 4) };
            assert_eq!((out.state, out.disarm, out.arm_off_holds), (State::Stop, true, false), "stick loss {loss}");
        }
        let mut r = Rig::pilot();
        assert!(r.h.ever_airborne());
        r.i.airborne = false;
        r.i.td_held = false;
        let out = column(&mut r, 4);
        assert_eq!((out.state, out.disarm, out.arm_off_holds), (State::Land, false, true));
        let t = r.now() + 1_000;
        assert_eq!(r.at(t).state, State::Land, "TD not held");
        r.i.td_held = true;
        let out = r.step();
        assert_eq!((out.state, out.disarm), (State::Stop, true), "TD held");
    }

    /// Every rule that changes the state carries its reason (logged only): each reason on its path, on a MAVLink
    /// flight (the N19 table) and on a CRSF flight (the other table); the initial STOP has none.
    #[test]
    fn every_state_change_carries_its_reason() {
        let check = |out: Output, state: State, reason: Reason, what: &str| {
            assert_eq!((out.state, out.reason), (state, Some(reason)), "{what}");
        };
        let mut r = Rig::new();
        assert_eq!(r.step().reason, None, "the initial STOP");
        assert_eq!(r.h.reason(), None);
        r.i.armed = true;
        check(r.report(T0 + 1_000, Switch3::High), State::Pilot, Reason::Armed, "arming");
        assert_eq!(r.h.reason(), Some(Reason::Armed));
        check(r.report_full(T0 + 2_000, FULL, Switch3::High, false, true), State::Stop, Reason::ArmOffGround, "arm-gnd");

        let mut r = Rig::pilot();
        check(r.report_full(r.now() + 1_000, FULL, Switch3::High, false, true), State::Land, Reason::ArmOff, "arm-off");
        let mut r = Rig::hold();
        check(r.report_full(r.now() + 1_000, FULL, Switch3::High, false, false), State::Stop, Reason::ArmMsOff, "ms-off");

        let mut r = Rig::hold();
        assert_eq!(r.h.reason(), Some(Reason::RcLost), "sticks lost");
        r.i.airborne = false;
        check(r.step(), State::Stop, Reason::Landed, "WR1");

        let mut r = Rig::hold();
        check(r.report(r.now() + 1_000, Switch3::High), State::Pilot, Reason::MsOff, "column 2");
        let mut r = Rig::hold();
        check(r.report(r.now() + 1_000, Switch3::Low), State::Takeover, Reason::MsOff, "column 3");
        let mut r = Rig::pilot();
        check(r.report_ms(r.now() + 1_000, Switch3::High, true), State::Hold, Reason::MsOn, "column 1");

        let mut r = Rig::hold();
        r.drop_tier(Tier::T2);
        check(r.hold_drop(), State::Land, Reason::Tier, "HOLD tier");
        let mut r = Rig::hold();
        r.i.battery_critical = true;
        check(r.step(), State::Land, Reason::BattCrit, "HOLD battery");
        let mut r = Rig::hold();
        r.i.track_error_m = Some(100.0);
        check(r.step(), State::Land, Reason::Track, "HOLD track fault");
        let mut r = Rig::ret();
        assert_eq!(r.h.reason(), Some(Reason::LinkLost), "link loss");
        r.i.home_reached = true;
        check(r.step(), State::Land, Reason::Home, "RETURN home");

        let mut r = Rig::auto();
        assert_eq!(r.h.reason(), Some(Reason::Start), "Start");
        r.i.mission.plan_id = Some(PLAN + 1);
        check(r.step(), State::Hold, Reason::Plan, "new plan");
        let mut r = Rig::auto();
        r.i.last_waypoint_reached = true;
        check(r.step(), State::Hold, Reason::LastWp, "last waypoint");
        let mut r = Rig::auto();
        let t = r.now() + STATION_LOST_US + 1;
        let out = r.at(t);
        check(out, State::Hold, Reason::StationLost, "station lost");
        assert!(out.events.place_kept);
        let mut r = Rig::auto();
        r.i.battery_critical = true;
        check(r.step(), State::Land, Reason::BattCrit, "AUTO battery");

        let mut r = Rig::auto();
        check(r.cmd(Command::Pause), State::Hold, Reason::Pause, "Pause in AUTO");
        let mut r = Rig::auto();
        check(r.cmd(Command::Abort), State::Hold, Reason::Abort, "Abort in AUTO");
        let mut r = Rig::ret();
        r.i.last_radio_heartbeat_us = Some(r.now());
        check(r.cmd(Command::Pause), State::Hold, Reason::Pause, "Pause in RETURN");
        let mut r = Rig::hold();
        check(r.cmd(Command::Return), State::Return, Reason::Return, "Return");
        let mut r = Rig::land();
        assert_eq!(r.h.reason(), Some(Reason::Land), "Land");
        r.drop_tier(Tier::T1);
        check(r.hold_drop(), State::Blind, Reason::Tier, "LAND to BLIND");
        r.i.tier = Tier::T3;
        check(r.step(), State::Land, Reason::Tier, "BLIND to LAND");

        let mut r = Rig::hold();
        assert_eq!(r.cmd_on(Command::Return, Link::Usb).command, denied(DenyReason::NotRadioLink));
        assert_eq!(r.h.reason(), Some(Reason::RcLost));

        let mut r = Rig::crsf_pilot();
        assert_eq!(r.h.reason(), Some(Reason::Armed));
        r.i.armed = false;
        check(r.step(), State::Stop, Reason::Disarmed, "disarmed");
        let mut r = Rig::crsf_pilot();
        check(r.report_with(r.now() + 1_000, FULL, Switch3::High, false), State::Stop, Reason::ArmOff, "CRSF arm-off");
        let mut r = Rig::crsf_land();
        assert_eq!(r.h.reason(), Some(Reason::RcLost), "CRSF sticks lost");
        r.i.airborne = false;
        check(r.step(), State::Stop, Reason::Landed, "CRSF WR1");
        let mut r = Rig::crsf_takeover();
        assert_eq!(r.h.reason(), Some(Reason::RcBack), "fresh report in LAND");
        assert_eq!(r.report(r.now() + 1_000, Switch3::High).state, State::Takeover, "the lock opens");
        check(r.report(r.now() + 1_000, Switch3::Low), State::Pilot, Reason::Handover, "hand-over");
        let mut r = Rig::crsf_takeover();
        let t = r.now() + LOSS + 1;
        check(r.at(t), State::Land, Reason::RcLost, "TAKEOVER sticks lost");
    }

    #[test]
    fn reason_tokens_are_short_distinct_words() {
        let all = [
            Reason::ArmMsOff,
            Reason::ArmOffGround,
            Reason::ArmOff,
            Reason::Disarmed,
            Reason::Landed,
            Reason::Armed,
            Reason::RcLost,
            Reason::RcBack,
            Reason::Handover,
            Reason::MsOff,
            Reason::MsOn,
            Reason::Tier,
            Reason::BattCrit,
            Reason::BattLow,
            Reason::Track,
            Reason::LinkLost,
            Reason::Plan,
            Reason::LastWp,
            Reason::StationLost,
            Reason::Home,
            Reason::Start,
            Reason::Pause,
            Reason::Abort,
            Reason::Return,
            Reason::Land,
        ];
        for (k, a) in all.iter().enumerate() {
            let t = a.token();
            assert!(!t.is_empty() && t.len() <= 10 && !t.contains(' '), "{a:?} {t}");
            for b in all.iter().skip(k + 1) {
                assert_ne!(t, b.token(), "{a:?} {b:?}");
            }
        }
    }

    /// The in-air restart: a restart arming (`Inputs::restart`) never opens the home window, on the ground or in the
    /// air; the same arming without it latches home at its first armed step (the control)
    #[test]
    fn a_restart_arming_never_latches_home() {
        for restart in [false, true] {
            let mut r = Rig::new();
            r.i.restart = restart;
            assert_eq!(r.at(T0).home, None);
            r.i.armed = true;
            for k in 1..50u64 {
                let o = r.at(T0 + k * 1_000);
                let want = (!restart).then_some(Home { ne_m: HOME_NE, d_m: HOME_D });
                assert_eq!(o.home, want, "restart {restart}, step {k}");
            }
        }
    }

    // ---- Low battery ----

    /// HOLD and AUTO with LOW: RETURN with home, LAND without; reason batt-low; LOW beats link loss and the last
    /// waypoint; critical, tier and track beat LOW.
    #[test]
    fn hands_off_batt_low_returns_from_hold_and_auto_and_its_priority() {
        let mut r = Rig::hold();
        r.i.battery_low = true;
        let out = r.step();
        assert_eq!((out.state, out.reason), (State::Return, Some(Reason::BattLow)));
        assert_eq!(out.return_height_m, Some(20.0));
        let mut r = Rig::auto();
        r.i.battery_low = true;
        r.i.last_waypoint_reached = true;
        let out = r.step();
        assert_eq!((out.state, out.reason), (State::Return, Some(Reason::BattLow)), "LOW beats last-wp");
        let mut r = Rig::hold();
        r.i.battery_low = true;
        let t = r.now() + LINK_LOSS_US + 1;
        assert_eq!((r.at(t).state, r.h.reason()), (State::Return, Some(Reason::BattLow)), "LOW beats link10s");
        for crit in [Rig::hold(), Rig::auto()] {
            let mut r = crit;
            r.i.battery_low = true;
            r.i.battery_critical = true;
            let out = r.step();
            assert_eq!((out.state, out.reason), (State::Land, Some(Reason::BattCrit)));
        }
        let mut r = Rig::hold();
        r.h.track_fault = true;
        r.i.battery_low = true;
        assert_eq!(r.step().reason, Some(Reason::Track));
        let mut r = Rig::hold();
        assert_eq!(r.drop_tier(Tier::T2).state, State::Hold);
        r.i.battery_low = true;
        let out = r.hold_drop();
        assert_eq!((out.state, out.reason), (State::Land, Some(Reason::Tier)));
    }

    /// No home: HOLD with LOW lands.
    #[test]
    fn hands_off_batt_low_without_home_lands() {
        let mut r = Rig::new();
        r.step();
        r.i.armed = true;
        r.i.position_ne_valid = false;
        r.report(T0 + 1_000, Switch3::High);
        r.i.airborne = true;
        r.report(T0 + 2_000, Switch3::High);
        r.i.position_ne_valid = true;
        let t = r.last_report() + LOSS + 1;
        assert_eq!((r.heartbeat(t).state, r.h.home()), (State::Hold, None));
        r.i.battery_low = true;
        let out = r.step();
        assert_eq!((out.state, out.reason), (State::Land, Some(Reason::BattLow)));
    }

    /// Once per arming: the mission switch OFF then ON after it gives HOLD and HOLD stays; Pause from RETURN stays
    /// HOLD; LOW reached in RETURN or LAND uses it too; a new arming fires again.
    #[test]
    fn hands_off_batt_low_fires_once_per_arming() {
        let mut r = Rig::ms_hold();
        r.i.battery_low = true;
        assert_eq!(r.step().state, State::Return);
        let t = r.now() + 1_000;
        assert_eq!(r.report_full(t, FULL, Switch3::High, true, false).state, State::Pilot);
        let t = r.now() + 1_000;
        let out = r.report_ms(t, Switch3::High, true);
        assert_eq!((out.state, out.reason), (State::Hold, Some(Reason::MsOn)));
        for k in 1..20 {
            assert_eq!(r.at(t + k * 10_000).state, State::Hold, "no re-trigger");
        }
        assert_eq!(r.cmd(Command::Start).command, denied(DenyReason::BatteryNotOk));
        assert_eq!(r.cmd(Command::Return).state, State::Return);
        let out = r.cmd(Command::Pause);
        assert_eq!((out.command, out.state), (Some(CommandResult::Accepted), State::Hold));
        assert_eq!(r.step().state, State::Hold, "Pause from RETURN: no bounce");
        let out = r.cmd(Command::Land);
        assert_eq!((out.command, out.state), (Some(CommandResult::Accepted), State::Land));
        let out = r.cmd(Command::Abort);
        assert_eq!((out.command, out.state), (Some(CommandResult::Accepted), State::Hold));
        assert_eq!(r.step().state, State::Hold);
        for first in [State::Return, State::Land] {
            let mut r = if first == State::Return { Rig::ret() } else { Rig::land() };
            r.i.battery_low = true;
            r.i.last_radio_heartbeat_us = Some(r.now());
            assert_eq!(r.step().state, first);
            let out = r.cmd(Command::Abort);
            assert_eq!((out.command, out.state), (Some(CommandResult::Accepted), State::Hold));
            assert_eq!(r.step().state, State::Hold, "LOW reached in {first:?}: used");
        }
        r.i.armed = false;
        r.i.airborne = false;
        assert_eq!(r.step().state, State::Stop);
        r.i.position_ne_m = HOME_NE;
        r.i.position_d_m = HOME_D;
        r.i.armed = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        r.i.airborne = true;
        let t = r.now() + 1_000;
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot, "LOW in PILOT: nothing");
        let t = r.last_report() + LOSS + 1;
        assert_eq!(r.heartbeat(t).state, State::Hold);
        let out = r.step();
        assert_eq!((out.state, out.reason), (State::Return, Some(Reason::BattLow)), "a new arming fires again");
    }

    /// LOW reached in PILOT waits: the mission switch ON gives HOLD and the next step RETURN; on a CRSF flight it
    /// never fires.
    #[test]
    fn hands_off_batt_low_in_pilot_is_deferred_and_never_on_crsf() {
        let mut r = Rig::pilot();
        r.i.battery_low = true;
        let t = r.now() + 1_000;
        r.i.last_radio_heartbeat_us = Some(t);
        assert_eq!(r.report(t, Switch3::High).state, State::Pilot);
        let t = r.now() + 1_000;
        assert_eq!(r.report_ms(t, Switch3::High, true).state, State::Hold);
        let out = r.at(t + 10_000);
        assert_eq!((out.state, out.reason), (State::Return, Some(Reason::BattLow)));
        let mut r = Rig::crsf_pilot();
        r.i.battery_low = true;
        for k in 1..20 {
            assert_eq!(r.report(T0 + 2_000 + k * 10_000, Switch3::High).state, State::Pilot);
        }
        let mut r = Rig::crsf_land();
        r.i.battery_low = true;
        assert_eq!(r.step().state, State::Land);
    }
}
