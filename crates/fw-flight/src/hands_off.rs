//! The hands-off state machine (decision-mavlink-mode-20261123, section 9 Amendment A as amended by section 10: A1,
//! A2', A3, A6, A7, A9, A10, A10' and the state table; and by section 11 Amendment B, B1-B9). Pure: no clock, no log, no statics, no allocation; every time
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
    /// A command into HOLD (and Abort in BLIND): the debounced tier is not T3' (B2, B6). Start: this step's tier is
    /// not T3 (checked after `HeadingNotOk`, B4).
    NotT3,
    /// A command into HOLD: the battery is critical, so HOLD would land at once (B2).
    BatteryCritical,
    /// A command into HOLD: the link is lost (`LINK_LOSS_US`), so HOLD would leave at once (B2).
    LinkLost,
    /// Start: no station HEARTBEAT on the radio link within `STATION_LOST_US`.
    StationAbsent,
    /// Start: no list with a waypoint is stored.
    NoWaypoints,
    /// Start: a stored z is outside the height limits in force.
    HeightOutsideLimits,
    /// Start: home is not set.
    NoHome,
    /// Start: the vehicle is more than `HOME_RANGE_M` from home, or its position is not valid.
    VehicleBeyondRange,
    /// Start: the waypoints' largest distance from home is unknown or not finite.
    WaypointDistanceUnknown,
    /// Start: a waypoint is more than `HOME_RANGE_M` from home.
    WaypointBeyondRange,
    /// Start: the heading monitor is not OK.
    HeadingNotOk,
    /// Start: the battery is not known to be non-critical.
    BatteryNotOk,
    /// Start: the last CRSF report before the sticks were lost did not have switch6 High (A10').
    SwitchNotHigh,
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
    /// LAND: the existing landing latch disarms at touchdown.
    pub landing: bool,
    /// LAND on this step's T3': hold the horizontal position; otherwise LAND descends level (B4).
    pub land_holds_position: bool,
    /// The highest level the flight code may demand this step, from this step's flags (B5, I12).
    pub demand: Demand,
}

/// The hands-off state machine.
#[derive(Clone, Debug)]
pub struct HandsOff {
    state: State,
    /// Stick loss, us: the gate's RC_FS_TIMEOUT (`Tuning::rc_fs_timeout_us`, decision Numbers "hands-off entry 1.0 s").
    stick_loss_us: u64,
    /// A2': set when the pilot flies; cleared on entry to any hands-off state.
    pilot_latch: bool,
    /// A2': in TAKEOVER, a fresh report has shown switch6 High.
    seen_high: bool,
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
}

impl HandsOff {
    /// STOP, disarmed, no home. The stick-loss time is the gate's (`tuning.rc_fs_timeout_us`).
    pub fn new(tuning: &Tuning) -> Self {
        Self {
            state: State::Stop,
            stick_loss_us: tuning.rc_fs_timeout_us,
            pilot_latch: true,
            seen_high: false,
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

    /// One flight step.
    pub fn step(&mut self, i: &Inputs) -> Output {
        let fresh = i
            .crsf
            .is_some_and(|r| i.now_us.saturating_sub(r.t_us) <= self.stick_loss_us);
        let mut ev = Events::NONE;
        self.update_home(i);
        if !i.armed {
            self.fs_in_force = i.fs_hold_en;
        }
        let instant = Self::instant_tier(i);
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
        Output {
            state: self.state,
            hands_off,
            synthetic_sticks: hands_off,
            command,
            home: self.home(),
            return_height_m: self.return_height_m,
            events: ev,
            landing: self.state == State::Land,
            land_holds_position: self.state == State::Land && demand == Demand::Position,
            demand,
        }
    }

    /// This step's tier with B4's T3' (T3 needs `heading_ok`).
    fn instant_tier(i: &Inputs) -> Tier {
        match i.tier {
            Tier::T3 if i.heading_ok => Tier::T3,
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
            if self.home_window {
                if i.airborne {
                    self.home_window = false;
                } else {
                    if self.home_d.is_none() && i.position_d_valid {
                        self.home_d = Some(i.position_d_m);
                    }
                    if self.home_ne.is_none() && i.position_ne_valid {
                        self.home_ne = Some(i.position_ne_m);
                    }
                }
            }
        }
        self.was_armed = i.armed;
    }

    /// The state table's exits that are not commands (FS_HOLD_EN 1).
    fn automatic(&mut self, i: &Inputs, fresh: bool, ev: &mut Events) {
        if !i.armed || i.crsf.is_some_and(|r| !r.arm) {
            self.go(State::Stop, i, ev);
            return;
        }
        let high = i.crsf.is_some_and(|r| r.switch6 == Switch3::High);
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
                    if high {
                        self.seen_high = true;
                    } else if self.seen_high {
                        self.pilot_latch = true;
                    }
                    if self.pilot_latch {
                        self.go(State::Pilot, i, ev);
                    }
                }
            }
            State::Hold | State::Auto | State::Return | State::Land | State::Blind if fresh => {
                self.go(State::Takeover, i, ev);
                self.seen_high = high;
            }
            State::Hold => {
                if self.debounced != Tier::T3 || i.battery_critical {
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
                if self.debounced != Tier::T3 || i.battery_critical {
                    self.go(self.land_state(), i, ev);
                } else if i.mission.plan_id != self.flown_plan || i.last_waypoint_reached {
                    self.go(State::Hold, i, ev);
                } else if !Self::station_present(i) {
                    self.pause(i, ev);
                }
            }
            State::Return => {
                if self.debounced != Tier::T3 || i.battery_critical || i.home_reached {
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
    }

    /// Sticks lost in PILOT or TAKEOVER: by the debounced tier when armed and airborne, else STOP.
    fn lost(&self, i: &Inputs) -> State {
        if !i.airborne {
            State::Stop
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
            self.seen_high = false;
        }
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

    /// A10, A10', A9 and the lead's rulings: the answer, and the change at this step when ACCEPTED.
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
        if !i.heading_ok {
            return Some(DenyReason::HeadingNotOk);
        }
        if i.tier != Tier::T3 {
            return Some(DenyReason::NotT3);
        }
        if !i.battery_ok {
            return Some(DenyReason::BatteryNotOk);
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
                position_d_m: HOME_D,
                position_d_valid: true,
                command: None,
                mission: mission(),
                last_waypoint_reached: false,
                home_reached: false,
                heading_ok: true,
                battery_ok: true,
                battery_critical: false,
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
        for _ in 0..200_000 {
            random_inputs(&mut rng, &mut i, true);
            let out = h.step(&i);
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

    /// HOLD with the station present, the vehicle `vehicle_m` north of home, the farthest waypoint at `waypoint_m`.
    fn start_at(vehicle_m: f32, waypoint_m: f32) -> Output {
        let mut r = Rig::hold();
        r.i.position_ne_m = [HOME_NE[0] + vehicle_m, HOME_NE[1]];
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
            (DenyReason::VehicleBeyondRange, |r| r.i.position_ne_m = [2000.0, 0.0]),
            (DenyReason::VehicleBeyondRange, |r| r.i.position_ne_valid = false),
            (DenyReason::WaypointDistanceUnknown, |r| r.i.mission.max_waypoint_distance_m = None),
            (DenyReason::WaypointDistanceUnknown, |r| r.i.mission.max_waypoint_distance_m = Some(f32::NAN)),
            (DenyReason::WaypointDistanceUnknown, |r| {
                r.i.mission.max_waypoint_distance_m = Some(f32::INFINITY)
            }),
            (DenyReason::WaypointBeyondRange, |r| r.i.mission.max_waypoint_distance_m = Some(1000.5)),
            (DenyReason::VehicleBeyondRange, |r| r.i.position_ne_m = [f32::NAN, 0.0]),
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
            assert_eq!(out.return_height_m, None);
            assert_eq!(out.events, Events::NONE);
            assert!(matches!(out.command, None | Some(CommandResult::Denied(_))));
        }
        assert!(pilot > 1_000 && stop > 1_000, "both reached: {pilot} {stop}");
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
                command: rng.pick(&[Command::Start, Command::Pause, Command::Abort, Command::Land]),
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
}
