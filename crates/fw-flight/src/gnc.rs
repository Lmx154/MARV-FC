//! The three seams of the flight code and their types: [`Estimator`], [`Controller`], [`Allocator`]
//! (decision-sim-seams-20261119, sections 3 to 5). Frames, units and signs: decision-sim-conventions-20261119.

use crate::imu::ImuSample;
use fw_core::esc::LINES;
use fw_core::sample::{BaroSample, GnssSample, MagSample};

/// What is new since the previous step, for the estimator. `imu1` is always present: its arrival makes the step and
/// its `t_us` is the step time. Each `Option` is `Some` only for a sample newer than the one given at the previous
/// step. If more than one arrived, only the newest is given, and the gap shows in `seq`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sensors {
    /// IMU1 (ICM-45686), the primary.
    pub imu1: ImuSample,
    /// IMU2 (LSM6DSK320X), the newest since the last step; it is not phase-locked to IMU1.
    pub imu2: Option<ImuSample>,
    /// Barometer.
    pub baro: Option<BaroSample>,
    /// Compass.
    pub mag: Option<MagSample>,
    /// GNSS.
    pub gnss: Option<GnssSample>,
    /// Reserved for the companion computer's pose and velocity (SLAM). Always `None`; nothing builds it.
    pub external: Option<ExternalNavSample>,
}

/// Reserved, not built: one external measurement, valid at board time `t_us`, in the flight code's NED frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExternalNavSample {
    /// Board time, us.
    pub t_us: u64,
    /// NED, m, from the origin; NED, m/s; body to NED [w, x, y, z], w >= 0.
    pub position_m: [f32; 3],
    /// NED, m/s.
    pub velocity_ms: [f32; 3],
    /// Body to NED [w, x, y, z], w >= 0.
    pub attitude: [f32; 4],
}

/// Which `NavState` fields may be used. A field whose flag is false holds a finite value that means nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Valid {
    /// Roll and pitch of `attitude`.
    pub tilt: bool,
    /// The yaw of `attitude` is referenced to true north (false: relative yaw, as Mahony without a compass).
    pub heading: bool,
    /// `rates_rads`.
    pub rates: bool,
    /// North and east of `velocity_ms`.
    pub velocity_ne: bool,
    /// Down of `velocity_ms`.
    pub velocity_d: bool,
    /// North and east of `position_m`.
    pub position_ne: bool,
    /// Down from the origin's altitude. A barometer-only height sets this flag alone.
    pub position_d: bool,
}

impl Valid {
    /// No field may be used.
    pub const NONE: Valid = Valid {
        tilt: false,
        heading: false,
        rates: false,
        velocity_ne: false,
        velocity_d: false,
        position_ne: false,
        position_d: false,
    };

    /// True when every flag set in `need` is also set in `self`.
    pub fn covers(&self, need: &Valid) -> bool {
        (self.tilt || !need.tilt)
            && (self.heading || !need.heading)
            && (self.rates || !need.rates)
            && (self.velocity_ne || !need.velocity_ne)
            && (self.velocity_d || !need.velocity_d)
            && (self.position_ne || !need.position_ne)
            && (self.position_d || !need.position_d)
    }
}

/// One standard deviation per validity group (PX4's eph, epv, evh, evv, plus tilt and heading). The full covariance
/// stays inside the estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavSigma {
    /// Tilt, rad.
    pub tilt_rad: f32,
    /// Heading, rad.
    pub heading_rad: f32,
    /// North-east velocity, m/s.
    pub velocity_ne_ms: f32,
    /// Down velocity, m/s.
    pub velocity_d_ms: f32,
    /// North-east position, m.
    pub position_ne_m: f32,
    /// Down position, m.
    pub position_d_m: f32,
}

/// The estimator's state for one step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavState {
    /// Board time the estimate refers to, us: the step time.
    pub t_us: u64,
    /// Body to NED, Hamilton, [w, x, y, z], unit length, w >= 0.
    pub attitude: [f32; 4],
    /// Body rates, rad/s, FRD, bias removed, filtered as the estimator chooses. The rate loop uses these.
    pub rates_rads: [f32; 3],
    /// NED, m/s.
    pub velocity_ms: [f32; 3],
    /// NED, m, from the origin (`Estimator::origin`).
    pub position_m: [f32; 3],
    /// Which fields may be used.
    pub valid: Valid,
    /// `None` for an estimator that has no covariance (Mahony).
    pub sigma: Option<NavSigma>,
}

/// A geodetic point in GNSS units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeoPoint {
    /// 1e-7 degree, south negative.
    pub lat_e7: i32,
    /// 1e-7 degree, west negative.
    pub lon_e7: i32,
    /// Above mean sea level, mm.
    pub alt_mm: i32,
}

/// The estimator seam.
pub trait Estimator {
    /// Takes this step's new samples, each by its own `t_us`, and returns the state at `sensors.imu1.t_us`.
    fn update(&mut self, sensors: &Sensors) -> NavState;
    /// The NED origin once latched (rule: step 7). `None` before that, and always for attitude-only estimators.
    fn origin(&self) -> Option<GeoPoint> {
        None
    }
    /// The gate's armed state after the previous step; `Flight::step` calls it once, before `update`.
    fn set_armed(&mut self, _armed: bool) {}
}

/// The flight mode the pilot selected. It tells the stick shaper which `Setpoint` to build. It is not a controller
/// input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    /// Body rates.
    Acro,
    /// Attitude.
    Angle,
    /// Altitude hold.
    Altitude,
    /// Position hold.
    Position,
}

/// What the controller tracks this step. Thrust: collective, N, positive up (conventions 7d). Heading and heading rate
/// are about NED down, positive nose right (conventions 7b, 7c).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Setpoint {
    /// Acro. Body rates, FRD, rad/s.
    Rates {
        /// Body rates, FRD, rad/s.
        rates_rads: [f32; 3],
        /// Collective thrust, N.
        thrust_n: f32,
    },
    /// Angle. Attitude body to NED [w, x, y, z], plus a heading-rate feed-forward, rad/s.
    Attitude {
        /// Body to NED [w, x, y, z].
        attitude: [f32; 4],
        /// Heading-rate feed-forward, rad/s.
        heading_rate_rads: f32,
        /// Collective thrust, N.
        thrust_n: f32,
    },
    /// Altitude hold. Tilt and heading come from `attitude`; height is given as down from the origin, m, with its
    /// rate, m/s.
    Altitude {
        /// Body to NED [w, x, y, z].
        attitude: [f32; 4],
        /// Heading-rate feed-forward, rad/s.
        heading_rate_rads: f32,
        /// Down from the origin, m.
        down_m: f32,
        /// Down velocity, m/s.
        velocity_d_ms: f32,
    },
    /// Position hold. NED position, m, with a NED velocity feed-forward, m/s; heading, rad.
    Position {
        /// NED, m.
        position_m: [f32; 3],
        /// NED velocity feed-forward, m/s.
        velocity_ms: [f32; 3],
        /// Heading, rad.
        heading_rad: f32,
        /// Heading-rate feed-forward, rad/s.
        heading_rate_rads: f32,
    },
    /// Declared, not built: every controller refuses it.
    Waypoint {
        /// NED, m.
        position_m: [f32; 3],
        /// Heading, rad.
        heading_rad: f32,
    },
}

impl Setpoint {
    /// The fields each variant needs. Rates: rates. Attitude: rates and tilt. Altitude: those plus velocity_d and
    /// position_d. Position and Waypoint: those plus heading, velocity_ne and position_ne.
    pub fn requires(&self) -> Valid {
        let rates = Valid {
            rates: true,
            ..Valid::NONE
        };
        let attitude = Valid {
            tilt: true,
            ..rates
        };
        let altitude = Valid {
            velocity_d: true,
            position_d: true,
            ..attitude
        };
        match self {
            Setpoint::Rates { .. } => rates,
            Setpoint::Attitude { .. } => attitude,
            Setpoint::Altitude { .. } => altitude,
            Setpoint::Position { .. } | Setpoint::Waypoint { .. } => Valid {
                heading: true,
                velocity_ne: true,
                position_ne: true,
                ..altitude
            },
        }
    }
}

/// The controller's demand. Collective thrust, N, >= 0, along body -z. Body torques, N.m, about the FRD axes: +x rolls
/// the right side down, +y pitches the nose up, +z yaws the nose right (conventions 7b).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustTorque {
    /// Collective thrust, N, >= 0.
    pub thrust_n: f32,
    /// Body torques, N.m, FRD.
    pub torque_nm: [f32; 3],
}

/// Which demands the previous allocation could not meet (ArduPilot `AP_Motors_limit`'s five flags). The controller
/// uses them for anti-windup.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Saturation {
    /// Roll torque.
    pub roll: bool,
    /// Pitch torque.
    pub pitch: bool,
    /// Yaw torque.
    pub yaw: bool,
    /// Thrust at its lower limit.
    pub thrust_low: bool,
    /// Thrust at its upper limit.
    pub thrust_high: bool,
}

/// The controller does not track this setpoint with this nav state. It never returns a guess instead.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Refused;

/// The controller seam.
pub trait Controller {
    /// One step. `limited` is the previous step's `Allocation::saturation`. dt is the difference between successive
    /// `nav.t_us` values, never a constant.
    fn update(
        &mut self,
        nav: &NavState,
        sp: &Setpoint,
        limited: &Saturation,
    ) -> Result<ThrustTorque, Refused>;
    /// Clears integrators and filter memory. The step calls it on every step while the vehicle is disarmed.
    fn reset(&mut self);
}

/// Four DShot levels, 1 to 2000, indexed by DShot line: 0 front right, 1 rear right, 2 front left, 3 rear left
/// (conventions 5). The lowest level the allocator emits is idle (the card's `idle_level`; value: owner, then the
/// thrust test). The allocator never asks for Stop; only the gate does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorLevels {
    /// By DShot line.
    pub level: [u16; LINES],
}

/// The allocator's output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Allocation {
    /// By DShot line.
    pub levels: MotorLevels,
    /// What could not be met.
    pub saturation: Saturation,
}

/// The allocator seam.
pub trait Allocator {
    /// Owns saturation priority and the thrust-to-level map.
    fn allocate(&mut self, demand: &ThrustTorque) -> Allocation;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requires_per_variant_and_covers() {
        let want = |tilt, heading, vne, vd, pne, pd| Valid {
            tilt,
            heading,
            rates: true,
            velocity_ne: vne,
            velocity_d: vd,
            position_ne: pne,
            position_d: pd,
        };
        let (q, v) = ([1.0, 0.0, 0.0, 0.0], [0.0; 3]);
        let rates = Setpoint::Rates {
            rates_rads: v,
            thrust_n: 0.0,
        };
        let attitude = Setpoint::Attitude {
            attitude: q,
            heading_rate_rads: 0.0,
            thrust_n: 0.0,
        };
        let altitude = Setpoint::Altitude {
            attitude: q,
            heading_rate_rads: 0.0,
            down_m: 0.0,
            velocity_d_ms: 0.0,
        };
        let position = Setpoint::Position {
            position_m: v,
            velocity_ms: v,
            heading_rad: 0.0,
            heading_rate_rads: 0.0,
        };
        let waypoint = Setpoint::Waypoint {
            position_m: v,
            heading_rad: 0.0,
        };
        let all = want(true, true, true, true, true, true);
        assert_eq!(
            rates.requires(),
            want(false, false, false, false, false, false)
        );
        assert_eq!(
            attitude.requires(),
            want(true, false, false, false, false, false)
        );
        assert_eq!(
            altitude.requires(),
            want(true, false, false, true, false, true)
        );
        assert_eq!(position.requires(), all);
        assert_eq!(waypoint.requires(), all);

        assert!(Valid::NONE.covers(&Valid::NONE));
        assert!(all.covers(&altitude.requires()));
        assert!(attitude.requires().covers(&rates.requires()));
        assert!(!Valid::NONE.covers(&rates.requires()));
        assert!(!altitude.requires().covers(&position.requires()));
        // Baro-only height: position_d alone does not cover Altitude.
        let baro = Valid {
            position_d: true,
            ..attitude.requires()
        };
        assert!(!baro.covers(&altitude.requires()));
    }
}
