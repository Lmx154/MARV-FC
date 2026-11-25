//! Gains and limits of the flight code, as constants. Units: rad, rad/s, us.

use fw_core::calib::GRAVITY_MSS;
use fw_core::esc::ANSWER_US;
use marv_card::{K_F, MASS, MAX_ROTOR_SPEED};

/// The collective at full throttle stick, N: `pilot_thrust_max_g` times the weight m g, or four rotors at full speed
/// if that is less. Angle's stick map and altitude hold's thrust clamp use this one value.
pub fn full_thrust_n(pilot_thrust_max_g: f32) -> f32 {
    (pilot_thrust_max_g * (MASS * GRAVITY_MSS)).min(4.0 * K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED)
}

/// Smallest `pilot_speed_xy_ms` the flight code accepts from the parameter `PILOT_SPEED_XY`, m/s: PX4 MPC_VEL_MANUAL's
/// range. The parameter table (`fw_settings::flight`) enforces the same range.
pub const PILOT_SPEED_XY_MIN_MS: f32 = 3.0;

/// Largest `pilot_speed_xy_ms` the flight code accepts, m/s (PX4 MPC_VEL_MANUAL's range, as above).
pub const PILOT_SPEED_XY_MAX_MS: f32 = 20.0;

/// Gains and limits of the flight code. Arrays are [roll, pitch, yaw].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tuning {
    /// Angle P for roll and pitch, 1/s. ArduPilot ATC_ANG_RP_P 4.5.
    pub atc_ang_rp_p: f32,
    /// Angle P for yaw, 1/s. ArduPilot ATC_ANG_YAW_P.
    pub atc_ang_yaw_p: f32,
    /// Rate P, N.m/(rad/s). From the card's inertia, cascade ratio 4 (ATC_RAT_*_P).
    pub atc_rat_p: [f32; 3],
    /// Rate I, N.m/rad. Rate P x 1.0 (roll, pitch) and x 0.1 (yaw), as ArduPilot (ATC_RAT_*_I).
    pub atc_rat_i: [f32; 3],
    /// Rate D, N.m.s/rad (yaw 0). Kd/Kp 0.045 s, placed on the card's motor lag (ATC_RAT_*_D).
    pub atc_rat_d: [f32; 3],
    /// Rate integrator limit, N.m. 0.3 x the torque authority (ATC_RAT_*_IMAX).
    pub atc_rat_imax: [f32; 3],
    /// D-term filter, Hz. ArduPilot's 20 (ATC_RAT_FLTD).
    pub atc_rat_fltd_hz: f32,
    /// Gyro filter, Hz; at least 50 Hz (INS_GYRO_LPF).
    pub ins_gyro_lpf_hz: f32,
    /// Accelerometer PT1 cut-off of the estimator, Hz.
    pub ins_accel_lpf_hz: f32,
    /// Largest tilt the sticks ask for, and the largest lean to arm, rad. ArduPilot's 30 deg (ANGLE_MAX).
    pub angle_max_rad: f32,
    /// Yaw rate at full stick, rad/s. PX4's 150 deg/s (PILOT_Y_RATE).
    pub pilot_y_rate_rads: f32,
    /// Collective at full throttle stick, as a multiple of the weight m g. 2.0 makes the stick one straight line.
    pub pilot_thrust_max_g: f32,
    /// Age of the newest stick report that counts as RC loss, us. ArduPilot's 1.0 s (RC_FS_TIMEOUT).
    pub rc_fs_timeout_us: u64,
    /// Highest throttle stick (0..1) that may arm. Betaflight's 1050 us = 0.05 (ARM_THR_MAX).
    pub arm_thr_max: f32,
    /// Below this multiple of the weight m g the collective cannot lift the vehicle at any point of the thrust
    /// spread, so the vehicle is treated as on the ground and the rate integrators are held at 0. 0.5/0.7 = 0.714:
    /// half of the hover collective at the weakest point of the +/-30 % thrust spread (ArduPilot's hover/2).
    pub ground_thrust_g: f32,
    /// `MahonyBaro`'s vertical filter: covariance propagation period, us.
    pub nav_predict_us: u64,
    /// The barometer's delay added to the sample's age, us: the BMP581's conversion time, x16 pressure 10.4 ms plus
    /// x2 temperature 1.1 ms (the card's baro.latency).
    pub nav_baro_delay_us: u64,
    /// The barometer's height noise, 1 sigma, m: ArduPilot ALT_M_NSE_DEFAULT 2.0 and PX4 ekf2_baro_noise 2.0.
    pub nav_baro_sigma_m: f32,
    /// The innovation gate, in sigmas of the scalar innovation.
    pub nav_gate_sigma: f32,
    /// Time without an accepted barometer fusion after which the vertical flags drop and the height is reset, us.
    pub nav_aid_timeout_us: u64,
    /// Accelerometer velocity random walk, m/s/sqrt(s).
    pub nav_vrw: f32,
    /// Accelerometer bias random walk, m/s^2/sqrt(s).
    pub nav_ba_rw: f32,
    /// Initial vertical velocity sigma, m/s.
    pub nav_p0_vel_ms: f32,
    /// Initial accelerometer bias sigma, m/s^2 (PX4 ekf2_abias_init, px4-common.h:348).
    pub nav_p0_ba_ms2: f32,
    /// Altitude hold: height
    /// error to climb-rate setpoint, 1/s. PX4 MPC_Z_P, ArduPilot's POS P 1.0.
    pub psc_posz_p: f32,
    /// Climb-rate error to vertical acceleration, (m/s^2)/(m/s). PX4 MPC_Z_VEL_P_ACC. PX4's thrust is m (g - a) in
    /// newtons, as here, so the gain transfers unchanged.
    pub psc_velz_p: f32,
    /// Climb-rate error integral to vertical acceleration, (m/s^2)/m. PX4 MPC_Z_VEL_I_ACC.
    pub psc_velz_i: f32,
    /// Limit of the vertical integrator, m/s^2: standard gravity (PX4 PositionControl.cpp:146, +-CONSTANTS_ONE_G).
    pub psc_velz_imax: f32,
    /// Fastest climb the throttle stick asks for, and the largest climb-rate setpoint, m/s. PX4 MPC_Z_VEL_MAX_UP.
    pub pilot_speed_up_ms: f32,
    /// Fastest descent the throttle stick asks for, and the largest descent-rate setpoint, m/s. PX4 MPC_Z_VEL_MAX_DN.
    pub pilot_speed_dn_ms: f32,
    /// Dead zone of the throttle stick in altitude hold, as a share of full travel on each side of the centre 0.5.
    /// ArduPilot THR_DZ, 100 us of 1000 (ArduCopter/config.h:523).
    pub pilot_thr_dz: f32,
    /// Altitude hold: the height target is integrated only while it is within this many seconds of the limit climb or
    /// descent speed from the estimate, s (Betaflight alt_hold_multirotor.c:146).
    pub pilot_alt_lead_s: f32,
    /// Altitude hold: the smallest cos(tilt) used in the 1/cos thrust compensation (ArduPilot
    /// AC_AttitudeControl_Multi.cpp:400).
    pub thr_tilt_cos_min: f32,
    /// Altitude hold: thrust factor clamp(gain x cos(tilt), 0, 1), which takes the thrust to 0 as the vehicle goes past
    /// 90 deg (ArduPilot AC_AttitudeControl_Multi.cpp:398).
    pub thr_inverted_gain: f32,
    /// Position hold: position
    /// error to velocity setpoint, 1/s. PX4 MPC_XY_P (ArduPilot 1.0).
    pub psc_posxy_p: f32,
    /// Velocity error to horizontal acceleration, (m/s^2)/(m/s). PX4 MPC_XY_VEL_P_ACC (ArduPilot 2.0).
    pub psc_velxy_p: f32,
    /// Velocity error integral to horizontal acceleration, (m/s^2)/m. PX4 MPC_XY_VEL_I_ACC (ArduPilot 1.0).
    pub psc_velxy_i: f32,
    /// Fastest horizontal speed the sticks ask for at full deflection, and the largest horizontal velocity setpoint,
    /// m/s. The default of the runtime parameter `PILOT_SPEED_XY` (range 3 to 20): the
    /// flight code takes the value in force at the arming step. ArduPilot POSCONTROL_SPEED_MS (AC_PosControl.h:25).
    pub pilot_speed_xy_ms: f32,
    /// Dead zone of the roll and pitch sticks in position hold, as a share of full travel on each side of the centre.
    /// PX4 MAN_DEADZONE; equals `pilot_thr_dz`.
    pub pilot_xy_dz: f32,
    /// Dead zone of the yaw stick in every mode, as a share of full travel on each side of the centre. ArduPilot
    /// `channel_yaw->set_default_dead_zone(15)` (ArduCopter/radio.cpp:13), 15 us of the 400 us half-range of the default
    /// calibration (RC min 1100, trim 1500, max 1900, `fw_settings::rc`), so 0.0375.
    pub pilot_y_dz: f32,
    /// Position hold: with the sticks centred the position is captured once the ground speed is below this, m/s. PX4
    /// MPC_HOLD_MAX_XY.
    pub pilot_hold_speed_ms: f32,
    /// Position hold's airborne latch: the time the collective must stay below the ground
    /// threshold, armed, before the vehicle counts as landed, us. ArduPilot LAND_DETECTOR_TRIGGER_SEC 1.0 s
    /// (ArduCopter/config.h:359-360; land_detector.cpp:78, :151-156).
    pub land_detect_us: u64,
    /// `Eskf`: the IMU history the delayed horizon
    /// may use, us.
    pub nav_buffer_us: u64,
    /// The delayed horizon: GNSS delay, us. 70 000 for the SAM-M10Q: measured with the receiver's PPS
    /// (edge to the fix's hand-off to the flight code: 59.4-75.5 ms, mean 69.6 ms).
    pub nav_gps_delay_us: u64,
    /// Lowest GNSS epoch rate accepted, Hz: 1000/245, ArduPilot's per-frame bound of 245 ms (`AP_GPS.cpp:849`; its 215 ms is a bound
    /// on the AVERAGE interval, `:861`; a parameter).
    pub nav_gps_min_rate_hz: f32,
    /// Floor of the GNSS position sigma, m.
    pub nav_gps_pos_sigma_min_m: f32,
    /// GNSS velocity sigma, m/s.
    pub nav_gps_vel_sigma_ms: f32,
    /// GNSS vertical velocity sigma of `Eskf`'s VelD option, m/s. ArduPilot copter
    /// VELD_M_NSE_DEFAULT 0.5 (ap-AP_NavEKF3.cpp:24); PX4 max(sAcc, 0.3) x 1.5 = 0.45.
    pub nav_gps_vd_sigma_ms: f32,
    /// Lowest ground speed whose course is used, m/s.
    pub nav_gps_course_min_ms: f32,
    /// No-aid position noise while armed, m (ArduPilot NOAID_M_NSE).
    pub nav_noaid_sigma_m: f32,
    /// No-aid position noise while disarmed, m (ap-AP_NavEKF3_PosVelFusion.cpp:782-784).
    pub nav_noaid_sigma_dis_m: f32,
    /// Initial tilt and yaw sigma, rad (PX4 EKF2_ANGERR_INIT).
    pub nav_p0_tilt_rad: f32,
    /// Initial gyro bias sigma, rad/s (px4-common.h:347).
    pub nav_p0_bg_rads: f32,
    /// Gyro angular random walk, rad/sqrt(s).
    pub nav_arw: f32,
    /// Gyro bias random walk, rad/s/sqrt(s).
    pub nav_bg_rw: f32,
    /// Largest heading sigma for `Valid.heading`, rad: 15 deg, PX4 EKFGSF_yaw_err_max.
    pub nav_heading_valid_sigma_rad: f32,
    /// Compass heading noise, 1 sigma, rad: 12.5 deg.
    pub nav_mag_heading_sigma_rad: f32,
    /// One compass yaw fusion per this period, us.
    pub nav_mag_fuse_us: u64,
    /// Velocity screen: GNSS speed below this share of the position-difference speed.
    pub nav_gps_spd_ratio: f32,
    /// Velocity screen: only at a position-difference speed at or above this, m/s.
    pub nav_gps_spd_min_ms: f32,
    /// Magnetic declination, east positive, rad: 3.195 deg E (WMM2025).
    pub nav_mag_declination_rad: f32,
    /// `EskfSteer` output predictor: time constant of the velocity tracking, s.
    /// PX4 EKF2_TAU_VEL default, `output_predictor.cpp:326`; ArduPilot EK3_TAU_OUTPUT 25 cs.
    pub nav_out_tau_vel_s: f32,
    /// `EskfSteer` output predictor: time constant of the position tracking, s.
    /// PX4 EKF2_TAU_POS, `output_predictor.cpp:327`.
    pub nav_out_tau_pos_s: f32,
    /// `EskfSteer` output predictor: the share of the attitude error taken out per delay time.
    /// PX4 `output_predictor.cpp:311`; ArduPilot `AP_NavEKF3_core.cpp:938`.
    pub nav_out_att_gain: f32,
    /// `EskfSteer` output predictor: gain of the integral term of the velocity and position tracking.
    /// PX4 `output_predictor.cpp:350`, `:354`; ArduPilot `AP_NavEKF3_core.cpp:970-973`.
    pub nav_out_int_gain: f32,
    /// RPM notch: harmonics of each
    /// rotor's frequency that get a notch, at most 3 (the multiple-angle identities stop there). Betaflight
    /// pg/rpm_filter.c:33 (rpm_filter_harmonics 3).
    pub notch_harmonics: usize,
    /// RPM notch: Q of every notch. Betaflight pg/rpm_filter.c:36 (rpm_filter_q 500, shown /100 = 5).
    pub notch_q: f32,
    /// RPM notch: lowest centre, Hz, and the frequency where a harmonic's weight starts to rise from 0.
    /// Chosen so the notch crosses smoothly near the ground idle (61 Hz).
    pub notch_min_hz: f32,
    /// RPM notch: the width of the weight's rise, Hz: full weight from `notch_min_hz` + this.
    pub notch_fade_hz: f32,
    /// RPM notch: highest centre as a share of the sample rate. Betaflight rpm_filter.c:87, :121 (0.48).
    pub notch_max_fs: f32,
    /// RPM notch: a line's eRPM older than this, us, counts as missing: `fw_core::esc::ANSWER_US` (100 ms).
    pub notch_hold_us: u64,
}

impl Tuning {
    /// The default tuning.
    pub const DEFAULT: Tuning = Tuning {
        atc_ang_rp_p: 4.5,
        atc_ang_yaw_p: 1.35,
        atc_rat_p: [0.166, 0.159, 0.0886],
        atc_rat_i: [0.166, 0.159, 0.00886],
        atc_rat_d: [7.46e-3, 7.16e-3, 0.0],
        atc_rat_imax: [0.273, 0.249, 0.0355],
        atc_rat_fltd_hz: 20.0,
        ins_gyro_lpf_hz: 60.0,
        ins_accel_lpf_hz: 10.0,
        angle_max_rad: 30.0_f32.to_radians(),
        pilot_y_rate_rads: 150.0_f32.to_radians(),
        pilot_thrust_max_g: 2.0,
        rc_fs_timeout_us: 1_000_000,
        arm_thr_max: 0.05,
        ground_thrust_g: 0.5 / 0.7,
        nav_predict_us: 10_000,
        nav_baro_delay_us: 11_500,
        nav_baro_sigma_m: 2.0,
        nav_gate_sigma: 5.0,
        nav_aid_timeout_us: 5_000_000,
        nav_vrw: 0.05,
        nav_ba_rw: 3e-3,
        nav_p0_vel_ms: 0.5,
        nav_p0_ba_ms2: 0.2,
        nav_buffer_us: 500_000,
        nav_gps_delay_us: 70_000,
        nav_gps_min_rate_hz: 1000.0 / 245.0,
        nav_gps_pos_sigma_min_m: 0.3,
        nav_gps_vel_sigma_ms: 0.2,
        nav_gps_vd_sigma_ms: 0.5,
        nav_gps_course_min_ms: 0.3,
        nav_noaid_sigma_m: 10.0,
        nav_noaid_sigma_dis_m: 0.5,
        nav_p0_tilt_rad: 0.1,
        nav_p0_bg_rads: 0.1,
        nav_arw: 1e-3,
        nav_bg_rw: 1e-4,
        nav_heading_valid_sigma_rad: 0.262,
        nav_mag_heading_sigma_rad: 0.218,
        nav_mag_fuse_us: 1_000_000,
        nav_gps_spd_ratio: 0.3,
        nav_gps_spd_min_ms: 0.5,
        nav_mag_declination_rad: 3.195_f32.to_radians(),
        psc_posz_p: 1.0,
        psc_velz_p: 4.0,
        psc_velz_i: 2.0,
        psc_velz_imax: GRAVITY_MSS,
        pilot_speed_up_ms: 3.0,
        pilot_speed_dn_ms: 1.5,
        pilot_thr_dz: 0.10,
        pilot_alt_lead_s: 1.0,
        thr_tilt_cos_min: 0.1,
        thr_inverted_gain: 10.0,
        nav_out_tau_vel_s: 0.25,
        nav_out_tau_pos_s: 0.25,
        nav_out_att_gain: 0.5,
        nav_out_int_gain: 0.1,
        notch_harmonics: 3,
        notch_q: 5.0,
        notch_min_hz: 50.0,
        notch_fade_hz: 10.0,
        notch_max_fs: 0.48,
        notch_hold_us: ANSWER_US,
        psc_posxy_p: 0.95,
        psc_velxy_p: 1.8,
        psc_velxy_i: 0.4,
        pilot_speed_xy_ms: 5.0,
        pilot_xy_dz: 0.10,
        pilot_y_dz: 0.0375,
        pilot_hold_speed_ms: 0.8,
        land_detect_us: 1_000_000,
    };

    /// Guidance: the horizontal acceleration of the leg carrot, m/s^2. PX4
    /// MPC_ACC_HOR 3.0 (multicopter_autonomous_params.yaml:82-91; ArduPilot WP_ACC 2.5, online only).
    pub const GUIDANCE_ACC_MS2: f32 = 3.0;
}

/// The number of runtime gain parameters.
pub const GAIN_COUNT: usize = 14;

/// The parameter names of [`Gains`], in the order of its values.
pub const GAIN_NAMES: [&str; GAIN_COUNT] = [
    "ATC_ANG_RP_P",
    "ATC_ANG_YAW_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_PIT_P",
    "ATC_RAT_YAW_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_PIT_I",
    "ATC_RAT_YAW_I",
    "ATC_RAT_RLL_D",
    "ATC_RAT_PIT_D",
    "ATC_RAT_RLL_IMAX",
    "ATC_RAT_PIT_IMAX",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_FLTD",
];

/// The 14 gains that are runtime parameters, in the order of [`GAIN_NAMES`]. Physical units
/// as in [`Tuning`]. The yaw D stays the compiled 0 and is not here. The defaults are `Gains::of(&Tuning::DEFAULT)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains(pub [f32; GAIN_COUNT]);

impl Gains {
    /// The lower bounds: default / 3 rounded outward to 3 significant figures (only refuses a downward
    /// slip).
    pub const MIN: Gains = Gains([
        1.5,     // ATC_ANG_RP_P: R
        0.45,    // ATC_ANG_YAW_P: R
        0.0553,  // ATC_RAT_RLL_P: R
        0.0530,  // ATC_RAT_PIT_P: R
        0.0295,  // ATC_RAT_YAW_P: R
        0.0553,  // ATC_RAT_RLL_I: R
        0.0530,  // ATC_RAT_PIT_I: R
        0.00295, // ATC_RAT_YAW_I: R
        0.00248, // ATC_RAT_RLL_D: R
        0.00238, // ATC_RAT_PIT_D: R
        0.0910,  // ATC_RAT_RLL_IMAX: R
        0.0830,  // ATC_RAT_PIT_IMAX: R
        0.0118,  // ATC_RAT_YAW_IMAX: R
        6.66,    // ATC_RAT_FLTD: R
    ]);

    /// The upper bounds: default x 3 rounded outward to 3 significant figures, except ATC_ANG_RP_P.
    pub const MAX: Gains = Gains([
        12.0,   // ATC_ANG_RP_P: the exception: ArduPilot 3-12, PX4 max 12; x3 = 13.5 exceeds both
        4.05,   // ATC_ANG_YAW_P: R
        0.498,  // ATC_RAT_RLL_P: R
        0.477,  // ATC_RAT_PIT_P: R
        0.266,  // ATC_RAT_YAW_P: R
        0.498,  // ATC_RAT_RLL_I: R
        0.477,  // ATC_RAT_PIT_I: R
        0.0266, // ATC_RAT_YAW_I: R
        0.0224, // ATC_RAT_RLL_D: R
        0.0215, // ATC_RAT_PIT_D: R
        0.819,  // ATC_RAT_RLL_IMAX: R, below the 0.911 N.m authority
        0.747,  // ATC_RAT_PIT_IMAX: R, below 0.831 N.m
        0.107,  // ATC_RAT_YAW_IMAX: R, below 0.118 N.m
        60.0,   // ATC_RAT_FLTD: R, equals the gyro PT1
    ]);

    /// The 14 gains of `tuning`.
    pub const fn of(tuning: &Tuning) -> Gains {
        let [p0, p1, p2] = tuning.atc_rat_p;
        let [i0, i1, i2] = tuning.atc_rat_i;
        let [d0, d1, _] = tuning.atc_rat_d;
        let [m0, m1, m2] = tuning.atc_rat_imax;
        Gains([
            tuning.atc_ang_rp_p,
            tuning.atc_ang_yaw_p,
            p0,
            p1,
            p2,
            i0,
            i1,
            i2,
            d0,
            d1,
            m0,
            m1,
            m2,
            tuning.atc_rat_fltd_hz,
        ])
    }

    /// True if every value is finite and within [`Gains::MIN`] to [`Gains::MAX`], ends included.
    pub fn in_range(&self) -> bool {
        self.0
            .iter()
            .zip(Self::MIN.0.iter().zip(Self::MAX.0.iter()))
            .all(|(v, (lo, hi))| (*lo..=*hi).contains(v))
    }
}

impl Tuning {
    /// This tuning with the 14 gains of `gains`; every other field, and the yaw D, unchanged.
    pub fn with_gains(self, gains: Gains) -> Tuning {
        let Gains([rp, yaw, p0, p1, p2, i0, i1, i2, d0, d1, m0, m1, m2, fltd]) = gains;
        let [_, _, d2] = self.atc_rat_d;
        Tuning {
            atc_ang_rp_p: rp,
            atc_ang_yaw_p: yaw,
            atc_rat_p: [p0, p1, p2],
            atc_rat_i: [i0, i1, i2],
            atc_rat_d: [d0, d1, d2],
            atc_rat_imax: [m0, m1, m2],
            atc_rat_fltd_hz: fltd,
            ..self
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::string::ToString;

    /// The 14 values of `t` read field by field, in the order of `GAIN_NAMES`, written out here and not by `Gains::of`.
    fn named(t: &Tuning) -> [f32; GAIN_COUNT] {
        [
            t.atc_ang_rp_p,
            t.atc_ang_yaw_p,
            t.atc_rat_p[0],
            t.atc_rat_p[1],
            t.atc_rat_p[2],
            t.atc_rat_i[0],
            t.atc_rat_i[1],
            t.atc_rat_i[2],
            t.atc_rat_d[0],
            t.atc_rat_d[1],
            t.atc_rat_imax[0],
            t.atc_rat_imax[1],
            t.atc_rat_imax[2],
            t.atc_rat_fltd_hz,
        ]
    }

    /// `t` with the field that parameter `i` of `GAIN_NAMES` names set to `v`, by an explicit assignment.
    fn with_field(mut t: Tuning, i: usize, v: f32) -> Tuning {
        match GAIN_NAMES[i] {
            "ATC_ANG_RP_P" => t.atc_ang_rp_p = v,
            "ATC_ANG_YAW_P" => t.atc_ang_yaw_p = v,
            "ATC_RAT_RLL_P" => t.atc_rat_p[0] = v,
            "ATC_RAT_PIT_P" => t.atc_rat_p[1] = v,
            "ATC_RAT_YAW_P" => t.atc_rat_p[2] = v,
            "ATC_RAT_RLL_I" => t.atc_rat_i[0] = v,
            "ATC_RAT_PIT_I" => t.atc_rat_i[1] = v,
            "ATC_RAT_YAW_I" => t.atc_rat_i[2] = v,
            "ATC_RAT_RLL_D" => t.atc_rat_d[0] = v,
            "ATC_RAT_PIT_D" => t.atc_rat_d[1] = v,
            "ATC_RAT_RLL_IMAX" => t.atc_rat_imax[0] = v,
            "ATC_RAT_PIT_IMAX" => t.atc_rat_imax[1] = v,
            "ATC_RAT_YAW_IMAX" => t.atc_rat_imax[2] = v,
            "ATC_RAT_FLTD" => t.atc_rat_fltd_hz = v,
            other => panic!("{other}"),
        }
        t
    }

    #[test]
    fn the_names_are_the_table_and_distinct() {
        assert_eq!(GAIN_NAMES.len(), 14);
        for (i, a) in GAIN_NAMES.iter().enumerate() {
            assert!(GAIN_NAMES[..i].iter().all(|b| a != b), "{a}");
        }
        assert_eq!(GAIN_NAMES[0], "ATC_ANG_RP_P");
        assert_eq!(GAIN_NAMES[13], "ATC_RAT_FLTD");
    }

    #[test]
    fn the_defaults_round_trip_through_gains_bit_for_bit() {
        let d = Tuning::DEFAULT;
        let back = d.with_gains(Gains::of(&d));
        assert_eq!(back, d);
        assert_eq!(std::format!("{back:?}"), std::format!("{d:?}"));
        // Yaw D is not a gain: any tuning keeps its own.
        let mut other = d;
        other.atc_rat_d[2] = 0.125;
        assert_eq!(other.with_gains(Gains::of(&d)).atc_rat_d[2].to_bits(), 0.125f32.to_bits());
        // `of` reads what `named` reads.
        assert_eq!(Gains::of(&d).0.map(f32::to_bits), named(&d).map(f32::to_bits));
    }

    #[test]
    fn changing_gain_i_changes_exactly_its_named_field() {
        let d = Tuning::DEFAULT;
        let base = Gains::of(&d);
        for i in 0..GAIN_COUNT {
            // A value that is none of the other fields' defaults.
            let marker = 1234.5 + i as f32;
            let mut g = base;
            g.0[i] = marker;
            let t = d.with_gains(g);
            let want = with_field(d, i, marker);
            assert_eq!(std::format!("{t:?}"), std::format!("{want:?}"), "{}", GAIN_NAMES[i]);
            for (j, v) in named(&t).into_iter().enumerate() {
                let expect = if i == j { marker } else { named(&d)[j] };
                assert_eq!(v.to_bits(), expect.to_bits(), "{} set, {} read", GAIN_NAMES[i], GAIN_NAMES[j]);
            }
            assert_eq!(Gains::of(&t), g, "{}", GAIN_NAMES[i]);
        }
    }

    #[test]
    fn the_defaults_are_inside_the_bounds_and_the_ends_are_accepted() {
        let d = Gains::of(&Tuning::DEFAULT);
        assert!(d.in_range());
        assert!(Gains::MIN.in_range() && Gains::MAX.in_range());
        for i in 0..GAIN_COUNT {
            let name = GAIN_NAMES[i];
            assert!(Gains::MIN.0[i] < d.0[i] && d.0[i] < Gains::MAX.0[i], "{name}");
            let below = f32::from_bits(Gains::MIN.0[i].to_bits() - 1);
            let above = f32::from_bits(Gains::MAX.0[i].to_bits() + 1);
            for (v, ok) in [
                (Gains::MIN.0[i], true),
                (Gains::MAX.0[i], true),
                (below, false),
                (above, false),
                (f32::NAN, false),
                (f32::INFINITY, false),
                (f32::NEG_INFINITY, false),
                (0.0, false),
                (-d.0[i], false),
            ] {
                let mut g = d;
                g.0[i] = v;
                assert_eq!(g.in_range(), ok, "{name} = {v}");
            }
        }
    }

    /// The text path the CLI and the parameter files use: f32 to text, text to f64, f64 to f32.
    #[test]
    fn the_defaults_and_the_bounds_survive_the_text_round_trip() {
        let sets = [("default", Gains::of(&Tuning::DEFAULT)), ("min", Gains::MIN), ("max", Gains::MAX)];
        for (which, g) in sets {
            for (name, d) in GAIN_NAMES.iter().zip(g.0) {
                let back = d.to_string().parse::<f64>().unwrap() as f32;
                assert_eq!(back.to_bits(), d.to_bits(), "{which} {name}: {d}");
            }
        }
    }
}
