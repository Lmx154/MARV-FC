//! Gains and limits of the flight code, as constants until the parameters come with the thrust test
//! (step 6). Fields: decision-first-pair-20261119, section 9; values and their sources: its section 13
//! (rulings file, ruling 3). Units: rad, rad/s, us.

use fw_core::calib::GRAVITY_MSS;
use marv_card::{K_F, MASS, MAX_ROTOR_SPEED};

/// The collective at full throttle stick, N: `pilot_thrust_max_g` times the weight m g, or four rotors at full speed
/// if that is less. Angle's stick map and altitude hold's thrust clamp use this one value.
pub fn full_thrust_n(pilot_thrust_max_g: f32) -> f32 {
    (pilot_thrust_max_g * (MASS * GRAVITY_MSS)).min(4.0 * K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED)
}

/// Smallest `pilot_speed_xy_ms` the flight code accepts from the parameter `PILOT_SPEED_XY`, m/s: PX4 MPC_VEL_MANUAL's
/// range (Amendment P, P9, owner's answer 2026-11-22). The parameter table (`fw_settings::flight`) enforces the same range.
pub const PILOT_SPEED_XY_MIN_MS: f32 = 3.0;

/// Largest `pilot_speed_xy_ms` the flight code accepts, m/s (PX4 MPC_VEL_MANUAL's range, as above).
pub const PILOT_SPEED_XY_MAX_MS: f32 = 20.0;

/// Gains and limits of the flight code. Arrays are [roll, pitch, yaw].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tuning {
    /// Angle P for roll and pitch, 1/s. ArduPilot's 4.5 (section 13, ATC_ANG_RP_P).
    pub atc_ang_rp_p: f32,
    /// Angle P for yaw, 1/s. Derived in section 3e (section 13, ATC_ANG_YAW_P).
    pub atc_ang_yaw_p: f32,
    /// Rate P, N.m/(rad/s). Derived from the card's inertia, cascade ratio 4 (section 13, ATC_RAT_*_P).
    pub atc_rat_p: [f32; 3],
    /// Rate I, N.m/rad. Rate P x 1.0 (roll, pitch) and x 0.1 (yaw), as ArduPilot (section 13, ATC_RAT_*_I).
    pub atc_rat_i: [f32; 3],
    /// Rate D, N.m.s/rad (yaw 0). Kd/Kp 0.045 s, placed on the card's motor lag (section 13, ATC_RAT_*_D).
    pub atc_rat_d: [f32; 3],
    /// Rate integrator limit, N.m. 0.3 x the torque authority (section 13, ATC_RAT_*_IMAX).
    pub atc_rat_imax: [f32; 3],
    /// D-term filter, Hz. ArduPilot's 20 (section 13, ATC_RAT_FLTD).
    pub atc_rat_fltd_hz: f32,
    /// Gyro filter, Hz. Bound at 50 Hz or more in section 3g (section 13, INS_GYRO_LPF).
    pub ins_gyro_lpf_hz: f32,
    /// Accelerometer PT1 cut-off of the estimator, Hz.
    pub ins_accel_lpf_hz: f32,
    /// Largest tilt the sticks ask for, and the largest lean to arm, rad. ArduPilot's 30 deg (section 13, ANGLE_MAX).
    pub angle_max_rad: f32,
    /// Yaw rate at full stick, rad/s. PX4's 150 deg/s (section 13, PILOT_Y_RATE).
    pub pilot_y_rate_rads: f32,
    /// Collective at full throttle stick, as a multiple of the weight m g. 2.0 makes the stick one straight line
    /// (first-pair rulings, Amendment C; the owner, 2026-11-19).
    pub pilot_thrust_max_g: f32,
    /// Age of the newest stick report that counts as RC loss, us. ArduPilot's 1.0 s (section 13, RC_FS_TIMEOUT).
    pub rc_fs_timeout_us: u64,
    /// Highest throttle stick (0..1) that may arm. Betaflight's 1050 us = 0.05 (section 13, ARM_THR_MAX).
    pub arm_thr_max: f32,
    /// Below this multiple of the weight m g the collective cannot lift the vehicle at any point of the thrust
    /// spread, so the vehicle is treated as on the ground and the rate integrators are held at 0. 0.5/0.7 = 0.714:
    /// half of the hover collective at the weakest point of the +/-30 % thrust spread
    /// (decision-flight-ground-20261119, section 3: 0.5/0.7, ArduPilot's hover/2 at the weakest spread point; PROPOSED).
    pub ground_thrust_g: f32,
    /// `MahonyBaro`'s vertical filter: covariance propagation period, us (decision-nav-eskf-20261120, B3, approved
    /// 2026-11-21).
    pub nav_predict_us: u64,
    /// The barometer's delay added to the sample's age, us: the BMP581's conversion time, x16 pressure 10.4 ms plus
    /// x2 temperature 1.1 ms (B3; the card's baro.latency).
    pub nav_baro_delay_us: u64,
    /// The barometer's height noise, 1 sigma, m (B3). 2.0, owner 2026-11-22: ArduPilot ALT_M_NSE_DEFAULT 2.0 and PX4
    /// ekf2_baro_noise 2.0; was 0.5 (rest rms), which made the vertical follow the in-flight baro error (0.09-0.57 m,
    /// 0.08-0.5 Hz, rising with tilt and speed) of log 281 into a height oscillation (flights/20261122-eskf3-flight2).
    pub nav_baro_sigma_m: f32,
    /// The innovation gate, in sigmas of the scalar innovation (B3).
    pub nav_gate_sigma: f32,
    /// Time without an accepted barometer fusion after which the vertical flags drop and the height is reset, us (B3).
    pub nav_aid_timeout_us: u64,
    /// Accelerometer velocity random walk, m/s/sqrt(s) (B3, `vrw`).
    pub nav_vrw: f32,
    /// Accelerometer bias random walk, m/s^2/sqrt(s) (B3, `ba_rw`).
    pub nav_ba_rw: f32,
    /// Initial vertical velocity sigma, m/s (B3, `P0_vel`: the emulation's value, forward.py:30).
    pub nav_p0_vel_ms: f32,
    /// Initial accelerometer bias sigma, m/s^2 (B3, `P0_ba`: PX4 ekf2_abias_init, px4-common.h:348).
    pub nav_p0_ba_ms2: f32,
    /// Altitude hold (decision-nav-eskf-20261120, Amendment F; the seven values owner-approved 2026-11-21): height
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
    /// descent speed from the estimate, s (Amendment F, F2: Betaflight alt_hold_multirotor.c:146).
    pub pilot_alt_lead_s: f32,
    /// Altitude hold: the smallest cos(tilt) used in the 1/cos thrust compensation (Amendment F, F4: ArduPilot
    /// AC_AttitudeControl_Multi.cpp:400).
    pub thr_tilt_cos_min: f32,
    /// Altitude hold: thrust factor clamp(gain x cos(tilt), 0, 1), which takes the thrust to 0 as the vehicle goes past
    /// 90 deg (Amendment F, F4: ArduPilot AC_AttitudeControl_Multi.cpp:398).
    pub thr_inverted_gain: f32,
    /// Position hold (decision-nav-eskf-20261120, Amendment P, P9; the six numbers owner-approved 2026-11-22): position
    /// error to velocity setpoint, 1/s. PX4 MPC_XY_P (ArduPilot 1.0).
    pub psc_posxy_p: f32,
    /// Velocity error to horizontal acceleration, (m/s^2)/(m/s). PX4 MPC_XY_VEL_P_ACC (ArduPilot 2.0).
    pub psc_velxy_p: f32,
    /// Velocity error integral to horizontal acceleration, (m/s^2)/m. PX4 MPC_XY_VEL_I_ACC (ArduPilot 1.0).
    pub psc_velxy_i: f32,
    /// Fastest horizontal speed the sticks ask for at full deflection, and the largest horizontal velocity setpoint,
    /// m/s. The default of the runtime parameter `PILOT_SPEED_XY` (owner, 2026-11-22: a parameter, range 3 to 20): the
    /// flight code takes the value in force at the arming step. ArduPilot POSCONTROL_SPEED_MS (AC_PosControl.h:25).
    pub pilot_speed_xy_ms: f32,
    /// Dead zone of the roll and pitch sticks in position hold, as a share of full travel on each side of the centre.
    /// PX4 MAN_DEADZONE; equals `pilot_thr_dz`.
    pub pilot_xy_dz: f32,
    /// Dead zone of the yaw stick in every mode, as a share of full travel on each side of the centre. ArduPilot
    /// `channel_yaw->set_default_dead_zone(15)` (ArduCopter/radio.cpp:13), 15 us of the 400 us half-range of the default
    /// calibration (RC min 1100, trim 1500, max 1900, `fw_settings::rc`), so 0.0375; owner approved 2026-11-22.
    pub pilot_y_dz: f32,
    /// Position hold: with the sticks centred the position is captured once the ground speed is below this, m/s. PX4
    /// MPC_HOLD_MAX_XY.
    pub pilot_hold_speed_ms: f32,
    /// Position hold's airborne latch (Amendment P, P3 replaced): the time the collective must stay below the ground
    /// threshold, armed, before the vehicle counts as landed, us. ArduPilot LAND_DETECTOR_TRIGGER_SEC 1.0 s
    /// (ArduCopter/config.h:359-360; land_detector.cpp:78, :151-156). Owner approved 2026-11-22.
    pub land_detect_us: u64,
    /// `Eskf` (decision-nav-eskf-20261120, Amendment C, C6; approved 2026-11-20/21): the IMU history the delayed horizon
    /// may use, us (section 10).
    pub nav_buffer_us: u64,
    /// The delayed horizon: GNSS delay, us. 70 000 for the SAM-M10Q, owner 2026-11-22: measured with the receiver's PPS
    /// (edge to the fix's hand-off to the flight code, 50 of 50 pairs: 59.4-75.5 ms, five samples 65.6-74.3 ms, mean
    /// 69.6 ms; firmware pps-latency); the replay scan of log 278 gave 0.057-0.103 s. Was 120 000 (ArduPilot get_lag).
    pub nav_gps_delay_us: u64,
    /// Lowest GNSS epoch rate accepted, Hz: 1000/245, ArduPilot's per-frame bound of 245 ms (`AP_GPS.cpp:849`; its 215 ms is a bound
    /// on the AVERAGE interval, `:861`; Amendment G, G4, owner-approved 2026-11-22; a parameter).
    pub nav_gps_min_rate_hz: f32,
    /// Floor of the GNSS position sigma, m (section 10, recon.py:39).
    pub nav_gps_pos_sigma_min_m: f32,
    /// GNSS velocity sigma, m/s (section 10).
    pub nav_gps_vel_sigma_ms: f32,
    /// Lowest ground speed whose course is used, m/s (section 10, recon.py:42).
    pub nav_gps_course_min_ms: f32,
    /// No-aid position noise while armed, m (section 10, ArduPilot NOAID_M_NSE).
    pub nav_noaid_sigma_m: f32,
    /// No-aid position noise while disarmed, m (B3, ap-AP_NavEKF3_PosVelFusion.cpp:782-784).
    pub nav_noaid_sigma_dis_m: f32,
    /// Initial tilt and yaw sigma, rad (B3, `P0_tilt`: PX4 EKF2_ANGERR_INIT).
    pub nav_p0_tilt_rad: f32,
    /// Initial gyro bias sigma, rad/s (B3, `P0_bg`: px4-common.h:347).
    pub nav_p0_bg_rads: f32,
    /// Gyro angular random walk, rad/sqrt(s) (section 10, `arw`).
    pub nav_arw: f32,
    /// Gyro bias random walk, rad/s/sqrt(s) (section 10, `bg_rw`).
    pub nav_bg_rw: f32,
    /// Largest heading sigma for `Valid.heading`, rad: 15 deg, PX4 EKFGSF_yaw_err_max (C6).
    pub nav_heading_valid_sigma_rad: f32,
    /// Compass heading noise, 1 sigma, rad: 12.5 deg (C6).
    pub nav_mag_heading_sigma_rad: f32,
    /// One compass yaw fusion per this period, us (C6).
    pub nav_mag_fuse_us: u64,
    /// Velocity screen: GNSS speed below this share of the position-difference speed (C6, E3b).
    pub nav_gps_spd_ratio: f32,
    /// Velocity screen: only at a position-difference speed at or above this, m/s (C6, E3b).
    pub nav_gps_spd_min_ms: f32,
    /// Magnetic declination, east positive, rad: 3.195 deg E, the owner's field, WMM2025 (C4, C6; compiled for now).
    pub nav_mag_declination_rad: f32,
    /// `EskfSteer` output predictor: time constant of the velocity tracking, s (Amendment G, G1; owner-approved
    /// 2026-11-22). PX4 EKF2_TAU_VEL default, `output_predictor.cpp:326`; ArduPilot EK3_TAU_OUTPUT 25 cs.
    pub nav_out_tau_vel_s: f32,
    /// `EskfSteer` output predictor: time constant of the position tracking, s (Amendment G, G1; owner-approved
    /// 2026-11-22). PX4 EKF2_TAU_POS, `output_predictor.cpp:327`.
    pub nav_out_tau_pos_s: f32,
    /// `EskfSteer` output predictor: the share of the attitude error taken out per delay time (Amendment G, G1;
    /// owner-approved 2026-11-22). PX4 `output_predictor.cpp:311`; ArduPilot `AP_NavEKF3_core.cpp:938`.
    pub nav_out_att_gain: f32,
    /// `EskfSteer` output predictor: gain of the integral term of the velocity and position tracking (Amendment G, G1;
    /// owner-approved 2026-11-22). PX4 `output_predictor.cpp:350`, `:354`; ArduPilot `AP_NavEKF3_core.cpp:970-973`.
    pub nav_out_int_gain: f32,
}

impl Tuning {
    /// The approved numbers (decision-first-pair-20261119, section 13).
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
        psc_posxy_p: 0.95,
        psc_velxy_p: 1.8,
        psc_velxy_i: 0.4,
        pilot_speed_xy_ms: 5.0,
        pilot_xy_dz: 0.10,
        pilot_y_dz: 0.0375,
        pilot_hold_speed_ms: 0.8,
        land_detect_us: 1_000_000,
    };
}
