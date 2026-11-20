//! Gains and limits of the flight code, as constants until the parameters come with the thrust test
//! (step 6). Fields: decision-first-pair-20261119, section 9; values and their sources: its section 13
//! (rulings file, ruling 3). Units: rad, rad/s, us.

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
    };
}
