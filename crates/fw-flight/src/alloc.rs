//! The allocator from the vehicle card: thrust and torques to four DShot levels (decision-first-pair-20261119,
//! section 1). Rotor thrusts T = MIX * [thrust, t_x, t_y, t_z] (card constants, rows = DShot lines), each kept in
//! [T_lo, T_hi]; yaw is given up first, then roll and pitch; the collective is only ever lowered, at the top
//! (no airmode). Units: N, N.m, rad/s.

use crate::gnc::{Allocation, Allocator, MotorLevels, Saturation, ThrustTorque};
use marv_card::{IDLE_LEVEL, K_F, LEVEL_MAX, LEVEL_MIN, MAX_ROTOR_SPEED, MIX, SPEED_AT_LEVEL_MIN};

/// The allocator built from the card's `const` data.
#[derive(Clone, Copy, Debug)]
pub struct CardAllocator {
    /// Lowest rotor thrust, N: the thrust at `IDLE_LEVEL`.
    t_lo: f32,
    /// Highest rotor thrust, N: the thrust at `MAX_ROTOR_SPEED`.
    t_hi: f32,
}

impl CardAllocator {
    /// The per-rotor thrust bounds from the card (decision 1b): T_lo = K_F w(IDLE_LEVEL)^2, T_hi = K_F w_max^2, N.
    pub fn new() -> Self {
        let idle = speed(IDLE_LEVEL);
        CardAllocator {
            t_lo: K_F * idle * idle,
            t_hi: K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED,
        }
    }

    /// Rotor thrusts by DShot line, N, each in [T_lo, T_hi], and what could not be met (decision 1c, steps 1 to 4).
    fn thrusts(&self, demand: &ThrustTorque) -> ([f32; 4], Saturation) {
        let (lo, hi) = (self.t_lo, self.t_hi);
        let [tx, ty, tz] = demand.torque_nm;

        // 1. Collective inside what four rotors can give.
        let c = demand.thrust_n.max(4.0 * lo).min(4.0 * hi);
        let thrust_low = c > demand.thrust_n;
        let mut thrust_high = c < demand.thrust_n;
        // At the floor the shares a and e are 0 by rule (no authority at idle, no airmode; decision 1d): all four
        // rotors at T_lo, so exactly IDLE_LEVEL, whatever the torques and the card's symmetry.
        if thrust_low {
            let all = Saturation {
                roll: true,
                pitch: true,
                yaw: true,
                thrust_low,
                thrust_high,
            };
            return ([lo; 4], all);
        }

        // 2. Roll and pitch: the largest share a that keeps every rotor above T_lo and fits the span T_hi - T_lo,
        //    then lower the collective by d (<= 0) so the top rotor is at most T_hi.
        let base = MIX.map(|[m_t, ..]| m_t * c);
        let roll_pitch = MIX.map(|[_, m_x, m_y, _]| m_x * tx + m_y * ty);
        let spread = largest(&roll_pitch) - smallest(&roll_pitch);
        let a = largest_fit(
            &base,
            &roll_pitch,
            lo,
            f32::INFINITY,
            ((hi - lo) / spread).min(1.0),
        );
        let mut u = base;
        for (ui, ri) in u.iter_mut().zip(roll_pitch) {
            *ui += a * ri;
        }
        let d = (hi - largest(&u)).min(0.0);
        thrust_high |= d < 0.0;
        for ui in u.iter_mut() {
            *ui += d;
        }

        // 3. Yaw: the largest share e that keeps every rotor inside [T_lo, T_hi].
        let yaw = MIX.map(|[.., m_z]| m_z * tz);
        let e = largest_fit(&u, &yaw, lo, hi, 1.0);

        // 4. Clamp (float rounding only).
        for (ui, yi) in u.iter_mut().zip(yaw) {
            *ui = (*ui + e * yi).max(lo).min(hi);
        }
        let saturation = Saturation {
            roll: a < 1.0,
            pitch: a < 1.0,
            yaw: e < 1.0,
            thrust_low,
            thrust_high,
        };
        (u, saturation)
    }
}

impl Default for CardAllocator {
    fn default() -> Self {
        Self::new()
    }
}

impl Allocator for CardAllocator {
    fn allocate(&mut self, demand: &ThrustTorque) -> Allocation {
        let (thrusts, saturation) = self.thrusts(demand);
        Allocation {
            levels: MotorLevels {
                level: thrusts.map(level),
            },
            saturation,
        }
    }
}

/// The plant's level-to-speed map, rad/s: linear from (LEVEL_MIN, SPEED_AT_LEVEL_MIN) to (LEVEL_MAX,
/// MAX_ROTOR_SPEED).
fn speed(level: u16) -> f32 {
    let span =
        (f32::from(level) - f32::from(LEVEL_MIN)) / (f32::from(LEVEL_MAX) - f32::from(LEVEL_MIN));
    SPEED_AT_LEVEL_MIN + span * (MAX_ROTOR_SPEED - SPEED_AT_LEVEL_MIN)
}

/// Rotor thrust, N, to DShot level: w = sqrt(T / K_F), then the inverse of [`speed`], rounded and clamped to
/// [IDLE_LEVEL, LEVEL_MAX] (decision 1b). A NaN thrust gives IDLE_LEVEL.
fn level(thrust_n: f32) -> u16 {
    let w = libm::sqrtf(thrust_n / K_F);
    let span = (w - SPEED_AT_LEVEL_MIN) / (MAX_ROTOR_SPEED - SPEED_AT_LEVEL_MIN);
    let l =
        libm::roundf(f32::from(LEVEL_MIN) + span * (f32::from(LEVEL_MAX) - f32::from(LEVEL_MIN)));
    // The card's build.rs fails unless IDLE_LEVEL <= LEVEL_MAX, so clamp cannot panic.
    (l as u16).clamp(IDLE_LEVEL, LEVEL_MAX)
}

/// The largest s in [0, cap] with lo <= base_i + s * delta_i <= hi on every line; 0 when no s fits (then nothing of
/// delta is applied, as decision 1d expects at the thrust floor).
fn largest_fit(base: &[f32; 4], delta: &[f32; 4], lo: f32, hi: f32, cap: f32) -> f32 {
    let (mut s_min, mut s_max) = (0.0f32, cap);
    for (&b, &dl) in base.iter().zip(delta) {
        if dl > 0.0 {
            s_min = s_min.max((lo - b) / dl);
            s_max = s_max.min((hi - b) / dl);
        } else if dl < 0.0 {
            s_min = s_min.max((hi - b) / dl);
            s_max = s_max.min((lo - b) / dl);
        } else if b < lo || b > hi {
            return 0.0;
        }
    }
    if s_min <= s_max { s_max } else { 0.0 }
}

fn largest(v: &[f32; 4]) -> f32 {
    v.iter().fold(f32::NEG_INFINITY, |m, &x| m.max(x))
}

fn smallest(v: &[f32; 4]) -> f32 {
    v.iter().fold(f32::INFINITY, |m, &x| m.min(x))
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::calib::GRAVITY_MSS;
    use marv_card::{MASS, ROTORS};

    const NO_FLAGS: Saturation = Saturation {
        roll: false,
        pitch: false,
        yaw: false,
        thrust_low: false,
        thrust_high: false,
    };

    fn demand(thrust_n: f32, torque_nm: [f32; 3]) -> ThrustTorque {
        ThrustTorque {
            thrust_n,
            torque_nm,
        }
    }

    #[test]
    fn allocator_zero_throttle_is_idle_on_all_four() {
        let out = CardAllocator::new().allocate(&demand(0.0, [0.0; 3]));
        assert_eq!(out.levels.level, [IDLE_LEVEL; 4]);
        let want = Saturation {
            roll: true,
            pitch: true,
            yaw: true,
            thrust_low: true,
            thrust_high: false,
        };
        assert_eq!(out.saturation, want);
        // A large torque demand at zero throttle: still idle on all four (decision 1d).
        let out = CardAllocator::new().allocate(&demand(0.0, [5.0, -5.0, 5.0]));
        assert_eq!(out.levels.level, [IDLE_LEVEL; 4]);
        assert_eq!(out.saturation, want);
    }

    #[test]
    fn allocator_signs_follow_the_conventions() {
        let mut alloc = CardAllocator::new();
        let hover = MASS * GRAVITY_MSS;
        let base = alloc.allocate(&demand(hover, [0.0; 3]));
        assert_eq!(base.saturation, NO_FLAGS);
        // Hover: each rotor at sqrt(m g / (4 K_F)) through the map.
        let w = libm::sqrtf(hover / (4.0 * K_F));
        let want = f32::from(LEVEL_MIN)
            + (w - SPEED_AT_LEVEL_MIN) / (MAX_ROTOR_SPEED - SPEED_AT_LEVEL_MIN)
                * (f32::from(LEVEL_MAX) - f32::from(LEVEL_MIN));
        for l in base.levels.level {
            assert!((f32::from(l) - want).abs() <= 1.0, "{l} vs {want}");
        }
        let lv = base.levels.level;
        assert!(
            lv.iter().max().unwrap() - lv.iter().min().unwrap() <= 1,
            "{lv:?}"
        );

        // A small torque that fits at hover on every axis (decision 3e: yaw has far less authority than roll and pitch).
        let small = 0.01;
        // (axis, lines that rise for + torque): +x left (2, 3); +y front (0, 2); +z CCW (0, 3) (conventions 7e, 7f).
        for (axis, up) in [(0, [2, 3]), (1, [0, 2]), (2, [0, 3])] {
            for sign in [1.0f32, -1.0] {
                let mut t = [0.0; 3];
                t[axis] = sign * small;
                let out = alloc.allocate(&demand(hover, t));
                assert_eq!(out.saturation, NO_FLAGS, "axis {axis} sign {sign}");
                for line in 0..4 {
                    let rises = up.contains(&line) == (sign > 0.0);
                    let (now, was) = (out.levels.level[line], base.levels.level[line]);
                    assert!(
                        if rises { now > was } else { now < was },
                        "axis {axis} sign {sign} line {line}: {now} vs {was}"
                    );
                }
            }
        }
    }

    #[test]
    fn allocator_gives_up_yaw_first() {
        let mut alloc = CardAllocator::new();
        let hover = MASS * GRAVITY_MSS;
        // Roll 0.1 N.m fits the roll authority; yaw 10 N.m is far beyond the yaw authority (decision 3e).
        let (tx, tz) = (0.1, 10.0);
        let d = demand(hover, [tx, 0.0, tz]);
        let (u, sat) = alloc.thrusts(&d);
        assert!(sat.yaw && !sat.roll && !sat.pitch, "{sat:?}");
        // The roll and pitch torques the four thrusts make (conventions 7e): roll fully applied, no pitch.
        let roll: f32 = ROTORS
            .iter()
            .zip(u)
            .map(|(r, t)| -r.position_m[1] * t)
            .sum();
        let pitch: f32 = ROTORS.iter().zip(u).map(|(r, t)| r.position_m[0] * t).sum();
        assert!((roll - tx).abs() < 1e-4, "roll {roll}");
        assert!(pitch.abs() < 1e-4, "pitch {pitch}");
        for t in u {
            assert!(t >= alloc.t_lo && t <= alloc.t_hi, "{u:?}");
        }
        for l in alloc.allocate(&d).levels.level {
            assert!((IDLE_LEVEL..=LEVEL_MAX).contains(&l));
        }
        // A collective above 4 T_hi.
        let out = alloc.allocate(&demand(8.0 * alloc.t_hi, [0.0; 3]));
        assert!(out.saturation.thrust_high, "{:?}", out.saturation);
        assert_eq!(out.levels.level, [LEVEL_MAX; 4]);
    }
}
