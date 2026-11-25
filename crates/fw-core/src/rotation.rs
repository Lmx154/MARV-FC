//! Sensor orientation: ArduPilot's rotation enumeration, values 0 to 15 (every flat mounting, right
//! side up or upside down, in 45-degree steps of yaw), as plain 3x3 matrices. Pure; the flight core
//! calls it for every IMU sample, so it runs from SRAM (`memory.x`).
//!
//! The numbering and the formulas are ArduPilot's (`libraries/AP_Math/rotations.h`, `Vector3::rotate`
//! in `vector3.cpp`): `rotate` of the matrix of rotation `R` turns a vector in the sensor's axes
//! into the vector in the axes it is mounted to; `rotate_inverse` multiplies by the transpose.
//! Body frame: FRD (x forward, y right, z down), as in MAVLink and ArduPilot.

use core::f32::consts::FRAC_1_SQRT_2;

const H: f32 = FRAC_1_SQRT_2;

/// ArduPilot's `Rotation` values 0 to 15 (`rotations.h`), under ArduPilot's names.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Rotation {
    /// 0, `ROTATION_NONE`.
    #[default]
    None = 0,
    /// 1, `ROTATION_YAW_45`.
    Yaw45 = 1,
    /// 2, `ROTATION_YAW_90`.
    Yaw90 = 2,
    /// 3, `ROTATION_YAW_135`.
    Yaw135 = 3,
    /// 4, `ROTATION_YAW_180`.
    Yaw180 = 4,
    /// 5, `ROTATION_YAW_225`.
    Yaw225 = 5,
    /// 6, `ROTATION_YAW_270`.
    Yaw270 = 6,
    /// 7, `ROTATION_YAW_315`.
    Yaw315 = 7,
    /// 8, `ROTATION_ROLL_180`.
    Roll180 = 8,
    /// 9, `ROTATION_ROLL_180_YAW_45`.
    Roll180Yaw45 = 9,
    /// 10, `ROTATION_ROLL_180_YAW_90`.
    Roll180Yaw90 = 10,
    /// 11, `ROTATION_ROLL_180_YAW_135`.
    Roll180Yaw135 = 11,
    /// 12, `ROTATION_PITCH_180`.
    Pitch180 = 12,
    /// 13, `ROTATION_ROLL_180_YAW_225`.
    Roll180Yaw225 = 13,
    /// 14, `ROTATION_ROLL_180_YAW_270`.
    Roll180Yaw270 = 14,
    /// 15, `ROTATION_ROLL_180_YAW_315`.
    Roll180Yaw315 = 15,
}

/// The value of the parameters' smallest accepted rotation.
pub const MIN: i32 = 0;
/// The value of the parameters' largest accepted rotation.
pub const MAX: i32 = 15;

impl Rotation {
    /// The rotation with ArduPilot's number `v`, or `None` outside 0 to 15.
    pub const fn from_i32(v: i32) -> Option<Self> {
        Some(match v {
            0 => Self::None,
            1 => Self::Yaw45,
            2 => Self::Yaw90,
            3 => Self::Yaw135,
            4 => Self::Yaw180,
            5 => Self::Yaw225,
            6 => Self::Yaw270,
            7 => Self::Yaw315,
            8 => Self::Roll180,
            9 => Self::Roll180Yaw45,
            10 => Self::Roll180Yaw90,
            11 => Self::Roll180Yaw135,
            12 => Self::Pitch180,
            13 => Self::Roll180Yaw225,
            14 => Self::Roll180Yaw270,
            15 => Self::Roll180Yaw315,
            _ => return Option::None,
        })
    }

    /// ArduPilot's number.
    pub const fn as_i32(self) -> i32 {
        self as i32
    }

    /// The matrix: `rotate(v) = M * v`.
    pub const fn matrix(self) -> [[f32; 3]; 3] {
        match self {
            Self::None => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw45 => [[H, -H, 0.0], [H, H, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw90 => [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw135 => [[-H, -H, 0.0], [H, -H, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw180 => [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw225 => [[-H, H, 0.0], [-H, -H, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw270 => [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            Self::Yaw315 => [[H, H, 0.0], [-H, H, 0.0], [0.0, 0.0, 1.0]],
            Self::Roll180 => [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw45 => [[H, H, 0.0], [H, -H, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw90 => [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw135 => [[-H, H, 0.0], [H, H, 0.0], [0.0, 0.0, -1.0]],
            Self::Pitch180 => [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw225 => [[-H, -H, 0.0], [-H, H, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw270 => [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            Self::Roll180Yaw315 => [[H, -H, 0.0], [-H, -H, 0.0], [0.0, 0.0, -1.0]],
        }
    }

    /// `M * v`: a vector in the sensor's axes in the axes it is mounted to.
    pub fn rotate(self, v: [f32; 3]) -> [f32; 3] {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.matrix();
        let [x, y, z] = v;
        [
            a * x + b * y + c * z,
            d * x + e * y + f * z,
            g * x + h * y + i * z,
        ]
    }

    /// `M^T * v`, the inverse of [`Self::rotate`] (the matrices are orthonormal).
    pub fn rotate_inverse(self, v: [f32; 3]) -> [f32; 3] {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.matrix();
        let [x, y, z] = v;
        [
            a * x + d * y + g * z,
            b * x + e * y + h * z,
            c * x + f * y + i * z,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ArduPilot's `Vector3::rotate` (`vector3.cpp`), written out as it is there.
    fn ardupilot(r: i32, v: [f32; 3]) -> [f32; 3] {
        let [mut x, mut y, mut z] = v;
        let t;
        match r {
            0 => {}
            1 => {
                t = H * (x - y);
                y = H * (x + y);
                x = t;
            }
            2 => {
                t = x;
                x = -y;
                y = t;
            }
            3 => {
                t = -H * (x + y);
                y = H * (x - y);
                x = t;
            }
            4 => {
                x = -x;
                y = -y;
            }
            5 => {
                t = H * (y - x);
                y = -H * (x + y);
                x = t;
            }
            6 => {
                t = x;
                x = y;
                y = -t;
            }
            7 => {
                t = H * (x + y);
                y = H * (y - x);
                x = t;
            }
            8 => {
                y = -y;
                z = -z;
            }
            9 => {
                t = H * (x + y);
                y = H * (x - y);
                x = t;
                z = -z;
            }
            10 => {
                t = x;
                x = y;
                y = t;
                z = -z;
            }
            11 => {
                t = H * (y - x);
                y = H * (y + x);
                x = t;
                z = -z;
            }
            12 => {
                x = -x;
                z = -z;
            }
            13 => {
                t = -H * (x + y);
                y = H * (y - x);
                x = t;
                z = -z;
            }
            14 => {
                t = x;
                x = -y;
                y = -t;
                z = -z;
            }
            15 => {
                t = H * (x - y);
                y = -H * (x + y);
                x = t;
                z = -z;
            }
            _ => panic!("{r}"),
        }
        [x, y, z]
    }

    fn close(a: [f32; 3], b: [f32; 3]) -> bool {
        a.iter().zip(b).all(|(p, q)| (p - q).abs() <= 1e-6)
    }

    fn det(m: [[f32; 3]; 3]) -> f32 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// all 16 values against ArduPilot's formulas on two vectors, each matrix orthonormal with
    /// determinant +1, and the inverse undoes the rotation.
    #[test]
    fn all_sixteen_equal_ardupilots_formulas_and_are_proper_rotations() {
        for r in MIN..=MAX {
            let rot = Rotation::from_i32(r).unwrap();
            assert_eq!(rot.as_i32(), r);
            for v in [[1.0, 2.0, 3.0], [0.3, -1.7, 2.9]] {
                assert!(close(rot.rotate(v), ardupilot(r, v)), "{r} {v:?}");
                assert!(close(rot.rotate_inverse(rot.rotate(v)), v), "{r} {v:?}");
                assert!(close(rot.rotate(rot.rotate_inverse(v)), v), "{r} {v:?}");
            }
            let m = rot.matrix();
            for i in 0..3 {
                for j in 0..3 {
                    let dot: f32 = (0..3).map(|k| m[k][i] * m[k][j]).sum();
                    let want = if i == j { 1.0 } else { 0.0 };
                    assert!((dot - want).abs() <= 1e-6, "{r} columns {i} {j}");
                }
            }
            assert!((det(m) - 1.0).abs() <= 1e-6, "{r}: {}", det(m));
        }
        assert_eq!(Rotation::from_i32(-1), None);
        assert_eq!(Rotation::from_i32(16), None);
        assert_eq!(Rotation::default(), Rotation::None);
    }

    /// ROLL_180 (the IMUs' mounting) turns chip gravity (0, 0, +g) into (0, 0, -g); ROLL_180_YAW_90
    /// (the compass) maps chip +Y to body +X, chip +X to body +Y, chip +Z to body -Z.
    #[test]
    fn the_mountings_map_the_owners_axes() {
        assert_eq!(
            Rotation::Roll180.rotate([0.0, 0.0, 9.81]),
            [0.0, 0.0, -9.81]
        );
        assert_eq!(Rotation::Roll180.rotate([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
        let mag = Rotation::from_i32(10).unwrap();
        assert_eq!(mag.rotate([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0]);
        assert_eq!(mag.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_eq!(mag.rotate([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0]);
    }
}
