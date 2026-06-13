//! Shared data shapes for the estimator-to-mixer control chain.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn magnitude(self) -> f32 {
        micromath::F32Ext::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn limit_magnitude(self, max: f32) -> Self {
        let magnitude = self.magnitude();
        if magnitude <= max || magnitude <= f32::EPSILON {
            self
        } else {
            self * (max / magnitude)
        }
    }
}

impl core::ops::Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl core::ops::Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl core::ops::Mul<f32> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VehicleState {
    pub position_ned_m: Vec3,
    pub velocity_ned_mps: Vec3,
    pub attitude: Quaternion,
    pub body_rates_rad_s: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MissionSetpoint {
    pub target_lat_deg: f64,
    pub target_lon_deg: f64,
    pub target_alt_m: f32,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LocalPositionSetpoint {
    pub position_ned_m: Vec3,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VelocitySetpoint {
    pub velocity_ned_mps: Vec3,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AccelerationSetpoint {
    pub acceleration_ned_mps2: Vec3,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EulerAngles {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AttitudeSetpoint {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub thrust: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateSetpoint {
    pub body_rates_rad_s: Vec3,
    pub thrust: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TorqueCommand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
}
