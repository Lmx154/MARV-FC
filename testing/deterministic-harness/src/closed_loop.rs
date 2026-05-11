use common::control::attitude::AttitudeSetpoint;

use crate::{
    ControlPipeline, ControlPipelineTrace, ControlSetpoint, EstimateSnapshot, ImuControlInput,
    MotorGeometry, OpenLoopPlant,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedLoopTruthState {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub euler_rad: [f32; 3],
    pub body_rates_rps: [f32; 3],
}

impl ClosedLoopTruthState {
    pub const LEVEL_ORIGIN: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        euler_rad: [0.0, 0.0, 0.0],
        body_rates_rps: [0.0, 0.0, 0.0],
    };

    pub fn estimate(self) -> EstimateSnapshot {
        EstimateSnapshot {
            position_ned_m: self.position_ned_m,
            velocity_ned_mps: self.velocity_ned_mps,
            quaternion: self.quaternion(),
            valid: true,
        }
    }

    pub fn imu(self) -> ImuControlInput {
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: self.body_rates_rps,
        }
    }

    pub fn quaternion(self) -> [f32; 4] {
        AttitudeSetpoint::from_euler_rad(self.euler_rad[0], self.euler_rad[1], self.euler_rad[2])
            .expect("truth euler angles should be finite")
            .quaternion
    }

    pub fn finite(self) -> bool {
        self.position_ned_m.iter().all(|value| value.is_finite())
            && self.velocity_ned_mps.iter().all(|value| value.is_finite())
            && self.euler_rad.iter().all(|value| value.is_finite())
            && self.body_rates_rps.iter().all(|value| value.is_finite())
            && self.quaternion().iter().all(|value| value.is_finite())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TruthPlant {
    pub geometry: MotorGeometry,
    pub state: ClosedLoopTruthState,
}

impl TruthPlant {
    pub fn new(geometry: MotorGeometry, state: ClosedLoopTruthState) -> Self {
        geometry.validate().expect("invalid motor geometry");
        Self { geometry, state }
    }

    pub fn step(&mut self, motors: [f32; 4], dt_s: f32) -> ClosedLoopTruthState {
        let mut open_loop = OpenLoopPlant::new(self.geometry);
        open_loop.state.velocity_down_mps = self.state.velocity_ned_mps[2];
        open_loop.state.roll_rate_rps = self.state.body_rates_rps[0];
        open_loop.state.pitch_rate_rps = self.state.body_rates_rps[1];
        open_loop.state.yaw_rate_rps = self.state.body_rates_rps[2];

        let response = open_loop.apply_outputs(motors, dt_s);
        self.state.velocity_ned_mps[2] = response.next_state.velocity_down_mps;
        self.state.position_ned_m[2] += self.state.velocity_ned_mps[2] * dt_s;
        self.state.body_rates_rps = [
            response.next_state.roll_rate_rps,
            response.next_state.pitch_rate_rps,
            response.next_state.yaw_rate_rps,
        ];
        self.state.euler_rad[0] += self.state.body_rates_rps[0] * dt_s;
        self.state.euler_rad[1] += self.state.body_rates_rps[1] * dt_s;
        self.state.euler_rad[2] += self.state.body_rates_rps[2] * dt_s;

        self.state
    }
}

impl Default for TruthPlant {
    fn default() -> Self {
        Self::new(MotorGeometry::default(), ClosedLoopTruthState::LEVEL_ORIGIN)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedLoopConfig {
    pub dt_s: f32,
    pub ticks: usize,
}

impl ClosedLoopConfig {
    pub const fn new(dt_s: f32, ticks: usize) -> Self {
        Self { dt_s, ticks }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedLoopTrace {
    pub tick: u64,
    pub sim_time_us: u64,
    pub truth: ClosedLoopTruthState,
    pub control: ControlPipelineTrace,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClosedLoopRunner {
    pub pipeline: ControlPipeline,
    pub plant: TruthPlant,
    pub config: ClosedLoopConfig,
}

impl ClosedLoopRunner {
    pub fn new(pipeline: ControlPipeline, plant: TruthPlant, config: ClosedLoopConfig) -> Self {
        assert!(config.dt_s.is_finite() && config.dt_s > 0.0);
        assert!(config.ticks > 0);
        Self {
            pipeline,
            plant,
            config,
        }
    }

    pub fn run(&mut self, setpoint: ControlSetpoint) -> Vec<ClosedLoopTrace> {
        let mut traces = Vec::with_capacity(self.config.ticks);

        for index in 0..self.config.ticks {
            let tick = (index + 1) as u64;
            let control = self.pipeline.step(
                self.plant.state.estimate(),
                self.plant.state.imu(),
                setpoint,
            );
            let truth = self.plant.step(control.motors, self.config.dt_s);
            traces.push(ClosedLoopTrace {
                tick,
                sim_time_us: (self.config.dt_s * 1_000_000.0).round() as u64 * tick,
                truth,
                control,
            });
        }

        traces
    }
}
