use crate::{
    ClosedLoopConfig, ClosedLoopTruthState, ControlPipeline, ControlPipelineTrace, ControlSetpoint,
    EstimateSnapshot, EstimatorReplayConfig, EstimatorReplayDriver, EstimatorReplayTrace,
    HarnessResult, ImuControlInput, SensorFrame, TruthPlant,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulatedSensorConfig {
    pub magnetic_field_body_ut: [f64; 3],
    pub gps_dropout_ticks: Option<(u64, u64)>,
    pub baro_spike: Option<BaroSpike>,
}

impl SimulatedSensorConfig {
    pub const fn new() -> Self {
        Self {
            magnetic_field_body_ut: [20.0, 0.0, 40.0],
            gps_dropout_ticks: None,
            baro_spike: None,
        }
    }

    pub fn with_gps_dropout(mut self, first_tick: u64, last_tick: u64) -> Self {
        assert!(first_tick <= last_tick);
        self.gps_dropout_ticks = Some((first_tick, last_tick));
        self
    }

    pub fn with_baro_spike(mut self, tick: u64, added_down_m: f64) -> Self {
        assert!(tick > 0);
        assert!(added_down_m.is_finite());
        self.baro_spike = Some(BaroSpike { tick, added_down_m });
        self
    }

    fn gps_valid(self, tick: u64) -> bool {
        !self
            .gps_dropout_ticks
            .is_some_and(|(first, last)| (first..=last).contains(&tick))
    }

    fn baro_down_m(self, tick: u64, truth_down_m: f32) -> f64 {
        let mut down = truth_down_m as f64;
        if let Some(spike) = self.baro_spike {
            if spike.tick == tick {
                down += spike.added_down_m;
            }
        }
        down
    }
}

impl Default for SimulatedSensorConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroSpike {
    pub tick: u64,
    pub added_down_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedLoopEstimatorConfig {
    pub closed_loop: ClosedLoopConfig,
    pub estimator: EstimatorReplayConfig,
    pub sensors: SimulatedSensorConfig,
}

impl ClosedLoopEstimatorConfig {
    pub const fn new(closed_loop: ClosedLoopConfig) -> Self {
        Self {
            closed_loop,
            estimator: EstimatorReplayConfig {
                gravity_body_mps2: [0.0, 0.0, -9.806_65],
                magnetic_field_inertial_ut: [20.0, 0.0, 40.0],
                max_baro_step_m: None,
            },
            sensors: SimulatedSensorConfig::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClosedLoopEstimatorTrace {
    pub tick: u64,
    pub sim_time_us: u64,
    pub truth: ClosedLoopTruthState,
    pub sensors: SensorFrame,
    pub estimator: EstimatorReplayTrace,
    pub control: ControlPipelineTrace,
}

impl ClosedLoopEstimatorTrace {
    pub fn estimate_snapshot(&self) -> EstimateSnapshot {
        estimate_snapshot(&self.estimator)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ClosedLoopEstimatorReport {
    pub traces: Vec<ClosedLoopEstimatorTrace>,
    pub gps_updates: u32,
    pub baro_updates: u32,
    pub rejected_baro_spikes: u32,
}

impl ClosedLoopEstimatorReport {
    pub fn last_trace(&self) -> Option<&ClosedLoopEstimatorTrace> {
        self.traces.last()
    }

    pub fn trace_csv(&self) -> String {
        let mut csv = String::from(
            "tick,sim_time_us,truth_n,truth_e,truth_d,truth_vn,truth_ve,truth_vd,estimate_valid,estimate_n,estimate_e,estimate_d,estimate_vn,estimate_ve,estimate_vd,gps_used,baro_used,baro_rejected,motor_1,motor_2,motor_3,motor_4,control_valid\n",
        );
        for trace in &self.traces {
            let motors = trace.control.motors;
            csv.push_str(&format!(
                "{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{},{},{:.6},{:.6},{:.6},{:.6},{}\n",
                trace.tick,
                trace.sim_time_us,
                trace.truth.position_ned_m[0],
                trace.truth.position_ned_m[1],
                trace.truth.position_ned_m[2],
                trace.truth.velocity_ned_mps[0],
                trace.truth.velocity_ned_mps[1],
                trace.truth.velocity_ned_mps[2],
                trace.estimator.estimate_valid,
                trace.estimator.position_ned_m[0],
                trace.estimator.position_ned_m[1],
                trace.estimator.position_ned_m[2],
                trace.estimator.velocity_ned_mps[0],
                trace.estimator.velocity_ned_mps[1],
                trace.estimator.velocity_ned_mps[2],
                trace.estimator.gps_used,
                trace.estimator.baro_used,
                trace.estimator.baro_rejected,
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                trace.control.control_valid,
            ));
        }
        csv
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClosedLoopEstimatorRunner {
    pub pipeline: ControlPipeline,
    pub plant: TruthPlant,
    pub config: ClosedLoopEstimatorConfig,
}

impl ClosedLoopEstimatorRunner {
    pub fn new(
        pipeline: ControlPipeline,
        plant: TruthPlant,
        config: ClosedLoopEstimatorConfig,
    ) -> Self {
        assert!(config.closed_loop.dt_s.is_finite() && config.closed_loop.dt_s > 0.0);
        assert!(config.closed_loop.ticks > 0);
        Self {
            pipeline,
            plant,
            config,
        }
    }

    pub fn run(&mut self, setpoint: ControlSetpoint) -> HarnessResult<ClosedLoopEstimatorReport> {
        let mut driver = EstimatorReplayDriver::new(self.config.estimator);
        let mut report = ClosedLoopEstimatorReport {
            traces: Vec::with_capacity(self.config.closed_loop.ticks),
            ..ClosedLoopEstimatorReport::default()
        };

        for index in 0..self.config.closed_loop.ticks {
            let tick = (index + 1) as u64;
            let sim_time_us = dt_us(self.config.closed_loop.dt_s) * tick;
            let sensors =
                simulated_sensors(tick, sim_time_us, self.plant.state, self.config.sensors);
            let estimator = driver.step(tick, &sensors)?;
            let control = self.pipeline.step(
                estimate_snapshot(&estimator),
                imu_from_sensors(&sensors),
                setpoint,
            );
            let truth = self
                .plant
                .step(control.motors, self.config.closed_loop.dt_s);

            report.traces.push(ClosedLoopEstimatorTrace {
                tick,
                sim_time_us,
                truth,
                sensors,
                estimator,
                control,
            });
        }

        report.gps_updates = driver.gps_updates();
        report.baro_updates = driver.baro_updates();
        report.rejected_baro_spikes = driver.rejected_baro_spikes();
        Ok(report)
    }
}

pub fn simulated_sensors(
    tick: u64,
    timestamp_us: u64,
    truth: ClosedLoopTruthState,
    config: SimulatedSensorConfig,
) -> SensorFrame {
    let gps_valid = config.gps_valid(tick);
    let position = truth.position_ned_m.map(|value| value as f64);
    let velocity = truth.velocity_ned_mps.map(|value| value as f64);

    SensorFrame {
        timestamp_us,
        accel_mps2: [0.0, 0.0, 0.0],
        gyro_rps: truth.body_rates_rps.map(|value| value as f64),
        mag_body_ut: Some(config.magnetic_field_body_ut),
        gps_position_ned_m: gps_valid.then_some(position),
        gps_velocity_ned_mps: gps_valid.then_some(velocity),
        baro_down_m: Some(config.baro_down_m(tick, truth.position_ned_m[2])),
    }
}

fn estimate_snapshot(trace: &EstimatorReplayTrace) -> EstimateSnapshot {
    let finite = trace.quaternion.iter().all(|value| value.is_finite())
        && trace.position_ned_m.iter().all(|value| value.is_finite())
        && trace.velocity_ned_mps.iter().all(|value| value.is_finite());

    EstimateSnapshot {
        position_ned_m: trace.position_ned_m.map(|value| value as f32),
        velocity_ned_mps: trace.velocity_ned_mps.map(|value| value as f32),
        quaternion: trace.quaternion.map(|value| value as f32),
        valid: trace.estimate_valid && finite,
    }
}

fn imu_from_sensors(sensors: &SensorFrame) -> ImuControlInput {
    ImuControlInput {
        accel_mps2: sensors.accel_mps2.map(|value| value as f32),
        gyro_rps: sensors.gyro_rps.map(|value| value as f32),
    }
}

fn dt_us(dt_s: f32) -> u64 {
    (dt_s * 1_000_000.0).round() as u64
}
