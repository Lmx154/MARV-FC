use common::localization::estimation::{LayeredNavigationStack, Vec3};

use crate::{Fixture, HarnessFailure, HarnessResult};

#[derive(Clone, Debug, PartialEq)]
pub struct SensorFrame {
    pub timestamp_us: u64,
    pub accel_mps2: [f64; 3],
    pub gyro_rps: [f64; 3],
    pub mag_body_ut: Option<[f64; 3]>,
    pub gps_position_ned_m: Option<[f64; 3]>,
    pub gps_velocity_ned_mps: Option<[f64; 3]>,
    pub baro_down_m: Option<f64>,
}

impl SensorFrame {
    pub fn from_fixture(fixture: &Fixture) -> HarnessResult<Vec<Self>> {
        let columns = SensorFixtureColumns::from_fixture(fixture)?;
        let mut frames = Vec::with_capacity(fixture.samples.len());

        for (index, sample) in fixture.samples.iter().enumerate() {
            let tick = (index + 1) as u64;
            let frame = Self {
                timestamp_us: sample.timestamp_us,
                accel_mps2: columns.vec3(
                    sample,
                    tick,
                    "accel",
                    ["accel_x", "accel_y", "accel_z"],
                )?,
                gyro_rps: columns.vec3(sample, tick, "gyro", ["gyro_x", "gyro_y", "gyro_z"])?,
                mag_body_ut: columns.optional_vec3(
                    sample,
                    tick,
                    "mag_valid",
                    "mag",
                    ["mag_x", "mag_y", "mag_z"],
                )?,
                gps_position_ned_m: columns.optional_vec3(
                    sample,
                    tick,
                    "gps_valid",
                    "gps_position",
                    ["gps_n", "gps_e", "gps_d"],
                )?,
                gps_velocity_ned_mps: columns.optional_vec3(
                    sample,
                    tick,
                    "gps_valid",
                    "gps_velocity",
                    ["gps_vn", "gps_ve", "gps_vd"],
                )?,
                baro_down_m: columns.optional_scalar(sample, tick, "baro_valid", "baro_down")?,
            };
            frame.validate(tick)?;
            frames.push(frame);
        }

        Ok(frames)
    }

    pub fn validate(&self, tick: u64) -> HarnessResult<()> {
        finite_f64x3(tick, self.timestamp_us, "accel_mps2", self.accel_mps2)?;
        finite_f64x3(tick, self.timestamp_us, "gyro_rps", self.gyro_rps)?;
        if let Some(value) = self.mag_body_ut {
            finite_f64x3(tick, self.timestamp_us, "mag_body_ut", value)?;
        }
        if let Some(value) = self.gps_position_ned_m {
            finite_f64x3(tick, self.timestamp_us, "gps_position_ned_m", value)?;
        }
        if let Some(value) = self.gps_velocity_ned_mps {
            finite_f64x3(tick, self.timestamp_us, "gps_velocity_ned_mps", value)?;
        }
        if self.baro_down_m.is_some_and(|value| !value.is_finite()) {
            return Err(HarnessFailure::new(
                tick,
                self.timestamp_us,
                "baro_down_m contains a non-finite asserted value",
            ));
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorReplayConfig {
    pub gravity_body_mps2: [f64; 3],
    pub magnetic_field_inertial_ut: [f64; 3],
    pub max_baro_step_m: Option<f64>,
}

impl Default for EstimatorReplayConfig {
    fn default() -> Self {
        Self {
            gravity_body_mps2: [0.0, 0.0, -9.806_65],
            magnetic_field_inertial_ut: [20.0, 0.0, 40.0],
            max_baro_step_m: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct EstimatorReplayTrace {
    pub tick: u64,
    pub timestamp_us: u64,
    pub dt_s: Option<f64>,
    pub estimate_valid: bool,
    pub quaternion: [f64; 4],
    pub position_ned_m: [f64; 3],
    pub velocity_ned_mps: [f64; 3],
    pub gps_used: bool,
    pub baro_used: bool,
    pub baro_rejected: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct EstimatorReplayReport {
    pub traces: Vec<EstimatorReplayTrace>,
    pub gps_updates: u32,
    pub baro_updates: u32,
    pub rejected_baro_spikes: u32,
}

impl EstimatorReplayReport {
    pub fn last_trace(&self) -> Option<&EstimatorReplayTrace> {
        self.traces.last()
    }
}

pub struct EstimatorReplayDriver {
    stack: LayeredNavigationStack<f64>,
    config: EstimatorReplayConfig,
    previous_timestamp_us: Option<u64>,
    last_accepted_baro_down_m: Option<f64>,
    gps_updates: u32,
    baro_updates: u32,
    rejected_baro_spikes: u32,
}

impl EstimatorReplayDriver {
    pub fn new(config: EstimatorReplayConfig) -> Self {
        Self {
            stack: LayeredNavigationStack::<f64>::default(),
            config,
            previous_timestamp_us: None,
            last_accepted_baro_down_m: None,
            gps_updates: 0,
            baro_updates: 0,
            rejected_baro_spikes: 0,
        }
    }

    pub fn gps_updates(&self) -> u32 {
        self.gps_updates
    }

    pub fn baro_updates(&self) -> u32 {
        self.baro_updates
    }

    pub fn rejected_baro_spikes(&self) -> u32 {
        self.rejected_baro_spikes
    }

    pub fn step(&mut self, tick: u64, frame: &SensorFrame) -> HarnessResult<EstimatorReplayTrace> {
        frame.validate(tick)?;

        if let Some(previous) = self.previous_timestamp_us {
            if frame.timestamp_us <= previous {
                return Err(HarnessFailure::new(
                    tick,
                    frame.timestamp_us,
                    format!(
                        "estimator replay timestamps must be strictly monotonic: {} <= {previous}",
                        frame.timestamp_us
                    ),
                ));
            }
        }

        let mut dt_s = None;
        let mut estimate_valid = false;
        let mut gps_used = false;
        let mut baro_used = false;
        let mut baro_rejected = false;

        if let Some(previous) = self.previous_timestamp_us {
            let dt = (frame.timestamp_us - previous) as f64 / 1_000_000.0;
            dt_s = Some(dt);
            self.stack
                .predict(
                    Vec3::<f64>::new(
                        frame.accel_mps2[0],
                        frame.accel_mps2[1],
                        frame.accel_mps2[2],
                    ),
                    Vec3::<f64>::new(frame.gyro_rps[0], frame.gyro_rps[1], frame.gyro_rps[2]),
                    dt,
                    Some(frame.timestamp_us as f64 / 1_000_000.0),
                )
                .map_err(|error| {
                    HarnessFailure::new(
                        tick,
                        frame.timestamp_us,
                        format!("estimator predict failed: {error:?}"),
                    )
                })?;
            self.stack
                .update_gravity_alignment(
                    Vec3::<f64>::new(
                        self.config.gravity_body_mps2[0],
                        self.config.gravity_body_mps2[1],
                        self.config.gravity_body_mps2[2],
                    ),
                    None,
                )
                .map_err(|error| {
                    HarnessFailure::new(
                        tick,
                        frame.timestamp_us,
                        format!("gravity update failed: {error:?}"),
                    )
                })?;

            if let Some(mag) = frame.mag_body_ut {
                self.stack
                    .update_magnetic_field(
                        Vec3::<f64>::new(mag[0], mag[1], mag[2]),
                        Vec3::<f64>::new(
                            self.config.magnetic_field_inertial_ut[0],
                            self.config.magnetic_field_inertial_ut[1],
                            self.config.magnetic_field_inertial_ut[2],
                        ),
                        None,
                    )
                    .map_err(|error| {
                        HarnessFailure::new(
                            tick,
                            frame.timestamp_us,
                            format!("magnetic update failed: {error:?}"),
                        )
                    })?;
            }

            if let Some(position) = frame.gps_position_ned_m {
                self.stack
                    .update_position(
                        Vec3::<f64>::new(position[0], position[1], position[2]),
                        None,
                    )
                    .map_err(|error| {
                        HarnessFailure::new(
                            tick,
                            frame.timestamp_us,
                            format!("gps position update failed: {error:?}"),
                        )
                    })?;
                gps_used = true;
                self.gps_updates = self.gps_updates.saturating_add(1);
            }
            if let Some(velocity) = frame.gps_velocity_ned_mps {
                self.stack
                    .update_velocity(
                        Vec3::<f64>::new(velocity[0], velocity[1], velocity[2]),
                        None,
                    )
                    .map_err(|error| {
                        HarnessFailure::new(
                            tick,
                            frame.timestamp_us,
                            format!("gps velocity update failed: {error:?}"),
                        )
                    })?;
            }
            if let Some(baro_down_m) = frame.baro_down_m {
                if self
                    .config
                    .max_baro_step_m
                    .zip(self.last_accepted_baro_down_m)
                    .is_some_and(|(limit, previous): (f64, f64)| {
                        (baro_down_m - previous).abs() > limit
                    })
                {
                    baro_rejected = true;
                    self.rejected_baro_spikes = self.rejected_baro_spikes.saturating_add(1);
                } else {
                    self.stack
                        .update_barometric_altitude(baro_down_m, None)
                        .map_err(|error| {
                            HarnessFailure::new(
                                tick,
                                frame.timestamp_us,
                                format!("barometric update failed: {error:?}"),
                            )
                        })?;
                    baro_used = true;
                    self.last_accepted_baro_down_m = Some(baro_down_m);
                    self.baro_updates = self.baro_updates.saturating_add(1);
                }
            }

            estimate_valid = true;
        }

        self.previous_timestamp_us = Some(frame.timestamp_us);
        let state = self.stack.state();
        Ok(EstimatorReplayTrace {
            tick,
            timestamp_us: frame.timestamp_us,
            dt_s,
            estimate_valid,
            quaternion: [
                state.quaternion[0],
                state.quaternion[1],
                state.quaternion[2],
                state.quaternion[3],
            ],
            position_ned_m: [
                state.position_m[0],
                state.position_m[1],
                state.position_m[2],
            ],
            velocity_ned_mps: [
                state.velocity_mps[0],
                state.velocity_mps[1],
                state.velocity_mps[2],
            ],
            gps_used,
            baro_used,
            baro_rejected,
        })
    }
}

pub fn replay_estimator_frames(
    frames: &[SensorFrame],
    config: EstimatorReplayConfig,
) -> HarnessResult<EstimatorReplayReport> {
    if frames.is_empty() {
        return Err(HarnessFailure::new(
            0,
            0,
            "estimator replay requires at least one frame",
        ));
    }

    let mut driver = EstimatorReplayDriver::new(config);
    let mut report = EstimatorReplayReport::default();

    for (index, frame) in frames.iter().enumerate() {
        let tick = (index + 1) as u64;
        report.traces.push(driver.step(tick, frame)?);
    }
    report.gps_updates = driver.gps_updates();
    report.baro_updates = driver.baro_updates();
    report.rejected_baro_spikes = driver.rejected_baro_spikes();

    Ok(report)
}

struct SensorFixtureColumns {
    columns: Vec<String>,
}

impl SensorFixtureColumns {
    fn from_fixture(fixture: &Fixture) -> HarnessResult<Self> {
        let columns = Self {
            columns: fixture.columns.clone(),
        };
        for required in [
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "mag_valid",
            "mag_x",
            "mag_y",
            "mag_z",
            "gps_valid",
            "gps_n",
            "gps_e",
            "gps_d",
            "gps_vn",
            "gps_ve",
            "gps_vd",
            "baro_valid",
            "baro_down",
        ] {
            columns.index(required)?;
        }
        Ok(columns)
    }

    fn vec3(
        &self,
        sample: &crate::FixtureSample,
        tick: u64,
        label: &str,
        names: [&str; 3],
    ) -> HarnessResult<[f64; 3]> {
        let value = [
            self.scalar(sample, names[0])?,
            self.scalar(sample, names[1])?,
            self.scalar(sample, names[2])?,
        ];
        finite_f64x3(tick, sample.timestamp_us, label, value)?;
        Ok(value)
    }

    fn optional_vec3(
        &self,
        sample: &crate::FixtureSample,
        tick: u64,
        valid_name: &str,
        label: &str,
        names: [&str; 3],
    ) -> HarnessResult<Option<[f64; 3]>> {
        if !self.valid(sample, valid_name)? {
            return Ok(None);
        }

        self.vec3(sample, tick, label, names).map(Some)
    }

    fn optional_scalar(
        &self,
        sample: &crate::FixtureSample,
        tick: u64,
        valid_name: &str,
        name: &str,
    ) -> HarnessResult<Option<f64>> {
        if !self.valid(sample, valid_name)? {
            return Ok(None);
        }

        let value = self.scalar(sample, name)?;
        if value.is_finite() {
            Ok(Some(value))
        } else {
            Err(HarnessFailure::new(
                tick,
                sample.timestamp_us,
                format!("{name} contains a non-finite asserted value"),
            ))
        }
    }

    fn valid(&self, sample: &crate::FixtureSample, name: &str) -> HarnessResult<bool> {
        Ok(self.scalar(sample, name)? >= 0.5)
    }

    fn scalar(&self, sample: &crate::FixtureSample, name: &str) -> HarnessResult<f64> {
        let index = self.index(name)?;
        Ok(sample.values[index] as f64)
    }

    fn index(&self, name: &str) -> HarnessResult<usize> {
        let column_index = self
            .columns
            .iter()
            .position(|column| column == name)
            .ok_or_else(|| {
                HarnessFailure::new(0, 0, format!("sensor fixture missing column {name}"))
            })?;
        column_index.checked_sub(1).ok_or_else(|| {
            HarnessFailure::new(
                0,
                0,
                format!("sensor fixture column {name} cannot be timestamp_us"),
            )
        })
    }
}

fn finite_f64x3(tick: u64, timestamp_us: u64, label: &str, values: [f64; 3]) -> HarnessResult<()> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(HarnessFailure::new(
            tick,
            timestamp_us,
            format!("{label} contains non-finite values: {values:?}"),
        ))
    }
}
