//! Runtime rules for accepting HIL semantic messages and publishing canonical samples.

use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, MagnetometerSample, MagnetometerSampleStamped, TimeSample,
};
use crate::services::hil::backend::SensorBackend;
use crate::services::hil::model::{
    HilCommandAck, HilCommandAckResult, HilControlAction, HilControlCommand, HilIngressMessage,
};
use crate::services::hil::routing::{
    HilBarometerRoute, HilControlCommandRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes,
    HilMagnetometerRoute, HilTimeRoute,
};
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilDispatch {
    pub tick: Option<TimeSample>,
    pub imu_published: bool,
    pub barometer_published: bool,
    pub gps_published: bool,
    pub magnetometer_published: bool,
    pub control_command_published: bool,
}

impl HilDispatch {
    pub fn merge(&mut self, other: Self) {
        if self.tick.is_none() {
            self.tick = other.tick;
        }
        self.imu_published |= other.imu_published;
        self.barometer_published |= other.barometer_published;
        self.gps_published |= other.gps_published;
        self.magnetometer_published |= other.magnetometer_published;
        self.control_command_published |= other.control_command_published;
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HilRuntime {
    last_tick_timestamp: Option<MeasurementTimestamp>,
    last_time_boot_ms: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilControlRuntime {
    system_id: u8,
    component_id: u8,
    active_backend: SensorBackend,
}

impl HilControlRuntime {
    pub const fn new(system_id: u8, component_id: u8, active_backend: SensorBackend) -> Self {
        Self {
            system_id,
            component_id,
            active_backend,
        }
    }

    pub fn accept_control_command(&mut self, command: HilControlCommand) -> Option<HilCommandAck> {
        if !self.matches_target(command) {
            return None;
        }

        let result = match command.action {
            HilControlAction::RequestBackend(requested_backend)
                if requested_backend == self.active_backend =>
            {
                HilCommandAckResult::Accepted
            }
            HilControlAction::RequestBackend(_) => HilCommandAckResult::Denied,
        };

        Some(HilCommandAck::new(
            command.command_id,
            result,
            command.source_system,
            command.source_component,
        ))
    }

    const fn matches_target(&self, command: HilControlCommand) -> bool {
        let system_matches = command.target_system == 0 || command.target_system == self.system_id;
        let component_matches =
            command.target_component == 0 || command.target_component == self.component_id;

        system_matches && component_matches
    }
}

impl HilRuntime {
    pub const fn new() -> Self {
        Self {
            last_tick_timestamp: None,
            last_time_boot_ms: None,
        }
    }

    pub fn reset(&mut self) {
        self.last_tick_timestamp = None;
        self.last_time_boot_ms = None;
    }

    pub fn accept<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        message: HilIngressMessage,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> HilDispatch
    where
        Time: HilTimeRoute,
        Imu: HilImuRoute,
        Barometer: HilBarometerRoute,
        Gps: HilGpsRoute,
        Magnetometer: HilMagnetometerRoute,
        Control: HilControlCommandRoute,
    {
        match message {
            HilIngressMessage::Tick(tick) => {
                self.accept_tick(tick.timestamp, tick.time_boot_ms, routes)
            }
            HilIngressMessage::ImuSample(sample) => {
                routes.imu.publish_imu(ImuSampleStamped::new(
                    sample.timestamp,
                    ImuSample {
                        accel_mps2: sample.accel_mps2,
                        gyro_rad_s: sample.gyro_rad_s,
                    },
                ));
                HilDispatch {
                    imu_published: true,
                    ..Default::default()
                }
            }
            HilIngressMessage::BarometerSample(sample) => {
                routes.barometer.publish_barometer(BarometerSampleStamped {
                    timestamp: sample.timestamp,
                    sample: BarometerSample {
                        pressure_pa: sample.pressure_pa,
                        temperature_c: sample.temperature_c,
                    },
                });
                HilDispatch {
                    barometer_published: true,
                    ..Default::default()
                }
            }
            HilIngressMessage::GpsSample(sample) => {
                routes.gps.publish_gps(GpsFixSampleStamped {
                    timestamp: sample.timestamp,
                    sample: GpsFixSample {
                        lat_deg: sample.lat_deg,
                        lon_deg: sample.lon_deg,
                        alt_m: sample.alt_m,
                        vel_ned_mps: sample.vel_ned_mps,
                        sats: sample.sats,
                    },
                });
                HilDispatch {
                    gps_published: true,
                    ..Default::default()
                }
            }
            HilIngressMessage::MagnetometerSample(sample) => {
                routes
                    .magnetometer
                    .publish_magnetometer(MagnetometerSampleStamped {
                        timestamp: sample.timestamp,
                        sample: MagnetometerSample {
                            field_ut: sample.field_ut,
                        },
                    });
                HilDispatch {
                    magnetometer_published: true,
                    ..Default::default()
                }
            }
            HilIngressMessage::ControlCommand(command) => {
                routes.control.publish_control_command(command);
                HilDispatch {
                    control_command_published: true,
                    ..Default::default()
                }
            }
        }
    }

    fn accept_tick<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        timestamp: MeasurementTimestamp,
        time_boot_ms: u32,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> HilDispatch
    where
        Time: HilTimeRoute,
    {
        if self
            .last_time_boot_ms
            .is_some_and(|previous| time_boot_ms <= previous)
        {
            return HilDispatch::default();
        }
        if self
            .last_tick_timestamp
            .is_some_and(|previous| timestamp <= previous)
        {
            return HilDispatch::default();
        }

        self.last_time_boot_ms = Some(time_boot_ms);
        self.last_tick_timestamp = Some(timestamp);

        let tick = TimeSample {
            timestamp,
            time_boot_ms,
        };
        routes.time.publish_time(tick);
        HilDispatch {
            tick: Some(tick),
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::HilControlRuntime;
    use crate::services::hil::backend::SensorBackend;
    use crate::services::hil::model::{HilCommandAckResult, HilControlCommand};

    #[test]
    fn control_runtime_accepts_commands_for_current_backend() {
        let mut runtime = HilControlRuntime::new(42, 3, SensorBackend::Hil);
        let ack = runtime
            .accept_control_command(HilControlCommand::request_backend(
                31_010,
                SensorBackend::Hil,
                7,
                9,
                42,
                3,
                0,
            ))
            .unwrap();

        assert_eq!(ack.command_id, 31_010);
        assert_eq!(ack.result, HilCommandAckResult::Accepted);
        assert_eq!(ack.target_system, 7);
        assert_eq!(ack.target_component, 9);
    }

    #[test]
    fn control_runtime_denies_backend_switch_it_cannot_apply() {
        let mut runtime = HilControlRuntime::new(42, 3, SensorBackend::Hil);
        let ack = runtime
            .accept_control_command(HilControlCommand::request_backend(
                31_010,
                SensorBackend::Real,
                7,
                9,
                42,
                3,
                0,
            ))
            .unwrap();

        assert_eq!(ack.result, HilCommandAckResult::Denied);
    }

    #[test]
    fn control_runtime_ignores_commands_for_other_targets() {
        let mut runtime = HilControlRuntime::new(42, 3, SensorBackend::Hil);
        let ack = runtime.accept_control_command(HilControlCommand::request_backend(
            31_010,
            SensorBackend::Hil,
            7,
            9,
            24,
            3,
            0,
        ));

        assert!(ack.is_none());
    }
}
