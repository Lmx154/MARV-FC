//! Runtime rules for accepting HIL semantic messages and publishing canonical samples.

use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, MagnetometerSample, MagnetometerSampleStamped, TimeSample,
};
use crate::services::hil::boot::UsbHilMode;
use crate::services::hil::model::{
    HilCommandAck, HilCommandAckResult, HilControlAction, HilControlCommand, HilIngressMessage,
    HilSessionState,
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
    session_state: HilSessionState,
    virtual_sensor_streaming_enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilControlRuntime {
    system_id: u8,
    component_id: u8,
    session_state: HilSessionState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilControlCommandOutcome {
    pub ack: Option<HilCommandAck>,
    pub session_state: Option<HilSessionState>,
}

impl HilControlRuntime {
    pub const fn new(system_id: u8, component_id: u8) -> Self {
        Self::with_session_state(system_id, component_id, HilSessionState::Inactive)
    }

    pub const fn with_session_state(
        system_id: u8,
        component_id: u8,
        session_state: HilSessionState,
    ) -> Self {
        Self {
            system_id,
            component_id,
            session_state,
        }
    }

    pub fn accept_control_command(
        &mut self,
        command: HilControlCommand,
    ) -> HilControlCommandOutcome {
        if !self.matches_target(command) {
            return HilControlCommandOutcome {
                ack: None,
                session_state: None,
            };
        }

        match command.action {
            HilControlAction::EnterHilMode => {
                let session_state = match self.session_state {
                    HilSessionState::Inactive => {
                        let next_state = HilSessionState::Limbo;
                        self.session_state = next_state;
                        Some(next_state)
                    }
                    HilSessionState::Limbo | HilSessionState::Selected(_) => None,
                };

                HilControlCommandOutcome {
                    ack: Some(HilCommandAck::new(
                        command.command_id,
                        HilCommandAckResult::Accepted,
                        command.source_system,
                        command.source_component,
                    )),
                    session_state,
                }
            }
            HilControlAction::SelectSubmode(submode) => {
                let (result, session_state) = match self.session_state {
                    HilSessionState::Inactive => (HilCommandAckResult::TemporarilyRejected, None),
                    HilSessionState::Limbo => {
                        let next_state = HilSessionState::Selected(submode);
                        self.session_state = next_state;
                        (HilCommandAckResult::Accepted, Some(next_state))
                    }
                    HilSessionState::Selected(current) if current == submode => {
                        (HilCommandAckResult::Accepted, None)
                    }
                    HilSessionState::Selected(_) => (HilCommandAckResult::Denied, None),
                };

                HilControlCommandOutcome {
                    ack: Some(HilCommandAck::new(
                        command.command_id,
                        result,
                        command.source_system,
                        command.source_component,
                    )),
                    session_state,
                }
            }
            HilControlAction::InvalidPayload => HilControlCommandOutcome {
                ack: Some(HilCommandAck::new(
                    command.command_id,
                    HilCommandAckResult::Unsupported,
                    command.source_system,
                    command.source_component,
                )),
                session_state: None,
            },
        }
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
            session_state: HilSessionState::Inactive,
            virtual_sensor_streaming_enabled: false,
        }
    }

    pub fn reset(&mut self) {
        self.last_tick_timestamp = None;
        self.last_time_boot_ms = None;
    }

    pub fn enable_virtual_sensor_streaming(&mut self) {
        if !self.virtual_sensor_streaming_enabled {
            self.virtual_sensor_streaming_enabled = true;
            self.reset();
        }
    }

    pub fn disable_virtual_sensor_streaming(&mut self) {
        if self.virtual_sensor_streaming_enabled {
            self.virtual_sensor_streaming_enabled = false;
            self.reset();
        }
    }

    pub fn set_session_state(&mut self, session_state: HilSessionState) {
        if self.session_state != session_state {
            self.session_state = session_state;
            self.reset();
        }
    }

    pub fn reconcile_usb_hil_mode(&mut self, mode: UsbHilMode) {
        if self.virtual_sensor_streaming_enabled {
            return;
        }

        let next_state = match mode {
            UsbHilMode::InitProbe | UsbHilMode::Passive => HilSessionState::Inactive,
            UsbHilMode::HilActive => match self.session_state {
                HilSessionState::Inactive => HilSessionState::Limbo,
                HilSessionState::Limbo | HilSessionState::Selected(_) => self.session_state,
            },
        };
        self.set_session_state(next_state);
    }

    pub const fn session_state(&self) -> HilSessionState {
        self.session_state
    }

    pub fn set_transitional_host_state_estimation_mode(&mut self) {
        self.enable_virtual_sensor_streaming();
    }

    fn accepts_virtual_sensor_data(&self) -> bool {
        self.virtual_sensor_streaming_enabled || self.session_state.accepts_data()
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
        if !matches!(message, HilIngressMessage::ControlCommand(_))
            && !self.accepts_virtual_sensor_data()
        {
            return HilDispatch::default();
        }

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
    use super::{HilControlRuntime, HilRuntime};
    use crate::messages::sensor::{ImuSample, ImuSampleStamped, TimeSample};
    use crate::services::hil::boot::UsbHilMode;
    use crate::services::hil::model::{
        HilCommandAckResult, HilControlCommand, HilIngressMessage, HilSessionState, HilSubmode,
        HilTick,
    };
    use crate::services::hil::routing::{HilImuRoute, HilIngressRoutes, HilTimeRoute};
    use crate::utilities::time::MeasurementTimestamp;

    #[derive(Default)]
    struct TestTimeRoute {
        last_tick: Option<TimeSample>,
    }

    impl HilTimeRoute for core::cell::RefCell<TestTimeRoute> {
        fn publish_time(&self, sample: TimeSample) {
            self.borrow_mut().last_tick = Some(sample);
        }
    }

    #[derive(Default)]
    struct TestImuRoute {
        last_sample: Option<ImuSampleStamped>,
    }

    impl HilImuRoute for core::cell::RefCell<TestImuRoute> {
        fn publish_imu(&self, sample: ImuSampleStamped) {
            self.borrow_mut().last_sample = Some(sample);
        }
    }

    #[test]
    fn control_runtime_accepts_enter_hil_mode_command() {
        let mut runtime = HilControlRuntime::new(42, 3);
        let outcome = runtime
            .accept_control_command(HilControlCommand::enter_hil_mode(31_010, 7, 9, 42, 3, 0));

        let ack = outcome.ack.unwrap();
        assert_eq!(ack.command_id, 31_010);
        assert_eq!(ack.result, HilCommandAckResult::Accepted);
        assert_eq!(outcome.session_state, Some(HilSessionState::Limbo));
    }

    #[test]
    fn control_runtime_selects_submode_from_limbo() {
        let mut runtime = HilControlRuntime::with_session_state(42, 3, HilSessionState::Limbo);
        let outcome = runtime.accept_control_command(HilControlCommand::select_submode(
            31_011,
            HilSubmode::FullRun,
            7,
            9,
            42,
            3,
            0,
        ));

        let ack = outcome.ack.unwrap();
        assert_eq!(ack.command_id, 31_011);
        assert_eq!(ack.result, HilCommandAckResult::Accepted);
        assert_eq!(
            outcome.session_state,
            Some(HilSessionState::Selected(HilSubmode::FullRun))
        );
    }

    #[test]
    fn control_runtime_denies_submode_switch_after_selection() {
        let mut runtime = HilControlRuntime::with_session_state(
            42,
            3,
            HilSessionState::Selected(HilSubmode::FullRun),
        );
        let outcome = runtime.accept_control_command(HilControlCommand::select_submode(
            31_011,
            HilSubmode::StateEstimation,
            7,
            9,
            42,
            3,
            0,
        ));

        assert_eq!(outcome.ack.unwrap().result, HilCommandAckResult::Denied);
        assert_eq!(outcome.session_state, None);
    }

    #[test]
    fn control_runtime_ignores_commands_for_other_targets() {
        let mut runtime = HilControlRuntime::with_session_state(42, 3, HilSessionState::Limbo);
        let outcome = runtime.accept_control_command(HilControlCommand::select_submode(
            31_011,
            HilSubmode::FullRun,
            7,
            9,
            24,
            3,
            0,
        ));

        assert!(outcome.ack.is_none());
    }

    #[test]
    fn control_runtime_acks_invalid_payload_as_unsupported() {
        let mut runtime = HilControlRuntime::with_session_state(42, 3, HilSessionState::Limbo);
        let outcome = runtime.accept_control_command(HilControlCommand {
            command_id: 31_011,
            action: crate::services::hil::model::HilControlAction::InvalidPayload,
            source_system: 7,
            source_component: 9,
            target_system: 42,
            target_component: 3,
            confirmation: 0,
        });

        assert_eq!(
            outcome.ack.unwrap().result,
            HilCommandAckResult::Unsupported
        );
        assert_eq!(outcome.session_state, None);
    }

    #[test]
    fn hil_runtime_promotes_hil_phase_to_limbo_once() {
        let mut runtime = HilRuntime::new();
        runtime.reconcile_usb_hil_mode(UsbHilMode::HilActive);
        assert_eq!(runtime.session_state(), HilSessionState::Limbo);

        runtime.set_session_state(HilSessionState::Selected(HilSubmode::FullRun));
        runtime.reconcile_usb_hil_mode(UsbHilMode::HilActive);
        assert_eq!(
            runtime.session_state(),
            HilSessionState::Selected(HilSubmode::FullRun)
        );
    }

    #[test]
    fn hil_runtime_demotes_non_hil_modes_to_inactive() {
        let mut runtime = HilRuntime::new();
        runtime.set_session_state(HilSessionState::Selected(HilSubmode::StateEstimation));

        runtime.reconcile_usb_hil_mode(UsbHilMode::Passive);
        assert_eq!(runtime.session_state(), HilSessionState::Inactive);
    }

    #[test]
    fn hil_runtime_drops_sensor_data_until_submode_is_selected() {
        let time = core::cell::RefCell::new(TestTimeRoute::default());
        let imu = core::cell::RefCell::new(TestImuRoute::default());
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut runtime = HilRuntime::new();
        runtime.set_session_state(HilSessionState::Limbo);

        let dispatch = runtime.accept(
            HilIngressMessage::Tick(HilTick {
                timestamp: MeasurementTimestamp::from_micros(1_000),
                time_boot_ms: 1,
            }),
            &routes,
        );
        assert!(dispatch.tick.is_none());
        assert!(time.borrow().last_tick.is_none());
    }

    #[test]
    fn hil_runtime_accepts_sensor_data_when_virtual_streaming_is_enabled() {
        let time = core::cell::RefCell::new(TestTimeRoute::default());
        let imu = core::cell::RefCell::new(TestImuRoute::default());
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut runtime = HilRuntime::new();
        runtime.enable_virtual_sensor_streaming();

        let tick = runtime.accept(
            HilIngressMessage::Tick(HilTick {
                timestamp: MeasurementTimestamp::from_micros(1_000),
                time_boot_ms: 1,
            }),
            &routes,
        );

        assert_eq!(tick.tick.unwrap().time_boot_ms, 1);
        assert_eq!(time.borrow().last_tick.unwrap().time_boot_ms, 1);
    }

    #[test]
    fn hil_runtime_routes_sensor_data_after_submode_selection() {
        let time = core::cell::RefCell::new(TestTimeRoute::default());
        let imu = core::cell::RefCell::new(TestImuRoute::default());
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut runtime = HilRuntime::new();
        runtime.set_session_state(HilSessionState::Selected(HilSubmode::StateEstimation));

        let tick = runtime.accept(
            HilIngressMessage::Tick(HilTick {
                timestamp: MeasurementTimestamp::from_micros(1_000),
                time_boot_ms: 1,
            }),
            &routes,
        );
        assert_eq!(tick.tick.unwrap().time_boot_ms, 1);
        assert_eq!(time.borrow().last_tick.unwrap().time_boot_ms, 1);

        let imu_dispatch = runtime.accept(
            HilIngressMessage::ImuSample(crate::services::hil::model::HilImuSample {
                timestamp: MeasurementTimestamp::from_micros(2_000),
                accel_mps2: [1.0, 2.0, 3.0],
                gyro_rad_s: [4.0, 5.0, 6.0],
            }),
            &routes,
        );
        assert!(imu_dispatch.imu_published);
        assert_eq!(
            imu.borrow().last_sample.unwrap(),
            ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(2_000),
                ImuSample {
                    accel_mps2: [1.0, 2.0, 3.0],
                    gyro_rad_s: [4.0, 5.0, 6.0],
                },
            )
        );
    }
}
