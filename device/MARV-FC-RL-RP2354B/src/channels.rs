#![allow(dead_code)]

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelId {
    FastSensorSample,
    EnvironmentalSample,
    AuxiliaryNavigationSample,
    LoggingRecord,
    FcRadioTraffic,
    CompanionTraffic,
    WatchdogStatus,
}

pub struct ChannelTopology {
    pub core0_local: &'static [ChannelId],
    pub core1_local: &'static [ChannelId],
    pub cross_core_bridges: &'static [ChannelId],
}

const CORE0_LOCAL: &[ChannelId] = &[
    ChannelId::FastSensorSample,
    ChannelId::EnvironmentalSample,
    ChannelId::AuxiliaryNavigationSample,
    ChannelId::LoggingRecord,
    ChannelId::WatchdogStatus,
];

const CORE1_LOCAL: &[ChannelId] = &[ChannelId::FcRadioTraffic, ChannelId::CompanionTraffic];

const CROSS_CORE_BRIDGES: &[ChannelId] = &[];

pub const TOPOLOGY: ChannelTopology = ChannelTopology {
    core0_local: CORE0_LOCAL,
    core1_local: CORE1_LOCAL,
    cross_core_bridges: CROSS_CORE_BRIDGES,
};
