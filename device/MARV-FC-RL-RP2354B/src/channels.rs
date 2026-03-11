#![allow(dead_code)]

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelId {
    FastSensorSample,
    EnvironmentalSample,
    AuxiliaryNavigationSample,
    FcRadioTraffic,
    CompanionTraffic,
    WatchdogStatus,
}

pub struct ChannelTopology {
    pub core0_feed_critical: &'static [ChannelId],
    pub core1_background: &'static [ChannelId],
}

const CORE0_FEED_CRITICAL: &[ChannelId] = &[
    ChannelId::FastSensorSample,
    ChannelId::EnvironmentalSample,
    ChannelId::AuxiliaryNavigationSample,
    ChannelId::WatchdogStatus,
];

const CORE1_BACKGROUND: &[ChannelId] = &[ChannelId::FcRadioTraffic, ChannelId::CompanionTraffic];

pub const TOPOLOGY: ChannelTopology = ChannelTopology {
    core0_feed_critical: CORE0_FEED_CRITICAL,
    core1_background: CORE1_BACKGROUND,
};
