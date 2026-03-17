//! Generic stamped sensor pub-sub channel aliases.

use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel, Subscriber, WaitResult};

use crate::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, MagnetometerSampleStamped,
};

pub type SampleChannel<M, T, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    PubSubChannel<M, T, DEPTH, SUBS, PUBS>;
pub type SamplePublisher<'a, M, T, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    ImmediatePublisher<'a, M, T, DEPTH, SUBS, PUBS>;
pub type SampleSubscriber<'a, M, T, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    Subscriber<'a, M, T, DEPTH, SUBS, PUBS>;
pub type SampleWaitResult<T> = WaitResult<T>;

pub type BarometerSampleChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SampleChannel<M, BarometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type BarometerSamplePublisher<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SamplePublisher<'a, M, BarometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type BarometerSampleSubscriber<
    'a,
    M,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
> = SampleSubscriber<'a, M, BarometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type BarometerSampleWaitResult = SampleWaitResult<BarometerSampleStamped>;

pub type MagnetometerSampleChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SampleChannel<M, MagnetometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type MagnetometerSamplePublisher<
    'a,
    M,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
> = SamplePublisher<'a, M, MagnetometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type MagnetometerSampleSubscriber<
    'a,
    M,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
> = SampleSubscriber<'a, M, MagnetometerSampleStamped, DEPTH, SUBS, PUBS>;
pub type MagnetometerSampleWaitResult = SampleWaitResult<MagnetometerSampleStamped>;

pub type GpsFixSampleChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SampleChannel<M, GpsFixSampleStamped, DEPTH, SUBS, PUBS>;
pub type GpsFixSamplePublisher<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SamplePublisher<'a, M, GpsFixSampleStamped, DEPTH, SUBS, PUBS>;
pub type GpsFixSampleSubscriber<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    SampleSubscriber<'a, M, GpsFixSampleStamped, DEPTH, SUBS, PUBS>;
pub type GpsFixSampleWaitResult = SampleWaitResult<GpsFixSampleStamped>;
