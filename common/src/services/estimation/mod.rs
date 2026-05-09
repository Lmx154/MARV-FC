//! Estimation service channel aliases.

use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel, Subscriber, WaitResult};

use crate::messages::estimate::StateEstimateStamped;

pub type StateEstimateChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    PubSubChannel<M, StateEstimateStamped, DEPTH, SUBS, PUBS>;
pub type StateEstimatePublisher<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    ImmediatePublisher<'a, M, StateEstimateStamped, DEPTH, SUBS, PUBS>;
pub type StateEstimateSubscriber<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    Subscriber<'a, M, StateEstimateStamped, DEPTH, SUBS, PUBS>;
pub type StateEstimateWaitResult = WaitResult<StateEstimateStamped>;
