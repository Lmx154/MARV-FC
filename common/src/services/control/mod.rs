//! Control service channel aliases.

use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel, Subscriber, WaitResult};

use crate::messages::control::ActuatorOutputStamped;

pub type ActuatorOutputChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    PubSubChannel<M, ActuatorOutputStamped, DEPTH, SUBS, PUBS>;
pub type ActuatorOutputPublisher<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    ImmediatePublisher<'a, M, ActuatorOutputStamped, DEPTH, SUBS, PUBS>;
pub type ActuatorOutputSubscriber<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    Subscriber<'a, M, ActuatorOutputStamped, DEPTH, SUBS, PUBS>;
pub type ActuatorOutputWaitResult = WaitResult<ActuatorOutputStamped>;
