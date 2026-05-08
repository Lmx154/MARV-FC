//! Reusable async task bodies for watchdog supervision.

use crate::messages::runtime::FlightPhase;
use crate::services::health::{
    FeedDecision, LivenessUpdate, WatchdogContract, WatchdogEvaluation, WatchdogSource,
    WatchdogSupervisor,
};
use defmt::info;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Ticker};

pub trait WatchdogDriver {
    fn start(&mut self);
    fn feed(&mut self);
}

pub trait LivenessUpdateReceiver {
    fn try_next_liveness(&mut self) -> Option<LivenessUpdate>;
}

impl<'a, Mutex, const N: usize> LivenessUpdateReceiver
    for embassy_sync::channel::Receiver<'a, Mutex, LivenessUpdate, N>
where
    Mutex: RawMutex,
{
    fn try_next_liveness(&mut self) -> Option<LivenessUpdate> {
        self.try_receive().ok()
    }
}

pub trait FlightPhaseSource {
    fn try_next_phase(&mut self) -> Option<FlightPhase>;
}

impl<'a, Mutex, const CAP: usize, const SUBS: usize, const PUBS: usize> FlightPhaseSource
    for embassy_sync::pubsub::Subscriber<'a, Mutex, FlightPhase, CAP, SUBS, PUBS>
where
    Mutex: RawMutex,
{
    fn try_next_phase(&mut self) -> Option<FlightPhase> {
        self.try_next_message_pure()
    }
}

#[allow(async_fn_in_trait)]
pub trait PureSubscriber {
    async fn next_pure_message(&mut self);
}

impl<'a, Mutex, Message, const CAP: usize, const SUBS: usize, const PUBS: usize> PureSubscriber
    for embassy_sync::pubsub::Subscriber<'a, Mutex, Message, CAP, SUBS, PUBS>
where
    Mutex: RawMutex,
    Message: Clone,
{
    async fn next_pure_message(&mut self) {
        let _ = self.next_message_pure().await;
    }
}

pub async fn run_watchdog_liveness_loop<Subscriber, Report>(
    mut subscriber: Subscriber,
    mask: u32,
    mut report_progress: Report,
) -> !
where
    Subscriber: PureSubscriber,
    Report: FnMut(u32),
{
    loop {
        subscriber.next_pure_message().await;
        report_progress(mask);
    }
}

pub async fn run_watchdog_supervisor_loop<
    Hardware,
    Updates,
    Phases,
    ContractForPhase,
    Now,
    const N: usize,
>(
    mut hardware: Hardware,
    mut updates: Updates,
    mut phases: Phases,
    sources: [WatchdogSource; N],
    init_contract: WatchdogContract,
    mut contract_for_phase: ContractForPhase,
    watchdog_enabled_in_hil: bool,
    mut now_ms: Now,
) -> !
where
    Hardware: WatchdogDriver,
    Updates: LivenessUpdateReceiver,
    Phases: FlightPhaseSource,
    ContractForPhase: FnMut(FlightPhase) -> WatchdogContract,
    Now: FnMut() -> u64,
{
    let mut phase = FlightPhase::Init;
    let mut supervisor = WatchdogSupervisor::<N>::new(sources, init_contract, now_ms());
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut last_decision = None;

    hardware.start();

    loop {
        ticker.next().await;

        while let Some(next_phase) = phases.try_next_phase() {
            phase = next_phase;
            supervisor.set_contract(contract_for_phase(phase), now_ms());
            info!("watchdog phase -> {:?}", phase);
            if phase.is_hil() && !watchdog_enabled_in_hil {
                info!("watchdog HIL liveness disabled by config");
            }
        }

        while let Some(update) = updates.try_next_liveness() {
            supervisor.record(update);
        }

        let evaluation = if phase.is_fault() {
            WatchdogEvaluation {
                decision: FeedDecision::DoNotFeed,
                stale_required_mask: 0,
                stale_degrade_mask: 0,
            }
        } else if phase.is_hil() && !watchdog_enabled_in_hil {
            WatchdogEvaluation {
                decision: FeedDecision::FeedAllowed,
                stale_required_mask: 0,
                stale_degrade_mask: 0,
            }
        } else {
            supervisor.evaluate(now_ms())
        };

        if last_decision != Some(evaluation.decision) {
            info!(
                "watchdog decision={:?} stale_required=0x{=u32:08X} stale_degrade=0x{=u32:08X}",
                evaluation.decision, evaluation.stale_required_mask, evaluation.stale_degrade_mask
            );
            last_decision = Some(evaluation.decision);
        }

        if !matches!(evaluation.decision, FeedDecision::DoNotFeed) {
            hardware.feed();
        }
    }
}
