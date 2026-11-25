//! The arm/save handshake: arming and saving can never both win.
//!
//! Flags: `ARMED` (stored only by the flight core), `SAVE_INTENT` and `PERMIT` (stored only by core 0).
//! Each side stores its own flag, then loads the other's and backs out if it is set. Every access is `SeqCst`:
//! this is the store-then-load (Dekker) pattern, where release/acquire would let both loads read the old value.
//! Each `begin_save`/`try_arm` is two callable steps (announce, confirm) so tests can interleave them.
//! Both sides may lose the same race; neither retries here. [`Handshake::armed`] can read `true`
//! between `announce_arm` and a `confirm_arm` that backs out.
//!
//! `PERMIT` (initially false) is the pre-arm permit; arming also needs it set. Arming and a core-0 action never overlap:
//! - Admission against the gate (different cores): the same Dekker pair (admission stores `PERMIT` false, then loads `ARMED`).
//! - Admission against the supervisor tick (both on core 0): admission is synchronous and the tick reads the action's
//!   state and stores `PERMIT` inside one critical section, so each is atomic relative to the other.
//!
//! The flight core loads `PERMIT` only in [`Handshake::confirm_arm`], so clearing it while armed changes nothing.

use core::sync::atomic::{AtomicBool, Ordering::SeqCst};

/// The flags of the arm/save handshake and the pre-arm permit; one `static` instance is shared by
/// both cores.
pub struct Handshake {
    armed: AtomicBool,
    save_intent: AtomicBool,
    permit: AtomicBool,
}

impl Handshake {
    /// Disarmed, no save intent, no permit.
    pub const fn new() -> Self {
        Self {
            armed: AtomicBool::new(false),
            save_intent: AtomicBool::new(false),
            permit: AtomicBool::new(false),
        }
    }

    /// Core 0, save step 1: stores `SAVE_INTENT`.
    pub fn announce_save(&self) {
        self.save_intent.store(true, SeqCst);
    }

    /// Core 0, save step 2: loads `ARMED`; `true` = may save (call [`Handshake::end_save`] when done); `false` = armed, `SAVE_INTENT` withdrawn, do not save.
    pub fn confirm_save(&self) -> bool {
        if self.armed.load(SeqCst) {
            self.save_intent.store(false, SeqCst);
            false
        } else {
            true
        }
    }

    /// Core 0: both save steps; `true` = may save.
    pub fn begin_save(&self) -> bool {
        self.announce_save();
        self.confirm_save()
    }

    /// Core 0: the save is over; clears `SAVE_INTENT`.
    pub fn end_save(&self) {
        self.save_intent.store(false, SeqCst);
    }

    /// Flight core, arm step 1: stores `ARMED`.
    pub fn announce_arm(&self) {
        self.armed.store(true, SeqCst);
    }

    /// Flight core, arm step 2: loads `SAVE_INTENT` and `PERMIT`; `true` = armed; `false` = a save is in progress or core 0 does not permit arming, `ARMED` withdrawn.
    pub fn confirm_arm(&self) -> bool {
        if self.save_intent.load(SeqCst) || !self.permit.load(SeqCst) {
            self.armed.store(false, SeqCst);
            false
        } else {
            true
        }
    }

    /// Flight core: both arm steps; `true` = armed.
    pub fn try_arm(&self) -> bool {
        self.announce_arm();
        self.confirm_arm()
    }

    /// Flight core: clears `ARMED`.
    pub fn disarm(&self) {
        self.armed.store(false, SeqCst);
    }

    /// Whether `ARMED` is set.
    pub fn armed(&self) -> bool {
        self.armed.load(SeqCst)
    }

    /// Core 0: stores `PERMIT` (the supervisor tick, from the pre-arm reasons; `false` at every command admission before it loads `ARMED`).
    pub fn set_permit(&self, permit: bool) {
        self.permit.store(permit, SeqCst);
    }

    /// Whether `PERMIT` is set.
    pub fn permitted(&self) -> bool {
        self.permit.load(SeqCst)
    }
}

impl Default for Handshake {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::AtomicU32;
    use std::thread;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug)]
    enum Step {
        ArmStore,
        ArmDecide,
        SaveStore,
        SaveDecide,
        /// Admission: `set_permit(false)`.
        PermitClear,
        /// Admission: loads `ARMED`; not armed: the action starts (its state is set).
        AdmitDecide,
        /// The tick: reads the action's state.
        TickRead,
        /// The tick: stores the permit from what it read.
        TickStore,
    }
    use Step::{
        AdmitDecide, ArmDecide, ArmStore, PermitClear, SaveDecide, SaveStore, TickRead, TickStore,
    };

    /// The six orders of two two-step sequences.
    const ORDERS: [[Step; 4]; 6] = [
        [ArmStore, ArmDecide, SaveStore, SaveDecide],
        [ArmStore, SaveStore, ArmDecide, SaveDecide],
        [ArmStore, SaveStore, SaveDecide, ArmDecide],
        [SaveStore, ArmStore, ArmDecide, SaveDecide],
        [SaveStore, ArmStore, SaveDecide, ArmDecide],
        [SaveStore, SaveDecide, ArmStore, ArmDecide],
    ];

    /// The pre-arm model's atoms.
    const GATE: [&[Step]; 2] = [&[ArmStore], &[ArmDecide]];
    const ADMIT: [&[Step]; 2] = [&[PermitClear], &[AdmitDecide]];
    /// The tick: one atom (its critical section reads the action's state and stores the permit).
    const TICK: &[&[Step]] = &[&[TickRead, TickStore]];

    /// Every merge of the gate's, admission's and the tick's atoms in which no tick atom lies between admission's two (admission is atomic relative to the tick).
    fn merges(tick: &[&'static [Step]]) -> Vec<Vec<Step>> {
        fn go(left: [usize; 3], actors: &mut Vec<usize>, all: &mut Vec<Vec<usize>>) {
            if left == [0; 3] {
                all.push(actors.clone());
            }
            for actor in 0..3 {
                if left[actor] > 0 {
                    let mut l = left;
                    l[actor] -= 1;
                    actors.push(actor);
                    go(l, actors, all);
                    actors.pop();
                }
            }
        }
        let mut all = Vec::new();
        go(
            [GATE.len(), ADMIT.len(), tick.len()],
            &mut Vec::new(),
            &mut all,
        );
        all.into_iter()
            .filter_map(|actors| {
                let mut next = [0usize; 3];
                let mut steps = Vec::new();
                for actor in actors {
                    if actor == 2 && next[1] == 1 {
                        return None;
                    }
                    let atom = match actor {
                        0 => GATE[next[0]],
                        1 => ADMIT[next[1]],
                        _ => tick[next[2]],
                    };
                    next[actor] += 1;
                    steps.extend_from_slice(atom);
                }
                Some(steps)
            })
            .collect()
    }

    #[test]
    fn all_six_interleavings_never_both_win() {
        let mut outcomes = Vec::new();
        for order in ORDERS {
            let hs = Handshake::new();
            hs.set_permit(true);
            let (mut armed, mut saving) = (false, false);
            for step in order {
                match step {
                    ArmStore => hs.announce_arm(),
                    ArmDecide => armed = hs.confirm_arm(),
                    SaveStore => hs.announce_save(),
                    SaveDecide => saving = hs.confirm_save(),
                    PermitClear | AdmitDecide | TickRead | TickStore => unreachable!(),
                }
            }
            assert!(!(armed && saving), "{order:?}");
            assert_eq!(hs.armed(), armed, "{order:?}");
            assert_eq!(hs.save_intent.load(SeqCst), saving, "{order:?}");
            outcomes.push((armed, saving));
        }
        std::println!("handshake interleavings (armed, saving): {outcomes:?}");

        let orders = merges(TICK);
        let mut outcomes = Vec::new();
        for order in &orders {
            let hs = Handshake::new();
            hs.set_permit(true);
            let (mut armed, mut action, mut seen) = (false, false, false);
            for step in order {
                match step {
                    ArmStore => hs.announce_arm(),
                    ArmDecide => armed = hs.confirm_arm(),
                    PermitClear => hs.set_permit(false),
                    AdmitDecide => action = !hs.armed(),
                    TickRead => seen = action,
                    TickStore => hs.set_permit(!seen),
                    SaveStore | SaveDecide => unreachable!(),
                }
            }
            assert!(!(armed && action), "armed during an action: {order:?}");
            assert_eq!(hs.armed(), armed, "{order:?}");
            outcomes.push((armed, action));
        }
        std::println!(
            "pre-arm interleavings: {} orders, (armed, action): {outcomes:?}",
            orders.len()
        );
    }

    #[test]
    fn negative_control_load_before_store_both_win() {
        let hs = Handshake::new();
        let arm_saw_intent = hs.save_intent.load(SeqCst);
        let save_saw_armed = hs.armed.load(SeqCst);
        hs.armed.store(true, SeqCst);
        hs.save_intent.store(true, SeqCst);
        let (armed, saving) = (!arm_saw_intent, !save_saw_armed);
        assert!(armed && saving);
    }

    #[test]
    fn two_threads_never_both_win() {
        const ROUNDS: u32 = 1_000_000;
        let hs = Handshake::new();
        hs.set_permit(true);
        let go = AtomicU32::new(0);
        let arm_done = AtomicU32::new(0);
        let save_done = AtomicU32::new(0);
        let arm_won = AtomicBool::new(false);
        let save_won = AtomicBool::new(false);
        let side = |done: &AtomicU32, won: &AtomicBool, attempt: &dyn Fn() -> bool| {
            for round in 1..=ROUNDS {
                while go.load(SeqCst) != round {
                    core::hint::spin_loop();
                }
                won.store(attempt(), SeqCst);
                done.store(round, SeqCst);
            }
        };
        let (mut armed_only, mut saved_only, mut neither) = (0u32, 0u32, 0u32);
        thread::scope(|s| {
            s.spawn(|| side(&arm_done, &arm_won, &|| hs.try_arm()));
            s.spawn(|| side(&save_done, &save_won, &|| hs.begin_save()));
            for round in 1..=ROUNDS {
                hs.disarm();
                hs.end_save();
                go.store(round, SeqCst);
                while arm_done.load(SeqCst) != round || save_done.load(SeqCst) != round {
                    core::hint::spin_loop();
                }
                match (arm_won.load(SeqCst), save_won.load(SeqCst)) {
                    (true, true) => panic!("both won in round {round}"),
                    (true, false) => armed_only += 1,
                    (false, true) => saved_only += 1,
                    (false, false) => neither += 1,
                }
            }
        });
        std::println!(
            "handshake two threads: armed only {armed_only}, saved only {saved_only}, neither {neither}"
        );
        assert_eq!(armed_only + saved_only + neither, ROUNDS);
    }
}
