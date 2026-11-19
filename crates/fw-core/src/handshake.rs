//! The arm/save handshake.
//!
//! Design: decision-foundation, two-core amendment, "Item 3, changes: the save, and arming".
//! Arming and saving can never both win. Two flags: `ARMED`, stored only by the flight core (set and cleared by the
//! gate, and cleared by the ESC task on a stale spinning command; both run on the flight core), and `SAVE_INTENT`,
//! stored only by core 0. Each side stores its own flag, then loads the other's, and backs out (clears its own flag) if the other is set. Every access is `SeqCst`: this is the
//! store-then-load (Dekker) pattern, where release/acquire would let both loads read the old
//! `false` (each store still in its core's write buffer); `SeqCst` puts the four accesses in one
//! total order, so whichever load comes second in it sees the other side's store.
//!
//! Each `begin_save`/`try_arm` is built from two separately callable steps (announce: store own
//! flag; confirm: load the other's and decide) so tests can interleave them. Both sides may lose
//! in the same race (each sees the other's flag before it is withdrawn); neither retries here.
//! [`Handshake::armed`] can read `true` for the short time between `announce_arm` and a
//! `confirm_arm` that backs out.

use core::sync::atomic::{AtomicBool, Ordering::SeqCst};

/// The two flags of the arm/save handshake; one `static` instance is shared by both cores.
pub struct Handshake {
    armed: AtomicBool,
    save_intent: AtomicBool,
}

impl Handshake {
    /// Disarmed, no save intent.
    pub const fn new() -> Self {
        Self {
            armed: AtomicBool::new(false),
            save_intent: AtomicBool::new(false),
        }
    }

    /// Core 0, save step 1: stores `SAVE_INTENT`.
    pub fn announce_save(&self) {
        self.save_intent.store(true, SeqCst);
    }

    /// Core 0, save step 2: loads `ARMED`; `true` = may save (call [`Handshake::end_save`]
    /// when done); `false` = armed, `SAVE_INTENT` withdrawn, do not save.
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

    /// Flight core, arm step 2: loads `SAVE_INTENT`; `true` = armed; `false` = a save is in
    /// progress, `ARMED` withdrawn.
    pub fn confirm_arm(&self) -> bool {
        if self.save_intent.load(SeqCst) {
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
    }
    use Step::{ArmDecide, ArmStore, SaveDecide, SaveStore};

    /// The six orders of two two-step sequences.
    const ORDERS: [[Step; 4]; 6] = [
        [ArmStore, ArmDecide, SaveStore, SaveDecide],
        [ArmStore, SaveStore, ArmDecide, SaveDecide],
        [ArmStore, SaveStore, SaveDecide, ArmDecide],
        [SaveStore, ArmStore, ArmDecide, SaveDecide],
        [SaveStore, ArmStore, SaveDecide, ArmDecide],
        [SaveStore, SaveDecide, ArmStore, ArmDecide],
    ];

    #[test]
    fn all_six_interleavings_never_both_win() {
        let mut outcomes = Vec::new();
        for order in ORDERS {
            let hs = Handshake::new();
            let (mut armed, mut saving) = (false, false);
            for step in order {
                match step {
                    ArmStore => hs.announce_arm(),
                    ArmDecide => armed = hs.confirm_arm(),
                    SaveStore => hs.announce_save(),
                    SaveDecide => saving = hs.confirm_save(),
                }
            }
            assert!(!(armed && saving), "{order:?}");
            assert_eq!(hs.armed(), armed, "{order:?}");
            assert_eq!(hs.save_intent.load(SeqCst), saving, "{order:?}");
            outcomes.push((armed, saving));
        }
        std::println!("handshake interleavings (armed, saving): {outcomes:?}");
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
