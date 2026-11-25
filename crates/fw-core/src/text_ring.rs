//! The STATUSTEXT ring shared by the ground-station links.
//!
//! [`DEPTH`] (8) entries in order of arrival (another depth through the const parameter `N`) and
//! one read cursor per link: every text reaches each link once, in order. When a link falls more
//! than `N` texts behind (it is slow, or down), the oldest
//! texts are dropped for that link only and counted; a link that never reads holds back neither the
//! writer nor the other links. Two sticky slots (the boot report) are kept until every link has
//! taken them, so a link that connects later still gets them; a link takes its sticky texts first,
//! oldest first, then the ring's.

/// Entries in the ring unless `N` says otherwise.
pub const DEPTH: usize = 8;
/// Sticky slots (the boot report).
pub const STICKY: usize = 2;

/// The ring of `N` entries with a cursor for each of `LINKS` links (link ids `0..LINKS`).
pub struct TextRing<T, const LINKS: usize, const N: usize = DEPTH> {
    entries: [Option<T>; N],
    /// Sequence number of the next text written.
    head: u32,
    /// Per link: the sequence number of the next text it reads.
    cursors: [u32; LINKS],
    /// Per link: texts dropped before it read them.
    dropped: [u32; LINKS],
    /// Sticky texts with their sequence number among the sticky texts.
    sticky: [Option<(u32, T)>; STICKY],
    /// Per sticky slot: the links that have not taken it yet.
    owed: [[bool; LINKS]; STICKY],
    sticky_next: u32,
}

impl<T: Copy, const LINKS: usize, const N: usize> TextRing<T, LINKS, N> {
    /// An empty ring.
    pub const fn new() -> Self {
        Self::starting_at(0)
    }

    const N_U32: u32 = N as u32;

    const fn index(seq: u32) -> usize {
        (seq % Self::N_U32) as usize
    }

    const fn starting_at(seq: u32) -> Self {
        Self {
            entries: [const { None }; N],
            head: seq,
            cursors: [seq; LINKS],
            dropped: [0; LINKS],
            sticky: [const { None }; STICKY],
            owed: [[false; LINKS]; STICKY],
            sticky_next: 0,
        }
    }

    /// Appends a text for every link. A link already `N` texts behind loses its oldest unread
    /// text (counted in [`Self::dropped`]). Returns the number of links that lost one.
    pub fn push(&mut self, text: T) -> usize {
        if let Some(slot) = self.entries.get_mut(Self::index(self.head)) {
            *slot = Some(text);
        }
        self.head = self.head.wrapping_add(1);
        let mut losers = 0;
        for (cursor, dropped) in self.cursors.iter_mut().zip(self.dropped.iter_mut()) {
            let lag = self.head.wrapping_sub(*cursor);
            if lag > Self::N_U32 {
                let over = lag - Self::N_U32;
                *cursor = cursor.wrapping_add(over);
                *dropped = dropped.saturating_add(over);
                losers += 1;
            }
        }
        losers
    }

    /// Keeps a text in a sticky slot until every link has taken it. Returns false (nothing kept)
    /// when both slots are still owed to some link.
    pub fn push_sticky(&mut self, text: T) -> bool {
        let Some(free) = self.sticky.iter().position(Option::is_none) else {
            return false;
        };
        if let (Some(slot), Some(owed)) = (self.sticky.get_mut(free), self.owed.get_mut(free)) {
            *slot = Some((self.sticky_next, text));
            *owed = [true; LINKS];
            self.sticky_next = self.sticky_next.wrapping_add(1);
        }
        true
    }

    /// The next text for `link`: its oldest sticky text not yet taken, else the next ring entry.
    /// `None` when it has read everything (or `link` is not below `LINKS`).
    pub fn pop(&mut self, link: usize) -> Option<T> {
        if let Some(text) = self.pop_sticky(link) {
            return Some(text);
        }
        let cursor = self.cursors.get_mut(link)?;
        if *cursor == self.head {
            return None;
        }
        let text = self.entries.get(Self::index(*cursor)).copied().flatten();
        *cursor = cursor.wrapping_add(1);
        text
    }

    fn pop_sticky(&mut self, link: usize) -> Option<T> {
        let next = self.sticky_next;
        let oldest = self
            .sticky
            .iter()
            .zip(self.owed.iter())
            .enumerate()
            .filter_map(|(i, (slot, owed))| match (slot, owed.get(link)) {
                (Some((seq, _)), Some(true)) => Some((i, next.wrapping_sub(*seq))),
                _ => None,
            })
            .max_by_key(|&(_, age)| age)
            .map(|(i, _)| i)?;
        let (slot, owed) = (self.sticky.get_mut(oldest)?, self.owed.get_mut(oldest)?);
        let text = slot.map(|(_, text)| text);
        if let Some(flag) = owed.get_mut(link) {
            *flag = false;
        }
        if owed.iter().all(|&flag| !flag) {
            *slot = None;
        }
        text
    }

    /// Whether `link` has a text to read (sticky or ring).
    pub fn pending(&self, link: usize) -> bool {
        let sticky = self.owed.iter().any(|owed| owed.get(link) == Some(&true));
        sticky || self.cursors.get(link).is_some_and(|&c| c != self.head)
    }

    /// Texts `link` lost because it was more than `N` behind (0 for an unknown link).
    pub fn dropped(&self, link: usize) -> u32 {
        self.dropped.get(link).copied().unwrap_or(0)
    }
}

impl<T: Copy, const LINKS: usize, const N: usize> Default for TextRing<T, LINKS, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::vec::Vec;

    fn drain(ring: &mut TextRing<u32, 2>, link: usize) -> Vec<u32> {
        let mut out = Vec::new();
        while let Some(t) = ring.pop(link) {
            out.push(t);
            assert!(out.len() <= DEPTH + STICKY, "pop never ends");
        }
        out
    }

    #[test]
    fn every_text_reaches_each_link_once_in_order() {
        let mut ring = TextRing::<u32, 2>::new();
        for t in 1..=5 {
            assert_eq!(ring.push(t), 0);
        }
        assert_eq!(drain(&mut ring, 0), [1, 2, 3, 4, 5]);
        assert_eq!(drain(&mut ring, 0), []);
        for t in 6..=8 {
            ring.push(t);
        }
        assert_eq!(ring.pop(1), Some(1));
        assert_eq!(drain(&mut ring, 0), [6, 7, 8]);
        assert_eq!(drain(&mut ring, 1), [2, 3, 4, 5, 6, 7, 8]);
        assert!(!ring.pending(0) && !ring.pending(1));
        assert_eq!((ring.dropped(0), ring.dropped(1)), (0, 0));
    }

    #[test]
    fn overflow_drops_the_oldest_for_the_slow_link_and_counts_it() {
        let mut ring = TextRing::<u32, 2>::new();
        for t in 1..=12 {
            let losers = ring.push(t);
            assert_eq!(losers, usize::from(t > 8), "push {t}");
            assert_eq!(ring.pop(0), Some(t));
        }
        assert_eq!(drain(&mut ring, 1), [5, 6, 7, 8, 9, 10, 11, 12]);
        assert_eq!((ring.dropped(0), ring.dropped(1)), (0, 4));
    }

    #[test]
    fn a_deeper_ring_holds_n_texts_for_the_slow_link() {
        let mut ring = TextRing::<u32, 2, 32>::new();
        for t in 1..=40 {
            ring.push(t);
            assert_eq!(ring.pop(0), Some(t));
        }
        let mut out = Vec::new();
        while let Some(t) = ring.pop(1) {
            out.push(t);
        }
        assert_eq!(out, (9..=40).collect::<Vec<_>>());
        assert_eq!(ring.dropped(1), 8);
    }

    #[test]
    fn sticky_texts_survive_until_every_link_took_them() {
        let mut ring = TextRing::<u32, 2>::new();
        assert!(ring.push_sticky(100));
        for t in 1..=20 {
            ring.push(t);
        }
        assert_eq!(ring.pop(0), Some(100));
        assert_eq!(drain(&mut ring, 0), [13, 14, 15, 16, 17, 18, 19, 20]);
        assert!(ring.push_sticky(101));
        // Both slots are still owed to link 1: a third sticky text is refused.
        assert!(!ring.push_sticky(102));
        assert_eq!(ring.pop(0), Some(101));
        assert!(!ring.push_sticky(102));
        // The link that connects late gets both sticky texts first, oldest first.
        assert_eq!(
            drain(&mut ring, 1),
            [100, 101, 13, 14, 15, 16, 17, 18, 19, 20]
        );
        assert!(ring.push_sticky(102) && ring.push_sticky(103));
        assert!(!ring.push_sticky(104));
    }

    #[test]
    fn sticky_order_holds_when_a_slot_is_reused() {
        let mut ring = TextRing::<u32, 2>::new();
        assert!(ring.push_sticky(1));
        assert!(ring.push_sticky(2));
        assert_eq!(ring.pop(0), Some(1));
        assert_eq!(ring.pop(1), Some(1));
        // Slot 0 is free again; text 3 lands there but is younger than text 2 in slot 1.
        assert!(ring.push_sticky(3));
        assert_eq!(drain(&mut ring, 0), [2, 3]);
        assert_eq!(drain(&mut ring, 1), [2, 3]);
    }

    #[test]
    fn a_link_that_never_reads_does_not_stall_the_other() {
        let mut ring = TextRing::<u32, 2>::new();
        assert!(ring.push_sticky(9999));
        assert_eq!(ring.pop(0), Some(9999));
        for t in 0..1000 {
            ring.push(t);
            assert_eq!(ring.pop(0), Some(t));
            assert!(!ring.pending(0));
        }
        assert!(ring.pending(1));
        assert_eq!(ring.dropped(1), 1000 - DEPTH as u32);
        assert_eq!(ring.pop(1), Some(9999));
        assert_eq!(drain(&mut ring, 1), (992..1000).collect::<Vec<_>>());
    }

    #[test]
    fn unknown_link_reads_nothing() {
        let mut ring = TextRing::<u32, 2>::new();
        ring.push(1);
        assert!(ring.push_sticky(2));
        assert_eq!(ring.pop(2), None);
        assert!(!ring.pending(2));
        assert_eq!(ring.dropped(2), 0);
    }

    #[test]
    fn sequence_numbers_wrap() {
        let mut ring = TextRing::<u32, 2>::starting_at(u32::MAX - 3);
        for t in 1..=11 {
            ring.push(t);
            assert_eq!(ring.pop(0), Some(t));
        }
        assert_eq!(drain(&mut ring, 1), [4, 5, 6, 7, 8, 9, 10, 11]);
        assert_eq!(ring.dropped(1), 3);
    }

    /// The ring against a per-link queue model over a bounded pseudo-random run of pushes, sticky
    /// pushes and reads.
    #[test]
    fn matches_a_per_link_queue_model() {
        let mut ring = TextRing::<u32, 2>::new();
        let mut queues: [VecDeque<u32>; 2] = [VecDeque::new(), VecDeque::new()];
        let mut dropped = [0u32; 2];
        // Sticky texts still owed: (text, owed per link), oldest first.
        let mut sticky: Vec<(u32, [bool; 2])> = Vec::new();
        let mut seed: u32 = 0x1234_5678;
        for step in 0..20_000u32 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            match (seed >> 24) % 16 {
                0..=5 => {
                    ring.push(step);
                    for (q, d) in queues.iter_mut().zip(dropped.iter_mut()) {
                        q.push_back(step);
                        if q.len() > DEPTH {
                            q.pop_front();
                            *d += 1;
                        }
                    }
                }
                6 => {
                    let kept = ring.push_sticky(step);
                    assert_eq!(kept, sticky.len() < STICKY, "step {step}");
                    if kept {
                        sticky.push((step, [true; 2]));
                    }
                }
                op => {
                    let link = usize::from(op >= 11);
                    let expected = match sticky.iter().position(|(_, owed)| owed[link]) {
                        Some(i) => {
                            let entry = &mut sticky[i];
                            entry.1[link] = false;
                            let text = entry.0;
                            if entry.1 == [false; 2] {
                                sticky.remove(i);
                            }
                            Some(text)
                        }
                        None => queues[link].pop_front(),
                    };
                    assert_eq!(ring.pop(link), expected, "step {step}, link {link}");
                }
            }
            for link in 0..2 {
                let model = !queues[link].is_empty() || sticky.iter().any(|(_, o)| o[link]);
                assert_eq!(ring.pending(link), model, "step {step}, link {link}");
                assert_eq!(
                    ring.dropped(link),
                    dropped[link],
                    "step {step}, link {link}"
                );
            }
        }
    }
}
