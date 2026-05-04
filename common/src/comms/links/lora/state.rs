use super::stats::LoraLinkStats;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum LoraLinkState {
    Acquiring,
    Linked,
    Degraded,
    Lost,
    RadioFault,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraLinkPolicy {
    pub degraded_after_misses: u8,
    pub lost_after_misses: u8,
    pub radio_fault_after_recovery_failures: u8,
}

impl LoraLinkPolicy {
    pub const fn new(
        degraded_after_misses: u8,
        lost_after_misses: u8,
        radio_fault_after_recovery_failures: u8,
    ) -> Self {
        Self {
            degraded_after_misses,
            lost_after_misses,
            radio_fault_after_recovery_failures,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraLinkHealth {
    state: LoraLinkState,
    policy: LoraLinkPolicy,
    linked_once: bool,
    consecutive_misses: u8,
    consecutive_recovery_failures: u8,
    stats: LoraLinkStats,
}

impl LoraLinkHealth {
    pub const fn new(policy: LoraLinkPolicy) -> Self {
        Self {
            state: LoraLinkState::Acquiring,
            policy,
            linked_once: false,
            consecutive_misses: 0,
            consecutive_recovery_failures: 0,
            stats: LoraLinkStats {
                tx_packets: 0,
                rx_packets: 0,
                tx_errors: 0,
                rx_errors: 0,
                malformed_frames: 0,
                unexpected_frames: 0,
                missed_peer_packets: 0,
                idle_timeouts: 0,
                rx_restarts: 0,
                radio_recoveries: 0,
                radio_recovery_failures: 0,
                last_rssi: 0,
                last_snr_x4: 0,
            },
        }
    }

    pub const fn state(&self) -> LoraLinkState {
        self.state
    }

    pub const fn consecutive_misses(&self) -> u8 {
        self.consecutive_misses
    }

    pub const fn linked_once(&self) -> bool {
        self.linked_once
    }

    pub const fn stats(&self) -> &LoraLinkStats {
        &self.stats
    }

    pub fn stats_mut(&mut self) -> &mut LoraLinkStats {
        &mut self.stats
    }

    pub fn note_valid_rx(&mut self, rssi: i16, snr_x4: i16) -> LoraLinkState {
        self.stats.note_rx_packet(rssi, snr_x4);
        self.linked_once = true;
        self.consecutive_misses = 0;
        self.consecutive_recovery_failures = 0;
        self.state = LoraLinkState::Linked;
        self.state
    }

    pub fn note_tx_packet(&mut self) {
        self.stats.note_tx_packet();
    }

    pub fn note_tx_error(&mut self) -> LoraLinkState {
        self.stats.note_tx_error();
        self.note_missed_peer_packet()
    }

    pub fn note_rx_error(&mut self) {
        self.stats.note_rx_error();
    }

    pub fn note_malformed_frame(&mut self) {
        self.stats.note_malformed_frame();
    }

    pub fn note_unexpected_frame(&mut self) {
        self.stats.note_unexpected_frame();
    }

    pub fn note_idle_timeout(&mut self) -> LoraLinkState {
        self.stats.note_idle_timeout();
        self.note_missed_peer_packet()
    }

    pub fn note_missed_peer_packet(&mut self) -> LoraLinkState {
        self.stats.note_missed_peer_packet();
        self.consecutive_misses = self.consecutive_misses.saturating_add(1);

        if self.state == LoraLinkState::RadioFault {
            return self.state;
        }

        if !self.linked_once {
            self.state = LoraLinkState::Acquiring;
        } else if self.consecutive_misses >= self.policy.lost_after_misses {
            self.state = LoraLinkState::Lost;
        } else if self.consecutive_misses >= self.policy.degraded_after_misses {
            self.state = LoraLinkState::Degraded;
        } else if self.state != LoraLinkState::Linked {
            self.state = LoraLinkState::Acquiring;
        }

        self.state
    }

    pub fn note_rx_restart(&mut self) {
        self.stats.note_rx_restart();
    }

    pub fn note_radio_recovery_ok(&mut self) -> LoraLinkState {
        self.stats.note_radio_recovery();
        self.consecutive_recovery_failures = 0;
        if self.state == LoraLinkState::RadioFault {
            self.state = if self.linked_once {
                LoraLinkState::Lost
            } else {
                LoraLinkState::Acquiring
            };
        }
        self.state
    }

    pub fn note_radio_recovery_failed(&mut self) -> LoraLinkState {
        self.stats.note_radio_recovery_failure();
        self.consecutive_recovery_failures = self.consecutive_recovery_failures.saturating_add(1);
        if self.consecutive_recovery_failures >= self.policy.radio_fault_after_recovery_failures {
            self.state = LoraLinkState::RadioFault;
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missed_packets_degrade_then_lose_link() {
        let mut health = LoraLinkHealth::new(LoraLinkPolicy::new(1, 3, 2));

        health.note_valid_rx(-80, 20);
        assert_eq!(health.state(), LoraLinkState::Linked);

        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Degraded);
        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Degraded);
        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Lost);
    }

    #[test]
    fn misses_before_first_contact_keep_acquiring() {
        let mut health = LoraLinkHealth::new(LoraLinkPolicy::new(1, 3, 2));

        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Acquiring);
        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Acquiring);
        assert_eq!(health.note_missed_peer_packet(), LoraLinkState::Acquiring);
    }

    #[test]
    fn recovery_failures_enter_radio_fault() {
        let mut health = LoraLinkHealth::new(LoraLinkPolicy::new(1, 3, 2));

        assert_eq!(
            health.note_radio_recovery_failed(),
            LoraLinkState::Acquiring
        );
        assert_eq!(
            health.note_radio_recovery_failed(),
            LoraLinkState::RadioFault
        );
    }
}
