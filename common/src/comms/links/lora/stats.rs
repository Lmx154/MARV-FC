#[derive(Clone, Copy, Debug, Default, defmt::Format, PartialEq, Eq)]
pub struct LoraLinkStats {
    pub tx_packets: u32,
    pub rx_packets: u32,
    pub tx_errors: u32,
    pub rx_errors: u32,
    pub malformed_frames: u32,
    pub unexpected_frames: u32,
    pub missed_peer_packets: u32,
    pub idle_timeouts: u32,
    pub rx_restarts: u32,
    pub radio_recoveries: u32,
    pub radio_recovery_failures: u32,
    pub last_rssi: i16,
    pub last_snr_x4: i16,
}

impl LoraLinkStats {
    pub fn note_tx_packet(&mut self) {
        self.tx_packets = self.tx_packets.wrapping_add(1);
    }

    pub fn note_tx_error(&mut self) {
        self.tx_errors = self.tx_errors.wrapping_add(1);
    }

    pub fn note_rx_packet(&mut self, rssi: i16, snr_x4: i16) {
        self.rx_packets = self.rx_packets.wrapping_add(1);
        self.last_rssi = rssi;
        self.last_snr_x4 = snr_x4;
    }

    pub fn note_rx_error(&mut self) {
        self.rx_errors = self.rx_errors.wrapping_add(1);
    }

    pub fn note_malformed_frame(&mut self) {
        self.malformed_frames = self.malformed_frames.wrapping_add(1);
    }

    pub fn note_unexpected_frame(&mut self) {
        self.unexpected_frames = self.unexpected_frames.wrapping_add(1);
    }

    pub fn note_missed_peer_packet(&mut self) {
        self.missed_peer_packets = self.missed_peer_packets.wrapping_add(1);
    }

    pub fn note_idle_timeout(&mut self) {
        self.idle_timeouts = self.idle_timeouts.wrapping_add(1);
    }

    pub fn note_rx_restart(&mut self) {
        self.rx_restarts = self.rx_restarts.wrapping_add(1);
    }

    pub fn note_radio_recovery(&mut self) {
        self.radio_recoveries = self.radio_recoveries.wrapping_add(1);
    }

    pub fn note_radio_recovery_failure(&mut self) {
        self.radio_recovery_failures = self.radio_recovery_failures.wrapping_add(1);
    }
}
