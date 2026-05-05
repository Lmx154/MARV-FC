use common::protocol::hilink;

pub const COMMAND_CORRELATION_DEPTH: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandCorrelation {
    pub normal_seq: u16,
    pub normal_msg_type: u8,
    pub command_seq: u16,
    pub command_id: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RadioStateCache {
    next_rf_command_seq: u16,
    next_normal_tx_seq: u16,
    correlation_cursor: usize,
    command_correlations: [Option<CommandCorrelation>; COMMAND_CORRELATION_DEPTH],
}

impl RadioStateCache {
    pub const fn new() -> Self {
        Self {
            next_rf_command_seq: 0,
            next_normal_tx_seq: 0,
            correlation_cursor: 0,
            command_correlations: [None; COMMAND_CORRELATION_DEPTH],
        }
    }

    pub fn next_rf_command_seq(&mut self) -> u16 {
        let seq = self.next_rf_command_seq;
        self.next_rf_command_seq = self.next_rf_command_seq.wrapping_add(1);
        seq
    }

    pub fn next_normal_tx_seq(&mut self) -> u16 {
        let seq = self.next_normal_tx_seq;
        self.next_normal_tx_seq = self.next_normal_tx_seq.wrapping_add(1);
        seq
    }

    pub fn store_command_correlation(
        &mut self,
        normal_header: hilink::Header,
        command_seq: u16,
        command_id: u16,
    ) {
        self.command_correlations[self.correlation_cursor] = Some(CommandCorrelation {
            normal_seq: normal_header.seq,
            normal_msg_type: normal_header.msg_type,
            command_seq,
            command_id,
        });
        self.correlation_cursor = (self.correlation_cursor + 1) % COMMAND_CORRELATION_DEPTH;
    }

    pub fn take_command_correlation(
        &mut self,
        command_id: u16,
        command_seq: u16,
    ) -> Option<CommandCorrelation> {
        for correlation in self.command_correlations.iter_mut() {
            let Some(stored) = *correlation else {
                continue;
            };
            if stored.command_id == command_id && stored.command_seq == command_seq {
                *correlation = None;
                return Some(stored);
            }
        }

        None
    }

    pub fn take_normal_correlation(
        &mut self,
        normal_seq: u16,
        normal_msg_type: u8,
    ) -> Option<CommandCorrelation> {
        for correlation in self.command_correlations.iter_mut() {
            let Some(stored) = *correlation else {
                continue;
            };
            if stored.normal_seq == normal_seq && stored.normal_msg_type == normal_msg_type {
                *correlation = None;
                return Some(stored);
            }
        }

        None
    }
}

impl Default for RadioStateCache {
    fn default() -> Self {
        Self::new()
    }
}
