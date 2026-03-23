//! Mode transition rules belong here.

mod init;

pub use init::{InitHilCommandDecision, evaluate_init_hil_command, phase_after_init};
