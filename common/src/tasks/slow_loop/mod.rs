//! Slow-loop portable task bodies belong here.

pub mod sd_card_smoke;

pub use sd_card_smoke::run_sd_card_smoke_test;
