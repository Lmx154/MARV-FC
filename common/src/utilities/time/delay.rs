#![allow(async_fn_in_trait)]

//! Simple async delay trait so `common` remains HAL-agnostic.

pub trait DelayMs {
    async fn delay_ms(&mut self, ms: u32);
}
