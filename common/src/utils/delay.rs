#![allow(async_fn_in_trait)]
/// Simple async delay trait so common can remain HAL-agnostic
pub trait DelayMs {
    /// Delay for approximately the given number of milliseconds
    async fn delay_ms(&mut self, ms: u32);
}
