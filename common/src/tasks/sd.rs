//! Generic SD tasks.

use crate::sd::{SdCrud, SdError, SdStore, SdSpiDevice};

/// One-shot task: create `hello.txt` and write `hello world`.
pub async fn run_sd_hello_world_task<DEV>(store: &mut SdStore<DEV>) -> Result<(), SdError>
where
    DEV: SdSpiDevice,
{
    store.create("hello.txt", b"hello world")
}
