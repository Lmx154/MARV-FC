//! Protocol adapters that translate inbound transport bytes into HIL semantic messages.

use core::future::Future;

use crate::services::hil::model::HilIngressMessage;

pub trait HilIngressProtocol {
    type Error;

    fn reset(&mut self);
    fn ingest_bytes(&mut self, bytes: &[u8]);
    fn try_next_message(&mut self) -> Option<Result<HilIngressMessage, Self::Error>>;
}

pub trait HilByteReader {
    type Error;

    fn read<'a>(
        &'a mut self,
        buffer: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, Self::Error>> + 'a;
}
