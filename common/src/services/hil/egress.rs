//! Protocol adapters that translate portable HIL outputs into transport frames.

use core::future::Future;

use crate::services::hil::model::HilEgressMessage;

pub trait HilEgressProtocol {
    type EncodedFrame: AsRef<[u8]>;
    type Error;

    fn encode_message(
        &mut self,
        message: HilEgressMessage,
    ) -> Result<Option<Self::EncodedFrame>, Self::Error>;
}

pub trait HilByteWriter {
    type Error;

    fn write<'a>(
        &'a mut self,
        bytes: &'a [u8],
    ) -> impl Future<Output = Result<(), Self::Error>> + 'a;
}
