//! Reusable async task bodies for HIL ingress and egress loops.

use crate::services::hil::egress::{HilByteWriter, HilEgressProtocol};
use crate::services::hil::ingress::{HilByteReader, HilIngressProtocol};
use crate::services::hil::model::HilEgressMessage;
use crate::services::hil::routing::{
    HilBarometerRoute, HilControlCommandRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes,
    HilMagnetometerRoute, HilTimeRoute,
};
use crate::services::hil::runtime::{HilDispatch, HilRuntime};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HilIngressLoopError<TransportError, DispatchError> {
    Transport(TransportError),
    Dispatch(DispatchError),
}

pub async fn run_hil_ingress_loop<
    Reader,
    Protocol,
    Time,
    Imu,
    Barometer,
    Gps,
    Magnetometer,
    Control,
    OnDispatch,
    DispatchError,
    OnProtocolError,
    const BUFFER: usize,
>(
    reader: &mut Reader,
    protocol: &mut Protocol,
    runtime: &mut HilRuntime,
    routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    read_buffer: &mut [u8; BUFFER],
    mut on_dispatch: OnDispatch,
    mut on_protocol_error: OnProtocolError,
) -> Result<(), HilIngressLoopError<Reader::Error, DispatchError>>
where
    Reader: HilByteReader,
    Protocol: HilIngressProtocol,
    Time: HilTimeRoute,
    Imu: HilImuRoute,
    Barometer: HilBarometerRoute,
    Gps: HilGpsRoute,
    Magnetometer: HilMagnetometerRoute,
    Control: HilControlCommandRoute,
    OnDispatch: FnMut(HilDispatch) -> Result<(), DispatchError>,
    OnProtocolError: FnMut(Protocol::Error),
{
    loop {
        let len = reader
            .read(read_buffer)
            .await
            .map_err(HilIngressLoopError::Transport)?;
        if len == 0 {
            continue;
        }

        protocol.ingest_bytes(&read_buffer[..len]);

        while let Some(message) = protocol.try_next_message() {
            match message {
                Ok(message) => on_dispatch(runtime.accept(message, routes))
                    .map_err(HilIngressLoopError::Dispatch)?,
                Err(error) => on_protocol_error(error),
            }
        }
    }
}

pub async fn send_hil_egress_message<Writer, Protocol>(
    writer: &mut Writer,
    protocol: &mut Protocol,
    message: HilEgressMessage,
) -> Result<(), Writer::Error>
where
    Writer: HilByteWriter,
    Protocol: HilEgressProtocol,
{
    if let Ok(Some(frame)) = protocol.encode_message(message) {
        writer.write(frame.as_ref()).await?;
    }

    Ok(())
}
