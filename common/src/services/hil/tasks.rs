//! Reusable async task bodies for HIL ingress and egress loops.

use crate::services::hil::egress::{HilByteWriter, HilEgressProtocol};
use crate::services::hil::ingress::{HilByteReader, HilIngressProtocol};
use crate::services::hil::model::{HilControlCommand, HilEgressMessage};
use crate::services::hil::routing::{
    HilBarometerRoute, HilControlCommandRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes,
    HilMagnetometerRoute, HilTimeRoute,
};
use crate::services::hil::runtime::{HilControlRuntime, HilDispatch, HilRuntime};
use embassy_sync::blocking_mutex::raw::RawMutex;

#[allow(async_fn_in_trait)]
pub trait HilControlCommandReceiver {
    async fn receive_hil_control_command(&mut self) -> HilControlCommand;
}

impl<'a, Mutex, const N: usize> HilControlCommandReceiver
    for embassy_sync::channel::Receiver<'a, Mutex, HilControlCommand, N>
where
    Mutex: RawMutex,
{
    async fn receive_hil_control_command(&mut self) -> HilControlCommand {
        self.receive().await
    }
}

#[allow(async_fn_in_trait)]
pub trait HilEgressSender {
    async fn send_hil_egress(&mut self, message: HilEgressMessage);
}

impl<'a, Mutex, const N: usize> HilEgressSender
    for embassy_sync::channel::Sender<'a, Mutex, HilEgressMessage, N>
where
    Mutex: RawMutex,
{
    async fn send_hil_egress(&mut self, message: HilEgressMessage) {
        self.send(message).await;
    }
}

pub trait HilEgressTrySender {
    fn try_send_hil_egress(&self, message: HilEgressMessage) -> bool;
}

impl<'a, Mutex, const N: usize> HilEgressTrySender
    for embassy_sync::channel::Sender<'a, Mutex, HilEgressMessage, N>
where
    Mutex: RawMutex,
{
    fn try_send_hil_egress(&self, message: HilEgressMessage) -> bool {
        self.try_send(message).is_ok()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HilIngressLoopError<TransportError, DispatchError> {
    Transport(TransportError),
    Dispatch(DispatchError),
}

pub async fn run_hil_control_command_loop<Receiver, Egress>(
    mut receiver: Receiver,
    mut egress: Egress,
    mut runtime: HilControlRuntime,
) -> !
where
    Receiver: HilControlCommandReceiver,
    Egress: HilEgressSender,
{
    loop {
        let command = receiver.receive_hil_control_command().await;
        if let Some(ack) = runtime.accept_control_command(command) {
            egress
                .send_hil_egress(HilEgressMessage::CommandAck(ack))
                .await;
        }
    }
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
