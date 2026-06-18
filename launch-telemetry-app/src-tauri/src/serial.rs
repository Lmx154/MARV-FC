//! The serial worker thread: owns the serial port, reads bytes, decodes HILink frames into the
//! aggregator, and emits telemetry/debug/link-status events to the UI at a steady rate.
//!
//! A single dedicated OS thread owns the blocking serial IO (keeping it off Tauri's async
//! runtime). It drains control messages, pumps the port, and emits a merged `TelemetryPacket`
//! on a fixed ~15 Hz tick so the differently-rated source messages become one coherent stream.

use std::io::{ErrorKind, Read, Write};
use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::time::{Duration, Instant};

use marv_hilink::{
    encode_packet, ArmPayload, DisarmPayload, MotorStopPayload, PingPayload, SetIdleFallbackPayload,
    SetRadioProfilePayload, WirePayload,
};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use tauri::{AppHandle, Emitter};

use crate::aggregator::TelemetryAggregator;
use crate::decode::{decode_frame, ParsedMessage, PARSED_CLASS_COUNT};
use crate::framing::FrameAccumulator;
use crate::model::{DebugKind, DebugLine, LinkStatus};
use crate::state::{CommandKind, ControlMsg};

const TELEMETRY_EVENT: &str = "telemetry";
const DEBUG_EVENT: &str = "debug";
const LINK_STATUS_EVENT: &str = "link-status";

const EMIT_INTERVAL: Duration = Duration::from_millis(66); // ~15 Hz UI push
const PARSED_LOG_INTERVAL: Duration = Duration::from_millis(500); // throttle PARSED log lines
const HOTPLUG_POLL_INTERVAL: Duration = Duration::from_millis(1_000);
const READ_TIMEOUT: Duration = Duration::from_millis(50);
const IDLE_RECV_TIMEOUT: Duration = Duration::from_millis(200);

/// Entry point for the worker thread.
pub fn run_worker(app: AppHandle, rx: Receiver<ControlMsg>) {
    Worker::new(app).run(rx);
}

struct Worker {
    app: AppHandle,
    port: Option<Box<dyn SerialPort>>,
    port_name: Option<String>,
    baud: u32,
    tx_seq: u16,
    acc: FrameAccumulator,
    agg: TelemetryAggregator,
    last_emit: Instant,
    last_parsed_log: [Instant; PARSED_CLASS_COUNT],
    last_hotplug_poll: Instant,
    last_dropped: usize,
}

impl Worker {
    fn new(app: AppHandle) -> Self {
        let now = Instant::now();
        Self {
            app,
            port: None,
            port_name: None,
            baud: 0,
            tx_seq: 0,
            acc: FrameAccumulator::new(),
            agg: TelemetryAggregator::new(),
            last_emit: now,
            last_parsed_log: [now; PARSED_CLASS_COUNT],
            last_hotplug_poll: now,
            last_dropped: 0,
        }
    }

    fn run(&mut self, rx: Receiver<ControlMsg>) {
        loop {
            if self.port.is_some() {
                // Connected: drain pending control messages without blocking.
                loop {
                    match rx.try_recv() {
                        Ok(msg) => self.handle(msg),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => return,
                    }
                }
            } else {
                // Idle: block (with timeout) so we don't spin while disconnected.
                match rx.recv_timeout(IDLE_RECV_TIMEOUT) {
                    Ok(msg) => self.handle(msg),
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            }

            if self.port.is_some() {
                self.pump_serial();
                self.maybe_hotplug();
                self.maybe_emit();
            }
        }
    }

    fn handle(&mut self, msg: ControlMsg) {
        match msg {
            ControlMsg::Connect { port, baud } => self.do_connect(port, baud),
            ControlMsg::Disconnect => self.handle_disconnect(None),
            ControlMsg::SendCommand(kind) => self.do_send_command(kind),
        }
    }

    fn do_connect(&mut self, port_name: String, baud: u32) {
        // Drop any existing connection silently before opening the new one.
        self.port = None;
        self.port_name = None;
        self.acc.reset();
        self.agg.reset();
        self.last_dropped = self.acc.dropped;

        match serialport::new(&port_name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(READ_TIMEOUT)
            .open()
        {
            Ok(port) => {
                self.port = Some(port);
                self.port_name = Some(port_name.clone());
                self.baud = baud;
                let now = Instant::now();
                self.last_emit = now;
                self.last_parsed_log = [now; PARSED_CLASS_COUNT];
                self.last_hotplug_poll = now;
                self.emit_debug(DebugKind::System, format!("connected to {port_name} @ {baud} 8N1"));
                self.emit_link_status(true, None);
            }
            Err(error) => {
                let text = format!("failed to open {port_name}: {error}");
                self.baud = baud;
                self.emit_debug(DebugKind::Error, text.clone());
                let _ = self.app.emit(
                    LINK_STATUS_EVENT,
                    &LinkStatus {
                        connected: false,
                        port: None,
                        baud,
                        last_error: Some(text),
                    },
                );
            }
        }
    }

    fn do_send_command(&mut self, kind: CommandKind) {
        let result = match kind {
            CommandKind::Arm => self.write_payload(&ArmPayload),
            CommandKind::Disarm => self.write_payload(&DisarmPayload),
            CommandKind::Ping => self.write_payload(&PingPayload),
            CommandKind::MotorStop => self.write_payload(&MotorStopPayload),
            CommandKind::SetRadioProfile {
                preset,
                tx_power_dbm,
                frequency_hz,
                flags,
            } => self.write_payload(&SetRadioProfilePayload {
                preset,
                tx_power_dbm,
                frequency_hz,
                flags,
            }),
            CommandKind::SetIdleFallback { idle_fallback_ms } => {
                self.write_payload(&SetIdleFallbackPayload { idle_fallback_ms })
            }
        };
        match result {
            Ok(()) => self.emit_debug(DebugKind::Link, format!("sent {kind:?} command")),
            Err(error) => {
                self.emit_debug(DebugKind::Error, format!("failed to send {kind:?}: {error}"))
            }
        }
    }

    fn write_payload<P: WirePayload>(&mut self, payload: &P) -> Result<(), String> {
        // Encode first (no port borrow), then write.
        let mut raw = [0u8; 192];
        let mut out = [0u8; 224];
        let seq = self.tx_seq;
        let len = encode_packet(payload, seq, 0, &mut raw, &mut out)
            .map_err(|e| format!("encode failed: {e:?}"))?;
        self.tx_seq = self.tx_seq.wrapping_add(1);

        let Some(port) = self.port.as_mut() else {
            return Err("not connected".to_string());
        };
        port.write_all(&out[..len]).map_err(|e| e.to_string())?;
        port.flush().map_err(|e| e.to_string())?;
        Ok(())
    }

    fn pump_serial(&mut self) {
        let bytes = match self.read_available() {
            Ok(bytes) => bytes,
            Err(error) => {
                self.handle_disconnect(Some(format!("read failed: {error}")));
                return;
            }
        };
        if bytes.is_empty() {
            return;
        }

        let frames = self.acc.push(&bytes);
        if self.acc.dropped != self.last_dropped {
            let dropped = self.acc.dropped - self.last_dropped;
            self.last_dropped = self.acc.dropped;
            self.emit_debug(
                DebugKind::Error,
                format!("dropped {dropped} oversized/garbage frame(s)"),
            );
        }

        for frame in frames {
            match decode_frame(&frame) {
                Ok(decoded) => {
                    let now = Instant::now();
                    let always_log = matches!(
                        decoded.message,
                        ParsedMessage::Ack(_) | ParsedMessage::Nack(_)
                    );
                    self.agg.ingest(&decoded, now);
                    // Throttle the console per message class (Ack/Nack always shown) so high-rate
                    // TELEM/IMU frames don't starve low-rate GPS/RADIO out of the log. Emit the raw
                    // bytes alongside the decoded interpretation so RAW and PARSED line up.
                    let key = decoded.message.class_key();
                    if always_log
                        || now.duration_since(self.last_parsed_log[key]) >= PARSED_LOG_INTERVAL
                    {
                        self.last_parsed_log[key] = now;
                        self.emit_debug(DebugKind::Raw, hex_frame(&frame));
                        self.emit_debug(DebugKind::Parsed, decoded.message.summary());
                    }
                }
                Err(error) => {
                    // A frame that fails to decode is exactly the "jibberish" worth seeing, so dump
                    // its bytes next to the error instead of swallowing them.
                    self.emit_debug(DebugKind::Raw, hex_frame(&frame));
                    self.emit_debug(DebugKind::Error, format!("decode error: {error:?}"));
                }
            }
        }
    }

    fn read_available(&mut self) -> Result<Vec<u8>, String> {
        let Some(port) = self.port.as_mut() else {
            return Ok(Vec::new());
        };
        let to_read = match port.bytes_to_read() {
            Ok(n) => n as usize,
            Err(error) => return Err(format!("inspect read buffer: {error}")),
        };
        if to_read == 0 {
            return Ok(Vec::new());
        }
        let mut buf = vec![0u8; to_read.min(8192)];
        match port.read(&mut buf) {
            Ok(read) => {
                buf.truncate(read);
                Ok(buf)
            }
            Err(error) if error.kind() == ErrorKind::TimedOut => Ok(Vec::new()),
            Err(error) => Err(format!("read bytes: {error}")),
        }
    }

    fn maybe_hotplug(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_hotplug_poll) < HOTPLUG_POLL_INTERVAL {
            return;
        }
        self.last_hotplug_poll = now;
        let Some(active) = self.port_name.clone() else {
            return;
        };

        let io_error = self.port.as_mut().and_then(|p| p.bytes_to_read().err());
        let missing = match serialport::available_ports() {
            Ok(ports) => !ports.iter().any(|p| p.port_name == active),
            Err(_) => false,
        };

        if io_error.is_some() || missing {
            let reason = if let Some(error) = io_error {
                format!("I/O probe failed: {error}")
            } else {
                "device no longer present".to_string()
            };
            self.handle_disconnect(Some(format!("{active} disconnected ({reason})")));
        }
    }

    fn maybe_emit(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_emit) < EMIT_INTERVAL {
            return;
        }
        self.last_emit = now;
        if !self.agg.has_data() {
            return;
        }
        let packet = self.agg.snapshot(now);
        let _ = self.app.emit(TELEMETRY_EVENT, &packet);
    }

    /// Tear down the connection. `reason = Some` is an error/hotplug drop; `None` is user-initiated.
    fn handle_disconnect(&mut self, reason: Option<String>) {
        let was_connected = self.port.is_some();
        self.port = None;
        self.port_name = None;
        self.acc.reset();
        self.agg.reset();

        if was_connected {
            match &reason {
                Some(text) => self.emit_debug(DebugKind::Error, text.clone()),
                None => self.emit_debug(DebugKind::System, "serial disconnected".to_string()),
            }
        }
        let _ = self.app.emit(
            LINK_STATUS_EVENT,
            &LinkStatus {
                connected: false,
                port: None,
                baud: self.baud,
                last_error: reason,
            },
        );
    }

    fn emit_debug(&self, kind: DebugKind, text: impl Into<String>) {
        let _ = self.app.emit(DEBUG_EVENT, &DebugLine::new(kind, text));
    }

    fn emit_link_status(&self, connected: bool, last_error: Option<String>) {
        let _ = self.app.emit(
            LINK_STATUS_EVENT,
            &LinkStatus {
                connected,
                port: self.port_name.clone(),
                baud: self.baud,
                last_error,
            },
        );
    }
}

/// Render a received frame as `"<N>B  aa bb cc …"` for the RAW debug filter. The frame still
/// carries its trailing `0x00` COBS delimiter, so the byte count matches what crossed the wire.
fn hex_frame(frame: &[u8]) -> String {
    use std::fmt::Write;
    let mut out = String::with_capacity(frame.len() * 3 + 6);
    let _ = write!(out, "{}B ", frame.len());
    for byte in frame {
        let _ = write!(out, " {byte:02x}");
    }
    out
}
