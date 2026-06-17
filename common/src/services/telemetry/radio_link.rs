//! Shared HILink radio-link telemetry emitter.
//!
//! Both the quadcopter (`MARV-FC-SP`) and rocket (`MARV-FC-RL`) flight controllers stream the same
//! periodic HILink telemetry to the ground radio over UART: a high-rate `TelemetrySnapshot` plus
//! lower-rate `Gps`, `Heartbeat`, and `SystemState` frames. The only per-vehicle differences are
//! the emit rates and how the flight state maps onto the wire `system_state` byte, so the loop body
//! lives here and each firmware injects its rates (via [`RadioTelemetryConfig`]) and a
//! `system_state` closure (e.g. the quad maps `FlightPhase::wire_code()` while the rocket maps a
//! `marv-hilink::flight_state` code).
//!
//! RX-originated replies (`Pong`/`Ack`/`Nack`/`ActuatorStatus`) are produced by device-local RX
//! handling and funnelled back through a [`RadioReply`] channel, so this loop stays the *single*
//! owner of the UART transmitter — no second writer, no outbound fan-in task.

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::pubsub::Subscriber;
use embassy_time::{Duration, Ticker};
use embedded_io_async::Write;

use crate::messages::runtime::FlightPhase;
use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, MagnetometerSample, MagnetometerSampleStamped,
};
use crate::protocol::hilink::{self, WirePayload};
use crate::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_m};

/// Encode scratch sized to the largest HILink frame a flight controller emits or replies with.
/// `HilSensorFrame` is the largest payload the FC handles, so it bounds every telemetry/reply frame.
const EMIT_FRAME_BYTES: usize = hilink::encoded_frame_len(hilink::HilSensorFrame::WIRE_LEN);

/// Errors surfaced by the emitter via its `on_error` callback.
#[derive(Clone, Copy, Debug, Eq, PartialEq, defmt::Format)]
pub enum RadioEmitError {
    /// HILink packet encoding failed (payload did not fit the scratch buffer).
    Encode,
    /// Writing the encoded frame to the UART transmitter failed.
    Write,
}

/// An RX-originated reply funnelled to the emitter so it remains the sole UART writer.
///
/// Built by each firmware's device-local RX handler from inbound commands and sent through a
/// [`Receiver`] the emitter drains. All variants carry `marv-hilink` payloads, so this stays
/// firmware-agnostic.
#[derive(Clone, Copy, Debug)]
pub enum RadioReply {
    /// Reply to a `Ping`; `peer_seq` is the sequence number being answered (for logging only).
    Pong { peer_seq: u16 },
    /// Acknowledge an accepted command.
    Ack(hilink::AckPayload),
    /// Reject a command.
    Nack(hilink::NackPayload),
    /// Actuator/bench status response.
    ActuatorStatus(hilink::ActuatorStatusPayload),
}

/// Per-vehicle emit rates and telemetry framing constants.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadioTelemetryConfig {
    /// `TelemetrySnapshot` emit period in ms (e.g. quad = 200 → 5 Hz, rocket = 10 → 100 Hz).
    pub snapshot_period_ms: u32,
    /// `Gps` emit period in ms.
    pub gps_period_ms: u32,
    /// `Heartbeat` + `SystemState` emit period in ms.
    pub status_period_ms: u32,
    /// Raw `Imu` (primary) + `AuxImu` (secondary) emit period in ms.
    pub imu_period_ms: u32,
    /// Raw `Mag` emit period in ms.
    pub mag_period_ms: u32,
    /// Raw `Baro` emit period in ms.
    pub baro_period_ms: u32,
    /// Sea-level reference pressure used for the pressure→altitude conversion.
    pub sea_level_pressure_pa: f32,
    /// `response_flags` bitset stamped into every emitted status/telemetry payload.
    pub flags: u32,
}

impl RadioTelemetryConfig {
    /// Build a config from the core emit periods, defaulting the sea-level reference to the
    /// standard atmosphere, the flags to `ESTIMATOR_VALID`, and the raw-sensor emit periods to
    /// the snapshot period (override with [`with_raw_sensor_periods`](Self::with_raw_sensor_periods)).
    #[must_use]
    pub const fn new(snapshot_period_ms: u32, gps_period_ms: u32, status_period_ms: u32) -> Self {
        Self {
            snapshot_period_ms,
            gps_period_ms,
            status_period_ms,
            imu_period_ms: snapshot_period_ms,
            mag_period_ms: snapshot_period_ms,
            baro_period_ms: snapshot_period_ms,
            sea_level_pressure_pa: STANDARD_SEA_LEVEL_PRESSURE_PA,
            flags: hilink::response_flags::ESTIMATOR_VALID,
        }
    }

    /// Set the raw `Imu`/`AuxImu`, `Mag`, and `Baro` emit periods (ms). These cap how often the
    /// FC pushes raw sensor frames over UART; the radio's airtime budget decimates further.
    #[must_use]
    pub const fn with_raw_sensor_periods(
        mut self,
        imu_period_ms: u32,
        mag_period_ms: u32,
        baro_period_ms: u32,
    ) -> Self {
        self.imu_period_ms = imu_period_ms;
        self.mag_period_ms = mag_period_ms;
        self.baro_period_ms = baro_period_ms;
        self
    }
}

fn sim_stamp(now_ms: u32) -> hilink::SimStamp {
    hilink::SimStamp {
        sim_tick: 0,
        sim_time_us: u64::from(now_ms) * 1_000,
    }
}

async fn send_payload<W, P>(
    tx: &mut W,
    seq: &mut u16,
    now_ms: u32,
    payload: &P,
) -> Result<(), RadioEmitError>
where
    W: Write,
    P: WirePayload,
{
    let mut raw = [0u8; EMIT_FRAME_BYTES];
    let mut encoded = [0u8; EMIT_FRAME_BYTES];
    let len = hilink::encode_packet(payload, *seq, now_ms, &mut raw, &mut encoded)
        .map_err(|_| RadioEmitError::Encode)?;
    tx.write_all(&encoded[..len])
        .await
        .map_err(|_| RadioEmitError::Write)?;
    *seq = seq.wrapping_add(1);
    Ok(())
}

async fn send_reply<W: Write>(
    tx: &mut W,
    seq: &mut u16,
    now_ms: u32,
    reply: RadioReply,
) -> Result<(), RadioEmitError> {
    match reply {
        RadioReply::Pong { .. } => send_payload(tx, seq, now_ms, &hilink::PongPayload).await,
        RadioReply::Ack(payload) => send_payload(tx, seq, now_ms, &payload).await,
        RadioReply::Nack(payload) => send_payload(tx, seq, now_ms, &payload).await,
        RadioReply::ActuatorStatus(payload) => send_payload(tx, seq, now_ms, &payload).await,
    }
}

fn telemetry_snapshot(
    now_ms: u32,
    system_state: u8,
    altitude_m: f32,
    flags: u32,
) -> hilink::TelemetrySnapshotPayload {
    hilink::TelemetrySnapshotPayload {
        stamp: sim_stamp(now_ms),
        system_state,
        reserved0: [0; 3],
        flags,
        position_ned_m: [0.0, 0.0, -altitude_m],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: 0.0,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
        accel_mag_cms2: 0,
    }
}

fn gps_payload(now_ms: u32, sample: Option<GpsFixSample>) -> hilink::GpsPayload {
    match sample {
        Some(sample) => hilink::GpsPayload {
            stamp: sim_stamp(now_ms),
            lat_deg: sample.lat_deg,
            lon_deg: sample.lon_deg,
            alt_msl_m: sample.alt_m,
            vel_ned_mps: sample.vel_ned_mps,
            sats: sample.sats,
            fix_type: sample.fix_type,
            reserved0: [0; 2],
        },
        None => hilink::GpsPayload {
            stamp: sim_stamp(now_ms),
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_msl_m: 0.0,
            vel_ned_mps: [0.0, 0.0, 0.0],
            sats: 0,
            fix_type: 0,
            reserved0: [0; 2],
        },
    }
}

/// Run the periodic HILink telemetry emitter, owning the UART transmitter for the program lifetime.
///
/// Drains the flight-phase, barometer, and GPS pub-sub subscribers for their latest values,
/// converts barometric pressure to altitude, and emits `Heartbeat`/`SystemState` (every
/// `status_period_ms`), `TelemetrySnapshot` (every `snapshot_period_ms`), and `Gps` (every
/// `gps_period_ms`). Between ticks it `select`s on the `replies` receiver so device-originated
/// `Pong`/`Ack`/`Nack`/`ActuatorStatus` frames are written promptly through the same transmitter.
///
/// `system_state` maps the current [`FlightPhase`] to the wire `system_state` byte — this is the
/// sole per-vehicle behavioural difference (quad codes vs. rocket `flight_state` codes).
#[allow(clippy::too_many_arguments)]
pub async fn run_radio_telemetry_emitter<
    M,
    Tx,
    NowFn,
    StateFn,
    ErrFn,
    const PHASE_DEPTH: usize,
    const PHASE_SUBS: usize,
    const PHASE_PUBS: usize,
    const BARO_DEPTH: usize,
    const BARO_SUBS: usize,
    const BARO_PUBS: usize,
    const GPS_DEPTH: usize,
    const GPS_SUBS: usize,
    const GPS_PUBS: usize,
    const IMU_DEPTH: usize,
    const IMU_SUBS: usize,
    const IMU_PUBS: usize,
    const AUX_IMU_DEPTH: usize,
    const AUX_IMU_SUBS: usize,
    const AUX_IMU_PUBS: usize,
    const MAG_DEPTH: usize,
    const MAG_SUBS: usize,
    const MAG_PUBS: usize,
    const REPLY_DEPTH: usize,
>(
    tx: &mut Tx,
    phase: &mut Subscriber<'_, M, FlightPhase, PHASE_DEPTH, PHASE_SUBS, PHASE_PUBS>,
    barometer: &mut Subscriber<'_, M, BarometerSampleStamped, BARO_DEPTH, BARO_SUBS, BARO_PUBS>,
    gps: &mut Subscriber<'_, M, GpsFixSampleStamped, GPS_DEPTH, GPS_SUBS, GPS_PUBS>,
    imu: &mut Subscriber<'_, M, ImuSampleStamped, IMU_DEPTH, IMU_SUBS, IMU_PUBS>,
    aux_imu: &mut Subscriber<'_, M, ImuSampleStamped, AUX_IMU_DEPTH, AUX_IMU_SUBS, AUX_IMU_PUBS>,
    magnetometer: &mut Subscriber<'_, M, MagnetometerSampleStamped, MAG_DEPTH, MAG_SUBS, MAG_PUBS>,
    replies: &Receiver<'_, M, RadioReply, REPLY_DEPTH>,
    config: RadioTelemetryConfig,
    mut now_ms: NowFn,
    mut system_state: StateFn,
    mut on_error: ErrFn,
) -> !
where
    M: RawMutex,
    Tx: Write,
    NowFn: FnMut() -> u32,
    StateFn: FnMut(FlightPhase) -> u8,
    ErrFn: FnMut(RadioEmitError),
{
    let snapshot_period_ms = config.snapshot_period_ms.max(1);
    let imu_period_ms = config.imu_period_ms.max(1);
    let mag_period_ms = config.mag_period_ms.max(1);
    let baro_period_ms = config.baro_period_ms.max(1);
    // Tick at the fastest configured cadence so each class can reach its own emit rate
    // regardless of the snapshot rate.
    let tick_ms = snapshot_period_ms
        .min(config.gps_period_ms.max(1))
        .min(config.status_period_ms.max(1))
        .min(imu_period_ms)
        .min(mag_period_ms)
        .min(baro_period_ms)
        .max(1);
    let mut seq = 0u16;
    let mut snapshots_sent = 0u32;
    let mut phase_state = FlightPhase::Init;
    let mut altitude_m = 0.0f32;
    let mut gps_fix: Option<GpsFixSample> = None;
    let mut imu_sample: Option<ImuSample> = None;
    let mut aux_imu_sample: Option<ImuSample> = None;
    let mut mag_sample: Option<MagnetometerSample> = None;
    let mut baro_sample: Option<BarometerSample> = None;
    let mut last_status_ms = now_ms().wrapping_sub(config.status_period_ms);
    let mut last_snapshot_ms = now_ms().wrapping_sub(snapshot_period_ms);
    let mut last_gps_ms = now_ms().wrapping_sub(config.gps_period_ms);
    let mut last_imu_ms = now_ms().wrapping_sub(imu_period_ms);
    let mut last_mag_ms = now_ms().wrapping_sub(mag_period_ms);
    let mut last_baro_ms = now_ms().wrapping_sub(baro_period_ms);
    let mut ticker = Ticker::every(Duration::from_millis(u64::from(tick_ms)));

    loop {
        match select(ticker.next(), replies.receive()).await {
            Either::Second(reply) => {
                let now = now_ms();
                if let Err(error) = send_reply(tx, &mut seq, now, reply).await {
                    on_error(error);
                }
            }
            Either::First(()) => {
                while let Some(next) = phase.try_next_message_pure() {
                    phase_state = next;
                }
                while let Some(sample) = barometer.try_next_message_pure() {
                    if let Some(next) =
                        pressure_altitude_m(sample.sample.pressure_pa, config.sea_level_pressure_pa)
                    {
                        altitude_m = next;
                    }
                    baro_sample = Some(sample.sample);
                }
                while let Some(sample) = gps.try_next_message_pure() {
                    gps_fix = Some(sample.sample);
                }
                while let Some(sample) = imu.try_next_message_pure() {
                    imu_sample = Some(sample.sample);
                }
                while let Some(sample) = aux_imu.try_next_message_pure() {
                    aux_imu_sample = Some(sample.sample);
                }
                while let Some(sample) = magnetometer.try_next_message_pure() {
                    mag_sample = Some(sample.sample);
                }

                let state = system_state(phase_state);
                let now = now_ms();

                if now.wrapping_sub(last_status_ms) >= config.status_period_ms {
                    last_status_ms = now;
                    // Periodic liveness: confirms the emitter loop is actually writing frames to
                    // the UART. If `snapshots_sent` stays at 0 the loop is stalled; if it climbs
                    // but the radio logs no "host uart hilink frame queued", the break is on the
                    // wire (FC TX ↔ radio RX wiring/baud), not in firmware.
                    info!(
                        "radio telemetry emitter alive: snapshots_sent={=u32} seq={=u16}",
                        snapshots_sent, seq
                    );
                    let heartbeat = hilink::HeartbeatPayload {
                        stamp: sim_stamp(now),
                        system_state: state,
                        reserved0: [0; 3],
                        flags: config.flags,
                    };
                    if let Err(error) = send_payload(tx, &mut seq, now, &heartbeat).await {
                        on_error(error);
                    }
                    let system = hilink::SystemStatePayload {
                        stamp: sim_stamp(now),
                        system_state: state,
                        reserved0: [0; 3],
                        flags: config.flags,
                        battery_voltage_v: 0.0,
                    };
                    if let Err(error) = send_payload(tx, &mut seq, now, &system).await {
                        on_error(error);
                    }
                }

                if now.wrapping_sub(last_snapshot_ms) >= snapshot_period_ms {
                    last_snapshot_ms = now;
                    let snapshot = telemetry_snapshot(now, state, altitude_m, config.flags);
                    if let Err(error) = send_payload(tx, &mut seq, now, &snapshot).await {
                        on_error(error);
                    } else {
                        snapshots_sent = snapshots_sent.wrapping_add(1);
                    }
                }

                if now.wrapping_sub(last_gps_ms) >= config.gps_period_ms {
                    last_gps_ms = now;
                    let payload = gps_payload(now, gps_fix);
                    if let Err(error) = send_payload(tx, &mut seq, now, &payload).await {
                        on_error(error);
                    }
                }

                if now.wrapping_sub(last_imu_ms) >= imu_period_ms {
                    last_imu_ms = now;
                    if let Some(sample) = imu_sample {
                        let payload = hilink::ImuPayload {
                            stamp: sim_stamp(now),
                            accel_mps2: sample.accel_mps2,
                            gyro_rps: sample.gyro_rad_s,
                        };
                        if let Err(error) = send_payload(tx, &mut seq, now, &payload).await {
                            on_error(error);
                        }
                    }
                    if let Some(sample) = aux_imu_sample {
                        let payload = hilink::AuxImuPayload {
                            stamp: sim_stamp(now),
                            accel_mps2: sample.accel_mps2,
                            gyro_rps: sample.gyro_rad_s,
                        };
                        if let Err(error) = send_payload(tx, &mut seq, now, &payload).await {
                            on_error(error);
                        }
                    }
                }

                if now.wrapping_sub(last_mag_ms) >= mag_period_ms {
                    last_mag_ms = now;
                    if let Some(sample) = mag_sample {
                        let payload = hilink::MagPayload {
                            stamp: sim_stamp(now),
                            field_ut: sample.field_ut,
                        };
                        if let Err(error) = send_payload(tx, &mut seq, now, &payload).await {
                            on_error(error);
                        }
                    }
                }

                if now.wrapping_sub(last_baro_ms) >= baro_period_ms {
                    last_baro_ms = now;
                    if let Some(sample) = baro_sample {
                        let payload = hilink::BaroPayload {
                            stamp: sim_stamp(now),
                            pressure_pa: sample.pressure_pa,
                            altitude_m,
                            temperature_c: sample.temperature_c,
                        };
                        if let Err(error) = send_payload(tx, &mut seq, now, &payload).await {
                            on_error(error);
                        }
                    }
                }
            }
        }
    }
}
