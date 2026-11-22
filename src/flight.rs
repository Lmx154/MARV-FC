//! The flight step on the board (decision-sim-seams-20261119, section 8; decision-first-pair-wiring-20261119, A1-A3).
//! Core 1 only: IMU1's task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per good IMU1 sample,
//! right after publishing it, with the newest CRSF stick report from `crate::rc::FLIGHT_RC` if one arrived since the
//! previous step, and the newest barometer, compass and GNSS samples from [`FLIGHT_BARO`], [`FLIGHT_MAG`] and
//! [`FLIGHT_GNSS`] likewise, then publishes the result through [`OUT`], which the ESC task sends to the motors, and then
//! pushes the step's log records into [`LOG`] (decision-flight-logging-20261119, section 3). On core 0 `logger`'s
//! collector touches this module as [`LOG`]'s consumer, and the barometer, compass and GNSS device tasks as the writers
//! of the three sample cells (decision-nav-eskf-20261120, section 4); core 0 learns "armed" from
//! `settings::HANDSHAKE` and "flight throttles sent" from `esc::ESC`.
//! The step runs the steering estimator `SteerMix` (decision-nav-eskf-20261120, Amendments G and G-1: the ESKF3 core,
//! `Eskf` with `Aiding::Full`, behind its output predictor `EskfSteer` gives attitude, rates and the horizontal state at
//! the current time; the zero-delay ESKF2, an `Eskf` with `Aiding::ImuBaro`, gives the vertical state), the cascaded PID
//! and the card's allocator with the constant gains of `fw_flight::tuning`. No shadow estimator runs on the board. The
//! composed state that steered goes to the log (NKF1, NKF2: I = 0), ESKF2's own state as I = 1, the ESKF3 core's
//! nominal state at its horizon as I = 2 (NKF1) and the two estimators' events as NKI (I = 0 the core, I = 1 ESKF2).
//! `MahonyBaro` stays in `fw_flight` and is not run on the board.
//! No `[STAT] FLT:` line: one was tried on 2026-11-19 and removed, because that one extra log line lengthened the
//! flight core's longest wait behind the log (README, Flight-core device loop). The `[STAT] NAV:` line of [`stat`]
//! (packet E2) prints the estimators' state from [`NAV_STAT`].

use fw_core::dflog;
use fw_core::rc::RcSample;
use fw_core::sample::{BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample};
use fw_core::xcore::{LogProducer, LogRing, Seqlock, SeqlockWriter, TripleBuffer, TripleReader};
use fw_flight::alloc::CardAllocator;
use fw_flight::esc::log_fields;
use fw_flight::eskf::{Aiding, Eskf, EskfError, EskfStats, RING};
use fw_flight::eskf_n0::{POS, VEL};
use fw_flight::eskf_out::{EskfSteer, SteerMix};
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{NavState, Sensors, Setpoint};
use fw_flight::imu::{ImuSample, SampleSink};
use fw_flight::pid::CascadedPid;
use fw_flight::step::{
    Flight, FlightInputs, ModeLog, StepOut, encode_mode, encode_nkf, encode_spn, encode_step, log_due,
};
use fw_flight::tuning::Tuning;
use static_cell::StaticCell;

/// The steering estimator: a [`SteerMix`] (Amendment G-1) whose parts live in [`STEER_CORE`], [`VERT`] and [`STEER`].
/// ESKF2 is a full-ring `Eskf` too (a zero-delay filter does not depend on the ring size, G1-a), so the image carries
/// one monomorphisation of the filter in RAM.
pub type BoardMix = SteerMix<'static, RING>;

/// The flight code this image runs; edit this line to change it (seams decision, section 9). To go back to
/// `MahonyBaro`: make this `Flight<MahonyBaro, CascadedPid, CardAllocator>`, pass `MahonyBaro::new(&Tuning::DEFAULT)`
/// to `Flight::new` in [`Stepper::new`] and give the I = 0 records in `log_nav` their old source
/// (`MahonyBaro::vert()`).
pub type BoardFlight = Flight<&'static mut BoardMix, CascadedPid, CardAllocator>;

// `Flight` owns its estimator: by value the `Eskf` (its IMU ring, waiting buffers and P, about 16 KB) would be built
// on core 1's 16 KB stack. `BoardFlight` must hold the reference only.
const _: () = assert!(
    core::mem::size_of::<BoardFlight>() < core::mem::size_of::<Eskf>(),
    "BoardFlight holds an estimator by value: it must hold the &'static mut into STEER"
);

// `Eskf::with_aiding` refuses only nav_gps_delay_us > nav_buffer_us (C2a) and `EskfSteer::new` a ring too small for
// nav_buffer_us / nav_predict_us + 1 entries: with the compiled tuning neither can fail.
const _: () = assert!(
    Tuning::DEFAULT.nav_gps_delay_us <= Tuning::DEFAULT.nav_buffer_us,
    "Eskf::new(&Tuning::DEFAULT) would refuse the compiled tuning"
);
const _: () = assert!(
    Tuning::DEFAULT.nav_predict_us != 0
        && Tuning::DEFAULT.nav_buffer_us / Tuning::DEFAULT.nav_predict_us + 1 <= fw_flight::eskf_out::OUT as u64,
    "EskfSteer::new(&Tuning::DEFAULT) would refuse the compiled tuning"
);

/// The ESKF3 core's storage (about 16 KB: the IMU ring, the waiting buffers, P), in `.bss`.
static STEER_CORE: StaticCell<Result<Eskf, EskfError>> = StaticCell::new();

/// Builds the ESKF3 core (`Aiding::Full`) into [`STEER_CORE`]. Call once, on core 0 before core 1 starts (the value
/// passes through core 0's stack, not core 1's 16 KB one). `None` if the constructor refused, which the assertions above
/// rule out, or on a second call; [`steer`] then returns `None` and the board does not fly.
// Never inlined: the roughly 16 KB value (and its move into the static) must live in this call's own frame, not in
// the frame of the caller's `poll`, which core 0 would then carry under everything the main task ever calls.
#[inline(never)]
pub fn steer_core() -> Option<&'static mut Eskf> {
    STEER_CORE.try_init_with(|| Eskf::new(&Tuning::DEFAULT))?.as_mut().ok()
}

/// The zero-delay ESKF2's storage (a full `Eskf`, about 16 KB; its ring is used as one entry), in `.bss`.
static VERT: StaticCell<Result<Eskf, EskfError>> = StaticCell::new();

/// Builds ESKF2 (`Aiding::ImuBaro`) into [`VERT`]; as [`steer_core`].
#[inline(never)]
pub fn vert() -> Option<&'static mut Eskf> {
    VERT.try_init_with(|| Eskf::with_aiding(&Tuning::DEFAULT, Aiding::ImuBaro))?.as_mut().ok()
}

/// The steering estimator's storage (the output predictor's ring and the two references), in `.bss`.
static STEER: StaticCell<BoardMix> = StaticCell::new();

/// Builds the steering estimator into [`STEER`] over the two estimators from [`steer_core`] and [`vert`]. Call once, on
/// core 0 before core 1 starts, after those two (they are separate calls so that their frames do not stack). `None` if
/// either is `None`, a constructor refused, or on a second call; [`Stepper::new`] then returns `None` and the board
/// does not fly.
#[inline(never)]
pub fn steer(
    core: Option<&'static mut Eskf>,
    vert: Option<&'static mut Eskf>,
) -> Option<&'static mut BoardMix> {
    let h = EskfSteer::new(core?, &Tuning::DEFAULT).ok()?;
    STEER.try_init(SteerMix::new(h, vert?))
}

/// The ESKF3 core's alignment as NKF1 valid bits (I = 2) and `EstStat::valid`: tilt aligned 1, yaw aligned 2.
fn core_valid(core: &Eskf) -> u8 {
    u8::from(core.tilt_aligned()) | u8::from(core.yaw_aligned()) << 1
}

/// One estimator's part of [`NavStat`].
#[derive(Clone, Copy, Default)]
pub struct EstStat {
    /// NAV's valid bits.
    pub valid: u8,
    /// Down position (m) and down velocity (m/s), and their sigmas (infinite when not known).
    pub d_m: f32,
    pub vd_ms: f32,
    pub sigma_d_m: f32,
    pub sigma_vd_ms: f32,
}

impl EstStat {
    /// The ESKF3 core at its horizon (it has no `NavState` of its own on the board): bit 0 tilt aligned, bit 1 yaw
    /// aligned, the nominal D and v_D, the sigmas of those two error states.
    fn of_core(core: &Eskf) -> Self {
        let (_, v, p) = core.nominal();
        let sig = core.sigmas();
        EstStat {
            valid: core_valid(core),
            d_m: p[2],
            vd_ms: v[2],
            sigma_d_m: sig[POS + 2],
            sigma_vd_ms: sig[VEL + 2],
        }
    }

    fn of(nav: &NavState) -> Self {
        let (sigma_d_m, sigma_vd_ms) =
            nav.sigma.map_or((f32::INFINITY, f32::INFINITY), |s| (s.position_d_m, s.velocity_d_ms));
        let v = nav.valid;
        EstStat {
            valid: u8::from(v.tilt)
                | u8::from(v.heading) << 1
                | u8::from(v.rates) << 2
                | u8::from(v.velocity_ne) << 3
                | u8::from(v.velocity_d) << 4
                | u8::from(v.position_ne) << 5
                | u8::from(v.position_d) << 6,
            d_m: nav.position_m[2],
            vd_ms: nav.velocity_ms[2],
            sigma_d_m,
            sigma_vd_ms,
        }
    }
}

/// What the `[STAT] NAV:` line prints: no horizontal position, no coordinate.
#[derive(Clone, Copy, Default)]
pub struct NavStat {
    /// Steps since the start.
    pub steps: u32,
    /// The composed state that steers (I = 0; the name dates from `MahonyBaro`): its valid bits, D and v_D with their
    /// sigmas, which are ESKF2's (Amendment G-1), and ESKF2's barometer counts (fused, gated, height resets).
    pub mb: EstStat,
    pub mb_baro: [u32; 3],
    /// The ESKF3 core (the name dates from the shadow): present (always true once the board flies), tilt aligned, yaw
    /// aligned, its state at the horizon (`EstStat::of_core`: valid bits 1 tilt aligned and 2 yaw aligned, nominal D and
    /// v_D, sigmas of those states) and its counters.
    pub shadow: bool,
    pub tilt_aligned: bool,
    pub yaw_aligned: bool,
    pub ek: EstStat,
    pub ek_stats: EskfStats,
}

/// Flight core -> core 0: the estimators' state, written every 64th step by IMU1's [`Stepper`], read by [`stat`].
pub static NAV_STAT: Seqlock<NavStat> = Seqlock::new(NavStat {
    steps: 0,
    mb: EstStat {
        valid: 0,
        d_m: 0.0,
        vd_ms: 0.0,
        sigma_d_m: 0.0,
        sigma_vd_ms: 0.0,
    },
    mb_baro: [0; 3],
    shadow: false,
    tilt_aligned: false,
    yaw_aligned: false,
    ek: EstStat {
        valid: 0,
        d_m: 0.0,
        vd_ms: 0.0,
        sigma_d_m: 0.0,
        sigma_vd_ms: 0.0,
    },
    ek_stats: EskfStats {
        predicts: 0,
        baro_fused: 0,
        baro_gated: 0,
        height_resets: 0,
        noaid: 0,
        noaid_dropped: 0,
        mag_refused: 0,
        mag_fused: 0,
        mag_gated: 0,
        yaw_aligns: 0,
        yaw_resets: 0,
        gps_epochs: 0,
        gps_unaccepted: 0,
        gps_prealign: 0,
        gps_disarmed: 0,
        gps_e1s: 0,
        gps_e3b: 0,
        gps_pos_fused: 0,
        gps_pos_gated: 0,
        gps_vel_fused: 0,
        gps_vel_gated: 0,
        gps_resets: 0,
        stale: 0,
        reinits: 0,
        max_jobs_per_step: 0,
        replaced: 0,
        dropped: 0,
    },
});

/// The `[STAT] NAV:` line (core 0, every 10 s, from the supervisor). Heights and vertical speeds only.
pub fn stat() {
    let Some(s) = NAV_STAT.try_read(4) else {
        defmt::info!("[STAT] NAV: not read");
        return;
    };
    let (m, e, k) = (&s.mb, &s.ek, &s.ek_stats);
    defmt::info!(
        "[STAT] NAV: steps {=u32}; steering (ESKF3 output + ESKF2 vertical) valid {=u8:#x}, D {=f32} m (sigma {=f32}), vD {=f32} m/s (sigma {=f32}), ESKF2 baro fused {=u32} gated {=u32} height resets {=u32}; ESKF3 core present {=bool}, tilt aligned {=bool}, yaw aligned {=bool}, align bits {=u8:#x}, horizon D {=f32} m (sigma {=f32}), vD {=f32} m/s (sigma {=f32}), predicts {=u32}, baro fused {=u32} gated {=u32}, compass refused {=u32} fused {=u32} gated {=u32}, GPS epochs {=u32} unaccepted {=u32} before yaw {=u32} disarmed {=u32} screened {=u32} scalars fused {=u32} gated {=u32}, resets height {=u32} yaw align {=u32} yaw {=u32} GPS {=u32}, reinits {=u32}, most jobs in a step {=u32}, replaced {=u32}, dropped {=u32}",
        s.steps,
        m.valid,
        m.d_m,
        m.sigma_d_m,
        m.vd_ms,
        m.sigma_vd_ms,
        s.mb_baro[0],
        s.mb_baro[1],
        s.mb_baro[2],
        s.shadow,
        s.tilt_aligned,
        s.yaw_aligned,
        e.valid,
        e.d_m,
        e.sigma_d_m,
        e.vd_ms,
        e.sigma_vd_ms,
        k.predicts,
        k.baro_fused,
        k.baro_gated,
        k.mag_refused,
        k.mag_fused,
        k.mag_gated,
        k.gps_epochs,
        k.gps_unaccepted,
        k.gps_prealign,
        k.gps_disarmed,
        k.gps_e1s.wrapping_add(k.gps_e3b),
        k.gps_pos_fused.wrapping_add(k.gps_vel_fused),
        k.gps_pos_gated.wrapping_add(k.gps_vel_gated),
        k.height_resets,
        k.yaw_aligns,
        k.yaw_resets,
        k.gps_resets,
        k.reinits,
        k.max_jobs_per_step,
        k.replaced,
        k.dropped
    );
}

/// What the flight step publishes after every step.
#[derive(Clone, Copy)]
pub struct FlightOut {
    /// The latest step's time, us.
    pub t_us: u64,
    /// The latest step's command.
    pub motors: MotorCommand,
}

/// Written by IMU1's task (core 1) after every step; read by the ESC task (core 1) at every exchange.
pub static OUT: Seqlock<FlightOut> = Seqlock::new(FlightOut {
    t_us: 0,
    motors: MotorCommand::STOP,
});

/// Core 0 -> flight core: the barometer's newest sample with its core-0 time (one writer, the barometer's
/// adapter as it publishes to the row; one reader, IMU1's [`Stepper`]).
pub static FLIGHT_BARO: TripleBuffer<BaroSample> = TripleBuffer::new(BaroSample {
    reading: BaroReading {
        pressure_pa: 0.0,
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
});

/// Core 0 -> flight core: the compass's newest sample, the calibrated field (one writer, the compass's adapter as
/// it publishes to the row; one reader, IMU1's [`Stepper`]).
pub static FLIGHT_MAG: TripleBuffer<MagSample> = TripleBuffer::new(MagSample {
    reading: MagReading {
        field_ut: [0.0; 3],
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
    expected: None,
});

/// Core 0 -> flight core: the receiver's newest epoch (one writer, the GNSS adapter as it publishes to the row;
/// one reader, IMU1's [`Stepper`]).
pub static FLIGHT_GNSS: TripleBuffer<GnssSample> = TripleBuffer::new(GnssSample {
    reading: GnssReading {
        fix_type: 1,
        lat_e7: None,
        lon_e7: None,
        alt_mm: None,
        speed_mm_s: None,
        course_cdeg: None,
        sats_used: None,
        hdop_x100: None,
        vdop_x100: None,
        time: None,
        date: None,
        lat_std_mm: None,
        lon_std_mm: None,
        alt_std_mm: None,
        vel_ned_mm_s: None,
    },
    t_us: 0,
    seq: 0,
});

/// The flight log ring's size (decision-flight-logging-20261119, section 3).
pub const LOG_BYTES: usize = 16384;
/// The flight records: producer IMU1's [`Stepper`] (core 1), consumer `logger`'s collector (core 0).
pub static LOG: LogRing<LOG_BYTES> = LogRing::new();

/// One step block: IMU1's sample, then NAV, SETP, DMD, MOT (182 bytes); disarmed, only every 8th step has the
/// four flight records, the other steps log the 40-byte IMU1 record alone.
const BLOCK: usize = dflog::IMU.length as usize
    + dflog::NAV.length as usize
    + dflog::SETP.length as usize
    + dflog::DMD.length as usize
    + dflog::MOT.length as usize;

/// IMU1's sample sink: the flight code and the writer of [`OUT`]. Built inside IMU1's task, on core 1.
pub struct Stepper {
    flight: BoardFlight,
    out: SeqlockWriter<'static, FlightOut>,
    rc: TripleReader<'static, RcSample>,
    baro: TripleReader<'static, BaroSample>,
    mag: TripleReader<'static, MagSample>,
    gnss: TripleReader<'static, GnssSample>,
    log: LogProducer<'static, LOG_BYTES>,
    /// Steps since the start, for `log_due`.
    step: u32,
    /// `TimeUS` of the previous ESC record.
    esc_since_us: u64,
    /// Covariance propagations already seen, of the ESKF3 core (I = 0 and I = 2) and of ESKF2 (I = 1): the NKF records'
    /// rate.
    predicts: [u32; 2],
    /// Corrections of the output predictor already seen (the NKI Src 10 and 11 rate).
    corrected: u32,
    stat: SeqlockWriter<'static, NavStat>,
    /// When MODE is due (the first step and every change of the selected or the effective mode).
    mode_log: ModeLog,
}

impl Stepper {
    /// `None` if the writer of [`OUT`], the producer of [`LOG`], the reader of `crate::rc::FLIGHT_RC` (`rc`) or the
    /// reader of [`FLIGHT_BARO`], [`FLIGHT_MAG`] or [`FLIGHT_GNSS`] or the writer of [`NAV_STAT`] was taken already.
    /// `steer`: from [`steer`]; `None` here too (the board does not fly without its steering estimator).
    pub fn new(
        rc: Option<TripleReader<'static, RcSample>>,
        steer: Option<&'static mut BoardMix>,
    ) -> Option<Self> {
        Some(Self {
            flight: Flight::new(
                steer?,
                CascadedPid::new(&Tuning::DEFAULT),
                CardAllocator::new(),
                &Tuning::DEFAULT,
                &crate::settings::HANDSHAKE,
            ),
            out: OUT.take_writer()?,
            rc: rc?,
            baro: FLIGHT_BARO.take_reader()?,
            mag: FLIGHT_MAG.take_reader()?,
            gnss: FLIGHT_GNSS.take_reader()?,
            log: LOG.take_producer()?,
            step: 0,
            esc_since_us: 0,
            predicts: [0; 2],
            corrected: 0,
            stat: NAV_STAT.take_writer()?,
            mode_log: ModeLog::new(),
        })
    }

    /// The navigation records (decision-nav-eskf-20261120, 8c, B5, C8; Amendments G and G-1): NKF1 and NKF2 at each
    /// covariance propagation of each estimator, armed, and at every 8th disarmed; NKI at each scalar fusion and reset.
    /// I = 0: the composed state that steered (`out.nav`, TimeUS the step time) with the ESKF3 core's biases, and the
    /// core's events (TimeUS the horizon time); I = 1: ESKF2's own state, biases and events; I = 2: NKF1 of the core's
    /// nominal state at its horizon (TimeUS the horizon time, valid bits as [`core_valid`]), at the core's propagations.
    /// NKI Src 10 (Inn the attitude tracking error rad, Var the velocity error m/s) and Src 11 (Inn the position error
    /// m, Var T_d s), I = 0, TimeUS the step time, Ok 1, at every 20th correction of the output predictor. Then, every
    /// 64th step, [`NAV_STAT`]. A record that does not fit is dropped and counted by the ring.
    fn log_nav(&mut self, out: &StepOut) {
        const NKF: usize = dflog::NKF1.length as usize + dflog::NKF2.length as usize;
        let armed = out.armed;
        let mix: &BoardMix = self.flight.estimator();
        let core = mix.steer().core();
        let vert = mix.vert();
        let p0 = core.stats().predicts;
        if p0 != self.predicts[0] {
            self.predicts[0] = p0;
            if armed || p0 % 8 == 0 {
                let (bg, ba) = core.biases();
                self.log.push_with(NKF, |b| encode_nkf(b, 0, &out.nav, bg, ba));
                if let Some(t_us) = core.horizon_us() {
                    let (q, v, p) = core.nominal();
                    let valid = core_valid(core);
                    self.log.push_with(dflog::NKF1.length.into(), |b| {
                        dflog::encode_nkf1(b, t_us, 2, q, v, p, valid)
                    });
                }
            }
        }
        for ev in core.events() {
            self.log.push_with(dflog::NKI.length.into(), |b| {
                dflog::encode_nki(b, ev.t_us, 0, ev.src, ev.inn, ev.var, ev.ok)
            });
        }
        let c = mix.steer().corrected();
        if c != self.corrected {
            self.corrected = c;
            if c % 20 == 0 {
                let tr = mix.steer().tracking();
                let t_us = out.nav.t_us;
                self.log.push_with(dflog::NKI.length.into(), |b| {
                    dflog::encode_nki(b, t_us, 0, 10, tr.att_rad, tr.vel_ms, 1)
                });
                self.log.push_with(dflog::NKI.length.into(), |b| {
                    dflog::encode_nki(b, t_us, 0, 11, tr.pos_m, tr.t_d_s, 1)
                });
            }
        }
        if let Some(nav) = mix.vert_nav() {
            let p1 = vert.stats().predicts;
            if p1 != self.predicts[1] {
                self.predicts[1] = p1;
                if armed || p1 % 8 == 0 {
                    let (bg, ba) = vert.biases();
                    self.log.push_with(NKF, |b| encode_nkf(b, 1, nav, bg, ba));
                }
            }
            for ev in vert.events() {
                self.log.push_with(dflog::NKI.length.into(), |b| {
                    dflog::encode_nki(b, ev.t_us, 1, ev.src, ev.inn, ev.var, ev.ok)
                });
            }
        }
        if self.step % 64 == 0 {
            self.stat.write(NavStat {
                steps: self.step,
                mb: EstStat::of(&out.nav),
                mb_baro: [vert.stats().baro_fused, vert.stats().baro_gated, vert.stats().height_resets],
                shadow: true,
                tilt_aligned: core.tilt_aligned(),
                yaw_aligned: core.yaw_aligned(),
                ek: EstStat::of_core(core),
                ek_stats: *core.stats(),
            });
        }
    }
}

impl SampleSink for Stepper {
    /// One flight step for the IMU1 sample just published, with the stick report and the barometer, compass and
    /// GNSS samples that are new since the previous step. IMU2 comes with later steps.
    fn sample(&mut self, sample: &ImuSample) {
        let (report, fresh) = self.rc.read();
        let (baro, baro_fresh) = self.baro.read();
        let (mag, mag_fresh) = self.mag.read();
        let (gnss, gnss_fresh) = self.gnss.read();
        let out = self.flight.step(&FlightInputs {
            sensors: Sensors {
                imu1: *sample,
                imu2: None,
                baro: baro_fresh.then_some(baro),
                mag: mag_fresh.then_some(mag),
                gnss: gnss_fresh.then_some(gnss),
                external: None,
            },
            rc: fresh.then_some(report),
        });
        self.out.write(FlightOut {
            t_us: out.t_us,
            motors: out.motors,
        });
        // The log, after the command is out. A record that does not fit is dropped and counted by the ring.
        let due = log_due(self.step, out.armed);
        self.step = self.step.wrapping_add(1);
        if due.imu {
            let r = &sample.reading;
            let len = if due.block { BLOCK } else { dflog::IMU.length as usize };
            self.log.push_with(len, |b| {
                let n =
                    dflog::encode_imu(b, sample.t_us, 0, r.gyro_rads, r.accel_ms2, r.temperature_c);
                if due.block {
                    n + encode_step(&out, b.get_mut(n..).unwrap_or_default())
                } else {
                    n
                }
            });
        }
        // MODE (decision-nav-eskf-20261120, 8c, F6): at the first step and on every change of the selected or the
        // effective mode, armed or not (a change is rare, so it has no cadence). A dropped record is offered again.
        if self.mode_log.pending(&out)
            && self
                .log
                .push_with(dflog::MODE.length.into(), |b| encode_mode(&out, b))
        {
            self.mode_log.done(&out);
        }
        // SPN: the altitude target, with the step block's cadence (armed: every step; disarmed: every 8th), only while
        // the setpoint is Altitude, so a flight in Angle logs none (F1).
        if due.block && matches!(out.setpoint, Setpoint::Altitude { .. }) {
            self.log
                .push_with(dflog::SPN.length.into(), |b| encode_spn(&out, b));
        }
        if due.esc
            && let Some(status) = crate::esc::ESC.try_read(1)
            && status.exchanges > 0
        {
            let f = log_fields(&status, self.esc_since_us);
            let t = status.last_exchange_us;
            self.esc_since_us = t;
            self.log.push_with(dflog::ESC.length.into(), |b| {
                dflog::encode_esc(b, t, f.erpm, f.volts_cv, f.temp_c, f.flags)
            });
        }
        if fresh {
            let s = report.sticks;
            self.log.push_with(dflog::RCS.length.into(), |b| {
                dflog::encode_rcs(
                    b,
                    report.t_us,
                    [s.roll, s.pitch, s.throttle, s.yaw],
                    report.arm,
                )
            });
        }
        self.log_nav(&out);
    }
}
