//! The flight step on the board, core 1 only: IMU1's task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per
//! good IMU1 sample, with the newest stick report (`crate::rc::FLIGHT_RC`) and the newest barometer, compass and GNSS samples
//! ([`FLIGHT_BARO`], [`FLIGHT_MAG`], [`FLIGHT_GNSS`]) since the previous step.
//! The result goes out through [`OUT`] to the ESC task; the step's log records go into [`LOG`], consumed by `logger` on core 0.
//! The steering estimator `SteerMix` is the ESKF3 core behind its output predictor `EskfSteer` (attitude, rates, horizontal state)
//! plus the zero-delay ESKF2 (vertical state). The `[STAT] NAV:` line of [`stat`] prints their state from [`NAV_STAT`].

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::dflog;
use fw_core::esc::LINES;
use fw_core::mission::{GDN_TARGETS, GdnTargets, select_targets};
use fw_core::rc::RcSample;
use fw_core::sample::{BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample};
use fw_core::xcore::{
    LogProducer, LogRing, Seqlock, SeqlockWriter, TripleBuffer, TripleReader, TripleWriter,
};
use fw_flight::alloc::CardAllocator;
use fw_flight::esc::log_fields;
use fw_flight::eskf::{Eskf, EskfError, EskfStats, RING};
use fw_flight::eskf_n0::{POS, VEL};
use fw_flight::eskf_out::{EskfSteer, SteerMix};
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{NavState, Sensors, Setpoint};
use fw_flight::imu::{ImuSample, SampleSink};
use fw_flight::pid::CascadedPid;
use fw_flight::step::{
    Flight, FlightInputs, ModeLog, StepOut, encode_mode, encode_nkf, encode_spn, encode_step, log_due,
};
use fw_flight::tuning::{GAIN_COUNT, GAIN_NAMES, Gains, Tuning};
use static_cell::StaticCell;

/// The steering estimator: a [`SteerMix`] whose parts live in [`STEER_CORE`], [`VERT`] and [`STEER`].
pub type BoardMix = SteerMix<'static, RING>;

/// The flight code this image runs.
pub type BoardFlight = Flight<&'static mut BoardMix, CascadedPid, CardAllocator>;

// `Flight` must hold the estimator by reference: an `Eskf` by value (about 16 KB) would be built on core 1's 16 KB stack.
const _: () = assert!(
    core::mem::size_of::<BoardFlight>() < core::mem::size_of::<Eskf>(),
    "BoardFlight holds an estimator by value: it must hold the &'static mut into STEER"
);

// The compiled tuning cannot make `Eskf::with_aiding` or `EskfSteer::new` refuse.
const _: () = assert!(
    Tuning::DEFAULT.nav_gps_delay_us <= Tuning::DEFAULT.nav_buffer_us,
    "Eskf::new(&Tuning::DEFAULT) would refuse the compiled tuning"
);
const _: () = assert!(
    Tuning::DEFAULT.nav_predict_us != 0
        && Tuning::DEFAULT.nav_buffer_us / Tuning::DEFAULT.nav_predict_us + 1 <= fw_flight::eskf_out::OUT as u64,
    "EskfSteer::new(&Tuning::DEFAULT) would refuse the compiled tuning"
);

/// The ESKF3 core's storage (17 592 B), in `.bss`.
static STEER_CORE: StaticCell<Result<Eskf, EskfError>> = StaticCell::new();

/// Builds the ESKF3 core into [`STEER_CORE`]; call once on core 0 before core 1 starts, so the value passes through core 0's stack.
/// `None` if the constructor refused or on a second call; [`steer`] then returns `None` and the board does not fly.
// Never inlined: the 17 592 B value must live in this call's own frame, not in the caller's `poll` frame on core 0.
#[inline(never)]
pub fn steer_core() -> Option<&'static mut Eskf> {
    STEER_CORE.try_init_with(|| Eskf::new(&Tuning::DEFAULT))?.as_mut().ok()
}

/// The zero-delay ESKF2's storage, in `.bss`.
static VERT: StaticCell<Result<Eskf, EskfError>> = StaticCell::new();

/// Builds ESKF2 (`Aiding::ImuBaro` with the GPS vertical velocity option) into [`VERT`], as [`steer_core`].
#[inline(never)]
pub fn vert() -> Option<&'static mut Eskf> {
    VERT.try_init_with(|| Eskf::with_gps_vd(&Tuning::DEFAULT))?.as_mut().ok()
}

/// The steering estimator's storage (the output predictor's ring and the two references), in `.bss`.
static STEER: StaticCell<BoardMix> = StaticCell::new();

/// Builds the steering estimator into [`STEER`] over [`steer_core`]'s and [`vert`]'s estimators; call once on core 0 before core 1 starts.
/// `None` if either is `None`, a constructor refused, or on a second call; [`Stepper::new`] then returns `None`.
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

#[derive(Clone, Copy, Default)]
pub struct EstStat {
    pub valid: u8,
    /// Down position (m) and down velocity (m/s), and their sigmas (infinite when not known).
    pub d_m: f32,
    pub vd_ms: f32,
    pub sigma_d_m: f32,
    pub sigma_vd_ms: f32,
}

impl EstStat {
    /// The ESKF3 core at its horizon: bit 0 tilt aligned, bit 1 yaw aligned, the nominal D and v_D, their sigmas.
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

#[derive(Clone, Copy, Default)]
pub struct NavStat {
    pub steps: u32,
    /// The composed state that steers: its valid bits, D and v_D with their sigmas (ESKF2's), and ESKF2's barometer counts
    /// (fused, gated, height resets).
    pub mb: EstStat,
    pub mb_baro: [u32; 3],
    /// The ESKF3 core: present, tilt aligned, yaw aligned, its state at the horizon (`EstStat::of_core`) and its counters.
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
        gps_vd_fused: 0,
        gps_vd_gated: 0,
        gps_vd_screened: 0,
        gps_vd_stale: 0,
    },
});

/// The `[STAT] NAV:` line (core 0, every 10 s, from the supervisor). Heights and vertical speeds only.
pub fn stat() {
    // The gain sets the flight core refused since the last line (`GAINS_REFUSED`; core 1 has no defmt, so the line is here).
    let refused = GAINS_REFUSED.load(Ordering::Relaxed);
    if refused != GAINS_REFUSED_SEEN.swap(refused, Ordering::Relaxed) {
        defmt::warn!(
            "[WARN] the flight core refused {=u32} set(s) of the ATC_* gains since boot; it keeps the previous set",
            refused
        );
    }
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
    pub t_us: u64,
    pub motors: MotorCommand,
}

pub static OUT: Seqlock<FlightOut> = Seqlock::new(FlightOut {
    t_us: 0,
    motors: MotorCommand::STOP,
});

/// Core 0 -> flight core: the barometer's newest sample with its core-0 time (writer: the barometer adapter; reader: IMU1's [`Stepper`]).
pub static FLIGHT_BARO: TripleBuffer<BaroSample> = TripleBuffer::new(BaroSample {
    reading: BaroReading {
        pressure_pa: 0.0,
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
});

/// Core 0 -> flight core: the compass's newest calibrated sample (writer: the compass adapter; reader: IMU1's [`Stepper`]).
pub static FLIGHT_MAG: TripleBuffer<MagSample> = TripleBuffer::new(MagSample {
    reading: MagReading {
        field_ut: [0.0; 3],
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
    expected: None,
});

/// Core 0 -> flight core: the receiver's newest epoch (writer: the GNSS adapter; reader: IMU1's [`Stepper`]).
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

/// Core 0 -> flight core: the stored `PILOT_SPEED_XY`, m/s (writer: core 0 via [`push_speed`]; reader: IMU1's [`Stepper`]).
/// The flight code takes it at every disarmed step, so the value at the arming step stays until the next disarm.
pub static FLIGHT_SPEED_XY: TripleBuffer<f32> = TripleBuffer::new(Tuning::DEFAULT.pilot_speed_xy_ms);

static SPEED_WRITER: Mutex<CriticalSectionRawMutex, RefCell<Option<TripleWriter<'static, f32>>>> =
    Mutex::new(RefCell::new(None));

/// Takes the writer of [`FLIGHT_SPEED_XY`] and pushes the stored value; call once on core 0 after `settings::boot`, before core 1 starts.
pub fn init_speed() {
    let Some(writer) = FLIGHT_SPEED_XY.take_writer() else {
        defmt::panic!("[FAIL] PILOT_SPEED_XY writer taken twice");
    };
    SPEED_WRITER.lock(|w| *w.borrow_mut() = Some(writer));
    push_speed("boot");
}

/// Pushes the stored `PILOT_SPEED_XY` to the flight core (core 0; call with no store access in progress). A value that
/// cannot be read or is not an `F32` is the table's default.
pub fn push_speed(why: &str) {
    let value = crate::store::with(|p| match p.get_named(fw_settings::flight::SPEED_XY) {
        Some(mavlink_device::ParamValue::F32(v)) => Some(v),
        _ => None,
    })
    .flatten()
    .unwrap_or(Tuning::DEFAULT.pilot_speed_xy_ms);
    SPEED_WRITER.lock(|w| {
        if let Some(writer) = w.borrow_mut().as_mut() {
            writer.write(value);
        }
    });
    defmt::info!(
        "[INFO] PILOT_SPEED_XY {=f32} m/s pushed to the flight core ({=str}); it takes effect at the next arming",
        value,
        why
    );
}

/// Core 0 -> flight core: the stored `MOT_POLES` (poles, not pole pairs; writer: core 0 via [`push_notch`]; reader: IMU1's [`Stepper`]).
/// The flight code takes it at every disarmed step (`Flight::set_notch`).
pub static FLIGHT_NOTCH: TripleBuffer<i32> = TripleBuffer::new(NOTCH_POLES_DEFAULT);

const NOTCH_POLES_DEFAULT: i32 = 14;

static NOTCH_WRITER: Mutex<CriticalSectionRawMutex, RefCell<Option<TripleWriter<'static, i32>>>> =
    Mutex::new(RefCell::new(None));

/// Takes the writer of [`FLIGHT_NOTCH`] and pushes the stored value; call once on core 0 after `settings::boot`, before core 1 starts.
pub fn init_notch() {
    let Some(writer) = FLIGHT_NOTCH.take_writer() else {
        defmt::panic!("[FAIL] MOT_POLES writer taken twice");
    };
    NOTCH_WRITER.lock(|w| *w.borrow_mut() = Some(writer));
    push_notch("boot");
}

/// Pushes the stored `MOT_POLES` to the flight core (core 0; call with no store access in progress). A value that cannot
/// be read or is not an `I32` is the table's default (14).
pub fn push_notch(why: &str) {
    let poles = crate::store::with(|p| match p.get_named(fw_settings::esc::POLES) {
        Some(mavlink_device::ParamValue::I32(v)) => Some(v),
        _ => None,
    })
    .flatten()
    .unwrap_or(NOTCH_POLES_DEFAULT);
    NOTCH_WRITER.lock(|w| {
        if let Some(writer) = w.borrow_mut().as_mut() {
            writer.write(poles);
        }
    });
    defmt::info!(
        "[INFO] MOT_POLES {=i32} pushed to the flight core ({=str}); it takes effect at the next arming",
        poles,
        why
    );
}

/// Core 0 -> flight core: the 14 stored gains `ATC_*` as one set (writer: core 0 via [`push_gains`]; reader: IMU1's [`Stepper`]).
/// The flight code takes the set at every disarmed step (`Flight::set_gains`).
pub static FLIGHT_GAINS: TripleBuffer<Gains> = TripleBuffer::new(Gains::of(&Tuning::DEFAULT));

/// Sets of [`FLIGHT_GAINS`] that `Flight::set_gains` refused (written by core 1, read by [`stat`] on core 0).
static GAINS_REFUSED: AtomicU32 = AtomicU32::new(0);

static GAINS_REFUSED_SEEN: AtomicU32 = AtomicU32::new(0);

static GAINS_WRITER: Mutex<CriticalSectionRawMutex, RefCell<Option<TripleWriter<'static, Gains>>>> =
    Mutex::new(RefCell::new(None));

/// Takes the writer of [`FLIGHT_GAINS`] and pushes the stored values; call once on core 0 after `settings::boot`, before core 1 starts.
pub fn init_gains() {
    let Some(writer) = FLIGHT_GAINS.take_writer() else {
        defmt::panic!("[FAIL] ATC_* gains writer taken twice");
    };
    GAINS_WRITER.lock(|w| *w.borrow_mut() = Some(writer));
    push_gains("boot");
}

/// Pushes the stored `ATC_*` gains to the flight core as one set (core 0; call with no store access in progress). A value
/// that cannot be read or is not an `F32` is its compiled default.
pub fn push_gains(why: &str) {
    let defaults = Gains::of(&Tuning::DEFAULT);
    let value = crate::store::with(|p| {
        let mut g = defaults;
        for (v, name) in g.0.iter_mut().zip(GAIN_NAMES) {
            if let Some(mavlink_device::ParamValue::F32(x)) = p.get_named(name) {
                *v = x;
            }
        }
        g
    })
    .unwrap_or(defaults);
    GAINS_WRITER.lock(|w| {
        if let Some(writer) = w.borrow_mut().as_mut() {
            writer.write(value);
        }
    });
    let changed = value.0.iter().zip(defaults.0.iter()).filter(|(a, b)| a.to_bits() != b.to_bits()).count();
    defmt::info!(
        "[INFO] ATC_* gains: {=usize} of {=usize} differ from the compiled defaults, pushed to the flight core ({=str}); they take effect at the next arming",
        changed,
        GAIN_COUNT,
        why
    );
}

pub type GdnSet = GdnTargets;

const GDN_OFF: GdnSet = GdnTargets {
    count: 0,
    radius_m: 2.0,
    targets: [(0, 0); GDN_TARGETS],
    alts_m: [0.0; GDN_TARGETS],
    return_m: 0.0,
    plan_id: 0,
    heights_ok: false,
    alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
};

/// Core 0 -> flight core: the committed mission with `GDN_RADIUS` and the height limits (writer: core 0 via [`push_gdn`]; reader: IMU1's [`Stepper`]).
pub static FLIGHT_GDN: TripleBuffer<GdnSet> = TripleBuffer::new(GDN_OFF);

static GDN_WRITER: Mutex<CriticalSectionRawMutex, RefCell<Option<TripleWriter<'static, GdnSet>>>> =
    Mutex::new(RefCell::new(None));

/// Takes the writer of [`FLIGHT_GDN`] and pushes the stored values; call once on core 0 after `settings::boot`, before core 1 starts.
pub fn init_gdn() {
    let Some(writer) = FLIGHT_GDN.take_writer() else {
        defmt::panic!("[FAIL] GDN writer taken twice");
    };
    GDN_WRITER.lock(|w| *w.borrow_mut() = Some(writer));
    push_gdn("boot");
}

/// Pushes the committed mission's waypoints with `GDN_RADIUS` and the height limits to the flight core (core 0; no store access in progress).
/// Runs when a `GDN_*` parameter changes and when the mission's change counter moves ([`crate::mission::changed`]).
/// A radius that cannot be read is the table's default.
pub fn push_gdn(why: &str) {
    use fw_settings::flight::{GDN_ALT_MAX, GDN_ALT_MIN, GDN_RADIUS};
    let (radius, limits) = crate::store::with(|p| {
        // The height limits as the upload reads them: a value that cannot be read is NaN (no heights pass).
        let float = |name| match p.get_named(name) {
            Some(mavlink_device::ParamValue::F32(v)) => v,
            _ => f32::NAN,
        };
        let radius = match p.get_named(GDN_RADIUS) {
            Some(mavlink_device::ParamValue::F32(v)) => v,
            _ => GDN_OFF.radius_m,
        };
        (radius, (float(GDN_ALT_MIN), float(GDN_ALT_MAX)))
    })
    .unwrap_or((GDN_OFF.radius_m, (f32::NAN, f32::NAN)));
    // The mission is read after the store is released (never nested); the counter noted is the one of the copy used.
    let (items, items_len, changes, plan_id) = crate::mission::committed();
    let value = select_targets(radius, items.get(..items_len).unwrap_or(&[]), plan_id, limits);
    crate::mission::note_pushed(changes);
    GDN_WRITER.lock(|w| {
        if let Some(writer) = w.borrow_mut().as_mut() {
            writer.write(value);
        }
    });
    defmt::info!(
        "[INFO] mission: {=u8} targets, GDN_RADIUS {=f32} m pushed to the flight core ({=str})",
        value.count,
        value.radius_m,
        why
    );
}

pub const LOG_BYTES: usize = 16384;
pub static LOG: LogRing<LOG_BYTES> = LogRing::new();

/// One step block: IMU1's sample, then NAV, SETP, DMD, MOT, PIDT, NTCH (281 bytes); disarmed, only every 8th step has the
/// six flight records, the other steps log the 40-byte IMU1 record alone.
const BLOCK: usize = dflog::IMU.length as usize
    + dflog::NAV.length as usize
    + dflog::SETP.length as usize
    + dflog::DMD.length as usize
    + dflog::MOT.length as usize
    + dflog::PIDT.length as usize
    + dflog::NTCH.length as usize;
// A block over the ring's push limit is refused whole and counted as a drop: every step would lose its records.
const _: () = assert!(BLOCK <= fw_core::xcore::PUSH_WITH_MAX);

/// IMU1's sample sink: the flight code and the writer of [`OUT`]. Built inside IMU1's task, on core 1.
pub struct Stepper {
    flight: BoardFlight,
    out: SeqlockWriter<'static, FlightOut>,
    rc: TripleReader<'static, RcSample>,
    baro: TripleReader<'static, BaroSample>,
    mag: TripleReader<'static, MagSample>,
    gnss: TripleReader<'static, GnssSample>,
    speed: TripleReader<'static, f32>,
    notch: TripleReader<'static, i32>,
    gdn: TripleReader<'static, GdnSet>,
    gains: TripleReader<'static, Gains>,
    station: crate::station::Core1,
    log: LogProducer<'static, LOG_BYTES>,
    step: u32,
    esc_since_us: u64,
    /// Covariance propagations already seen, of the ESKF3 core (I = 0 and I = 2) and of ESKF2 (I = 1): the NKF records'
    /// rate.
    predicts: [u32; 2],
    /// Corrections of the output predictor already seen (the NKI Src 10 and 11 rate).
    corrected: u32,
    stat: SeqlockWriter<'static, NavStat>,
    /// When MODE is due (the first step and every change of the selected or the effective mode).
    mode_log: ModeLog,
    ho_logged: Option<fw_flight::hands_off::State>,
    wp_logged: Option<u8>,
    k_logged: bool,
    /// The card check's MSG was written in this arming (cleared on a disarmed step).
    card_logged: bool,
    plan_logged: Option<(Option<u32>, u8)>,
}

impl Stepper {
    /// `None` if one of the cells it takes (the writers of [`OUT`] and [`NAV_STAT`], the producer of [`LOG`], the readers of the flight inputs) was taken already.
    /// `steer`: from [`steer`]; `None` here too (the board does not fly without its steering estimator).
    /// `period_us`: IMU1's sample period from its setup (the value the IMU1 task runs on); the notch's sample rate is
    /// 1e6 / `period_us`.
    pub fn new(
        rc: Option<TripleReader<'static, RcSample>>,
        steer: Option<&'static mut BoardMix>,
        period_us: u32,
    ) -> Option<Self> {
        Some(Self {
            flight: Flight::new(
                steer?,
                CascadedPid::new(&Tuning::DEFAULT, 1_000_000.0 / period_us.max(1) as f32),
                CardAllocator::new(),
                &Tuning::DEFAULT,
                &crate::settings::HANDSHAKE,
            ),
            out: OUT.take_writer()?,
            rc: rc?,
            baro: FLIGHT_BARO.take_reader()?,
            mag: FLIGHT_MAG.take_reader()?,
            gnss: FLIGHT_GNSS.take_reader()?,
            speed: FLIGHT_SPEED_XY.take_reader()?,
            notch: FLIGHT_NOTCH.take_reader()?,
            gdn: FLIGHT_GDN.take_reader()?,
            gains: FLIGHT_GAINS.take_reader()?,
            station: crate::station::Core1::new()?,
            log: LOG.take_producer()?,
            step: 0,
            esc_since_us: 0,
            predicts: [0; 2],
            corrected: 0,
            stat: NAV_STAT.take_writer()?,
            mode_log: ModeLog::new(),
            ho_logged: None,
            wp_logged: None,
            k_logged: false,
            card_logged: false,
            plan_logged: None,
        })
    }

    /// The navigation records: NKF1 and NKF2 at each covariance propagation of each estimator, armed, and at every 8th disarmed; NKI at each scalar fusion and reset.
    /// I = 0: the state that steered (TimeUS the step time) with the ESKF3 core's biases and events; I = 1: ESKF2's own state, biases and events; I = 2: the core's nominal state at its horizon.
    /// NKI Src 10 (Inn the attitude tracking error rad, Var the velocity error m/s) and Src 11 (Inn the position error m, Var T_d s) at every 20th output-predictor correction.
    /// Then, every 64th step, [`NAV_STAT`]. A record that does not fit is dropped and counted by the ring.
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

impl Stepper {
    /// One MSG record of `text` (up to its first NUL), one push of its own; false when the ring dropped it. One
    fn push_msg(&mut self, t_us: u64, text: &[u8]) -> bool {
        self.log.push_with(dflog::MSG.length.into(), |b| dflog::encode_msg(b, t_us, text))
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
        let (speed, speed_fresh) = self.speed.read();
        if speed_fresh {
            self.flight.set_pilot_speed_xy(speed);
        }
        let (notch, notch_fresh) = self.notch.read();
        if notch_fresh {
            self.flight.set_notch(notch);
        }
        // A refused set keeps the previous one; core 1 has no defmt, so it only counts (`stat` prints).
        let (gains, gains_fresh) = self.gains.read();
        if gains_fresh && !self.flight.set_gains(gains) {
            GAINS_REFUSED.fetch_add(1, Ordering::Relaxed);
        }
        // The whole set, read wait-free: the flight's mission facts and AUTO's list (taken at Start) at once, armed or not.
        let (gdn, gdn_fresh) = self.gdn.read();
        if gdn_fresh {
            self.flight.set_mission(gdn);
        }
        // The ESC status, once: the eRPM of the step's notch and the ESC record below both come from this copy. A read
        // that fails (the writer paused in mid-write) is no status: every line is missing.
        let esc = crate::esc::ESC.try_read(1);
        let erpm = esc.map_or([None; LINES], |s| s.lines.map(|l| l.erpm));
        let command = crate::station::before_step(&mut self.station, &mut self.flight);
        // The low-battery monitor's one byte (core 0): the descent request (CRITICAL with `BATT_FS_CRT_ACT` 1) and the
        // hands-off state machine's battery_critical and battery_ok.
        self.flight.set_battery_cell(crate::batt_fs::CELL.load(Ordering::Relaxed));
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
            erpm,
        });
        let answered = crate::station::after_step(&mut self.flight, command);
        self.out.write(FlightOut {
            t_us: out.t_us,
            motors: out.motors,
        });
        crate::batt_fs::DESCENT.store(self.flight.batt_descent() as u8, Ordering::Relaxed);
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
        // MODE at the first step and on every change of the selected or the effective mode, armed or not; a dropped record is offered again.
        if self.mode_log.pending(&out)
            && self
                .log
                .push_with(dflog::MODE.length.into(), |b| encode_mode(&out, b))
        {
            self.mode_log.done(&out);
        }
        // The hands-off state: a MSG at every change, a push of its own; a dropped one is offered again.
        // The text says why the state was entered ("HO 3 HOLD stn-lost"); at AUTO's exit on station loss a second MSG gives the station HEARTBEAT's age and the RC port's latest ground-station sender.
        let ho = self.flight.hands_off_state();
        let reason = self.flight.hands_off_reason();
        if self.ho_logged != Some(ho)
            && self.push_msg(out.t_us, &fw_flight::step::hands_off_reason_msg(ho, reason))
        {
            self.ho_logged = Some(ho);
            if reason == Some(fw_flight::hands_off::Reason::StationLost) {
                let age_ms = self
                    .flight
                    .last_radio_heartbeat_us()
                    .map(|t| (out.t_us.saturating_sub(t) / 1000).min(u64::from(u32::MAX)) as u32);
                self.push_msg(out.t_us, &fw_flight::step::station_lost_msg(age_ms, crate::station::gcs_sender()));
            }
        }
        // The station's flight command of this step and its answer: "CMD START ok", "CMD PAUSE den 5" (one push, dropped
        // if full).
        if let Some((command, result)) = answered {
            self.push_msg(out.t_us, &fw_flight::step::command_msg(command, result));
        }
        // The plan id the state machine is given and its waypoints, at the first step and at each change: "PLAN 0x1862F4A2
        // n=4"; a dropped one is offered again.
        let plan = self.flight.plan_seen();
        if self.plan_logged != Some(plan)
            && self.push_msg(out.t_us, &fw_flight::step::plan_msg(plan.0, plan.1))
        {
            self.plan_logged = Some(plan);
        }
        // AUTO's active waypoint: "HO WP <k>" at each change while the guidance runs; a dropped one is offered again.
        let wp = self.flight.leg_active();
        if let Some(k) = wp
            && wp != self.wp_logged
            && self.push_msg(out.t_us, &fw_flight::step::leg_msg(k))
        {
            self.wp_logged = wp;
        } else if wp.is_none() {
            self.wp_logged = None;
        }
        // Crash rule K's would-fire: "K 1 <lean error accel speed>" when it would fire, "K 0" when it clears; a dropped one is offered again.
        let crash = self.flight.crash_status();
        if let Some(text) = fw_flight::step::crash_msg(self.k_logged, &crash)
            && self.push_msg(out.t_us, &text)
        {
            self.k_logged = crash.would_fire();
        }
        // The card check, one MSG per arming once H is valid and more than 12 % from the card's m g ("HOVER <H / m g> % of card"); a dropped one is offered again.
        if !out.armed {
            self.card_logged = false;
        } else if !self.card_logged
            && let Some(pct) = self.flight.card_check_pct()
            && self.push_msg(out.t_us, &fw_flight::step::card_msg(pct))
        {
            self.card_logged = true;
        }
        // The in-air restart: the free-fall latch's changes ("FALL on", "FALL off impact"), then a refused switch-on edge ("ARM no thr") or a restart arming ("ARM fall t=630 f=95 thr=74"); one push each, dropped if full.
        if let Some(note) = self.flight.fall_note() {
            self.push_msg(out.t_us, &fw_flight::step::fall_msg(note));
        }
        if let Some(note) = self.flight.arm_note() {
            self.push_msg(out.t_us, &fw_flight::step::arm_msg(note));
        }
        // SPN: the altitude or position target, with the step block's cadence (armed: every step; disarmed: every 8th), only while the setpoint is Altitude or Position.
        if due.block && matches!(out.setpoint, Setpoint::Altitude { .. } | Setpoint::Position { .. }) {
            self.log
                .push_with(dflog::SPN.length.into(), |b| encode_spn(&out, b));
        }
        if due.esc
            && let Some(status) = esc
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
