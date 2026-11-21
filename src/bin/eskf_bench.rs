//! The N0 bench (decision-nav-eskf-20261120 in MARV-V2-drivers, section 11 row N0): times the 15-state
//! ESKF arithmetic of `fw_flight::eskf_n0` on core 1, the flight core, from SRAM (the `.ram_text` rules
//! of memory.x and the copy of src/ram.rs, as in the flight image), with core 1's cycle counter (DWT
//! CYCCNT, one count per clk_sys cycle; SysTick on the processor clock if CYCCNT does not count). The
//! clocks are embassy-rp's defaults, as in the flight image. Starts no device (no IMU, GNSS, ESC, radio
//! receiver, USB or SD), arms nothing, saves nothing. Two passes: core 0 idle (waiting on its timer),
//! then core 0 copying a 32 KiB SRAM buffer without pause (an SRAM contention bound). Core 0 sorts and
//! prints. Run: `BIN=eskf_bench tools/run.sh 60`.
#![no_std]
#![no_main]

#[path = "../ram.rs"]
mod ram;

use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, PIO2};
use embassy_rp::{bind_interrupts, dma, pio};
use embassy_time::{Instant, Timer};
use fw_flight::eskf_n0::{self as e, Cov, N, Noise, Nominal};
use panic_probe as _;
use static_cell::ConstStaticCell;

// The handlers memory.x asserts in SRAM and tools/check-ram-code.py takes as roots; never enabled here.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
});

const REPS: usize = 2000;
const JOBS: usize = 8;
const NAMES: [&str; JOBS] = [
    "overhead (empty region)",
    "a nominal step",
    "b propagation structured",
    "b propagation dense",
    "c fusion position row",
    "c fusion dense row",
    "d a + b structured",
    "d a + c dense row",
];
const PASSES: u32 = 2;
/// The offline armed values of the decision's section 10 (vrw 0.05, arw 1e-3, bg_rw 1e-4, ba_rw 3e-3), squared.
const NOISE: Noise = Noise { gyro: 1e-6, accel: 2.5e-3, gyro_bias: 1e-8, accel_bias: 9e-6 };
const PS: usize = 8;

struct Cycles(UnsafeCell<[[u32; REPS]; JOBS]>);
// SAFETY: core 1 writes only between CMD = pass + 1 and DONE = pass + 1 (Release); core 0 reads only
// after it saw DONE = pass + 1 (Acquire) and before it stores the next CMD.
unsafe impl Sync for Cycles {}
static CYCLES: Cycles = Cycles(UnsafeCell::new([[0; REPS]; JOBS]));

struct Buf(UnsafeCell<[u32; 8192]>);
// SAFETY: core 0 only.
unsafe impl Sync for Buf {}
static SRC: Buf = Buf(UnsafeCell::new([0x5a5a_5a5a; 8192]));
static DST: Buf = Buf(UnsafeCell::new([0; 8192]));

static CMD: AtomicU32 = AtomicU32::new(0);
static DONE: AtomicU32 = AtomicU32::new(0);
static COUNTER: AtomicU32 = AtomicU32::new(0);
static ISER0: AtomicU32 = AtomicU32::new(0);
static ISER1: AtomicU32 = AtomicU32::new(0);
static SPAN_CYC: AtomicU32 = AtomicU32::new(0);
static SPAN_US: AtomicU32 = AtomicU32::new(0);
static BAD: AtomicU32 = AtomicU32::new(0);
static REFUSED: AtomicU32 = AtomicU32::new(0);
static SINK: AtomicU32 = AtomicU32::new(0);

static CORE1_STACK: ConstStaticCell<Stack<16384>> = ConstStaticCell::new(Stack::new());

const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;
const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
const NVIC_ISER0: *const u32 = 0xE000_E100 as *const u32;
const NVIC_ISER1: *const u32 = 0xE000_E104 as *const u32;

#[inline(always)]
fn stamp(dwt: bool) -> u32 {
    compiler_fence(Ordering::SeqCst);
    // SAFETY: reads of this core's own DWT or SysTick counter; no side effect.
    let c = unsafe {
        if dwt { read_volatile(DWT_CYCCNT) } else { !read_volatile(SYST_CVR) & 0x00ff_ffff }
    };
    compiler_fence(Ordering::SeqCst);
    c
}

#[inline(always)]
fn uniform(s: &mut u32) -> f32 {
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    (*s >> 8) as f32 * (2.0 / 16_777_216.0) - 1.0
}

fn base_p(s: &mut u32) -> Cov {
    let scale = [0.05f32, 0.05, 0.1, 0.3, 0.3, 0.3, 3.0, 3.0, 5.0, 1e-3, 1e-3, 1e-3, 0.03, 0.03, 0.03];
    let mut l = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..=i {
            l[i][j] = uniform(s) * if i == j { 1.0 } else { 0.4 };
        }
    }
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let mut v = if i == j { 0.2 } else { 0.0 };
            for k in 0..N {
                v += l[i][k] * l[j][k];
            }
            p[i][j] = v * scale[i] * scale[j];
            p[j][i] = p[i][j];
        }
    }
    p
}

fn record(job: usize, rep: usize, d: u32) {
    // SAFETY: see `Cycles`; job < JOBS and rep < REPS by the callers' loops.
    unsafe { (*CYCLES.0.get())[job][rep] = d };
}

fn check(p: &Cov, n: &Nominal) {
    let mut t = n.q[0] + n.v[0] + n.p[0];
    for (i, row) in p.iter().enumerate() {
        t += row[i];
    }
    if !t.is_finite() {
        BAD.fetch_add(1, Ordering::Relaxed);
    }
    SINK.fetch_add(t.to_bits(), Ordering::Relaxed);
}

fn core1_main() -> ! {
    // SAFETY: this core's own DEMCR, DWT and SysTick; nothing else on this core uses them.
    let dwt = unsafe {
        write_volatile(DEMCR, read_volatile(DEMCR) | (1 << 24));
        write_volatile(DWT_CYCCNT, 0);
        write_volatile(DWT_CTRL, read_volatile(DWT_CTRL) | 1);
        let a = read_volatile(DWT_CYCCNT);
        cortex_m::asm::delay(100);
        let ok = read_volatile(DWT_CYCCNT) != a;
        if !ok {
            write_volatile(SYST_RVR, 0x00ff_ffff);
            write_volatile(SYST_CVR, 0);
            write_volatile(SYST_CSR, 0b101);
        }
        ok
    };
    let mask = if dwt { u32::MAX } else { 0x00ff_ffff };
    COUNTER.store(if dwt { 1 } else { 2 }, Ordering::Relaxed);
    // SAFETY: reads of this core's own NVIC enable registers.
    unsafe {
        ISER0.store(read_volatile(NVIC_ISER0), Ordering::Relaxed);
        ISER1.store(read_volatile(NVIC_ISER1), Ordering::Relaxed);
    }
    let mut s = 0x2468_ace1u32;
    let mut ps = [[[0.0f32; N]; N]; PS];
    for p in ps.iter_mut() {
        *p = base_p(&mut s);
    }
    let mut walk = Nominal {
        q: [1.0, 0.0, 0.0, 0.0],
        v: [0.0; 3],
        p: [0.0; 3],
        bg: [0.0; 3],
        ba: [0.0; 3],
        dvel: [0.0; 3],
        dt_sum: 0.0,
    };
    let mut pass = 0;
    loop {
        while CMD.load(Ordering::Acquire) != pass + 1 {}
        let (c0, t0) = (stamp(dwt), Instant::now().as_micros() as u32);
        for rep in 0..REPS {
            // A random walk over all attitudes: up to 0.4 rad about each axis per repetition, untimed.
            let turn = [300.0 * uniform(&mut s), 300.0 * uniform(&mut s), 300.0 * uniform(&mut s)];
            e::nominal_step(&mut walk, &turn, &[0.0; 3], 1.0 / 803.5);
            let q = walk.q;
            let r = e::rotation(&q);
            let g = fw_core::calib::GRAVITY_MSS;
            let accel = [
                -r[2][0] * g + 2.0 * uniform(&mut s),
                -r[2][1] * g + 2.0 * uniform(&mut s),
                -r[2][2] * g + 2.0 * uniform(&mut s),
            ];
            let gyro = [3.0 * uniform(&mut s), 3.0 * uniform(&mut s), 1.5 * uniform(&mut s)];
            let dt = (1.0 / 803.5) * (1.0 + 0.02 * uniform(&mut s));
            let dtc = 0.01 * (1.0 + 0.05 * uniform(&mut s));
            let n0 = Nominal {
                q,
                v: [5.0 * uniform(&mut s), 5.0 * uniform(&mut s), uniform(&mut s)],
                p: [50.0 * uniform(&mut s), 50.0 * uniform(&mut s), 10.0 * uniform(&mut s)],
                bg: [0.01 * uniform(&mut s), 0.01 * uniform(&mut s), 0.01 * uniform(&mut s)],
                ba: [0.2 * uniform(&mut s), 0.2 * uniform(&mut s), 0.2 * uniform(&mut s)],
                dvel: [accel[0] * dtc, accel[1] * dtc, accel[2] * dtc],
                dt_sum: dtc,
            };
            let mut h = [0.0f32; N];
            for x in h.iter_mut() {
                *x = uniform(&mut s);
            }
            let innov_pos = 0.5 * uniform(&mut s);
            let var_pos = 0.09 + 0.9 * (0.5 + 0.5 * uniform(&mut s));
            let innov_dense = 0.3 * uniform(&mut s);
            let p0 = &ps[rep % PS];
            let axis = rep % 3;

            let a = stamp(dwt);
            let b = stamp(dwt);
            record(0, rep, b.wrapping_sub(a) & mask);

            let mut n = n0;
            let a = stamp(dwt);
            e::nominal_step(&mut n, &gyro, &accel, dt);
            let b = stamp(dwt);
            record(1, rep, b.wrapping_sub(a) & mask);
            check(p0, &n);

            let mut p = *p0;
            let a = stamp(dwt);
            e::covariance_predict(&mut p, &n0.q, &n0.dvel, dtc, &NOISE);
            let b = stamp(dwt);
            record(2, rep, b.wrapping_sub(a) & mask);
            check(&p, &n0);

            let mut p = *p0;
            let a = stamp(dwt);
            e::covariance_predict_dense(&mut p, &n0.q, &n0.dvel, dtc, &NOISE);
            let b = stamp(dwt);
            record(3, rep, b.wrapping_sub(a) & mask);
            check(&p, &n0);

            let (mut n, mut p) = (n0, *p0);
            let a = stamp(dwt);
            let ok = e::fuse_position(&mut n, &mut p, axis, innov_pos, var_pos, 5.0);
            let b = stamp(dwt);
            record(4, rep, b.wrapping_sub(a) & mask);
            check(&p, &n);
            REFUSED.fetch_add(u32::from(!ok), Ordering::Relaxed);

            let (mut n, mut p) = (n0, *p0);
            let a = stamp(dwt);
            let ok = e::fuse(&mut n, &mut p, &h, innov_dense, 0.25, 5.0);
            let b = stamp(dwt);
            record(5, rep, b.wrapping_sub(a) & mask);
            check(&p, &n);
            REFUSED.fetch_add(u32::from(!ok), Ordering::Relaxed);

            let (mut n, mut p) = (n0, *p0);
            let a = stamp(dwt);
            e::nominal_step(&mut n, &gyro, &accel, dt);
            e::covariance_predict(&mut p, &n.q, &n.dvel, n.dt_sum, &NOISE);
            let b = stamp(dwt);
            record(6, rep, b.wrapping_sub(a) & mask);
            check(&p, &n);

            let (mut n, mut p) = (n0, *p0);
            let a = stamp(dwt);
            e::nominal_step(&mut n, &gyro, &accel, dt);
            let ok = e::fuse(&mut n, &mut p, &h, innov_dense, 0.25, 5.0);
            let b = stamp(dwt);
            record(7, rep, b.wrapping_sub(a) & mask);
            check(&p, &n);
            REFUSED.fetch_add(u32::from(!ok), Ordering::Relaxed);
        }
        let (c1, t1) = (stamp(dwt), Instant::now().as_micros() as u32);
        SPAN_CYC.store(c1.wrapping_sub(c0) & mask, Ordering::Relaxed);
        SPAN_US.store(t1.wrapping_sub(t0), Ordering::Relaxed);
        pass += 1;
        DONE.store(pass, Ordering::Release);
    }
}

fn report(pass: u32, clk: u32) {
    let mhz = clk as f32 / 1e6;
    defmt::info!(
        "[INFO] N0 pass {=u32} ({=str}): counter {=u32} (1 DWT CYCCNT, 2 SysTick); span {=u32} cycles over {=u32} us of TIMER0; non-finite {=u32}; refused {=u32}; sink {=u32:#x}",
        pass,
        if pass == 0 { "core 0 idle" } else { "core 0 copying SRAM" },
        COUNTER.load(Ordering::Relaxed),
        SPAN_CYC.load(Ordering::Relaxed),
        SPAN_US.load(Ordering::Relaxed),
        BAD.load(Ordering::Relaxed),
        REFUSED.load(Ordering::Relaxed),
        SINK.load(Ordering::Relaxed)
    );
    let mut v = [0u32; REPS];
    for (job, name) in NAMES.iter().enumerate() {
        // SAFETY: see `Cycles`: DONE = pass + 1 was seen with Acquire, CMD not yet advanced.
        v.copy_from_slice(unsafe { &(*CYCLES.0.get())[job] });
        v.sort_unstable();
        let (min, med, max) = (v[0], v[REPS / 2], v[REPS - 1]);
        let over = v.iter().filter(|&&c| c > med + med / 10 + 20).count() as u32;
        defmt::info!(
            "[INFO] N0 pass {=u32} RESULT {=str}: cycles min {=u32} median {=u32} max {=u32}; us min {=f32} median {=f32} max {=f32}; {=u32} of {=u32} over median+10%",
            pass,
            *name,
            min,
            med,
            max,
            min as f32 / mhz,
            med as f32 / mhz,
            max as f32 / mhz,
            over,
            REPS as u32
        );
    }
}

fn hammer() {
    // SAFETY: core 0 only, no other reference to the buffers is alive.
    unsafe { (*DST.0.get()).copy_from_slice(&*SRC.0.get()) };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let _irqs = Irqs;
    let clk = embassy_rp::clocks::clk_sys_freq();
    defmt::info!(
        "[INFO] N0 bench: clk_sys {=u32} Hz; {=u32} repetitions per job; addresses nominal_step {=u32:#010x} covariance_predict {=u32:#010x} covariance_predict_dense {=u32:#010x} fuse {=u32:#010x} fuse_position {=u32:#010x} core1_main {=u32:#010x}",
        clk,
        REPS as u32,
        e::nominal_step as *const () as u32,
        e::covariance_predict as *const () as u32,
        e::covariance_predict_dense as *const () as u32,
        e::fuse as *const () as u32,
        e::fuse_position as *const () as u32,
        core1_main as *const () as u32
    );
    spawn_core1(p.CORE1, CORE1_STACK.take(), move || core1_main());
    Timer::after_millis(100).await;
    defmt::info!(
        "[INFO] N0 core 1 NVIC ISER0 {=u32:#010x} ISER1 {=u32:#010x}",
        ISER0.load(Ordering::Relaxed),
        ISER1.load(Ordering::Relaxed)
    );
    for pass in 0..PASSES {
        BAD.store(0, Ordering::Relaxed);
        REFUSED.store(0, Ordering::Relaxed);
        let start = Instant::now();
        CMD.store(pass + 1, Ordering::Release);
        while DONE.load(Ordering::Acquire) != pass + 1 {
            if pass == 0 {
                Timer::after_millis(1).await;
            } else {
                hammer();
            }
        }
        defmt::info!("[INFO] N0 pass {=u32} took {=u64} ms", pass, start.elapsed().as_millis());
        report(pass, clk);
    }
    defmt::info!("[INFO] N0 bench done");
    loop {
        Timer::after_secs(1).await;
    }
}
