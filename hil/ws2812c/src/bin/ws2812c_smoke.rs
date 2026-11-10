//! On-board check of the ws2812c driver: the status LED D20 (WS2812C-2020), data on GPIO43 through
//! 100 ohm. The LED cannot be read back, so the test measures the waveform on GPIO43 itself.
//!
//! Capture: a second PIO state machine (PIO2) counts how long the pad stays low and high, two PIO
//! cycles per count (13.3 ns at 150 MHz), and pushes each count at the edge that ends the phase; a
//! DMA channel drains the FIFO into RAM. Pad inputs reach every PIO block whatever the pin's
//! function, so the capture reads the pad while PIO1 or SPI1 drives it. Each measured time is the
//! true time to within one count (+-13.3 ns): see `CYCLES_HIGH` and `CYCLES_LOW` for the constant
//! program overhead that is added.
//!
//! Stages: W1 PIO waveform, W2 reset gap, W3 SPI waveform and gap on SPI1 (TX GPIO43, SCK GPIO26),
//! W4 blocking front-end, W5 colours for the eye. PIO0 is left to the SD driver.
#![no_std]
#![no_main]

use core::future::Future;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::{clk_peri_freq, clk_sys_freq};
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO1, PIO2};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{self, Config, FifoJoin, Pio, StateMachine};
use embassy_rp::spi::{self, Phase, Polarity, Spi};
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use ws2812c::limits::*;
use ws2812c::pio::PioTransport;
use ws2812c::spi::{SPI_MAX_HZ, SPI_MIN_HZ, SPI_NOMINAL_HZ, SpiTransport};
use ws2812c::{Rgb, Ws2812c, blocking};

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// GPIO43 as the PIO block with GPIOBASE 16 counts it.
const PIN_IN_WINDOW: u8 = 43 - 16;
/// PIO cycles from the detection of a rising edge to the detection of the falling edge, besides
/// the two cycles per counted iteration: `mov isr`, `push`, `mov x` and the first sample.
const CYCLES_HIGH: u32 = 4;
/// The same for a low phase: the extra `jmp end_hi` makes it one more.
const CYCLES_LOW: u32 = 5;
/// Counts in one frame: the idle low time before the first bit, then 24 highs and 23 lows.
const FRAME_COUNTS: usize = 48;
/// Two frames: the first, the low time between them, the second without its leading idle count.
const TWO_FRAME_COUNTS: usize = FRAME_COUNTS + FRAME_COUNTS;
/// A high longer than this is a 1, shorter is a 0 (the middle of the two high-time windows).
const ONE_THRESHOLD_NS: u32 = 480;

/// The colour of the waveform stages: G = 0xA5, R = 0x3C, B = 0x0F, 24 bits of mixed ones and zeros.
const TEST_COLOR: Rgb = Rgb::new(0x3C, 0xA5, 0x0F);

struct Report {
    failures: u32,
}

/// `check!(rep, cond, "{=str} format", args..)`: the first placeholder receives "[PASS]" or
/// "[FAIL]". Returns `cond`.
macro_rules! check {
    ($rep:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {{
        let ok: bool = $cond;
        if ok {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $rep.failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
        ok
    }};
}

/// Smallest and largest of a set of times, in nanoseconds.
#[derive(Clone, Copy)]
struct Range {
    min: u32,
    max: u32,
}

impl Range {
    const EMPTY: Self = Self {
        min: u32::MAX,
        max: 0,
    };

    fn add(&mut self, ns: u32) {
        self.min = self.min.min(ns);
        self.max = self.max.max(ns);
    }
}

/// What one frame looked like on the pad.
struct Frame {
    /// The 24 decoded bits, first bit in bit 23.
    bits: u32,
    /// Times outside the limits of their bit value, plus bits that decoded wrong.
    bad: u32,
    t0h: Range,
    t1h: Range,
    t0l: Range,
    t1l: Range,
}

fn ns_high(count: u32, clk: u32) -> u32 {
    ((2 * count as u64 + CYCLES_HIGH as u64) * 1_000_000_000 / clk as u64) as u32
}

fn ns_low(count: u32, clk: u32) -> u32 {
    ((2 * count as u64 + CYCLES_LOW as u64) * 1_000_000_000 / clk as u64) as u32
}

/// Decodes one frame from its 47 counts (H1, L1, H2, ..., H24) against the word that was sent.
fn analyse(counts: &[u32], word: u32, clk: u32) -> Frame {
    let mut f = Frame {
        bits: 0,
        bad: 0,
        t0h: Range::EMPTY,
        t1h: Range::EMPTY,
        t0l: Range::EMPTY,
        t1l: Range::EMPTY,
    };
    for i in 0..24 {
        let sent = word >> (31 - i) & 1;
        let high = ns_high(counts[2 * i], clk);
        let decoded = (high > ONE_THRESHOLD_NS) as u32;
        f.bits = f.bits << 1 | decoded;
        if decoded != sent {
            f.bad += 1;
        }
        let (h, lo, hi) = if sent == 1 {
            (&mut f.t1h, T1H_MIN_NS, T1H_MAX_NS)
        } else {
            (&mut f.t0h, T0H_MIN_NS, T0H_MAX_NS)
        };
        h.add(high);
        if !(lo..=hi).contains(&high) {
            f.bad += 1;
        }
        if i < 23 {
            let low = ns_low(counts[2 * i + 1], clk);
            let (l, lo, hi) = if sent == 1 {
                (&mut f.t1l, T1L_MIN_NS, T1L_MAX_NS)
            } else {
                (&mut f.t0l, T0L_MIN_NS, T0L_MAX_NS)
            };
            l.add(low);
            if !(lo..=hi).contains(&low) {
                f.bad += 1;
            }
        }
    }
    f
}

fn report_frame(label: &str, f: &Frame) {
    defmt::info!(
        "[STAT] {=str}: T0H {=u32}..{=u32} ns, T1H {=u32}..{=u32} ns, T0L {=u32}..{=u32} ns, T1L {=u32}..{=u32} ns",
        label,
        f.t0h.min,
        f.t0h.max,
        f.t1h.min,
        f.t1h.max,
        f.t0l.min,
        f.t0l.max,
        f.t1l.min,
        f.t1l.max
    );
}

/// The edge-timing state machine and the DMA channel that drains it.
struct Capture<'d> {
    sm: StateMachine<'d, PIO2, 0>,
    dma: dma::Channel<'d>,
    cfg: Config<'d, PIO2>,
    clk: u32,
}

impl<'d> Capture<'d> {
    /// Loads the edge counter into PIO2 and prepares (does not start) the state machine.
    ///
    /// The program alternates two loops of two cycles each: while the pad is low it counts down
    /// X; at the rising edge it pushes the count and starts again; while the pad is high it counts
    /// down X again and pushes at the falling edge. The count is `!X`, the number of iterations.
    fn new(
        common: &mut pio::Common<'d, PIO2>,
        sm: StateMachine<'d, PIO2, 0>,
        dma: dma::Channel<'d>,
    ) -> Self {
        let prg = pio_asm!(
            "lo:",
            "    jmp pin end_lo",
            "    jmp x-- lo",
            "    jmp lo",
            "end_lo:",
            "    mov isr, !x",
            "    push noblock",
            "    mov x, !null",
            "hi:",
            "    jmp pin hi_dec",
            "    jmp end_hi",
            "hi_dec:",
            "    jmp x-- hi",
            "    jmp hi",
            "end_hi:",
            "    mov isr, !x",
            "    push noblock",
            "    mov x, !null",
        );
        let loaded = common.load_program(&prg.program);
        let mut cfg = Config::default();
        cfg.use_program(&loaded, &[]);
        cfg.fifo_join = FifoJoin::RxOnly;
        let mut exec = cfg.get_exec();
        // JMP PIN counts from GPIOBASE (16 for GPIO43), like every pin selection.
        exec.jmp_pin = PIN_IN_WINDOW;
        // SAFETY: only the JMP PIN source changes; the wrap and side-set settings stay as
        // `use_program` wrote them.
        unsafe { cfg.set_exec(exec) };
        Self {
            sm,
            dma,
            cfg,
            clk: clk_sys_freq(),
        }
    }

    /// Records `buf.len()` counts while `during` runs. The first element of the result is false if
    /// the counts did not all arrive within 50 ms.
    async fn record<F: Future>(&mut self, buf: &mut [u32], during: F) -> (bool, F::Output) {
        self.sm.set_enable(false);
        self.sm.restart();
        self.sm.clear_fifos();
        // Also jumps to the program origin. With no pin in its pin ranges it writes GPIOBASE = 0.
        self.sm.set_config(&self.cfg);
        // The pad is GPIO43: the block's GPIOBASE must be 16 for the JMP PIN field to point at it.
        embassy_rp::pac::PIO2
            .gpiobase()
            .write(|w| w.set_gpiobase(true));
        // The pad is idle low until `during` starts, so nothing arrives before the DMA is armed.
        self.sm.set_enable(true);
        let transfer = self.sm.rx().dma_pull(&mut self.dma, buf, false);
        let (captured, out) = join(with_timeout(Duration::from_millis(50), transfer), during).await;
        self.sm.set_enable(false);
        (captured.is_ok(), out)
    }
}

/// One frame of `TEST_COLOR` captured while `during` sends it. None if the capture timed out.
async fn waveform<F: Future>(cap: &mut Capture<'_>, during: F) -> Option<Frame> {
    let mut buf = [0xDEAD_BEEF_u32; FRAME_COUNTS];
    let (done, _) = cap.record(&mut buf, during).await;
    if !done {
        defmt::info!(
            "[INFO] capture timed out; first counts {=u32:#x} {=u32:#x} {=u32:#x}",
            buf[0],
            buf[1],
            buf[2]
        );
        return None;
    }
    Some(analyse(&buf[1..], TEST_COLOR.wire(), cap.clk))
}

/// Two frames of `TEST_COLOR` back to back: both analyses and the low time between them in ns.
async fn two_frames<F: Future>(cap: &mut Capture<'_>, during: F) -> Option<(Frame, Frame, u32)> {
    let mut buf = [0xDEAD_BEEF_u32; TWO_FRAME_COUNTS];
    let (done, _) = cap.record(&mut buf, during).await;
    if !done {
        defmt::info!(
            "[INFO] capture timed out; first counts {=u32:#x} {=u32:#x} {=u32:#x}",
            buf[0],
            buf[1],
            buf[2]
        );
        return None;
    }
    let word = TEST_COLOR.wire();
    let a = analyse(&buf[1..FRAME_COUNTS], word, cap.clk);
    let gap = ns_low(buf[FRAME_COUNTS], cap.clk);
    let b = analyse(&buf[FRAME_COUNTS + 1..], word, cap.clk);
    Some((a, b, gap))
}

/// Verdict for one captured frame: the right 24 bits and every time inside the limits.
fn check_frame(rep: &mut Report, label: &str, f: &Option<Frame>) {
    let Some(f) = f else {
        check!(rep, false, "{=str} {=str}: no waveform captured", label);
        return;
    };
    report_frame(label, f);
    let sent = TEST_COLOR.wire() >> 8;
    check!(
        rep,
        f.bits == sent && f.bad == 0,
        "{=str} {=str}: decoded {=u32:#08x}, sent {=u32:#08x}, {=u32} times outside the limits or wrong",
        label,
        f.bits,
        sent,
        f.bad
    );
}

/// Verdict for two frames: the gap between them is over the reset time, both frames are right.
fn check_gap(rep: &mut Report, label: &str, r: &Option<(Frame, Frame, u32)>) {
    let Some((a, b, gap)) = r else {
        check!(rep, false, "{=str} {=str}: no waveform captured", label);
        return;
    };
    defmt::info!(
        "[STAT] {=str}: low time between the frames {=u32} us",
        label,
        *gap / 1000
    );
    check!(
        rep,
        *gap > RESET_MIN_US * 1000,
        "{=str} {=str}: low time between two set calls {=u32} us (limit over {=u32} us)",
        label,
        *gap / 1000,
        RESET_MIN_US
    );
    let sent = TEST_COLOR.wire() >> 8;
    check!(
        rep,
        a.bits == sent && b.bits == sent && a.bad == 0 && b.bad == 0,
        "{=str} {=str}: both frames decode to the sent bits inside the limits",
        label
    );
}

/// The SPI1 clock the controller really makes, from its registers.
fn spi1_hz() -> u32 {
    let spi1 = embassy_rp::pac::SPI1;
    let prescale = spi1.cpsr().read().cpsdvsr() as u32;
    let post = spi1.cr0().read().scr() as u32 + 1;
    clk_peri_freq() / (prescale * post)
}

fn spi_config(phase: Phase, polarity: Polarity) -> spi::Config {
    let mut cfg = spi::Config::default();
    cfg.frequency = SPI_NOMINAL_HZ;
    cfg.phase = phase;
    cfg.polarity = polarity;
    cfg
}

/// The four SPI modes: (name, phase, polarity).
const MODES: [(&str, Phase, Polarity); 4] = [
    (
        "mode 0 (CPOL 0, CPHA 0)",
        Phase::CaptureOnFirstTransition,
        Polarity::IdleLow,
    ),
    (
        "mode 1 (CPOL 0, CPHA 1)",
        Phase::CaptureOnSecondTransition,
        Polarity::IdleLow,
    ),
    (
        "mode 2 (CPOL 1, CPHA 0)",
        Phase::CaptureOnFirstTransition,
        Polarity::IdleHigh,
    ),
    (
        "mode 3 (CPOL 1, CPHA 1)",
        Phase::CaptureOnSecondTransition,
        Polarity::IdleHigh,
    ),
];
/// The SPI mode W3 verifies. The trial over `MODES` shows that on this controller (PL022) CPHA 0
/// leaves a gap of about 0.5 us between bytes (chip select would pulse between frames), and CPHA 1
/// shifts bytes back to back. CPOL only sets the idle level of SCK, which nothing listens to.
const SPI_MODE: usize = 1;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut rep = Report { failures: 0 };
    defmt::info!(
        "[INFO] ws2812c_smoke: WS2812C-2020 D20 on GPIO43, clk_sys {=u32} Hz, clk_peri {=u32} Hz",
        clk_sys_freq(),
        clk_peri_freq()
    );

    let Pio {
        common: mut cap_common,
        sm0: cap_sm,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let mut cap = Capture::new(&mut cap_common, cap_sm, dma::Channel::new(p.DMA_CH0, Irqs));

    // W1, W2: the PIO transport.
    {
        let Pio {
            mut common, sm0, ..
        } = Pio::new(p.PIO1.reborrow(), Irqs);
        let bus = PioTransport::new(&mut common, sm0, p.PIN_43.reborrow()).unwrap();
        let t = bus.timing();
        let b = t.bit_times();
        defmt::info!(
            "[INFO] PIO timing: divider {=u16}, cycles t1 {=u8} t2 {=u8} t3 {=u8}; designed T0H {=u32} T1H {=u32} T0L {=u32} T1L {=u32} ns",
            t.divider,
            t.t1,
            t.t2,
            t.t3,
            b.t0h_ns,
            b.t1h_ns,
            b.t0l_ns,
            b.t1l_ns
        );
        let mut led = Ws2812c::new(bus);

        let f = waveform(&mut cap, async {
            led.set(TEST_COLOR).await.unwrap();
        })
        .await;
        check_frame(&mut rep, "W1 PIO", &f);

        let g = two_frames(&mut cap, async {
            led.set(TEST_COLOR).await.unwrap();
            led.set(TEST_COLOR).await.unwrap();
        })
        .await;
        check_gap(&mut rep, "W2 PIO", &g);
        led.off().await.unwrap();
    }

    // W3: the SPI transport on SPI1, TX on GPIO43, SCK on the unconnected GPIO26.
    {
        let mut ok = [false; 4];
        let mut made = 0;
        for (i, (name, phase, polarity)) in MODES.iter().enumerate() {
            let bus = Spi::new_txonly(
                p.SPI1.reborrow(),
                p.PIN_26.reborrow(),
                p.PIN_43.reborrow(),
                p.DMA_CH1.reborrow(),
                Irqs,
                spi_config(*phase, *polarity),
            );
            made = spi1_hz();
            let mut led = Ws2812c::new(SpiTransport::new(bus));
            let f = waveform(&mut cap, async {
                led.set(TEST_COLOR).await.unwrap();
            })
            .await;
            match f {
                Some(f) => {
                    ok[i] = f.bits == TEST_COLOR.wire() >> 8 && f.bad == 0;
                    report_frame(name, &f);
                    defmt::info!(
                        "[INFO] SPI {=str}: {=u32} times outside the limits, decoded {=u32:#08x}",
                        name,
                        f.bad,
                        f.bits
                    );
                }
                None => defmt::info!("[INFO] SPI {=str}: no waveform captured", name),
            }
            let _ = led.off().await;
        }
        defmt::info!(
            "[INFO] SPI modes inside the limits (0..3): {=bool} {=bool} {=bool} {=bool}; 8-bit frames (embassy-rp fixes the frame size); clock {=u32} Hz (allowed {=u32}..{=u32})",
            ok[0],
            ok[1],
            ok[2],
            ok[3],
            made,
            SPI_MIN_HZ,
            SPI_MAX_HZ
        );
        let (name, phase, polarity) = MODES[SPI_MODE];
        let bus = Spi::new_txonly(
            p.SPI1.reborrow(),
            p.PIN_26.reborrow(),
            p.PIN_43.reborrow(),
            p.DMA_CH1.reborrow(),
            Irqs,
            spi_config(phase, polarity),
        );
        let mut led = Ws2812c::new(SpiTransport::new(bus));
        defmt::info!("[INFO] W3 uses SPI1 {=str}, {=u32} Hz", name, spi1_hz());
        let f = waveform(&mut cap, async {
            led.set(TEST_COLOR).await.unwrap();
        })
        .await;
        check_frame(&mut rep, "W3 SPI", &f);
        let g = two_frames(&mut cap, async {
            led.set(TEST_COLOR).await.unwrap();
            led.set(TEST_COLOR).await.unwrap();
        })
        .await;
        check_gap(&mut rep, "W3 SPI", &g);
        led.off().await.unwrap();
    }

    // W4, W5: the PIO transport again, the blocking front-end first.
    {
        let Pio {
            mut common, sm0, ..
        } = Pio::new(p.PIO1.reborrow(), Irqs);
        let bus = PioTransport::new(&mut common, sm0, p.PIN_43.reborrow()).unwrap();
        let mut blocking_led = blocking::Ws2812c::new(bus);
        // The blocking send runs to completion inside the future, while the DMA drains the
        // capture on its own.
        let f = waveform(&mut cap, async {
            blocking_led.set(TEST_COLOR).unwrap();
        })
        .await;
        check_frame(&mut rep, "W4 blocking", &f);
        blocking_led.off().unwrap();

        let mut led = Ws2812c::new(blocking_led.release());
        led.set_brightness(32);
        defmt::info!("[INFO] manual check: the LED shows red, green, blue, white, then off");
        for color in [Rgb::RED, Rgb::GREEN, Rgb::BLUE, Rgb::WHITE] {
            led.set(color).await.unwrap();
            Timer::after_millis(600).await;
        }
        led.off().await.unwrap();
        // Keep the line driven low until the LED has latched the off frame; releasing the PIO
        // lets the pad float.
        Timer::after_millis(1).await;
    }

    hil_common::exit(rep.failures)
}
