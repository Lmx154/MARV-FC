//! On-board check of the icm45686 driver on the MARV V2 Rev 1 board (U21, SPI0, INT1 on GPIO15).
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_15, PWM_SLICE7};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::{AccelFullScale, ClockSource, Config, GyroFullScale, Icm45686, Odr, RawSample};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const WINDOW: Duration = Duration::from_secs(2);
const STALL: Duration = Duration::from_millis(100);

/// INT1 used twice: a PWM slice counts every rising edge in hardware, the GPIO edge interrupt wakes the
/// reader.
struct DataReady<'d> {
    input: Input<'d>,
    pwm: Pwm<'d>,
    last: u16,
    wraps: u64,
}

impl<'d> DataReady<'d> {
    fn new(slice: Peri<'d, PWM_SLICE7>, pin: Peri<'d, PIN_15>) -> Self {
        // SAFETY: the Input only arms GPIO edge interrupts and the PWM slice only counts; neither
        // reconfigures the pad after construction.
        let input = Input::new(unsafe { pin.clone_unchecked() }, Pull::None);
        let pwm = Pwm::new_input(
            slice,
            pin,
            Pull::None,
            InputMode::RisingEdge,
            pwm::Config::default(),
        );
        let last = pwm.counter();
        Self {
            input,
            pwm,
            last,
            wraps: 0,
        }
    }

    fn count(&mut self) -> u64 {
        let c = self.pwm.counter();
        if c < self.last {
            self.wraps += 1;
        }
        self.last = c;
        (self.wraps << 16) | u64::from(c)
    }

    /// Waits until the hardware edge count exceeds `seen`; returns the count, or `None` after `timeout`.
    async fn wait_beyond(&mut self, seen: u64, timeout: Duration) -> Option<u64> {
        let Self {
            input,
            pwm,
            last,
            wraps,
        } = self;
        let wait = async {
            loop {
                let mut edge = pin!(input.wait_for_rising_edge());
                let c = poll_fn(|cx| {
                    let fired = edge.as_mut().poll(cx).is_ready();
                    let raw = pwm.counter();
                    if raw < *last {
                        *wraps += 1;
                    }
                    *last = raw;
                    let c = (*wraps << 16) | u64::from(raw);
                    if c > seen {
                        Poll::Ready(Some(c))
                    } else if fired {
                        Poll::Ready(None)
                    } else {
                        Poll::Pending
                    }
                })
                .await;
                if let Some(c) = c {
                    return c;
                }
            }
        };
        with_timeout(timeout, wait).await.ok()
    }
}

struct Window {
    received: u32,
    produced: u64,
    errors: u32,
    stalls: u32,
    rate_hz: f64,
    read_mean_us: f32,
    read_worst_us: u32,
    mean: RawSample,
}

async fn window<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, dr: &mut DataReady<'_>) -> Window {
    let start = dr.count();
    let t_start = Instant::now();
    let end = t_start + WINDOW;
    let mut seen = start;
    let mut w = Window {
        received: 0,
        produced: 0,
        errors: 0,
        stalls: 0,
        rate_hz: 0.0,
        read_mean_us: 0.0,
        read_worst_us: 0,
        mean: RawSample::default(),
    };
    let mut first: Option<(u64, Instant)> = None;
    let mut last: Option<(u64, Instant)> = None;
    let mut read_sum_us = 0u64;
    let mut sum = [0i64; 7];
    let mut draining = false;
    let mut stop = 0u64;
    loop {
        if !draining && Instant::now() >= end {
            draining = true;
            stop = dr.count();
        }
        if draining && seen >= stop {
            break;
        }
        match dr.wait_beyond(seen, STALL).await {
            None => {
                w.stalls += 1;
                if draining {
                    break;
                }
            }
            Some(c) => {
                seen = c;
                let t = Instant::now();
                first.get_or_insert((c, t));
                last = Some((c, t));
                let r = imu.read().await;
                let us = t.elapsed().as_micros() as u32;
                read_sum_us += u64::from(us);
                w.read_worst_us = w.read_worst_us.max(us);
                match r {
                    Ok(s) => {
                        w.received += 1;
                        for (acc, v) in sum.iter_mut().zip(
                            s.accel
                                .iter()
                                .chain(&s.gyro)
                                .chain(core::iter::once(&s.temp)),
                        ) {
                            *acc += i64::from(*v);
                        }
                    }
                    Err(_) => w.errors += 1,
                }
            }
        }
    }
    w.produced = seen - start;
    if let (Some((c0, t0)), Some((c1, t1))) = (first, last) {
        if c1 > c0 {
            w.rate_hz = (c1 - c0) as f64 * 1_000_000.0 / (t1 - t0).as_micros() as f64;
        }
    }
    let n = i64::from(w.received.max(1));
    let m = |i: usize| (sum[i] / n) as i16;
    w.mean = RawSample {
        accel: [m(0), m(1), m(2)],
        gyro: [m(3), m(4), m(5)],
        temp: m(6),
    };
    w.read_mean_us = read_sum_us as f32 / w.received.max(1) as f32;
    w
}

fn verdict(failures: &mut u32, ok: bool, name: &str) {
    if ok {
        defmt::info!("[PASS] {=str}", name);
    } else {
        *failures += 1;
        defmt::error!("[FAIL] {=str}", name);
    }
}

/// Accel magnitude in g and the worst gyro axis in degrees per second, from a mean sample.
fn at_rest(failures: &mut u32, name: &str, s: &RawSample, config: &Config) {
    let a = s.accel_ms2(config.accel_fs);
    let g = s.gyro_rads(config.gyro_fs);
    let t = s.temperature_c();
    let a2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (9.80665 * 9.80665);
    let gyro_max = 5.0_f32.to_radians();
    defmt::info!(
        "[STAT] {=str} accel_g^2={} (limit 0.81..1.21) gyro_rad_s={} {} {} (limit {}) temp_c={}",
        name,
        a2,
        g[0],
        g[1],
        g[2],
        gyro_max,
        t
    );
    let ok =
        a2 > 0.81 && a2 < 1.21 && g.iter().all(|v| v.abs() <= gyro_max) && t > 10.0 && t < 50.0;
    verdict(failures, ok, "plausible at rest");
}

fn report(failures: &mut u32, name: &str, nominal: f32, w: &Window, rate_tol: f64) {
    let missed = w.produced.saturating_sub(u64::from(w.received));
    let nominal = f64::from(nominal);
    defmt::info!(
        "[STAT] {=str} samples={} edges={} missed={} bus_errors={} stalls={} rate_hz={} nominal={} ratio={} read_mean_us={} read_worst_us={}",
        name,
        w.received,
        w.produced,
        missed,
        w.errors,
        w.stalls,
        w.rate_hz,
        nominal,
        w.rate_hz / nominal,
        w.read_mean_us,
        w.read_worst_us
    );
    verdict(
        failures,
        w.received > 0 && w.errors == 0 && w.stalls == 0 && missed == 0,
        "one read per data-ready edge, none missed",
    );
    let ok = (w.rate_hz / nominal - 1.0).abs() < rate_tol;
    verdict(failures, ok, "rate within tolerance of nominal");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    let mut dr = DataReady::new(p.PWM_SLICE7.reborrow(), p.PIN_15.reborrow());

    {
        let bus = Spi::new(
            p.SPI0.reborrow(),
            p.PIN_18.reborrow(),
            p.PIN_19.reborrow(),
            p.PIN_16.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            Irqs,
            spi_config.clone(),
        );
        let cs = Output::new(p.PIN_20.reborrow(), Level::High);
        let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut imu = Icm45686::new(device);
        let mut delay = Delay;
        let base = Config {
            accel_fs: AccelFullScale::G4,
            gyro_fs: GyroFullScale::Dps250,
            int1_data_ready: true,
            ..Config::default()
        };

        for (name, odr, clock, tol) in [
            (
                "800 Hz internal clock",
                Odr::Hz800,
                ClockSource::Internal,
                0.02,
            ),
            (
                "6400 Hz internal clock",
                Odr::Hz6400,
                ClockSource::Internal,
                0.02,
            ),
            (
                "200 Hz external clock",
                Odr::Hz200,
                ClockSource::ExternalClkin,
                0.0005,
            ),
        ] {
            let config = Config { odr, clock, ..base };
            if let Err(e) = imu.init(&mut delay, &config).await {
                failures += 1;
                defmt::error!("[FAIL] init ({=str}): {}", name, defmt::Debug2Format(&e));
                continue;
            }
            let id = imu.read_register(icm45686::reg::WHO_AM_I).await;
            verdict(
                &mut failures,
                matches!(id, Ok(icm45686::reg::WHO_AM_I_VALUE)),
                "WHO_AM_I == 0xE9",
            );
            let w = window(&mut imu, &mut dr).await;
            report(&mut failures, name, odr.hz(), &w, tol);
            at_rest(&mut failures, name, &w.mean, &config);
        }
    }

    {
        let bus = Spi::new_blocking(
            p.SPI0.reborrow(),
            p.PIN_18.reborrow(),
            p.PIN_19.reborrow(),
            p.PIN_16.reborrow(),
            spi_config,
        );
        let cs = Output::new(p.PIN_20.reborrow(), Level::High);
        let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut imu = icm45686::blocking::Icm45686::new(device);
        let config = Config {
            accel_fs: AccelFullScale::G4,
            gyro_fs: GyroFullScale::Dps250,
            ..Config::default()
        };
        match imu.init(&mut Delay, &config).and_then(|()| imu.read()) {
            Ok(s) => at_rest(&mut failures, "blocking front-end", &s, &config),
            Err(e) => {
                failures += 1;
                defmt::error!("[FAIL] blocking init/read: {}", defmt::Debug2Format(&e));
            }
        }
    }

    hil_common::exit(failures)
}
