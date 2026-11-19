//! The status LED (D20, one WS2812C on GPIO43): a core-0 task on PIO1 state machine 1 that shows the
//! boot roll call, the running display and the one-off flashes of `fw_core::led`
//! (`decision-status-led-20261118`). Every 20 ms it gathers what the display reads (the device rows,
//! the motor spin, the calibration flag, the open log, the GNSS fix, `LED_BRIGHT`), asks
//! `fw_core::led` and writes the LED only when the colour or the brightness changed.
//!
//! Nothing depends on the LED: if the transport cannot be built the task logs once and ends. There is
//! no LED device row, and no cross-core cell (every input is readable on core 0).

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_rp::peripherals::{PIN_43, PIO1};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{Peri, bind_interrupts};
use embassy_time::{Duration, Instant, Ticker};
use fw_core::health::State;
use fw_core::led::{Inputs, Led, Output, ROWS};
use fw_core::watchdog::task;
use fw_flight::tagged;
use mavlink_device::ParamValue;
use mavlink_device::marv::MarvDevice;
use ws2812c::pio::PioTransport;
use ws2812c::{Rgb, Ws2812c};

use crate::device::{BARO, BATT, GNSS, IMU1, IMU2, MAG, Reading, find};

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

/// How often the task looks at the inputs (ms).
const TICK_MS: u64 = 20;
/// `LED_BRIGHT` is read from the store every this many ticks (13 x 20 ms = 260 ms), not at every tick: the
/// name lookup holds the store's critical section, whose spinlock both cores share.
const LEVEL_EVERY: u32 = 13;
/// The latest GNSS sample counts for the green pulse while it is younger than this (us).
const GNSS_FRESH_US: u64 = 3_000_000;
/// `LED_BRIGHT` when the store cannot be read (the table's default).
const DEFAULT_LEVEL: i32 = 1;

const NONE: u8 = 0;
const GOOD: u8 = 1;
const BAD: u8 = 2;

/// The event waiting for the task: [`NONE`], [`GOOD`] or [`BAD`]. A newer event replaces an older one.
static EVENT: AtomicU8 = AtomicU8::new(NONE);

/// A settings save or a calibration ended: `good` (accepted) three green flashes, otherwise three red.
/// Any core-0 code may call it; the task shows it once the roll call is over.
pub fn event(good: bool) {
    EVENT.store(if good { GOOD } else { BAD }, Ordering::Relaxed);
}

/// `LED_BRIGHT` from the store.
fn level() -> i32 {
    crate::store::with(|p| match p.get_named(fw_settings::led::BRIGHT) {
        Some(ParamValue::I32(v)) => Some(v),
        _ => None,
    })
    .flatten()
    .unwrap_or(DEFAULT_LEVEL)
}

/// Whether a log is open on the SD card (a build of the SD probe has no log: not a reason for yellow).
fn log_open() -> bool {
    #[cfg(not(feature = "sd-fat-probe"))]
    return crate::logger::OPEN.load(Ordering::Relaxed) != 0;
    #[cfg(feature = "sd-fat-probe")]
    return true;
}

/// Everything the display reads, now, with the level last read from the store.
fn inputs(now_us: u64, level: i32) -> Inputs {
    let sd = find(MarvDevice::MARV_DEVICE_SD).map_or(State::Absent, |d| d.row().state);
    let rows: [State; ROWS] = [
        IMU1.row().state,
        IMU2.row().state,
        BARO.row().state,
        MAG.row().state,
        GNSS.row().state,
        BATT.row().state,
        crate::rc::row().state,
        sd,
        crate::esc::row().state,
    ];
    let gnss_fix = GNSS.latest.try_get().is_some_and(|s| {
        now_us.saturating_sub(s.t_us) < GNSS_FRESH_US
            && matches!(s.reading, Reading::Gnss { fix_type, .. } if fix_type >= 3)
    });
    Inputs {
        rows,
        motor_spin: crate::actions::motor_spin(),
        calibrating: crate::calibration::calibrating(),
        log_open: log_open(),
        gnss_fix,
        level,
    }
}

/// The status LED's task (core 0). Spawned in `main` with PIO1 and GPIO43.
#[embassy_executor::task]
pub async fn task(pio: Peri<'static, PIO1>, pin: Peri<'static, PIN_43>) {
    tagged(&crate::watchdog::RUNNING0, task::LED, run(pio, pin)).await
}

async fn run(pio: Peri<'static, PIO1>, pin: Peri<'static, PIN_43>) {
    let Pio {
        mut common, sm1, ..
    } = Pio::new(pio, Irqs);
    let bus = match PioTransport::new(&mut common, sm1, pin) {
        Ok(bus) => bus,
        Err(e) => {
            defmt::warn!(
                "[INFO] LED: no PIO timing at this clock ({}): the status LED stays off",
                e
            );
            return;
        }
    };
    let mut strip = Ws2812c::new(bus);
    let mut led = Led::new();
    let mut shown: Option<Output> = None;
    let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
    let mut tick: u32 = 0;
    let mut level_now = DEFAULT_LEVEL;
    loop {
        if tick.is_multiple_of(LEVEL_EVERY) {
            level_now = level();
        }
        tick = tick.wrapping_add(1);
        match EVENT.swap(NONE, Ordering::Relaxed) {
            GOOD => led.event(true),
            BAD => led.event(false),
            _ => {}
        }
        let now = Instant::now();
        let out = led.show(
            &inputs(now.as_micros(), level_now),
            u32::try_from(now.as_millis()).unwrap_or(u32::MAX),
        );
        if shown != Some(out) {
            strip.set_brightness(out.brightness);
            let _ = strip
                .set(Rgb::new(out.colour.r, out.colour.g, out.colour.b))
                .await;
            shown = Some(out);
        }
        ticker.next().await;
    }
}
