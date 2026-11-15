//! MARV V2 firmware, the two-core skeleton (design: decision-foundation, two-core amendment).
//!
//! Core 0, the I/O core: `usb_link` (a MAVLink node, system 1 component 1, on USB), `supervisor`
//! (10 ms tick: the flight core's liveness ping at 10 Hz, the start-up checks, a `[STAT]` line
//! every 10 s) and `beat0`. Core 1, the flight core: only `fw_flight::liveness`. The cores share
//! only the atomics and `fw_core::xcore` cells below, in internal SRAM. No device, no settings, no
//! watchdog, no commands yet. It runs until `tools/run.sh` ends it.
//!
//! USB identity: VID 0xc0de, PID 0xcafe, "MARV", "MARV V2 firmware", "marv-v2-fw".
//! Feature `no-fbeat` (negative control): core 1 starts but its liveness task is not spawned.
#![no_std]
#![no_main]

use core::any::type_name;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
use embedded_io_async::{ErrorType, Read, Write};
use fw_core::params::{ParamDef, Params};
use fw_core::sys_status::{sensor_masks, system_status};
use fw_core::xcore::Kick;
use mavlink_device::dialect::{MavSeverity, MavState, MavSysStatusSensor, SYS_STATUS_DATA};
use mavlink_device::{
    Config as NodeConfig, Node, Outgoing, ParamStore, RunError, Stream, StreamSlot, Telemetry,
    Texts,
};
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};
use usb_cdc::UsbCdc;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

// ---------------------------------------------------------------------------------------------
// Shared between the cores: plain atomics and `xcore` cells only.
// ---------------------------------------------------------------------------------------------

/// Core 0 -> flight core: the liveness ping, bumped by the supervisor at 10 Hz.
static PING: AtomicU32 = AtomicU32::new(0);
/// Flight core -> core 0: the last `PING` the flight core copied.
static PONG: AtomicU32 = AtomicU32::new(0);
/// The flight core's one wake.
static KICK: Kick = Kick::new();
/// Core 1's ACTLR as it read it at the top of its entry; valid once `CORE1_ACTLR_READ` is true.
static CORE1_ACTLR: AtomicU32 = AtomicU32::new(0);
static CORE1_ACTLR_READ: AtomicBool = AtomicBool::new(false);

/// Core 1's stack and executor.
const CORE1_STACK_BYTES: usize = 16 * 1024;
static CORE1_STACK: ConstStaticCell<Stack<CORE1_STACK_BYTES>> = ConstStaticCell::new(Stack::new());
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// ---------------------------------------------------------------------------------------------
// Core 0 only.
// ---------------------------------------------------------------------------------------------

/// Core 0's executor beat, every 50 ms (the watchdog monitor will read it).
static BEAT0: AtomicU32 = AtomicU32::new(0);
/// What the node wrote and read on USB: bytes, and frames written (one `write_all` per frame).
static TX_BYTES: AtomicU32 = AtomicU32::new(0);
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
/// The USB link as `usb_link` sees it: `LINK_DOWN`, `LINK_GATE` (up, start gate closed), `LINK_RUN`.
static LINK_STATE: AtomicU8 = AtomicU8::new(LINK_DOWN);
const LINK_DOWN: u8 = 0;
const LINK_GATE: u8 = 1;
const LINK_RUN: u8 = 2;
/// The supervisor's STATUSTEXTs for the node's text hook (the design's ring of 8 is a later packet).
static TEXTS: Channel<CriticalSectionRawMutex, (MavSeverity, &'static str), 4> = Channel::new();
/// The parameter table: empty until the devices bring theirs.
static PARAM_TABLE: [ParamDef; 0] = [];

const KICK_PING: u8 = 1;
const EXTEXCLALL: u32 = 1 << 29;
const PONG_STALE: Duration = Duration::from_millis(200);
const SYS_STATUS_ID: u32 = 1;
const TEXT_UP: &str = concat!("MARV V2 fw ", env!("CARGO_PKG_VERSION"), ": flight core up");

fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// This core's ACTLR.
fn read_actlr() -> u32 {
    // SAFETY: a read of the executing core's own ACTLR in the System Control Space; no side effect.
    unsafe { (*cortex_m::peripheral::ICB::PTR).actlr.read() }
}

fn check(pass: bool, what: &str, value: u32) {
    if pass {
        defmt::info!("[PASS] {=str} ({=u32:#010x})", what, value);
    } else {
        defmt::error!("[FAIL] {=str} ({=u32:#010x})", what, value);
    }
}

// ---------------------------------------------------------------------------------------------
// Core 1, the flight core.
// ---------------------------------------------------------------------------------------------

#[cfg(not(feature = "no-fbeat"))]
#[embassy_executor::task]
async fn fbeat() -> ! {
    fw_flight::liveness(&KICK, &PING, &PONG).await
}

fn core1_main() -> ! {
    CORE1_ACTLR.store(read_actlr(), Ordering::Relaxed);
    CORE1_ACTLR_READ.store(true, Ordering::Release);
    let executor = EXECUTOR1.init(Executor::new());
    executor.run(|spawner| {
        #[cfg(not(feature = "no-fbeat"))]
        if let Ok(token) = fbeat() {
            spawner.spawn(token);
        }
        #[cfg(feature = "no-fbeat")]
        let _ = spawner;
    })
}

// ---------------------------------------------------------------------------------------------
// Core 0, the I/O core.
// ---------------------------------------------------------------------------------------------

/// A USB half that counts what passes through it.
struct Counted<T>(T);

impl<T: ErrorType> ErrorType for Counted<T> {
    type Error = T::Error;
}

impl<T: Read> Read for Counted<T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.0.read(buf).await?;
        RX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
        Ok(n)
    }
}

impl<T: Write> Write for Counted<T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let n = self.0.write(buf).await?;
        TX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush().await
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.0.write_all(buf).await?;
        TX_BYTES.fetch_add(buf.len() as u32, Ordering::Relaxed);
        TX_FRAMES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// The node's hooks: HEARTBEAT status, SYS_STATUS, and the supervisor's texts. No device rows yet.
struct Board;

impl Telemetry for Board {
    fn sample(&mut self, message_id: u32, _now_us: u64, out: &mut Outgoing<'_>) -> bool {
        if message_id != SYS_STATUS_ID {
            return false;
        }
        let masks = sensor_masks(&[]);
        out.push(&SYS_STATUS_DATA {
            onboard_control_sensors_present: MavSysStatusSensor::from_bits_truncate(masks.present),
            onboard_control_sensors_enabled: MavSysStatusSensor::from_bits_truncate(masks.enabled),
            onboard_control_sensors_health: MavSysStatusSensor::from_bits_truncate(masks.health),
            // MAVLink's "not sent" values.
            voltage_battery: u16::MAX,
            current_battery: -1,
            battery_remaining: -1,
            ..SYS_STATUS_DATA::DEFAULT
        })
        .is_ok()
    }

    fn status(&mut self, now_us: u64) -> Option<MavState> {
        Some(system_status(&[], now_us / 1000, false))
    }

    fn texts(&mut self, _now_us: u64, out: &mut Texts<'_>) {
        if let Ok((severity, text)) = TEXTS.try_receive() {
            let _ = out.push(severity, text);
        }
    }
}

fn text(severity: MavSeverity, text: &'static str) {
    if TEXTS.try_send((severity, text)).is_err() {
        defmt::warn!("[INFO] text queue full, dropped: {=str}", text);
    }
}

/// The start gate of `docs/link-contract.md` (MARV-GCS), as in mavlink-device's `mavlink_demo`:
/// nothing is written until a ground-station HEARTBEAT has arrived or DTR has read true at every
/// check for 100 ms. Returns false when a read ends (link down).
async fn start_gate(
    rx: &mut impl Read,
    link: &usb_cdc::Link<'_>,
    node: &mut Node<'_>,
    params: &mut impl ParamStore,
) -> bool {
    let begun = Instant::now();
    let mut dtr_since: Option<Instant> = None;
    let mut buf = [0u8; 64];
    loop {
        match select(rx.read(&mut buf), Timer::after(Duration::from_millis(5))).await {
            Either::First(Ok(n)) if n > 0 => {
                if let Some(bytes) = buf.get(..n) {
                    node.receive(bytes, now_us(), params);
                }
            }
            Either::First(_) => return false,
            Either::Second(()) => {}
        }
        let now = Instant::now();
        let gcs = node.gcs_present(now.as_micros());
        if link.dtr() {
            dtr_since.get_or_insert(now);
        } else {
            dtr_since = None;
        }
        let dtr_held = dtr_since.is_some_and(|t| (now - t) >= Duration::from_millis(100));
        if gcs || dtr_held {
            defmt::info!(
                "[INFO] usb_link: gate open by {=str} after {=u64} ms",
                if gcs { "HEARTBEAT" } else { "DTR" },
                (now - begun).as_millis()
            );
            return true;
        }
    }
}

#[embassy_executor::task]
async fn usb_device(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn usb_link(
    tx: usb_cdc::Tx<'static, UsbDriver>,
    rx: usb_cdc::Rx<'static, UsbDriver>,
    link: usb_cdc::Link<'static>,
) -> ! {
    let (mut tx, mut rx) = (Counted(tx), Counted(rx));
    let mut slots = [StreamSlot::new(Stream {
        message_id: SYS_STATUS_ID,
        default_interval_us: 1_000_000,
        min_interval_us: 100_000,
    })];
    let mut node = Node::new(NodeConfig::new(1, 1), &mut slots);
    let Ok(mut params) = Params::<0>::new(&PARAM_TABLE) else {
        defmt::panic!("[FAIL] parameter table rejected");
    };
    let mut board = Board;
    let mut delay = Delay;
    loop {
        LINK_STATE.store(LINK_DOWN, Ordering::Relaxed);
        link.wait_up().await;
        LINK_STATE.store(LINK_GATE, Ordering::Relaxed);
        if !start_gate(&mut rx, &link, &mut node, &mut params).await {
            node.on_link_down();
            continue;
        }
        LINK_STATE.store(LINK_RUN, Ordering::Relaxed);
        let end = node
            .run(
                &mut rx,
                &mut tx,
                &mut delay,
                now_us,
                &mut params,
                &mut board,
            )
            .await;
        let why = match end {
            RunError::Read(_) => "read error",
            RunError::Write(_) => "write error",
            RunError::EndOfStream => "end of stream",
        };
        defmt::info!(
            "[INFO] usb_link: node stopped ({=str}): {}",
            why,
            node.stats()
        );
        node.on_link_down();
    }
}

/// Every 10 ms: the age of `PONG` (time since it last equalled `PING`); at 10 Hz a new `PING` and a
/// kick of the flight core; the start-up check "PONG follows PING"; the liveness texts; every 10 s
/// a `[STAT]` line.
#[embassy_executor::task]
async fn supervisor() -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut tick: u32 = 0;
    let mut fresh_at = Instant::now();
    let mut max_age_us: u64 = 0;
    let mut first_ping_at: Option<Instant> = None;
    let mut checked = false;
    let mut up_sent = false;
    let mut answering = true;
    loop {
        ticker.next().await;
        let now = Instant::now();
        let ping = PING.load(Ordering::Relaxed);
        let pong = PONG.load(Ordering::Acquire);
        if pong == ping {
            fresh_at = now;
        }
        let age = now - fresh_at;
        max_age_us = max_age_us.max(age.as_micros());

        if let Some(sent) = first_ping_at {
            if !checked && pong == ping {
                checked = true;
                defmt::info!(
                    "[PASS] PONG follows PING within 200 ms ({=u64} us)",
                    (now - sent).as_micros()
                );
            } else if !checked && now - sent > PONG_STALE {
                checked = true;
                defmt::error!("[FAIL] PONG follows PING within 200 ms (PONG {=u32})", pong);
            }
            if !up_sent && pong == ping {
                up_sent = true;
                text(MavSeverity::MAV_SEVERITY_INFO, TEXT_UP);
            }
        }
        if answering && age > PONG_STALE {
            answering = false;
            defmt::warn!(
                "[INFO] flight core not answering (PONG age {=u64} ms)",
                age.as_millis()
            );
            text(
                MavSeverity::MAV_SEVERITY_WARNING,
                "flight core not answering",
            );
        } else if !answering && age <= PONG_STALE {
            answering = true;
            defmt::info!("[INFO] flight core answering");
            text(MavSeverity::MAV_SEVERITY_INFO, "flight core answering");
        }

        if tick.is_multiple_of(10) {
            PING.store(ping.wrapping_add(1), Ordering::Release);
            KICK.kick(KICK_PING);
            first_ping_at.get_or_insert(now);
        }
        if tick.is_multiple_of(1000) && tick > 0 {
            defmt::info!(
                "[STAT] uptime {=u64} s, PONG age max {=u64} us, PING {=u32} PONG {=u32}, usb wrote {=u32} B {=u32} frames, read {=u32} B, link {=str}",
                now.as_secs(),
                max_age_us,
                PING.load(Ordering::Relaxed),
                pong,
                TX_BYTES.load(Ordering::Relaxed),
                TX_FRAMES.load(Ordering::Relaxed),
                RX_BYTES.load(Ordering::Relaxed),
                match LINK_STATE.load(Ordering::Relaxed) {
                    LINK_RUN => "up, node running",
                    LINK_GATE => "up, start gate closed",
                    _ => "down",
                }
            );
        }
        tick = tick.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn beat0() -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        ticker.next().await;
        BEAT0.fetch_add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!(
        "[INFO] MARV V2 fw {=str}: two-core skeleton",
        env!("CARGO_PKG_VERSION")
    );
    defmt::info!("[INFO] icm45686: {}", type_name::<icm45686::Info>());
    defmt::info!("[INFO] lsm6dsk320x: {}", type_name::<lsm6dsk320x::Info>());
    defmt::info!("[INFO] bmp581: {}", type_name::<bmp581::Info>());
    defmt::info!("[INFO] bmm350: {}", type_name::<bmm350::Info>());
    defmt::info!("[INFO] lc76g: {}", type_name::<lc76g::Config>());
    defmt::info!("[INFO] pio-sdmmc: {}", type_name::<pio_sdmmc::Config>());
    defmt::info!("[INFO] qmi-psram: {}", type_name::<qmi_psram::Config>());
    defmt::info!("[INFO] ws2812c: {}", type_name::<ws2812c::Rgb>());

    let actlr0 = read_actlr();
    check(
        actlr0 & EXTEXCLALL != 0,
        "EXTEXCLALL set on core 0, ACTLR",
        actlr0,
    );

    // F4: all DMA channels share DMA_IRQ_0, which belongs to the flight core.
    interrupt::DMA_IRQ_0.disable();
    let dma_irq0_on = interrupt::DMA_IRQ_0.is_enabled();
    check(
        !dma_irq0_on,
        "DMA_IRQ_0 not enabled on core 0, enabled",
        u32::from(dma_irq0_on),
    );

    spawn_core1(p.CORE1, CORE1_STACK.take(), core1_main);

    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("MARV");
    config.product = Some("MARV V2 firmware");
    config.serial_number = Some("marv-v2-fw");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (tx, rx, link) = cdc.split();
    let device = builder.build();
    spawner.spawn(defmt::unwrap!(usb_device(device)));
    spawner.spawn(defmt::unwrap!(usb_link(tx, rx, link)));
    spawner.spawn(defmt::unwrap!(supervisor()));
    spawner.spawn(defmt::unwrap!(beat0()));

    let started = Instant::now();
    while !CORE1_ACTLR_READ.load(Ordering::Acquire) && started.elapsed() < PONG_STALE {
        Timer::after(Duration::from_millis(1)).await;
    }
    let actlr1 = CORE1_ACTLR.load(Ordering::Relaxed);
    check(
        CORE1_ACTLR_READ.load(Ordering::Acquire) && actlr1 & EXTEXCLALL != 0,
        "EXTEXCLALL set on core 1, ACTLR",
        actlr1,
    );
}
