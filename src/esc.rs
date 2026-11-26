//! The four ESCs: bidirectional DShot300 at 1 kHz on PIO2 SM0, GPIO36-39 (driver line 0 = GPIO36 = M4 ... line 3 = GPIO39 = M1), the reply capture on DMA channel 5 (`dshot` `PioDshot`).
//! Stop frames from every boot, and command 13 in the EDT bursts of `fw_flight::esc`; throttle on all four lines from the flight command (`crate::flight::OUT`, Stop when older than `fw_flight::esc::MOTOR_HOLD_MAX_US`),
//! or, while that is Stop, on one line for the props-off motor test (command 209: at most 5 %, 3 s, the command in the cell [`TEST`]).
//! [`driver`] is called inside core 1's entry, so `PIO2_IRQ_0` and the DMA channel's `DMA_IRQ_0` are enabled there; [`task`] runs on core 1's executor (no timer, no log, no lock: it waits with `SpinDelay` and publishes to [`ESC`]).
//! The rest runs on core 0: the ESC row, ESC_STATUS and ESC_INFO (rpm with `MOT_POLES`), the `[STAT]` line.

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_rp::peripherals::{DMA_CH5, PIN_36, PIN_37, PIN_38, PIN_39, PIO2};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::Instant;
use embedded_hal_async::delay::DelayNs;
use fw_core::health::ErrorKind;
use fw_core::sys_status::DeviceRow;
use fw_core::xcore::{Seqlock, TripleBuffer, TripleReader};
use fw_flight::esc::{Esc, EscStatus, TestCause, TestCmd, next_deadline, stale_spin};
use mavlink_device::ParamValue;
use mavlink_device::dialect::{ESC_INFO_DATA, ESC_STATUS_DATA, MavSeverity};
use mavlink_device::marv::{MARV_DEVICE_STATUS_DATA, MarvDevice};

use crate::imu1::SpinDelay;

bind_interrupts!(pub struct Irqs {
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
});

/// Flight core -> core 0: the ESC task's status after every exchange (one writer, the task).
pub static ESC: Seqlock<EscStatus> = Seqlock::new(EscStatus::INITIAL);

/// Core 0 -> flight core: the motor-test command (one writer, `actions`; one reader, the task). The
/// initial value is a stop, so a reset never resumes a test.
static TEST: TripleBuffer<TestCmd> = TripleBuffer::new(TestCmd::INITIAL);

/// The task's reader of [`TEST`]; call once on core 0 before core 1 starts.
pub fn test_reader() -> TripleReader<'static, TestCmd> {
    match TEST.take_reader() {
        Some(reader) => reader,
        None => defmt::panic!("[FAIL] motor-test reader taken twice"),
    }
}

/// [`TEST`]'s writer, for core 0's `actions` state. Called once, on core 0.
pub fn test_writer() -> fw_core::xcore::TripleWriter<'static, TestCmd> {
    match TEST.take_writer() {
        Some(writer) => writer,
        None => defmt::panic!("[FAIL] motor-test writer taken twice"),
    }
}

/// Copies of [`ESC`] tried before core 0 gives up (as the IMU status).
const READ_ATTEMPTS: u32 = 64;

const DEFAULT_POLES: i32 = 14;

/// The peripherals the ESC task owns, moved into core 1's entry; the pins in the driver's order
/// (consecutive, ascending, line 0 first).
pub struct Parts {
    pub pio: Peri<'static, PIO2>,
    pub pin36: Peri<'static, PIN_36>,
    pub pin37: Peri<'static, PIN_37>,
    pub pin38: Peri<'static, PIN_38>,
    pub pin39: Peri<'static, PIN_39>,
    pub dma: Peri<'static, DMA_CH5>,
    pub clk_sys_hz: u32,
}

pub type Driver = dshot::PioDshot<'static, PIO2>;

/// Builds PIO2 and the driver: call on core 1. `None` if the driver refused the parts.
pub fn driver(p: Parts) -> Option<Driver> {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.pio, Irqs);
    let pins = [
        common.make_pio_pin(p.pin36),
        common.make_pio_pin(p.pin37),
        common.make_pio_pin(p.pin38),
        common.make_pio_pin(p.pin39),
    ];
    let dma = dma::Channel::new(p.dma, crate::DmaIrqs);
    let parts = dshot::Parts {
        common,
        sm0,
        pins,
        dma,
    };
    dshot::PioDshot::new(parts, dshot::Rate::Dshot300, p.clk_sys_hz).ok()
}

/// The ESC task (core 1): the start sequence, then one exchange per millisecond for ever. It never
/// stops on errors.
#[embassy_executor::task]
pub async fn task(mut drv: Driver, mut test: TripleReader<'static, TestCmd>) -> ! {
    let Some(mut cell) = ESC.take_writer() else {
        match core::future::pending::<core::convert::Infallible>().await {}
    };
    let mut esc = Esc::new();
    esc.start_begin();
    cell.write(*esc.status());
    drv.start(&mut SpinDelay).await;
    esc.start_end(crate::now_us());
    cell.write(*esc.status());
    let mut deadline = crate::now_us();
    loop {
        let t0 = crate::now_us();
        let (cmd, _) = test.read();
        // One attempt: the writer (IMU1's task) runs on this executor and never yields inside `write`.
        let flight = match crate::flight::OUT.try_read(1) {
            Some(o) => (o.t_us, o.motors),
            None => {
                esc.flight_read_failed();
                (0, fw_flight::gate::MotorCommand::STOP)
            }
        };
        if stale_spin(flight.0, flight.1, t0) {
            crate::settings::HANDSHAKE.disarm();
        }
        let frames = esc.frames(t0, cmd, flight);
        let x = drv.exchange(frames).await;
        let t1 = crate::now_us();
        esc.record(
            t0,
            u32::try_from(t1.saturating_sub(t0)).unwrap_or(u32::MAX),
            &x.captures,
        );
        cell.write(*esc.status());
        deadline = next_deadline(deadline, t1);
        let wait = deadline.saturating_sub(crate::now_us());
        SpinDelay
            .delay_us(u32::try_from(wait).unwrap_or(u32::MAX))
            .await;
    }
}

/// The `seq` of the latest ending [`report`] has printed (0: none; core 0 numbers commands from 1).
static ENDED_SEQ: AtomicU32 = AtomicU32::new(0);

/// The latest status (core 0); the initial status if no consistent copy was had, which the row shows as stuck.
fn snapshot() -> EscStatus {
    ESC.try_read(READ_ATTEMPTS).unwrap_or(EscStatus::INITIAL)
}

pub fn flight_spin() -> bool {
    snapshot().flight_spin
}

/// The ESC row's state and whether flight throttles went out, from one status copy (core 0: the LED).
pub fn led_inputs() -> (fw_core::health::State, bool) {
    let s = snapshot();
    (
        fw_core::esc::verdict(&s, crate::now_us()).state,
        s.flight_spin,
    )
}

/// Every supervisor tick (core 0): one `[INFO]` line for each new motor-test ending in the flight
/// core's status (its `seq` differs from the last one printed), and a WARNING STATUSTEXT for a `Cap`
/// ending (core 0 admits nothing above the cap, so it means the frame guard caught something).
pub fn report() {
    let Some(e) = snapshot().test_ended else {
        return;
    };
    if ENDED_SEQ.swap(e.seq, Ordering::Relaxed) == e.seq {
        return;
    }
    // The element-to-line mapping is its own inverse.
    let motor = fw_core::esc::line_of_element(usize::from(e.line)).map_or(0, |m| m as u8 + 1);
    let cause = match e.cause {
        TestCause::Expiry => "expiry",
        TestCause::Limit => "3 s limit",
        TestCause::Stop => "stop",
        TestCause::Replaced => "replaced",
        TestCause::Cap => "cap",
    };
    match e.first_us {
        Some(first) => defmt::info!(
            "[INFO] motor test M{=u8}: level {=u16}, ended by {=str} after {=u64} ms",
            motor,
            e.level,
            cause,
            e.at_us.saturating_sub(first) / 1000
        ),
        None => defmt::info!(
            "[INFO] motor test M{=u8}: level {=u16}, ended by {=str}, never started",
            motor,
            e.level,
            cause
        ),
    }
    if e.cause == TestCause::Cap {
        crate::text(
            MavSeverity::MAV_SEVERITY_WARNING,
            "motor test: ended by the cap guard",
        );
    }
}

fn poles() -> i32 {
    crate::store::with(|p| match p.get_named(fw_settings::esc::POLES) {
        Some(ParamValue::I32(v)) => Some(v),
        _ => None,
    })
    .flatten()
    .unwrap_or(DEFAULT_POLES)
}

/// ESC_STATUS (291) while the flight core's ESC data is not older than 1 s.
pub fn esc_status(now_us: u64) -> Option<ESC_STATUS_DATA> {
    fw_core::esc::esc_status(&snapshot(), now_us, poles())
}

/// ESC_INFO (290) while the flight core's ESC data is not older than 1 s.
pub fn esc_info(now_us: u64) -> Option<ESC_INFO_DATA> {
    fw_core::esc::esc_info(&snapshot(), now_us)
}

/// The ESC row for SYS_STATUS (no bits) and the HEARTBEAT state.
pub fn row() -> DeviceRow {
    let now_us = crate::now_us();
    let s = snapshot();
    let v = fw_core::esc::verdict(&s, now_us);
    DeviceRow {
        device: MarvDevice::MARV_DEVICE_ESC,
        present: s.lines.iter().any(|l| l.last_good_us.is_some()),
        state: v.state,
        stuck: v.last_error == ErrorKind::Stuck,
        locked: false,
    }
}

pub fn message(now_us: u64) -> MARV_DEVICE_STATUS_DATA {
    fw_core::esc::device_status(&snapshot(), now_us)
}

/// The `[STAT] ESC:` line (core 0, every 10 s): the row, the exchanges, and per motor (M1 = GPIO39
/// ... M4 = GPIO36) good replies, non-replies, dropouts, EDT bursts and the latest raw voltage
/// (mV), temperature (degC) and current (mA), -1 when none was received.
pub fn stat(now: Instant) {
    let now_us = now.as_micros();
    let s = snapshot();
    let v = fw_core::esc::verdict(&s, now_us);
    let m = |e: usize| {
        let l = fw_core::esc::line_of_element(e)
            .and_then(|i| s.lines.get(i))
            .copied()
            .unwrap_or(fw_core::esc::EscLine::INITIAL);
        let raw = |x: Option<(u32, u64)>| x.map_or(-1, |(v, _)| i64::from(v));
        (
            l.good,
            l.non_replies,
            l.dropouts,
            l.edt_bursts,
            raw(l.voltage_mv),
            raw(l.temperature_c.map(|(c, t)| (u32::from(c), t))),
            raw(l.current_ma),
        )
    };
    let (m1, m2, m3, m4) = (m(0), m(1), m(2), m(3));
    defmt::info!(
        "[STAT] ESC: state {=str} error {=str}, {=u32} exchanges, {=u32} starts, longest spacing {=u32} us, longest exchange {=u32} us, shortest exchange {=u32} us; good/bad/dropouts/EDT bursts, V mV, T degC, I mA: M1 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M2 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M3 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M4 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; throttle frames {=u32}",
        crate::device::state_name(v.state),
        crate::device::error_name(v.last_error),
        s.exchanges,
        s.starts,
        s.max_spacing_us,
        s.max_exchange_us,
        if s.min_exchange_us == u32::MAX {
            0
        } else {
            s.min_exchange_us
        },
        m1.0,
        m1.1,
        m1.2,
        m1.3,
        m1.4,
        m1.5,
        m1.6,
        m2.0,
        m2.1,
        m2.2,
        m2.3,
        m2.4,
        m2.5,
        m2.6,
        m3.0,
        m3.1,
        m3.2,
        m3.3,
        m3.4,
        m3.5,
        m3.6,
        m4.0,
        m4.1,
        m4.2,
        m4.3,
        m4.4,
        m4.5,
        m4.6,
        s.throttle_frames
    );
}
