//! The four ESCs: bidirectional DShot300 at 1 kHz on PIO2 SM0, GPIO36-39 (driver line 0 = GPIO36 = M4
//! ... line 3 = GPIO39 = M1), the reply capture on DMA channel 5 (`dshot` `PioDshot`). Design:
//! `decision-esc-telemetry-20261118` Q1-Q3. Stop frames from every boot, and command 13 in the EDT
//! bursts of `fw_flight::esc`; no throttle.
//!
//! [`driver`] and [`task`] run on core 1: [`driver`] is called inside core 1's entry, so
//! `PIO2_IRQ_0` and the DMA channel's `DMA_IRQ_0` are enabled there, and [`task`] runs on core 1's
//! executor (no timer, no log, no lock: it waits with `SpinDelay` and publishes to [`ESC`]). The
//! rest runs on core 0: the ESC row, ESC_STATUS and ESC_INFO (rpm with `MOT_POLES`), the `[STAT]`
//! line.

use embassy_rp::peripherals::{DMA_CH5, PIN_36, PIN_37, PIN_38, PIN_39, PIO2};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::Instant;
use embedded_hal_async::delay::DelayNs;
use fw_core::health::ErrorKind;
use fw_core::sys_status::DeviceRow;
use fw_core::xcore::Seqlock;
use fw_flight::esc::{Esc, EscStatus, next_deadline};
use mavlink_device::ParamValue;
use mavlink_device::dialect::{ESC_INFO_DATA, ESC_STATUS_DATA};
use mavlink_device::marv::{MARV_DEVICE_STATUS_DATA, MarvDevice};

use crate::imu1::SpinDelay;

bind_interrupts!(pub struct Irqs {
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
});

/// Flight core -> core 0: the ESC task's status after every exchange (one writer, the task).
pub static ESC: Seqlock<EscStatus> = Seqlock::new(EscStatus::INITIAL);

/// Copies of [`ESC`] tried before core 0 gives up (as the IMU status).
const READ_ATTEMPTS: u32 = 64;

/// `MOT_POLES` when the store cannot be read (the table's default).
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
    /// `clk_sys` in Hz, read on core 0 (no `clocks` call on core 1).
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
pub async fn task(mut drv: Driver) -> ! {
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
        let frames = esc.frames(t0, fw_flight::esc::TestCmd::INITIAL);
        #[cfg(feature = "esc-no-edt")]
        let frames = frames.map(|_| dshot::LineFrame::STOP);
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

// ---------------------------------------------------------------------------------------------
// Core 0.
// ---------------------------------------------------------------------------------------------

/// The latest status (core 0); the initial status if no consistent copy was had, which the row
/// shows as stuck.
fn snapshot() -> EscStatus {
    ESC.try_read(READ_ATTEMPTS).unwrap_or(EscStatus::INITIAL)
}

/// `MOT_POLES` from the store.
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

/// The ESC row's `MARV_DEVICE_STATUS` (report only, recomputed for every message).
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
        "[STAT] ESC: state {=str} error {=str}, {=u32} exchanges, {=u32} starts, longest spacing {=u32} us, longest exchange {=u32} us, shortest exchange {=u32} us; good/bad/dropouts/EDT bursts, V mV, T degC, I mA: M1 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M2 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M3 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}; M4 {=u32}/{=u32}/{=u32}/{=u32}, {=i64}, {=i64}, {=i64}",
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
        m4.6
    );
}
