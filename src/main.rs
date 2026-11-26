//! MARV V2 firmware: core 0 is the I/O core (USB MAVLink node, supervisor, devices, radio receiver), core 1 the flight core.
//! The cores share only atomics and `fw_core::xcore` cells in internal SRAM; the hardware watchdog (`watchdog`) is fed only while `BEAT0` and `PONG` advance.
//! Optional features (`no-fbeat`, `inject-*`, `stall-core0`, `f4-negative`, `stack-negative`, `save-loop`, `fake-armed`,
//! `corrupt-newest`, `guard-negative`, `save-foreign`, `hang-*`, `spin-1s`) are fault injections and negative controls for the on-board checks.
#![no_std]
#![no_main]
// The SD task's file-system futures nest deeper than the default limit allows for their layout.
#![recursion_limit = "256"]

mod actions;
mod baro;
mod batt;
mod batt_fs;
mod busclear;
mod calibration;
mod device;
mod esc;
mod flight;
mod flight_latency;
mod gnss;
mod i2cbus;
mod imu1;
mod imu2;
#[cfg(feature = "inject-imu")]
mod imu_inject;
#[cfg(any(feature = "inject", feature = "inject-mag"))]
mod inject;
mod led;
mod logdl;
mod logger;
mod mag;
mod mission;
mod psram;
mod ram;
mod rc;
mod sd;
mod settings;
mod station;
mod store;
mod watchdog;
#[cfg(feature = "xip-park")]
mod xip_park;

use core::any::type_name;
use core::cell::{RefCell, UnsafeCell};
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use defmt_rtt as _;
use device::{
    BARO, BATT, DEVICE_COUNT, DEVICES, Device, GNSS, IMU1, IMU2, MAG, Reading, Sample, Supervision,
    TextLine,
};
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH5, USB};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, dma};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
use embedded_io_async::{ErrorType, Read, Write};
use fw_core::sample::{BaroReading, MagReading};
use fw_core::sys_status::{DeviceRow, sensor_masks, system_status};
use fw_core::text_ring::TextRing;
use fw_core::watchdog::task;
use fw_core::xcore::Kick;
use fw_flight::tagged;
use mavlink_device::dialect::{
    HIGHRES_IMU_DATA, HOME_POSITION_DATA, HighresImuUpdatedFlags, MavSeverity, MavState, MavSysStatusSensor,
    RC_CHANNELS_DATA, SCALED_IMU2_DATA, SCALED_PRESSURE_DATA, SYS_STATUS_DATA,
};
use mavlink_device::marv::MARV_DEVICE_STATUS_DATA;
use mavlink_device::{
    CommandOutcome, CommandRequest, Config as NodeConfig, FRAME_BUF, LateAck, MAX_BATCH, Node,
    Outgoing, ParamStore, RunError, Stream, StreamSlot, Telemetry, Texts,
};
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};
use usb_cdc::UsbCdc;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// The program's one `DMA_IRQ_0` binding, owned by the flight core's channels; `f4-negative` also binds core 0's channel 6.
#[cfg(not(feature = "f4-negative"))]
bind_interrupts!(pub struct DmaIrqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>, dma::InterruptHandler<DMA_CH3>,
        dma::InterruptHandler<DMA_CH5>;
});
#[cfg(feature = "f4-negative")]
bind_interrupts!(pub struct DmaIrqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>, dma::InterruptHandler<DMA_CH3>,
        dma::InterruptHandler<DMA_CH5>, dma::InterruptHandler<embassy_rp::peripherals::DMA_CH6>;
});

type UsbDriver = Driver<'static, USB>;

// Shared between the cores: plain atomics and `xcore` cells only.

static PING: AtomicU32 = AtomicU32::new(0);
/// Flight core -> core 0: the last `PING` the flight core copied (the watchdog monitor reads it).
pub static PONG: AtomicU32 = AtomicU32::new(0);
/// Core 0: the supervisor's clock (ms, wrapping) when it last saw `PONG` equal to `PING`.
static PONG_FRESH_MS: AtomicU32 = AtomicU32::new(0);
/// The HEARTBEAT `system_status` last sent, for the log line at each change (255: none yet).
static HEARTBEAT_STATE: AtomicU8 = AtomicU8::new(u8::MAX);
static KICK: Kick = Kick::new();
/// Core 1's ACTLR as it read it at the top of its entry; valid once `CORE1_ACTLR_READ` is true.
static CORE1_ACTLR: AtomicU32 = AtomicU32::new(0);
static CORE1_ACTLR_READ: AtomicBool = AtomicBool::new(false);
/// Core 1's `DMA_IRQ_0` after it built IMU1's and IMU2's SPI: 0 not read yet, 1 disabled, 2 enabled.
static CORE1_DMA_IRQ0: AtomicU8 = AtomicU8::new(0);
/// Core 1's `DMA_IRQ_1` (the SD card's line, core 0 only), read at the same point: 0 not read yet, 1 disabled, 2 enabled.
pub static CORE1_DMA_IRQ1: AtomicU8 = AtomicU8::new(0);
/// Core 1's `PIO2_IRQ_0` after it built the ESC driver: 0 not read yet, 1 disabled, 2 enabled, 3 the driver refused its parts.
static CORE1_PIO2_IRQ0: AtomicU8 = AtomicU8::new(0);
/// Core 1's VTOR as it read it back after pointing it at `CORE1_VECTOR_TABLE`; 0 until then.
static CORE1_VTOR: AtomicU32 = AtomicU32::new(0);

/// Core 1's own vector table in SRAM, so its exceptions never read flash; core 0 fills it before core 1 starts.
const CORE1_VECTORS: usize = 128;
#[repr(C, align(512))]
struct Core1Vectors(UnsafeCell<[u32; CORE1_VECTORS]>);
// SAFETY: written only by core 0 in `main` before `spawn_core1`, read-only afterwards.
unsafe impl Sync for Core1Vectors {}
static CORE1_VECTOR_TABLE: Core1Vectors = Core1Vectors(UnsafeCell::new([0; CORE1_VECTORS]));

const CORE1_STACK_BYTES: usize = 16 * 1024;
static CORE1_STACK: ConstStaticCell<Stack<CORE1_STACK_BYTES>> = ConstStaticCell::new(Stack::new());
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// Core 0 only.

pub static BEAT0: AtomicU32 = AtomicU32::new(0);
static TX_BYTES: AtomicU32 = AtomicU32::new(0);
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
/// The USB link as `usb_link` sees it: `LINK_DOWN`, `LINK_GATE` (up, start gate closed), `LINK_RUN`.
static LINK_STATE: AtomicU8 = AtomicU8::new(LINK_DOWN);
const LINK_DOWN: u8 = 0;
const LINK_GATE: u8 = 1;
const LINK_RUN: u8 = 2;
pub const LINK_USB: usize = 0;
pub const LINK_RADIO: usize = 1;
pub const LINK_LOG: usize = 2;
/// The STATUSTEXT ring: a cursor per link plus one for the log; boot texts sit in sticky slots until every cursor took them.
type Ring = TextRing<(MavSeverity, TextLine), 3, TEXTS_DEPTH>;
static TEXTS: Mutex<CriticalSectionRawMutex, RefCell<Ring>> = Mutex::new(RefCell::new(Ring::new()));
const TEXTS_DEPTH: usize = 8;
pub const ROWS: usize = DEVICE_COUNT + 2;

const KICK_PING: u8 = 1;
const EXTEXCLALL: u32 = 1 << 29;
const PONG_STALE: Duration = Duration::from_millis(200);
const SYS_STATUS_ID: u32 = 1;
const DEVICE_STATUS_ID: u32 = 42421;
const DEVICE_STATUS_INTERVAL_US: i32 = 1_000_000 / ROWS.div_ceil(MAX_BATCH) as i32;
/// The sensor values: HIGHRES_IMU (IMU1 as id 0), SCALED_IMU2 (IMU2) and SCALED_PRESSURE (BARO), 25 Hz each.
const HIGHRES_IMU_ID: u32 = 105;
const SCALED_IMU2_ID: u32 = 116;
/// Standard gravity, m/s^2 per g (SCALED_IMU2 carries milli-g).
const STANDARD_GRAVITY: f32 = 9.80665;
const SCALED_PRESSURE_ID: u32 = 29;
const VALUES_INTERVAL_US: i32 = 40_000;
const GPS_RAW_INT_ID: u32 = 24;
const GPS_INTERVAL_US: i32 = 1_000_000;
/// HOME_POSITION while home is set and the state is not STOP.
const HOME_POSITION_ID: u32 = 242;
const RC_CHANNELS_ID: u32 = 65;
const RC_CHANNELS_INTERVAL_US: i32 = 200_000;
/// MANUAL_CONTROL (the mapped sticks) at 5 Hz while CRSF stick reports are fresh (USB only).
const MANUAL_CONTROL_ID: u32 = 69;
const MANUAL_CONTROL_INTERVAL_US: i32 = 200_000;
/// ESC_STATUS at 10 Hz and ESC_INFO at 1 Hz while the flight core's ESC data is under 1 s old (USB only).
const ESC_STATUS_ID: u32 = 291;
const ESC_STATUS_INTERVAL_US: i32 = 100_000;
const ESC_INFO_ID: u32 = 290;
const ESC_INFO_INTERVAL_US: i32 = 1_000_000;
/// The receiver's value is not sent once its latest sample is older than this (3 fix periods).
const GPS_STALE_US: u64 = 3_000_000;
const STALE_US: u64 = 1_000_000;
const TEXT_UP: &str = concat!("MARV V2 fw ", env!("CARGO_PKG_VERSION"), ": flight core up");

#[cfg_attr(
    feature = "ram-negative",
    unsafe(link_section = ".text.ram_negative"),
    inline(never)
)]
fn now_us() -> u64 {
    #[cfg(feature = "psram-negative")]
    let _ = psram::negative_touch();
    Instant::now().as_micros()
}

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

// Core 1, the flight core.

#[cfg(not(feature = "no-fbeat"))]
#[embassy_executor::task]
async fn fbeat() -> ! {
    tagged(
        &watchdog::RUNNING1,
        task::FBEAT,
        fw_flight::liveness(&KICK, &PING, &PONG),
    )
    .await
}

fn core1_main(
    imu1_parts: imu1::Parts,
    imu2_parts: imu2::Parts,
    esc_parts: esc::Parts,
    esc_test: fw_core::xcore::TripleReader<'static, fw_core::esc::TestCmd>,
    flight_rc: Option<fw_core::xcore::TripleReader<'static, fw_core::rc::RcSample>>,
    steer: Option<&'static mut crate::flight::BoardMix>,
) -> ! {
    let table = CORE1_VECTOR_TABLE.0.get() as u32;
    // SAFETY: the executing core's own VTOR; the table is 512-aligned and core 0 filled it before this core started.
    let vtor = unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.vtor.write(table);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        scb.vtor.read()
    };
    CORE1_VTOR.store(vtor, Ordering::Release);
    CORE1_ACTLR.store(read_actlr(), Ordering::Relaxed);
    CORE1_ACTLR_READ.store(true, Ordering::Release);
    let (imu1_period_us, imu2_period_us) = (imu1_parts.setup.period_us, imu2_parts.setup.period_us);
    // Built here so that the DMA and GPIO interrupts are enabled on this core's NVIC.
    let imu1 = imu1::adapter(imu1_parts);
    let imu2 = imu2::adapter(imu2_parts);
    let esc_driver = esc::driver(esc_parts);
    let pio2_irq0 = if esc_driver.is_some() {
        1 + u8::from(interrupt::PIO2_IRQ_0.is_enabled())
    } else {
        3
    };
    CORE1_PIO2_IRQ0.store(pio2_irq0, Ordering::Release);
    CORE1_DMA_IRQ1.store(
        1 + u8::from(interrupt::DMA_IRQ_1.is_enabled()),
        Ordering::Release,
    );
    let dma_on = interrupt::DMA_IRQ_0.is_enabled();
    CORE1_DMA_IRQ0.store(1 + u8::from(dma_on), Ordering::Release);
    let executor = EXECUTOR1.init(Executor::new());
    executor.run(|spawner| {
        #[cfg(not(feature = "no-fbeat"))]
        if let Ok(token) = fbeat() {
            spawner.spawn(token);
        }
        if let Ok(token) = imu1::task(imu1, imu1_period_us, flight_rc, steer) {
            spawner.spawn(token);
        }
        if let Ok(token) = imu2::task(imu2, imu2_period_us) {
            spawner.spawn(token);
        }
        if let Some(Ok(token)) = esc_driver.map(|driver| esc::task(driver, esc_test)) {
            spawner.spawn(token);
        }
    })
}

// Core 0, the I/O core.

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

struct Board {
    cursor: usize,
    logs: logdl::Usb,
}

fn rows() -> [DeviceRow; ROWS] {
    let devices = DEVICES.map(Device::row);
    core::array::from_fn(|i| match devices.get(i) {
        Some(row) => *row,
        None if i == DEVICE_COUNT => rc::row(),
        None => esc::row(),
    })
}

pub fn device_message(i: usize, now_us: u64) -> Option<MARV_DEVICE_STATUS_DATA> {
    match DEVICES.get(i) {
        Some(dev) => Some(dev.message(now_us)),
        None if i == DEVICE_COUNT => Some(rc::message(now_us)),
        None => (i == DEVICE_COUNT + 1).then(|| esc::message(now_us)),
    }
}

pub fn push_gps_raw_int(now_us: u64, out: &mut Outgoing<'_>) -> bool {
    let Some(sample) = GNSS.latest.try_get() else {
        return false;
    };
    if now_us.saturating_sub(sample.t_us) >= GPS_STALE_US {
        return false;
    }
    gnss::raw_int(&sample.reading, sample.t_us).is_some_and(|message| out.push(&message).is_ok())
}

pub fn push_home_position(out: &mut Outgoing<'_>) -> bool {
    let Some((home, origin)) = station::home() else {
        return false;
    };
    let [x, y] = home.ne_m;
    let p = fw_flight::eskf::geo_from_ned(&origin, [x, y, home.d_m]);
    out.push(&HOME_POSITION_DATA {
        latitude: p.lat_e7,
        longitude: p.lon_e7,
        altitude: p.alt_mm,
        x,
        y,
        z: home.d_m,
        q: [1.0, 0.0, 0.0, 0.0],
        approach_x: 0.0,
        approach_y: 0.0,
        approach_z: 0.0,
        time_usec: 0,
    })
    .is_ok()
}

pub fn heartbeat_state(now_us: u64) -> MavState {
    system_status(&rows(), now_us / 1000, calibration::calibrating())
}

pub fn push_sys_status(now_us: u64, out: &mut Outgoing<'_>) -> bool {
    // The latest BATT sample while under 1 s old, else MAVLink's "not sent" values (millivolts, centiamps).
    let (voltage_battery, current_battery) = match BATT.latest.try_get() {
        Some(Sample {
            reading:
                Reading::Batt {
                    voltage_v,
                    current_a,
                },
            t_us,
            ..
        }) if now_us.saturating_sub(t_us) < STALE_US => (
            (voltage_v * 1000.0).clamp(0.0, 65_534.0) as u16,
            (current_a * 100.0).clamp(-32_768.0, 32_767.0) as i16,
        ),
        _ => (u16::MAX, -1),
    };
    let masks = sensor_masks(&rows());
    out.push(&SYS_STATUS_DATA {
        onboard_control_sensors_present: MavSysStatusSensor::from_bits_truncate(masks.present),
        onboard_control_sensors_enabled: MavSysStatusSensor::from_bits_truncate(masks.enabled),
        onboard_control_sensors_health: MavSysStatusSensor::from_bits_truncate(masks.health),
        voltage_battery,
        current_battery,
        battery_remaining: -1,
        ..SYS_STATUS_DATA::DEFAULT
    })
    .is_ok()
}

pub fn push_highres_imu(now_us: u64, out: &mut Outgoing<'_>) -> bool {
    let Some(sample) = IMU1.latest.try_get() else {
        return false;
    };
    let Reading::Imu {
        gyro_rads: [xgyro, ygyro, zgyro],
        accel_ms2: [xacc, yacc, zacc],
        temperature_c,
    } = sample.reading
    else {
        return false;
    };
    // A field is shown as updated only while its latest sample is younger than 1 s.
    let fresh = |t_us: u64| now_us.saturating_sub(t_us) < STALE_US;
    let (imu_values, imu_flags) = if fresh(sample.t_us) {
        (
            [xacc, yacc, zacc, xgyro, ygyro, zgyro, temperature_c],
            HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XGYRO
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YGYRO
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZGYRO
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_TEMPERATURE,
        )
    } else {
        ([f32::NAN; 7], HighresImuUpdatedFlags::empty())
    };
    let [xacc, yacc, zacc, xgyro, ygyro, zgyro, temperature_c] = imu_values;
    // The magnetometer's latest sample, in gauss (1 G = 100 uT).
    let (mag, mag_flags) = match MAG.latest.try_get() {
        Some(Sample {
            reading:
                Reading::Mag {
                    reading:
                        MagReading {
                            field_ut: [x, y, z],
                            ..
                        },
                    ..
                },
            t_us,
            ..
        }) if fresh(t_us) => (
            [x / 100.0, y / 100.0, z / 100.0],
            HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XMAG
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YMAG
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZMAG,
        ),
        _ => ([f32::NAN; 3], HighresImuUpdatedFlags::empty()),
    };
    out.push(&HIGHRES_IMU_DATA {
        time_usec: sample.t_us,
        xacc,
        yacc,
        zacc,
        xgyro,
        ygyro,
        zgyro,
        xmag: mag[0],
        ymag: mag[1],
        zmag: mag[2],
        abs_pressure: f32::NAN,
        diff_pressure: f32::NAN,
        pressure_alt: f32::NAN,
        temperature: temperature_c,
        fields_updated: imu_flags | mag_flags,
        id: 0,
    })
    .is_ok()
}

pub fn push_scaled_imu2(now_us: u64, out: &mut Outgoing<'_>) -> bool {
    let Some(sample) = IMU2.latest.try_get() else {
        return false;
    };
    let Reading::Imu {
        gyro_rads: [xgyro, ygyro, zgyro],
        accel_ms2: [xacc, yacc, zacc],
        temperature_c,
    } = sample.reading
    else {
        return false;
    };
    // No fresh sample: no message (integer fields have no "unknown" value).
    if now_us.saturating_sub(sample.t_us) >= STALE_US {
        return false;
    }
    let mg = |a: f32| (a * (1000.0 / STANDARD_GRAVITY)) as i16;
    let mrads = |w: f32| (w * 1000.0) as i16;
    // 0 means "no temperature" in SCALED_IMU2: 0.00 C is sent as 0.01 C.
    let temperature = match (temperature_c * 100.0) as i16 {
        0 => 1,
        t => t,
    };
    // The LSM6DSK320X has no magnetometer: those fields are 0.
    out.push(&SCALED_IMU2_DATA {
        time_boot_ms: u32::try_from(sample.t_us / 1000).unwrap_or(u32::MAX),
        xacc: mg(xacc),
        yacc: mg(yacc),
        zacc: mg(zacc),
        xgyro: mrads(xgyro),
        ygyro: mrads(ygyro),
        zgyro: mrads(zgyro),
        xmag: 0,
        ymag: 0,
        zmag: 0,
        temperature,
    })
    .is_ok()
}

pub fn push_scaled_pressure(now_us: u64, out: &mut Outgoing<'_>) -> bool {
    let Some(sample) = BARO.latest.try_get() else {
        return false;
    };
    let Reading::Baro(BaroReading {
        pressure_pa,
        temperature_c,
    }) = sample.reading
    else {
        return false;
    };
    if now_us.saturating_sub(sample.t_us) >= STALE_US {
        return false;
    }
    out.push(&SCALED_PRESSURE_DATA {
        time_boot_ms: u32::try_from(sample.t_us / 1000).unwrap_or(u32::MAX),
        press_abs: pressure_pa / 100.0,
        press_diff: 0.0,
        temperature: (temperature_c * 100.0) as i16,
        temperature_press_diff: 0,
    })
    .is_ok()
}

impl Telemetry for Board {
    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool {
        if message_id == HIGHRES_IMU_ID {
            return push_highres_imu(now_us, out);
        }
        if message_id == SCALED_IMU2_ID {
            return push_scaled_imu2(now_us, out);
        }
        if message_id == SCALED_PRESSURE_ID {
            return push_scaled_pressure(now_us, out);
        }
        if message_id == GPS_RAW_INT_ID {
            return push_gps_raw_int(now_us, out);
        }
        if message_id == HOME_POSITION_ID {
            return push_home_position(out);
        }
        if message_id == RC_CHANNELS_ID {
            return rc::rc_channels(now_us).is_some_and(|m: RC_CHANNELS_DATA| out.push(&m).is_ok());
        }
        if message_id == MANUAL_CONTROL_ID {
            return rc::manual_control(now_us).is_some_and(|m| out.push(&m).is_ok());
        }
        if message_id == ESC_STATUS_ID {
            return esc::esc_status(now_us).is_some_and(|m| out.push(&m).is_ok());
        }
        if message_id == ESC_INFO_ID {
            return esc::esc_info(now_us).is_some_and(|m| out.push(&m).is_ok());
        }
        if message_id == logdl::LOG_DATA_ID {
            return self.logs.sample(out);
        }
        if message_id == mission::SLOT_ID {
            return mission::sample(fw_core::mission::Link::Usb, now_us, out);
        }
        if message_id == DEVICE_STATUS_ID {
            let mut sent = false;
            for _ in 0..ROWS.min(MAX_BATCH) {
                if self.cursor >= ROWS {
                    self.cursor = 0;
                }
                let Some(message) = device_message(self.cursor, now_us) else {
                    break;
                };
                self.cursor += 1;
                sent |= out.push(&message).is_ok();
            }
            return sent;
        }
        if message_id != SYS_STATUS_ID {
            return false;
        }
        push_sys_status(now_us, out)
    }

    fn status(&mut self, now_us: u64) -> Option<MavState> {
        let state = heartbeat_state(now_us);
        if HEARTBEAT_STATE.swap(state as u8, Ordering::Relaxed) != state as u8 {
            defmt::info!(
                "[INFO] HEARTBEAT system_status {=str}",
                match state {
                    MavState::MAV_STATE_CALIBRATING => "CALIBRATING",
                    MavState::MAV_STATE_STANDBY => "STANDBY",
                    _ => "other",
                }
            );
        }
        Some(state)
    }

    fn texts(&mut self, _now_us: u64, out: &mut Texts<'_>) {
        if let Some((severity, text)) = take_text(LINK_USB) {
            defmt::info!("[INFO] text to the link: {=str}", text.as_str());
            let _ = out.push(severity, text.as_str());
        }
    }

    fn command(&mut self, req: &CommandRequest, now_us: u64) -> CommandOutcome {
        actions::command(req, LINK_USB as u8, now_us / 1000)
    }

    fn command_deferrable(&mut self, req: &CommandRequest, now_us: u64) -> Option<CommandOutcome> {
        actions::command_deferrable(req, LINK_USB as u8, now_us / 1000)
    }

    fn custom_mode(&mut self, _now_us: u64) -> u32 {
        station::custom_mode()
    }

    fn acks(&mut self, now_us: u64) -> Option<LateAck> {
        actions::next_ack(LINK_USB as u8, now_us / 1000)
    }
}

fn texts_pending(link: usize) -> bool {
    TEXTS.lock(|r| r.borrow().pending(link))
}

fn take_text(link: usize) -> Option<(MavSeverity, TextLine)> {
    TEXTS.lock(|r| r.borrow_mut().pop(link))
}

/// A boot text kept until every link took it; an ordinary text when both sticky slots are still owed.
fn text_sticky(severity: MavSeverity, text: &str) {
    let line = TextLine::new(format_args!("{}", text));
    if !TEXTS.lock(|r| r.borrow_mut().push_sticky((severity, line))) {
        text_fmt(severity, format_args!("{}", text));
    }
}

fn text(severity: MavSeverity, text: &str) {
    text_fmt(severity, format_args!("{}", text));
}

fn text_fmt(severity: MavSeverity, args: fmt::Arguments<'_>) {
    let line = TextLine::new(args);
    let losers = TEXTS.lock(|r| r.borrow_mut().push((severity, line)));
    if losers > 0 {
        defmt::warn!(
            "[INFO] text ring: the oldest unread text dropped for {=usize} link(s)",
            losers
        );
    }
}

/// The start gate of `docs/link-contract.md`: nothing is written until a ground-station HEARTBEAT has arrived
/// or DTR has read true at every check for 100 ms. Returns false when a read ends (link down).
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
    tagged(&watchdog::RUNNING0, task::USB, device.run()).await
}

#[embassy_executor::task]
async fn usb_link(
    tx: usb_cdc::Tx<'static, UsbDriver>,
    rx: usb_cdc::Rx<'static, UsbDriver>,
    link: usb_cdc::Link<'static>,
) -> ! {
    tagged(&watchdog::RUNNING0, task::LINK, link_loop(tx, rx, link)).await
}

async fn link_loop(
    tx: usb_cdc::Tx<'static, UsbDriver>,
    rx: usb_cdc::Rx<'static, UsbDriver>,
    link: usb_cdc::Link<'static>,
) -> ! {
    let (mut tx, mut rx) = (Counted(tx), Counted(rx));
    let mut slots = [
        StreamSlot::new(Stream {
            message_id: SYS_STATUS_ID,
            default_interval_us: 1_000_000,
            min_interval_us: 100_000,
        }),
        StreamSlot::new(Stream {
            message_id: DEVICE_STATUS_ID,
            default_interval_us: DEVICE_STATUS_INTERVAL_US,
            min_interval_us: 100_000,
        }),
        StreamSlot::new(Stream {
            message_id: HIGHRES_IMU_ID,
            default_interval_us: VALUES_INTERVAL_US,
            min_interval_us: 5_000,
        }),
        StreamSlot::new(Stream {
            message_id: SCALED_IMU2_ID,
            default_interval_us: VALUES_INTERVAL_US,
            min_interval_us: 5_000,
        }),
        StreamSlot::new(Stream {
            message_id: SCALED_PRESSURE_ID,
            default_interval_us: VALUES_INTERVAL_US,
            min_interval_us: 20_000,
        }),
        StreamSlot::new(Stream {
            message_id: GPS_RAW_INT_ID,
            default_interval_us: GPS_INTERVAL_US,
            min_interval_us: 100_000,
        }),
        StreamSlot::new(Stream {
            message_id: HOME_POSITION_ID,
            default_interval_us: GPS_INTERVAL_US,
            min_interval_us: 100_000,
        }),
        StreamSlot::new(Stream {
            message_id: RC_CHANNELS_ID,
            default_interval_us: RC_CHANNELS_INTERVAL_US,
            min_interval_us: 20_000,
        }),
        StreamSlot::new(Stream {
            message_id: MANUAL_CONTROL_ID,
            default_interval_us: MANUAL_CONTROL_INTERVAL_US,
            min_interval_us: 20_000,
        }),
        StreamSlot::new(Stream {
            message_id: ESC_STATUS_ID,
            default_interval_us: ESC_STATUS_INTERVAL_US,
            min_interval_us: 20_000,
        }),
        StreamSlot::new(Stream {
            message_id: ESC_INFO_ID,
            default_interval_us: ESC_INFO_INTERVAL_US,
            min_interval_us: 100_000,
        }),
        StreamSlot::new(Stream {
            message_id: logdl::LOG_DATA_ID,
            default_interval_us: logdl::INTERVAL_US,
            min_interval_us: logdl::INTERVAL_US as u32,
        }),
        // The mission service's reply slot (`mission::sample`).
        StreamSlot::new(Stream {
            message_id: mission::SLOT_ID,
            default_interval_us: mission::USB_INTERVAL_US,
            min_interval_us: mission::USB_INTERVAL_US as u32,
        }),
    ];
    let mut node = Node::new(USB_NODE, &mut slots);
    let mut params = store::Store;
    let mut board = Board {
        cursor: 0,
        logs: logdl::Usb::new(1, 1),
    };
    loop {
        LINK_STATE.store(LINK_DOWN, Ordering::Relaxed);
        link.wait_up().await;
        LINK_STATE.store(LINK_GATE, Ordering::Relaxed);
        if !start_gate(&mut rx, &link, &mut node, &mut params).await {
            node.on_link_down();
            actions::link_lost(LINK_USB as u8);
            mission::link_down(fw_core::mission::Link::Usb);
            continue;
        }
        LINK_STATE.store(LINK_RUN, Ordering::Relaxed);
        let end = drive(&mut node, &mut rx, &mut tx, &mut params, &mut board).await;
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
        board.logs.stop();
        actions::link_lost(LINK_USB as u8);
        mission::link_down(fw_core::mission::Link::Usb);
    }
}

/// The USB node's accept list: the log-download requests, then the six mission messages.
const USB_ACCEPT: [(u32, u8); logdl::ACCEPT.len() + mission::ACCEPT.len()] = {
    let mut list = [(0, 0); logdl::ACCEPT.len() + mission::ACCEPT.len()];
    let mut i = 0;
    while i < logdl::ACCEPT.len() {
        list[i] = logdl::ACCEPT[i];
        i += 1;
    }
    let mut j = 0;
    while j < mission::ACCEPT.len() {
        list[logdl::ACCEPT.len() + j] = mission::ACCEPT[j];
        j += 1;
    }
    list
};

/// The USB node: system 1, component 1; the station counts as present for 3 s after its HEARTBEAT.
const USB_NODE: NodeConfig = match NodeConfig::new(1, 1)
    .with_gcs_timeout_us(logdl::GCS_TIMEOUT_US)
    .with_accept_list(&USB_ACCEPT)
{
    Ok(config) => config,
    Err(_) => panic!("accept list too long"),
};
const READ_BUF: usize = 64;
const MAX_WAIT_US: u64 = 100_000;

/// The USB node's driver, mirroring mavlink-device's `Node::run` with two differences: received bytes go to
/// `receive_with` (log-download requests to `board.logs`), and a log transfer stops when the station's HEARTBEAT is
/// older than `logdl::GCS_TIMEOUT_US`.
async fn drive<R: Read, W: Write>(
    node: &mut Node<'_>,
    rx: &mut R,
    tx: &mut W,
    params: &mut store::Store,
    board: &mut Board,
) -> RunError<R::Error, W::Error> {
    let mut frame = [0u8; FRAME_BUF];
    let mut buf = [0u8; READ_BUF];
    loop {
        while let Some(n) = node.poll_transmit(now_us(), &*params, board, &mut frame) {
            let Some(bytes) = frame.get(..n) else {
                continue;
            };
            if let Err(e) = tx.write_all(bytes).await {
                return RunError::Write(e);
            }
        }
        if !node.gcs_present(now_us()) {
            board.logs.stop();
        }
        let wait_us = (node.next_deadline_us().saturating_sub(now_us())).clamp(1, MAX_WAIT_US);
        let wait_us = u32::try_from(wait_us).unwrap_or(u32::MAX);
        match select(rx.read(&mut buf), Timer::after_micros(u64::from(wait_us))).await {
            Either::Second(()) => {}
            Either::First(Err(e)) => return RunError::Read(e),
            Either::First(Ok(0)) => return RunError::EndOfStream,
            Either::First(Ok(n)) => {
                if let Some(bytes) = buf.get(..n) {
                    node.receive_with(bytes, now_us(), params, &mut |m| {
                        if !mission::message(fw_core::mission::Link::Usb, m) {
                            board.logs.message(m);
                        }
                    });
                }
            }
        }
    }
}

/// Every 10 ms: PONG age, a 10 Hz `PING` and flight-core kick, the start-up checks, liveness and device texts; every 10 s a `[STAT]` line.
#[embassy_executor::task]
async fn supervisor() -> ! {
    tagged(&watchdog::RUNNING0, task::SUPERVISOR, supervise()).await
}

async fn supervise() -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut tick: u32 = 0;
    let mut fresh_at = Instant::now();
    let mut max_age_us: u64 = 0;
    let mut first_ping_at: Option<Instant> = None;
    let mut checked = false;
    let mut up_sent = false;
    let mut answering = true;
    let mut devices = Supervision::new(Instant::now());
    let mut battery = batt_fs::Watch::new();
    let (mut stalls_begun, mut stalls_ended) = (0, 0);
    loop {
        ticker.next().await;
        let now = Instant::now();
        watchdog::report(&mut stalls_begun, &mut stalls_ended);
        devices.tick(now);
        actions::tick(now.as_millis());
        actions::permit(now.as_millis(), IMU1.row().state, esc::row().state);
        esc::report();
        let ping = PING.load(Ordering::Relaxed);
        let pong = PONG.load(Ordering::Acquire);
        if pong == ping {
            fresh_at = now;
            PONG_FRESH_MS.store(now.as_millis() as u32, Ordering::Release);
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
                text_fmt(
                    MavSeverity::MAV_SEVERITY_INFO,
                    format_args!("{}, reset: {}", TEXT_UP, watchdog::cause()),
                );
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
            battery.tick(now.as_micros());
            PING.store(ping.wrapping_add(1), Ordering::Release);
            KICK.kick(KICK_PING);
            first_ping_at.get_or_insert(now);
        }
        if tick.is_multiple_of(1000) && tick > 0 {
            psram::periodic();
            if !cfg!(feature = "quiet-stat") {
                defmt::info!(
                    "[STAT] uptime {=u64} s, PONG age max {=u64} us, PING {=u32} PONG {=u32}, usb wrote {=u32} B {=u32} frames, read {=u32} B, link {=str}; log download: {=u32} transfers, {=u32} B sent, SD step max {=u32} us, piece or flush late max {=u32} us; psram {=str}, canary checks {=u32} failed {=u32}, remaps {=u32}, context violations {=u32}",
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
                    },
                    logdl::TRANSFERS.load(Ordering::Relaxed),
                    logdl::BYTES_SENT.load(Ordering::Relaxed),
                    logdl::STEP_MAX_US.load(Ordering::Relaxed),
                    logdl::LATE_MAX_US.load(Ordering::Relaxed),
                    psram::status_text(),
                    psram::checks(),
                    psram::check_fails(),
                    psram::remaps(),
                    psram::violations()
                );
            }
            devices.stat(now);
            rc::stat(now);
            esc::stat(now);
            flight::stat();
            calibration::stat();
            watchdog::stat();
        }
        // The mission's change counter moved (a commit or a clear): the list goes to the flight core again.
        if mission::changed() {
            flight::push_gdn("mission changed");
        }
        tick = tick.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn beat0() -> ! {
    tagged(&watchdog::RUNNING0, task::BEAT, async {
        let mut ticker = Ticker::every(Duration::from_millis(50));
        loop {
            ticker.next().await;
            BEAT0.fetch_add(1, Ordering::Relaxed);
        }
    })
    .await
}

/// Feature `stall-core0` (invariant F1): once, 20 s after start, spins 1.2 s without yielding; skipped after a watchdog reset.
#[cfg(feature = "stall-core0")]
#[embassy_executor::task]
async fn stall() {
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    Timer::after(Duration::from_secs(20)).await;
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    if !watchdog::HANG_ARMED.swap(false, Ordering::Relaxed) {
        return;
    }
    let imus = [("IMU1", &imu1::CELLS), ("IMU2", &imu2::CELLS)];
    let before = imus.map(|(_, cells)| cells.read());
    let begun = Instant::now();
    while begun.elapsed() < Duration::from_millis(1200) {
        core::hint::spin_loop();
    }
    let ended = Instant::now();
    let after = imus.map(|(_, cells)| cells.read());
    for (((name, _), before), after) in imus.iter().zip(before).zip(after) {
        let (s0, e0) = before.map_or((0, 0), |s| (s.samples, s.health.errors));
        let (s1, e1) = after.map_or((0, 0), |s| (s.samples, s.health.errors));
        defmt::info!(
            "[INFO] stall-core0: spun from {=u64} to {=u64} ms; {=str} samples {=u32} -> {=u32} (+{=u32}), errors {=u32} -> {=u32}",
            begun.as_millis(),
            ended.as_millis(),
            name,
            s0,
            s1,
            s1.wrapping_sub(s0),
            e0,
            e1
        );
    }
}

/// Feature `stack-negative`: once, 10 s after a start the watchdog did not cause, recurses until the stack pointer is 2 KiB
/// below `_stack_end`. With the guard the first push below it faults; without it the task logs `[FAIL]`.
#[cfg(feature = "stack-negative")]
#[inline(never)]
fn stack_dive(limit: u32, depth: u32) -> u32 {
    let mut frame = [0u32; 64];
    for (i, w) in frame.iter_mut().enumerate() {
        // SAFETY: a write to a local.
        unsafe { core::ptr::write_volatile(w, 0x5AC0_0000 | i as u32) };
    }
    let below = if cortex_m::register::msp::read() > limit {
        stack_dive(limit, depth + 1)
    } else {
        depth
    };
    core::hint::black_box(&frame);
    below
}

#[cfg(feature = "stack-negative")]
#[embassy_executor::task]
async fn stack_negative() {
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    Timer::after(Duration::from_secs(10)).await;
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    if !watchdog::HANG_ARMED.swap(false, Ordering::Relaxed) {
        return;
    }
    unsafe extern "C" {
        static _stack_end: u32;
    }
    let end = core::ptr::addr_of!(_stack_end) as u32;
    let probe = (end - 1024) as *const u32;
    // SAFETY: a word of the statics just below the stack, read and never written here.
    let before = unsafe { core::ptr::read_volatile(probe) };
    defmt::info!(
        "[INFO] stack-negative: _stack_end {=u32:#010x}, MSP {=u32:#010x}, MSPLIM {=u32:#010x}; diving to {=u32:#010x}",
        end,
        cortex_m::register::msp::read(),
        cortex_m::register::msplim::read(),
        end - 2048
    );
    let depth = stack_dive(end - 2048, 0);
    // SAFETY: as above.
    let after = unsafe { core::ptr::read_volatile(probe) };
    defmt::error!(
        "[FAIL] stack-negative: recursed {=u32} frames to 2 KiB below _stack_end and no fault; word at _stack_end-1024 {=u32:#010x} -> {=u32:#010x}",
        depth,
        before,
        after
    );
}

/// The two IMUs' setups from the stored settings, read once at boot and handed to core 1 by value; a change takes
/// effect after the next reboot. The board orientation is read here once, for both setups and `calibration::init`.
fn imu_setups() -> (
    fw_settings::imu::Imu1Setup,
    fw_settings::imu::Imu2Setup,
    fw_core::rotation::Rotation,
) {
    let board = store::board();
    defmt::info!(
        "[INFO] AHRS_ORIENTATION {=i32}, MAG_ORIENT {=i32} (until the next reboot for the first)",
        board.as_i32(),
        store::mag_frame().0.as_i32()
    );
    for n in 0..2usize {
        let (g, dps, hz) = store::imu_values(n).unwrap_or((0, 0, 0));
        defmt::info!(
            "[INFO] IMU{=usize}: {=i32} g, {=i32} dps, {=i32} Hz (until the next reboot)",
            n + 1,
            g,
            dps,
            hz
        );
    }
    let (Ok(setup1), Ok(setup2)) = (store::imu1_setup(), store::imu2_setup()) else {
        defmt::panic!("[FAIL] IMU setup refused after the boot reset");
    };
    device::IMU1.set_period_us(setup1.period_us);
    device::IMU2.set_period_us(setup2.period_us);
    (setup1.with_board(board), setup2.with_board(board), board)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // MSPLIM := `_stack_end` (memory.x): core 0's stack overflow faults instead of overwriting statics.
    // Core 1's guard is set by `spawn_core1` (embassy-rp `core1_setup`).
    defmt::unwrap!(embassy_rp::install_core0_stack_guard());
    watchdog::paint_core0();
    let p = embassy_rp::init(Default::default());
    watchdog::RUNNING0.store(task::MAIN, Ordering::Relaxed);
    let mut wd = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    watchdog::boot(&mut wd);
    #[cfg(not(feature = "no-fbeat"))]
    {
        wd.pause_on_debug(true);
        wd.start(Duration::from_millis(u64::from(
            fw_core::watchdog::PERIOD_MS,
        )));
        defmt::info!(
            "[INFO] watchdog started: {=u32} ms, paused while a core is halted by the debugger; the monitor on SWI_IRQ_1 feeds it while BEAT0 and PONG are younger than {=u32} ms",
            fw_core::watchdog::PERIOD_MS,
            fw_core::watchdog::STALL_MS
        );
    }
    #[cfg(feature = "no-fbeat")]
    defmt::info!(
        "[INFO] watchdog NOT started (no-fbeat: the flight core never answers, so it would reset the board in a loop); the monitor runs without effect and still carries out a reboot"
    );
    defmt::info!(
        "[INFO] MARV V2 fw {=str}: two cores",
        env!("CARGO_PKG_VERSION")
    );
    defmt::info!("[INFO] icm45686: {}", type_name::<icm45686::Info>());
    defmt::info!("[INFO] lsm6dsk320x: {}", type_name::<lsm6dsk320x::Info>());
    defmt::info!("[INFO] bmp581: {}", type_name::<bmp581::Info>());
    defmt::info!("[INFO] bmm350: {}", type_name::<bmm350::Info>());
    defmt::info!("[INFO] sam-m10q: {}", type_name::<sam_m10q::Config>());
    defmt::info!("[INFO] pio-sdmmc: {}", type_name::<pio_sdmmc::Config>());

    let actlr0 = read_actlr();
    check(
        actlr0 & EXTEXCLALL != 0,
        "EXTEXCLALL set on core 0, ACTLR",
        actlr0,
    );

    // F4: all DMA channels share DMA_IRQ_0, which belongs to the flight core.
    interrupt::DMA_IRQ_0.disable();
    #[cfg(feature = "f4-negative")]
    let _channel = embassy_rp::dma::Channel::new(p.DMA_CH6, DmaIrqs);
    let dma_irq0_on = interrupt::DMA_IRQ_0.is_enabled();
    check(
        !dma_irq0_on,
        "DMA_IRQ_0 not enabled on core 0, enabled",
        u32::from(dma_irq0_on),
    );
    #[cfg(feature = "f4-negative")]
    interrupt::DMA_IRQ_0.disable();

    store::init();
    settings::init(p.FLASH);
    #[cfg(feature = "guard-negative")]
    settings::guard_negative();
    #[cfg(feature = "corrupt-newest")]
    settings::corrupt_newest();
    settings::boot();
    let (imu1_setup, imu2_setup, board) = imu_setups();
    calibration::init(board);
    flight::init_speed();
    flight::init_notch();
    flight::init_gdn();
    flight::init_gains();
    let [imu1_offsets, imu2_offsets] = calibration::readers();
    #[cfg(feature = "fake-armed")]
    settings::fake_arm();
    // The PSRAM comes after every boot-time flash write and before `spawn_core1`: the driver talks to the chip while
    // nothing else may run from flash. A missing or failing chip is a logged FAULT, not a stop.
    psram::init(p.QMI_CS1, p.PIN_0);
    let imu1_parts = imu1::Parts {
        spi: p.SPI0,
        sck: p.PIN_18,
        mosi: p.PIN_19,
        miso: p.PIN_16,
        cs: p.PIN_20,
        int1: p.PIN_15,
        tx_dma: p.DMA_CH0,
        rx_dma: p.DMA_CH1,
        offsets: imu1_offsets,
        setup: imu1_setup,
    };
    let imu2_parts = imu2::Parts {
        spi: p.SPI1,
        sck: p.PIN_10,
        mosi: p.PIN_11,
        miso: p.PIN_8,
        cs: p.PIN_9,
        int1: p.PIN_7,
        tx_dma: p.DMA_CH2,
        rx_dma: p.DMA_CH3,
        offsets: imu2_offsets,
        setup: imu2_setup,
    };
    let esc_parts = esc::Parts {
        pio: p.PIO2,
        pin36: p.PIN_36,
        pin37: p.PIN_37,
        pin38: p.PIN_38,
        pin39: p.PIN_39,
        dma: p.DMA_CH5,
        clk_sys_hz: embassy_rp::clocks::clk_sys_freq(),
    };
    let esc_test = esc::test_reader();
    actions::init_motor_test();
    let flight_rc = rc::FLIGHT_RC.take_reader();
    // The steering estimator's parts are built here, on core 0's large stack, into their statics, one call each.
    let steer_core = flight::steer_core();
    let vert = flight::vert();
    let steer = flight::steer(steer_core, vert);
    unsafe extern "C" {
        static __vector_table: u32;
        static __vector_table_end: u32;
    }
    let vectors = &raw const __vector_table;
    let words = (&raw const __vector_table_end as usize - vectors as usize) / 4;
    defmt::assert!(words <= CORE1_VECTORS);
    // SAFETY: `words` words of core 0's vector table in flash (bounds from the linker script) into
    // core 1's table, which nothing else touches before core 1 starts below.
    unsafe {
        core::ptr::copy_nonoverlapping(vectors, CORE1_VECTOR_TABLE.0.get().cast::<u32>(), words);
    }
    let stack1 = CORE1_STACK.take();
    watchdog::paint_core1(&mut stack1.mem);
    spawn_core1(p.CORE1, stack1, move || {
        core1_main(
            imu1_parts,
            imu2_parts,
            esc_parts,
            esc_test,
            flight_rc,
            steer,
        )
    });

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
    spawner.spawn(defmt::unwrap!(led::task(p.PIO1, p.PIN_43)));
    spawner.spawn(defmt::unwrap!(baro::task(baro::adapter(
        p.I2C1, p.PIN_23, p.PIN_22, p.PIN_21
    ))));
    spawner.spawn(defmt::unwrap!(mag::task(mag::adapter(
        p.I2C0, p.PIN_13, p.PIN_12, p.PIN_4
    ))));
    spawner.spawn(defmt::unwrap!(gnss::task(gnss::adapter(
        p.UART1, p.PIN_24, p.PIN_25
    ))));
    spawner.spawn(defmt::unwrap!(gnss::pps_task(p.PIN_5)));
    spawner.spawn(defmt::unwrap!(batt::task(batt::adapter(
        p.ADC, p.PIN_41, p.PIN_42
    ))));
    rc::start(p.UART0, p.PIN_14, p.PIN_17);
    watchdog::start_monitor(wd);
    #[cfg(feature = "hang-flight")]
    spawner.spawn(defmt::unwrap!(watchdog::hang_flight_arm()));
    #[cfg(feature = "stall-core0")]
    spawner.spawn(defmt::unwrap!(stall()));
    #[cfg(feature = "stack-negative")]
    spawner.spawn(defmt::unwrap!(stack_negative()));
    #[cfg(feature = "save-loop")]
    spawner.spawn(defmt::unwrap!(settings::save_loop()));
    #[cfg(feature = "xip-park")]
    spawner.spawn(defmt::unwrap!(xip_park::task()));

    tagged(&watchdog::RUNNING0, task::MAIN, startup_checks()).await;
    spawner.spawn(defmt::unwrap!(logger::task()));
    spawner.spawn(defmt::unwrap!(sd::task(sd::Parts {
        pio: p.PIO0,
        dma: p.DMA_CH4,
        clk: p.PIN_32,
        cmd: p.PIN_33,
        dat0: p.PIN_28,
        dat1: p.PIN_29,
        dat2: p.PIN_30,
        dat3: p.PIN_31,
    })));
}

/// The start-up checks that wait for core 1, and the watchdog's first feeds.
async fn startup_checks() {
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
    let vtor1 = CORE1_VTOR.load(Ordering::Acquire);
    check(
        vtor1 == CORE1_VECTOR_TABLE.0.get() as u32 && (0x2000_0000..0x2008_0000).contains(&vtor1),
        "core 1 VTOR is its RAM vector table",
        vtor1,
    );
    let started = Instant::now();
    while CORE1_DMA_IRQ0.load(Ordering::Acquire) == 0 && started.elapsed() < PONG_STALE {
        Timer::after(Duration::from_millis(1)).await;
    }
    let dma1 = CORE1_DMA_IRQ0.load(Ordering::Acquire);
    check(
        dma1 == 2,
        "DMA_IRQ_0 enabled on core 1 after IMU1's and IMU2's SPI were built there (0 not read, 1 off, 2 on)",
        u32::from(dma1),
    );
    let dma0_on = interrupt::DMA_IRQ_0.is_enabled();
    check(
        !dma0_on,
        "DMA_IRQ_0 still not enabled on core 0 after core 1 started, enabled",
        u32::from(dma0_on),
    );
    let pio2_1 = CORE1_PIO2_IRQ0.load(Ordering::Acquire);
    check(
        pio2_1 == 2,
        "PIO2_IRQ_0 enabled on core 1 after the ESC driver was built there (0 not read, 1 off, 2 on, 3 driver refused its parts)",
        u32::from(pio2_1),
    );
    let pio2_0 = interrupt::PIO2_IRQ_0.is_enabled();
    check(
        !pio2_0,
        "PIO2_IRQ_0 not enabled on core 0, enabled",
        u32::from(pio2_0),
    );
    #[cfg(not(feature = "no-fbeat"))]
    {
        Timer::after(Duration::from_millis(500)).await;
        let feeds = watchdog::FEEDS.load(Ordering::Relaxed);
        check(
            feeds >= 3,
            "watchdog fed by its monitor (feeds by 500 ms after the checks above)",
            feeds,
        );
    }
}
