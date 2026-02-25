// Application wiring and runtime state.
#[path = "core1.rs"]
mod core1;
#[path = "dma-tasks.rs"]
mod dma_tasks;
#[path = "hardware/mod.rs"]
mod hardware;
#[path = "init.rs"]
mod init;
#[path = "telemetry-presets.rs"]
mod telemetry_presets;

use defmt::*;
use hardware::pinout::fc;
use hardware::{
    RpUsbDevice, UsbCdc, build_usb_cdc, load_config_and_params_from_sd, save_params_to_sd,
};

use init::{
    I2C0_MUTEX, I2C1_MUTEX, I2c0Type, I2c1Type, RpUart, SPI1_MUTEX, SharedI2c, SharedI2c0,
    Spi1Device,
};

use common::coms::scheduler::LinkScheduler;
use common::coms::transport::uart::MAVLINK_MAX_FRAME;
use common::config::Config as AppConfig;
use common::drivers::bmi088::{Bmi088, Bmi088Raw};
use common::drivers::bmm350::RawMag;
use common::drivers::bmm350::{BMM350_ADDR, Bmm350};
use common::drivers::bmp581::{BMP581_ADDR_PRIMARY, Bmp581, Bmp581Config};
use common::drivers::lsm6dsv32x::{Lsm6dsv32x, Lsm6dsv32xRaw};
use common::drivers::neom9n::{GpsData, NeoM9n, UBLOX_I2C_ADDR};
use common::params::{ParamId, ParamRegistry};
use common::policies::fc_state::{FcState, FcStatePolicy};
use common::protocol::mavlink::encode::{
    HeartbeatConfig, build_device_op_read_reply_frame, build_device_op_write_reply_frame,
    build_frame_from_msg, build_heartbeat_frame, build_param_value_frame, build_statustext,
    build_statustext_frame,
};
use common::protocol::mavlink::handlers::{MessageHandlerResult, dispatch_mavlink_message};
use common::protocol::mavlink::prelude::{Frame, V2};
use common::protocol::mavlink::telemetry::{MavlinkTelemetryBundle, MavlinkTelemetrySource};
use common::tasks::coms::{
    MavEndpointConfig, TelemetrySample, TelemetrySource, UartTelemetryRate, UartTelemetrySnapshot,
    UartTelemetrySource, UartTxConfig,
};
use common::tasks::sensors::{
    DataSink, run_bmi088_task, run_bmm350_task, run_bmp581_task, run_lsm6dsv32x_task,
    run_neom9n_task,
};
use common::utils::i2cscanner::{DEFAULT_END_ADDR, DEFAULT_START_ADDR};

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Config as I2cConfig, I2c};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::uart::{Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Timer};
use embassy_usb::driver::EndpointError;
use embedded_hal_async::i2c::I2c as _;
use heapless::Deque;
use smart_leds::RGB8;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
// Bind I2C0/I2C1 interrupts for async I2C drivers
use embassy_rp::i2c::InterruptHandler as I2cInterruptHandler;
embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => I2cInterruptHandler<embassy_rp::peripherals::I2C1>;
});

embassy_rp::bind_interrupts!(struct UartIrqs {
    UART0_IRQ => UartInterruptHandler<embassy_rp::peripherals::UART0>;
});

embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<embassy_rp::peripherals::PIO0>;
});

embassy_rp::bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<embassy_rp::peripherals::USB>;
});

// Compile-time pinout contract for this binary.
// If `hardware/pinout.rs` changes, these checks force the wiring code below to be updated.
const _: () = {
    // SPI1 IMU bus
    core::assert!(fc::imu_spi1::MISO == 12);
    core::assert!(fc::imu_spi1::MOSI == 11);
    core::assert!(fc::imu_spi1::SCK == 10);
    core::assert!(fc::imu_spi1::CS_ACCEL == 13);
    core::assert!(fc::imu_spi1::CS_GYRO == 14);
    core::assert!(fc::imu_spi1::CS_LSM == 16);

    // SD SPI0 bus
    core::assert!(fc::sd_spi0::MISO == 20);
    core::assert!(fc::sd_spi0::MOSI == 19);
    core::assert!(fc::sd_spi0::SCK == 18);
    core::assert!(fc::sd_spi0::CS == 21);

    // I2C buses
    core::assert!(fc::i2c0::SDA == 8);
    core::assert!(fc::i2c0::SCL == 9);
    core::assert!(fc::i2c1::SDA == 2);
    core::assert!(fc::i2c1::SCL == 3);

    // LED and UART
    core::assert!(fc::led::DATA == 6);
    core::assert!(fc::uart0_fc_radio::TX == 0);
    core::assert!(fc::uart0_fc_radio::RX == 1);
};

// Parameter persistence tracking
static PARAMS_DIRTY: AtomicBool = AtomicBool::new(false);
static PARAMS_LAST_MODIFIED: AtomicU32 = AtomicU32::new(0);

// Provide an async delay adapter for generic drivers
struct EmbassyDelay;
impl common::utils::delay::DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after_millis(ms.into()).await;
    }
}
impl embedded_hal_async::delay::DelayNs for EmbassyDelay {
    async fn delay_ns(&mut self, ns: u32) {
        Timer::after_nanos(ns as u64).await;
    }
}

// ---- Shared state for unified logging ----

#[derive(Copy, Clone, Default)]
struct ImuData {
    accel: [i16; 3],
    gyro: [i16; 3],
    seq: u32,
}

#[derive(Copy, Clone, Default)]
struct MagData {
    xyz: [i32; 3],
    seq: u32,
}

#[derive(Copy, Clone, Default)]
struct BaroData {
    t_c_x100: i32,
    p_pa: i32,
    seq: u32,
}

#[derive(Copy, Clone, Default)]
struct GpsState {
    fix: Option<GpsData>,
    seq: u32,
}

#[derive(Copy, Clone, Default)]
struct SensorsState {
    imu: ImuData,
    imu2: ImuData,
    mag: MagData,
    baro: BaroData,
    gps: GpsState,
}

static STATE: Mutex<RawMutex, SensorsState> = Mutex::new(SensorsState {
    imu: ImuData {
        accel: [0; 3],
        gyro: [0; 3],
        seq: 0,
    },
    imu2: ImuData {
        accel: [0; 3],
        gyro: [0; 3],
        seq: 0,
    },
    mag: MagData {
        xyz: [0; 3],
        seq: 0,
    },
    baro: BaroData {
        t_c_x100: 0,
        p_pa: 0,
        seq: 0,
    },
    gps: GpsState { fix: None, seq: 0 },
});

// Parameter registry - shared between UART RX and TX tasks
static PARAMS: StaticCell<Mutex<RawMutex, ParamRegistry>> = StaticCell::new();
static FC_STATE: StaticCell<Mutex<RawMutex, FcState>> = StaticCell::new();

fn clamp_i16(v: i32) -> i16 {
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn hz_to_period_ms(hz: u16, min_ms: u32, max_ms: u32) -> u32 {
    if hz == 0 {
        return max_ms;
    }
    let ms = 1000 / hz as u32;
    ms.clamp(min_ms, max_ms).max(1)
}

async fn log_i2c_scan<I2C>(bus_name: &str, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    info!(
        "{} scanner: scanning 0x{:02X}..0x{:02X}",
        bus_name, DEFAULT_START_ADDR, DEFAULT_END_ADDR
    );
    let mut found = [0u8; 112];
    let mut count: usize = 0;

    for addr in DEFAULT_START_ADDR..=DEFAULT_END_ADDR {
        // Use a read ACK probe instead of a dummy write; this avoids false positives
        // seen on rp235x with write-only probing.
        let mut probe = [0u8; 1];
        if i2c.read(addr, &mut probe).await.is_ok() {
            found[count] = addr;
            count += 1;
        }
    }

    if count == 0 {
        info!("{} scanner: no devices found", bus_name);
        return;
    }

    info!("{} scanner: found {} device(s)", bus_name, count);
    for addr in &found[..count] {
        info!("{} scanner: device @ 0x{:02X}", bus_name, *addr);
    }
}

fn fc_policy_from_params(params: &ParamRegistry) -> FcStatePolicy {
    FcStatePolicy {
        usb_forces_config: params.bool(ParamId::FcModeUsbForcesConfig),
        allow_arm_in_config: params.bool(ParamId::FcArmAllowedInConfig),
    }
}

fn fc_heartbeat_hz(params: &ParamRegistry) -> u32 {
    params.u32(ParamId::FcStateHeartbeatHz).max(1)
}

struct StateTelemetrySource;

impl TelemetrySource for StateTelemetrySource {
    async fn latest(&self) -> TelemetrySample {
        let snap = STATE.lock().await.clone();
        let (lat, lon, alt, sats, fix) = if let Some(f) = snap.gps.fix {
            (
                f.latitude,
                f.longitude,
                f.altitude,
                f.satellites,
                f.fix_type,
            )
        } else {
            (0, 0, 0, 0, 0)
        };
        TelemetrySample {
            accel: snap.imu.accel,
            gyro: snap.imu.gyro,
            mag: [
                clamp_i16(snap.mag.xyz[0]),
                clamp_i16(snap.mag.xyz[1]),
                clamp_i16(snap.mag.xyz[2]),
            ],
            baro_p_pa: snap.baro.p_pa,
            baro_t_cx10: clamp_i16((snap.baro.t_c_x100 / 10) as i32),
            gps_lat_e7: lat,
            gps_lon_e7: lon,
            gps_alt_mm: alt,
            gps_sat: sats,
            gps_fix: fix,
        }
    }
}

impl MavlinkTelemetrySource for StateTelemetrySource {
    async fn bundle(&self, now_ms: u32, now_us: u64) -> MavlinkTelemetryBundle {
        let sample = self.latest().await;
        MavlinkTelemetryBundle {
            sys_status: common::protocol::mavlink::encode::build_sys_status(now_ms, &sample),
            raw_imu: common::protocol::mavlink::encode::build_raw_imu(now_us, &sample),
            scaled_pressure: common::protocol::mavlink::encode::build_scaled_pressure(
                now_ms, &sample,
            ),
            gps_raw_int: common::protocol::mavlink::encode::build_gps_raw_int(now_us, &sample),
            attitude: common::protocol::mavlink::encode::build_attitude(now_ms),
        }
    }
}

struct StateUartTelemetrySource;

impl UartTelemetrySource for StateUartTelemetrySource {
    async fn snapshot(&self) -> UartTelemetrySnapshot {
        let snap = STATE.lock().await.clone();
        UartTelemetrySnapshot {
            imu_seq: snap.imu.seq,
            imu_accel: snap.imu.accel,
            imu_gyro: snap.imu.gyro,
            mag_seq: snap.mag.seq,
            mag_xyz: snap.mag.xyz,
            baro_seq: snap.baro.seq,
            baro_t_c_x100: snap.baro.t_c_x100,
            baro_p_pa: snap.baro.p_pa,
            gps_seq: snap.gps.seq,
            gps_fix: snap.gps.fix,
        }
    }
}

struct ParamUartTelemetryRate {
    params: &'static Mutex<RawMutex, ParamRegistry>,
}

impl UartTelemetryRate for ParamUartTelemetryRate {
    async fn rate_hz(&self) -> u32 {
        let p = self.params.lock().await;
        p.u32(ParamId::RadioTelemRateHz)
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MARV-FC Dual IMU Test"),
    embassy_rp::binary_info::rp_program_description!(
        c"BMI088 + LSM6DSV32X IMU sensor test reading accelerometer and gyroscope data"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

pub(crate) async fn spawn_all(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("MARV-FC: Unified sensor output starting...");

    // Configure SPI1 for BMI088 + LSM6DSV32X
    // Hardware pins from pcbhardware.md:
    // MOSI: GP11, MISO: GP12, SCK: GP10
    // CS_ACCEL: GP13, CS_GYRO: GP14, CS_LSM: GP16
    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let sck = p.PIN_10;
    let cs_accel = Output::new(p.PIN_13, Level::High);
    let cs_gyro = Output::new(p.PIN_14, Level::High);
    let cs_lsm = Output::new(p.PIN_16, Level::High);

    // SK6812/WS2812-style addressable LED on GP6 using PIO0 SM0 + DMA CH4
    let mut pio0 = Pio::new(p.PIO0, PioIrqs);
    let ws2812_program = PioWs2812Program::new(&mut pio0.common);
    let ws2812 = PioWs2812::<_, 0, 1>::new(
        &mut pio0.common,
        pio0.sm0,
        p.DMA_CH4,
        p.PIN_6,
        &ws2812_program,
    );

    // Configure SPI with 10 MHz for high-rate IMU reads
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000; // 10 MHz - safe for BMI088/LSM6DSV32X

    let spi = Spi::new(p.SPI1, sck, mosi, miso, p.DMA_CH0, p.DMA_CH1, spi_config);

    // Configure SPI0 for microSD (blackbox logging)
    // Hardware pins from pcbhardware.md:
    // MOSI: GP19, MISO: GP20, SCK: GP18, CS: GP21
    let sd_miso = p.PIN_20;
    let sd_mosi = p.PIN_19;
    let sd_sck = p.PIN_18;
    let sd_cs = Output::new(p.PIN_21, Level::High);
    let mut sd_spi_cfg = SpiConfig::default();
    sd_spi_cfg.frequency = 1_000_000; // conservative bring-up frequency
    let sd_bus = Spi::new_blocking(p.SPI0, sd_sck, sd_mosi, sd_miso, sd_spi_cfg);

    // Keep SD API wiring in place, but startup currently uses defaults.
    let mut param_registry = ParamRegistry::new();
    let (config, _) = match load_config_and_params_from_sd(sd_bus, sd_cs, &mut param_registry) {
        Ok((cfg, count)) => {
            if count > 0 {
                info!("Loaded {} startup parameters", count);
            } else {
                info!("Using default startup parameters");
            }
            (cfg, count)
        }
        Err(_e) => {
            info!("Using default startup config and parameters");
            (AppConfig::default(), 0)
        }
    };

    let imu_interval_ms = hz_to_period_ms(config.imu_hz, 1, 1000);
    let mag_interval_ms = hz_to_period_ms(config.mag_hz, 10, 1000);
    let baro_interval_ms = hz_to_period_ms(config.baro_hz, 2, 1000);
    let gps_interval_ms = hz_to_period_ms(config.gps_hz, 20, 2000);

    let spi_bus = SPI1_MUTEX.init(Mutex::new(spi));
    let spi_accel = SpiDevice::new(spi_bus, cs_accel);
    let spi_gyro = SpiDevice::new(spi_bus, cs_gyro);
    let spi_lsm = SpiDevice::new(spi_bus, cs_lsm);

    // Create IMU instances; initialization happens inside the tasks
    let imu_bmi088 = Bmi088::new(spi_accel, spi_gyro);
    let imu_lsm6dsv32x = Lsm6dsv32x::new(spi_lsm);

    // ----- I2C0 (BMP581 barometer + ADXL375 high-g accel) -----
    // Hardware pins from pcbhardware.md:
    // I2C0 SDA: GP8, SCL: GP9
    let i2c0_sda = p.PIN_8;
    let i2c0_scl = p.PIN_9;
    let mut i2c0_cfg = I2cConfig::default();
    i2c0_cfg.frequency = 100_000; // 100 kHz for robust bring-up
    let i2c0 = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, i2c0_cfg);
    let i2c0_mutex = I2C0_MUTEX.init(Mutex::new(i2c0));

    // ----- I2C1 (BMM350 magnetometer, GPS) -----
    // Hardware pins from pcbhardware.md:
    // I2C1 SDA: GP2, SCL: GP3
    let i2c1_scl = p.PIN_3;
    let i2c1_sda = p.PIN_2;
    let mut i2c1_cfg = I2cConfig::default();
    i2c1_cfg.frequency = 100_000; // start at 100 kHz
    let i2c1 = I2c::new_async(p.I2C1, i2c1_scl, i2c1_sda, Irqs, i2c1_cfg);
    let i2c1_mutex = I2C1_MUTEX.init(Mutex::new(i2c1));

    // One-time startup scan to list devices visible on each I2C bus.
    let mut i2c0_scan = SharedI2c0 { bus: i2c0_mutex };
    log_i2c_scan("I2C0", &mut i2c0_scan).await;
    let mut i2c1_scan = SharedI2c { bus: i2c1_mutex };
    log_i2c_scan("I2C1", &mut i2c1_scan).await;

    let i2c_for_bmp = SharedI2c0 { bus: i2c0_mutex };
    // let i2c_for_adxl = SharedI2c0 { bus: i2c0_mutex };
    let i2c_for_bmm = SharedI2c { bus: i2c1_mutex };
    let i2c_for_gps = SharedI2c { bus: i2c1_mutex };

    // ----- UART0 (FC -> Radio) -----
    // TX: GP0, RX: GP1
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart_fc_radio = RpUart(Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, UartIrqs, p.DMA_CH2, p.DMA_CH3, uart_cfg,
    ));

    // ----- USB-CDC (primary configuration interface) -----
    let usb_driver = UsbDriver::new(p.USB, UsbIrqs);
    let (usb_dev, usb_cdc) = build_usb_cdc(usb_driver);

    let bmp_addr = BMP581_ADDR_PRIMARY;

    // Initialize parameter registry with loaded values
    let params = PARAMS.init(Mutex::new(param_registry));
    let fc_state = FC_STATE.init(Mutex::new(FcState::new()));

    // Create MAVLink endpoint config from params
    let params_guard = params.lock().await;
    let sys_id = params_guard.u32(ParamId::SysId) as u8;
    let comp_id = params_guard.u32(ParamId::CompId) as u8;
    // Get LED color from parameters
    let led_r = params_guard.u32(ParamId::LedColorR).min(255) as u8;
    let led_g = params_guard.u32(ParamId::LedColorG).min(255) as u8;
    let led_b = params_guard.u32(ParamId::LedColorB).min(255) as u8;
    drop(params_guard);
    let mav_cfg = MavEndpointConfig { sys_id, comp_id };
    info!(
        "MAVLink endpoint: sys_id={} comp_id={}",
        mav_cfg.sys_id, mav_cfg.comp_id
    );
    info!("LED color: R={} G={} B={}", led_r, led_g, led_b);

    // Start USB device task
    spawner.spawn(core1::usb_device_task(usb_dev)).unwrap();

    // Spawn sensor tasks
    spawner
        .spawn(core1::imu_task(imu_bmi088, imu_interval_ms))
        .unwrap();
    spawner
        .spawn(core1::imu2_task(imu_lsm6dsv32x, imu_interval_ms))
        .unwrap();
    spawner
        .spawn(core1::mag_task(i2c_for_bmm, mag_interval_ms))
        .unwrap();
    spawner
        .spawn(core1::baro_task(i2c_for_bmp, baro_interval_ms, bmp_addr))
        .unwrap();
    spawner
        .spawn(core1::gps_task(i2c_for_gps, gps_interval_ms))
        .unwrap();
    let led_color = RGB8 {
        r: led_r,
        g: led_g,
        b: led_b,
    };
    spawner
        .spawn(dma_tasks::logger_task(ws2812, led_color))
        .unwrap();
    spawner
        .spawn(core1::usb_mavlink_task(
            usb_cdc, mav_cfg, params, fc_state, i2c0_mutex, i2c1_mutex,
        ))
        .unwrap();
    spawner
        .spawn(dma_tasks::uart_link_task(uart_fc_radio, params))
        .unwrap();
}

// ----- Data sinks to bridge generic tasks into shared state -----

struct ImuSink;
impl DataSink<Bmi088Raw> for ImuSink {
    async fn publish(&mut self, data: Bmi088Raw) {
        let mut guard = STATE.lock().await;
        guard.imu.accel = data.accel;
        guard.imu.gyro = data.gyro;
        guard.imu.seq = guard.imu.seq.wrapping_add(1);
    }
}

struct Imu2Sink;
impl DataSink<Lsm6dsv32xRaw> for Imu2Sink {
    async fn publish(&mut self, data: Lsm6dsv32xRaw) {
        let mut guard = STATE.lock().await;
        guard.imu2.accel = data.accel;
        guard.imu2.gyro = data.gyro;
        guard.imu2.seq = guard.imu2.seq.wrapping_add(1);
    }
}

struct MagSink;
impl DataSink<RawMag> for MagSink {
    async fn publish(&mut self, data: RawMag) {
        let mut guard = STATE.lock().await;
        guard.mag.xyz = data.xyz;
        guard.mag.seq = guard.mag.seq.wrapping_add(1);
    }
}

struct BaroSink;
impl DataSink<(i32, i32)> for BaroSink {
    async fn publish(&mut self, data: (i32, i32)) {
        let mut guard = STATE.lock().await;
        guard.baro.t_c_x100 = data.0;
        guard.baro.p_pa = data.1;
        guard.baro.seq = guard.baro.seq.wrapping_add(1);
    }
}

struct GpsSink;
impl DataSink<GpsData> for GpsSink {
    async fn publish(&mut self, data: GpsData) {
        let mut guard = STATE.lock().await;
        guard.gps.fix = Some(data);
        guard.gps.seq = guard.gps.seq.wrapping_add(1);
    }
}

// Task implementations are split into `core1.rs` and `dma-tasks.rs`.
