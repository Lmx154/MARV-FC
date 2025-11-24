#![no_std]
#![no_main]
// Firmware entrypoint; only device-specific wiring lives here

use defmt::*;
use common::drivers::bmi088::{Bmi088, Bmi088Raw};
use common::drivers::bmm350::{Bmm350, BMM350_ADDR};
use common::drivers::bmp390::{Bmp390, BMP3X_ADDR_SDO_HIGH};
use common::drivers::bmm350::RawMag;
use common::drivers::neom9n::{NeoM9n, UBLOX_I2C_ADDR, GpsData};
use common::tasks::coms::{
    build_telemetry_frame, MavEndpointConfig, TelemetrySample, TelemetrySource,
};
use common::tasks::sensors::{
    run_bmi088_task,
    run_bmm350_task,
    run_bmp390_task,
    run_neom9n_task,
    DataSink,
};
use common::coms::uart_coms::{AsyncUartBus, MAVLINK_MAX_FRAME};
use common::mavlink2;
use common::utils::i2cscanner::scan_i2c_bus_default;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::i2c::{Config as I2cConfig, I2c, Async as I2cAsync};
use embassy_rp::uart::{Config as UartConfig, Uart, InterruptHandler as UartInterruptHandler, Async as UartAsync};
use embassy_time::{Timer, Instant};
use {defmt_rtt as _, panic_probe as _};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
// Bind I2C0/I2C1 interrupts for async I2C drivers
use embassy_rp::i2c::InterruptHandler as I2cInterruptHandler;
embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => I2cInterruptHandler<embassy_rp::peripherals::I2C1>;
});

embassy_rp::bind_interrupts!(struct UartIrqs {
    UART0_IRQ => UartInterruptHandler<embassy_rp::peripherals::UART0>;
});

// ADXL375 I2C address (adjust if ALT_ADDRESS wiring differs)
// const ADXL375_ADDR: u8 = 0x53;

// Provide an async delay adapter for generic drivers
struct EmbassyDelay;
impl common::utils::delay::DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) { Timer::after_millis(ms.into()).await; }
}

// ---- Shared state for unified logging ----

#[derive(Copy, Clone, Default)]
struct ImuData { accel: [i16; 3], gyro: [i16; 3], seq: u32 }

#[derive(Copy, Clone, Default)]
struct HighGData { accel: [i16; 3], seq: u32 }

#[derive(Copy, Clone, Default)]
struct MagData { xyz: [i32; 3], seq: u32 }

#[derive(Copy, Clone, Default)]
struct BaroData { t_c_x100: i32, p_pa: i32, seq: u32 }

#[derive(Copy, Clone, Default)]
struct GpsState { fix: Option<GpsData>, seq: u32 }

#[derive(Copy, Clone, Default)]
struct SensorsState {
    imu: ImuData,
    highg: HighGData,
    mag: MagData,
    baro: BaroData,
    gps: GpsState,
}

static STATE: Mutex<RawMutex, SensorsState> = Mutex::new(SensorsState {
    imu: ImuData { accel: [0;3], gyro: [0;3], seq: 0 },
    highg: HighGData { accel: [0;3], seq: 0 },
    mag: MagData { xyz: [0;3], seq: 0 },
    baro: BaroData { t_c_x100: 0, p_pa: 0, seq: 0 },
    gps: GpsState { fix: None, seq: 0 },
});

fn clamp_i16(v: i32) -> i16 {
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

struct StateTelemetrySource;

impl TelemetrySource for StateTelemetrySource {
    async fn latest(&self) -> TelemetrySample {
        let snap = STATE.lock().await.clone();
        let (lat, lon, alt, sats, fix) = if let Some(f) = snap.gps.fix {
            (f.latitude, f.longitude, f.altitude, f.satellites, f.fix_type)
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

// Shared I2C0/I2C1 bus managers for multiple I2C devices
type I2c0Type = I2c<'static, embassy_rp::peripherals::I2C0, I2cAsync>;
type I2c1Type = I2c<'static, embassy_rp::peripherals::I2C1, I2cAsync>;
// We'll implement a tiny shared I2C wrapper using a Mutex to allow multiple tasks to access the same bus concurrently.
static I2C0_MUTEX: StaticCell<Mutex<RawMutex, I2c0Type>> = StaticCell::new();
static I2C1_MUTEX: StaticCell<Mutex<RawMutex, I2c1Type>> = StaticCell::new();

#[derive(Clone, Copy)]
struct SharedI2c0<'a> {
    bus: &'a Mutex<RawMutex, I2c0Type>,
}

impl<'a> embedded_hal_async::i2c::ErrorType for SharedI2c0<'a> {
    type Error = embassy_rp::i2c::Error;
}

impl<'a> embedded_hal_async::i2c::I2c for SharedI2c0<'a> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write_read(address, write, read).await
    }

    async fn transaction(&mut self, address: u8, operations: &mut [embedded_hal_async::i2c::Operation<'_>]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.transaction(address, operations).await
    }
}

#[derive(Clone, Copy)]
struct SharedI2c<'a> {
    bus: &'a Mutex<RawMutex, I2c1Type>,
}

impl<'a> embedded_hal_async::i2c::ErrorType for SharedI2c<'a> {
    type Error = embassy_rp::i2c::Error;
}

impl<'a> embedded_hal_async::i2c::I2c for SharedI2c<'a> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.write_read(address, write, read).await
    }

    async fn transaction(&mut self, address: u8, operations: &mut [embedded_hal_async::i2c::Operation<'_>]) -> Result<(), Self::Error> {
        let mut g = self.bus.lock().await;
        g.transaction(address, operations).await
    }
}

// Generic async UART trait implementation for the RP UART peripheral.
struct RpUart<'d>(Uart<'d, UartAsync>);

impl<'d> AsyncUartBus for RpUart<'d> {
    type Error = embassy_rp::uart::Error;

    async fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), Self::Error> {
        Uart::write(&mut self.0, bytes).await
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> core::result::Result<(), Self::Error> {
        Uart::read(&mut self.0, buf).await
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MARV-FC BMI088 Test"),
    embassy_rp::binary_info::rp_program_description!(
        c"BMI088 IMU sensor test reading accelerometer and gyroscope data"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("MARV-FC: Unified sensor output starting...");

    // Configure SPI0 for BMI088
    // Hardware pins from hardware.md:
    // MOSI: GP3, MISO: GP0, SCK: GP2
    // CS_ACCEL: GP1, CS_GYRO: GP4
    let miso = p.PIN_0;
    let mosi = p.PIN_3;
    let sck = p.PIN_2;
    let cs_accel = Output::new(p.PIN_1, Level::High);
    let cs_gyro = Output::new(p.PIN_4, Level::High);

    // LED for status indication
    let led = Output::new(p.PIN_16, Level::Low);

    // Configure SPI with 10 MHz for maximum performance stress test
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000; // 10 MHz - max for BMI088

    let spi = Spi::new(
        p.SPI0,
        sck,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        spi_config,
    );

    // Create BMI088 instance; initialization happens inside the task
    let imu_bmi088 = Bmi088::new(spi, cs_accel, cs_gyro);

    // ----- I2C0 (BMP390 barometer + ADXL375 high-g accel) -----
    // Hardware pins from hardware.md:
    // I2C0 SDA: GP12, SCL: GP13
    let i2c0_sda = p.PIN_12;
    let i2c0_scl = p.PIN_13;
    let mut i2c0_cfg = I2cConfig::default();
    i2c0_cfg.frequency = 100_000; // 100 kHz for robust bring-up
    let i2c0 = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, i2c0_cfg);
    let i2c0_mutex = I2C0_MUTEX.init(Mutex::new(i2c0));
    let i2c_for_bmp = SharedI2c0 { bus: i2c0_mutex };
    // let i2c_for_adxl = SharedI2c0 { bus: i2c0_mutex };

    // ----- I2C1 (BMM350 magnetometer, GPS) -----
    // Hardware pins from hardware.md:
    // I2C1 SDA: GP6, SCL: GP7
    let i2c1_scl = p.PIN_7;
    let i2c1_sda = p.PIN_6;
    let mut i2c1_cfg = I2cConfig::default();
    i2c1_cfg.frequency = 100_000; // start at 100 kHz
    let i2c1 = I2c::new_async(p.I2C1, i2c1_scl, i2c1_sda, Irqs, i2c1_cfg);
    let i2c1_mutex = I2C1_MUTEX.init(Mutex::new(i2c1));
    let i2c_for_bmm = SharedI2c { bus: i2c1_mutex };
    let i2c_for_gps = SharedI2c { bus: i2c1_mutex };

    // ----- UART0 (FC -> Radio) -----
    // TX: GP14, RX: GP15
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart_fc_radio = RpUart(Uart::new(
        p.UART0,
        p.PIN_14,
        p.PIN_15,
        UartIrqs,
        p.DMA_CH2,
        p.DMA_CH3,
        uart_cfg,
    ));

    // ----- I2C bus scans at startup (for debug / bring-up / menus) -----
    {
        info!("I2C0 scan: starting (0x08-0x77)...");
        let mut bus0 = SharedI2c0 { bus: i2c0_mutex };
        let scan0 = scan_i2c_bus_default(&mut bus0).await;
        if scan0.is_empty() {
            warn!("I2C0 scan: no devices found in 0x08-0x77");
        } else {
            info!("I2C0 scan: found {} device(s)", scan0.count);
            for addr in scan0.iter() {
                info!("  I2C0 device @ 0x{:02X}", addr);
            }
        }

        info!("I2C1 scan: starting (0x08-0x77)...");
        let mut bus1 = SharedI2c { bus: i2c1_mutex };
        let scan1 = scan_i2c_bus_default(&mut bus1).await;
        if scan1.is_empty() {
            warn!("I2C1 scan: no devices found in 0x08-0x77");
        } else {
            info!("I2C1 scan: found {} device(s)", scan1.count);
            for addr in scan1.iter() {
                info!("  I2C1 device @ 0x{:02X}", addr);
            }
        }
    }

    let mav_cfg = MavEndpointConfig { sys_id: 1, comp_id: 1 };

    // Spawn sensor tasks
    spawner.spawn(imu_task(imu_bmi088)).unwrap();
    spawner.spawn(mag_task(i2c_for_bmm)).unwrap();
    spawner.spawn(baro_task(i2c_for_bmp)).unwrap();
    spawner.spawn(gps_task(i2c_for_gps)).unwrap();
    spawner.spawn(logger_task(led)).unwrap();
    spawner.spawn(uart_tx_task(uart_fc_radio, mav_cfg)).unwrap();
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

// ----- Tasks -----

#[embassy_executor::task]
async fn imu_task(
    mut imu: Bmi088<
        Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
        Output<'static>,
        Output<'static>
    >
) {
    let mut delay = EmbassyDelay;
    let mut sink = ImuSink;
    // 1 ms interval (~1 kHz pacing)
    run_bmi088_task(&mut imu, &mut delay, &mut sink, 1).await;
}

#[embassy_executor::task]
async fn mag_task(i2c_dev: SharedI2c<'static>) {
    let mut delay = EmbassyDelay;
    info!("BMM350: starting task on I2C1 @0x{:02X}", BMM350_ADDR);
    let mut mag = Bmm350::new(i2c_dev, BMM350_ADDR);
    mag.set_debug(false);
    let mut sink = MagSink;
    run_bmm350_task(&mut mag, &mut delay, &mut sink, 40).await; // ~25 Hz pacing
}

#[embassy_executor::task]
async fn baro_task(i2c_dev: SharedI2c0<'static>) {
    let mut delay = EmbassyDelay;
    info!("BMP390: starting task on I2C0 @0x{:02X}", BMP3X_ADDR_SDO_HIGH);
    let mut bmp = Bmp390::new(i2c_dev, BMP3X_ADDR_SDO_HIGH);
    bmp.set_debug(false);
    let mut sink = BaroSink;
    run_bmp390_task(&mut bmp, &mut delay, &mut sink, 4).await;
}

// #[embassy_executor::task]
// async fn adxl_task(i2c_dev: SharedI2c0<'static>) {
//     let mut delay = EmbassyDelay;
//     info!("ADXL375: starting task on I2C0 @0x{:02X}", ADXL375_ADDR);
//     // INT pin not wired (or ignored) here, so we use a dummy type and None
//     let mut accel: Adxl375<SharedI2c0<'static>, Input<'static>> =
//         Adxl375::new(i2c_dev, ADXL375_ADDR, None);
//     let mut sink = HighGSink;
//     // e.g. 1 ms interval (~1 kHz logical pacing; actual ODR set in driver init)
//     run_adxl375_task(&mut accel, &mut delay, &mut sink, 1).await;
// }

#[embassy_executor::task]
async fn logger_task(mut led: Output<'static>) {
    let mut n: u32 = 0;
    // Track previous counters and time to compute real per-second rates
    let mut prev_imu_seq: u32 = 0;
    let mut prev_highg_seq: u32 = 0;
    let mut prev_mag_seq: u32 = 0;
    let mut prev_baro_seq: u32 = 0;
    let mut prev_gps_seq: u32 = 0;
    let mut prev_t = Instant::now();
    loop {
        // Fetch a snapshot and emit unified line at fastest cadence (~1 kHz)
        let s = STATE.lock().await.clone();
        // More detailed data at debug level every loop
        if let Some(fix) = s.gps.fix {
            debug!(
                "IMU A[{},{},{}] G[{},{},{}] | HG [{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS {:02}/{:02}/{:04} {:02}:{:02}:{:02} lat {}.{:07} lon {}.{:07} alt {}m sats {} fix {} | seqs i:{} hg:{} m:{} b:{} g:{}",
                s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                s.highg.accel[0], s.highg.accel[1], s.highg.accel[2],
                s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                fix.month, fix.day, fix.year, fix.hour, fix.minute, fix.second,
                fix.latitude/10_000_000, (fix.latitude%10_000_000).abs(),
                fix.longitude/10_000_000, (fix.longitude%10_000_000).abs(),
                fix.altitude/1000, fix.satellites, fix.fix_type,
                s.imu.seq, s.highg.seq, s.mag.seq, s.baro.seq, s.gps.seq
            );
        } else {
            debug!(
                "IMU A[{},{},{}] G[{},{},{}] | HG [{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS no-fix | seqs i:{} hg:{} m:{} b:{} g:{}",
                s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                s.highg.accel[0], s.highg.accel[1], s.highg.accel[2],
                s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                s.imu.seq, s.highg.seq, s.mag.seq, s.baro.seq, s.gps.seq
            );
        }

        // Once per ~1s (or slightly more), compute true rates from deltas and print with current values
        let elapsed_ms = prev_t.elapsed().as_millis() as u32;
        if elapsed_ms >= 1000 {
            let imu_rate = (s.imu.seq.wrapping_sub(prev_imu_seq) * 1000) / elapsed_ms.max(1);
            let highg_rate = (s.highg.seq.wrapping_sub(prev_highg_seq) * 1000) / elapsed_ms.max(1);
            let mag_rate = (s.mag.seq.wrapping_sub(prev_mag_seq) * 1000) / elapsed_ms.max(1);
            let baro_rate = (s.baro.seq.wrapping_sub(prev_baro_seq) * 1000) / elapsed_ms.max(1);
            let gps_rate = (s.gps.seq.wrapping_sub(prev_gps_seq) * 1000) / elapsed_ms.max(1);
            if let Some(fix) = s.gps.fix {
                info!(
                    "Rates IMU:{} Hz HG:{} Hz MAG:{} Hz BARO:{} Hz GPS:{} Hz | IMU A[{},{},{}] G[{},{},{}] | HG [{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS {:02}/{:02}/{:04} {:02}:{:02}:{:02} lat {}.{:07} lon {}.{:07} alt {}m sats {} fix {}",
                    imu_rate, highg_rate, mag_rate, baro_rate, gps_rate,
                    s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                    s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                    s.highg.accel[0], s.highg.accel[1], s.highg.accel[2],
                    s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                    (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                    fix.month, fix.day, fix.year, fix.hour, fix.minute, fix.second,
                    fix.latitude/10_000_000, (fix.latitude%10_000_000).abs(),
                    fix.longitude/10_000_000, (fix.longitude%10_000_000).abs(),
                    fix.altitude/1000, fix.satellites, fix.fix_type
                );
            } else {
                info!(
                    "Rates IMU:{} Hz HG:{} Hz MAG:{} Hz BARO:{} Hz GPS:{} Hz | IMU A[{},{},{}] G[{},{},{}] | HG [{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS no-fix",
                    imu_rate, highg_rate, mag_rate, baro_rate, gps_rate,
                    s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                    s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                    s.highg.accel[0], s.highg.accel[1], s.highg.accel[2],
                    s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                    (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa
                );
            }
            prev_imu_seq = s.imu.seq;
            prev_highg_seq = s.highg.seq;
            prev_mag_seq = s.mag.seq;
            prev_baro_seq = s.baro.seq;
            prev_gps_seq = s.gps.seq;
            prev_t = Instant::now();
        }

        // Simple heartbeat
        led.set_high();
        Timer::after_micros(200).await;
        led.set_low();
        Timer::after_micros(800).await; // total 1ms period
        n = n.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart: RpUart<'static>, cfg: MavEndpointConfig) {
    let mut seq: u8 = 0;
    let mut scratch = [0u8; MAVLINK_MAX_FRAME];
    let source = StateTelemetrySource;
    loop {
        let sample = source.latest().await;
        if let Some(frame) = build_telemetry_frame(cfg, seq, &sample) {
            seq = seq.wrapping_add(1);
            match mavlink2::send_frame_over_uart(&mut uart, &frame, &mut scratch).await {
                Ok(()) => {
                    debug!("UART telemetry sent seq={}", frame.sequence());
                }
                Err(e) => {
                    warn!("UART telemetry send failed: {:?}", e);
                }
            }
        } else {
            warn!("UART telemetry encode failed (buffer too small)");
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn gps_task(i2c_dev: SharedI2c<'static>) {
    let mut delay = EmbassyDelay;
    info!("NEO-M9N: starting task on I2C1 @0x{:02X}", UBLOX_I2C_ADDR);
    let mut gps = NeoM9n::new(i2c_dev, UBLOX_I2C_ADDR);
    gps.set_debug(false);
    let mut sink = GpsSink;
    // ~20 Hz polling; device may output ~10 Hz by default
    run_neom9n_task(&mut gps, &mut delay, &mut sink, 50).await;
}
