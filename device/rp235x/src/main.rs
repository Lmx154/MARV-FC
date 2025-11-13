#![no_std]
#![no_main]
// Firmware entrypoint; only device-specific wiring lives here

use defmt::*;
use common::drivers::bmi088::{Bmi088, Bmi088Raw};
use common::drivers::bmm350::{Bmm350, BMM350_ADDR};
use common::drivers::bmp390::{Bmp390, BMP3X_ADDR_SDO_HIGH};
use common::drivers::bmm350::RawMag;
use common::tasks::sensors::{run_bmi088_task, run_bmm350_task, run_bmp390_task, run_neom9n_task, DataSink};
use common::drivers::neom9n::{NeoM9n, UBLOX_I2C_ADDR, GpsData};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::i2c::{Config as I2cConfig, I2c, Async as I2cAsync};
use embassy_time::{Timer, Instant};
use {defmt_rtt as _, panic_probe as _};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
// Bind I2C0/I2C1 interrupts for async I2C drivers
use embassy_rp::i2c::InterruptHandler;
embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

// Provide an async delay adapter for generic drivers
struct EmbassyDelay;
impl common::utils::delay::DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) { Timer::after_millis(ms.into()).await; }
}

// ---- Shared state for unified logging ----

#[derive(Copy, Clone, Default)]
struct ImuData { accel: [i16; 3], gyro: [i16; 3], seq: u32 }

#[derive(Copy, Clone, Default)]
struct MagData { xyz: [i32; 3], seq: u32 }

#[derive(Copy, Clone, Default)]
struct BaroData { t_c_x100: i32, p_pa: i32, seq: u32 }

#[derive(Copy, Clone, Default)]
struct GpsState { fix: Option<GpsData>, seq: u32 }

#[derive(Copy, Clone, Default)]
struct SensorsState {
    imu: ImuData,
    mag: MagData,
    baro: BaroData,
    gps: GpsState,
}

static STATE: Mutex<RawMutex, SensorsState> = Mutex::new(SensorsState { imu: ImuData { accel: [0;3], gyro: [0;3], seq: 0 }, mag: MagData { xyz: [0;3], seq: 0 }, baro: BaroData { t_c_x100: 0, p_pa: 0, seq: 0 }, gps: GpsState { fix: None, seq: 0 } });

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

    // ----- I2C0 (BMP390 barometer) -----
    // Hardware pins from hardware.md:
    // I2C0 SDA: GP12, SCL: GP13
    let i2c0_sda = p.PIN_12;
    let i2c0_scl = p.PIN_13;
    let mut i2c0_cfg = I2cConfig::default();
    i2c0_cfg.frequency = 100_000; // 100 kHz for robust bring-up
    let i2c0 = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, i2c0_cfg);
    let i2c0_mutex = I2C0_MUTEX.init(Mutex::new(i2c0));
    let i2c_for_bmp = SharedI2c0 { bus: i2c0_mutex };

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

    // Spawn sensor tasks
    spawner.spawn(imu_task(imu_bmi088)).unwrap();
    spawner.spawn(mag_task(i2c_for_bmm)).unwrap();
    spawner.spawn(baro_task(i2c_for_bmp)).unwrap();
    spawner.spawn(gps_task(i2c_for_gps)).unwrap();
    spawner.spawn(logger_task(led)).unwrap();
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
async fn imu_task(mut imu: Bmi088<Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>, Output<'static>, Output<'static>>) {
    let mut delay = EmbassyDelay;
    let mut sink = ImuSink;
    // 0 => run as fast as possible; change to 1 for ~1kHz pacing
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

#[embassy_executor::task]
async fn logger_task(mut led: Output<'static>) {
    let mut n: u32 = 0;
    // Track previous counters and time to compute real per-second rates
    let mut prev_imu_seq: u32 = 0;
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
                "IMU A[{},{},{}] G[{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS {:02}/{:02}/{:04} {:02}:{:02}:{:02} lat {}.{:07} lon {}.{:07} alt {}m sats {} fix {} | seqs i:{} m:{} b:{} g:{}",
                s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                fix.month, fix.day, fix.year, fix.hour, fix.minute, fix.second,
                fix.latitude/10_000_000, (fix.latitude%10_000_000).abs(),
                fix.longitude/10_000_000, (fix.longitude%10_000_000).abs(),
                fix.altitude/1000, fix.satellites, fix.fix_type,
                s.imu.seq, s.mag.seq, s.baro.seq, s.gps.seq
            );
        } else {
            debug!(
                "IMU A[{},{},{}] G[{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS no-fix | seqs i:{} m:{} b:{} g:{}",
                s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                s.imu.seq, s.mag.seq, s.baro.seq, s.gps.seq
            );
        }

        // Once per ~1s (or slightly more), compute true rates from deltas and print with current values
        let elapsed_ms = prev_t.elapsed().as_millis() as u32;
        if elapsed_ms >= 1000 {
            let imu_rate = (s.imu.seq.wrapping_sub(prev_imu_seq) * 1000) / elapsed_ms.max(1);
            let mag_rate = (s.mag.seq.wrapping_sub(prev_mag_seq) * 1000) / elapsed_ms.max(1);
            let baro_rate = (s.baro.seq.wrapping_sub(prev_baro_seq) * 1000) / elapsed_ms.max(1);
            let gps_rate = (s.gps.seq.wrapping_sub(prev_gps_seq) * 1000) / elapsed_ms.max(1);
            if let Some(fix) = s.gps.fix {
                info!(
                    "Rates IMU:{} Hz MAG:{} Hz BARO:{} Hz GPS:{} Hz | IMU A[{},{},{}] G[{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS {:02}/{:02}/{:04} {:02}:{:02}:{:02} lat {}.{:07} lon {}.{:07} alt {}m sats {} fix {}",
                    imu_rate, mag_rate, baro_rate, gps_rate,
                    s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                    s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                    s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                    (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa,
                    fix.month, fix.day, fix.year, fix.hour, fix.minute, fix.second,
                    fix.latitude/10_000_000, (fix.latitude%10_000_000).abs(),
                    fix.longitude/10_000_000, (fix.longitude%10_000_000).abs(),
                    fix.altitude/1000, fix.satellites, fix.fix_type
                );
            } else {
                info!(
                    "Rates IMU:{} Hz MAG:{} Hz BARO:{} Hz GPS:{} Hz | IMU A[{},{},{}] G[{},{},{}] | MAG [{},{},{}] | BARO T={}c P={} Pa | GPS no-fix",
                    imu_rate, mag_rate, baro_rate, gps_rate,
                    s.imu.accel[0], s.imu.accel[1], s.imu.accel[2],
                    s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
                    s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2],
                    (s.baro.t_c_x100 as f32) / 100.0, s.baro.p_pa
                );
            }
            prev_imu_seq = s.imu.seq;
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
async fn gps_task(i2c_dev: SharedI2c<'static>) {
    let mut delay = EmbassyDelay;
    info!("NEO-M9N: starting task on I2C1 @0x{:02X}", UBLOX_I2C_ADDR);
    let mut gps = NeoM9n::new(i2c_dev, UBLOX_I2C_ADDR);
    gps.set_debug(false);
    let mut sink = GpsSink;
    // ~20 Hz polling; device may output ~10 Hz by default
    run_neom9n_task(&mut gps, &mut delay, &mut sink, 50).await;
}
