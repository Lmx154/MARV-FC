#![no_std]
#![no_main]
// Firmware entrypoint; only device-specific wiring lives here

use defmt::*;
use common::drivers::bmi088::Bmi088;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Provide an async delay adapter for generic drivers
struct EmbassyDelay;
impl common::utils::delay::DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after_millis(ms.into()).await;
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
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("MARV-FC: BMI088 IMU Stress Test starting...");

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
    let mut led = Output::new(p.PIN_16, Level::Low);

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

    // Create and initialize BMI088 driver
    let mut imu = Bmi088::new(spi, cs_accel, cs_gyro);

    info!("Initializing BMI088 at 10 MHz SPI...");
    let mut delay = EmbassyDelay;
    match imu.init(&mut delay).await {
        Ok(_) => info!("BMI088 initialized successfully!"),
        Err(e) => {
            error!("BMI088 initialization failed: {:?}", e);
            loop {
                // Blink LED rapidly to indicate error
                led.set_high();
                Timer::after_millis(100).await;
                led.set_low();
                Timer::after_millis(100).await;
            }
        }
    }

    info!("=== STRESS TEST MODE ===");
    info!("Running at maximum speed (1000 Hz)");
    info!("Monitoring error rate and data consistency");
    
    // Statistics tracking
    let mut sample_count = 0u32;
    let mut error_count = 0u32;
    let mut accel_sum = [0i64; 3];
    let mut gyro_sum = [0i64; 3];
    let mut last_accel = [0i16; 3];
    let mut large_jump_count = 0u32;
    
    const SAMPLE_RATE_HZ: u32 = 1000;
    const REPORT_INTERVAL: u32 = 1000; // Report every 1000 samples (1 second)
    const JUMP_THRESHOLD: i16 = 2000; // Threshold for detecting bad readings

    loop {
        led.set_high();

        // Read combined accelerometer and gyroscope data
        match imu.read_raw().await {
            Ok(data) => {
                // Check for unrealistic jumps in accelerometer data
                if sample_count > 0 {
                    for i in 0..3 {
                        let diff = (data.accel[i] - last_accel[i]).abs();
                        if diff > JUMP_THRESHOLD {
                            large_jump_count += 1;
                            warn!(
                                "Large accel jump on axis {}: {} -> {} (diff={})",
                                i, last_accel[i], data.accel[i], diff
                            );
                        }
                    }
                }
                last_accel = data.accel;

                // Accumulate statistics
                for i in 0..3 {
                    accel_sum[i] += data.accel[i] as i64;
                    gyro_sum[i] += data.gyro[i] as i64;
                }

                sample_count += 1;

                // Report statistics every REPORT_INTERVAL samples
                if sample_count % REPORT_INTERVAL == 0 {
                    let accel_avg = [
                        (accel_sum[0] / REPORT_INTERVAL as i64) as i16,
                        (accel_sum[1] / REPORT_INTERVAL as i64) as i16,
                        (accel_sum[2] / REPORT_INTERVAL as i64) as i16,
                    ];
                    let gyro_avg = [
                        (gyro_sum[0] / REPORT_INTERVAL as i64) as i16,
                        (gyro_sum[1] / REPORT_INTERVAL as i64) as i16,
                        (gyro_sum[2] / REPORT_INTERVAL as i64) as i16,
                    ];

                    let error_rate = (error_count * 1000 / sample_count) as u32; // errors per 1000 samples
                    let jump_rate = (large_jump_count * 1000 / sample_count) as u32; // jumps per 1000 samples

                    info!(
                        "=== Stats @ {} samples ({} sec) ===",
                        sample_count,
                        sample_count / SAMPLE_RATE_HZ
                    );
                    info!(
                        "Accel AVG: [{}, {}, {}] | Gyro AVG: [{}, {}, {}]",
                        accel_avg[0], accel_avg[1], accel_avg[2],
                        gyro_avg[0], gyro_avg[1], gyro_avg[2]
                    );
                    info!(
                        "Errors: {} ({}/1000) | Jumps: {} ({}/1000)",
                        error_count, error_rate, large_jump_count, jump_rate
                    );
                    info!(
                        "Latest: Accel[{}, {}, {}] Gyro[{}, {}, {}]",
                        data.accel[0], data.accel[1], data.accel[2],
                        data.gyro[0], data.gyro[1], data.gyro[2]
                    );

                    // Reset interval statistics
                    accel_sum = [0; 3];
                    gyro_sum = [0; 3];
                }

                // Log individual samples every 100 samples for spot checking
                if sample_count % 100 == 0 {
                    debug!(
                        "Sample #{}: A[{}, {}, {}] G[{}, {}, {}]",
                        sample_count,
                        data.accel[0], data.accel[1], data.accel[2],
                        data.gyro[0], data.gyro[1], data.gyro[2]
                    );
                }
            }
            Err(e) => {
                error_count += 1;
                error!("IMU read error #{}: {:?}", error_count, e);
            }
        }

        led.set_low();
        
        // Run at 1000 Hz (1 ms interval)
        Timer::after_micros(1000).await;
    }
}
