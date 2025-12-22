#![no_std]
#![no_main]
// Firmware entrypoint; only device-specific wiring lives here

mod hardware;

use defmt::*;
use hardware::{load_config_and_params_from_sd, save_params_to_sd};

use common::config::Config as AppConfig;
use common::drivers::bmi088::{Bmi088, Bmi088Raw};
use common::drivers::bmm350::{Bmm350, BMM350_ADDR};
use common::drivers::bmp390::{Bmp390, BMP3X_ADDR_SDO_HIGH};
use common::drivers::bmm350::RawMag;
use common::drivers::neom9n::{NeoM9n, UBLOX_I2C_ADDR, GpsData};
use common::tasks::coms::{
    MavEndpointConfig, TelemetrySample, TelemetrySource,
};
use common::tasks::sensors::{
    run_bmi088_task,
    run_bmm350_task,
    run_bmp390_task,
    run_neom9n_task,
    DataSink,
};
use common::coms::uart_coms::{AsyncUartBus, MAVLINK_MAX_FRAME};
use common::protocol::mavlink as mavlink;
use common::protocol::mavlink::encode::{
    build_param_value_frame,
    build_statustext,
    build_statustext_frame,
    build_heartbeat_frame,
    build_telemetry_frame,
    HeartbeatConfig,
    build_device_op_read_reply_frame,
    build_device_op_write_reply_frame,
    build_frame_from_msg,
};
use common::protocol::mavlink::link_authority::{
    build_link_authority_frame, LAS_STATE_ARMED, LAS_STATE_DISARMED,
};
use common::protocol::mavlink::handlers::{dispatch_mavlink_message, MessageHandlerResult};
use common::protocol::mavlink::link_mac_config::build_link_mac_config_command_frame;
use common::coms::transport::lora::mac::LinkMacConfig;
use common::protocol::mavlink::telemetry::{MavlinkTelemetryBundle, MavlinkTelemetrySource};
use common::protocol::mavlink::prelude::{Frame, V2};
use common::params::{ParamRegistry, ParamId};
use common::policies::fc_state::{FcState, FcStatePolicy};
use common::coms::scheduler::LinkScheduler;
use common::utils::i2cscanner::scan_i2c_bus_default;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Config as I2cConfig, I2c, Async as I2cAsync};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::uart::{Config as UartConfig, Uart, InterruptHandler as UartInterruptHandler, Async as UartAsync};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State as CdcAcmState},
    driver::EndpointError,
    Builder, Config as UsbConfig, UsbDevice,
};
use embedded_hal_async::i2c::I2c as _;
use embassy_time::{Timer, Instant};
use {defmt_rtt as _, panic_probe as _};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_futures::select;
use static_cell::StaticCell;
use smart_leds::RGB8;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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

static USB_CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// Parameter persistence tracking
static PARAMS_DIRTY: AtomicBool = AtomicBool::new(false);
static PARAMS_LAST_MODIFIED: AtomicU32 = AtomicU32::new(0);

// USB-CDC wrapper for MAVLink
struct UsbCdc<'d> {
    class: CdcAcmClass<'d, UsbDriver<'d, embassy_rp::peripherals::USB>>,
    max_packet: usize,
}

impl<'d> UsbCdc<'d> {
    async fn wait_connection(&mut self) {
        self.class.wait_connection().await;
    }

    async fn read_packet(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.class.read_packet(buf).await
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), EndpointError> {
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let n = core::cmp::min(remaining.len(), self.max_packet);
            self.class.write_packet(&remaining[..n]).await?;
            remaining = &remaining[n..];
        }
        // Short packet to flush if we ended on a packet boundary
        if !bytes.is_empty() && bytes.len() % self.max_packet == 0 {
            self.class.write_packet(&[]).await?;
        }
        Ok(())
    }
}

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

impl MavlinkTelemetrySource for StateTelemetrySource {
    async fn bundle(&self, now_ms: u32, now_us: u64) -> MavlinkTelemetryBundle {
        let sample = self.latest().await;
        MavlinkTelemetryBundle {
            sys_status: common::protocol::mavlink::encode::build_sys_status(now_ms, &sample),
            raw_imu: common::protocol::mavlink::encode::build_raw_imu(now_us, &sample),
            scaled_pressure: common::protocol::mavlink::encode::build_scaled_pressure(now_ms, &sample),
            gps_raw_int: common::protocol::mavlink::encode::build_gps_raw_int(now_us, &sample),
            attitude: common::protocol::mavlink::encode::build_attitude(now_ms),
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

    // SK6812/WS2812-style addressable LED on GP16 using PIO0 SM0 + DMA CH4
    let mut pio0 = Pio::new(p.PIO0, PioIrqs);
    let ws2812_program = PioWs2812Program::new(&mut pio0.common);
    let ws2812 = PioWs2812::<_, 0, 1>::new(
        &mut pio0.common,
        pio0.sm0,
        p.DMA_CH4,
        p.PIN_16,
        &ws2812_program,
    );

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

    // Configure SPI1 for microSD (blackbox logging)
    // Hardware pins from hardware.md:
    // MOSI: GP11, MISO: GP8, SCK: GP10, CS: GP9
    let sd_miso = p.PIN_8;
    let sd_mosi = p.PIN_11;
    let sd_sck = p.PIN_10;
    let sd_cs = Output::new(p.PIN_9, Level::High);
    let mut sd_spi_cfg = SpiConfig::default();
    sd_spi_cfg.frequency = 1_000_000; // conservative bring-up frequency
    let sd_bus = Spi::new_blocking(
        p.SPI1,
        sd_sck,
        sd_mosi,
        sd_miso,
        sd_spi_cfg,
    );

    // Load both config and parameters from SD card
    let mut param_registry = ParamRegistry::new();
    let (config, _) = match load_config_and_params_from_sd(sd_bus, sd_cs, &mut param_registry) {
        Ok((cfg, count)) => {
            info!("Config loaded from SD: {:?}", cfg);
            if count > 0 {
                info!("Loaded {} parameters from SD card", count);
            } else {
                info!("No PARAMS.TXT found, using parameter defaults");
            }
            (cfg, count)
        }
        Err(e) => {
            warn!("SD card load failed, using defaults: {:?}", e);
            (AppConfig::default(), 0)
        }
    };

    let imu_interval_ms = hz_to_period_ms(config.imu_hz, 1, 1000);
    let mag_interval_ms = hz_to_period_ms(config.mag_hz, 10, 1000);
    let baro_interval_ms = hz_to_period_ms(config.baro_hz, 2, 1000);
    let gps_interval_ms = hz_to_period_ms(config.gps_hz, 20, 2000);

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

    // ----- USB-CDC (primary configuration interface) -----
    let usb_driver = UsbDriver::new(p.USB, UsbIrqs);
    let mut usb_config = UsbConfig::new(0xC0DE, 0x4001);
    usb_config.manufacturer = Some("MARV-FC");
    usb_config.product = Some("Flight Computer");
    usb_config.serial_number = Some("FC001");
    usb_config.max_packet_size_0 = 64;
    usb_config.max_power = 100;

    let cdc_state = USB_CDC_STATE.init(CdcAcmState::new());
    let cfg_desc = USB_CONFIG_DESCRIPTOR.init([0u8; 256]);
    let bos_desc = USB_BOS_DESCRIPTOR.init([0u8; 256]);
    let msos_desc = USB_MSOS_DESCRIPTOR.init([0u8; 256]);
    let control_buf = USB_CONTROL_BUF.init([0u8; 64]);

    let mut usb_builder = Builder::new(usb_driver, usb_config, cfg_desc, bos_desc, msos_desc, control_buf);
    let cdc = CdcAcmClass::new(&mut usb_builder, cdc_state, 64);
    let max_packet = cdc.max_packet_size() as usize;
    let usb_dev = usb_builder.build();
    let usb_cdc = UsbCdc {
        class: cdc,
        max_packet,
    };

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
                debug!("  I2C0 device @ 0x{:02X}", addr);
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
                debug!("  I2C1 device @ 0x{:02X}", addr);
            }
        }
    }

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
    info!("MAVLink endpoint: sys_id={} comp_id={}", mav_cfg.sys_id, mav_cfg.comp_id);
    info!("LED color: R={} G={} B={}", led_r, led_g, led_b);

    // Start USB device task
    spawner.spawn(usb_device_task(usb_dev)).unwrap();

    // Spawn sensor tasks
    spawner.spawn(imu_task(imu_bmi088, imu_interval_ms)).unwrap();
    spawner.spawn(mag_task(i2c_for_bmm, mag_interval_ms)).unwrap();
    spawner.spawn(baro_task(i2c_for_bmp, baro_interval_ms)).unwrap();
    spawner.spawn(gps_task(i2c_for_gps, gps_interval_ms)).unwrap();
    let led_color = RGB8 { r: led_r, g: led_g, b: led_b };
    spawner.spawn(logger_task(ws2812, led_color)).unwrap();
    spawner
        .spawn(usb_mavlink_task(
            usb_cdc,
            mav_cfg,
            params,
            fc_state,
            i2c0_mutex,
            i2c1_mutex,
        ))
        .unwrap();
    spawner
        .spawn(uart_mavlink_task(uart_fc_radio, mav_cfg, params, fc_state))
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
    >,
    interval_ms: u32,
) {
    let mut delay = EmbassyDelay;
    let mut sink = ImuSink;
    run_bmi088_task(&mut imu, &mut delay, &mut sink, interval_ms).await;
}

#[embassy_executor::task]
async fn mag_task(i2c_dev: SharedI2c<'static>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    info!("BMM350: starting task on I2C1 @0x{:02X}", BMM350_ADDR);
    let mut mag = Bmm350::new(i2c_dev, BMM350_ADDR);
    mag.set_debug(false);
    let mut sink = MagSink;
    run_bmm350_task(&mut mag, &mut delay, &mut sink, interval_ms).await;
}

#[embassy_executor::task]
async fn baro_task(i2c_dev: SharedI2c0<'static>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    info!("BMP390: starting task on I2C0 @0x{:02X}", BMP3X_ADDR_SDO_HIGH);
    let mut bmp = Bmp390::new(i2c_dev, BMP3X_ADDR_SDO_HIGH);
    bmp.set_debug(false);
    let mut sink = BaroSink;
    run_bmp390_task(&mut bmp, &mut delay, &mut sink, interval_ms).await;
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
async fn logger_task(
    mut led: PioWs2812<'static, embassy_rp::peripherals::PIO0, 0, 1>,
    base_color: RGB8,
) {
    let mut n: u32 = 0;
    let mut colors = [RGB8 { r: 0, g: 0, b: 0 }; 1];
    let mut led_on = false;
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

        // Simple heartbeat on addressable LED: brief blink at ~2 Hz using configured color
        let pulse = (n & 0xFF) < 50;
        if pulse != led_on {
            led_on = pulse;
            colors[0] = if led_on { base_color } else { RGB8 { r: 0, g: 0, b: 0 } };
            led.write(&colors).await;
        }
        Timer::after_micros(1000).await;
        n = n.wrapping_add(1);
    }
}

// USB device task - runs the USB stack
#[embassy_executor::task]
async fn usb_device_task(
    mut device: UsbDevice<'static, UsbDriver<'static, embassy_rp::peripherals::USB>>,
) {
    device.run().await;
}

// USB-CDC MAVLink task with parameter protocol support
#[embassy_executor::task]
async fn usb_mavlink_task(
    mut usb: UsbCdc<'static>,
    cfg: MavEndpointConfig,
    params: &'static Mutex<RawMutex, ParamRegistry>,
    fc_state: &'static Mutex<RawMutex, FcState>,
    i2c0: &'static Mutex<RawMutex, I2c0Type>,
    i2c1: &'static Mutex<RawMutex, I2c1Type>,
) {
    info!("USB-CDC: Starting bidirectional MAVLink task");
    let source = StateTelemetrySource;
    let hb_cfg = HeartbeatConfig::default();

    let mut seq: u8 = 0;
    let mut tx_buf = [0u8; MAVLINK_MAX_FRAME];
    let mut rx_buf = [0u8; MAVLINK_MAX_FRAME];

    // Per-link scheduler: USB typically fast (default TEL_RATE_HZ=50), heartbeat at 1Hz.
    let now_ms = Instant::now().as_millis() as u32;
    let mut sched = LinkScheduler::new(now_ms, 1, 50);

    loop {
        info!("USB: Waiting for host connection...");
        usb.wait_connection().await;
        info!("USB: Host connected");

        // Apply USB-connected policy and emit state + ready text if enabled.
        {
            let p = params.lock().await;
            let statustext_en = p.bool(ParamId::StatustextEn);
            let policy = fc_policy_from_params(&p);
            let mut link_failed = false;

            {
                let mut state = fc_state.lock().await;
                if let Some(snapshot) = state.set_usb_connected(true, policy) {
                    if statustext_en {
                        let status = build_statustext(snapshot.status_text());
                        let frame = build_statustext_frame(cfg, seq, status);
                        seq = seq.wrapping_add(1);
                        if let Ok(len) = frame.serialize(&mut tx_buf) {
                            if let Err(e) = usb.write(&tx_buf[..len]).await {
                                if matches!(e, EndpointError::Disabled) {
                                    link_failed = true;
                                }
                            }
                        }
                    }
                }
            }

            if statustext_en {
                let status = build_statustext("MARV-FC ready");
                let frame = build_statustext_frame(cfg, seq, status);
                seq = seq.wrapping_add(1);
                if let Ok(len) = frame.serialize(&mut tx_buf) {
                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                        if matches!(e, EndpointError::Disabled) {
                            link_failed = true;
                        }
                    }
                }
            }

            drop(p);

            if link_failed {
                let mut state = fc_state.lock().await;
                let _ = state.set_usb_connected(false, policy);
                continue;
            }
        }

        loop {
            let recv_result = embassy_futures::select::select(
                usb.read_packet(&mut rx_buf),
                Timer::after_millis(10),
            )
            .await;

            let mut link_alive = true;

            match recv_result {
                embassy_futures::select::Either::First(Ok(n)) if n > 0 => {
                    if let Ok(frame) = unsafe { Frame::<V2>::deserialize(&rx_buf[..n]) } {
                        let mut p = params.lock().await;
                        let policy = fc_policy_from_params(&p);
                        let statustext_en = p.bool(ParamId::StatustextEn);
                        let mut state = fc_state.lock().await;
                        let (result, params_changed) = dispatch_mavlink_message(
                            frame,
                            cfg,
                            seq,
                            &mut p,
                            &mut state,
                            policy,
                            statustext_en,
                        );
                        drop(state);

                        match result {
                            MessageHandlerResult::SendFrame(reply) => {
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::SendFrames(frames) => {
                                for frame in frames.into_iter() {
                                    seq = seq.wrapping_add(1);
                                    if let Ok(len) = frame.serialize(&mut tx_buf) {
                                        if let Err(e) = usb.write(&tx_buf[..len]).await {
                                            if matches!(e, EndpointError::Disabled) {
                                                link_alive = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::SendAllParams => {
                                info!("MAVLink: PARAM_REQUEST_LIST - sending {} params", p.count());
                                for idx in 0..p.count() {
                                    if let Some(reply) = build_param_value_frame(cfg, seq, &p, idx) {
                                        seq = seq.wrapping_add(1);
                                        if let Ok(len) = reply.serialize(&mut tx_buf) {
                                            if let Err(e) = usb.write(&tx_buf[..len]).await {
                                                if matches!(e, EndpointError::Disabled) {
                                                    link_alive = false;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                info!("MAVLink: PARAM_REQUEST_LIST complete");
                            }
                            MessageHandlerResult::NoResponse | MessageHandlerResult::Unhandled => {}
                            MessageHandlerResult::DeviceOpRead(req) => {
                                // Mission Planner uses DEVICE_OP_READ for I2C/SPI tooling, including the built-in I2C scan UI.
                                // Implement I2C-only for now.
                                const OP_OK: u8 = 0;
                                const OP_BAD_BUS: u8 = 1;
                                const OP_BAD_DEV: u8 = 2;
                                const OP_BAD_RESPONSE: u8 = 4;
                                const BUS_I2C: u8 = 0;

                                let mut data = [0u8; 128];
                                let mut data_len: usize = 0;

                                let result_code = if req.bustype != BUS_I2C {
                                    OP_BAD_BUS
                                } else {
                                    let addr = req.address;
                                    if req.count == 0 {
                                        // Probe: use a 1-byte dummy write (matches our i2cscanner behavior).
                                        let probe: [u8; 1] = [0x00];
                                        let res = match req.bus {
                                            0 => {
                                                let mut bus = SharedI2c0 { bus: i2c0 };
                                                bus.write(addr, &probe).await
                                            }
                                            1 => {
                                                let mut bus = SharedI2c { bus: i2c1 };
                                                bus.write(addr, &probe).await
                                            }
                                            _ => return,
                                        };

                                        match res {
                                            Ok(()) => OP_OK,
                                            Err(_) => OP_BAD_DEV,
                                        }
                                    } else {
                                        let n = core::cmp::min(req.count as usize, data.len());
                                        let reg = [req.regstart];
                                        let res = match req.bus {
                                            0 => {
                                                let mut bus = SharedI2c0 { bus: i2c0 };
                                                bus.write_read(addr, &reg, &mut data[..n]).await
                                            }
                                            1 => {
                                                let mut bus = SharedI2c { bus: i2c1 };
                                                bus.write_read(addr, &reg, &mut data[..n]).await
                                            }
                                            _ => return,
                                        };

                                        match res {
                                            Ok(()) => {
                                                data_len = n;
                                                OP_OK
                                            }
                                            Err(_) => OP_BAD_RESPONSE,
                                        }
                                    }
                                };

                                let reply = build_device_op_read_reply_frame(
                                    cfg,
                                    seq,
                                    req.request_id,
                                    result_code,
                                    req.regstart,
                                    &data[..data_len],
                                );
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::DeviceOpWrite(req) => {
                                // Minimal I2C write support for DEVICE_OP_WRITE (used by some MP tooling).
                                const OP_OK: u8 = 0;
                                const OP_BAD_BUS: u8 = 1;
                                const OP_BAD_RESPONSE: u8 = 4;
                                const BUS_I2C: u8 = 0;

                                let result_code = if req.bustype != BUS_I2C {
                                    OP_BAD_BUS
                                } else {
                                    let addr = req.address;
                                    let n = core::cmp::min(req.count as usize, 128);
                                    let mut payload = [0u8; 129];
                                    payload[0] = req.regstart;
                                    payload[1..1 + n].copy_from_slice(&req.data[..n]);

                                    let res = match req.bus {
                                        0 => {
                                            let mut bus = SharedI2c0 { bus: i2c0 };
                                            bus.write(addr, &payload[..1 + n]).await
                                        }
                                        1 => {
                                            let mut bus = SharedI2c { bus: i2c1 };
                                            bus.write(addr, &payload[..1 + n]).await
                                        }
                                        _ => return,
                                    };

                                    match res {
                                        Ok(()) => OP_OK,
                                        Err(_) => OP_BAD_RESPONSE,
                                    }
                                };

                                let reply =
                                    build_device_op_write_reply_frame(cfg, seq, req.request_id, result_code);
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                        }
                        drop(p);

                        if params_changed {
                            PARAMS_DIRTY.store(true, Ordering::Relaxed);
                            PARAMS_LAST_MODIFIED
                                .store(Instant::now().as_millis() as u32, Ordering::Relaxed);
                        }
                    }
                }
                embassy_futures::select::Either::First(Ok(_)) => {
                    // Received 0 bytes or empty packet - ignore
                }
                embassy_futures::select::Either::First(Err(e)) => {
                    if matches!(e, EndpointError::Disabled) {
                        link_alive = false;
                    } else {
                        debug!("USB: Read error");
                    }
                }
                embassy_futures::select::Either::Second(_) => {
                    // Timeout - no incoming data
                }
            }

            if !link_alive {
                break;
            }

            // Scheduler-driven periodic sends (telemetry + heartbeat).
            let now_ms = Instant::now().as_millis() as u32;
            {
                let p = params.lock().await;
                let telem_rate_hz = p.u32(ParamId::TelemRateHz).min(1000);
                let hb_en = p.bool(ParamId::HeartbeatEn);
                let hb_hz = fc_heartbeat_hz(&p);

                sched.set_telemetry_hz(now_ms, telem_rate_hz);
                sched.set_heartbeat_enabled(hb_en);
                sched.set_heartbeat_hz(now_ms, hb_hz);
            }

            let decision = sched.poll(now_ms);

            if decision.send_telemetry {
                let now_us = (now_ms as u64) * 1000;
                let bundle = source.bundle(now_ms, now_us).await;

                let frames = [
                    build_frame_from_msg(cfg, seq.wrapping_add(0), &bundle.sys_status),
                    build_frame_from_msg(cfg, seq.wrapping_add(1), &bundle.raw_imu),
                    build_frame_from_msg(cfg, seq.wrapping_add(2), &bundle.scaled_pressure),
                    build_frame_from_msg(cfg, seq.wrapping_add(3), &bundle.gps_raw_int),
                    build_frame_from_msg(cfg, seq.wrapping_add(4), &bundle.attitude),
                ];

                for frame in frames.iter() {
                    seq = seq.wrapping_add(1);
                    if let Ok(len) = frame.serialize(&mut tx_buf) {
                        if let Err(e) = usb.write(&tx_buf[..len]).await {
                            if matches!(e, EndpointError::Disabled) {
                                link_alive = false;
                                break;
                            }
                        }
                    }
                }
            }

            if decision.send_heartbeat {
                let snapshot = {
                    let state = fc_state.lock().await;
                    state.snapshot()
                };
                let frame = build_heartbeat_frame(cfg, seq, &snapshot, hb_cfg);
                seq = seq.wrapping_add(1);
                if let Ok(len) = frame.serialize(&mut tx_buf) {
                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                        if matches!(e, EndpointError::Disabled) {
                            link_alive = false;
                        }
                    }
                }
            }
            
            // Debounced parameter save: wait 5 seconds after last modification
            if PARAMS_DIRTY.load(Ordering::Relaxed) {
                let last_mod = PARAMS_LAST_MODIFIED.load(Ordering::Relaxed);
                let now_ms = Instant::now().as_millis() as u32;
                
                // If 5 seconds have passed since last modification, save to SD
                if now_ms.wrapping_sub(last_mod) >= 5000 {
                    info!("Saving parameters to SD card...");
                    
                    // Lock params and save
                    let p = params.lock().await;
                    match save_params_to_sd(&p) {
                        Ok(()) => {
                            info!("Parameters saved successfully");
                            PARAMS_DIRTY.store(false, Ordering::Relaxed);
                        }
                        Err(e) => {
                            warn!("Failed to save parameters: {}", e);
                            // Keep dirty flag set to retry later
                        }
                    }
                    drop(p);
                }
            }

            if !link_alive {
                break;
            }
        }

        // Mark USB disconnected so policy can drop back to IDLE.
        {
            let p = params.lock().await;
            let policy = fc_policy_from_params(&p);
            let mut state = fc_state.lock().await;
            let _ = state.set_usb_connected(false, policy);
        }
    }
}

// UART MAVLink task (FC <-> radio):
// - RX/command handling has priority.
// - Telemetry is sent only when link is idle.
#[embassy_executor::task]
async fn uart_mavlink_task(
    mut uart: RpUart<'static>,
    cfg: MavEndpointConfig,
    params: &'static Mutex<RawMutex, ParamRegistry>,
    fc_state: &'static Mutex<RawMutex, FcState>,
) {
    info!("UART: Starting bidirectional MAVLink task for radio");
    let source = StateTelemetrySource;
    let hb_cfg = HeartbeatConfig::default();
    let mut seq: u8 = 0;
    let mut tx_buf = [0u8; MAVLINK_MAX_FRAME];
    let mut rx_scratch = [0u8; MAVLINK_MAX_FRAME];

    let now_ms = Instant::now().as_millis() as u32;
    let mut sched = LinkScheduler::new(now_ms, 1, 10);
    let mut last_las_tx_ms: u32 = now_ms.wrapping_sub(1_000);

    // Last link MAC config we pushed to the Radio/GS side.
    let mut last_link_cfg: Option<LinkMacConfig> = None;
    let mut last_link_cfg_tx_ms: u32 = now_ms;

    loop {
        let now_ms = Instant::now().as_millis() as u32;
        let (
            fast_en,
            fast_hz,
            fast_max_bytes,
            norm_en,
            norm_hz,
            norm_qmax,
            tel_qos,
            hb_en,
            hb_hz,
            las_max_age_ms,
            link_cfg,
        ) = {
            let p = params.lock().await;
            let fast_en = p.bool(ParamId::TelFastEn);
            let fast_hz = p.u32(ParamId::TelFastHz).min(1000);
            let fast_max_bytes = p
                .u32(ParamId::TelFastMaxB)
                .min(MAVLINK_MAX_FRAME as u32)
                .min(common::coms::transport::lora::mac::INNER_MTU as u32) as usize;
            let norm_en = p.bool(ParamId::TelNormEn);
            let norm_hz = p.u32(ParamId::TelNormMaxHz).min(1000);
            let norm_qmax = p.u32(ParamId::TelNormQmax).min(16) as usize;
            let tel_qos = p.u32(ParamId::TelQos).min(255) as u8;
            let hb_en = p.bool(ParamId::HeartbeatEn);
            let hb_hz = fc_heartbeat_hz(&p);
            let las_max_age_ms = p.u32(ParamId::LasMaxAgeMs).min(u16::MAX as u32);
            let link_cfg = LinkMacConfig::from_params(&p);
            (
                fast_en,
                fast_hz,
                fast_max_bytes,
                norm_en,
                norm_hz,
                norm_qmax,
                tel_qos,
                hb_en,
                hb_hz,
                las_max_age_ms,
                link_cfg,
            )
        };

        sched.set_heartbeat_enabled(hb_en);
        sched.set_heartbeat_hz(now_ms, hb_hz);
        sched.set_fast_enabled(fast_en);
        sched.set_fast_hz(now_ms, fast_hz);
        sched.set_telemetry_hz(now_ms, if norm_en { norm_hz } else { 0 });
        let las_interval_ms = (las_max_age_ms / 2).max(100);

        // Give RX priority. If we see no RX traffic for a short window,
        // we'll use the idle time to send telemetry.
        let idle_timeout_ms = if fast_en && fast_hz != 0 {
            hz_to_period_ms(fast_hz.min(u16::MAX as u32) as u16, 1, 30)
        } else {
            30
        };
        let rx_fut = mavlink::recv_frame_over_uart(&mut uart, &mut rx_scratch);
        let idle_fut = Timer::after_millis(u64::from(idle_timeout_ms));
        let mut rx_activity = false;

        match select::select(rx_fut, idle_fut).await {
            select::Either::First(res) => {
                rx_activity = true;
                let frame = match res {
                    Ok(f) => f,
                    Err(e) => {
                        warn!("UART: RX error: {:?}", e);
                        continue;
                    }
                };

                // Handle command immediately.
                let mut p = params.lock().await;
                let policy = fc_policy_from_params(&p);
                let statustext_en = p.bool(ParamId::StatustextEn);
                let mut state = fc_state.lock().await;
                let (result, params_changed) = dispatch_mavlink_message(
                    frame,
                    cfg,
                    seq,
                    &mut p,
                    &mut state,
                    policy,
                    statustext_en,
                );
                drop(state);

                match result {
                    MessageHandlerResult::SendFrame(reply) => {
                        seq = seq.wrapping_add(1);
                        let _ = mavlink::send_frame_over_uart(&mut uart, &reply, &mut tx_buf).await;
                    }
                    MessageHandlerResult::SendFrames(frames) => {
                        for frame in frames.into_iter() {
                            seq = seq.wrapping_add(1);
                            let _ = mavlink::send_frame_over_uart(&mut uart, &frame, &mut tx_buf).await;
                        }
                    }
                    MessageHandlerResult::SendAllParams => {
                        info!("UART: PARAM_REQUEST_LIST - sending {} params", p.count());
                        for idx in 0..p.count() {
                            if let Some(reply) = build_param_value_frame(cfg, seq, &p, idx) {
                                seq = seq.wrapping_add(1);
                                let _ = mavlink::send_frame_over_uart(&mut uart, &reply, &mut tx_buf).await;
                            }
                        }
                        info!("UART: PARAM_REQUEST_LIST complete");
                    }
                    MessageHandlerResult::NoResponse | MessageHandlerResult::Unhandled => {}
                    MessageHandlerResult::DeviceOpRead(_) | MessageHandlerResult::DeviceOpWrite(_) => {
                        // Device ops are currently served only over USB (Mission Planner).
                    }
                }
                drop(p);

                if params_changed {
                    PARAMS_DIRTY.store(true, Ordering::Relaxed);
                    PARAMS_LAST_MODIFIED.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
                }
            }
            select::Either::Second(_) => {
                // Idle window elapsed: ok to send telemetry if due.
            }
        }

        let now_ms = Instant::now().as_millis() as u32;
        let decision = sched.poll(now_ms);

        let allow_fast_during_rx = tel_qos == 0;
        if decision.send_fast && fast_en && (allow_fast_during_rx || !rx_activity) {
            if fast_max_bytes != 0 {
                let now_us = (now_ms as u64) * 1000;
                let sample = source.latest().await;
                if let Some(frame) = build_telemetry_frame(cfg, seq, now_us, &sample) {
                    if frame.size() <= fast_max_bytes {
                        seq = seq.wrapping_add(1);
                        let _ = mavlink::send_frame_over_uart(&mut uart, &frame, &mut tx_buf).await;
                    }
                }
            }
        }

        if decision.send_telemetry && norm_en && !rx_activity {
            let now_us = (now_ms as u64) * 1000;
            let bundle = source.bundle(now_ms, now_us).await;

            let frames = [
                build_frame_from_msg(cfg, seq.wrapping_add(0), &bundle.sys_status),
                build_frame_from_msg(cfg, seq.wrapping_add(1), &bundle.raw_imu),
                build_frame_from_msg(cfg, seq.wrapping_add(2), &bundle.scaled_pressure),
                build_frame_from_msg(cfg, seq.wrapping_add(3), &bundle.gps_raw_int),
                build_frame_from_msg(cfg, seq.wrapping_add(4), &bundle.attitude),
            ];

            let max_frames = norm_qmax.min(frames.len());
            for frame in frames.iter().take(max_frames) {
                seq = seq.wrapping_add(1);
                // Send to radio via UART - best effort, ignore errors.
                let _ = mavlink::send_frame_over_uart(&mut uart, frame, &mut tx_buf).await;
            }
        }

        if decision.send_heartbeat {
            let snapshot = {
                let state = fc_state.lock().await;
                state.snapshot()
            };
            let frame = build_heartbeat_frame(cfg, seq, &snapshot, hb_cfg);
            seq = seq.wrapping_add(1);
            let _ = mavlink::send_frame_over_uart(&mut uart, &frame, &mut tx_buf).await;
        }

        if now_ms.wrapping_sub(last_las_tx_ms) >= las_interval_ms {
            let snapshot = {
                let state = fc_state.lock().await;
                state.snapshot()
            };
            let las_state = if snapshot.armed {
                LAS_STATE_ARMED
            } else {
                LAS_STATE_DISARMED
            };
            let max_age_ms = las_max_age_ms.max(1) as u16;
            let frame = build_link_authority_frame(cfg, seq, 0, 0, las_state, max_age_ms);
            seq = seq.wrapping_add(1);
            let _ = mavlink::send_frame_over_uart(&mut uart, &frame, &mut tx_buf).await;
            last_las_tx_ms = now_ms;
        }

        // Propagate link MAC config to the Radio/GS link without reflashing.
        // We send on change (and also once at startup), at a low rate.
        let now_ms = Instant::now().as_millis() as u32;
        let should_send = match last_link_cfg {
            None => true,
            Some(prev) => prev.tick_hz != link_cfg.tick_hz
                || prev.fast_max_bytes != link_cfg.fast_max_bytes
                || prev.slot_mode != link_cfg.slot_mode,
        };

        if should_send && now_ms.wrapping_sub(last_link_cfg_tx_ms) >= 500 {
            // Broadcast to any listener; Radio/GS will intercept and apply.
            let frame = build_link_mac_config_command_frame(cfg, seq, 0, 0, link_cfg);
            seq = seq.wrapping_add(1);
            let _ = mavlink::send_frame_over_uart(&mut uart, &frame, &mut tx_buf).await;
            last_link_cfg = Some(link_cfg);
            last_link_cfg_tx_ms = now_ms;
        }
    }
}

#[embassy_executor::task]
async fn gps_task(i2c_dev: SharedI2c<'static>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    info!("NEO-M9N: starting task on I2C1 @0x{:02X}", UBLOX_I2C_ADDR);
    let mut gps = NeoM9n::new(i2c_dev, UBLOX_I2C_ADDR);
    gps.set_debug(false);
    let mut sink = GpsSink;
    run_neom9n_task(&mut gps, &mut delay, &mut sink, interval_ms).await;
}
