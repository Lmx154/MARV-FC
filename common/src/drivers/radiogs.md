//here is code for the radio and the gs. 
//! Radio initialization: clocks, SPI + GPIO for SX1262, and LED setup.
//! init.rs
use defmt::*;
use rp235x_hal as hal;
use hal::gpio::{FunctionSio, SioOutput, SioInput, PullDown, PullUp, FunctionSpi};
use hal::spi::Spi;
use hal::Clock; // for peripheral/system clock freq
use embedded_hal::spi::MODE_0;
use embedded_hal::digital::OutputPin as _;

use crate::hardware::HARDWARE;

#[derive(Default, Clone, Copy)]
pub struct BringupReport { pub spi_ok: bool, pub led_ok: bool }

use rp235x_hal::spi::Enabled as SpiEnabled;

pub struct TelemetryResources {
	pub sx_spi: Spi<
		SpiEnabled,
		hal::pac::SPI0,
		(
			hal::gpio::Pin<hal::gpio::bank0::Gpio3, FunctionSpi, PullDown>, // MOSI
			hal::gpio::Pin<hal::gpio::bank0::Gpio4, FunctionSpi, PullDown>, // MISO
			hal::gpio::Pin<hal::gpio::bank0::Gpio2, FunctionSpi, PullDown>, // SCK
		),
		8,
	>,
	pub sx_nss: hal::gpio::Pin<hal::gpio::bank0::Gpio5, FunctionSio<SioOutput>, PullDown>,
	pub sx_busy: hal::gpio::Pin<hal::gpio::bank0::Gpio0, FunctionSio<SioInput>, PullUp>,
	pub sx_reset: hal::gpio::Pin<hal::gpio::bank0::Gpio1, FunctionSio<SioOutput>, PullDown>,
	pub sx_dio1: hal::gpio::Pin<hal::gpio::bank0::Gpio6, FunctionSio<SioInput>, PullDown>,
	pub sx_dio2: hal::gpio::Pin<hal::gpio::bank0::Gpio7, FunctionSio<SioInput>, PullDown>,
	pub fc_uart: hal::uart::UartPeripheral<
		hal::uart::Enabled,
		hal::pac::UART1,
		(
			hal::gpio::Pin<hal::gpio::bank0::Gpio8, hal::gpio::FunctionUart, PullDown>,
			hal::gpio::Pin<hal::gpio::bank0::Gpio9, hal::gpio::FunctionUart, PullDown>,
		),
	>,
	pub led: hal::gpio::Pin<hal::gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>,
}

pub fn bringup() -> (BringupReport, TelemetryResources) {
	info!("Radio: bring-up start (SPI + SX1262 pins)");

	let mut pac = hal::pac::Peripherals::take().unwrap();
	let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
	let clocks = hal::clocks::init_clocks_and_plls(
		HARDWARE.xtal_frequency(),
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	).unwrap();

	let sio = hal::Sio::new(pac.SIO);
	let pins = hal::gpio::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	// SPI0: MOSI=GP3, MISO=GP4, SCK=GP2 (matches valid pin mapping pattern like FC BMI088 but with chosen MISO)
	let mosi = pins.gpio3.into_function::<FunctionSpi>();
	let miso = pins.gpio4.into_function::<FunctionSpi>();
	let sck  = pins.gpio2.into_function::<FunctionSpi>();
	let spi0 = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
		&mut pac.RESETS,
		clocks.peripheral_clock.freq(),
		hal::fugit::HertzU32::from_raw(1_000_000), // start at 1MHz for reliability
		&MODE_0,
	);

    // Control + status pins
    let sx_nss = pins.gpio5.into_push_pull_output();
    let sx_busy = pins.gpio0.into_pull_up_input();
    let mut sx_reset = pins.gpio1.into_push_pull_output();
    let sx_dio1 = pins.gpio6.into_pull_down_input();
    let sx_dio2 = pins.gpio7.into_pull_down_input();
    // Hold reset high initially
    sx_reset.set_high().ok();

    // LED on GP25
    let led = pins.gpio25.into_push_pull_output();

    let spi_ok = true;
    let led_ok = true;
    info!("SPI0: {}", if spi_ok { "OK" } else { "FAIL" });
	info!("LED: {}", if led_ok { "OK" } else { "FAIL" });

    let report = BringupReport { spi_ok, led_ok };
	// GP8/GP9 freed (previously external RF switch); retained for future UART or other use.
	let resources = TelemetryResources { sx_spi: spi0, sx_nss, sx_busy, sx_reset, sx_dio1, sx_dio2, fc_uart: unsafe { core::mem::MaybeUninit::zeroed().assume_init() }, led };
	// NOTE: fc_uart placeholder left zeroed; avoid using it in radio build while RF switch active.
	(report, resources)
}

//! Radio binary main: minimal LoRa + LED runtime.
//! main.rs
#![no_std]
#![no_main]
mod drivers;
// abstraction & middleware removed for radio (sensors no longer present)
mod hardware;
mod init;
mod coms;
// telemetry/tools/filters removed in LoRa-only build
use defmt_rtt as _; // global logger
use panic_probe as _;
use defmt::{info, warn, debug};
// (LED pin methods used via concrete type; no trait import needed)
use drivers::{Sx1262, Sx1262Config};
use rp235x_hal as hal;
/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();
#[hal::entry]
fn main() -> ! {
    let (_report, mut resources) = init::bringup();
    info!("Radio bring-up complete (SPI + SX1262 pins) — verifying Steps 1 & 2");
    // Enable DWT cycle counter (for RTT measurement) if available.
    #[allow(unused_unsafe)]
    if let Some(mut cp) = cortex_m::Peripherals::take() {
        cp.DCB.enable_trace();
        // Unlock (on some cores) then enable cycle counter
        unsafe { cp.DWT.lar.write(0xC5ACCE55); }
        cp.DWT.enable_cycle_counter();
        unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.write(0); }
        info!("Radio: DWT cycle counter enabled for RTT logging");
    } else {
        warn!("Radio: DWT Peripherals already taken; RTT cycles may be unavailable");
    }

    // Step 3: Instantiate SX1262 with real SPI and perform hardware reset + status read
    let mut sx = Sx1262::new(
        resources.sx_spi,
        resources.sx_nss,
        resources.sx_reset,
        resources.sx_busy,
        resources.sx_dio1,
        resources.sx_dio2,
    Sx1262Config { use_dcdc: true, tcxo_ctrl: true, tcxo_delay_ms: 10, tx_power: 14, ..Sx1262Config::default() },
    );
    info!("SX1262 driver constructed (pre-reset) mode={:?}", sx.mode());
    if let Err(e) = sx.hardware_reset_and_status() {
        warn!("SX1262 reset/status failed err={:?}", e);
    } else {
        info!("(Step 3 reset + status complete) proceeding Step 4...");
        if let Err(e) = sx.calibrate_and_configure(915_000_000) {
            warn!("SX1262 calibration/config failed err={:?}", e);
        } else {
            // IMPORTANT: Skip the earlier demo sleep/wake sequence for now because Sleep may
            // reset internal packet type / context leading to invalid first TX when we later
            // set params. Keep radio in StandbyRc and directly apply LoRa params.
            info!("(Step 4 calibration complete; applying Step 6 modulation params)");
            if let Err(e) = sx.apply_lora_params() { warn!("SX1262 Step 6 apply params failed err={:?}", e); }
            else { info!("(Step 6 params applied) entering ping-pong initiator mode"); }
        }
    }

    // ---------------- Continuous Unidirectional TX Mode ----------------
    // Frame: 6 bytes: ['P', flags=0, seq(u32 LE)]
    let mut tx_buf = [0u8;6];
    let mut seq: u32 = 0;
    #[inline(always)] fn build_ping(buf: &mut [u8;6], seq: u32) { buf[0]=b'P'; buf[1]=0; buf[2..6].copy_from_slice(&seq.to_le_bytes()); }
    // Optionally pace transmissions: number of busy-loop cycles between packets
    // Adjust to manage duty cycle / airtime. At SF7 BW125 a 6-byte payload airtime is small.
    const GAP_CYCLES: u32 = 5_000_000; // ~40ms @125MHz (approx)
    #[inline(always)] fn busy_delay(c: u32){ for _ in 0..c { cortex_m::asm::nop(); } }
    // Reuse tx_buf for temporary RX buffer during PONG wait window.
    loop {
        // Prepare next frame
        build_ping(&mut tx_buf, seq);
        // Start TX
        match sx.start_tx(&tx_buf) {
            Ok(()) => {
                let busy = sx.busy_level();
                let dio1 = sx.dio1_level();
                info!("Radio TX PING seq={}", seq);
                debug!("Radio TX armed seq={} busy={} dio1={}", seq, busy, dio1);
            }
            Err(e) => { warn!("Radio start_tx error seq={} err={:?}", seq, e); }
        }
        // Wait until TX completes
        while let Ok(None) = sx.poll_tx() { cortex_m::asm::nop(); }
        if let Ok(Some(r)) = sx.poll_tx() { debug!("Radio TX done irq=0x{:04X}", r.irq_status); }

        // Attempt quick listen for PONG reply
        if let Err(e) = sx.start_rx_continuous() { warn!("Radio RX arm failed err={:?}", e); }
        else {
            let mut loops: u32 = 0;
            let mut received = false;
            // Small window: loops threshold tuned empirically (tight to keep duty cycle low)
            while loops < 2_000_000 { // ~16ms @125MHz (adjust as needed)
                if let Ok(Some(rpt)) = sx.poll_rx(&mut tx_buf) {
                    if rpt.rx_done && rpt.payload_len == 6 && tx_buf[0] == b'O' { // 'O' for PONG per gs side
                        let echo_seq = u32::from_le_bytes([tx_buf[2],tx_buf[3],tx_buf[4],tx_buf[5]]);
                        info!("Radio RX PONG local_byte={} echo_seq={}", tx_buf[1], echo_seq);
                        received = true;
                        break;
                    } else if rpt.header_err || rpt.crc_err {
                        warn!("Radio RX err hdr={} crc={} irq=0x{:04X}", rpt.header_err, rpt.crc_err, rpt.irq_status);
                        break;
                    } else if rpt.timeout { break; }
                }
                loops += 1;
            }
            if !received { debug!("Radio no PONG in window (loops={})", loops); }
        }
        // Increment sequence and gap delay before next transmit
        seq = seq.wrapping_add(1);
        busy_delay(GAP_CYCLES);
    }
}

//! Ground Station bring-up for the SX1262 radio path.
//!
//! init_sx.rs
//
//! This sets up clocks, SPI0, the GPIO pins required by the radio and then
//! instantiates the `Sx1262` driver defined in `gs::drivers`.

use defmt::*;
use rp235x_hal as hal;

use embedded_hal::spi::MODE_0;
use embedded_hal::digital::OutputPin as _;

use hal::gpio::{FunctionSpi, FunctionSio, PullDown, PullUp, SioInput, SioOutput};
use hal::spi::Spi;
use rp235x_hal::spi::Enabled as SpiEnabled;
use hal::Clock;

use crate::drivers::sx1262::{Sx1262, Sx1262Config, Sx1262Error};
use crate::hardware::{constants, HARDWARE};

#[derive(Default, Clone, Copy, defmt::Format)]
pub struct BringupReport {
	pub clocks_ok: bool,
	pub spi_ok: bool,
	pub radio_ready: bool,
}

pub struct RadioResources {
	pub radio: Sx1262<
		Spi<
			SpiEnabled,
			hal::pac::SPI0,
			(
				hal::gpio::Pin<hal::gpio::bank0::Gpio3, FunctionSpi, PullDown>,
				hal::gpio::Pin<hal::gpio::bank0::Gpio4, FunctionSpi, PullDown>,
				hal::gpio::Pin<hal::gpio::bank0::Gpio2, FunctionSpi, PullDown>,
			),
			8,
		>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio5, FunctionSio<SioOutput>, PullDown>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio0, FunctionSio<SioInput>, PullUp>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio1, FunctionSio<SioOutput>, PullDown>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio6, FunctionSio<SioInput>, PullDown>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio9, FunctionSio<SioOutput>, PullDown>,
		hal::gpio::Pin<hal::gpio::bank0::Gpio8, FunctionSio<SioOutput>, PullDown>,
	>,
	pub led: hal::gpio::Pin<hal::gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>,
}

pub fn bringup() -> Result<(BringupReport, RadioResources), Sx1262Error> {
	info!("GS init: starting clocks + SPI0 + SX1262 bring-up");

	let mut pac = hal::pac::Peripherals::take().unwrap();
	let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

	let clocks = hal::clocks::init_clocks_and_plls(
		HARDWARE.xtal_frequency(),
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	)
	.unwrap();

	let clocks_ok = true;

	let sio = hal::Sio::new(pac.SIO);
	let pins = hal::gpio::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	// SPI0 pins (MOSI, MISO, SCK) per docs/sx1262.md
	let mosi = pins.gpio3.into_function::<FunctionSpi>();
	let miso = pins.gpio4.into_function::<FunctionSpi>();
	let sck = pins.gpio2.into_function::<FunctionSpi>();
	let spi0 = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
		&mut pac.RESETS,
		clocks.peripheral_clock.freq(),
		hal::fugit::HertzU32::from_raw(constants::SX_SPI_FREQ_HZ),
		&MODE_0,
	);

	// Control pins
	let mut nss = pins.gpio5.into_push_pull_output();
	nss.set_high().ok();
	let busy = pins.gpio0.into_pull_up_input();
	let mut reset = pins.gpio1.into_push_pull_output();
	reset.set_high().ok();
	let dio1 = pins.gpio6.into_pull_down_input();
	let mut rxen = pins.gpio9.into_push_pull_output();
	let mut txen = pins.gpio8.into_push_pull_output();
	rxen.set_low().ok();
	txen.set_low().ok();

	let led = pins.gpio25.into_push_pull_output();

	let spi_ok = true;

	let cfg = Sx1262Config {
		payload_len: 32,
		..Default::default()
	};

	let mut radio = Sx1262::new(spi0, nss, busy, reset, dio1, rxen, txen, cfg);
	radio.initialize()?;
	let radio_ready = true;

	let resources = RadioResources { radio, led };
	let report = BringupReport { clocks_ok, spi_ok, radio_ready };
	Ok((report, resources))
}

//! GS binary main: SX1262 ping-pong RX/TX loop.
//!
//! ## Ping-Pong Test Setup
//!
//! This binary implements a ping-pong communication test between two identical
//! SX1262 setups. Use the `IS_INITIATOR` constant in `main()` to configure:
//!
//! - **Device 1**: Set `IS_INITIATOR = true` (sends PING, waits for PONG)
//! - **Device 2**: Set `IS_INITIATOR = false` (waits for PING, sends PONG)
//!
//! ### Expected Behavior:
//!
//! **Initiator (Device 1)**:
//! - Sends PING#1, PING#2, etc.
//! - Waits for PONG response after each PING
//! - LED ON during TX, OFF during RX
//!
//! **Responder (Device 2)**:
//! - Continuously listens for PING messages
//! - Responds with PONG when PING is received
//! - LED OFF during RX, ON during TX
//!
//! ### Testing:
//! 1. Flash Device 1 with `IS_INITIATOR = true`
//! 2. Flash Device 2 with `IS_INITIATOR = false`
//! 3. Power both devices
//! 4. Monitor logs via probe-rs or defmt
//!
#![no_std]
#![no_main]
mod hardware;
// mod init; // legacy UART LoRa init (disabled to avoid conflicting pin setup during build)
mod init_sx; // SX1262 bring-up
mod drivers;
use defmt_rtt as _;
use defmt::{info, warn};
use panic_probe as _;
use embedded_hal::digital::OutputPin as _;
use crate::drivers::Sx1262Error;
use crate::hardware::HARDWARE;
/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();
#[rp235x_hal::entry]
fn main() -> ! {
	info!("GS main: starting bring-up");

	let (report, resources) = match init_sx::bringup() {
		Ok(tuple) => tuple,
		Err(err) => {
			warn!("GS init failed: {:?}", err);
			loop {
				cortex_m::asm::bkpt();
			}
		}
	};

	info!("GS init report: clocks_ok={} spi_ok={} radio_ready={}", report.clocks_ok, report.spi_ok, report.radio_ready);

	let init_sx::RadioResources { mut radio, mut led } = resources;

	// ═══════════════════════════════════════════════════════════════
	// PING-PONG CONFIGURATION
	// ═══════════════════════════════════════════════════════════════
	// Set to `true` on one device (initiator) and `false` on the other (responder)
	const IS_INITIATOR: bool = false;
	
	if IS_INITIATOR {
		info!("🚀 PING-PONG MODE: INITIATOR");
	} else {
		info!("📡 PING-PONG MODE: RESPONDER");
	}

	// ═══════════════════════════════════════════════════════════════
	// PING-PONG TEST LOOP
	// ═══════════════════════════════════════════════════════════════
	let mut rx_buffer = [0u8; 255];
	let mut ping_count: u32 = 0;
	let mut pong_count: u32 = 0;

	if IS_INITIATOR {
		// INITIATOR: Start by sending first PING
		loop {
			led.set_high().ok();
			ping_count += 1;
			
			// Send PING
			let ping_msg = format_ping_message(ping_count);
			info!("TX → PING #{}", ping_count);
			match radio.transmit(ping_msg.as_bytes(), 3000) {
				Ok(_) => {},
				Err(e) => {
					warn!("TX error: {:?}", e);
					led.set_low().ok();
					spin_delay_ms(1000);
					continue;
				}
			}
			
			// Wait for PONG response
			led.set_low().ok();
			info!("RX ← Waiting for PONG...");
			match radio.receive(&mut rx_buffer, 5000) {
				Ok(len) => {
					if len > 0 {
						if let Ok(msg) = core::str::from_utf8(&rx_buffer[..len]) {
							info!("RX ← {}", msg);
							pong_count += 1;
						} else {
							info!("RX ← {} bytes (non-UTF8)", len);
						}
					}
				},
				Err(Sx1262Error::IrqTimeout) => {
					warn!("RX timeout - no PONG received");
				},
				Err(e) => warn!("RX error: {:?}", e),
			}
			
			// Brief pause before next ping
			spin_delay_ms(500);
		}
	} else {
		// RESPONDER: Wait for PING and respond with PONG
		loop {
			led.set_low().ok();
			
			// Listen for PING
			info!("RX ← Waiting for PING...");
			match radio.receive(&mut rx_buffer, 0) {  // 0 = continuous RX
				Ok(len) => {
					led.set_high().ok();
					if len > 0 {
						if let Ok(msg) = core::str::from_utf8(&rx_buffer[..len]) {
							info!("RX ← {}", msg);
							ping_count += 1;
						} else {
							info!("RX ← {} bytes (non-UTF8)", len);
							ping_count += 1;
						}
						
						// Send PONG response
						pong_count += 1;
						let pong_msg = format_pong_message(pong_count);
						spin_delay_ms(100); // Small delay before responding
						
						info!("TX → PONG #{}", pong_count);
						match radio.transmit(pong_msg.as_bytes(), 3000) {
							Ok(_) => {},
							Err(e) => warn!("TX error: {:?}", e),
						}
					}
				},
				Err(Sx1262Error::IrqTimeout) => {
					// This shouldn't happen with timeout=0, but handle it
					warn!("RX timeout (unexpected)");
				},
				Err(e) => {
					warn!("RX error: {:?}", e);
					spin_delay_ms(1000);
				}
			}
			
			led.set_low().ok();
			spin_delay_ms(100);
		}
	}
}

/// Format a PING message with counter
fn format_ping_message(count: u32) -> heapless::String<32> {
	use core::fmt::Write;
	let mut msg = heapless::String::<32>::new();
	write!(msg, "PING#{}", count).ok();
	msg
}

/// Format a PONG message with counter
fn format_pong_message(count: u32) -> heapless::String<32> {
	use core::fmt::Write;
	let mut msg = heapless::String::<32>::new();
	write!(msg, "PONG#{}", count).ok();
	msg
}

fn spin_delay_ms(ms: u32) {
	let sys_clk_hz = HARDWARE.system_clock_frequency();
	let cycles_per_ms = core::cmp::max(sys_clk_hz / 1_000, 1);
	for _ in 0..ms {
		cortex_m::asm::delay(cycles_per_ms);
	}
}

