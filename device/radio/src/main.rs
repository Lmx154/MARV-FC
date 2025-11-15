//radio/src/main.rs
#![no_std]
#![no_main]

// Minimal radio firmware entrypoint for RP2350A Pico 2 using SX1262 only.

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output, Input, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Timer, Duration};

use common::drivers::sx1262::{Sx1262, Sx1262Config, RfSwitchMode, RfState, RfSwitch};
use common::utils::delay::DelayMs;

// Simple Embassy-based async delay bridge
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms as u64)).await;
    }
}

// External RF switch using TXEN (GP8) and RXEN (GP9)
struct ExtRfSwitch {
    txen: Output<'static>,
    rxen: Output<'static>,
}
impl RfSwitch for ExtRfSwitch {
    fn set(&mut self, state: RfState) {
        match state {
            RfState::Off => { 
                self.txen.set_low(); 
                self.rxen.set_low();
                info!("RF Switch: OFF (RXEN=L, TXEN=L)");
            }
            RfState::Rx => { 
                self.rxen.set_low();  // RXEN LOW for RX
                self.txen.set_high(); // TXEN HIGH for RX
                info!("RF Switch: RX (RXEN=L, TXEN=H)");
            }
            RfState::Tx => { 
                self.rxen.set_high(); // RXEN HIGH for TX
                self.txen.set_low();  // TXEN LOW for TX
                info!("RF Switch: TX (RXEN=H, TXEN=L)");
            }
        }
    }
}

// On-board Pico LED at GP25 is active HIGH.
struct Led { pin: Output<'static> }
impl Led {
    fn new(pin: Output<'static>) -> Self { Self { pin } }
    fn on(&mut self) { self.pin.set_high(); }
    fn off(&mut self) { self.pin.set_low(); }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MARV-FC Radio"),
    embassy_rp::binary_info::rp_program_description!(
        c"MARV-FC dedicated radio firmware using SX1262 on RP2350A Pico 2"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("MARV-FC Radio: boot (STARTER)");

    // Use on-board Pico LED (GP25)
    let mut led = Led::new(Output::new(p.PIN_25, Level::Low));

    // SPI wiring for SX1262 (assumes same pins as Core1262-HF-like module).
    // Adjust pins to match actual hardware once confirmed.
    // Per docs/hardware.md: SPI0 CLK=GP2, MOSI=GP3, MISO=GP4
    let sck = p.PIN_2;
    let mosi = p.PIN_3;
    let miso = p.PIN_4;

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000; // 10 MHz typical for SX1262

    let spi = Spi::new(
        p.SPI0,
        sck,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    // Control pins per docs/hardware.md:
    // CS/NSS=GP5, RESET=GP1, BUSY=GP0, DIO1=GP6, DIO2=GP7
    let nss = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    // BUSY is driven by the radio; avoid internal pull that could bias the level.
    let busy = Input::new(p.PIN_0, Pull::None);
    let dio1 = Input::new(p.PIN_6, Pull::None);
    let dio2 = Input::new(p.PIN_7, Pull::None);

    // External RF switch pins (TXEN=GP8, RXEN=GP9)
    // Per datasheet: RXEN low + TXEN high = RX mode, RXEN high + TXEN low = TX mode
    let rf_switch = ExtRfSwitch {
        txen: Output::new(p.PIN_8, Level::Low),
        rxen: Output::new(p.PIN_9, Level::Low),
    };

    let cfg = Sx1262Config::default();

    let mut radio = Sx1262::new(
        spi,
        nss,
        reset,
        busy,
        dio1,
        dio2,
        rf_switch,
        RfSwitchMode::External, // Use external TXEN/RXEN control per module datasheet
        cfg,
    );

    let mut delay = EmbassyDelay;

    // LoRa center frequency per hardware: 915 MHz (Waveshare SX1262 HF)
    let center_freq_hz: u32 = 915_000_000;

    match radio.init_lora(&mut delay, center_freq_hz).await {
        Ok(_) => info!("SX1262: init OK at {} Hz", center_freq_hz),
        Err(e) => {
            warn!("SX1262: init failed: {:?}", e);
            loop { Timer::after_secs(1).await; } // halt on error
        }
    }

    // Small delay after init for radio to fully stabilize
    Timer::after_millis(100).await;

    // Ensure both ends use the same LoRa sync word (private=0x1424)
    if let Err(e) = radio.set_lora_sync_word(&mut delay, 0x1424).await {
        warn!("Failed to set sync word: {:?}", e);
    }

    // Verify sync word was set correctly
    if let Ok(sw) = radio.read_registers::<2, _>(&mut delay, 0x0740).await {
        info!("SX1262: SyncWord after override=0x{:02X}{:02X}", sw[0], sw[1]);
    }

    // Ping-pong game: RADIO is the starter, sends first
    const PING_MSG: &[u8] = b"PING";
    const PONG_MSG: &[u8] = b"PONG";
    let mut rx_buf = [0u8; 255];

    info!("Starting ping-pong game as STARTER");

    loop {
        // Send PING
        info!("Sending PING...");
        led.on(); // Brief LED pulse during TX

        match radio.tx_send_blocking(&mut delay, PING_MSG, 5000).await {
            Ok(report) if report.done => {
                info!("PING sent successfully");
            }
            Ok(report) => {
                warn!("PING send failed: timeout={}", report.timeout);
            }
            Err(e) => {
                warn!("PING send error: {:?}", e);
            }
        }

        // Small visual pulse, then immediately arm RX to avoid missing PONG
        Timer::after_millis(50).await;
        led.off();

        // Wait for PONG response
        info!("Waiting for PONG...");
        match radio.start_rx_continuous(&mut delay).await {
            Ok(_) => {}
            Err(e) => {
                warn!("RX start failed: {:?}", e);
                Timer::after_secs(1).await;
                continue;
            }
        }

        // Poll for response (with timeout)
        let mut received = false;
        for _ in 0..100 { // 5 seconds timeout (100 * 50ms)
            Timer::after_millis(50).await;
            match radio.poll_rx(&mut delay, &mut rx_buf).await {
                Ok(Some(report)) => {
                    info!("Radio: RX report IRQ=0x{:04X} done={} timeout={} len={}", 
                        report.irq, report.done, report.timeout, report.len);
                    if report.done && report.len == PONG_MSG.len() as u8 {
                        if &rx_buf[..report.len as usize] == PONG_MSG {
                            info!("PONG received! RSSI={}, SNR/4={}", report.rssi, report.snr_x4);
                            received = true;
                            break;
                        }
                    } else if report.timeout {
                        warn!("RX timeout");
                        break;
                    }
                }
                Ok(None) => {} // No IRQ yet
                Err(e) => warn!("RX poll error: {:?}", e),
            }
        }

        if !received {
            warn!("No PONG received, retrying...");
            Timer::after_secs(1).await;
        }
    }
}
