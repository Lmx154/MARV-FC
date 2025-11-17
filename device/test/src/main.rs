#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::spi::NoDelay;

use sx126x::{
    SX126x,
    conf::Config,
    op::{
        PacketType, CalibParam,
        LoraModParams, LoRaSpreadFactor, LoRaBandWidth, LoraCodingRate,
        LoRaPacketParams, LoRaHeaderType, LoRaCrcType, LoRaInvertIq,
        TxParams, RampTime, PaConfig, DeviceSel,
        RxTxTimeout, IrqMask,
        TcxoVoltage, TcxoDelay,
        StandbyConfig
    }
};

const INITIATOR: bool = true;
const RF_FREQ_HZ: u32 = 915_000_000;
const XTAL_FREQ_HZ: f32 = 32_000_000.0;

// crude blocking delay
fn delay_ms(ms: u32) {
    for _ in 0..(ms * 30_000) {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting SX1262 Exclusive-SPI Blocking Test");

    let p = embassy_rp::init(Default::default());

    // -----------------------------
    // SPI SETUP (Embassy-RP)
    // -----------------------------
    let mut cfg = SpiConfig::default();
    cfg.frequency = 10_000_000;

    let spi = Spi::new_blocking(
        p.SPI0,
        p.PIN_2, // SCK
        p.PIN_3, // MOSI
        p.PIN_4, // MISO
        cfg,
    );

    // CS = GP5
    let cs = Output::new(p.PIN_5, Level::High);

    // Wrap in a blocking SpiDevice compatible with sx126x
    let spi_dev = ExclusiveDevice::new(spi, cs, NoDelay).unwrap();

    // -----------------------------
    // RADIO CONTROL PINS
    // -----------------------------
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new(p.PIN_0, Pull::None);
    let dio1  = Input::new(p.PIN_6, Pull::None);

    // Manual RF switch (txen passed into driver as ANT; no external rxen in this example)
    let txen = Output::new(p.PIN_8, Level::Low);

    // LED (GP25)
    let mut led = Output::new(p.PIN_25, Level::Low);

    // -----------------------------
    // CREATE RADIO
    // -----------------------------
    let mut radio = SX126x::new(spi_dev, (reset, busy, txen, dio1));

    // -----------------------------
    // REQUIRED: TCXO ENABLE BEFORE INIT
    // -----------------------------
    radio.reset().unwrap();
    radio.wait_on_busy().unwrap();

    radio.set_standby(StandbyConfig::StbyRc).unwrap();
    radio.wait_on_busy().unwrap();

    radio
        .set_dio3_as_tcxo_ctrl(TcxoVoltage::Volt1_8, TcxoDelay::from_ms(5))
        .unwrap();
    radio.wait_on_busy().unwrap();

    // -----------------------------
    // CONFIGURE MODEM
    // -----------------------------
    let rf_reg = sx126x::calc_rf_freq(RF_FREQ_HZ as f32, XTAL_FREQ_HZ);

    let mod_params = LoraModParams::default()
        .set_spread_factor(LoRaSpreadFactor::SF9)
        .set_bandwidth(LoRaBandWidth::BW125)
        .set_coding_rate(LoraCodingRate::CR4_5)
        .into();

    let pkt_params = LoRaPacketParams {
        preamble_len: 8,
        header_type: LoRaHeaderType::VarLen,
        payload_len: 32,
        crc_type: LoRaCrcType::CrcOn,
        invert_iq: LoRaInvertIq::Standard,
    }
    .into();

    let conf = Config {
        packet_type: PacketType::LoRa,
        sync_word: 0x1424,
        calib_param: CalibParam::all(),
        mod_params,
        pa_config: PaConfig::default()
            .set_device_sel(DeviceSel::SX1262)
            .set_pa_duty_cycle(0x04)
            .set_hp_max(0x07),
        packet_params: Some(pkt_params),
        tx_params: TxParams::default()
            .set_power_dbm(14)
            .set_ramp_time(RampTime::Ramp40u),
        dio1_irq_mask: IrqMask::all(),
        dio2_irq_mask: IrqMask::none(),
        dio3_irq_mask: IrqMask::none(),
        rf_freq: rf_reg,
        rf_frequency: RF_FREQ_HZ,
        tcxo_opts: Some((TcxoVoltage::Volt1_8, TcxoDelay::from_ms(5))),
    };

    info!("Calling radio.init()");
    radio.init(conf).unwrap();
    radio.wait_on_busy().unwrap();

    // Must be off (manual switch)
    radio.set_dio2_as_rf_switch_ctrl(false).unwrap();

    // Enter RX mode (driver controls ANT pin)
    radio.set_ant_enabled(false).unwrap();

    let mut rxbuf = [0u8; 64];

    info!("Starting ping-pong loop…");
    loop {
        if INITIATOR {
            send(&mut radio, b"PING");
            if recv(&mut radio, &mut rxbuf) == Some(b"PONG") {
                led.set_high();
                delay_ms(150);
                led.set_low();
            }
            delay_ms(1000);
        } else {
            if recv(&mut radio, &mut rxbuf) == Some(b"PING") {
                led.set_high();
                send(&mut radio, b"PONG");
                led.set_low();
            }
        }
    }
}

// ------------------------
// HELPERS
// ------------------------

fn send<TSPI, RST, BSY, ANT, DIO1, TSPIERR, TPINERR>(
    radio: &mut SX126x<TSPI, RST, BSY, ANT, DIO1>,
    msg: &[u8],
) where
    TSPI: embedded_hal::spi::SpiDevice<u8, Error = TSPIERR>,
    RST: OutputPin<Error = TPINERR>,
    BSY: InputPin<Error = TPINERR>,
    ANT: OutputPin<Error = TPINERR>,
    DIO1: InputPin<Error = TPINERR>,
    TSPIERR: core::fmt::Debug,
    TPINERR: core::fmt::Debug,
{
    // Let the driver toggle the antenna pin for us
    radio.set_ant_enabled(true).unwrap();

    radio
        .write_bytes(msg, RxTxTimeout::from_ms(3000), 8, LoRaCrcType::CrcOn)
        .unwrap();

    radio.set_ant_enabled(false).unwrap();
}

fn recv<'a, TSPI, RST, BSY, ANT, DIO1, TSPIERR, TPINERR>(
    radio: &'a mut SX126x<TSPI, RST, BSY, ANT, DIO1>,
    buf: &'a mut [u8],
) -> Option<&'a [u8]>
where
    TSPI: embedded_hal::spi::SpiDevice<u8, Error = TSPIERR>,
    RST: OutputPin<Error = TPINERR>,
    BSY: InputPin<Error = TPINERR>,
    ANT: OutputPin<Error = TPINERR>,
    DIO1: InputPin<Error = TPINERR>,
    TSPIERR: core::fmt::Debug,
    TPINERR: core::fmt::Debug,
{
    radio.set_rx(RxTxTimeout::from_ms(4000)).ok()?;
    radio.wait_on_busy().ok()?;

    loop {
        let irq = radio.get_irq_status().ok()?;
        if irq.rx_done() {
            break;
        }
    }

    let status = radio.get_rx_buffer_status().ok()?;
    let len = status.payload_length_rx() as usize;
    let offset = status.rx_start_buffer_pointer();

    radio.read_buffer(offset, &mut buf[..len]).ok()?;
    radio.clear_irq_status(IrqMask::all()).ok()?;

    Some(&buf[..len])
}
