#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State as CdcAcmState},
    driver::EndpointError,
    Builder, Config, UsbDevice,
};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use core::cmp::min;
use core::fmt::Write;
use heapless::Vec;
use heapless::String;

use common::drivers::sx1262::*;
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::link::LoRaLink;
use common::coms::transport::lora::mac::{
    EnqueueClass, LinkMac, LinkMacConfig, MacRole, TickResult,
};
use common::coms::transport::lora::aggregate::for_each_section;
use common::coms::transport::lora::mac_codec::{unwrap_mac_inner, wrap_mac_packet};
use common::protocol::mavlink::encode::build_command_ack_frame;
use common::protocol::mavlink::encode::{build_statustext, build_statustext_frame};
use common::protocol::mavlink::link_mac_config::{parse_link_mac_config_cmd, LinkMacConfigCmdParse};
use common::protocol::mavlink::rf_reconfig::{
    build_rf_reconfig_command_frame, parse_rf_reconfig_cmd, RfReconfigCmdParse, RfReconfigOp,
    MARV_CMD_RF_RECONFIG,
};
use common::protocol::mavlink::prelude::dialects::common::enums::{MavCmd, MavResult};
use common::utils::delay::DelayMs;
use common::tasks::coms::MavEndpointConfig;
use common::coms::usb_cdc::AsyncUsbCdc;
use common::coms::usb_cdc;
use common::coms::transport::uart::MAVLINK_MAX_FRAME;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Receiver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;

// MAVLink common msg id for RADIO_STATUS
const MAVLINK_MSG_ID_RADIO_STATUS: u32 = 109;
// MAVLink common msg id for COMMAND_ACK
const MAVLINK_MSG_ID_COMMAND_ACK: u32 = 77;

// Component id used by the Radio bridge.
const RADIO_COMP_ID: u8 = 190;

embassy_rp::bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<embassy_rp::peripherals::USB>;
});

static USB_CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
type FrameMsg = Vec<u8, MAVLINK_MAX_FRAME>;
static FRAME_CH: Channel<RawMutex, FrameMsg, 4> = Channel::new();

// Host -> LoRa (commands/params/etc)
static USB_TO_LORA_CH: Channel<RawMutex, FrameMsg, 8> = Channel::new();

// (LoRa -> USB forwarding uses FRAME_CH directly.)

struct UsbCdc<'d> {
    class: CdcAcmClass<'d, UsbDriver<'d, embassy_rp::peripherals::USB>>,
    max_packet: usize,
}

impl<'d> UsbCdc<'d> {
    async fn wait_connection(&mut self) {
        self.class.wait_connection().await;
    }

    async fn read_packet(&mut self, buf: &mut [u8]) -> core::result::Result<usize, EndpointError> {
        self.class.read_packet(buf).await
    }
}

impl<'d> AsyncUsbCdc for UsbCdc<'d> {
    type Error = EndpointError;

    async fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), Self::Error> {
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let n = min(remaining.len(), self.max_packet);
            self.class.write_packet(&remaining[..n]).await?;
            remaining = &remaining[n..];
        }

        // Short packet to flush if we ended on a packet boundary.
        if !bytes.is_empty() && bytes.len() % self.max_packet == 0 {
            self.class.write_packet(&[]).await?;
        }
        Ok(())
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut device: UsbDevice<'static, UsbDriver<'static, embassy_rp::peripherals::USB>>,
) {
    device.run().await;
}

#[embassy_executor::task]
async fn usb_bridge_task(
    mut serial: UsbCdc<'static>,
    frame_rx: Receiver<'static, RawMutex, FrameMsg, 4>,
) {
    let mut rx_buf = [0u8; 64];
    let mut scratch = [0u8; 128];
    let frame_rx = frame_rx;

    // Accumulate raw bytes from USB until we can extract whole MAVLink2 frames.
    // Capacity is intentionally bounded; on overflow we drop oldest bytes.
    let mut stream_buf: Vec<u8, 512> = Vec::new();

    let _ = usb_cdc::write_line(&mut serial, "USB CDC bridge ready", &mut scratch).await;

    loop {
        // Race incoming frames vs host commands so neither starves.
        let tel_fut = frame_rx.receive();
        let usb_fut = serial.read_packet(&mut rx_buf);
        match embassy_futures::select::select(tel_fut, usb_fut).await {
            embassy_futures::select::Either::First(ev) => {
                let _ = serial.write(&ev).await;
            }
            embassy_futures::select::Either::Second(res) => match res {
                Ok(n) if n > 0 => {
                    // Binary bridge: do NOT emit ASCII replies (they corrupt MAVLink streams).
                    if push_bytes_with_drop_oldest(&mut stream_buf, &rx_buf[..n]).is_err() {
                        warn!("USB RX: stream buffer overflow");
                    }

                    while let Some(frame) = try_pop_mavlink2_frame(&mut stream_buf) {
                        if USB_TO_LORA_CH.sender().try_send(frame).is_err() {
                            warn!("USB->LoRa queue full (dropping)");
                        }
                    }
                }
                Ok(_) => {}
                Err(e) => {
                    let _ = usb_cdc::write_line(&mut serial, "usb read err", &mut scratch).await;
                    warn!("USB CDC read error: {:?}", e);
                }
            },
        }
    }
}

fn push_bytes_with_drop_oldest<const N: usize>(
    buf: &mut Vec<u8, N>,
    data: &[u8],
) -> core::result::Result<(), ()> {
    for &b in data {
        if buf.push(b).is_err() {
            // Drop oldest byte to make room.
            if !buf.is_empty() {
                buf.remove(0);
            }
            if buf.push(b).is_err() {
                return Err(());
            }
        }
    }
    Ok(())
}

fn try_pop_mavlink2_frame<const N: usize>(buf: &mut Vec<u8, N>) -> Option<FrameMsg> {
    const MAGIC: u8 = 0xFD;
    const HEADER_LEN: usize = 10;
    const CRC_LEN: usize = 2;
    const SIG_LEN: usize = 13;

    // Find magic.
    let start = buf.iter().position(|&b| b == MAGIC)?;
    if start != 0 {
        // Discard leading noise bytes.
        for _ in 0..start {
            buf.remove(0);
        }
    }

    if buf.len() < HEADER_LEN {
        return None;
    }

    let payload_len = buf[1] as usize;
    let incompat_flags = buf[2];
    let sig_len = if (incompat_flags & 0x01) != 0 { SIG_LEN } else { 0 };
    let frame_len = HEADER_LEN + payload_len + CRC_LEN + sig_len;

    if buf.len() < frame_len {
        return None;
    }

    let mut out: FrameMsg = FrameMsg::new();
    if out.extend_from_slice(&buf[..frame_len]).is_err() {
        // Frame too big for MAVLINK_MAX_FRAME (should not happen).
        for _ in 0..frame_len {
            buf.remove(0);
        }
        return None;
    }

    for _ in 0..frame_len {
        buf.remove(0);
    }
    Some(out)
}

fn mavlink2_msg_id(frame_bytes: &[u8]) -> Option<u32> {
    // MAVLink2 header: magic(0), len(1), incompat(2), compat(3), seq(4), sys(5), comp(6), msgid(7..9)
    if frame_bytes.len() < 10 || frame_bytes[0] != 0xFD {
        return None;
    }
    let id0 = frame_bytes[7] as u32;
    let id1 = frame_bytes[8] as u32;
    let id2 = frame_bytes[9] as u32;
    Some(id0 | (id1 << 8) | (id2 << 16))
}

fn is_fast_lane_msg_id(msg_id: u32) -> bool {
    const MAVLINK_MSG_ID_RAW_IMU: u32 = 27;
    // MARV-FC dialect fast-lane message IDs (see mavlink/marv_fc.xml)
    const MAVLINK_MSG_ID_FAST_RC: u32 = 42_000;
    const MAVLINK_MSG_ID_FAST_ATTITUDE_RATES: u32 = 42_001;

    // Transitional FAST telemetry carrier: RAW_IMU fits within inner MTU.
    msg_id == MAVLINK_MSG_ID_RAW_IMU
        || msg_id == MAVLINK_MSG_ID_FAST_RC
        || msg_id == MAVLINK_MSG_ID_FAST_ATTITUDE_RATES
}

fn maybe_log_packed_radio_status(frame_bytes: &[u8]) {
    // Expect MAVLink2 frame bytes.
    // Header layout:
    // 0 magic, 1 payload len, 2 incompat, 3 compat, 4 seq, 5 sys, 6 comp, 7..9 msgid
    const MAV2_MAGIC: u8 = 0xFD;
    const MAV2_HEADER_LEN: usize = 10;

    if frame_bytes.len() < MAV2_HEADER_LEN {
        return;
    }
    if frame_bytes[0] != MAV2_MAGIC {
        return;
    }

    let payload_len = frame_bytes[1] as usize;
    if frame_bytes.len() < MAV2_HEADER_LEN + payload_len {
        return;
    }

    let msg_id = (frame_bytes[7] as u32) | ((frame_bytes[8] as u32) << 8) | ((frame_bytes[9] as u32) << 16);
    if msg_id != MAVLINK_MSG_ID_RADIO_STATUS {
        return;
    }

    // RADIO_STATUS payload is 9 bytes:
    // rssi(i8), remrssi(u8), txbuf(u8), noise(i8), remnoise(u8), rxerrors(u16), fixed(u16)
    if payload_len < 9 {
        return;
    }

    let p = &frame_bytes[MAV2_HEADER_LEN..MAV2_HEADER_LEN + payload_len];
    let rssi_dbm: i8 = p[0] as i8;
    let remrssi: u8 = p[1];
    let txbuf_packed: u8 = p[2];
    let snr_db: i8 = p[3] as i8;
    let remnoise: u8 = p[4];
    let rxerrors: u16 = u16::from_le_bytes([p[5], p[6]]);
    let fixed: u16 = u16::from_le_bytes([p[7], p[8]]);

    // Unpack per docs/wireless-telemetry-ack-design.md §10.1
    let rel_q: u8 = remrssi & 0x07;
    let norm_q: u8 = (remrssi >> 3) & 0x07;
    let slot_mode_2b: u8 = (remrssi >> 6) & 0x03;
    let slot_mode_str: &str = match slot_mode_2b {
        0 => "UuuD",
        1 => "UdUd",
        2 => "Uddd",
        _ => "RESV",
    };

    let fast_replaced_delta_4b: u8 = remnoise & 0x0F;
    let mtu_drop_delta_4b: u8 = (remnoise >> 4) & 0x0F;

    let queue_fill_q4: u8 = txbuf_packed & 0x0F;
    let missed_master_ticks_delta_4b: u8 = (txbuf_packed >> 4) & 0x0F;

    let tick_hz_u8: u8 = (rxerrors & 0x00FF) as u8;
    let rx_hi: u8 = (rxerrors >> 8) as u8;
    let fast_present: bool = (rx_hi & 0x80) != 0;

    let rf_txn_state: u8 = rx_hi & 0x0F;
    let rf_last_result: u8 = (rx_hi >> 4) & 0x07;
    let rf_txn_state_str: &str = match rf_txn_state {
        0 => "Idle",
        1 => "Staged",
        2 => "Committed",
        3 => "ApplyPending",
        _ => "RESV",
    };
    let rf_last_result_str: &str = match rf_last_result {
        0 => "None",
        1 => "Ok",
        2 => "Denied",
        3 => "Failed",
        4 => "Timeout",
        _ => "RESV",
    };

    let drop_norm_delta: u8 = (fixed & 0x00FF) as u8;
    let drop_rel_delta: u8 = (fixed >> 8) as u8;

    info!(
        "RADIO_STATUS packed: rssi={}dBm snr={}dB rel_q={} norm_q={} slot={} fastQ={} tick_hz≈{} qfill_q4={} missed_tick_d4={} mtu_drop_d4={} fast_repl_d4={} rf_state={} rf_res={} rel_drop_d={} norm_drop_d={} ",
        rssi_dbm,
        snr_db,
        rel_q,
        norm_q,
        slot_mode_str,
        fast_present,
        tick_hz_u8,
        queue_fill_q4,
        missed_master_ticks_delta_4b,
        mtu_drop_delta_4b,
        fast_replaced_delta_4b,
        rf_txn_state_str,
        rf_last_result_str,
        drop_rel_delta,
        drop_norm_delta,
    );
}

// Simple delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("GS Boot");

    let p = embassy_rp::init(Default::default());

    let mav_cfg = MavEndpointConfig {
        sys_id: 1,
        comp_id: 1,
    };

    // USB CDC (for forwarding telemetry to host).
    let usb_driver = UsbDriver::new(p.USB, UsbIrqs);
    let mut usb_config = Config::new(0xC0DE, 0x4001);
    usb_config.manufacturer = Some("MARV-FC");
    usb_config.product = Some("GS USB CDC");
    usb_config.serial_number = Some("0001");
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
    let mut usb_serial = UsbCdc {
        class: cdc,
        max_packet,
    };

    spawner.spawn(usb_task(usb_dev)).unwrap();
    info!("GS USB: waiting for host connection...");
    usb_serial.wait_connection().await;
    info!("GS USB: host connected");
    let frame_rx = FRAME_CH.receiver();
    spawner.spawn(usb_bridge_task(usb_serial, frame_rx)).unwrap();

    // SPI
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    let spi = Spi::new(
        p.SPI0,
        p.PIN_2,  // SCK
        p.PIN_3,  // MOSI
        p.PIN_4,  // MISO
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    let nss   = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new(p.PIN_0, Pull::None);
    let dio1  = Input::new(p.PIN_6, Pull::None);

    // RF Switch (external TXEN/RXEN)
    struct ExtSw {
        tx: Output<'static>,
        rx: Output<'static>,
    }
    impl RfSwitch for ExtSw {
        fn set(&mut self, s: RfState) {
            match s {
                RfState::Rx => {
                    self.rx.set_low();
                    self.tx.set_high();
                }
                RfState::Tx => {
                    self.rx.set_high();
                    self.tx.set_low();
                }
                RfState::Off => {
                    self.rx.set_low();
                    self.tx.set_low();
                }
            }
        }
    }

    let rf_sw = ExtSw {
        tx: Output::new(p.PIN_8, Level::Low),
        rx: Output::new(p.PIN_9, Level::Low),
    };

    let cfg = LoRaConfig::preset_default();
    info!(
        "GS LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw, cfg);
    let mut delay = EmbassyDelay;

    radio.init(&mut delay).await.unwrap();

    let mut link = LoRaLink::new(&mut radio);
    info!("GS: LoRaLink MAVLink bridge (no link-level ARQ)");

    if let Err(e) = link.start_rx(&mut delay).await {
        warn!("GS: start_rx failed: {:?}", e);
    }

    gs_lora_loop(&mut link, &mut delay, mav_cfg).await;
}

async fn gs_lora_loop<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
    cfg: MavEndpointConfig,
) where
    RADIO: common::coms::transport::lora::link::Sx1262Interface,
{
    let mut rx_buf = [0u8; 255];
    let mut tx_seq: u8 = 0;

    // Staged RF reconfiguration (GS coordinator side).
    const RF_ACK_TIMEOUT_MS: u32 = 2_000;

    #[derive(Clone, Copy)]
    struct GsStagedRfTxn {
        txn_id: u16,
        proposed_at_ms: u32,
        apply_after_ticks: u8,
        apply_at_tick_seq: Option<u8>,
        committed: bool,
        radio_propose_ok: bool,
        radio_commit_ok: bool,
        cfg: LoRaConfig,
    }

    let mut current_rf_cfg: LoRaConfig = LoRaConfig::preset_default();
    let mut staged_rf: Option<GsStagedRfTxn> = None;
    let mut pending_apply: Option<LoRaConfig> = None;
    let mut awaiting_radio_ack: Option<(RfReconfigOp, u32)> = None;

    fn emit_statustext_to_usb(
        cfg: MavEndpointConfig,
        tx_seq: &mut u8,
        text: &str,
    ) {
        let st = build_statustext(text);
        let frame = build_statustext_frame(cfg, *tx_seq, st);
        *tx_seq = tx_seq.wrapping_add(1);
        let mut ser = [0u8; MAVLINK_MAX_FRAME];
        if let Ok(n) = frame.serialize(&mut ser) {
            let mut out: FrameMsg = FrameMsg::new();
            if out.extend_from_slice(&ser[..n]).is_ok() {
                let _ = FRAME_CH.sender().try_send(out);
            }
        }
    }

    fn parse_command_ack(frame_bytes: &[u8]) -> Option<(u16, u8, u8, u8, u8)> {
        // Returns (command_id, result_u8, sender_component, target_system, target_component)
        const MAV2_MAGIC: u8 = 0xFD;
        const MAV2_HEADER_LEN: usize = 10;
        if frame_bytes.len() < MAV2_HEADER_LEN || frame_bytes[0] != MAV2_MAGIC {
            return None;
        }
        let payload_len = frame_bytes[1] as usize;
        if frame_bytes.len() < MAV2_HEADER_LEN + payload_len {
            return None;
        }
        let msg_id = (frame_bytes[7] as u32)
            | ((frame_bytes[8] as u32) << 8)
            | ((frame_bytes[9] as u32) << 16);
        if msg_id != MAVLINK_MSG_ID_COMMAND_ACK {
            return None;
        }
        let sender_component = frame_bytes[6];
        // COMMAND_ACK payload is 10 bytes.
        if payload_len < 10 {
            return None;
        }
        let p = &frame_bytes[MAV2_HEADER_LEN..MAV2_HEADER_LEN + payload_len];
        let command_id = u16::from_le_bytes([p[0], p[1]]);
        let result = p[2];
        let target_system = p[8];
        let target_component = p[9];
        Some((command_id, result, sender_component, target_system, target_component))
    }

    // Single-source MAC config for this firmware (later: params / negotiated control-plane).
    let mut mac_cfg = LinkMacConfig::default();
    // Deterministic tick MAC (GS is the uplink side of the slot schedule).
    // NOTE: These are initial defaults; we'll later plumb them into params.
    let mut mac: LinkMac<8, 8> = LinkMac::new(
        embassy_time::Instant::now().as_millis() as u32,
        mac_cfg.tick_hz,
        mac_cfg.slot_mode,
        MacRole::Uplink,
    );

    let usb_to_lora_rx = USB_TO_LORA_CH.receiver();

    fn ms_to_ticks_ceil(ms: u32, tick_interval_ms: u32) -> u8 {
        let ti = tick_interval_ms.max(1);
        // ceil(ms / ti)
        let ticks = ms.saturating_add(ti - 1) / ti;
        ticks.min(0xFF) as u8
    }

    fn rf_apply_due_for_tick(staged: Option<GsStagedRfTxn>, tick_seq: u8) -> bool {
        let Some(st) = staged else {
            return false;
        };
        if !(st.committed && st.radio_commit_ok) {
            return false;
        }
        match st.apply_at_tick_seq {
            Some(t) => t == tick_seq,
            None => false,
        }
    }

    loop {
        // RF txn housekeeping: ack timeouts and apply scheduling.
        let now_ms = embassy_time::Instant::now().as_millis() as u32;
        if let Some((op, sent_at_ms)) = awaiting_radio_ack {
            if now_ms.wrapping_sub(sent_at_ms) >= RF_ACK_TIMEOUT_MS {
                awaiting_radio_ack = None;
                staged_rf = None;
                pending_apply = None;
                let mut msg: String<50> = String::new();
                let _ = match op {
                    RfReconfigOp::Propose => msg.push_str("RF: radio PROPOSE ack timeout"),
                    RfReconfigOp::Commit => msg.push_str("RF: radio COMMIT ack timeout"),
                    RfReconfigOp::Abort => msg.push_str("RF: radio ABORT ack timeout"),
                };
                emit_statustext_to_usb(cfg, &mut tx_seq, &msg);
            }
        }
        if let Some(cfg_to_apply) = pending_apply.take() {
            // Apply locally.
            let res = link.apply_radio_config(delay, cfg_to_apply).await;
            match res {
                Ok(()) => {
                    current_rf_cfg = cfg_to_apply;
                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: GS applied config");
                }
                Err(_) => {
                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: GS apply failed");
                }
            }
        }

        // Poll LoRa RX frequently, but allow outbound USB traffic to preempt.
        let tx_fut = usb_to_lora_rx.receive();
        let now_ms = embassy_time::Instant::now().as_millis() as u32;
        let wait_ms = mac.time_until_tick_ms(now_ms).max(1);
        let tick_fut = Timer::after(Duration::from_millis(wait_ms.into()));

        match embassy_futures::select::select(tx_fut, tick_fut).await {
            embassy_futures::select::Either::First(msg) => {
                // GS-coordinator staged RF reconfiguration from host.
                match parse_rf_reconfig_cmd(&msg, cfg.sys_id, cfg.comp_id) {
                    RfReconfigCmdParse::Accepted {
                        cmd,
                        sender_system,
                        sender_component,
                    } => {
                        // Execute coordinator logic + forward to radio.
                        let now_ms = embassy_time::Instant::now().as_millis() as u32;
                        match cmd.op {
                            RfReconfigOp::Propose => {
                                let Some(settings) = cmd.settings else {
                                    let host_ack = build_command_ack_frame(
                                        cfg,
                                        tx_seq,
                                        MavCmd::User2,
                                        MavResult::Failed,
                                        sender_system,
                                        sender_component,
                                    );
                                    tx_seq = tx_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = host_ack.serialize(&mut ser) {
                                        let mut out: FrameMsg = FrameMsg::new();
                                        if out.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = FRAME_CH.sender().try_send(out);
                                        }
                                    }
                                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: propose missing settings");
                                    // Do not forward malformed propose.
                                    continue;
                                };

                                let host_ack = build_command_ack_frame(
                                    cfg,
                                    tx_seq,
                                    MavCmd::User2,
                                    MavResult::Accepted,
                                    sender_system,
                                    sender_component,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = host_ack.serialize(&mut ser) {
                                    let mut out: FrameMsg = FrameMsg::new();
                                    if out.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = FRAME_CH.sender().try_send(out);
                                    }
                                }

                                let next = settings.apply_to_base(current_rf_cfg);
                                let apply_after_ticks = ms_to_ticks_ceil(cmd.apply_after_ms, mac.tick_interval_ms());

                                staged_rf = Some(GsStagedRfTxn {
                                    txn_id: cmd.txn_id,
                                    proposed_at_ms: now_ms,
                                    apply_after_ticks,
                                    // Filled in once we actually transmit the PROPOSE (tick-aligned base).
                                    apply_at_tick_seq: None,
                                    committed: false,
                                    radio_propose_ok: false,
                                    radio_commit_ok: false,
                                    cfg: next,
                                });

                                let mut s: String<50> = String::new();
                                let _ = core::write!(
                                    &mut s,
                                    "RF: staged txn={} apply+{}ms (~{}t)",
                                    cmd.txn_id,
                                    cmd.apply_after_ms,
                                    apply_after_ticks
                                );
                                emit_statustext_to_usb(cfg, &mut tx_seq, &s);

                                // Forward to radio.
                                let fwd = build_rf_reconfig_command_frame(
                                    cfg,
                                    tx_seq,
                                    cfg.sys_id,
                                    RADIO_COMP_ID,
                                    cmd,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = fwd.serialize(&mut ser) {
                                    let mut fwd_msg: FrameMsg = FrameMsg::new();
                                    if fwd_msg.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = mac.enqueue(EnqueueClass::Reliable, &fwd_msg);
                                        awaiting_radio_ack = Some((RfReconfigOp::Propose, now_ms));
                                    }
                                }
                                continue;
                            }
                            RfReconfigOp::Commit => {
                                let Some(st) = staged_rf else {
                                    let host_ack = build_command_ack_frame(
                                        cfg,
                                        tx_seq,
                                        MavCmd::User2,
                                        MavResult::Failed,
                                        sender_system,
                                        sender_component,
                                    );
                                    tx_seq = tx_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = host_ack.serialize(&mut ser) {
                                        let mut out: FrameMsg = FrameMsg::new();
                                        if out.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = FRAME_CH.sender().try_send(out);
                                        }
                                    }
                                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: commit with no staged txn");
                                    continue;
                                };
                                if st.txn_id != cmd.txn_id {
                                    let host_ack = build_command_ack_frame(
                                        cfg,
                                        tx_seq,
                                        MavCmd::User2,
                                        MavResult::Failed,
                                        sender_system,
                                        sender_component,
                                    );
                                    tx_seq = tx_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = host_ack.serialize(&mut ser) {
                                        let mut out: FrameMsg = FrameMsg::new();
                                        if out.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = FRAME_CH.sender().try_send(out);
                                        }
                                    }
                                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: commit txn_id mismatch");
                                    continue;
                                }
                                if now_ms.wrapping_sub(st.proposed_at_ms) >= RF_ACK_TIMEOUT_MS {
                                    staged_rf = None;
                                    let host_ack = build_command_ack_frame(
                                        cfg,
                                        tx_seq,
                                        MavCmd::User2,
                                        MavResult::Failed,
                                        sender_system,
                                        sender_component,
                                    );
                                    tx_seq = tx_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = host_ack.serialize(&mut ser) {
                                        let mut out: FrameMsg = FrameMsg::new();
                                        if out.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = FRAME_CH.sender().try_send(out);
                                        }
                                    }
                                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: staged txn expired");
                                    continue;
                                }

                                if !st.radio_propose_ok {
                                    let host_ack = build_command_ack_frame(
                                        cfg,
                                        tx_seq,
                                        MavCmd::User2,
                                        MavResult::TemporarilyRejected,
                                        sender_system,
                                        sender_component,
                                    );
                                    tx_seq = tx_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = host_ack.serialize(&mut ser) {
                                        let mut out: FrameMsg = FrameMsg::new();
                                        if out.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = FRAME_CH.sender().try_send(out);
                                        }
                                    }
                                    emit_statustext_to_usb(cfg, &mut tx_seq, "RF: commit blocked (radio not staged yet)");
                                    continue;
                                }

                                let host_ack = build_command_ack_frame(
                                    cfg,
                                    tx_seq,
                                    MavCmd::User2,
                                    MavResult::Accepted,
                                    sender_system,
                                    sender_component,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = host_ack.serialize(&mut ser) {
                                    let mut out: FrameMsg = FrameMsg::new();
                                    if out.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = FRAME_CH.sender().try_send(out);
                                    }
                                }

                                staged_rf = Some(GsStagedRfTxn {
                                    committed: true,
                                    ..st
                                });

                                let fwd = build_rf_reconfig_command_frame(
                                    cfg,
                                    tx_seq,
                                    cfg.sys_id,
                                    RADIO_COMP_ID,
                                    cmd,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = fwd.serialize(&mut ser) {
                                    let mut fwd_msg: FrameMsg = FrameMsg::new();
                                    if fwd_msg.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = mac.enqueue(EnqueueClass::Reliable, &fwd_msg);
                                        awaiting_radio_ack = Some((RfReconfigOp::Commit, now_ms));
                                        emit_statustext_to_usb(cfg, &mut tx_seq, "RF: commit sent to radio");
                                    }
                                }
                                continue;
                            }
                            RfReconfigOp::Abort => {
                                let host_ack = build_command_ack_frame(
                                    cfg,
                                    tx_seq,
                                    MavCmd::User2,
                                    MavResult::Accepted,
                                    sender_system,
                                    sender_component,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = host_ack.serialize(&mut ser) {
                                    let mut out: FrameMsg = FrameMsg::new();
                                    if out.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = FRAME_CH.sender().try_send(out);
                                    }
                                }

                                staged_rf = None;
                                pending_apply = None;

                                let fwd = build_rf_reconfig_command_frame(
                                    cfg,
                                    tx_seq,
                                    cfg.sys_id,
                                    RADIO_COMP_ID,
                                    cmd,
                                );
                                tx_seq = tx_seq.wrapping_add(1);
                                let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                if let Ok(n) = fwd.serialize(&mut ser) {
                                    let mut fwd_msg: FrameMsg = FrameMsg::new();
                                    if fwd_msg.extend_from_slice(&ser[..n]).is_ok() {
                                        let _ = mac.enqueue(EnqueueClass::Reliable, &fwd_msg);
                                        awaiting_radio_ack = Some((RfReconfigOp::Abort, now_ms));
                                    }
                                }
                                emit_statustext_to_usb(cfg, &mut tx_seq, "RF: aborted");
                                continue;
                            }
                        }
                    }
                    RfReconfigCmdParse::Rejected {
                        sender_system,
                        sender_component,
                    } => {
                        let ack = build_command_ack_frame(
                            cfg,
                            tx_seq,
                            MavCmd::User2,
                            MavResult::Unsupported,
                            sender_system,
                            sender_component,
                        );
                        tx_seq = tx_seq.wrapping_add(1);
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = ack.serialize(&mut ser) {
                            let mut out: FrameMsg = FrameMsg::new();
                            if out.extend_from_slice(&ser[..n]).is_ok() {
                                let _ = FRAME_CH.sender().try_send(out);
                            }
                        }
                        emit_statustext_to_usb(cfg, &mut tx_seq, "RF: rejected encoding");
                        continue;
                    }
                    RfReconfigCmdParse::NotForUs | RfReconfigCmdParse::NotRfReconfig => {}
                }

                match parse_link_mac_config_cmd(&msg, cfg.sys_id, cfg.comp_id) {
                    LinkMacConfigCmdParse::Accepted {
                        cfg: new_cfg,
                        sender_system,
                        sender_component,
                    } => {
                        mac_cfg = new_cfg;
                        mac.set_tick_hz(
                            embassy_time::Instant::now().as_millis() as u32,
                            mac_cfg.tick_hz,
                        );
                        mac.set_slot_mode(mac_cfg.slot_mode);
                        info!(
                            "GS: applied LINK_MAC_CONFIG tick_hz={} slot_mode={:?} fast_max_bytes={}",
                            mac_cfg.tick_hz,
                            mac_cfg.slot_mode,
                            mac_cfg.fast_max_bytes
                        );

                        // ACK back to USB sender (host).
                        let ack = build_command_ack_frame(
                            cfg,
                            tx_seq,
                            MavCmd::User1,
                            MavResult::Accepted,
                            sender_system,
                            sender_component,
                        );
                        tx_seq = tx_seq.wrapping_add(1);
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = ack.serialize(&mut ser) {
                            let mut out: FrameMsg = FrameMsg::new();
                            if out.extend_from_slice(&ser[..n]).is_ok() {
                                let _ = FRAME_CH.sender().try_send(out);
                            }
                        }
                    }
                    LinkMacConfigCmdParse::Rejected {
                        sender_system,
                        sender_component,
                    } => {
                        // We recognized this command id and it targets us, but encoding
                        // didn't match our expected magic/version.
                        let ack = build_command_ack_frame(
                            cfg,
                            tx_seq,
                            MavCmd::User1,
                            MavResult::Unsupported,
                            sender_system,
                            sender_component,
                        );
                        tx_seq = tx_seq.wrapping_add(1);
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = ack.serialize(&mut ser) {
                            let mut out: FrameMsg = FrameMsg::new();
                            if out.extend_from_slice(&ser[..n]).is_ok() {
                                let _ = FRAME_CH.sender().try_send(out);
                            }
                        }
                    }
                    LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {}
                }

                // Classify into FAST vs RELIABLE.
                // For now, NORMAL is unused on GS.
                let class = match mavlink2_msg_id(&msg) {
                    Some(id) if is_fast_lane_msg_id(id) && msg.len() <= (mac_cfg.fast_max_bytes as usize) => {
                        EnqueueClass::Fast
                    }
                    _ => EnqueueClass::Reliable,
                };
                let _ = mac.enqueue(class, &msg);
            }
            embassy_futures::select::Either::Second(()) => {
                // Always service RX.
                match link.try_recv(delay, &mut rx_buf).await {
                    Ok(Some(len)) => {
                        // Unwrap MAC header if present; forward only the inner MAVLink frame.
                        let inner = unwrap_mac_inner(&rx_buf[..len]);
                        if inner.is_empty() {
                            // SYNC-only.
                        } else if let Ok(true) = for_each_section(inner, |kind, bytes| {
                            // Forward both FAST and MAV sections upward as raw MAVLink frames.
                            let _ = kind;
                            let mut out: FrameMsg = FrameMsg::new();
                            if out.extend_from_slice(bytes).is_ok() {
                                // Observe radio COMMAND_ACK for rf-reconfig.
                                if let Some((cmd_id, result_u8, sender_comp, target_sys, target_comp)) =
                                    parse_command_ack(&out)
                                {
                                    if cmd_id == MARV_CMD_RF_RECONFIG
                                        && sender_comp == RADIO_COMP_ID
                                        && target_sys == cfg.sys_id
                                        && target_comp == cfg.comp_id
                                    {
                                        if let Some((op, _sent_at)) = awaiting_radio_ack {
                                            awaiting_radio_ack = None;
                                            if result_u8 == 0 {
                                                // Accepted.
                                                if let Some(st) = staged_rf {
                                                    staged_rf = Some(match op {
                                                        RfReconfigOp::Propose => GsStagedRfTxn {
                                                            radio_propose_ok: true,
                                                            ..st
                                                        },
                                                        RfReconfigOp::Commit => GsStagedRfTxn {
                                                            radio_commit_ok: true,
                                                            ..st
                                                        },
                                                        RfReconfigOp::Abort => st,
                                                    });
                                                }
                                                let _ = match op {
                                                    RfReconfigOp::Propose => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio staged"),
                                                    RfReconfigOp::Commit => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio committed"),
                                                    RfReconfigOp::Abort => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio aborted"),
                                                };
                                            } else {
                                                // Denied/failed.
                                                staged_rf = None;
                                                pending_apply = None;
                                                let mut s: String<50> = String::new();
                                                let _ = core::write!(&mut s, "RF: radio denied (res={})", result_u8);
                                                emit_statustext_to_usb(cfg, &mut tx_seq, &s);
                                            }
                                        }
                                    }
                                }

                                match parse_link_mac_config_cmd(&out, cfg.sys_id, cfg.comp_id) {
                                    LinkMacConfigCmdParse::Accepted {
                                        cfg: new_cfg,
                                        sender_system,
                                        sender_component,
                                    } => {
                                        mac_cfg = new_cfg;
                                        mac.set_tick_hz(
                                            embassy_time::Instant::now().as_millis() as u32,
                                            mac_cfg.tick_hz,
                                        );
                                        mac.set_slot_mode(mac_cfg.slot_mode);
                                        info!(
                                            "GS: applied LINK_MAC_CONFIG tick_hz={} slot_mode={:?} fast_max_bytes={}",
                                            mac_cfg.tick_hz,
                                            mac_cfg.slot_mode,
                                            mac_cfg.fast_max_bytes
                                        );

                                        // ACK back over LoRa to originator.
                                        let ack = build_command_ack_frame(
                                            cfg,
                                            tx_seq,
                                            MavCmd::User1,
                                            MavResult::Accepted,
                                            sender_system,
                                            sender_component,
                                        );
                                        tx_seq = tx_seq.wrapping_add(1);
                                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                        if let Ok(n) = ack.serialize(&mut ser) {
                                            let mut ack_msg: FrameMsg = FrameMsg::new();
                                            if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                                let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                            }
                                        }
                                    }
                                    LinkMacConfigCmdParse::Rejected {
                                        sender_system,
                                        sender_component,
                                    } => {
                                        let ack = build_command_ack_frame(
                                            cfg,
                                            tx_seq,
                                            MavCmd::User1,
                                            MavResult::Unsupported,
                                            sender_system,
                                            sender_component,
                                        );
                                        tx_seq = tx_seq.wrapping_add(1);
                                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                        if let Ok(n) = ack.serialize(&mut ser) {
                                            let mut ack_msg: FrameMsg = FrameMsg::new();
                                            if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                                let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                            }
                                        }
                                    }
                                    LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {}
                                }
                                maybe_log_packed_radio_status(&out);
                                if FRAME_CH.sender().try_send(out).is_err() {
                                    warn!("LoRa->USB queue full (dropping)");
                                }
                            }
                        }) {
                            // Aggregate handled.
                        } else {
                            // Legacy: treat entire inner as a single MAVLink frame.
                            let mut out: FrameMsg = FrameMsg::new();
                            if out.extend_from_slice(inner).is_ok() {
                                // Observe radio COMMAND_ACK for rf-reconfig.
                                if let Some((cmd_id, result_u8, sender_comp, target_sys, target_comp)) =
                                    parse_command_ack(&out)
                                {
                                    if cmd_id == MARV_CMD_RF_RECONFIG
                                        && sender_comp == RADIO_COMP_ID
                                        && target_sys == cfg.sys_id
                                        && target_comp == cfg.comp_id
                                    {
                                        if let Some((op, _sent_at)) = awaiting_radio_ack {
                                            awaiting_radio_ack = None;
                                            if result_u8 == 0 {
                                                if let Some(st) = staged_rf {
                                                    staged_rf = Some(match op {
                                                        RfReconfigOp::Propose => GsStagedRfTxn {
                                                            radio_propose_ok: true,
                                                            ..st
                                                        },
                                                        RfReconfigOp::Commit => GsStagedRfTxn {
                                                            radio_commit_ok: true,
                                                            ..st
                                                        },
                                                        RfReconfigOp::Abort => st,
                                                    });
                                                }
                                                let _ = match op {
                                                    RfReconfigOp::Propose => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio staged"),
                                                    RfReconfigOp::Commit => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio committed"),
                                                    RfReconfigOp::Abort => emit_statustext_to_usb(cfg, &mut tx_seq, "RF: radio aborted"),
                                                };
                                            } else {
                                                staged_rf = None;
                                                pending_apply = None;
                                                let mut s: String<50> = String::new();
                                                let _ = core::write!(&mut s, "RF: radio denied (res={})", result_u8);
                                                emit_statustext_to_usb(cfg, &mut tx_seq, &s);
                                            }
                                        }
                                    }
                                }

                                match parse_link_mac_config_cmd(&out, cfg.sys_id, cfg.comp_id) {
                                    LinkMacConfigCmdParse::Accepted {
                                        cfg: new_cfg,
                                        sender_system,
                                        sender_component,
                                    } => {
                                        mac_cfg = new_cfg;
                                        mac.set_tick_hz(
                                            embassy_time::Instant::now().as_millis() as u32,
                                            mac_cfg.tick_hz,
                                        );
                                        mac.set_slot_mode(mac_cfg.slot_mode);
                                        info!(
                                            "GS: applied LINK_MAC_CONFIG tick_hz={} slot_mode={:?} fast_max_bytes={}",
                                            mac_cfg.tick_hz,
                                            mac_cfg.slot_mode,
                                            mac_cfg.fast_max_bytes
                                        );

                                        let ack = build_command_ack_frame(
                                            cfg,
                                            tx_seq,
                                            MavCmd::User1,
                                            MavResult::Accepted,
                                            sender_system,
                                            sender_component,
                                        );
                                        tx_seq = tx_seq.wrapping_add(1);
                                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                        if let Ok(n) = ack.serialize(&mut ser) {
                                            let mut ack_msg: FrameMsg = FrameMsg::new();
                                            if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                                let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                            }
                                        }
                                    }
                                    LinkMacConfigCmdParse::Rejected {
                                        sender_system,
                                        sender_component,
                                    } => {
                                        let ack = build_command_ack_frame(
                                            cfg,
                                            tx_seq,
                                            MavCmd::User1,
                                            MavResult::Unsupported,
                                            sender_system,
                                            sender_component,
                                        );
                                        tx_seq = tx_seq.wrapping_add(1);
                                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                        if let Ok(n) = ack.serialize(&mut ser) {
                                            let mut ack_msg: FrameMsg = FrameMsg::new();
                                            if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                                let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                            }
                                        }
                                    }
                                    LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {}
                                }
                                maybe_log_packed_radio_status(&out);
                                if FRAME_CH.sender().try_send(out).is_err() {
                                    warn!("LoRa->USB queue full (dropping)");
                                }
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => warn!("GS LoRa RX error: {:?}", e),
                }

                // MAC tick: GS is master.
                // - TX slot: send either queued payload or SYNC-only.
                // - RX slot: do not transmit.
                let now_ms = embassy_time::Instant::now().as_millis() as u32;
                match mac.poll_tick(now_ms) {
                    TickResult::TxSlot {
                        tick_seq,
                        payload,
                        ..
                    } => {
                        // Tick-aligned RF apply: trigger on the exact tick boundary.
                        if rf_apply_due_for_tick(staged_rf, tick_seq) {
                            if let Some(st) = staged_rf {
                                pending_apply = Some(st.cfg);
                                staged_rf = None;
                            }
                        }

                        let mut tx = heapless::Vec::<u8, { common::coms::transport::lora::mac::LORA_MAC_MTU }>::new();
                        if let Some(inner) = payload {
                            // Capture the tick that carried the RF PROPOSE to the radio.
                            // This makes local apply phase-align with the radio even if the USB->TX path jitters.
                            if let Some(st) = staged_rf {
                                if st.apply_at_tick_seq.is_none() {
                                    let mut saw_propose_for_radio = false;

                                    match common::coms::transport::lora::aggregate::for_each_section(
                                        &inner,
                                        |_, bytes| {
                                            if let RfReconfigCmdParse::Accepted { cmd, .. } =
                                                parse_rf_reconfig_cmd(bytes, cfg.sys_id, RADIO_COMP_ID)
                                            {
                                                if cmd.op == RfReconfigOp::Propose && cmd.txn_id == st.txn_id {
                                                    saw_propose_for_radio = true;
                                                }
                                            }
                                        },
                                    ) {
                                        Ok(true) => {}
                                        Ok(false) => {
                                            if let RfReconfigCmdParse::Accepted { cmd, .. } =
                                                parse_rf_reconfig_cmd(&inner, cfg.sys_id, RADIO_COMP_ID)
                                            {
                                                if cmd.op == RfReconfigOp::Propose && cmd.txn_id == st.txn_id {
                                                    saw_propose_for_radio = true;
                                                }
                                            }
                                        }
                                        Err(_) => {}
                                    }

                                    if saw_propose_for_radio {
                                        staged_rf = Some(GsStagedRfTxn {
                                            apply_at_tick_seq: Some(tick_seq.wrapping_add(st.apply_after_ticks)),
                                            ..st
                                        });
                                    }
                                }
                            }

                            if wrap_mac_packet(&mut tx, tick_seq, true, &inner).is_ok() {
                                if let Err(e) = link.send_unreliable(delay, &tx).await {
                                    warn!("GS LoRa TX error: {:?}", e);
                                }
                            }
                        } else {
                            // SYNC-only (no inner payload) to keep radio aligned.
                            if wrap_mac_packet(&mut tx, tick_seq, false, &[]).is_ok() {
                                let _ = link.send_unreliable(delay, &tx).await;
                            }
                        }
                    }
                    TickResult::RxSlot { tick_seq, .. } => {
                        // Tick-aligned RF apply also triggers on RX-slot tick boundaries.
                        if rf_apply_due_for_tick(staged_rf, tick_seq) {
                            if let Some(st) = staged_rf {
                                pending_apply = Some(st.cfg);
                                staged_rf = None;
                            }
                        }
                    }
                    TickResult::NotDue => {}
                }
            }
        }
    }
}

// MAC wrap/unwrap helpers live in `common::coms::transport::lora::mac_codec`.
