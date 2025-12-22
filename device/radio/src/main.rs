#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::uart::{Config as UartConfig, Uart, InterruptHandler as UartInterruptHandler, Async as UartAsync};
use embassy_time::{Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use heapless::Vec;
use heapless::String;
use core::fmt::Write;
use core::sync::atomic::{AtomicU8, Ordering};
use core::sync::atomic::AtomicU16;
use core::sync::atomic::AtomicU32;

use common::drivers::sx1262::*;
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::link::{LoRaLink, Sx1262Interface};
use common::coms::transport::lora::mac::{
    EnqueueClass, LinkMac, MacRole, SlotMode, TickResult, LinkMacConfig,
};
use common::coms::transport::lora::aggregate::for_each_section;
use common::coms::transport::lora::mac_codec::{mac_tick_from_packet, unwrap_mac_inner, wrap_mac_packet};
use common::protocol::mavlink;
use common::protocol::mavlink::encode::{
    build_command_ack_frame, build_radio_status_frame, build_statustext, build_statustext_frame,
    MavEndpointConfig,
};
use common::protocol::mavlink::link_authority::{
    parse_link_authority_frame, LAS_STATE_ARMED, LAS_STATE_DISARMED, LAS_STATE_UNKNOWN,
};
use common::protocol::mavlink::link_mac_config::{parse_link_mac_config_cmd, LinkMacConfigCmdParse};
use common::protocol::mavlink::link_metrics::{build_link_metrics_frame, LinkMetrics};
use common::protocol::mavlink::rf_reconfig::{parse_rf_reconfig_cmd, RfReconfigCmdParse, RfReconfigCommand, RfReconfigOp};
use common::protocol::mavlink::prelude::dialects::common::enums::{MavCmd, MavResult};
use common::utils::delay::DelayMs;
use common::tasks::coms::statustext_to_str;
use common::protocol::mavlink::prelude::dialects::common::Common;
use common::coms::transport::uart::{AsyncUartBus, MAVLINK_MAX_FRAME};

// Arbitrary-but-stable component id for the radio bridge.
const RADIO_COMP_ID: u8 = 190;

type FrameMsg = Vec<u8, MAVLINK_MAX_FRAME>;

// UART -> LoRa (high priority: commands/params/etc)
static UART_TO_LORA_HI: Channel<RawMutex, FrameMsg, 8> = Channel::new();
// LoRa -> UART (all inbound frames)
static LORA_TO_UART: Channel<RawMutex, FrameMsg, 8> = Channel::new();

// Latest telemetry frame from UART -> LoRa (overwrite semantics, Betaflight-style).
static UART_TO_LORA_TELEM_LATEST: Mutex<RawMutex, Option<FrameMsg>> = Mutex::new(None);
static UART_TO_LORA_TELEM_SIG: Signal<RawMutex, ()> = Signal::new();

// Track the FC system id observed on the UART side so RADIO_STATUS shows up under the same
// MAVLink system in GCS tooling.
static LAST_FC_SYS_ID: AtomicU8 = AtomicU8::new(1);

// Runtime-updatable link MAC config (applied to the LoRa tick MAC on the radio).
static MAC_TICK_HZ: AtomicU16 = AtomicU16::new(common::coms::transport::lora::mac::LINK_TICK_HZ_DEFAULT);
static MAC_FAST_MAX_BYTES: AtomicU16 =
    AtomicU16::new(common::coms::transport::lora::mac::INNER_MTU as u16);
static MAC_SLOT_MODE: AtomicU8 = AtomicU8::new(1); // 0=UuuD, 1=UdUd, 2=Uddd

// Link Authority State (LAS) as consumed by the radio.
//
// Source: explicit MARV_LINK_AUTH over UART.
// Safety rule: RF reconfiguration is denied unless DISARMED and fresh.
static LAS_STATE: AtomicU8 = AtomicU8::new(0); // 0=Unknown, 1=Disarmed, 2=Armed
static LAS_LAST_UPDATE_MS: AtomicU32 = AtomicU32::new(0);
static LAS_MAX_AGE_MS: AtomicU32 = AtomicU32::new(1_000);

// Cumulative FAST seq loss observed on UART RX (FC -> Radio).
static FAST_SEQ_MISSED_UART: AtomicU32 = AtomicU32::new(0);

// Runtime RF reconfiguration state (staged apply).
const RF_TXN_GUARD_MS: u32 = 2_000;

#[derive(Clone, Copy)]
struct StagedRfTxn {
    txn_id: u16,
    proposed_at_ms: u32,
    apply_at_tick_seq: u8,
    committed: bool,
    cfg: LoRaConfig,
}

fn ms_to_ticks_ceil(ms: u32, tick_interval_ms: u32) -> u8 {
    let ti = tick_interval_ms.max(1);
    let ticks = ms.saturating_add(ti - 1) / ti;
    ticks.min(0xFF) as u8
}

fn las_allows_rf_change(now_ms: u32) -> bool {
    let state = LAS_STATE.load(Ordering::Relaxed);
    let last = LAS_LAST_UPDATE_MS.load(Ordering::Relaxed);
    if state != 1 {
        // not DISARMED
        return false;
    }
    let age = now_ms.wrapping_sub(last);
    let max_age_ms = LAS_MAX_AGE_MS.load(Ordering::Relaxed).max(1);
    age < max_age_ms
}

// Simple embassy-based delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

embassy_rp::bind_interrupts!(struct UartIrqs {
    UART0_IRQ => UartInterruptHandler<embassy_rp::peripherals::UART0>;
});

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Radio Boot");

    let p = embassy_rp::init(Default::default());

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
        "Radio LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw, cfg);
    let mut delay = EmbassyDelay;

    radio.init(&mut delay).await.unwrap();

    let mut link = LoRaLink::new(&mut radio);
    info!("Radio: LoRaLink bridge (no link-level ARQ)");

    // UART0 (Radio <-> FC) on GP12 (TX) / GP13 (RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart = RpUart(Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        UartIrqs,
        p.DMA_CH2,
        p.DMA_CH3,
        uart_cfg,
    ));

    // Spawn UART task (bidirectional) so UART RX is not blocked by LoRa TX pacing/ACK waits.
    // Initial MAC config (later: params / negotiated control-plane).
    let mac_cfg = LinkMacConfig::default();
    MAC_TICK_HZ.store(mac_cfg.tick_hz, Ordering::Relaxed);
    MAC_FAST_MAX_BYTES.store(mac_cfg.fast_max_bytes, Ordering::Relaxed);
    MAC_SLOT_MODE.store(
        match mac_cfg.slot_mode {
            SlotMode::UuuD => 0,
            SlotMode::UdUd => 1,
            SlotMode::Uddd => 2,
        },
        Ordering::Relaxed,
    );
    spawner.spawn(uart_task(uart)).unwrap();

    // Main task owns the LoRa link and services both RX and TX.
    lora_loop(&mut link, &mut delay).await;
}

#[embassy_executor::task]
async fn uart_task(mut uart: RpUart<'static>) {
    let mut scratch = [0u8; MAVLINK_MAX_FRAME];
    let mut ser_buf = [0u8; MAVLINK_MAX_FRAME];

    let lora_to_uart_rx = LORA_TO_UART.receiver();
    let mut fast_seq_uart = FastSeqTracker::default();

    loop {
        let rx_fut = mavlink::recv_frame_over_uart(&mut uart, &mut scratch);
        let tx_fut = lora_to_uart_rx.receive();

        match embassy_futures::select::select(rx_fut, tx_fut).await {
            embassy_futures::select::Either::First(res) => {
                let frame = match res {
                    Ok(f) => f,
                    Err(e) => {
                        warn!("UART RX error: {:?}", e);
                        continue;
                    }
                };

                // Optional logs (keep lightweight; do not decode everything).
                LAST_FC_SYS_ID.store(frame.system_id(), Ordering::Relaxed);
                let local_sys = LAST_FC_SYS_ID.load(Ordering::Relaxed);
                if let Some(auth) = parse_link_authority_frame(&frame, local_sys, RADIO_COMP_ID) {
                    let now_ms = Instant::now().as_millis() as u32;
                    let state = match auth.state {
                        LAS_STATE_DISARMED => LAS_STATE_DISARMED,
                        LAS_STATE_ARMED => LAS_STATE_ARMED,
                        _ => LAS_STATE_UNKNOWN,
                    };
                    LAS_STATE.store(state, Ordering::Relaxed);
                    LAS_LAST_UPDATE_MS.store(now_ms, Ordering::Relaxed);
                    LAS_MAX_AGE_MS.store(auth.max_age_ms.max(1) as u32, Ordering::Relaxed);
                }

                match frame.decode::<Common>() {
                    Ok(Common::Statustext(msg)) => {
                        let text = statustext_to_str(&msg);
                        info!(
                            "UART RX sys={} comp={} seq={}: {}",
                            frame.system_id(),
                            frame.component_id(),
                            frame.sequence(),
                            text
                        );
                    }
                    Ok(Common::RawImu(pkt)) => {
                        info!(
                            "UART RX RAW_IMU t={}us acc[{}, {}, {}] gyro[{}, {}, {}] mag[{}, {}, {}]",
                            pkt.time_usec,
                            pkt.xacc,
                            pkt.yacc,
                            pkt.zacc,
                            pkt.xgyro,
                            pkt.ygyro,
                            pkt.zgyro,
                            pkt.xmag,
                            pkt.ymag,
                            pkt.zmag,
                        );
                    }
                    _ => {}
                }

                // Serialize once and forward as raw bytes.
                let written = match frame.serialize(&mut ser_buf) {
                    Ok(n) => n,
                    Err(_) => {
                        warn!("UART RX: serialize failed");
                        continue;
                    }
                };
                let mut msg: FrameMsg = FrameMsg::new();
                if msg.extend_from_slice(&ser_buf[..written]).is_err() {
                    warn!("UART RX: frame too big for queue");
                    continue;
                }

                match parse_link_mac_config_cmd(&msg, local_sys, RADIO_COMP_ID) {
                    LinkMacConfigCmdParse::Accepted {
                        cfg: new_cfg,
                        sender_system,
                        sender_component,
                    } => {
                        MAC_TICK_HZ.store(new_cfg.tick_hz, Ordering::Relaxed);
                        MAC_FAST_MAX_BYTES.store(new_cfg.fast_max_bytes, Ordering::Relaxed);
                        MAC_SLOT_MODE.store(
                            match new_cfg.slot_mode {
                                SlotMode::UuuD => 0,
                                SlotMode::UdUd => 1,
                                SlotMode::Uddd => 2,
                            },
                            Ordering::Relaxed,
                        );
                        info!(
                            "Radio: applied LINK_MAC_CONFIG from UART tick_hz={} slot_mode={:?} fast_max_bytes={}",
                            new_cfg.tick_hz,
                            new_cfg.slot_mode,
                            new_cfg.fast_max_bytes
                        );

                        // ACK back over UART to command origin.
                        let ack_cfg = MavEndpointConfig {
                            sys_id: local_sys,
                            comp_id: RADIO_COMP_ID,
                        };
                        let ack = build_command_ack_frame(
                            ack_cfg,
                            frame.sequence().wrapping_add(1),
                            MavCmd::User1,
                            MavResult::Accepted,
                            sender_system,
                            sender_component,
                        );
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = ack.serialize(&mut ser) {
                            let _ = uart.write(&ser[..n]).await;
                        }
                    }
                    LinkMacConfigCmdParse::Rejected {
                        sender_system,
                        sender_component,
                    } => {
                        let ack_cfg = MavEndpointConfig {
                            sys_id: local_sys,
                            comp_id: RADIO_COMP_ID,
                        };
                        let ack = build_command_ack_frame(
                            ack_cfg,
                            frame.sequence().wrapping_add(1),
                            MavCmd::User1,
                            MavResult::Unsupported,
                            sender_system,
                            sender_component,
                        );
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = ack.serialize(&mut ser) {
                            let _ = uart.write(&ser[..n]).await;
                        }
                    }
                    LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {}
                }

                // Note: RF reconfiguration commands are expected from the GS over LoRa.
                // We intentionally do not intercept rf-reconfig over UART here.

                if let Some(msg_id) = mavlink2_msg_id(&msg) {
                    if let Some(seq) = fast_seq_from_mavlink_frame(&msg, msg_id) {
                        let missed = fast_seq_uart.observe(msg_id, seq);
                        if missed != 0 {
                            FAST_SEQ_MISSED_UART.fetch_add(missed, Ordering::Relaxed);
                        }
                    }
                    if is_fast_lane_msg_id(msg_id) {
                        // Enforce FAST byte budget: oversized frames are demoted to RELIABLE.
                        let fast_max_bytes = MAC_FAST_MAX_BYTES.load(Ordering::Relaxed) as usize;
                        if msg.len() <= fast_max_bytes {
                            *UART_TO_LORA_TELEM_LATEST.lock().await = Some(msg);
                            UART_TO_LORA_TELEM_SIG.signal(());
                        } else if UART_TO_LORA_HI.sender().try_send(msg).is_err() {
                            warn!("UART->LoRa queue full (dropping)");
                        }
                    } else if UART_TO_LORA_HI.sender().try_send(msg).is_err() {
                        warn!("UART->LoRa queue full (dropping)");
                    }
                } else if UART_TO_LORA_HI.sender().try_send(msg).is_err() {
                    warn!("UART->LoRa queue full (dropping)");
                }
            }
            embassy_futures::select::Either::Second(msg) => {
                if uart.write(&msg).await.is_err() {
                    warn!("UART TX error");
                }
            }
        }
    }
}

async fn lora_loop<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
) where
    RADIO: Sx1262Interface,
{
    if let Err(e) = link.start_rx(delay).await {
        warn!("Radio: start_rx failed: {:?}", e);
    }

    // Deterministic tick MAC (Radio is the downlink side of the slot schedule).
    // NOTE: These are initial defaults; we'll later plumb them into params.
    let mut mac: LinkMac<8, 8> = LinkMac::new(
        Instant::now().as_millis() as u32,
        MAC_TICK_HZ.load(Ordering::Relaxed),
        match MAC_SLOT_MODE.load(Ordering::Relaxed) {
            0 => SlotMode::UuuD,
            2 => SlotMode::Uddd,
            _ => SlotMode::UdUd,
        },
        MacRole::Downlink,
    );
    let mut last_applied_tick_hz: u16 = MAC_TICK_HZ.load(Ordering::Relaxed);
    let mut last_applied_slot_mode: u8 = MAC_SLOT_MODE.load(Ordering::Relaxed);
    let mut rx_buf = [0u8; 255];
    let mut ack_seq: u8 = 0;

    let mut current_rf_cfg = LoRaConfig::preset_default();
    let mut staged_rf: Option<StagedRfTxn> = None;
    let mut pending_apply: Option<LoRaConfig> = None;

    // Periodic link-quality reporting.
    // Keep this slow and on the NORMAL lane so it never steals control airtime.
    let mut last_radio_status_ms: u32 = Instant::now().as_millis() as u32;
    let mut radio_status_seq: u8 = 0;
    let mut link_metrics_seq: u8 = 0;
    let mut last_fast_replaced: u32 = 0;
    let mut last_dropped_too_large: u32 = 0;
    let mut last_drop_rel_overflow: u32 = 0;
    let mut last_drop_norm_overflow: u32 = 0;

    // Always-on loss observability (PER proxy): count missed master tick_seq values.
    // This is independent of MAVLink FAST seq (not always present in transitional carriers).
    let mut last_master_tick_seq: Option<u8> = None;
    let mut missed_master_ticks: u32 = 0;
    let mut last_missed_master_ticks: u32 = 0;

    let mut fast_seq_lora = FastSeqTracker::default();
    let mut missed_fast_seq_lora: u32 = 0;

    // Always-on RF txn observability.
    // rf_last_result codes: 0=None, 1=Ok, 2=Denied, 3=Failed, 4=Timeout
    let mut rf_last_result: u8 = 0;

    let hi_rx = UART_TO_LORA_HI.receiver();

    loop {
        // Service LoRa RX frequently (low latency), but always prioritize high-priority TX.
        let hi_fut = hi_rx.receive();
        let tel_fut = UART_TO_LORA_TELEM_SIG.wait();
        let now_ms = Instant::now().as_millis() as u32;
        let wait_ms = mac.time_until_tick_ms(now_ms).min(2).max(1);
        let tick_fut = Timer::after(Duration::from_millis(wait_ms.into()));

        match embassy_futures::select::select3(hi_fut, tel_fut, tick_fut).await {
            embassy_futures::select::Either3::First(msg) => {
                // RELIABLE lane = prioritized FIFO, but still sent unreliably on-air.
                let _ = mac.enqueue(EnqueueClass::Reliable, &msg);
            }
            embassy_futures::select::Either3::Second(()) => {
                let maybe = UART_TO_LORA_TELEM_LATEST.lock().await.take();
                if let Some(msg) = maybe {
                    // FAST lane = newest-wins.
                    let _ = mac.enqueue(EnqueueClass::Fast, &msg);
                }
            }
            embassy_futures::select::Either3::Third(()) => {
                // Always service RX.
                match link.try_recv(delay, &mut rx_buf).await {
                    Ok(Some(len)) => {
                        // If this is a GS-master MAC packet, use it to sync our tick/slot phase.
                        let now_ms = Instant::now().as_millis() as u32;
                        let master_tick_opt = mac_tick_from_packet(&rx_buf[..len]);
                        if let Some(master_tick) = master_tick_opt {
                            if let Some(prev) = last_master_tick_seq {
                                let diff = master_tick.wrapping_sub(prev);
                                if diff != 0 {
                                    // diff includes the newly-received tick; subtract 1 to get missed.
                                    missed_master_ticks = missed_master_ticks.wrapping_add(diff.wrapping_sub(1) as u32);
                                }
                            }
                            last_master_tick_seq = Some(master_tick);
                            mac.on_master_sync(now_ms, master_tick);
                        }

                        // Forward only inner MAVLink bytes to the FC.
                        let inner = unwrap_mac_inner(&rx_buf[..len]);
                        if inner.is_empty() {
                            // SYNC-only.
                        } else if let Ok(true) = for_each_section(inner, |kind, bytes| {
                            // Forward both FAST and MAV sections upward as raw MAVLink frames.
                            let _ = kind;

                            // Intercept and consume RF reconfiguration commands from the link.
                            match parse_rf_reconfig_cmd(
                                bytes,
                                LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                RADIO_COMP_ID,
                            ) {
                                RfReconfigCmdParse::Accepted {
                                    cmd,
                                    sender_system,
                                    sender_component,
                                } => {
                                    let now_ms = Instant::now().as_millis() as u32;

                                    let tick_base: u8 = master_tick_opt.unwrap_or(mac.next_tick_seq());
                                    let tick_interval_ms: u32 = mac.tick_interval_ms();

                                    let result = handle_rf_reconfig_cmd(
                                        now_ms,
                                        tick_base,
                                        tick_interval_ms,
                                        cmd,
                                        sender_system,
                                        sender_component,
                                        &mut staged_rf,
                                        &mut pending_apply,
                                        &mut current_rf_cfg,
                                    );

                                    // Persist last non-OK outcome for machine-parsable telemetry.
                                    // (Accepted is not terminal; apply outcome is reported separately.)
                                    rf_last_result = match result {
                                        MavResult::Denied => 2,
                                        MavResult::Failed | MavResult::Unsupported => 3,
                                        _ => rf_last_result,
                                    };

                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User2,
                                        result,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }

                                    // Minimal observability for field debugging.
                                    let mut s: String<50> = String::new();
                                    let _ = match cmd.op {
                                        RfReconfigOp::Propose => {
                                            core::write!(&mut s, "RF: PROPOSE txn={} res={:?}", cmd.txn_id, result)
                                        }
                                        RfReconfigOp::Commit => {
                                            core::write!(&mut s, "RF: COMMIT txn={} res={:?}", cmd.txn_id, result)
                                        }
                                        RfReconfigOp::Abort => {
                                            core::write!(&mut s, "RF: ABORT txn={} res={:?}", cmd.txn_id, result)
                                        }
                                    };
                                    let st = build_statustext(&s);
                                    let st_frame = build_statustext_frame(ack_cfg, ack_seq, st);
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = st_frame.serialize(&mut ser) {
                                        let mut st_msg: FrameMsg = FrameMsg::new();
                                        if st_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &st_msg);
                                        }
                                    }
                                    return;
                                }
                                RfReconfigCmdParse::Rejected {
                                    sender_system,
                                    sender_component,
                                } => {
                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User2,
                                        MavResult::Unsupported,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }
                                    return;
                                }
                                RfReconfigCmdParse::NotForUs | RfReconfigCmdParse::NotRfReconfig => {}
                            }

                            // Intercept and consume LINK_MAC_CONFIG commands from the link.
                            match parse_link_mac_config_cmd(
                                bytes,
                                LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                RADIO_COMP_ID,
                            ) {
                                LinkMacConfigCmdParse::Accepted {
                                    cfg: new_cfg,
                                    sender_system,
                                    sender_component,
                                } => {
                                    MAC_TICK_HZ.store(new_cfg.tick_hz, Ordering::Relaxed);
                                    MAC_FAST_MAX_BYTES.store(new_cfg.fast_max_bytes, Ordering::Relaxed);
                                    MAC_SLOT_MODE.store(
                                        match new_cfg.slot_mode {
                                            SlotMode::UuuD => 0,
                                            SlotMode::UdUd => 1,
                                            SlotMode::Uddd => 2,
                                        },
                                        Ordering::Relaxed,
                                    );
                                    info!(
                                        "Radio: applied LINK_MAC_CONFIG from LoRa tick_hz={} slot_mode={:?} fast_max_bytes={}",
                                        new_cfg.tick_hz,
                                        new_cfg.slot_mode,
                                        new_cfg.fast_max_bytes
                                    );

                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User1,
                                        MavResult::Accepted,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }
                                    return;
                                }
                                LinkMacConfigCmdParse::Rejected {
                                    sender_system,
                                    sender_component,
                                } => {
                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User1,
                                        MavResult::Unsupported,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }
                                    return;
                                }
                                LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {}
                            }

                            if let Some(msg_id) = mavlink2_msg_id(bytes) {
                                if let Some(seq) = fast_seq_from_mavlink_frame(bytes, msg_id) {
                                    let missed = fast_seq_lora.observe(msg_id, seq);
                                    missed_fast_seq_lora = missed_fast_seq_lora.wrapping_add(missed);
                                }
                            }

                            let mut msg: FrameMsg = FrameMsg::new();
                            if msg.extend_from_slice(bytes).is_ok() {
                                if LORA_TO_UART.sender().try_send(msg).is_err() {
                                    warn!("LoRa->UART queue full (dropping)");
                                }
                            }
                        }) {
                            // Aggregate handled.
                        } else {
                            // Legacy: treat entire inner as a single MAVLink frame.
                            let mut consumed_rf_reconfig = false;
                            match parse_rf_reconfig_cmd(
                                inner,
                                LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                RADIO_COMP_ID,
                            ) {
                                RfReconfigCmdParse::Accepted {
                                    cmd,
                                    sender_system,
                                    sender_component,
                                } => {
                                    consumed_rf_reconfig = true;
                                    let now_ms = Instant::now().as_millis() as u32;
                                    let tick_base: u8 = mac.next_tick_seq();
                                    let tick_interval_ms: u32 = mac.tick_interval_ms();
                                    let result = handle_rf_reconfig_cmd(
                                        now_ms,
                                        tick_base,
                                        tick_interval_ms,
                                        cmd,
                                        sender_system,
                                        sender_component,
                                        &mut staged_rf,
                                        &mut pending_apply,
                                        &mut current_rf_cfg,
                                    );

                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User2,
                                        result,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }

                                    // Minimal observability for field debugging.
                                    let mut s: String<50> = String::new();
                                    let _ = match cmd.op {
                                        RfReconfigOp::Propose => {
                                            core::write!(&mut s, "RF: PROPOSE txn={} res={:?}", cmd.txn_id, result)
                                        }
                                        RfReconfigOp::Commit => {
                                            core::write!(&mut s, "RF: COMMIT txn={} res={:?}", cmd.txn_id, result)
                                        }
                                        RfReconfigOp::Abort => {
                                            core::write!(&mut s, "RF: ABORT txn={} res={:?}", cmd.txn_id, result)
                                        }
                                    };
                                    let st = build_statustext(&s);
                                    let st_frame = build_statustext_frame(ack_cfg, ack_seq, st);
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = st_frame.serialize(&mut ser) {
                                        let mut st_msg: FrameMsg = FrameMsg::new();
                                        if st_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &st_msg);
                                        }
                                    }
                                }
                                RfReconfigCmdParse::Rejected {
                                    sender_system,
                                    sender_component,
                                } => {
                                    consumed_rf_reconfig = true;
                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User2,
                                        MavResult::Unsupported,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }
                                }
                                RfReconfigCmdParse::NotForUs | RfReconfigCmdParse::NotRfReconfig => {
                                    // fallthrough to LINK_MAC_CONFIG/forwarding below
                                }
                            }

                            if consumed_rf_reconfig {
                                // Consumed locally; do not forward to FC.
                            } else {
                                match parse_link_mac_config_cmd(
                                    inner,
                                    LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                    RADIO_COMP_ID,
                                ) {
                                LinkMacConfigCmdParse::Accepted {
                                    cfg: new_cfg,
                                    sender_system,
                                    sender_component,
                                } => {
                                    MAC_TICK_HZ.store(new_cfg.tick_hz, Ordering::Relaxed);
                                    MAC_FAST_MAX_BYTES.store(new_cfg.fast_max_bytes, Ordering::Relaxed);
                                    MAC_SLOT_MODE.store(
                                        match new_cfg.slot_mode {
                                            SlotMode::UuuD => 0,
                                            SlotMode::UdUd => 1,
                                            SlotMode::Uddd => 2,
                                        },
                                        Ordering::Relaxed,
                                    );
                                    info!(
                                        "Radio: applied LINK_MAC_CONFIG from LoRa tick_hz={} slot_mode={:?} fast_max_bytes={}",
                                        new_cfg.tick_hz,
                                        new_cfg.slot_mode,
                                        new_cfg.fast_max_bytes
                                    );

                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };

                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User1,
                                        MavResult::Accepted,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
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
                                    let ack_cfg = MavEndpointConfig {
                                        sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                        comp_id: RADIO_COMP_ID,
                                    };
                                    let ack = build_command_ack_frame(
                                        ack_cfg,
                                        ack_seq,
                                        MavCmd::User1,
                                        MavResult::Unsupported,
                                        sender_system,
                                        sender_component,
                                    );
                                    ack_seq = ack_seq.wrapping_add(1);
                                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                                    if let Ok(n) = ack.serialize(&mut ser) {
                                        let mut ack_msg: FrameMsg = FrameMsg::new();
                                        if ack_msg.extend_from_slice(&ser[..n]).is_ok() {
                                            let _ = mac.enqueue(EnqueueClass::Reliable, &ack_msg);
                                        }
                                    }
                                }
                                LinkMacConfigCmdParse::NotForUs | LinkMacConfigCmdParse::NotLinkMacConfig => {
                                    if let Some(msg_id) = mavlink2_msg_id(inner) {
                                        if let Some(seq) = fast_seq_from_mavlink_frame(inner, msg_id) {
                                            let missed = fast_seq_lora.observe(msg_id, seq);
                                            missed_fast_seq_lora =
                                                missed_fast_seq_lora.wrapping_add(missed);
                                        }
                                    }
                                    let mut msg: FrameMsg = FrameMsg::new();
                                    if msg.extend_from_slice(inner).is_ok() {
                                        if LORA_TO_UART.sender().try_send(msg).is_err() {
                                            warn!("LoRa->UART queue full (dropping)");
                                        }
                                    }
                                }
                                }
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => warn!("LoRa RX error: {:?}", e),
                }

                // RF txn housekeeping (timeouts and scheduled apply).
                let now_ms = Instant::now().as_millis() as u32;
                if let Some(st) = staged_rf {
                    // Drop uncommitted staging after guard timeout.
                    if !st.committed && now_ms.wrapping_sub(st.proposed_at_ms) >= RF_TXN_GUARD_MS {
                        staged_rf = None;
                        rf_last_result = 4; // Timeout
                    }
                }

                // Apply pending RF config (async).
                if let Some(cfg) = pending_apply.take() {
                    // Enforce LAS lockout at the actual apply moment too.
                    if las_allows_rf_change(now_ms) {
                        if let Err(e) = link.apply_radio_config(delay, cfg).await {
                            warn!("RF reconfig apply failed: {:?}", e);
                            rf_last_result = 3; // Failed
                            let ack_cfg = MavEndpointConfig {
                                sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                comp_id: RADIO_COMP_ID,
                            };
                            let st = build_statustext("RF: APPLY failed");
                            let st_frame = build_statustext_frame(ack_cfg, ack_seq, st);
                            ack_seq = ack_seq.wrapping_add(1);
                            let mut ser = [0u8; MAVLINK_MAX_FRAME];
                            if let Ok(n) = st_frame.serialize(&mut ser) {
                                let mut st_msg: FrameMsg = FrameMsg::new();
                                if st_msg.extend_from_slice(&ser[..n]).is_ok() {
                                    let _ = mac.enqueue(EnqueueClass::Reliable, &st_msg);
                                }
                            }
                        } else {
                            current_rf_cfg = cfg;
                            rf_last_result = 1; // Ok
                            info!(
                                "RF reconfig applied: f={}Hz sf={} bw=0x{:02X} cr=0x{:02X} sw=0x{:04X}",
                                current_rf_cfg.freq_hz,
                                current_rf_cfg.sf,
                                current_rf_cfg.bw,
                                current_rf_cfg.cr,
                                current_rf_cfg.sync_word
                            );

                            let ack_cfg = MavEndpointConfig {
                                sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                                comp_id: RADIO_COMP_ID,
                            };
                            let st = build_statustext("RF: APPLY ok");
                            let st_frame = build_statustext_frame(ack_cfg, ack_seq, st);
                            ack_seq = ack_seq.wrapping_add(1);
                            let mut ser = [0u8; MAVLINK_MAX_FRAME];
                            if let Ok(n) = st_frame.serialize(&mut ser) {
                                let mut st_msg: FrameMsg = FrameMsg::new();
                                if st_msg.extend_from_slice(&ser[..n]).is_ok() {
                                    let _ = mac.enqueue(EnqueueClass::Reliable, &st_msg);
                                }
                            }
                        }
                    } else {
                        warn!("RF reconfig apply denied (LAS lockout)");
                        rf_last_result = 2; // Denied
                        let ack_cfg = MavEndpointConfig {
                            sys_id: LAST_FC_SYS_ID.load(Ordering::Relaxed),
                            comp_id: RADIO_COMP_ID,
                        };
                        let st = build_statustext("RF: APPLY denied (LAS)");
                        let st_frame = build_statustext_frame(ack_cfg, ack_seq, st);
                        ack_seq = ack_seq.wrapping_add(1);
                        let mut ser = [0u8; MAVLINK_MAX_FRAME];
                        if let Ok(n) = st_frame.serialize(&mut ser) {
                            let mut st_msg: FrameMsg = FrameMsg::new();
                            if st_msg.extend_from_slice(&ser[..n]).is_ok() {
                                let _ = mac.enqueue(EnqueueClass::Reliable, &st_msg);
                            }
                        }
                    }
                }

                // Periodically enqueue RADIO_STATUS + LINK_METRICS so GS can forward link stats upward.
                let now_ms = Instant::now().as_millis() as u32;
                if now_ms.wrapping_sub(last_radio_status_ms) >= 1_000 {
                    last_radio_status_ms = now_ms;

                    let sys_id = LAST_FC_SYS_ID.load(Ordering::Relaxed);
                    // Arbitrary-but-stable component id for the radio bridge.
                    const RADIO_COMP_ID: u8 = 190;

                    let stats = link.stats();
                    let (has_fast, rel_len, norm_len) = mac.queue_depths();
                    let queued = (has_fast as usize) + rel_len + norm_len;
                    let total_cap = 1 + 8 + 8; // FAST(1) + LinkMac<8,8>

                    // RADIO_STATUS packing scheme (no custom MAVLink dialect):
                    //
                    // Human-meaningful fields:
                    // - rssi: last RX RSSI (dBm), stored as int8_t in a u8 slot.
                    // - noise: repurposed to carry SNR (dB) because RADIO_STATUS has no SNR.
                    // - txbuf: repurposed for packed metrics (see below)
                    //
                    // Packed fields (stable mapping; see docs/wireless-telemetry-ack-design.md):
                    // - remrssi:
                    //     bits 0..2: RELIABLE queue depth (0..7, saturating)
                    //     bits 3..5: NORMAL   queue depth (0..7, saturating)
                    //     bits 6..7: slot_mode (0=UuuD, 1=UdUd, 2=Uddd)
                    // - remnoise: packed deltas (see below)
                    // - rxerrors:
                    //     low  byte: tick_hz (approx = 1000 / tick_interval_ms, 0..255)
                    //     high byte: bit7=FAST present, bits4..6=rf_last_result, bits0..3=rf_txn_state
                    // - fixed: dropped_reliable_overflow (high byte) and dropped_normal_overflow (low byte)
                    //          as deltas over last report (0..255 each)

                    let counters = mac.counters();

                    // 4-bit deltas (0..15, saturating) for compact always-on telemetry.
                    let fast_replaced_delta_4b: u8 = counters
                        .fast_replaced
                        .saturating_sub(last_fast_replaced)
                        .min(0x0F) as u8;
                    last_fast_replaced = counters.fast_replaced;

                    let mtu_drop_delta_4b: u8 = counters
                        .dropped_too_large
                        .saturating_sub(last_dropped_too_large)
                        .min(0x0F) as u8;
                    last_dropped_too_large = counters.dropped_too_large;

                    let missed_tick_delta_4b: u8 = missed_master_ticks
                        .saturating_sub(last_missed_master_ticks)
                        .min(0x0F) as u8;
                    last_missed_master_ticks = missed_master_ticks;

                    let drop_rel_delta: u8 = counters
                        .dropped_reliable_overflow
                        .saturating_sub(last_drop_rel_overflow)
                        .min(0xFF) as u8;
                    last_drop_rel_overflow = counters.dropped_reliable_overflow;

                    let drop_norm_delta: u8 = counters
                        .dropped_normal_overflow
                        .saturating_sub(last_drop_norm_overflow)
                        .min(0xFF) as u8;
                    last_drop_norm_overflow = counters.dropped_normal_overflow;

                    let rel_3b: u8 = (rel_len.min(7)) as u8;
                    let norm_3b: u8 = (norm_len.min(7)) as u8;
                    let slot_mode_2b: u8 = match mac.slot_mode() {
                        SlotMode::UuuD => 0,
                        SlotMode::UdUd => 1,
                        SlotMode::Uddd => 2,
                    };
                    let packed_queue_and_mode: u8 = rel_3b | (norm_3b << 3) | (slot_mode_2b << 6);

                    let tick_interval_ms = mac.tick_interval_ms().max(1);
                    let tick_hz_u8: u8 = ((1000 / tick_interval_ms).min(0xFF)) as u8;

                    // RF txn state (stable, machine-parsable).
                    // 0=Idle, 1=Staged, 2=Committed(waiting), 3=ApplyPending
                    let rf_txn_state: u8 = if pending_apply.is_some() {
                        3
                    } else if let Some(st) = staged_rf {
                        if st.committed { 2 } else { 1 }
                    } else {
                        0
                    };
                    let rf_apply_at_tick: u32 = staged_rf
                        .map(|st| st.apply_at_tick_seq as u32)
                        .unwrap_or(0);

                    // Pack: [7]=has_fast, [6..4]=last_result, [3..0]=state
                    let packed_rxerrors_hi: u8 = ((has_fast as u8) << 7) | ((rf_last_result & 0x07) << 4) | (rf_txn_state & 0x0F);
                    let packed_rxerrors: u16 = ((packed_rxerrors_hi as u16) << 8) | (tick_hz_u8 as u16);

                    let packed_fixed: u16 = ((drop_rel_delta as u16) << 8) | (drop_norm_delta as u16);

                    // Pack low-resolution "always-on" deltas into spare bytes.
                    // txbuf: low nibble = queue fill (0..15), high nibble = missed master ticks delta (0..15)
                    let queue_fill_q4: u8 = ((queued * 15) / total_cap).min(15) as u8;
                    let packed_txbuf: u8 = (missed_tick_delta_4b << 4) | (queue_fill_q4 & 0x0F);

                    // remnoise: low nibble = fast_replaced delta (0..15), high nibble = MTU drops delta (0..15)
                    let packed_remnoise: u8 = (mtu_drop_delta_4b << 4) | (fast_replaced_delta_4b & 0x0F);

                    let missed_fast_seq_uart = FAST_SEQ_MISSED_UART.load(Ordering::Relaxed);
                    let metrics = LinkMetrics {
                        rx_packets: stats.rx_packets,
                        tx_packets: stats.tx_packets,
                        q_fast: has_fast as u32,
                        q_reliable: rel_len as u32,
                        q_normal: norm_len as u32,
                        enq_fast: counters.enqueued_fast,
                        enq_reliable: counters.enqueued_reliable,
                        enq_normal: counters.enqueued_normal,
                        tx_fast: counters.tx_fast,
                        tx_reliable: counters.tx_reliable,
                        tx_normal: counters.tx_normal,
                        drop_fast_replaced: counters.fast_replaced,
                        drop_reliable_overflow: counters.dropped_reliable_overflow,
                        drop_normal_overflow: counters.dropped_normal_overflow,
                        drop_too_large: counters.dropped_too_large,
                        missed_master_ticks,
                        missed_fast_seq_rx: missed_fast_seq_lora,
                        missed_fast_seq_tx: missed_fast_seq_uart,
                        tx_inhibit_desync: counters.tx_inhibit_desync,
                        rf_txn_state: rf_txn_state as u32,
                        rf_last_result: rf_last_result as u32,
                        rf_apply_at_tick,
                        tick_hz: tick_hz_u8 as u32,
                        slot_mode: slot_mode_2b as u32,
                    };
                    let metrics_frame = build_link_metrics_frame(
                        MavEndpointConfig {
                            sys_id,
                            comp_id: RADIO_COMP_ID,
                        },
                        link_metrics_seq,
                        &metrics,
                    );
                    link_metrics_seq = link_metrics_seq.wrapping_add(1);

                    let mut metrics_ser = [0u8; MAVLINK_MAX_FRAME];
                    if let Ok(n) = metrics_frame.serialize(&mut metrics_ser) {
                        let mut msg: FrameMsg = FrameMsg::new();
                        if msg.extend_from_slice(&metrics_ser[..n]).is_ok() {
                            let _ = mac.enqueue(EnqueueClass::Normal, &msg);
                        }
                    }

                    let frame = build_radio_status_frame(
                        MavEndpointConfig {
                            sys_id,
                            comp_id: RADIO_COMP_ID,
                        },
                        radio_status_seq,
                        stats.last_rssi_dbm,
                        stats.last_snr_x4,
                        packed_queue_and_mode,
                        packed_txbuf,
                        packed_remnoise,
                        packed_rxerrors,
                        packed_fixed,
                    );
                    radio_status_seq = radio_status_seq.wrapping_add(1);

                    let mut ser = [0u8; MAVLINK_MAX_FRAME];
                    if let Ok(n) = frame.serialize(&mut ser) {
                        let mut msg: FrameMsg = FrameMsg::new();
                        if msg.extend_from_slice(&ser[..n]).is_ok() {
                            let _ = mac.enqueue(EnqueueClass::Normal, &msg);
                        }
                    }
                }

                // MAC tick: Radio transmits only on its TX slots.
                // Apply runtime config updates (tick_hz / slot_mode) without reflashing.
                let cfg_tick_hz = MAC_TICK_HZ.load(Ordering::Relaxed);
                let cfg_slot_mode = MAC_SLOT_MODE.load(Ordering::Relaxed);
                if cfg_slot_mode != last_applied_slot_mode {
                    let sm = match cfg_slot_mode {
                        0 => SlotMode::UuuD,
                        2 => SlotMode::Uddd,
                        _ => SlotMode::UdUd,
                    };
                    mac.set_slot_mode(sm);
                    last_applied_slot_mode = cfg_slot_mode;
                }
                // Track last applied tick by remembering the intended Hz.
                // (We don't store Hz in mac; tick_interval_ms is the internal form.)
                if cfg_tick_hz != last_applied_tick_hz {
                    mac.set_tick_hz(Instant::now().as_millis() as u32, cfg_tick_hz);
                    last_applied_tick_hz = cfg_tick_hz;
                }

                let now_ms = Instant::now().as_millis() as u32;
                match mac.poll_tick(now_ms) {
                    TickResult::TxSlot { tick_seq, payload, .. } => {
                        // Tick-aligned RF apply scheduling: trigger on the exact tick boundary.
                        if let Some(st) = staged_rf {
                            if st.committed && tick_seq == st.apply_at_tick_seq && pending_apply.is_none() {
                                pending_apply = Some(st.cfg);
                                staged_rf = None;
                            }
                        }

                        if let Some(inner) = payload {
                            // Robustness: never transmit unless we've heard the GS-master recently.
                            let sync_max_age_ms = mac.tick_interval_ms().saturating_mul(8).max(200);
                            if !mac.master_synced_within_ms(now_ms, sync_max_age_ms) {
                                mac.record_tx_inhibit_desync();
                                continue;
                            }

                            let mut tx = heapless::Vec::<u8, { common::coms::transport::lora::mac::LORA_MAC_MTU }>::new();
                            if wrap_mac_packet(&mut tx, tick_seq, true, &inner).is_ok() {
                                if let Err(e) = link.send_unreliable(delay, &tx).await {
                                    warn!("LoRa TX error: {:?}", e);
                                }
                            }
                        }
                    }
                    TickResult::RxSlot { tick_seq, .. } => {
                        // Apply can be scheduled for any tick boundary, even if this node doesn't TX.
                        if let Some(st) = staged_rf {
                            if st.committed && tick_seq == st.apply_at_tick_seq && pending_apply.is_none() {
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

fn handle_rf_reconfig_cmd(
    now_ms: u32,
    tick_base: u8,
    tick_interval_ms: u32,
    cmd: RfReconfigCommand,
    _sender_system: u8,
    _sender_component: u8,
    staged_rf: &mut Option<StagedRfTxn>,
    pending_apply: &mut Option<LoRaConfig>,
    current_rf_cfg: &mut LoRaConfig,
) -> MavResult {
    // Safety gate: only allow when explicitly DISARMED and fresh.
    let las_ok = las_allows_rf_change(now_ms);

    match cmd.op {
        RfReconfigOp::Propose => {
            if !las_ok {
                return MavResult::Denied;
            }
            let Some(settings) = cmd.settings else {
                return MavResult::Failed;
            };
            let next = settings.apply_to_base(*current_rf_cfg);

            let apply_after_ticks = ms_to_ticks_ceil(cmd.apply_after_ms, tick_interval_ms);
            let apply_at_tick_seq = tick_base.wrapping_add(apply_after_ticks);

            *staged_rf = Some(StagedRfTxn {
                txn_id: cmd.txn_id,
                proposed_at_ms: now_ms,
                apply_at_tick_seq,
                committed: false,
                cfg: next,
            });

            MavResult::Accepted
        }
        RfReconfigOp::Commit => {
            if !las_ok {
                return MavResult::Denied;
            }
            let Some(mut st) = *staged_rf else {
                return MavResult::Failed;
            };
            if st.txn_id != cmd.txn_id {
                return MavResult::Failed;
            }

            // Guard timeout: commit must arrive soon after propose.
            if now_ms.wrapping_sub(st.proposed_at_ms) >= RF_TXN_GUARD_MS {
                *staged_rf = None;
                return MavResult::Failed;
            }

            st.committed = true;

            // If the commit arrives after the scheduled apply tick, apply on the next tick boundary.
            // Wrap-safe "past" check in u8 space: diff < 128 means tick_base is after apply_at_tick_seq.
            let diff = tick_base.wrapping_sub(st.apply_at_tick_seq);
            if diff != 0 && diff < 128 {
                st.apply_at_tick_seq = tick_base.wrapping_add(1);
            }

            // Do not apply immediately here; apply is triggered tick-aligned in the main loop.
            // (If we missed the tick, we rescheduled above.)
            *staged_rf = Some(st);

            MavResult::Accepted
        }
        RfReconfigOp::Abort => {
            // Abort is allowed even if LAS is stale/unknown; it only reduces risk.
            if let Some(st) = staged_rf {
                if st.txn_id == cmd.txn_id {
                    *staged_rf = None;
                }
            }
            *pending_apply = None;
            MavResult::Accepted
        }
    }
}

// MAC wrap/unwrap helpers live in `common::coms::transport::lora::mac_codec`.

const MAVLINK2_HEADER_LEN: usize = 10;
const MAVLINK_MSG_ID_RAW_IMU: u32 = 27;
const MAVLINK_MSG_ID_FAST_RC: u32 = 42_000;
const MAVLINK_MSG_ID_FAST_ATTITUDE_RATES: u32 = 42_001;
// MAVLink wire ordering places larger fields first.
const FAST_RC_SEQ_OFFSET: usize = 16 * 2;
const FAST_ATTITUDE_RATES_SEQ_OFFSET: usize = 6 * 2;

fn mavlink2_msg_id(frame_bytes: &[u8]) -> Option<u32> {
    // MAVLink2 header: magic(0), len(1), incompat(2), compat(3), seq(4), sys(5), comp(6), msgid(7..9)
    if frame_bytes.len() < MAVLINK2_HEADER_LEN || frame_bytes[0] != 0xFD {
        return None;
    }
    let id0 = frame_bytes[7] as u32;
    let id1 = frame_bytes[8] as u32;
    let id2 = frame_bytes[9] as u32;
    Some(id0 | (id1 << 8) | (id2 << 16))
}

fn fast_seq_offset_for_msg_id(msg_id: u32) -> Option<usize> {
    match msg_id {
        MAVLINK_MSG_ID_FAST_RC => Some(FAST_RC_SEQ_OFFSET),
        MAVLINK_MSG_ID_FAST_ATTITUDE_RATES => Some(FAST_ATTITUDE_RATES_SEQ_OFFSET),
        _ => None,
    }
}

fn fast_seq_from_mavlink_frame(frame_bytes: &[u8], msg_id: u32) -> Option<u8> {
    let offset = fast_seq_offset_for_msg_id(msg_id)?;
    if frame_bytes.len() < MAVLINK2_HEADER_LEN {
        return None;
    }
    let payload_len = frame_bytes[1] as usize;
    if payload_len <= offset {
        return None;
    }
    let payload_start = MAVLINK2_HEADER_LEN;
    if frame_bytes.len() < payload_start + payload_len {
        return None;
    }
    Some(frame_bytes[payload_start + offset])
}

#[derive(Default)]
struct FastSeqTracker {
    last_fast_rc: Option<u8>,
    last_fast_attitude_rates: Option<u8>,
}

impl FastSeqTracker {
    fn observe(&mut self, msg_id: u32, seq: u8) -> u32 {
        let last = match msg_id {
            MAVLINK_MSG_ID_FAST_RC => &mut self.last_fast_rc,
            MAVLINK_MSG_ID_FAST_ATTITUDE_RATES => &mut self.last_fast_attitude_rates,
            _ => return 0,
        };

        let missed = if let Some(prev) = *last {
            let diff = seq.wrapping_sub(prev);
            if diff != 0 {
                diff.wrapping_sub(1) as u32
            } else {
                0
            }
        } else {
            0
        };

        *last = Some(seq);
        missed
    }
}

fn is_fast_lane_msg_id(msg_id: u32) -> bool {
    // Transitional FAST telemetry carrier: RAW_IMU fits within inner MTU.
    msg_id == MAVLINK_MSG_ID_RAW_IMU
        || msg_id == MAVLINK_MSG_ID_FAST_RC
        || msg_id == MAVLINK_MSG_ID_FAST_ATTITUDE_RATES
}
