//! The station's flight commands, mode and heartbeat between the links on core 0 and the hands-off state machine in the
//! flight step on core 1 (link-contract section 10, Amendments A and C).
//!
//! Commands: core 0 ([`command`], from the nodes' `Telemetry::command_deferrable` through `actions`) decodes; a flight
//! command is posted to [`FLIGHT_CMD`] and the node sends no COMMAND_ACK then. Core 0 checks the command from the link's
//! `Telemetry::acks` ([`next_ack`], every `poll_transmit` of that link's node, at least every 100 ms) and offers the one
//! COMMAND_ACK, ACCEPTED or DENIED with the reason code in `result_param2`, to the sender on the link the command came
//! on. The flight step has [`BOUND_US`] to take it; not taken by then (withdrawn, never applied), another flight command
//! outstanding, or parameters not in the table: DENIED, code 0. A command taken is answered only with the flight step's
//! answer, waited for until it comes or the link goes down (then no ACK, see [`link_lost`]). Core 1 ([`before_step`], [`after_step`], from
//! `flight::Stepper::sample`): the request, taken at most once, for the step that follows; that step's answer back.
//!
//! Mode: after each step core 1 stores the state machine's mode code in [`MODE`] (0 until the first step); both nodes put
//! it in HEARTBEAT `custom_mode` ([`custom_mode`]).
//!
//! Heartbeat and plan: core 0 writes the RC port's time of the latest station HEARTBEAT on the radio link, from the
//! station's sender only ([`FLIGHT_STATION_HB`], the clock of `rc::FLIGHT_RADIO_STATUS`) and the committed list's plan id ([`FLIGHT_PLAN`],
//! 0 for no list); core 1 gives them to `Flight::set_last_radio_heartbeat_us` and `Flight::set_plan_id`.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::flight_cmd::{Decoded, Exchange, Mailbox, Pending, decode, request_byte, station_heartbeat_us};
use fw_core::xcore::{TripleBuffer, TripleReader, TripleWriter};
use fw_flight::gnc::{Allocator, Controller, Estimator};
use fw_flight::hands_off::{Command, CommandResult};
use fw_flight::step::Flight;
use mavlink_device::marv::{MavCmd, MavResult};
use mavlink_device::{CommandOutcome, CommandRequest, LateAck, Peer};

/// Core 0 -> flight step and back: the one flight command in flight.
pub static FLIGHT_CMD: Mailbox = Mailbox::new();

/// How long the flight step has to take a command, us (its answer, given in the step that takes it, has no bound). The
/// station sends again after 1.5 s without an ACK; the ACK goes out at the node's next `poll_transmit` after the answer
/// (within 100 ms).
pub const BOUND_US: u64 = 50_000;

/// Flight step -> core 0: the hands-off state's mode code after the latest step; 0 before the first.
pub static MODE: AtomicU8 = AtomicU8::new(0);

/// Core 0 -> flight step: the RC port's time of the latest ground-station HEARTBEAT on the radio link, us (written only
/// when it is new; nothing before the first).
pub static FLIGHT_STATION_HB: TripleBuffer<u64> = TripleBuffer::new(0);

/// Core 0 -> flight step: the plan id of the committed list; 0 when none is stored (the empty list's plan id).
pub static FLIGHT_PLAN: TripleBuffer<u32> = TripleBuffer::new(0);

/// Core 0 -> flight step's log: the sender of the RC port's latest ground-station HEARTBEAT (any sender of type GCS) as
/// `0x1_0000 | system << 8 | component`; 0 before the first. Logged only ([`gcs_sender`]); nothing decides on it.
pub static GCS_SENDER: AtomicU32 = AtomicU32::new(0);

/// The sender in [`GCS_SENDER`]; `None` before the first.
pub fn gcs_sender() -> Option<(u8, u8)> {
    let v = GCS_SENDER.load(Ordering::Relaxed);
    (v != 0).then_some(((v >> 8) as u8, v as u8))
}

/// Core 0: the flight command outstanding, its link and its COMMAND_ACK's command number and target.
#[derive(Clone, Copy)]
struct Outstanding {
    pending: Pending,
    link: u8,
    command: MavCmd,
    target: Peer,
}

static OUTSTANDING: Mutex<CriticalSectionRawMutex, Cell<Option<Outstanding>>> = Mutex::new(Cell::new(None));

/// Core 0: the writers of [`FLIGHT_STATION_HB`] (with the station's own last time, the one written) and [`FLIGHT_PLAN`],
/// taken at first use.
struct Writers {
    hb: Option<(TripleWriter<'static, u64>, Option<u64>)>,
    plan: Option<TripleWriter<'static, u32>>,
}

static WRITERS: Mutex<CriticalSectionRawMutex, RefCell<Writers>> =
    Mutex::new(RefCell::new(Writers { hb: None, plan: None }));

/// Core 0: `None` for every command but the five (it goes on as before); `Some(None)`: posted, its ACK follows through
/// [`next_ack`]; `Some(Some(ack))`: DENIED code 0 at once.
pub fn command(req: &CommandRequest, link: u8) -> Option<Option<CommandOutcome>> {
    let c = match decode(req.command, &req.params) {
        Decoded::Other => return None,
        Decoded::BadParams => {
            defmt::info!("[INFO] flight command {=u32} on link {=u8}: DENIED, parameters not in the table", req.command as u32, link);
            return Some(Some(outcome(0)));
        }
        Decoded::Command(c) => c,
    };
    let radio = link == crate::LINK_RADIO as u8;
    let posted = OUTSTANDING.lock(|o| {
        let pending = Pending::post(&FLIGHT_CMD, request_byte(c, radio), crate::now_us())?;
        o.set(Some(Outstanding {
            pending,
            link,
            command: req.command,
            target: req.sender,
        }));
        Some(())
    });
    if posted.is_none() {
        defmt::warn!("[INFO] flight command {=u32} on link {=u8}: DENIED, another one outstanding", req.command as u32, link);
        return Some(Some(outcome(0)));
    }
    defmt::info!("[INFO] flight command {=u32} on link {=u8}: posted to the flight step", req.command as u32, link);
    Some(None)
}

/// Core 0, from link `link`'s `Telemetry::acks`: the ACK of the flight command outstanding on that link, once it is
/// answered or timed out.
pub fn next_ack(link: u8) -> Option<LateAck> {
    let (o, result) = OUTSTANDING.lock(|cell| {
        let mut o = cell.take()?;
        if o.link != link {
            cell.set(Some(o));
            return None;
        }
        match o.pending.check(&FLIGHT_CMD, crate::now_us(), BOUND_US) {
            Some(r) => Some((o, r)),
            None => {
                cell.set(Some(o));
                None
            }
        }
    })?;
    let byte = match result {
        Exchange::Answered(b) => b,
        Exchange::Timeout => {
            defmt::warn!(
                "[INFO] flight command {=u32} on link {=u8}: DENIED, not taken by the flight step (withdrawn)",
                o.command as u32,
                link
            );
            0
        }
    };
    defmt::info!("[INFO] flight command {=u32} on link {=u8}: answer {=u8}", o.command as u32, link, byte);
    Some(LateAck {
        command: o.command,
        target: o.target,
        outcome: outcome(byte),
    })
}

/// Core 0: link `link` went down; its outstanding flight command, if any, is dropped (withdrawn if not taken yet).
pub fn link_lost(link: u8) {
    OUTSTANDING.lock(|cell| match cell.take() {
        Some(o) if o.link == link => o.pending.abandon(&FLIGHT_CMD),
        other => cell.set(other),
    });
}

fn outcome(byte: u8) -> CommandOutcome {
    let (accepted, code) = fw_flight::station::ack_of(byte);
    CommandOutcome {
        result: if accepted {
            MavResult::MAV_RESULT_ACCEPTED
        } else {
            MavResult::MAV_RESULT_DENIED
        },
        progress: 0,
        result_param2: code,
    }
}

/// Core 0, every node's `Telemetry::custom_mode`: the mode code after the latest flight step (0 before the first).
pub fn custom_mode() -> u32 {
    u32::from(MODE.load(Ordering::Relaxed))
}

/// Core 0, from the RC port's `Hooks::stats`: its time and sender of the latest ground-station HEARTBEAT (`None` before
/// the first); the time counts only from the station's sender (`station_heartbeat_us`) and is written to the flight step
/// when it is new.
pub fn station_heartbeat(t_us: Option<u64>, sender: Option<(u8, u8)>) {
    if let Some((sys, comp)) = sender {
        GCS_SENDER.store(0x1_0000 | u32::from(sys) << 8 | u32::from(comp), Ordering::Relaxed);
    }
    WRITERS.lock(|w| {
        let mut w = w.borrow_mut();
        if w.hb.is_none() {
            w.hb = FLIGHT_STATION_HB.take_writer().map(|wr| (wr, None));
        }
        if let Some((wr, last)) = &mut w.hb
            && let Some(t) = station_heartbeat_us(*last, t_us, sender)
            && *last != Some(t)
        {
            wr.write(t);
            *last = Some(t);
        }
    });
}

/// Core 0, on a change of the committed list: its plan id to the flight step (0 when it is empty).
pub fn push_plan() {
    let (_, len, _, plan_id) = crate::mission::committed();
    WRITERS.lock(|w| {
        let mut w = w.borrow_mut();
        if w.plan.is_none() {
            w.plan = FLIGHT_PLAN.take_writer();
        }
        if let Some(wr) = &mut w.plan {
            wr.write(if len == 0 { 0 } else { plan_id });
        }
    });
}

/// Core 1: the readers of [`FLIGHT_STATION_HB`] and [`FLIGHT_PLAN`].
pub struct Core1 {
    hb: TripleReader<'static, u64>,
    plan: TripleReader<'static, u32>,
}

impl Core1 {
    /// `None` if a reader was taken already.
    pub fn new() -> Option<Self> {
        Some(Self {
            hb: FLIGHT_STATION_HB.take_reader()?,
            plan: FLIGHT_PLAN.take_reader()?,
        })
    }
}

/// Core 1, before `Flight::step`: the station heartbeat time and the plan id when new, and the waiting command, given to
/// the flight for that step; the command's number and the command.
pub fn before_step<E: Estimator, C: Controller, A: Allocator>(
    io: &mut Core1,
    flight: &mut Flight<E, C, A>,
) -> Option<(u32, Command)> {
    let (hb, hb_fresh) = io.hb.read();
    if hb_fresh {
        flight.set_last_radio_heartbeat_us(Some(hb));
    }
    let (plan, plan_fresh) = io.plan.read();
    if plan_fresh {
        flight.set_plan_id(fw_flight::station::plan_id_of(plan));
    }
    let (seq, c) = fw_flight::station::take(&FLIGHT_CMD)?;
    flight.set_command(c);
    Some((seq, c.command))
}

/// Core 1, after that step: its answer to request `seq`, and the mode code; the command and the answer, for the log.
pub fn after_step<E: Estimator, C: Controller, A: Allocator>(
    flight: &mut Flight<E, C, A>,
    request: Option<(u32, Command)>,
) -> Option<(Command, Option<CommandResult>)> {
    let answered = request.map(|(seq, command)| {
        let result = flight.take_command_result();
        fw_flight::station::answer(&FLIGHT_CMD, seq, result);
        (command, result)
    });
    MODE.store(flight.hands_off_state().code(), Ordering::Relaxed);
    answered
}
