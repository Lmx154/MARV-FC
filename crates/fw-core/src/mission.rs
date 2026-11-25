//! The vehicle's side of the mission (waypoint list) service: MARV-GCS `docs/link-contract.md` section 17
//! (MAVLink's mission protocol cut down to six messages, plan type 0).
//!
//! [`MissionService`] is pure: no I/O, no clock of its own (time is passed in as microseconds), no
//! allocation and no dependency on the MAVLink crates. Its inputs are the plain [`Frame`] and [`Rx`]
//! values that mirror the fields of the six messages (MISSION_COUNT, MISSION_REQUEST_INT,
//! MISSION_ITEM_INT, MISSION_ACK, MISSION_REQUEST_LIST, MISSION_CLEAR_ALL); its outputs are [`Tx`]
//! values: "send this message to this system and component on this link". At most one output
//! comes from each [`MissionService::handle`] and each [`MissionService::poll`].
//!
//! It holds the committed list (up to 16 waypoints and the return item, last, of 12 bytes each: `x`, `y` and the 4 bytes of `z`), a staging
//! copy that replaces it in one step on the last valid item, and one transaction at a time across
//! both links, owned by (link, system id, component id). The reply slot and its pacing, the
//! accept list, the frame encoding and decoding and the critical-section static are the
//! board's; the contract's reply-pacing rule (one slot per link) is not modelled here: a message
//! is "handed to the slot" when it is returned, and the timers start at that call's time.
//!
//! Where the contract is not explicit (none changes a number or a code):
//! - A re-request caused by a wrong item restarts the 1500 ms timer and leaves the retry count
//!   as it was (it is not counted, and it does not reset the count either).
//! - The download's 9000 ms run from the last mission message of the owner that passed the
//!   addressing check, whatever the message was (the start message included, a dropped item too).
//! - The addressing rule is applied here ([`Frame::target_system`], [`Frame::target_component`]):
//!   a frame for any other target is dropped without any effect.

#[cfg(test)]
use crate::record::crc32;
use crate::record::crc32_update;

/// Waypoints the list holds.
pub const CAPACITY: usize = 16;
/// Bytes of one stored waypoint: `x`, `y` and `z` as little-endian 32-bit values.
pub const ITEM_BYTES: usize = 12;
/// The `frame` the vehicle stores and sends: `MAV_FRAME_GLOBAL_RELATIVE_ALT`.
pub const FRAME_STORED: u8 = 3;
/// The `frame` accepted as the same frame: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
pub const FRAME_ALT_INT: u8 = 6;
/// The `command` of a waypoint: `MAV_CMD_NAV_WAYPOINT`.
pub const COMMAND_WAYPOINT: u16 = 16;
/// The return item's `command` (`MAV_CMD_NAV_RETURN_TO_LAUNCH`): the last item of every upload, its `z` the return height.
pub const COMMAND_RETURN: u16 = 20;
/// Items the list holds: up to [`CAPACITY`] waypoints and the return item.
pub const MAX_ITEMS: usize = CAPACITY + 1;
/// The default `GDN_ALT_MIN`, m: what the service holds until [`MissionService::set_limits`] is called.
pub const ALT_MIN_DEFAULT: f32 = 5.0;
/// The default `GDN_ALT_MAX`, m.
pub const ALT_MAX_DEFAULT: f32 = 50.0;
/// The vehicle's system id: a frame is handled only when its target system is 0 or this.
pub const VEHICLE_SYSTEM: u8 = 1;
/// The vehicle's component id: a frame is handled only when its target component is 0 or this.
pub const VEHICLE_COMPONENT: u8 = 1;
/// A request is sent again this long after its previous send (1500 ms).
pub const RETRY_US: u64 = 1_500_000;
/// Retries of one request (6 sends); 1500 ms after the sixth the vehicle gives up.
pub const MAX_RETRIES: u8 = 5;
/// A download ends after this long (9000 ms) without a mission message from its owner.
pub const DOWNLOAD_IDLE_US: u64 = 9_000_000;

/// `MAV_MISSION_RESULT` values the vehicle sends.
pub mod result {
    /// `MAV_MISSION_ACCEPTED`.
    pub const ACCEPTED: u8 = 0;
    /// `MAV_MISSION_ERROR`.
    pub const ERROR: u8 = 1;
    /// `MAV_MISSION_UNSUPPORTED_FRAME`.
    pub const UNSUPPORTED_FRAME: u8 = 2;
    /// `MAV_MISSION_UNSUPPORTED`.
    pub const UNSUPPORTED: u8 = 3;
    /// `MAV_MISSION_NO_SPACE`.
    pub const NO_SPACE: u8 = 4;
    /// MAV_MISSION_INVALID.
    pub const INVALID: u8 = 5;
    /// `MAV_MISSION_INVALID_PARAM5_X`.
    pub const INVALID_PARAM5_X: u8 = 10;
    /// `MAV_MISSION_INVALID_PARAM6_Y`.
    pub const INVALID_PARAM6_Y: u8 = 11;
    /// `MAV_MISSION_INVALID_SEQUENCE`.
    pub const INVALID_PARAM7: u8 = 12;
    /// MAV_MISSION_INVALID_SEQUENCE.
    pub const INVALID_SEQUENCE: u8 = 13;
    /// `MAV_MISSION_DENIED`.
    pub const DENIED: u8 = 14;
    /// `MAV_MISSION_OPERATION_CANCELLED`.
    pub const OPERATION_CANCELLED: u8 = 15;
}

/// The vehicle's two links; one service serves both.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// The USB serial link.
    Usb,
    /// The radio link.
    Radio,
}

/// Who owns a transaction: the link, system id and component id of the frame that started it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Owner {
    /// The link the frame came from.
    pub link: Link,
    /// The sender's system id (frame header).
    pub system: u8,
    /// The sender's component id (frame header).
    pub component: u8,
}

/// One stored waypoint (`frame` 3 and `command` 16 are not stored).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Item {
    /// Latitude in degrees x 10^7.
    pub x: i32,
    /// Longitude in degrees x 10^7.
    pub y: i32,
    /// The 4 bytes of `z` (IEEE-754 single-precision bits), stored unchanged and never used.
    pub z_bits: u32,
}

impl Item {
    const ZERO: Self = Self { x: 0, y: 0, z_bits: 0 };

    /// The 12 bytes the plan id covers: `x`, `y`, `z` little-endian.
    pub fn bytes(&self) -> [u8; ITEM_BYTES] {
        let mut out = [0u8; ITEM_BYTES];
        let x = self.x.to_le_bytes();
        let y = self.y.to_le_bytes();
        let z = self.z_bits.to_le_bytes();
        for (dst, src) in out.iter_mut().zip(x.iter().chain(y.iter()).chain(z.iter())) {
            *dst = *src;
        }
        out
    }
}

/// A MISSION_ITEM_INT as the vehicle checks it (`current`, `autocontinue` and `param1` to `param4` are ignored by the contract and not carried).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxItem {
    /// `seq`.
    pub seq: u16,
    /// `frame`.
    pub frame: u8,
    /// `command`.
    pub command: u16,
    /// `x`, latitude in degrees x 10^7.
    pub x: i32,
    /// `y`, longitude in degrees x 10^7.
    pub y: i32,
    /// `z` as IEEE-754 single-precision bits.
    pub z_bits: u32,
    /// `mission_type`.
    pub mission_type: u8,
}

/// A mission message received from the station, with the fields the vehicle looks at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rx {
    /// MISSION_COUNT (`opaque_id` is ignored).
    Count {
        /// `count`.
        count: u16,
        /// `mission_type`.
        mission_type: u8,
    },
    /// MISSION_REQUEST_LIST.
    RequestList {
        /// `mission_type`.
        mission_type: u8,
    },
    /// MISSION_CLEAR_ALL.
    ClearAll {
        /// `mission_type`.
        mission_type: u8,
    },
    /// MISSION_ACK (any `type` ends the transaction; `opaque_id` is ignored).
    Ack {
        /// `type`.
        result: u8,
        /// `mission_type`.
        mission_type: u8,
    },
    /// MISSION_REQUEST_INT.
    RequestInt {
        /// `seq`.
        seq: u16,
        /// `mission_type`.
        mission_type: u8,
    },
    /// MISSION_ITEM_INT.
    ItemInt(RxItem),
}

/// A mission frame that arrived: where it came from and where it was addressed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// The link it came from.
    pub link: Link,
    /// The sender's system id (frame header).
    pub src_system: u8,
    /// The sender's component id (frame header).
    pub src_component: u8,
    /// `target_system`.
    pub target_system: u8,
    /// `target_component`.
    pub target_component: u8,
    /// The message.
    pub msg: Rx,
}

/// A mission message the vehicle sends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxMsg {
    /// MISSION_REQUEST_INT (`mission_type` 0).
    RequestInt {
        /// `seq`.
        seq: u16,
    },
    /// MISSION_COUNT (`mission_type` 0), the answer to MISSION_REQUEST_LIST.
    Count {
        /// The number stored.
        count: u16,
        /// `opaque_id`: the plan id of the stored list.
        opaque_id: u32,
    },
    /// MISSION_ACK.
    Ack {
        /// `type`, a [`result`] value.
        result: u8,
        /// `mission_type`: that of the message answered (0 when the vehicle gives up by itself).
        mission_type: u8,
        /// `opaque_id`: the new plan id of an accepted upload, 0 in every other MISSION_ACK.
        opaque_id: u32,
    },
    /// MISSION_ITEM_INT of a download, the return item (`seq` n - 1): `command` [`COMMAND_RETURN`], `frame` [`FRAME_STORED`], `x` 0, `y` 0, `current` 0, `autocontinue` 1, `param1` to `param4` 0.0, `mission_type` 0.
    ReturnItem {
        /// `seq`.
        seq: u16,
        /// `z` as single-precision bits, as stored.
        z_bits: u32,
    },
    /// MISSION_ITEM_INT of a download: `frame` [`FRAME_STORED`], `command` [`COMMAND_WAYPOINT`], `current` 0, `autocontinue` 1, `param1` to `param4` 0.0, `mission_type` 0.
    Item {
        /// `seq`.
        seq: u16,
        /// `x`.
        x: i32,
        /// `y`.
        y: i32,
        /// `z` as single-precision bits, as stored.
        z_bits: u32,
    },
}

/// A message to send: to this system and component, on this link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tx {
    /// The link to send it on.
    pub link: Link,
    /// `target_system`: the system that sent the message answered (the owner for a timer's message).
    pub dst_system: u8,
    /// `target_component`: the component that sent the message answered.
    pub dst_component: u8,
    /// The message.
    pub msg: TxMsg,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Upload {
        /// Items announced by MISSION_COUNT (1 to 17: the waypoints and the return item).
        n: u8,
        /// The `seq` expected next.
        next: u16,
        /// Retries of the request for `next` so far.
        retries: u8,
        /// When the request for `next` was last sent (or re-requested).
        sent_us: u64,
        /// The running CRC-32 register over the accepted items' bytes (starts at `u32::MAX`).
        crc: u32,
    },
    Download {
        /// When the owner's last mission message arrived.
        last_rx_us: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Txn {
    owner: Owner,
    kind: Kind,
}

/// The last upload that ended at an item (accepted, or refused at an item): kept to answer a repeat of that item (a lost final MISSION_ACK).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LastUpload {
    owner: Owner,
    seq: u16,
    result: u8,
    mission_type: u8,
    opaque_id: u32,
}

/// The plan id of a CRC-32 over a non-empty list: 0 means "no plan", so 0 becomes 1.
fn plan_id_from_crc(crc: u32) -> u32 {
    if crc == 0 { 1 } else { crc }
}

/// The plan id of a list: the CRC-32 of each item's `x`, `y`, `z` little-endian; 0 when empty.
#[cfg(test)]
fn plan_id_of(items: &[Item]) -> u32 {
    if items.is_empty() {
        return 0;
    }
    let mut buf = [0u8; MAX_ITEMS * ITEM_BYTES];
    let mut len = 0;
    for (chunk, item) in buf.as_chunks_mut::<ITEM_BYTES>().0.iter_mut().zip(items) {
        chunk.copy_from_slice(&item.bytes());
        len += ITEM_BYTES;
    }
    plan_id_from_crc(crc32(buf.get(..len).unwrap_or(&[])))
}

/// The checks an upload item passes, in the contract's order (the `seq` check is before them).
fn validate(armed: bool, last: bool, limits: (f32, f32), it: &RxItem) -> Result<Item, u8> {
    let z = f32::from_bits(it.z_bits);
    if it.mission_type != 0 {
        Err(result::UNSUPPORTED)
    } else if armed {
        Err(result::ERROR)
    } else if last && it.command != COMMAND_RETURN {
        Err(result::INVALID)
    } else if !last && it.command == COMMAND_RETURN {
        Err(result::INVALID)
    } else if !last && it.command != COMMAND_WAYPOINT {
        Err(result::UNSUPPORTED)
    } else if it.frame != FRAME_STORED && it.frame != FRAME_ALT_INT {
        Err(result::UNSUPPORTED_FRAME)
    } else if last && it.x != 0 || !last && !(-900_000_000..=900_000_000).contains(&it.x) {
        Err(result::INVALID_PARAM5_X)
    } else if last && it.y != 0 || !last && !(-1_800_000_000..=1_800_000_000).contains(&it.y) {
        Err(result::INVALID_PARAM6_Y)
    } else if !(z.is_finite() && z >= limits.0 && z <= limits.1) {
        Err(result::INVALID_PARAM7)
    } else {
        Ok(Item { x: it.x, y: it.y, z_bits: it.z_bits })
    }
}

fn to(owner: Owner, msg: TxMsg) -> Option<Tx> {
    Some(Tx { link: owner.link, dst_system: owner.system, dst_component: owner.component, msg })
}

fn ack(owner: Owner, result: u8, mission_type: u8, opaque_id: u32) -> Option<Tx> {
    to(owner, TxMsg::Ack { result, mission_type, opaque_id })
}

/// MISSION_REQUEST_LIST.
pub const MISSION_REQUEST_LIST_ID: u32 = 43;
/// MISSION_COUNT.
pub const MISSION_COUNT_ID: u32 = 44;
/// MISSION_CLEAR_ALL.
pub const MISSION_CLEAR_ALL_ID: u32 = 45;
/// MISSION_ACK.
pub const MISSION_ACK_ID: u32 = 47;
/// MISSION_REQUEST_INT.
pub const MISSION_REQUEST_INT_ID: u32 = 51;
/// MISSION_ITEM_INT.
pub const MISSION_ITEM_INT_ID: u32 = 73;

/// Reads a MAVLink 2 payload of `message_id` as the vehicle's input: the target system, the target component and the message.
pub fn decode(message_id: u32, payload: &[u8]) -> Option<(u8, u8, Rx)> {
    fn extend<const N: usize>(p: &[u8]) -> [u8; N] {
        let mut b = [0u8; N];
        for (dst, src) in b.iter_mut().zip(p) {
            *dst = *src;
        }
        b
    }
    fn at(b: &[u8], i: usize) -> u8 {
        b.get(i).copied().unwrap_or(0)
    }
    fn le16(b: &[u8], i: usize) -> u16 {
        u16::from_le_bytes([at(b, i), at(b, i + 1)])
    }
    fn le32(b: &[u8], i: usize) -> u32 {
        u32::from_le_bytes([at(b, i), at(b, i + 1), at(b, i + 2), at(b, i + 3)])
    }
    match message_id {
        MISSION_COUNT_ID => {
            let b = extend::<9>(payload);
            Some((at(&b, 2), at(&b, 3), Rx::Count { count: le16(&b, 0), mission_type: at(&b, 4) }))
        }
        MISSION_REQUEST_LIST_ID => {
            let b = extend::<3>(payload);
            Some((at(&b, 0), at(&b, 1), Rx::RequestList { mission_type: at(&b, 2) }))
        }
        MISSION_CLEAR_ALL_ID => {
            let b = extend::<3>(payload);
            Some((at(&b, 0), at(&b, 1), Rx::ClearAll { mission_type: at(&b, 2) }))
        }
        MISSION_ACK_ID => {
            let b = extend::<8>(payload);
            Some((at(&b, 0), at(&b, 1), Rx::Ack { result: at(&b, 2), mission_type: at(&b, 3) }))
        }
        MISSION_REQUEST_INT_ID => {
            let b = extend::<5>(payload);
            Some((at(&b, 2), at(&b, 3), Rx::RequestInt { seq: le16(&b, 0), mission_type: at(&b, 4) }))
        }
        MISSION_ITEM_INT_ID => {
            let b = extend::<38>(payload);
            Some((
                at(&b, 32),
                at(&b, 33),
                Rx::ItemInt(RxItem {
                    seq: le16(&b, 28),
                    frame: at(&b, 34),
                    command: le16(&b, 30),
                    x: le32(&b, 16) as i32,
                    y: le32(&b, 20) as i32,
                    z_bits: le32(&b, 24),
                    mission_type: at(&b, 37),
                }),
            ))
        }
        _ => None,
    }
}

/// How many targets the guidance takes from the mission: all of its waypoints (not the return item).
pub const GDN_TARGETS: usize = CAPACITY;

/// The guidance's target set as core 0 pushes it to the flight core: how many targets are in use (0 is off), the arrival radius (m) and the targets as (latitude, longitude) in 1e-7 degree, the unused ones 0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GdnTargets {
    /// Targets in use, at most [`GDN_TARGETS`].
    pub count: u8,
    /// The arrival radius, m.
    pub radius_m: f32,
    /// The targets as (latitude, longitude).
    pub targets: [(i32, i32); GDN_TARGETS],
    /// The waypoints' `z`, m above home, one per target (0.0 for the unused ones and with `GDN_SRC` 0).
    pub alts_m: [f32; GDN_TARGETS],
    /// The return height, m above home: the return item's `z` (0.0 with `GDN_SRC` 0 or no mission).
    pub return_m: f32,
    /// The committed mission's plan id (0 with `GDN_SRC` 0 or no mission).
    pub plan_id: u32,
    /// The set is the committed mission (`GDN_SRC` 1), not the `GDN_*` parameters.
    pub from_mission: bool,
    /// `GDN_SRC` 1, the mission has its return item, and every waypoint `z` and the return height are finite and within the height limits given to [`select_targets`] (`GDN_ALT_MIN`, `GDN_ALT_MAX` when the set was pushed).
    pub heights_ok: bool,
    /// `GDN_ALT_MAX` given to [`select_targets`], m (either source): the cap on RETURN's height on the flight core.
    pub alt_max_m: f32,
    /// The committed mission holds at least one waypoint (any `GDN_SRC`): Start's refusal tells "not selected" from "none".
    pub mission_stored: bool,
}

/// The target set to push: with `source` 1 the committed mission's waypoints (`mission` without its last item, the return item, whose `z` is the return height) with `radius_m` and the mission's `plan_id`; with any other `source` (0, the default) exactly the `GDN_*` parameters, `count_param` clamped to `0..=param_targets.len()` and the first of `param_targets`, whatever the mission holds.
pub fn select_targets(
    source: i32,
    count_param: i32,
    radius_m: f32,
    param_targets: &[(i32, i32)],
    mission: &[Item],
    plan_id: u32,
    alt_limits_m: (f32, f32),
) -> GdnTargets {
    let mut targets = [(0, 0); GDN_TARGETS];
    let mut alts_m = [0.0; GDN_TARGETS];
    let mut return_m = 0.0;
    let mut plan = 0;
    let mut heights_ok = false;
    let count = if source == 1 {
        let (waypoints, ret) = match mission.split_last() {
            Some((ret, waypoints)) => (waypoints, Some(ret)),
            None => (mission, None),
        };
        for ((dst, alt), item) in targets.iter_mut().zip(alts_m.iter_mut()).zip(waypoints) {
            *dst = (item.x, item.y);
            *alt = f32::from_bits(item.z_bits);
        }
        if let Some(ret) = ret {
            return_m = f32::from_bits(ret.z_bits);
            plan = plan_id;
            let (lo, hi) = alt_limits_m;
            heights_ok = waypoints.iter().chain(core::iter::once(ret)).all(|it| {
                let z = f32::from_bits(it.z_bits);
                z >= lo && z <= hi
            });
        }
        waypoints.len().min(GDN_TARGETS)
    } else {
        for (dst, src) in targets.iter_mut().zip(param_targets) {
            *dst = *src;
        }
        let limit = i32::try_from(param_targets.len().min(GDN_TARGETS)).unwrap_or(0);
        usize::try_from(count_param.clamp(0, limit)).unwrap_or(0)
    };
    GdnTargets {
        count: u8::try_from(count).unwrap_or(0),
        radius_m,
        targets,
        alts_m,
        return_m,
        plan_id: plan,
        from_mission: source == 1,
        heights_ok,
        alt_max_m: alt_limits_m.1,
        mission_stored: mission.len() >= 2,
    }
}

/// The vehicle's mission service; see the module documentation.
#[derive(Clone, Debug)]
pub struct MissionService {
    items: [Item; MAX_ITEMS],
    count: u8,
    plan_id: u32,
    changes: u32,
    staging: [Item; MAX_ITEMS],
    /// `GDN_ALT_MIN` and `GDN_ALT_MAX` in force (m), as of the last [`MissionService::set_limits`].
    limits: (f32, f32),
    txn: Option<Txn>,
    last_upload: Option<LastUpload>,
}

impl Default for MissionService {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionService {
    /// An empty list (plan id 0, change counter 0) and no transaction: the state after boot.
    pub const fn new() -> Self {
        Self {
            items: [Item::ZERO; MAX_ITEMS],
            count: 0,
            plan_id: 0,
            changes: 0,
            staging: [Item::ZERO; MAX_ITEMS],
            limits: (ALT_MIN_DEFAULT, ALT_MAX_DEFAULT),
            txn: None,
            last_upload: None,
        }
    }

    /// Sets the height limits (`GDN_ALT_MIN`, `GDN_ALT_MAX`, m) the next uploaded items are checked against.
    pub fn set_limits(&mut self, alt_min: f32, alt_max: f32) {
        self.limits = (alt_min, alt_max);
    }

    /// The committed items, `seq` 0 first: the waypoints, then the return item (empty when no list is stored).
    pub fn items(&self) -> &[Item] {
        self.items.get(..usize::from(self.count)).unwrap_or(&[])
    }

    /// The number of committed items (the waypoints and the return item).
    pub fn count(&self) -> u8 {
        self.count
    }

    /// The plan id of the committed list: 0 when empty, else the CRC-32 (0 becomes 1).
    pub fn plan_id(&self) -> u32 {
        self.plan_id
    }

    /// Increments (wrapping) on every commit and every clear, also a clear of an empty list.
    pub fn change_counter(&self) -> u32 {
        self.changes
    }

    /// Whether a transaction runs, on either link.
    pub fn busy(&self) -> bool {
        self.txn.is_some()
    }

    /// The owner of the running transaction, if any.
    pub fn owner(&self) -> Option<Owner> {
        self.txn.map(|t| t.owner)
    }

    /// Handles one mission frame at `now_us` with the vehicle's `armed` state; returns the message to send, if any.
    pub fn handle(&mut self, now_us: u64, armed: bool, frame: &Frame) -> Option<Tx> {
        if !matches!(frame.target_system, 0 | VEHICLE_SYSTEM)
            || !matches!(frame.target_component, 0 | VEHICLE_COMPONENT)
        {
            return None;
        }
        let from = Owner {
            link: frame.link,
            system: frame.src_system,
            component: frame.src_component,
        };
        let start = matches!(
            frame.msg,
            Rx::Count { .. } | Rx::RequestList { .. } | Rx::ClearAll { .. }
        );
        if let Some(txn) = self.txn {
            if txn.owner != from {
                let mission_type = match frame.msg {
                    Rx::Count { mission_type, .. }
                    | Rx::RequestList { mission_type }
                    | Rx::ClearAll { mission_type }
                    | Rx::RequestInt { mission_type, .. } => mission_type,
                    Rx::ItemInt(item) => item.mission_type,
                    Rx::Ack { .. } => return None,
                };
                return ack(from, result::DENIED, mission_type, 0);
            }
            if start {
                self.txn = None;
            }
        }
        if start {
            self.last_upload = None;
        }
        if self.txn.is_some() {
            self.in_transaction(now_us, armed, from, frame.msg)
        } else {
            self.idle(now_us, armed, from, frame.msg)
        }
    }

    /// The timers at `now_us`: the vehicle's request sent again 1500 ms after its previous send (up to 5 retries, then MISSION_ACK OPERATION_CANCELLED to the owner and the upload ends), and the download dropped after 9000 ms without a message from its owner.
    pub fn poll(&mut self, now_us: u64) -> Option<Tx> {
        let txn = self.txn?;
        match txn.kind {
            Kind::Upload { n, next, retries, sent_us, crc } => {
                if now_us.saturating_sub(sent_us) < RETRY_US {
                    return None;
                }
                if retries < MAX_RETRIES {
                    self.txn = Some(Txn {
                        owner: txn.owner,
                        kind: Kind::Upload { n, next, retries: retries + 1, sent_us: now_us, crc },
                    });
                    to(txn.owner, TxMsg::RequestInt { seq: next })
                } else {
                    self.txn = None;
                    ack(txn.owner, result::OPERATION_CANCELLED, 0, 0)
                }
            }
            Kind::Download { last_rx_us } => {
                if now_us.saturating_sub(last_rx_us) < DOWNLOAD_IDLE_US {
                    return None;
                }
                self.txn = None;
                ack(txn.owner, result::OPERATION_CANCELLED, 0, 0)
            }
        }
    }

    /// `link` went down: its owner's transaction ends at once (staging discarded, the list unchanged, nothing sent) and the last upload of an owner on that link is forgotten.
    pub fn link_down(&mut self, link: Link) {
        if self.txn.is_some_and(|t| t.owner.link == link) {
            self.txn = None;
        }
        if self.last_upload.is_some_and(|l| l.owner.link == link) {
            self.last_upload = None;
        }
    }

    fn clear(&mut self) {
        self.items = [Item::ZERO; MAX_ITEMS];
        self.count = 0;
        self.plan_id = 0;
        self.changes = self.changes.wrapping_add(1);
    }

    fn commit(&mut self, n: u8, crc: u32) {
        self.items = self.staging;
        self.count = n;
        self.plan_id = plan_id_from_crc(!crc);
        self.changes = self.changes.wrapping_add(1);
    }

    /// Rule 3: no transaction runs.
    fn idle(&mut self, now_us: u64, armed: bool, from: Owner, msg: Rx) -> Option<Tx> {
        match msg {
            Rx::Count { count, mission_type } => {
                if mission_type != 0 {
                    ack(from, result::UNSUPPORTED, mission_type, 0)
                } else if armed {
                    ack(from, result::ERROR, mission_type, 0)
                } else if usize::from(count) > MAX_ITEMS {
                    ack(from, result::NO_SPACE, mission_type, 0)
                } else if count == 0 {
                    self.clear();
                    ack(from, result::ACCEPTED, mission_type, 0)
                } else {
                    let n = u8::try_from(count).ok()?;
                    self.txn = Some(Txn {
                        owner: from,
                        kind: Kind::Upload { n, next: 0, retries: 0, sent_us: now_us, crc: u32::MAX },
                    });
                    to(from, TxMsg::RequestInt { seq: 0 })
                }
            }
            Rx::ClearAll { mission_type } => {
                if mission_type != 0 {
                    ack(from, result::UNSUPPORTED, mission_type, 0)
                } else if armed {
                    ack(from, result::ERROR, mission_type, 0)
                } else {
                    self.clear();
                    ack(from, result::ACCEPTED, mission_type, 0)
                }
            }
            Rx::RequestList { mission_type } => {
                if mission_type != 0 {
                    return ack(from, result::UNSUPPORTED, mission_type, 0);
                }
                self.txn = Some(Txn { owner: from, kind: Kind::Download { last_rx_us: now_us } });
                to(
                    from,
                    TxMsg::Count { count: u16::from(self.count), opaque_id: self.plan_id },
                )
            }
            Rx::ItemInt(item) => match self.last_upload {
                Some(last) if last.owner == from && last.seq == item.seq => {
                    ack(from, last.result, last.mission_type, last.opaque_id)
                }
                _ => ack(from, result::ERROR, item.mission_type, 0),
            },
            Rx::RequestInt { mission_type, .. } => ack(from, result::ERROR, mission_type, 0),
            Rx::Ack { .. } => None,
        }
    }

    /// Rules 4 and 5: a message of the owner (not a start message) while its transaction runs.
    fn in_transaction(&mut self, now_us: u64, armed: bool, from: Owner, msg: Rx) -> Option<Tx> {
        let txn = self.txn?;
        match txn.kind {
            Kind::Upload { n, next, retries, crc, .. } => match msg {
                Rx::ItemInt(item) => self.upload_item(now_us, armed, from, (n, next, retries, crc), item),
                Rx::Ack { .. } => {
                    self.txn = None;
                    None
                }
                _ => None,
            },
            Kind::Download { .. } => {
                self.txn = Some(Txn { owner: from, kind: Kind::Download { last_rx_us: now_us } });
                match msg {
                    Rx::RequestInt { seq, mission_type } => {
                        if mission_type != 0 {
                            self.txn = None;
                            return ack(from, result::UNSUPPORTED, mission_type, 0);
                        }
                        match self.items().get(usize::from(seq)) {
                            Some(item) if usize::from(seq) + 1 == usize::from(self.count) => {
                                to(from, TxMsg::ReturnItem { seq, z_bits: item.z_bits })
                            }
                            Some(item) => to(
                                from,
                                TxMsg::Item { seq, x: item.x, y: item.y, z_bits: item.z_bits },
                            ),
                            None => {
                                self.txn = None;
                                ack(from, result::INVALID_SEQUENCE, mission_type, 0)
                            }
                        }
                    }
                    Rx::Ack { .. } => {
                        self.txn = None;
                        None
                    }
                    _ => None,
                }
            }
        }
    }

    fn upload_item(
        &mut self,
        now_us: u64,
        armed: bool,
        from: Owner,
        (n, next, retries, crc): (u8, u16, u8, u32),
        item: RxItem,
    ) -> Option<Tx> {
        if item.seq != next {
            self.txn = Some(Txn {
                owner: from,
                kind: Kind::Upload { n, next, retries, sent_us: now_us, crc },
            });
            return to(from, TxMsg::RequestInt { seq: next });
        }
        let last = usize::from(next) + 1 >= usize::from(n);
        let stored = match validate(armed, last, self.limits, &item) {
            Ok(stored) => stored,
            Err(code) => {
                self.txn = None;
                self.last_upload = Some(LastUpload {
                    owner: from,
                    seq: item.seq,
                    result: code,
                    mission_type: item.mission_type,
                    opaque_id: 0,
                });
                return ack(from, code, item.mission_type, 0);
            }
        };
        if let Some(slot) = self.staging.get_mut(usize::from(next)) {
            *slot = stored;
        }
        let crc = crc32_update(crc, &stored.bytes());
        if last {
            self.commit(n, crc);
            self.txn = None;
            self.last_upload = Some(LastUpload {
                owner: from,
                seq: item.seq,
                result: result::ACCEPTED,
                mission_type: item.mission_type,
                opaque_id: self.plan_id,
            });
            ack(from, result::ACCEPTED, item.mission_type, self.plan_id)
        } else {
            let next = next + 1;
            self.txn = Some(Txn {
                owner: from,
                kind: Kind::Upload { n, next, retries: 0, sent_us: now_us, crc },
            });
            to(from, TxMsg::RequestInt { seq: next })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000;
    const STATION: (u8, u8) = (255, 190);

    fn rx_item(seq: u16, x: i32, y: i32, z: f32) -> RxItem {
        RxItem {
            seq,
            frame: 3,
            command: 16,
            x,
            y,
            z_bits: z.to_bits(),
            mission_type: 0,
        }
    }

    fn frame_from(link: Link, (sys, comp): (u8, u8), msg: Rx) -> Frame {
        Frame {
            link,
            src_system: sys,
            src_component: comp,
            target_system: 1,
            target_component: 1,
            msg,
        }
    }

    fn fr(link: Link, msg: Rx) -> Frame {
        frame_from(link, STATION, msg)
    }

    fn count(count: u16) -> Rx {
        Rx::Count { count, mission_type: 0 }
    }

    fn req_list() -> Rx {
        Rx::RequestList { mission_type: 0 }
    }

    fn clear_all() -> Rx {
        Rx::ClearAll { mission_type: 0 }
    }

    fn req(seq: u16) -> Rx {
        Rx::RequestInt { seq, mission_type: 0 }
    }

    fn item(seq: u16, x: i32, y: i32) -> Rx {
        Rx::ItemInt(rx_item(seq, x, y, 10.0))
    }

    fn done_ack() -> Rx {
        Rx::Ack { result: result::ACCEPTED, mission_type: 0 }
    }

    /// What the vehicle must send: to the station (255, 190) on `link`.
    fn to_station(link: Link, msg: TxMsg) -> Option<Tx> {
        Some(Tx { link, dst_system: 255, dst_component: 190, msg })
    }

    fn want_ack(link: Link, result: u8, mission_type: u8, opaque_id: u32) -> Option<Tx> {
        to_station(link, TxMsg::Ack { result, mission_type, opaque_id })
    }

    fn want_req(link: Link, seq: u16) -> Option<Tx> {
        to_station(link, TxMsg::RequestInt { seq })
    }

    /// The contract's three points (open ocean, made up).
    fn points3() -> [Item; 3] {
        [
            Item { x: 100_000_000, y: -300_000_000, z_bits: 10.0f32.to_bits() },
            Item { x: 100_010_000, y: -300_000_000, z_bits: 12.5f32.to_bits() },
            Item { x: 100_010_000, y: -299_990_000, z_bits: 15.0f32.to_bits() },
        ]
    }

    const PLAN3: u32 = 0x99C3_BD49;

    /// The three points and a return item of 20.0 m: the contract's worked example 3.
    fn full3() -> [Item; 4] {
        let p = points3();
        [p[0], p[1], p[2], Item { x: 0, y: 0, z_bits: 20.0f32.to_bits() }]
    }

    const PLAN3R: u32 = 0x1862_F4A2;

    /// An item as the station sends it: the return item (x = y = 0, z) is the last one of `n`.
    fn rx_of_n(it: &Item, seq: u16, n: usize) -> Rx {
        Rx::ItemInt(RxItem {
            seq,
            frame: 3,
            command: if usize::from(seq) + 1 == n { 20 } else { 16 },
            x: it.x,
            y: it.y,
            z_bits: it.z_bits,
            mission_type: 0,
        })
    }

    /// An item as the station sends it; one with x = y = 0 is the return item (command 20).
    fn rx_of(it: &Item, seq: u16) -> Rx {
        Rx::ItemInt(RxItem {
            seq,
            frame: 3,
            command: if it.x == 0 && it.y == 0 { 20 } else { 16 },
            x: it.x,
            y: it.y,
            z_bits: it.z_bits,
            mission_type: 0,
        })
    }

    /// A whole upload by the station on `link`, one step per 40 ms from `t0`; every step is checked;
    /// returns the vehicle's final answer.
    fn upload(s: &mut MissionService, link: Link, pts: &[Item], t0: u64) -> Option<Tx> {
        let mut t = t0;
        let mut out = s.handle(t, false, &fr(link, count(pts.len() as u16)));
        for (i, it) in pts.iter().enumerate() {
            assert_eq!(out, want_req(link, i as u16), "request {i}");
            t += 40 * MS;
            out = s.handle(t, false, &fr(link, rx_of_n(it, i as u16, pts.len())));
        }
        out
    }

    /// A list of `n` items as uploaded: `n` - 1 waypoints (z 10 to 25 m) and the return item (0, 0, 20 m); empty for 0.
    fn upload_to_the_end(s: &mut MissionService, link: Link, pts: &[Item]) -> Option<Tx> {
        let mut out = s.handle(0, false, &fr(link, count(pts.len() as u16)));
        for (i, it) in pts.iter().enumerate() {
            if out != want_req(link, i as u16) {
                return out;
            }
            out = s.handle(1 + i as u64, false, &fr(link, rx_of_n(it, i as u16, pts.len())));
        }
        out
    }

    fn many(n: usize) -> std::vec::Vec<Item> {
        (0..n)
            .map(|i| {
                if i + 1 == n {
                    Item { x: 0, y: 0, z_bits: 20.0f32.to_bits() }
                } else {
                    Item {
                        x: 100_000_000 + i as i32 * 1000,
                        y: -300_000_000,
                        z_bits: (10.0 + i as f32).to_bits(),
                    }
                }
            })
            .collect()
    }

    fn held(s: &MissionService) -> (std::vec::Vec<Item>, u32, u32) {
        (s.items().to_vec(), s.plan_id(), s.change_counter())
    }


    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn test_vector_bytes_and_plan_id() {
        let pts = points3();
        let want: [[u8; 12]; 3] = [
            [0x00, 0xE1, 0xF5, 0x05, 0x00, 0x5D, 0x1E, 0xEE, 0x00, 0x00, 0x20, 0x41],
            [0x10, 0x08, 0xF6, 0x05, 0x00, 0x5D, 0x1E, 0xEE, 0x00, 0x00, 0x48, 0x41],
            [0x10, 0x08, 0xF6, 0x05, 0x10, 0x84, 0x1E, 0xEE, 0x00, 0x00, 0x70, 0x41],
        ];
        for (p, w) in pts.iter().zip(want.iter()) {
            assert_eq!(&p.bytes(), w);
        }
        assert_eq!(plan_id_of(&pts), PLAN3);
        assert_eq!(0x99C3_BD49u32, 2_579_742_025);
        let full = full3();
        assert_eq!(&full[3].bytes(), &[0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0xA0, 0x41]);
        assert_eq!(plan_id_of(&full), PLAN3R);
        let mut s = MissionService::new();
        let end = upload(&mut s, Link::Radio, &full, 0);
        assert_eq!(end, want_ack(Link::Radio, 0, 0, 0x1862_F4A2));
        assert_eq!(s.plan_id(), 0x1862_F4A2);
        assert_eq!(0x1862_F4A2u32, 409_138_338);
    }

    #[test]
    fn empty_list_plan_id_is_zero() {
        assert_eq!(plan_id_of(&[]), 0);
        let s = MissionService::new();
        assert_eq!(s.plan_id(), 0);
        assert_eq!(s.count(), 0);
        assert_eq!(s.change_counter(), 0);
        assert!(s.items().is_empty());
    }

    #[test]
    fn zero_crc_becomes_one() {
        assert_eq!(plan_id_from_crc(0), 1);
        assert_eq!(plan_id_from_crc(5), 5);
        let ret = Item { x: 0, y: 0, z_bits: 20.0f32.to_bits() };
        let base = Item { x: 100_000_000, y: -300_000_000, z_bits: 10.0f32.to_bits() };
        let with = |m: u128| {
            let mut it = base;
            it.x ^= (m & 0xFF_FFFF) as i32;
            it.y ^= ((m >> 24) & 0xFF_FFFF) as i32;
            it.z_bits ^= ((m >> 48) & 0xF_FFFF) as u32;
            it
        };
        let crc_of = |it: &Item| {
            let mut b = [0u8; 2 * ITEM_BYTES];
            b[..ITEM_BYTES].copy_from_slice(&it.bytes());
            b[ITEM_BYTES..].copy_from_slice(&ret.bytes());
            crc32(&b)
        };
        let a = crc_of(&base);
        let mut rows: std::vec::Vec<(u32, u128)> = std::vec::Vec::new();
        for i in 0..68 {
            let mut v = crc_of(&with(1u128 << i)) ^ a;
            let mut m = 1u128 << i;
            for &(rv, rm) in &rows {
                if v ^ rv < v {
                    v ^= rv;
                    m ^= rm;
                }
            }
            if v != 0 {
                rows.push((v, m));
                rows.sort_by_key(|p| core::cmp::Reverse(p.0));
            }
        }
        let mut v = a;
        let mut flips = 0u128;
        for &(rv, rm) in &rows {
            if v ^ rv < v {
                v ^= rv;
                flips ^= rm;
            }
        }
        assert_eq!(v, 0, "CRC not solvable");
        let waypoint = with(flips);
        assert_eq!(crc_of(&waypoint), 0);
        assert!(f32::from_bits(waypoint.z_bits) >= 10.0 && f32::from_bits(waypoint.z_bits) < 12.0);
        assert_eq!(plan_id_of(&[waypoint, ret]), 1);
        let mut s = MissionService::new();
        let end = upload(&mut s, Link::Usb, &[waypoint, ret], 0);
        assert_eq!(end, want_ack(Link::Usb, 0, 0, 1));
        assert_eq!(s.plan_id(), 1);
    }


    #[test]
    fn worked_example_3() {
        let pts = full3();
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(4))), want_req(r, 0));
        assert_eq!(s.handle(45 * MS, false, &fr(r, rx_of_n(&pts[0], 0, 4))), want_req(r, 1));
        assert_eq!(s.poll(1000 * MS), None);
        assert_eq!(s.poll(1544 * MS), None);
        assert_eq!(s.poll(1545 * MS), want_req(r, 1));
        assert_eq!(s.handle(1560 * MS, false, &fr(r, rx_of_n(&pts[1], 1, 4))), want_req(r, 2));
        assert_eq!(s.handle(1580 * MS, false, &fr(r, rx_of_n(&pts[2], 2, 4))), want_req(r, 3));
        assert!(s.items().is_empty());
        assert_eq!(
            s.handle(1600 * MS, false, &fr(r, rx_of_n(&pts[3], 3, 4))),
            want_ack(r, result::ACCEPTED, 0, 409_138_338)
        );
        assert_eq!(s.items(), &pts);
        assert_eq!(s.plan_id(), PLAN3R);
        assert_eq!(s.count(), 4);
        assert_eq!(s.change_counter(), 1);
        assert!(!s.busy());
        assert_eq!(
            s.handle(1640 * MS, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 4, opaque_id: 409_138_338 })
        );
        for (i, p) in pts.iter().enumerate() {
            let t = (1680 + 40 * i as u64) * MS;
            let want = if i == 3 {
                TxMsg::ReturnItem { seq: 3, z_bits: 20.0f32.to_bits() }
            } else {
                TxMsg::Item { seq: i as u16, x: p.x, y: p.y, z_bits: p.z_bits }
            };
            assert_eq!(s.handle(t, false, &fr(r, req(i as u16))), to_station(r, want));
        }
        assert_eq!(s.handle(1800 * MS, false, &fr(r, done_ack())), None);
        assert!(!s.busy());
        assert_eq!(s.poll(100_000 * MS), None);
    }

    #[test]
    fn worked_example_3_with_a_return_height_of_4_ends_at_seq_3_with_param7() {
        let mut pts = full3();
        pts[3].z_bits = 4.0f32.to_bits();
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(upload(&mut s, r, &full3(), 0), want_ack(r, 0, 0, PLAN3R));
        let before = held(&s);
        assert_eq!(upload(&mut s, r, &pts, 100), want_ack(r, result::INVALID_PARAM7, 0, 0));
        assert!(!s.busy());
        assert_eq!(held(&s), before);
    }

    #[test]
    fn worked_example_2() {
        let pts = full3();
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(upload(&mut s, r, &pts, 0), want_ack(r, 0, 0, PLAN3R));
        let before = held(&s);
        assert_eq!(
            s.handle(10_000 * MS, false, &fr(r, count(18))),
            want_ack(r, result::NO_SPACE, 0, 0)
        );
        assert!(!s.busy());
        assert_eq!(held(&s), before);
        assert_eq!(
            s.handle(10_040 * MS, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 4, opaque_id: 409_138_338 })
        );
        for (i, p) in pts.iter().take(3).enumerate() {
            assert_eq!(
                s.handle(10_080 * MS, false, &fr(r, req(i as u16))),
                to_station(r, TxMsg::Item { seq: i as u16, x: p.x, y: p.y, z_bits: p.z_bits })
            );
        }
        assert_eq!(
            s.handle(10_080 * MS, false, &fr(r, req(3))),
            to_station(r, TxMsg::ReturnItem { seq: 3, z_bits: 20.0f32.to_bits() })
        );
        assert_eq!(s.handle(10_200 * MS, false, &fr(r, done_ack())), None);
        assert_eq!(held(&s), before);
    }


    #[test]
    fn seventeen_accepted_eighteen_no_space() {
        let mut s = MissionService::new();
        let pts = many(17);
        let end = upload(&mut s, Link::Usb, &pts, 0);
        assert_eq!(end, want_ack(Link::Usb, 0, 0, plan_id_of(&pts)));
        assert_eq!(s.count(), 17);
        assert_eq!(s.items(), &pts[..]);
        let before = held(&s);
        assert_eq!(
            s.handle(0, false, &fr(Link::Usb, count(18))),
            want_ack(Link::Usb, result::NO_SPACE, 0, 0)
        );
        assert_eq!(
            s.handle(0, false, &fr(Link::Usb, count(65535))),
            want_ack(Link::Usb, result::NO_SPACE, 0, 0)
        );
        assert!(!s.busy());
        assert_eq!(held(&s), before);
    }

    #[test]
    fn one_item_upload_and_replace() {
        let mut s = MissionService::new();
        let one = many(1);
        assert_eq!(upload(&mut s, Link::Usb, &one, 0), want_ack(Link::Usb, 0, 0, plan_id_of(&one)));
        assert_eq!(s.change_counter(), 1);
        assert_eq!(s.items(), &one[..]);
        assert_eq!(s.count(), 1);
        let two = many(5);
        assert_eq!(upload(&mut s, Link::Usb, &two, 0), want_ack(Link::Usb, 0, 0, plan_id_of(&two)));
        assert_eq!(s.items(), &two[..]);
        assert_eq!(s.change_counter(), 2);
        let three = many(3);
        let _ = upload(&mut s, Link::Usb, &three, 0);
        assert_eq!(s.items(), &three[..]);
        assert_eq!(s.count(), 3);
    }

    #[test]
    fn boundary_coordinates_accepted() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let pts = [
            Item { x: 900_000_000, y: 1_800_000_000, z_bits: 5.0f32.to_bits() },
            Item { x: -900_000_000, y: -1_800_000_000, z_bits: 50.0f32.to_bits() },
            Item { x: 0, y: 0, z_bits: 5.0f32.to_bits() },
        ];
        assert_eq!(upload(&mut s, r, &pts, 0), want_ack(r, 0, 0, plan_id_of(&pts)));
        assert_eq!(s.items(), &pts);
    }

    #[test]
    fn count_zero_clears() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        assert_eq!(s.change_counter(), 1);
        assert_eq!(s.handle(0, false, &fr(r, count(0))), want_ack(r, 0, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.count(), 0);
        assert_eq!(s.plan_id(), 0);
        assert!(s.items().is_empty());
        assert_eq!(s.change_counter(), 2);
        assert_eq!(s.handle(0, false, &fr(r, count(0))), want_ack(r, 0, 0, 0));
        assert_eq!(s.change_counter(), 3);
    }

    #[test]
    fn clear_all_clears_and_acks_zero_opaque() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let _ = upload(&mut s, r, &full3(), 0);
        assert_eq!(s.handle(0, false, &fr(r, clear_all())), want_ack(r, 0, 0, 0));
        assert!(!s.busy());
        assert_eq!((s.count(), s.plan_id(), s.change_counter()), (0, 0, 2));
        assert_eq!(s.handle(0, false, &fr(r, clear_all())), want_ack(r, 0, 0, 0));
        assert_eq!(s.change_counter(), 3);
    }


    fn refusal(frame_msg: Rx, armed: bool, code: u8, mission_type: u8) {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let before = held(&s);
        assert_eq!(
            s.handle(1_000 * MS, armed, &fr(r, frame_msg)),
            want_ack(r, code, mission_type, 0)
        );
        assert!(!s.busy());
        assert_eq!(held(&s), before);
    }

    #[test]
    fn count_mission_type_unsupported() {
        refusal(Rx::Count { count: 4, mission_type: 1 }, false, result::UNSUPPORTED, 1);
        refusal(Rx::Count { count: 0, mission_type: 255 }, false, result::UNSUPPORTED, 255);
    }

    #[test]
    fn count_armed_error() {
        refusal(count(3), true, result::ERROR, 0);
        refusal(count(0), true, result::ERROR, 0);
    }

    #[test]
    fn count_above_17_no_space() {
        refusal(count(18), false, result::NO_SPACE, 0);
    }

    #[test]
    fn clear_mission_type_unsupported() {
        refusal(Rx::ClearAll { mission_type: 1 }, false, result::UNSUPPORTED, 1);
        refusal(Rx::ClearAll { mission_type: 255 }, false, result::UNSUPPORTED, 255);
    }

    #[test]
    fn clear_armed_error() {
        refusal(clear_all(), true, result::ERROR, 0);
    }

    #[test]
    fn request_list_mission_type_unsupported() {
        refusal(Rx::RequestList { mission_type: 2 }, false, result::UNSUPPORTED, 2);
    }

    #[test]
    fn request_list_allowed_while_armed() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let _ = upload(&mut s, r, &full3(), 0);
        assert_eq!(
            s.handle(0, true, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 4, opaque_id: PLAN3R })
        );
    }

    /// Starts an upload of 3, sends the item `seq` 0 as `mutate` makes it and returns the answer.
    fn item_refusal(mutate: impl Fn(&mut RxItem), armed: bool) -> (MissionService, Option<Tx>) {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let before = held(&s);
        assert_eq!(s.handle(1_000 * MS, false, &fr(r, count(3))), want_req(r, 0));
        let mut it = rx_item(0, 1, 2, 10.0);
        mutate(&mut it);
        let out = s.handle(1_040 * MS, armed, &fr(r, Rx::ItemInt(it)));
        assert!(!s.busy(), "a refusal ends the transaction");
        assert_eq!(held(&s), before, "the list is unchanged");
        (s, out)
    }

    #[test]
    fn item_mission_type_unsupported() {
        let (_, out) = item_refusal(|i| i.mission_type = 1, false);
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED, 1, 0));
    }

    #[test]
    fn item_armed_error() {
        let (_, out) = item_refusal(|_| {}, true);
        assert_eq!(out, want_ack(Link::Radio, result::ERROR, 0, 0));
    }

    #[test]
    fn item_command_unsupported() {
        let (_, out) = item_refusal(|i| i.command = 22, false);
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED, 0, 0));
    }

    #[test]
    fn item_frame_unsupported_frame() {
        for frame in [0u8, 2, 4, 5, 7, 10, 255] {
            let (_, out) = item_refusal(|i| i.frame = frame, false);
            assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED_FRAME, 0, 0), "frame {frame}");
        }
    }

    #[test]
    fn item_x_out_of_range() {
        for x in [900_000_001, -900_000_001, i32::MAX, i32::MIN] {
            let (_, out) = item_refusal(|i| i.x = x, false);
            assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM5_X, 0, 0), "x {x}");
        }
    }

    #[test]
    fn item_y_out_of_range() {
        for y in [1_800_000_001, -1_800_000_001, i32::MAX, i32::MIN] {
            let (_, out) = item_refusal(|i| i.y = y, false);
            assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM6_Y, 0, 0), "y {y}");
        }
    }

    #[test]
    fn item_without_transaction_error() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, item(0, 1, 2))), want_ack(r, result::ERROR, 0, 0));
        let mut it = rx_item(0, 1, 2, 10.0);
        it.mission_type = 9;
        assert_eq!(
            s.handle(0, false, &fr(r, Rx::ItemInt(it))),
            want_ack(r, result::ERROR, 9, 0)
        );
        assert!(!s.busy());
        assert_eq!(s.change_counter(), 0);
    }

    #[test]
    fn request_int_without_transaction_error() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, req(0))), want_ack(r, result::ERROR, 0, 0));
    }

    #[test]
    fn ack_without_transaction_ignored() {
        let mut s = MissionService::new();
        assert_eq!(s.handle(0, false, &fr(Link::Usb, done_ack())), None);
    }

    #[test]
    fn download_request_invalid_sequence() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let before = held(&s);
        for seq in [4u16, 5, 65535] {
            assert!(s.handle(0, false, &fr(r, req_list())).is_some());
            assert_eq!(
                s.handle(0, false, &fr(r, req(seq))),
                want_ack(r, result::INVALID_SEQUENCE, 0, 0),
                "seq {seq}"
            );
            assert!(!s.busy(), "it ends the download");
        }
        assert_eq!(held(&s), before);
        let mut e = MissionService::new();
        assert!(e.handle(0, false, &fr(r, req_list())).is_some());
        assert_eq!(e.handle(0, false, &fr(r, req(0))), want_ack(r, result::INVALID_SEQUENCE, 0, 0));
    }

    #[test]
    fn download_request_mission_type_unsupported() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        assert!(s.handle(0, false, &fr(r, req_list())).is_some());
        assert_eq!(
            s.handle(0, false, &fr(r, Rx::RequestInt { seq: 0, mission_type: 1 })),
            want_ack(r, result::UNSUPPORTED, 1, 0)
        );
        assert!(!s.busy());
    }

    #[test]
    fn giving_up_sends_operation_cancelled() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(2))), want_req(r, 0));
        let mut t = 0;
        for _ in 0..5 {
            t += 1500 * MS;
            assert_eq!(s.poll(t), want_req(r, 0));
        }
        t += 1500 * MS;
        assert_eq!(s.poll(t), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
        assert!(!s.busy());
    }


    #[test]
    fn count_check_order() {
        refusal(Rx::Count { count: 17, mission_type: 1 }, true, result::UNSUPPORTED, 1);
        refusal(count(17), true, result::ERROR, 0);
        refusal(Rx::Count { count: 17, mission_type: 1 }, false, result::UNSUPPORTED, 1);
    }

    #[test]
    fn clear_check_order() {
        refusal(Rx::ClearAll { mission_type: 255 }, true, result::UNSUPPORTED, 255);
    }

    /// A stored list, then an upload of 4 whose first three items are good and whose last (the return item) is as `mutate` makes it, sent with the service's limits `limits`; returns the answer and whether the list kept its content.
    fn last_item(mutate: impl Fn(&mut RxItem), limits: (f32, f32)) -> (Option<Tx>, bool) {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let before = held(&s);
        let new = many(4);
        assert_eq!(s.handle(1_000 * MS, false, &fr(r, count(4))), want_req(r, 0));
        for (i, it) in new.iter().enumerate().take(3) {
            assert_eq!(s.handle(1_001 * MS, false, &fr(r, rx_of(it, i as u16))), want_req(r, i as u16 + 1));
        }
        s.set_limits(limits.0, limits.1);
        let Rx::ItemInt(mut last) = rx_of(&new[3], 3) else { panic!() };
        mutate(&mut last);
        let out = s.handle(1_002 * MS, false, &fr(r, Rx::ItemInt(last)));
        assert!(!s.busy());
        (out, held(&s) == before)
    }

    const DEFAULT_LIMITS: (f32, f32) = (5.0, 50.0);

    fn refused_last(mutate: impl Fn(&mut RxItem), code: u8, mission_type: u8) {
        let (out, unchanged) = last_item(mutate, DEFAULT_LIMITS);
        assert_eq!(out, want_ack(Link::Radio, code, mission_type, 0));
        assert!(unchanged, "the previous list and plan id survive");
    }

    #[test]
    fn last_item_command_not_20_is_invalid() {
        for command in [16u16, 0, 1, 22, 21, 19, 400] {
            refused_last(|i| i.command = command, result::INVALID, 0);
        }
    }

    #[test]
    fn command_20_at_any_other_seq_is_invalid() {
        let (_, out) = item_refusal(|i| i.command = 20, false);
        assert_eq!(out, want_ack(Link::Radio, result::INVALID, 0, 0));
    }

    #[test]
    fn return_item_x_not_0_is_param5() {
        for x in [1, -1, 100_000_000, i32::MAX, i32::MIN] {
            refused_last(|i| i.x = x, result::INVALID_PARAM5_X, 0);
        }
    }

    #[test]
    fn return_item_y_not_0_is_param6() {
        for y in [1, -1, 100_000_000, i32::MAX, i32::MIN] {
            refused_last(|i| i.y = y, result::INVALID_PARAM6_Y, 0);
        }
    }

    #[test]
    fn return_item_frame_3_and_6_accepted_others_unsupported_frame() {
        for frame in [0u8, 2, 4, 5, 7, 10, 255] {
            refused_last(|i| i.frame = frame, result::UNSUPPORTED_FRAME, 0);
        }
        for frame in [3u8, 6] {
            let (out, unchanged) = last_item(|i| i.frame = frame, DEFAULT_LIMITS);
            assert_eq!(out, want_ack(Link::Radio, result::ACCEPTED, 0, plan_id_of(&many(4))), "frame {frame}");
            assert!(!unchanged);
        }
    }

    #[test]
    fn return_item_mission_type_and_armed_come_first() {
        refused_last(|i| i.mission_type = 1, result::UNSUPPORTED, 1);
        let (_, out) = item_refusal(|i| i.command = 20, true);
        assert_eq!(out, want_ack(Link::Radio, result::ERROR, 0, 0));
    }

    #[test]
    fn return_item_check_order_command_frame_x_y_z() {
        refused_last(|i| { i.command = 16; i.frame = 99; i.x = 5; i.y = 5; i.z_bits = f32::NAN.to_bits(); }, result::INVALID, 0);
        refused_last(|i| { i.frame = 99; i.x = 5; i.y = 5; i.z_bits = f32::NAN.to_bits(); }, result::UNSUPPORTED_FRAME, 0);
        refused_last(|i| { i.x = 5; i.y = 5; i.z_bits = f32::NAN.to_bits(); }, result::INVALID_PARAM5_X, 0);
        refused_last(|i| { i.y = 5; i.z_bits = f32::NAN.to_bits(); }, result::INVALID_PARAM6_Y, 0);
        refused_last(|i| i.z_bits = f32::NAN.to_bits(), result::INVALID_PARAM7, 0);
    }

    #[test]
    fn waypoint_check_order_ends_with_z() {
        let (_, out) = item_refusal(|i| { i.z_bits = f32::NAN.to_bits(); i.y = i32::MAX; }, false);
        assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM6_Y, 0, 0));
        let (_, out) = item_refusal(|i| i.z_bits = 4.999f32.to_bits(), false);
        assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM7, 0, 0));
    }

    fn just_below(v: f32) -> f32 {
        f32::from_bits(v.to_bits() - 1)
    }

    fn just_above(v: f32) -> f32 {
        f32::from_bits(v.to_bits() + 1)
    }

    #[test]
    fn z_at_each_limit_accepted_a_hair_outside_param7_waypoint_and_return_item() {
        for (lo, hi) in [(5.0f32, 50.0f32), (3.0, 120.0), (7.5, 7.5)] {
            for z in [lo, hi, (lo + hi) / 2.0] {
                let mut s = MissionService::new();
                s.set_limits(lo, hi);
                let list = [
                    Item { x: 100_000_000, y: -300_000_000, z_bits: z.to_bits() },
                    Item { x: 0, y: 0, z_bits: z.to_bits() },
                ];
                assert_eq!(
                    upload(&mut s, Link::Usb, &list, 0),
                    want_ack(Link::Usb, 0, 0, plan_id_of(&list)),
                    "z {z} in [{lo}, {hi}]"
                );
            }
            for z in [just_below(lo), just_above(hi), lo - 1.0, hi + 1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.0, 0.0, f32::MAX] {
                for at in 0..2usize {
                    let mut s = MissionService::new();
                    s.set_limits(lo, hi);
                    let mid = ((lo + hi) / 2.0).to_bits();
                    let mut list = [
                        Item { x: 100_000_000, y: -300_000_000, z_bits: mid },
                        Item { x: 0, y: 0, z_bits: mid },
                    ];
                    list[at].z_bits = z.to_bits();
                    assert_eq!(
                        upload_to_the_end(&mut s, Link::Usb, &list),
                        want_ack(Link::Usb, result::INVALID_PARAM7, 0, 0),
                        "z {z} at item {at} in [{lo}, {hi}]"
                    );
                    assert!(s.items().is_empty() && !s.busy());
                }
            }
        }
    }

    #[test]
    fn a_minimum_above_the_maximum_refuses_every_z() {
        for z in [3.0f32, 10.0, 50.0, 100.0, 120.0] {
            let (out, unchanged) = last_item(|i| i.z_bits = z.to_bits(), (100.0, 10.0));
            assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM7, 0, 0), "z {z}");
            assert!(unchanged);
        }
        for bad in [(f32::NAN, 50.0), (5.0, f32::NAN)] {
            let (out, _) = last_item(|_| {}, bad);
            assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM7, 0, 0));
        }
    }

    #[test]
    fn the_limits_in_force_at_the_item_are_used_and_a_stored_list_is_not_rechecked() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(upload(&mut s, r, &full3(), 0), want_ack(r, 0, 0, PLAN3R));
        let before = held(&s);
        s.set_limits(30.0, 40.0);
        assert_eq!(held(&s), before, "a limit changed afterwards leaves the list as it is");
        assert_eq!(
            s.handle(0, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 4, opaque_id: PLAN3R })
        );
    }

    #[test]
    fn count_1_is_the_return_item_alone_and_17_is_accepted() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, count(1))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, item(0, 1, 1))), want_ack(r, result::INVALID, 0, 0));
        assert_eq!(s.handle(2, false, &fr(r, count(17))), want_req(r, 0));
        assert_eq!(s.handle(3, false, &fr(r, count(18))), want_ack(r, result::NO_SPACE, 0, 0));
    }

    #[test]
    fn download_of_a_list_of_one_is_the_return_item() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let one = many(1);
        let _ = upload(&mut s, r, &one, 0);
        assert_eq!(
            s.handle(1, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 1, opaque_id: plan_id_of(&one) })
        );
        assert_eq!(
            s.handle(2, false, &fr(r, req(0))),
            to_station(r, TxMsg::ReturnItem { seq: 0, z_bits: 20.0f32.to_bits() })
        );
    }

    #[test]
    fn item_check_order() {
        let (_, out) = item_refusal(|i| i.mission_type = 1, true);
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED, 1, 0));
        let (_, out) = item_refusal(|i| i.command = 22, true);
        assert_eq!(out, want_ack(Link::Radio, result::ERROR, 0, 0));
        let (_, out) = item_refusal(|i| { i.command = 22; i.frame = 99; }, false);
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED, 0, 0));
        let (_, out) = item_refusal(|i| { i.frame = 99; i.x = i32::MAX; i.y = i32::MAX; }, false);
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED_FRAME, 0, 0));
        let (_, out) = item_refusal(|i| { i.x = i32::MAX; i.y = i32::MAX; }, false);
        assert_eq!(out, want_ack(Link::Radio, result::INVALID_PARAM5_X, 0, 0));
        let (_, out) = item_refusal(
            |i| { i.mission_type = 3; i.command = 22; i.frame = 99; i.x = i32::MAX; i.y = i32::MAX; },
            true,
        );
        assert_eq!(out, want_ack(Link::Radio, result::UNSUPPORTED, 3, 0));
    }

    #[test]
    fn item_seq_check_before_validation() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        let mut bad = rx_item(1, i32::MAX, i32::MAX, 0.0);
        bad.mission_type = 9;
        bad.command = 1;
        bad.frame = 77;
        assert_eq!(s.handle(10 * MS, true, &fr(r, Rx::ItemInt(bad))), want_req(r, 0));
        assert!(s.busy());
    }

    #[test]
    fn denied_before_every_other_check() {
        let mut s = MissionService::new();
        assert_eq!(s.handle(0, false, &fr(Link::Radio, count(3))), want_req(Link::Radio, 0));
        let other = frame_from(Link::Usb, STATION, Rx::Count { count: 17, mission_type: 9 });
        assert_eq!(s.handle(0, true, &other), want_ack(Link::Usb, result::DENIED, 9, 0));
    }


    #[test]
    fn out_of_order_and_repeat_dropped_and_rerequested() {
        let pts = many(4);
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, count(4))), want_req(r, 0));
        assert_eq!(s.handle(MS, false, &fr(r, rx_of(&pts[0], 0))), want_req(r, 1));
        assert_eq!(s.handle(2 * MS, false, &fr(r, rx_of(&pts[0], 0))), want_req(r, 1));
        assert_eq!(s.handle(3 * MS, false, &fr(r, rx_of(&pts[3], 3))), want_req(r, 1));
        assert_eq!(s.handle(4 * MS, false, &fr(r, rx_of(&pts[2], 2))), want_req(r, 1));
        assert!(s.items().is_empty(), "nothing committed yet");
        assert_eq!(s.handle(5 * MS, false, &fr(r, rx_of(&pts[1], 1))), want_req(r, 2));
        assert_eq!(s.handle(6 * MS, false, &fr(r, rx_of(&pts[2], 2))), want_req(r, 3));
        assert_eq!(
            s.handle(7 * MS, false, &fr(r, rx_of(&pts[3], 3))),
            want_ack(r, 0, 0, plan_id_of(&pts))
        );
        assert_eq!(s.items(), &pts[..]);
    }

    #[test]
    fn rerequest_is_not_a_retry_and_restarts_the_timer() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        assert_eq!(s.poll(1500 * MS), want_req(r, 0));
        assert_eq!(s.handle(2000 * MS, false, &fr(r, item(2, 0, 0))), want_req(r, 0));
        assert_eq!(s.poll(3499 * MS), None);
        let mut sent = 1;
        let mut t = 2000 * MS;
        let mut end = None;
        while end.is_none() {
            t += 1500 * MS;
            match s.poll(t) {
                Some(Tx { msg: TxMsg::RequestInt { seq: 0 }, .. }) => sent += 1,
                other => end = Some(other),
            }
            assert!(sent <= 5, "no more than 5 retries in all");
        }
        assert_eq!(sent, 5);
        assert_eq!(end, Some(want_ack(r, result::OPERATION_CANCELLED, 0, 0)));
        assert!(!s.busy());
    }

    #[test]
    fn steady_wrong_items_never_time_out() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        for k in 1..=40u64 {
            assert_eq!(s.poll(k * 1000 * MS), None);
            assert_eq!(s.handle(k * 1000 * MS, false, &fr(r, item(1, 0, 0))), want_req(r, 0));
        }
        assert!(s.busy());
    }

    #[test]
    fn retry_timing_and_give_up() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let t0 = 7_000 * MS;
        assert_eq!(s.handle(t0, false, &fr(r, count(5))), want_req(r, 0));
        assert_eq!(s.handle(t0 + 10 * MS, false, &fr(r, item(0, 1, 1))), want_req(r, 1));
        let start = t0 + 10 * MS;
        for k in 1..=5u64 {
            assert_eq!(s.poll(start + k * 1500 * MS - 1), None, "retry {k} early");
            assert_eq!(s.poll(start + k * 1500 * MS), want_req(r, 1), "retry {k}");
            assert_eq!(s.poll(start + k * 1500 * MS), None, "retry {k} once");
        }
        let sixth = start + 5 * 1500 * MS;
        assert_eq!(s.poll(sixth + 1500 * MS - 1), None);
        assert!(s.busy());
        assert_eq!(s.poll(sixth + 1500 * MS), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.poll(sixth + 100_000 * MS), None);
        assert_eq!(s.change_counter(), 0);
        assert!(s.items().is_empty());
    }

    #[test]
    fn late_poll_retries_from_the_actual_send() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, count(2))), want_req(r, 0));
        assert_eq!(s.poll(1800 * MS), want_req(r, 0));
        assert_eq!(s.poll(3299 * MS), None);
        assert_eq!(s.poll(3300 * MS), want_req(r, 0));
    }

    #[test]
    fn new_request_restarts_retry_count() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let pts = many(3);
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        let mut t = 0;
        for _ in 0..5 {
            t += 1500 * MS;
            assert_eq!(s.poll(t), want_req(r, 0));
        }
        assert_eq!(s.handle(t + 1, false, &fr(r, rx_of(&pts[0], 0))), want_req(r, 1));
        t += 1;
        for _ in 0..5 {
            t += 1500 * MS;
            assert_eq!(s.poll(t), want_req(r, 1));
        }
        t += 1500 * MS;
        assert_eq!(s.poll(t), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
    }

    #[test]
    fn download_idle_nine_seconds() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let t0 = 20_000 * MS;
        assert!(s.handle(t0, false, &fr(r, req_list())).is_some());
        assert_eq!(s.poll(t0 + 1500 * MS), None);
        assert_eq!(s.poll(t0 + 8999 * MS), None);
        assert!(s.handle(t0 + 5000 * MS, false, &fr(r, req(0))).is_some());
        assert_eq!(s.poll(t0 + 13_999 * MS), None);
        assert_eq!(s.poll(t0 + 14_000 * MS), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.poll(t0 + 99_000 * MS), None);
        assert_eq!(s.plan_id(), PLAN3R);
        assert!(s.handle(t0 + 100_000 * MS, false, &fr(r, req_list())).is_some());
        assert_eq!(s.poll(t0 + 109_000 * MS), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
    }


    #[test]
    fn atomic_on_failure_at_the_last_item() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let first = full3();
        assert_eq!(upload(&mut s, r, &first, 0), want_ack(r, 0, 0, PLAN3R));
        let before = held(&s);
        let new = many(4);
        assert_eq!(s.handle(1_000 * MS, false, &fr(r, count(4))), want_req(r, 0));
        for (i, it) in new.iter().enumerate().take(3) {
            assert_eq!(
                s.handle(1_001 * MS, false, &fr(r, rx_of(it, i as u16))),
                want_req(r, i as u16 + 1)
            );
            assert_eq!(held(&s), before);
        }
        let mut last = rx_item(3, 0, 0, 4.0);
        last.command = 20;
        assert_eq!(
            s.handle(1_002 * MS, false, &fr(r, Rx::ItemInt(last))),
            want_ack(r, result::INVALID_PARAM7, 0, 0)
        );
        assert_eq!(held(&s), before);
        assert_eq!(s.items(), &first);
        let again = many(2);
        let _ = upload(&mut s, r, &again, 2_000 * MS);
        assert_eq!(s.items(), &again[..]);
        assert_eq!(s.change_counter(), before.2 + 1);
    }

    #[test]
    fn atomic_on_timeout_cancel_restart_and_link_down() {
        let r = Link::Radio;
        let fresh = || {
            let mut s = MissionService::new();
            let _ = upload(&mut s, r, &full3(), 0);
            let before = held(&s);
            let new = many(3);
            assert_eq!(s.handle(1_000 * MS, false, &fr(r, count(3))), want_req(r, 0));
            assert_eq!(s.handle(1_001 * MS, false, &fr(r, rx_of(&new[0], 0))), want_req(r, 1));
            assert_eq!(s.handle(1_002 * MS, false, &fr(r, rx_of(&new[1], 1))), want_req(r, 2));
            (s, before)
        };
        let (mut s, before) = fresh();
        let mut t = 1_002 * MS;
        for _ in 0..5 {
            t += 1500 * MS;
            assert!(s.poll(t).is_some());
        }
        t += 1500 * MS;
        assert_eq!(s.poll(t), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
        assert_eq!(held(&s), before);
        let (mut s, before) = fresh();
        assert_eq!(
            s.handle(2_000 * MS, false, &fr(r, Rx::Ack { result: 15, mission_type: 0 })),
            None
        );
        assert!(!s.busy());
        assert_eq!(held(&s), before);
        let (mut s, before) = fresh();
        s.link_down(r);
        assert!(!s.busy());
        assert_eq!(held(&s), before);
        assert_eq!(
            s.handle(2_000 * MS, false, &fr(r, item(2, 1, 1))),
            want_ack(r, result::ERROR, 0, 0)
        );
        assert_eq!(held(&s), before);
        let (mut s, before) = fresh();
        let mut bad = rx_item(2, 1, 1, 0.0);
        bad.command = 1;
        assert!(s.handle(2_000 * MS, false, &fr(r, Rx::ItemInt(bad))).is_some());
        assert_eq!(held(&s), before);
        let (mut s, before) = fresh();
        assert_eq!(
            s.handle(2_000 * MS, true, &fr(r, item(2, 1, 1))),
            want_ack(r, result::ERROR, 0, 0)
        );
        assert_eq!(held(&s), before);
    }


    #[test]
    fn second_owner_denied_and_first_continues() {
        let pts = many(3);
        let owners = [
            (Link::Usb, STATION),
            (Link::Radio, (255, 191)),
            (Link::Radio, (254, 190)),
        ];
        for (link, who) in owners {
            let mut s = MissionService::new();
            let r = Link::Radio;
            assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
            assert_eq!(s.handle(MS, false, &fr(r, rx_of(&pts[0], 0))), want_req(r, 1));
            let denied = |mt: u8| {
                Some(Tx {
                    link,
                    dst_system: who.0,
                    dst_component: who.1,
                    msg: TxMsg::Ack { result: result::DENIED, mission_type: mt, opaque_id: 0 },
                })
            };
            let msgs = [
                (Rx::Count { count: 2, mission_type: 7 }, 7),
                (Rx::RequestList { mission_type: 6 }, 6),
                (Rx::ClearAll { mission_type: 5 }, 5),
                (Rx::RequestInt { seq: 0, mission_type: 4 }, 4),
                (Rx::ItemInt(RxItem { mission_type: 3, ..rx_item(1, 0, 0, 0.0) }), 3),
                (count(0), 0),
                (clear_all(), 0),
            ];
            for (m, mt) in msgs {
                assert_eq!(
                    s.handle(2 * MS, false, &frame_from(link, who, m)),
                    denied(mt),
                    "{m:?} from {link:?} {who:?}"
                );
            }
            assert_eq!(s.handle(2 * MS, false, &frame_from(link, who, done_ack())), None);
            assert_eq!(s.owner(), Some(Owner { link: r, system: 255, component: 190 }));
            assert!(s.items().is_empty());
            assert_eq!(s.handle(3 * MS, false, &fr(r, rx_of(&pts[1], 1))), want_req(r, 2));
            assert_eq!(
                s.handle(4 * MS, false, &fr(r, rx_of(&pts[2], 2))),
                want_ack(r, 0, 0, plan_id_of(&pts))
            );
            assert_eq!(s.items(), &pts[..]);
            assert_eq!(s.change_counter(), 1);
        }
    }

    #[test]
    fn second_owner_denied_during_download_and_after_it_may_start() {
        let mut s = MissionService::new();
        let _ = upload(&mut s, Link::Radio, &full3(), 0);
        assert!(s.handle(0, false, &fr(Link::Usb, req_list())).is_some());
        assert_eq!(
            s.handle(1, false, &fr(Link::Radio, req_list())),
            want_ack(Link::Radio, result::DENIED, 0, 0)
        );
        assert_eq!(
            s.handle(1, false, &fr(Link::Radio, clear_all())),
            want_ack(Link::Radio, result::DENIED, 0, 0)
        );
        assert_eq!(s.count(), 4);
        assert_eq!(s.handle(2, false, &fr(Link::Usb, done_ack())), None);
        assert_eq!(
            s.handle(3, false, &fr(Link::Radio, clear_all())),
            want_ack(Link::Radio, 0, 0, 0)
        );
    }

    #[test]
    fn denied_does_not_end_anything() {
        let mut s = MissionService::new();
        assert_eq!(s.handle(0, false, &fr(Link::Radio, count(2))), want_req(Link::Radio, 0));
        for k in 0..20 {
            assert!(s.handle(k * 70 * MS, false, &fr(Link::Usb, count(2))).is_some());
        }
        assert_eq!(s.poll(1499 * MS), None);
        assert_eq!(s.poll(1500 * MS), want_req(Link::Radio, 0));
    }

    #[test]
    fn link_down_ends_only_that_links_transaction() {
        let pts = many(3);
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        s.link_down(Link::Usb);
        assert_eq!(s.owner(), Some(Owner { link: r, system: 255, component: 190 }));
        assert_eq!(s.handle(1, false, &fr(r, rx_of(&pts[0], 0))), want_req(r, 1));
        s.link_down(Link::Usb);
        assert_eq!(s.handle(2, false, &fr(r, rx_of(&pts[1], 1))), want_req(r, 2));
        assert_eq!(s.handle(3, false, &fr(r, rx_of(&pts[2], 2))), want_ack(r, 0, 0, plan_id_of(&pts)));
        assert!(s.handle(4, false, &fr(Link::Usb, req_list())).is_some());
        s.link_down(Link::Radio);
        assert!(s.busy());
        assert!(s.handle(5, false, &fr(Link::Usb, req(1))).is_some());
        s.link_down(Link::Usb);
        assert!(!s.busy());
        assert_eq!(s.poll(1_000_000 * MS), None);
        assert_eq!(s.items(), &pts[..]);
        assert_eq!(s.handle(6, false, &fr(Link::Radio, count(2))), want_req(r, 0));
        s.link_down(Link::Radio);
        assert_eq!(s.handle(7, false, &fr(Link::Usb, count(2))), want_req(Link::Usb, 0));
    }


    #[test]
    fn armed_refuses_count_clear_and_every_upload_item_but_serves_downloads() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let pts = many(3);
        let _ = upload(&mut s, r, &pts, 0);
        let before = held(&s);
        assert_eq!(s.handle(0, true, &fr(r, count(2))), want_ack(r, result::ERROR, 0, 0));
        assert_eq!(s.handle(0, true, &fr(r, count(0))), want_ack(r, result::ERROR, 0, 0));
        assert_eq!(s.handle(0, true, &fr(r, clear_all())), want_ack(r, result::ERROR, 0, 0));
        assert_eq!(held(&s), before);
        for k in 0..3u16 {
            let mut s = MissionService::new();
            assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
            for i in 0..k {
                assert_eq!(
                    s.handle(1, false, &fr(r, rx_of(&pts[usize::from(i)], i))),
                    want_req(r, i + 1)
                );
            }
            assert_eq!(
                s.handle(2, true, &fr(r, rx_of(&pts[usize::from(k)], k))),
                want_ack(r, result::ERROR, 0, 0),
                "item {k}"
            );
            assert!(!s.busy());
            assert!(s.items().is_empty());
            assert_eq!(s.change_counter(), 0);
        }
        assert_eq!(
            s.handle(0, true, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 3, opaque_id: plan_id_of(&pts) })
        );
        for (i, p) in pts.iter().enumerate() {
            let want = if i == 2 {
                TxMsg::ReturnItem { seq: 2, z_bits: p.z_bits }
            } else {
                TxMsg::Item { seq: i as u16, x: p.x, y: p.y, z_bits: p.z_bits }
            };
            assert_eq!(s.handle(1, true, &fr(r, req(i as u16))), to_station(r, want));
        }
        assert_eq!(s.handle(2, true, &fr(r, done_ack())), None);
        assert_eq!(held(&s), before);
    }


    #[test]
    fn download_returns_exactly_what_was_committed() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let sent = [
            Item { x: -900_000_000, y: 1_800_000_000, z_bits: 5.0f32.to_bits() },
            Item { x: 1, y: -1, z_bits: 50.0f32.to_bits() },
            Item { x: 0, y: 0, z_bits: 12.5f32.to_bits() },
        ];
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        for (i, p) in sent.iter().enumerate() {
            let mut it = RxItem {
                seq: i as u16,
                frame: 6,
                command: if i == 2 { 20 } else { 16 },
                x: p.x,
                y: p.y,
                z_bits: p.z_bits,
                mission_type: 0,
            };
            it.frame = if i == 1 { 3 } else { 6 };
            let out = s.handle(1 + i as u64, false, &fr(r, Rx::ItemInt(it)));
            if i < 2 {
                assert_eq!(out, want_req(r, i as u16 + 1));
            } else {
                assert_eq!(out, want_ack(r, 0, 0, plan_id_of(&sent)));
            }
        }
        assert_eq!(
            s.handle(10, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 3, opaque_id: plan_id_of(&sent) })
        );
        for seq in [2u16, 0, 1, 1, 0, 2, 2] {
            let p = sent[usize::from(seq)];
            let want = if seq == 2 {
                TxMsg::ReturnItem { seq, z_bits: p.z_bits }
            } else {
                TxMsg::Item { seq, x: p.x, y: p.y, z_bits: p.z_bits }
            };
            assert_eq!(s.handle(11, false, &fr(r, req(seq))), to_station(r, want));
        }
        assert!(s.busy());
        assert_eq!(s.handle(12, false, &fr(r, item(0, 1, 1))), None);
        assert_eq!(s.handle(13, false, &fr(r, done_ack())), None);
        assert!(!s.busy());
        assert_eq!(s.items(), &sent);
    }

    #[test]
    fn download_of_empty_list_waits_for_the_ack() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(
            s.handle(0, false, &fr(r, req_list())),
            to_station(r, TxMsg::Count { count: 0, opaque_id: 0 })
        );
        assert!(s.busy());
        assert_eq!(s.handle(1, false, &fr(r, done_ack())), None);
        assert!(!s.busy());
    }


    #[test]
    fn repeated_count_restarts_the_upload_silently() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let a = many(4);
        assert_eq!(s.handle(0, false, &fr(r, count(4))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, rx_of(&a[0], 0))), want_req(r, 1));
        assert_eq!(s.handle(2, false, &fr(r, rx_of(&a[1], 1))), want_req(r, 2));
        assert_eq!(s.handle(3, false, &fr(r, count(2))), want_req(r, 0));
        let b = [a[2], a[3]];
        assert_eq!(s.handle(4, false, &fr(r, rx_of(&b[0], 0))), want_req(r, 1));
        assert_eq!(s.handle(5, false, &fr(r, rx_of(&b[1], 1))), want_ack(r, 0, 0, plan_id_of(&b)));
        assert_eq!(s.items(), &b);
        assert_eq!(s.change_counter(), 1);
        assert_eq!(s.handle(6, false, &fr(r, count(2))), want_req(r, 0));
        assert_eq!(s.handle(7, false, &fr(r, count(99))), want_ack(r, result::NO_SPACE, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.items(), &b);
    }

    #[test]
    fn repeated_request_list_and_clear_by_the_owner() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        let _ = upload(&mut s, r, &full3(), 0);
        let count_msg = to_station(r, TxMsg::Count { count: 4, opaque_id: PLAN3R });
        assert_eq!(s.handle(0, false, &fr(r, req_list())), count_msg);
        assert!(s.handle(1, false, &fr(r, req(2))).is_some());
        assert_eq!(s.handle(2, false, &fr(r, req_list())), count_msg);
        assert_eq!(s.handle(3, false, &fr(r, clear_all())), want_ack(r, 0, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.count(), 0);
        let _ = upload(&mut s, r, &full3(), 10);
        assert_eq!(s.handle(100, false, &fr(r, count(3))), want_req(r, 0));
        assert_eq!(s.handle(101, false, &fr(r, clear_all())), want_ack(r, 0, 0, 0));
        assert!(!s.busy());
        assert_eq!(s.count(), 0);
    }

    #[test]
    fn upload_ignores_request_int_and_ack_cancels_without_reply() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(2))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, req(0))), None);
        assert!(s.busy());
        assert_eq!(s.poll(1500 * MS), want_req(r, 0));
        for result in [0u8, 1, 4, 15] {
            let mut s = MissionService::new();
            assert_eq!(s.handle(0, false, &fr(r, count(2))), want_req(r, 0));
            assert_eq!(s.handle(1, false, &fr(r, Rx::Ack { result, mission_type: 0 })), None);
            assert!(!s.busy());
            assert_eq!(s.poll(100_000 * MS), None);
        }
    }


    #[test]
    fn lost_final_ack_is_answered_again() {
        let pts = full3();
        let mut s = MissionService::new();
        let r = Link::Radio;
        let end = upload(&mut s, r, &pts, 0);
        assert_eq!(end, want_ack(r, 0, 0, PLAN3R));
        let changes = s.change_counter();
        for k in 0..3 {
            assert_eq!(
                s.handle(1000 * MS + k, false, &fr(r, rx_of(&pts[3], 3))),
                want_ack(r, 0, 0, PLAN3R)
            );
            assert!(!s.busy());
            assert_eq!(s.change_counter(), changes);
        }
        assert_eq!(s.handle(1100 * MS, true, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 0, 0, PLAN3R));
        assert_eq!(s.handle(1200 * MS, false, &fr(r, rx_of(&pts[2], 2))), want_ack(r, 1, 0, 0));
        let o = frame_from(r, (255, 191), rx_of(&pts[3], 3));
        assert_eq!(
            s.handle(1300 * MS, false, &o),
            Some(Tx { link: r, dst_system: 255, dst_component: 191, msg: TxMsg::Ack { result: 1, mission_type: 0, opaque_id: 0 } })
        );
        assert_eq!(
            s.handle(1400 * MS, false, &fr(Link::Usb, rx_of(&pts[3], 3))),
            want_ack(Link::Usb, 1, 0, 0)
        );
        assert_eq!(s.handle(1500 * MS, false, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 0, 0, PLAN3R));
    }

    #[test]
    fn lost_final_ack_of_a_refusal_at_an_item() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, item(0, 1, 1))), want_req(r, 1));
        let mut bad = rx_item(1, 1, 1, 0.0);
        bad.frame = 9;
        assert_eq!(
            s.handle(2, false, &fr(r, Rx::ItemInt(bad))),
            want_ack(r, result::UNSUPPORTED_FRAME, 0, 0)
        );
        assert_eq!(
            s.handle(3, false, &fr(r, Rx::ItemInt(bad))),
            want_ack(r, result::UNSUPPORTED_FRAME, 0, 0)
        );
        assert!(!s.busy());
        assert_eq!(s.change_counter(), 0);
    }

    #[test]
    fn lost_final_ack_is_forgotten_by_a_start_message_and_by_link_down() {
        let pts = full3();
        let r = Link::Radio;
        for start in [count(3), count(17), count(0), req_list(), clear_all(), Rx::Count { count: 4, mission_type: 4 }] {
            let mut s = MissionService::new();
            let _ = upload(&mut s, r, &pts, 0);
            let _ = s.handle(10, false, &fr(r, start));
            let _ = s.handle(11, false, &fr(r, done_ack()));
            s.link_down(r);
            let _ = s.handle(12, false, &fr(r, done_ack()));
            assert_eq!(s.handle(13, false, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 1, 0, 0), "{start:?}");
        }
        for start in [count(17), req_list(), clear_all()] {
            let mut s = MissionService::new();
            let _ = upload(&mut s, r, &pts, 0);
            let _ = s.handle(10, false, &fr(r, start));
            let _ = s.handle(11, false, &fr(r, done_ack()));
            assert_eq!(s.handle(13, false, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 1, 0, 0), "{start:?}");
        }
        let mut s = MissionService::new();
        let _ = upload(&mut s, r, &pts, 0);
        s.link_down(Link::Usb);
        assert_eq!(s.handle(1, false, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 0, 0, PLAN3R));
        s.link_down(r);
        assert_eq!(s.handle(2, false, &fr(r, rx_of(&pts[3], 3))), want_ack(r, 1, 0, 0));
    }

    #[test]
    fn no_memory_of_a_timed_out_upload() {
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(1))), want_req(r, 0));
        let mut t = 0;
        for _ in 0..6 {
            t += 1500 * MS;
            let _ = s.poll(t);
        }
        assert!(!s.busy());
        assert_eq!(s.handle(t, false, &fr(r, item(0, 1, 1))), want_ack(r, 1, 0, 0));
        assert_eq!(s.change_counter(), 0);
    }


    #[test]
    fn addressing_other_targets_dropped() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        let mut f = fr(r, count(2));
        f.target_system = 2;
        assert_eq!(s.handle(0, false, &f), None);
        f.target_system = 1;
        f.target_component = 2;
        assert_eq!(s.handle(0, false, &f), None);
        assert!(!s.busy());
        f.target_system = 0;
        f.target_component = 0;
        assert_eq!(s.handle(0, false, &f), want_req(r, 0));
        let mut g = frame_from(Link::Radio, (255, 191), count(2));
        g.target_system = 9;
        assert_eq!(s.handle(1, false, &g), None);
    }

    #[test]
    fn dropped_frame_does_not_refresh_download_timer() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert!(s.handle(0, false, &fr(r, req_list())).is_some());
        let mut f = fr(r, req(0));
        f.target_component = 3;
        assert_eq!(s.handle(8000 * MS, false, &f), None);
        assert_eq!(s.poll(9000 * MS), want_ack(r, result::OPERATION_CANCELLED, 0, 0));
    }

    #[test]
    fn change_counter_counts_commits_and_clears_only() {
        let mut s = MissionService::new();
        let r = Link::Usb;
        assert_eq!(s.change_counter(), 0);
        let _ = upload(&mut s, r, &many(2), 0);
        assert_eq!(s.change_counter(), 1);
        assert!(s.handle(0, false, &fr(r, req_list())).is_some());
        assert_eq!(s.handle(1, false, &fr(r, done_ack())), None);
        let _ = s.handle(2, false, &fr(r, count(17)));
        let _ = s.handle(2, true, &fr(r, clear_all()));
        assert_eq!(s.change_counter(), 1);
        let _ = s.handle(3, false, &fr(r, clear_all()));
        assert_eq!(s.change_counter(), 2);
        let _ = upload(&mut s, r, &many(1), 4);
        assert_eq!(s.change_counter(), 3);
    }

    #[test]
    fn poll_without_transaction_is_silent() {
        let mut s = MissionService::new();
        assert_eq!(s.poll(0), None);
        assert_eq!(s.poll(u64::MAX), None);
    }


    const PARAM_TARGETS: [(i32, i32); 4] = [(10, 20), (30, 40), (50, 60), (70, 80)];
    /// `GDN_ALT_MIN`, `GDN_ALT_MAX` at their defaults.
    const LIMITS: (f32, f32) = (ALT_MIN_DEFAULT, ALT_MAX_DEFAULT);

    #[test]
    fn source_0_is_the_parameters_whatever_the_mission_holds() {
        let mut want = GdnTargets {
            count: 3,
            radius_m: 2.5,
            targets: [(0, 0); GDN_TARGETS],
            alts_m: [0.0; GDN_TARGETS],
            return_m: 0.0,
            plan_id: 0,
            from_mission: false,
            heights_ok: false,
            alt_max_m: ALT_MAX_DEFAULT,
            mission_stored: false,
        };
        want.targets[..4].copy_from_slice(&PARAM_TARGETS);
        let missions = [many(0), many(1), many(3), many(7), many(16), points3().to_vec()];
        for mission in &missions {
            let want = GdnTargets { mission_stored: mission.len() >= 2, ..want };
            assert_eq!(select_targets(0, 3, 2.5, &PARAM_TARGETS, mission, 7, LIMITS), want, "{} items", mission.len());
        }
        let want = GdnTargets { mission_stored: true, ..want };
        assert_eq!(select_targets(2, 3, 2.5, &PARAM_TARGETS, &many(5), 7, LIMITS), want);
        assert_eq!(select_targets(-1, 3, 2.5, &PARAM_TARGETS, &many(5), 7, LIMITS), want);
        assert_eq!(select_targets(0, 3, 2.5, &PARAM_TARGETS, &many(5), 7, (5.0, 35.0)).alt_max_m, 35.0);
        assert_eq!(select_targets(1, 3, 2.5, &PARAM_TARGETS, &many(5), 7, (5.0, 35.0)).alt_max_m, 35.0);
    }

    #[test]
    fn source_0_clamps_the_count_as_the_board_did() {
        let clamp = |n| select_targets(0, n, 2.0, &PARAM_TARGETS, &many(17), 7, LIMITS).count;
        assert_eq!([clamp(-5), clamp(0), clamp(4), clamp(5), clamp(i32::MAX)], [0, 0, 4, 4, 4]);
        assert_eq!(select_targets(0, 0, 2.0, &PARAM_TARGETS, &many(17), 7, LIMITS).count, 0);
    }

    #[test]
    fn source_1_differs_when_the_mission_is_not_empty() {
        for n in [2usize, 4, 17] {
            let mission = many(n);
            let params = select_targets(0, 3, 2.5, &PARAM_TARGETS, &mission, 7, LIMITS);
            let from_mission = select_targets(1, 3, 2.5, &PARAM_TARGETS, &mission, 7, LIMITS);
            assert_ne!(from_mission, params, "{n} items");
            assert_eq!(usize::from(from_mission.count), n - 1);
            assert_eq!(from_mission.radius_m, 2.5, "the radius is still GDN_RADIUS");
            for (k, it) in mission[..n - 1].iter().enumerate() {
                assert_eq!(from_mission.targets[k], (it.x, it.y));
                assert_eq!(from_mission.alts_m[k], f32::from_bits(it.z_bits));
            }
            assert!(from_mission.targets[n - 1..].iter().all(|t| *t == (0, 0)));
            assert!(from_mission.alts_m[n - 1..].iter().all(|z| *z == 0.0));
            assert_eq!(from_mission.return_m, 20.0);
            assert_eq!(from_mission.plan_id, 7);
        }
    }

    #[test]
    fn source_1_with_the_return_item_alone_has_no_targets_but_the_return_height() {
        let g = select_targets(1, 4, 2.5, &PARAM_TARGETS, &many(1), 9, LIMITS);
        assert_eq!((g.count, g.return_m, g.plan_id), (0, 20.0, 9));
        assert!(g.targets.iter().all(|t| *t == (0, 0)));
        assert!(g.alts_m.iter().all(|z| *z == 0.0));
    }

    #[test]
    fn mission_stored_is_at_least_one_waypoint_whatever_the_source() {
        for source in [0, 1, 2] {
            let g = |mission: &[Item]| select_targets(source, 3, 2.5, &PARAM_TARGETS, mission, 7, LIMITS).mission_stored;
            assert!(!g(&[]), "source {source}: empty");
            assert!(!g(&many(1)), "source {source}: the return item alone");
            assert!(g(&many(2)), "source {source}: one waypoint and the return item");
            assert!(g(&many(4)), "source {source}: three waypoints and the return item");
            assert!(g(&many(16)), "source {source}: full");
        }
        let a = select_targets(0, 3, 2.5, &PARAM_TARGETS, &many(4), 7, LIMITS);
        assert_eq!((a.count, a.from_mission, a.heights_ok, a.plan_id, a.return_m), (3, false, false, 0, 0.0));
        let b = select_targets(1, 3, 2.5, &PARAM_TARGETS, &many(4), 7, LIMITS);
        assert_eq!((b.count, b.from_mission, b.heights_ok, b.plan_id, b.return_m), (3, true, true, 7, 20.0));
    }

    #[test]
    fn source_1_with_an_empty_mission_is_off() {
        let g = select_targets(1, 4, 3.0, &PARAM_TARGETS, &[], 7, LIMITS);
        assert_eq!(g.count, 0);
        assert_eq!(g.radius_m, 3.0);
        assert_eq!((g.return_m, g.plan_id), (0.0, 0));
        assert!(g.targets.iter().all(|t| *t == (0, 0)));
    }

    #[test]
    fn source_1_ignores_the_parameter_targets_and_count() {
        let mission = many(3);
        let a = select_targets(1, 0, 2.0, &PARAM_TARGETS, &mission, 7, LIMITS);
        let b = select_targets(1, 4, 2.0, &[], &mission, 7, LIMITS);
        assert_eq!(a, b);
        assert_eq!(a.count, 2);
    }

    #[test]
    fn the_source_and_the_height_check_against_the_limits_given() {
        let g = |source, mission: &[Item], limits| select_targets(source, 3, 2.0, &PARAM_TARGETS, mission, 7, limits);
        assert!(!g(0, &many(4), LIMITS).from_mission && !g(0, &many(4), LIMITS).heights_ok, "the parameters: no heights");
        assert!(g(1, &many(4), LIMITS).from_mission && g(1, &many(4), LIMITS).heights_ok);
        assert!(g(1, &many(4), (10.0, 20.0)).heights_ok);
        assert!(!g(1, &many(4), (10.5, 20.0)).heights_ok, "waypoint 0 below");
        assert!(!g(1, &many(4), (5.0, 19.0)).heights_ok, "the return height above");
        assert!(!g(1, &many(4), (f32::NAN, 50.0)).heights_ok);
        assert!(g(1, &[], LIMITS).from_mission && !g(1, &[], LIMITS).heights_ok);
        assert!(g(1, &many(1), LIMITS).heights_ok);
    }


    #[test]
    fn decode_reads_every_message_raw_and_zero_extends() {
        assert_eq!(decode(44, &[3, 0, 1, 1]), Some((1, 1, Rx::Count { count: 3, mission_type: 0 })));
        assert_eq!(
            decode(44, &[17, 0, 0, 1, 255, 1, 2, 3, 4]),
            Some((0, 1, Rx::Count { count: 17, mission_type: 255 }))
        );
        assert_eq!(decode(44, &[]), Some((0, 0, Rx::Count { count: 0, mission_type: 0 })));
        assert_eq!(decode(43, &[1, 1]), Some((1, 1, Rx::RequestList { mission_type: 0 })));
        assert_eq!(decode(45, &[1, 1, 255]), Some((1, 1, Rx::ClearAll { mission_type: 255 })));
        assert_eq!(decode(47, &[1, 1, 15]), Some((1, 1, Rx::Ack { result: 15, mission_type: 0 })));
        assert_eq!(decode(51, &[2, 1, 1, 1]), Some((1, 1, Rx::RequestInt { seq: 258, mission_type: 0 })));
        assert_eq!(decode(51, &[2, 1, 1, 1, 9]), Some((1, 1, Rx::RequestInt { seq: 258, mission_type: 9 })));
        for id in [0u32, 1, 39, 40, 42, 46, 48, 50, 52, 70, 74, 109, 42421] {
            assert_eq!(decode(id, &[0; 38]), None, "id {id}");
        }
    }

    #[test]
    fn decode_item_int_of_example_1_item_1() {
        let mut p = std::vec::Vec::new();
        p.extend_from_slice(&[0u8; 16]);
        p.extend_from_slice(&100_010_000i32.to_le_bytes());
        p.extend_from_slice(&(-300_000_000i32).to_le_bytes());
        p.extend_from_slice(&12.5f32.to_le_bytes());
        p.extend_from_slice(&1u16.to_le_bytes());
        p.extend_from_slice(&16u16.to_le_bytes());
        p.extend_from_slice(&[1, 1, 3, 0, 1]);
        assert_eq!(p.len(), 37);
        let (ts, tc, rx) = decode(73, &p).unwrap();
        assert_eq!((ts, tc), (1, 1));
        assert_eq!(rx, Rx::ItemInt(rx_item(1, 100_010_000, -300_000_000, 12.5)));
        let mut s = MissionService::new();
        let r = Link::Radio;
        assert_eq!(s.handle(0, false, &fr(r, count(3))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, item(0, 1, 1))), want_req(r, 1));
        assert_eq!(s.handle(2, false, &fr(r, rx)), want_req(r, 2));
        let mut q = p.clone();
        q.resize(38, 0);
        q[34] = 99;
        let Rx::ItemInt(bad) = decode(73, &q).unwrap().2 else { panic!() };
        assert_eq!(bad.frame, 99);
        q[34] = 3;
        q[30..32].copy_from_slice(&400u16.to_le_bytes());
        let Rx::ItemInt(bad) = decode(73, &q).unwrap().2 else { panic!() };
        assert_eq!(bad.command, 400);
        q[37] = 7;
        let Rx::ItemInt(bad) = decode(73, &q).unwrap().2 else { panic!() };
        assert_eq!(bad.mission_type, 7);
    }

    #[test]
    fn incremental_plan_id_equals_one_shot_crc() {
        let one_shot = |pts: &[Item]| {
            let mut bytes = std::vec::Vec::new();
            for p in pts {
                bytes.extend_from_slice(&p.x.to_le_bytes());
                bytes.extend_from_slice(&p.y.to_le_bytes());
                bytes.extend_from_slice(&p.z_bits.to_le_bytes());
            }
            plan_id_from_crc(crc32(&bytes))
        };
        let varied = |n: usize| -> std::vec::Vec<Item> {
            (0..n)
                .map(|i| Item {
                    x: if i + 1 == n { 0 } else { (i as i32 * 7_919_113 - 400_000_000) % 900_000_000 },
                    y: if i + 1 == n { 0 } else { -(i as i32) * 12_345_679 + 5 },
                    z_bits: (i as f32 * 2.75 + 5.1).to_bits(),
                })
                .collect()
        };
        let r = Link::Usb;
        for n in [1usize, 2, 3, 7, 17] {
            let pts = varied(n);
            let mut s = MissionService::new();
            let end = upload(&mut s, r, &pts, 0);
            assert_eq!(end, want_ack(r, 0, 0, one_shot(&pts)), "n={n}");
            assert_eq!(s.plan_id(), one_shot(&pts));
            assert_eq!(s.plan_id(), plan_id_of(s.items()));
        }
        let mut s = MissionService::new();
        let _ = upload(&mut s, r, &full3(), 0);
        assert_eq!(s.plan_id(), PLAN3R);
        assert_eq!(one_shot(&full3()), PLAN3R);
        assert_eq!(one_shot(&points3()), PLAN3);

        let a = varied(6);
        let b = varied(9).split_off(2);
        let mut s = MissionService::new();
        assert_eq!(s.handle(0, false, &fr(r, count(6))), want_req(r, 0));
        assert_eq!(s.handle(1, false, &fr(r, rx_of(&a[0], 0))), want_req(r, 1));
        assert_eq!(s.handle(2, false, &fr(r, rx_of(&a[1], 1))), want_req(r, 2));
        assert_eq!(s.handle(3, false, &fr(r, rx_of(&a[1], 1))), want_req(r, 2));
        assert_eq!(s.handle(4, false, &fr(r, rx_of(&a[4], 4))), want_req(r, 2));
        assert_eq!(s.handle(5, false, &fr(r, count(7))), want_req(r, 0));
        let mut out = None;
        let mut t = 6;
        for (i, it) in b.iter().enumerate() {
            if i == 3 {
                assert_eq!(s.handle(t, false, &fr(r, rx_of(&b[5], 5))), want_req(r, 3));
                assert_eq!(s.handle(t + 1, false, &fr(r, rx_of(&b[1], 1))), want_req(r, 3));
                t += 2;
            }
            out = s.handle(t, false, &fr(r, rx_of(it, i as u16)));
            t += 1;
        }
        assert_eq!(out, want_ack(r, 0, 0, one_shot(&b)));
        assert_eq!(s.items(), &b[..]);
        assert_eq!(s.plan_id(), one_shot(&b));
    }
}
