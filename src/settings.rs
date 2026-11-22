//! The parameters in the in-package flash (design: decision-foundation item 3, the two-core
//! amendment "Item 3, changes", and the owner's answers of 2026-11-15, which replace the
//! automatic save). Two 4096-byte sectors A and B at the end of the 2 MiB flash (`memory.x`
//! `SETTINGS`), one `fw_core::record` record each. Read at boot before the devices start, and
//! again on "reload"; written ONLY by an explicit save (MAV_CMD_PREFLIGHT_STORAGE param1 = 1),
//! never automatically. Core 0 only (the HAL refuses a flash write from core 1).
//!
//! Every erase and program goes through [`guarded`], which refuses any range outside the
//! settings region with a `[FAIL]` line. An erase or program stops both cores (interrupts off on
//! core 0, core 1 paused through its FIFO interrupt): the stall is measured and published as
//! [`SAVE_EPOCH`] and the last window ([`overlaps_save`]), so the supervisor can discard what it
//! caused.

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_rp::Peri;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use fw_core::handshake::Handshake;
use fw_core::params::{Apply, LoadError};
use fw_core::record::{self, Boot, Decoded, Entry, SECTOR_LEN, Sector};
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::MarvDevice;
use mavlink_device::{ParamName, ParamStore, ParamValue};
use static_cell::ConstStaticCell;

use crate::store;

/// The flash size the HAL's `Flash` type checks against (RP2350B with 2 MiB in the package).
const FLASH_SIZE: usize = 2 * 1024 * 1024;
/// The settings region as offsets from the start of flash (0x101FE000 to 0x10200000 in the
/// address map): the ONLY range this firmware erases or programs.
pub const REGION_START: u32 = 0x001F_E000;
pub const REGION_END: u32 = 0x0020_0000;
const SECTOR: u32 = SECTOR_LEN as u32;
const _: () = assert!(REGION_END - REGION_START == 2 * SECTOR);
const _: () = assert!(REGION_END as usize == FLASH_SIZE);

/// A save needs the flight core's `PONG` seen fresh this recently (the HAL pauses core 1 through
/// its FIFO interrupt, and waits for it without a time limit).
const PONG_FRESH_MAX_MS: u32 = 100;
/// Skipped entries named in a text each; the rest are counted in one more text.
const SKIP_TEXTS: usize = 4;
/// Entries the feature `save-foreign` adds to one save.
const FOREIGN: usize = if cfg!(feature = "save-foreign") { 2 } else { 0 };

/// The arm/save handshake; `ARMED` is set and cleared by the flight core's gate (`fw_flight::gate`).
pub static HANDSHAKE: Handshake = Handshake::new();
/// Bumped after every flash stall.
pub static SAVE_EPOCH: AtomicU32 = AtomicU32::new(0);
/// The range guard's refusals since boot.
pub static GUARD_REFUSALS: AtomicU32 = AtomicU32::new(0);

/// The last flash stall on core 0's clock; `end_us` is `u64::MAX` while it runs.
#[derive(Clone, Copy)]
struct Window {
    start_us: u64,
    end_us: u64,
}

static WINDOW: Mutex<CriticalSectionRawMutex, Cell<Option<Window>>> = Mutex::new(Cell::new(None));

/// True when the last flash stall overlapped `from..=to`.
pub fn overlaps_save(from: Instant, to: Instant) -> bool {
    WINDOW
        .lock(Cell::get)
        .is_some_and(|w| w.start_us <= to.as_micros() && w.end_us >= from.as_micros())
}

type Chip = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// The flash and the three sector buffers (A, B, the record being written), owned by whoever
/// took them out of [`STORAGE`]; no lock is held across a flash operation.
struct Storage {
    flash: Chip,
    bufs: &'static mut [[u8; SECTOR_LEN]; 3],
    write_sector: Sector,
    next_sequence: u32,
}

static BUFS: ConstStaticCell<[[u8; SECTOR_LEN]; 3]> = ConstStaticCell::new([[0; SECTOR_LEN]; 3]);
static STORAGE: Mutex<CriticalSectionRawMutex, Cell<Option<Storage>>> = Mutex::new(Cell::new(None));

/// Builds the storage (no flash access). Called once, before [`boot`].
pub fn init(flash: Peri<'static, FLASH>) {
    let storage = Storage {
        flash: Flash::new_blocking(flash),
        bufs: BUFS.take(),
        write_sector: Sector::A,
        next_sequence: 1,
    };
    STORAGE.lock(|c| c.set(Some(storage)));
}

fn with_storage<R>(f: impl FnOnce(&mut Storage) -> R) -> Option<R> {
    let mut storage = STORAGE.lock(Cell::take)?;
    let result = f(&mut storage);
    STORAGE.lock(|c| c.set(Some(storage)));
    Some(result)
}

const fn offset(sector: Sector) -> u32 {
    match sector {
        Sector::A => REGION_START,
        Sector::B => REGION_START + SECTOR,
    }
}

const fn sector_name(sector: Sector) -> &'static str {
    match sector {
        Sector::A => "A",
        Sector::B => "B",
    }
}

#[derive(Clone, Copy)]
enum Op<'a> {
    Erase { from: u32, to: u32 },
    Program { offset: u32, bytes: &'a [u8] },
}

/// THE one path to a flash erase or program: refuses (a `[FAIL]` line and an error, no flash
/// access) any range that is empty or not inside `REGION_START..REGION_END`.
fn guarded(flash: &mut Chip, op: Op<'_>) -> Result<(), &'static str> {
    let (what, from, to) = match op {
        Op::Erase { from, to } => ("erase", from, to),
        Op::Program { offset, bytes } => (
            "program",
            offset,
            offset.saturating_add(u32::try_from(bytes.len()).unwrap_or(u32::MAX)),
        ),
    };
    if from < REGION_START || to > REGION_END || from >= to {
        GUARD_REFUSALS.fetch_add(1, Ordering::Relaxed);
        defmt::error!(
            "[FAIL] settings: {=str} of {=u32:#x}..{=u32:#x} refused, outside the settings region {=u32:#x}..{=u32:#x}",
            what,
            from,
            to,
            REGION_START,
            REGION_END
        );
        return Err("refused by the range guard");
    }
    let result = match op {
        Op::Erase { from, to } => flash.blocking_erase(from, to),
        Op::Program { offset, bytes } => flash.blocking_write(offset, bytes),
    };
    // The boot ROM's flash routines reset the QMI window of the PSRAM: re-map it and check its canary right
    // here, success or failure, before anything else runs (the P2 argument is in `psram::after_flash_op`).
    crate::psram::after_flash_op();
    result.map_err(|e| {
        defmt::error!("[INFO] settings: {=str} failed: {}", what, e);
        "flash error"
    })
}

fn read(flash: &mut Chip, sector: Sector, buf: &mut [u8; SECTOR_LEN]) -> Result<(), &'static str> {
    flash
        .blocking_read(offset(sector), buf)
        .map_err(|_| "flash read error")
}

/// One sector as the decoder sees it, for the log.
#[derive(defmt::Format)]
enum Seen {
    Erased,
    Invalid,
    Valid { sequence: u32, entries: usize },
}

fn seen(bytes: &[u8; SECTOR_LEN]) -> Seen {
    match record::decode(bytes) {
        Decoded::Erased => Seen::Erased,
        Decoded::Invalid => Seen::Invalid,
        Decoded::Valid(r) => Seen::Valid {
            sequence: r.sequence(),
            entries: r.declared_count(),
        },
    }
}

fn reason(error: LoadError) -> &'static str {
    match error {
        LoadError::UnknownName => "unknown name",
        LoadError::WrongType => "wrong type",
        LoadError::OutOfRange => "out of range",
    }
}

/// After a load: each device's stored values through its validator (`settings_to_config` and the
/// IMUs' `imuN_setup`, what the device's `validate` runs). A refused set goes back to the device's
/// `Configure`- and `Reboot`-class defaults in RAM (its calibration offsets stay; the flash is not
/// touched) and the device's name and the reason are returned, so the device starts on valid values.
fn check_devices() -> [Option<(&'static str, &'static str)>; 4] {
    [
        reset_if_refused(
            MarvDevice::MARV_DEVICE_BARO,
            "BARO",
            store::baro_config().err(),
        ),
        reset_if_refused(
            MarvDevice::MARV_DEVICE_MAG,
            "MAG",
            store::mag_config().err(),
        ),
        reset_if_refused(
            MarvDevice::MARV_DEVICE_IMU1,
            "IMU1",
            store::imu1_setup().err(),
        ),
        reset_if_refused(
            MarvDevice::MARV_DEVICE_IMU2,
            "IMU2",
            store::imu2_setup().err(),
        ),
    ]
}

fn reset_if_refused(
    device: MarvDevice,
    name: &'static str,
    refused: Option<&'static str>,
) -> Option<(&'static str, &'static str)> {
    let reason = refused?;
    let id = device as u8;
    store::with(|p| fw_settings::reset_device_settings(p, id));
    defmt::warn!(
        "[INFO] settings: {=str} stored settings refused ({=str}): {=str} settings to their defaults in RAM, flash unchanged",
        name,
        reason,
        name
    );
    Some((name, reason))
}

fn values() -> [Option<ParamValue>; store::COUNT] {
    store::with(|p| core::array::from_fn(|i| p.get(u16::try_from(i).unwrap_or(u16::MAX))))
        .unwrap_or([None; store::COUNT])
}

/// The boot rule (also "reload"): reads both sectors, `record::choose`; the store goes to its
/// defaults, then the current record's entries are loaded (each failing entry skipped with a
/// text, at most [`SKIP_TEXTS`] named). No valid record: defaults, one text; nothing is erased.
/// Returns the APPLY mask of the devices whose values changed.
fn load_from_flash(st: &mut Storage, what: &str) -> Result<u32, &'static str> {
    let [a, b, _] = &mut *st.bufs;
    read(&mut st.flash, Sector::A, a)?;
    read(&mut st.flash, Sector::B, b)?;
    let choice = record::choose(a, b);
    st.write_sector = choice.write_sector;
    st.next_sequence = choice.next_sequence;

    let before = values();
    let mut skips: [Option<(ParamName, &'static str)>; SKIP_TEXTS] = [None; SKIP_TEXTS];
    let mut loaded = 0u16;
    let mut skipped = 0u16;
    store::with(|p| {
        p.reset_all();
        let _ = (p.take_configure_mask(), p.take_live_mask(), p.take_notice());
        if let Boot::Current { record, .. } = choice.boot {
            for entry in record.entries() {
                let why = match p.load(&entry.name, entry.value) {
                    Ok(true) => {
                        loaded = loaded.saturating_add(1);
                        continue;
                    }
                    Ok(false) => "read only",
                    Err(e) => reason(e),
                };
                if let Some(slot) = skips.get_mut(usize::from(skipped)) {
                    *slot = Some((entry.name, why));
                }
                skipped = skipped.saturating_add(1);
            }
            skipped = skipped.saturating_add(u16::try_from(record.skipped()).unwrap_or(u16::MAX));
        }
        p.clear_dirty();
    })
    .ok_or("store not ready")?;
    let refused = check_devices();
    let after = values();
    let mut configure = 0u32;
    for ((def, old), new) in fw_settings::TABLE.iter().zip(before).zip(after) {
        if old != new && matches!(def.apply, Apply::Configure) {
            configure |= 1u32.checked_shl(u32::from(def.device)).unwrap_or(0);
        }
    }

    let (class, sector, sequence) = match choice.boot {
        Boot::Current { sector, record } => {
            let mut named = 0u16;
            for (name, why) in skips.iter().flatten() {
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("settings: skipped {} ({})", name.as_str(), why),
                );
                named += 1;
            }
            if skipped > named {
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("settings: ... and {} more skipped", skipped - named),
                );
            }
            ("record", sector_name(sector), record.sequence())
        }
        Boot::DefaultsErased => {
            crate::text_sticky(
                MavSeverity::MAV_SEVERITY_INFO,
                "settings: none stored, defaults",
            );
            ("none stored (both sectors erased), defaults", "-", 0)
        }
        Boot::DefaultsInvalid => {
            crate::text_sticky(
                MavSeverity::MAV_SEVERITY_WARNING,
                "settings: stored record invalid, defaults",
            );
            ("stored record invalid, defaults (nothing erased)", "-", 0)
        }
    };
    for (name, _) in refused.iter().flatten() {
        crate::text_fmt(
            MavSeverity::MAV_SEVERITY_WARNING,
            format_args!("{}: stored settings invalid, defaults", name),
        );
    }
    defmt::info!(
        "[INFO] settings {=str}: {=str}, sector {=str}, sequence {=u32}, {=u16} loaded, {=u16} skipped; sector A {}, sector B {}; next save to sector {=str} with sequence {=u32}; apply mask {=u32:#x}",
        what,
        class,
        sector,
        sequence,
        loaded,
        skipped,
        seen(a),
        seen(b),
        sector_name(choice.write_sector),
        choice.next_sequence,
        configure
    );
    Ok(configure)
}

/// At boot, after `store::init` and before the devices start.
pub fn boot() {
    match with_storage(|st| load_from_flash(st, "boot")) {
        Some(Ok(_)) => {}
        Some(Err(e)) => defmt::error!("[FAIL] settings boot: {=str}, running on defaults", e),
        None => defmt::error!("[FAIL] settings boot: storage not ready, running on defaults"),
    }
}

/// MAV_CMD_PREFLIGHT_STORAGE param1 = 0: the boot rule again; the devices whose values changed
/// get their APPLY request.
pub fn reload() -> Result<(), &'static str> {
    let configure = with_storage(|st| load_from_flash(st, "reload")).ok_or("storage busy")??;
    crate::device::raise_apply(configure);
    crate::calibration::raise_push(u32::MAX);
    Ok(())
}

/// MAV_CMD_PREFLIGHT_STORAGE param1 = 1, the ONLY writer of the flash: refused unless the flight
/// core's `PONG` is fresh and the arm/save handshake grants it; writes the sector not holding the
/// current record, reads it back, and clears the store's dirty flag only when it verified.
/// Returns the measured stall in us.
pub fn save() -> Result<u64, &'static str> {
    let now_ms = Instant::now().as_millis() as u32;
    let age_ms = now_ms.wrapping_sub(crate::PONG_FRESH_MS.load(Ordering::Acquire));
    if age_ms > PONG_FRESH_MAX_MS {
        defmt::warn!(
            "[INFO] settings save refused: flight core not answering (PONG last fresh {=u32} ms ago), nothing written",
            age_ms
        );
        return Err("flight core not answering");
    }
    let result = with_storage(|st| {
        if !HANDSHAKE.begin_save() {
            defmt::warn!("[INFO] settings save refused: armed, nothing written");
            return Err("armed");
        }
        let written = write_record(st);
        HANDSHAKE.end_save();
        written
    })
    .unwrap_or(Err("storage busy"));
    if result.is_ok() {
        store::with(|p| p.clear_dirty());
    }
    result
}

fn write_record(st: &mut Storage) -> Result<u64, &'static str> {
    let mut entries: [Option<Entry>; store::COUNT + FOREIGN] = [None; store::COUNT + FOREIGN];
    let count = store::with(|p| {
        let mut n = 0usize;
        for (name, value) in p.non_default() {
            if let Some(slot) = entries.get_mut(n) {
                *slot = Some(Entry { name, value });
                n += 1;
            }
        }
        n
    })
    .ok_or("store not ready")?;
    #[cfg(feature = "save-foreign")]
    let count = add_foreign(&mut entries, count);
    let sector = st.write_sector;
    let sequence = st.next_sequence;
    let at = offset(sector);
    let [_, check, out] = &mut *st.bufs;
    let len = record::encode(sequence, entries.iter().flatten().copied(), out)
        .map_err(|_| "too many entries")?;

    let begun = Instant::now();
    WINDOW.lock(|w| {
        w.set(Some(Window {
            start_us: begun.as_micros(),
            end_us: u64::MAX,
        }));
    });
    let erased = guarded(
        &mut st.flash,
        Op::Erase {
            from: at,
            to: at + SECTOR,
        },
    );
    let erase_end = Instant::now();
    let programmed = erased.and_then(|()| {
        guarded(
            &mut st.flash,
            Op::Program {
                offset: at,
                bytes: out.get(..len).unwrap_or(&[]),
            },
        )
    });
    let ended = Instant::now();
    WINDOW.lock(|w| {
        w.set(Some(Window {
            start_us: begun.as_micros(),
            end_us: ended.as_micros(),
        }));
    });
    SAVE_EPOCH.fetch_add(1, Ordering::Release);

    let verified = programmed
        .and_then(|()| read(&mut st.flash, sector, check))
        .and_then(|()| match record::decode(check) {
            Decoded::Valid(r)
                if *check == *out
                    && r.sequence() == sequence
                    && r.declared_count() == count
                    && r.skipped() == 0 =>
            {
                Ok(())
            }
            _ => Err("verify failed"),
        });
    let stall_us = (ended - begun).as_micros();
    defmt::info!(
        "[INFO] settings save: sector {=str}, sequence {=u32}, {=usize} entries, {=usize} bytes, stall {=u64} us (erase {=u64} us, program {=u64} us), epoch {=u32}, verify {=str}",
        sector_name(sector),
        sequence,
        count,
        len,
        stall_us,
        (erase_end - begun).as_micros(),
        (ended - erase_end).as_micros(),
        SAVE_EPOCH.load(Ordering::Relaxed),
        match verified {
            Ok(()) => "ok",
            Err(e) => e,
        }
    );
    verified?;
    st.write_sector = sector.other();
    st.next_sequence = sequence.saturating_add(1);
    Ok(stall_us)
}

/// Feature `corrupt-newest`: at boot, before the settings are read, ONE byte inside the newest
/// valid record (the first entry's value byte, else the CRC's first byte, else the magic's,
/// whichever is not already 0x00) is programmed to 0x00 through [`guarded`].
#[cfg(feature = "corrupt-newest")]
pub fn corrupt_newest() {
    let _ = with_storage(|st| {
        let [a, b, _] = &mut *st.bufs;
        if read(&mut st.flash, Sector::A, a).is_err() || read(&mut st.flash, Sector::B, b).is_err()
        {
            defmt::error!("[INFO] corrupt-newest: read failed, nothing changed");
            return;
        }
        let Boot::Current { sector, record } = record::choose(a, b).boot else {
            defmt::info!("[INFO] corrupt-newest: no valid record, nothing changed");
            return;
        };
        let bytes: &[u8; SECTOR_LEN] = if sector == Sector::A { a } else { b };
        let count = record.declared_count();
        let candidates = [
            (count > 0).then_some(record::HEADER_LEN + 20),
            Some(record::HEADER_LEN + record::ENTRY_LEN * count),
            Some(0),
        ];
        let Some((k, old)) = candidates
            .into_iter()
            .flatten()
            .find_map(|k| bytes.get(k).copied().filter(|&v| v != 0).map(|v| (k, v)))
        else {
            return;
        };
        let result = guarded(
            &mut st.flash,
            Op::Program {
                offset: offset(sector) + k as u32,
                bytes: &[0x00],
            },
        );
        defmt::info!(
            "[INFO] corrupt-newest: byte {=usize} of sector {=str} (sequence {=u32}) {=u8:#04x} -> 0x00: {=str}",
            k,
            sector_name(sector),
            record.sequence(),
            old,
            if result.is_ok() {
                "programmed"
            } else {
                "FAILED"
            }
        );
    });
}

/// Feature `save-foreign`: the first save after boot also stores `ZZZ_UNKNOWN` = 1 (no such
/// parameter) and `BARO_ODR_HZ` = 33 (in the table's range, refused by the validator), as a record
/// from another firmware version might.
#[cfg(feature = "save-foreign")]
fn add_foreign(entries: &mut [Option<Entry>], mut count: usize) -> usize {
    use core::sync::atomic::AtomicBool;
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return count;
    }
    let (Ok(unknown), Ok(odr)) = (ParamName::new("ZZZ_UNKNOWN"), ParamName::new("BARO_ODR_HZ"))
    else {
        return count;
    };
    for (name, value) in [(unknown, ParamValue::I32(1)), (odr, ParamValue::I32(33))] {
        if let Some(entry) = entries
            .iter_mut()
            .flatten()
            .find(|e| e.name.as_str() == name.as_str())
        {
            entry.value = value;
        } else if let Some(slot) = entries.get_mut(count) {
            *slot = Some(Entry { name, value });
            count += 1;
        }
    }
    defmt::warn!("[INFO] save-foreign: this save also stores ZZZ_UNKNOWN = 1 and BARO_ODR_HZ = 33");
    count
}

/// Feature `guard-negative`: three ranges the guard must refuse before any flash call (the
/// sector just below the region, a byte just above its end, an empty range inside it).
#[cfg(feature = "guard-negative")]
pub fn guard_negative() {
    let _ = with_storage(|st| {
        let before = GUARD_REFUSALS.load(Ordering::Relaxed);
        let results = [
            guarded(
                &mut st.flash,
                Op::Erase {
                    from: REGION_START - SECTOR,
                    to: REGION_START,
                },
            ),
            guarded(
                &mut st.flash,
                Op::Program {
                    offset: REGION_END,
                    bytes: &[0x00],
                },
            ),
            guarded(
                &mut st.flash,
                Op::Program {
                    offset: REGION_START,
                    bytes: &[],
                },
            ),
        ];
        let counted = GUARD_REFUSALS.load(Ordering::Relaxed).wrapping_sub(before);
        let by_guard = results
            .iter()
            .filter(|r| **r == Err("refused by the range guard"))
            .count();
        if by_guard == 3 && counted == 3 {
            defmt::info!(
                "[PASS] guard-negative: 3 of 3 refused by the guard before any flash call, GUARD_REFUSALS {=u32}",
                counted
            );
        } else {
            defmt::error!(
                "[FAIL] guard-negative: {=usize} of 3 refused by the guard, GUARD_REFUSALS +{=u32}",
                by_guard,
                counted
            );
        }
    });
}

/// Feature `fake-armed`: the handshake's armed flag is set at boot (on core 0, standing in for
/// the flight core's arming code), so every save must end FAILED "armed".
#[cfg(feature = "fake-armed")]
pub fn fake_arm() {
    HANDSHAKE.set_permit(true);
    let armed = HANDSHAKE.try_arm();
    defmt::info!("[INFO] fake-armed: armed flag set at boot: {=bool}", armed);
}

/// Feature `save-loop`: 60 s after start, 100 saves one second apart, `BARO_IIR_T` alternating
/// between 1 and 3 (both valid, both not the default), then one summary line.
#[cfg(feature = "save-loop")]
#[embassy_executor::task]
pub async fn save_loop() {
    use embassy_time::{Duration, Timer};

    use crate::device::{BARO, SAVE_DISCARDS, STATE_CHANGES};
    use crate::imu1::CELLS;

    const SAVES: u32 = 100;
    Timer::after(Duration::from_secs(60)).await;
    // Once per power-up or flash: never again after a reset the watchdog caused (no reset loop).
    if !crate::watchdog::HANG_ARMED.load(Ordering::Relaxed) {
        defmt::warn!("[INFO] save-loop: not run after a watchdog reset");
        return;
    }
    let mut params = store::Store;
    let Some(index) = ParamName::new("BARO_IIR_T")
        .ok()
        .and_then(|n| params.find(&n))
    else {
        defmt::error!("[FAIL] save-loop: BARO_IIR_T not found");
        return;
    };
    let imu_before = CELLS.read();
    let baro_before = BARO.status();
    let changes_before = STATE_CHANGES.load(Ordering::Relaxed);
    let discards_before = SAVE_DISCARDS.load(Ordering::Relaxed);
    defmt::info!("[INFO] save-loop: {=u32} saves start", SAVES);
    let (mut verified, mut min_us, mut max_us, mut sum_us, mut last) =
        (0u32, u64::MAX, 0u64, 0u64, 0i32);
    for i in 0..SAVES {
        let value = if i % 2 == 0 { 1 } else { 3 };
        let _ = params.set(index, ParamValue::I32(value));
        match save() {
            Ok(stall_us) => {
                verified += 1;
                min_us = min_us.min(stall_us);
                max_us = max_us.max(stall_us);
                sum_us += stall_us;
                last = value;
            }
            Err(e) => defmt::warn!("[INFO] save-loop: save {=u32} FAILED: {=str}", i + 1, e),
        }
        Timer::after(Duration::from_secs(1)).await;
    }
    let imu_after = CELLS.read();
    let baro_after = BARO.status();
    let (im0, ig0, il0, ie0, it0) = imu_before.map_or((0, 0, 0, 0, 0), |s| {
        (
            s.missed_edges,
            s.max_gap_us,
            s.long_gaps,
            s.health.errors,
            s.health.timeouts,
        )
    });
    let (im1, ig1, il1, ie1, it1) = imu_after.map_or((0, 0, 0, 0, 0), |s| {
        (
            s.missed_edges,
            s.max_gap_us,
            s.long_gaps,
            s.health.errors,
            s.health.timeouts,
        )
    });
    defmt::info!(
        "[INFO] save-loop summary: {=u32} saves, {=u32} verified, stall min {=u64} max {=u64} mean {=u64} us; device transitions {=u32}; IMU1 missed edges +{=u32}, long gaps +{=u32}, largest gap {=u64} -> {=u64} us, errors +{=u32}, timeouts +{=u32}; BARO timeouts +{=u32}, missed edges +{=u32}, errors +{=u32}, largest gap {=u64} us; save windows discarded {=u32}; guard refusals {=u32}; last value saved BARO_IIR_T = {=i32}",
        SAVES,
        verified,
        if verified > 0 { min_us } else { 0 },
        max_us,
        sum_us / u64::from(verified.max(1)),
        STATE_CHANGES
            .load(Ordering::Relaxed)
            .wrapping_sub(changes_before),
        im1.wrapping_sub(im0),
        il1.wrapping_sub(il0),
        ig0,
        ig1,
        ie1.wrapping_sub(ie0),
        it1.wrapping_sub(it0),
        baro_after
            .health
            .timeouts
            .wrapping_sub(baro_before.health.timeouts),
        baro_after
            .missed_edges
            .wrapping_sub(baro_before.missed_edges),
        baro_after
            .health
            .errors
            .wrapping_sub(baro_before.health.errors),
        baro_after.max_gap_us,
        SAVE_DISCARDS
            .load(Ordering::Relaxed)
            .wrapping_sub(discards_before),
        GUARD_REFUSALS.load(Ordering::Relaxed),
        last
    );
}
