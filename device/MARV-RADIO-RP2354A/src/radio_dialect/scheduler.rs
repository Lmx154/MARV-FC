use crate::channels::{
    HILINK_BRIDGE_FRAME_BYTES, HOST_TO_LORA_P0_CHANNEL, HOST_TO_LORA_P1_CHANNEL,
    HOST_TO_LORA_P2_CHANNEL, HOST_TO_LORA_P3_CHANNEL, HOST_TO_LORA_P4_CHANNEL, HilinkBridgeFrame,
};
use crate::config::FirmwareRole;
use crate::radio_dialect::airtime::{AirtimeBudget, DownlinkClass};
use crate::radio_dialect::{normal, policy, rf, state_cache::RadioStateCache};

pub fn classify_host_frame(frame: &HilinkBridgeFrame) -> policy::RadioPriority {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let Ok(packet) = normal::decode_frame(frame.as_slice(), &mut raw) else {
        return policy::RadioPriority::P3Snapshot;
    };

    policy::classify_normal_msg(&packet)
}

pub fn receive_next_host_frame() -> Option<HilinkBridgeFrame> {
    if let Ok(frame) = HOST_TO_LORA_P0_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P1_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P2_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P3_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else if let Ok(frame) = HOST_TO_LORA_P4_CHANNEL.receiver().try_receive() {
        Some(frame)
    } else {
        None
    }
}

pub fn select_scheduled_rf_frame(
    role: FirmwareRole,
    cache: &mut RadioStateCache,
    budget: &AirtimeBudget,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    // Critical, event-driven frames preempt the airtime budget — they are rare and the budget's
    // duty reserve leaves headroom for them.
    if let Some(ack) = cache.pop_pending_lora_command_ack() {
        return rf::encode_rf_frame(&ack).ok();
    }

    if let Some(event) = cache.pop_pending_lora_event() {
        return rf::encode_rf_frame(&event).ok();
    }

    if let Some(faults) = cache.take_dirty_lora_faults() {
        return rf::encode_rf_frame(&faults).ok();
    }

    match role {
        FirmwareRole::GroundStation => select_background_rf_frame(cache, budget, now_ms),
        FirmwareRole::Radio => select_vehicle_scheduled_rf_frame(cache, budget, now_ms),
    }
}

fn select_vehicle_scheduled_rf_frame(
    cache: &mut RadioStateCache,
    budget: &AirtimeBudget,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    // Order resolves ties when several classes are due in the same loop iteration (only one
    // frame goes out per iteration). Per-class periods already cap each class's share, so the
    // high-rate sensors lead to avoid being starved by the slower gps/link-status frames.
    if let Some(snapshot) =
        cache.due_vehicle_flight_snapshot(now_ms, budget.period_ms(DownlinkClass::Flight))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_flight_snapshot_sent(now_ms);
        return Some(frame);
    }

    if let Some(snapshot) =
        cache.due_vehicle_imu1_snapshot(now_ms, budget.period_ms(DownlinkClass::Imu1))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_imu1_snapshot_sent(now_ms);
        return Some(frame);
    }

    if let Some(snapshot) =
        cache.due_vehicle_imu2_snapshot(now_ms, budget.period_ms(DownlinkClass::Imu2))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_imu2_snapshot_sent(now_ms);
        return Some(frame);
    }

    if let Some(snapshot) =
        cache.due_vehicle_mag_snapshot(now_ms, budget.period_ms(DownlinkClass::Mag))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_mag_snapshot_sent(now_ms);
        return Some(frame);
    }

    if let Some(snapshot) =
        cache.due_vehicle_baro_snapshot(now_ms, budget.period_ms(DownlinkClass::Baro))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_baro_snapshot_sent(now_ms);
        return Some(frame);
    }

    if let Some(snapshot) =
        cache.due_vehicle_gps_snapshot(now_ms, budget.period_ms(DownlinkClass::Gps))
    {
        let frame = rf::encode_rf_frame(&snapshot).ok()?;
        cache.note_vehicle_gps_snapshot_sent(now_ms);
        return Some(frame);
    }

    select_background_rf_frame(cache, budget, now_ms)
}

fn select_background_rf_frame(
    cache: &mut RadioStateCache,
    budget: &AirtimeBudget,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    if let Some(status) =
        cache.due_lora_link_status(now_ms, budget.period_ms(DownlinkClass::LinkStatus))
    {
        let frame = rf::encode_rf_frame(&status).ok()?;
        cache.note_lora_link_status_sent(now_ms);
        return Some(frame);
    }

    None
}
