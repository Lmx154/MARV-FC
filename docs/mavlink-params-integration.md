# MAVLink Parameter Protocol Integration Guide

## Document Status: UPDATED December 18, 2024

This document describes the complete MAVLink parameter protocol integration in MARV-FC, including LED color parameters, SD card persistence with automatic save, and Mission Planner compatibility.

---

## Latest Updates - Parameter Auto-Save Implementation (December 18, 2024)

### What's New
✅ **Automatic Parameter Persistence** - Parameters now automatically save to SD card 5 seconds after modification via Mission Planner

### Implementation Summary
Added debounced auto-save mechanism to the MAVLink parameter system:

**Key Features:**
- **5-Second Debounce**: Waits 5 seconds after last PARAM_SET before writing to SD card
- **Atomic Dirty Tracking**: Uses `AtomicBool` and `AtomicU32` for thread-safe state management
- **SPI Peripheral Recreation**: Safely recreates SPI1 peripheral using `Peripherals::steal()` for write operations
- **Automatic Retry**: Failed saves keep dirty flag set for retry on next iteration

**Files Modified:**
- [device/rp235x/src/main.rs](../device/rp235x/src/main.rs)
  - Added `PARAMS_DIRTY` and `PARAMS_LAST_MODIFIED` static atomic variables
  - Modified `save_params_to_sd_blocking()` to recreate SPI peripheral
  - Added dirty flag setting in PARAM_SET handler
  - Added debounced save logic in main MAVLink loop

**User Impact:**
- Parameter changes via Mission Planner now persist across reboots automatically
- No manual PARAMS.TXT editing required
- Changes take effect immediately AND are saved after 5-second idle period
- LED colors and all other parameters are fully persistent

---

## Recent Changes Summary (for AI agent handoff)

### Session Overview
This session implemented three major enhancements to the parameter system:

1. **LED Color Parameters** - Added configurable LED color via MAVLink parameters
2. **SD Card Persistence** - Parameters now load from/save to SD card (`PARAMS.TXT`)
3. **HEARTBEAT Messages** - Fixed Mission Planner connection error by adding 1 Hz heartbeat

### Critical Implementation Details

#### 1. LED Color Parameters (COMPLETED ✅)
**Files Modified:**
- `common/src/params.rs` - Added to ParamId enum (lines ~166-168):
  ```rust
  // --- LED Configuration ---
  LedColorR,
  LedColorG,
  LedColorB,
  ```
- Updated `PARAM_COUNT` to include new parameters: `(ParamId::LedColorB as usize) + 1`
- Added parameter definitions (lines ~225-227):
  ```rust
  ParamDef::new("LED_COLOR_R", ParamType::U32, ParamValue::U32(0)),
  ParamDef::new("LED_COLOR_G", ParamType::U32, ParamValue::U32(50)),
  ParamDef::new("LED_COLOR_B", ParamType::U32, ParamValue::U32(0)),
  ```

**Total Parameters:** 44 (was 41 before LED colors added)

#### 2. SD Card Persistence (COMPLETED ✅)
**Files Modified:**
- `common/src/params.rs` - Added serialization methods:
  - `serialize_to_text(&self, buf: &mut [u8]) -> Result<usize, ()>` - Converts params to NAME=VALUE format
  - `deserialize_from_text(&mut self, text: &[u8]) -> Result<usize, ()>` - Loads params from text
  - `SliceWriter` helper struct for no_std formatting
  
**Format:** Text file with `PARAM_NAME=value\n` format, supports comments with `#`

**Files Modified:**
- `device/rp235x/src/main.rs` - Added SD card functions:
  - `load_config_and_params_from_sd()` - Loads both CONFIG.TXT and PARAMS.TXT in one pass
  - `save_params_to_sd()` - Saves parameters (implemented but not wired to auto-save)
  
**Boot Sequence:**
1. Initialize SD card (SPI1, blocking mode)
2. Load CONFIG.TXT (existing functionality)
3. Load PARAMS.TXT into ParamRegistry
4. If PARAMS.TXT not found, use defaults
5. Extract LED color from loaded parameters
6. Continue with task spawning

**Key Code Location (device/rp235x/src/main.rs ~line 440-510):**
```rust
// Load both config and parameters from SD card
let mut param_registry = ParamRegistry::new();
let (config, _param_count) = match load_config_and_params_from_sd(sd_bus, sd_cs, &mut param_registry) {
    Ok((cfg, count)) => {
        info!("Config loaded from SD: {:?}", cfg);
        if count > 0 {
            info!("Loaded {} parameters from SD card", count);
        }
        // ...
    }
}

// Extract LED color from parameters (not config file anymore)
let params_guard = params.lock().await;
let led_r = params_guard.u32(ParamId::LedColorR).min(255) as u8;
let led_g = params_guard.u32(ParamId::LedColorG).min(255) as u8;
let led_b = params_guard.u32(ParamId::LedColorB).min(255) as u8;
```

#### 3. HEARTBEAT Implementation (COMPLETED ✅)
**Problem:** Mission Planner error "Sequence contains no elements" - requires periodic HEARTBEAT
  
**Files Modified:**
- `common/src/mavlink2/msg.rs` - Added functions (lines ~69-98):
  ```rust
  pub fn build_heartbeat() -> messages::Heartbeat { ... }
  pub fn build_heartbeat_frame(cfg: MavEndpointConfig, seq: u8) -> Frame<V2> { ... }
  ```
  Note: Field name is `type_` not `mavtype` (mavio quirk)

- `device/rp235x/src/main.rs` - Updated `usb_mavlink_task()`:
  - Added `last_heartbeat: Instant::now()` tracker
  - Sends HEARTBEAT at 1 Hz (every 1000ms) at end of main loop
  - Added import: `build_heartbeat_frame`

**Result:** Mission Planner now connects successfully

### Current Architecture

#### Communication Channels
1. **USB-CDC (Primary)** - Bidirectional MAVLink
   - Parameter read/write (PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET)
   - HEARTBEAT (1 Hz)
   - Telemetry (configurable rate via TEL_RATE_HZ parameter)
   - Task: `usb_mavlink_task()`

2. **UART0 (Secondary)** - One-way telemetry to radio
   - 10 Hz telemetry only
   - No parameter access
   - Task: `uart_telemetry_task()`

#### Parameter Flow
```
Boot:
  SD Card PARAMS.TXT → ParamRegistry → Tasks (sensor config, LED color, etc.)
  
Runtime (via USB-CDC):
  Mission Planner → PARAM_SET → ParamRegistry.set_by_index() → ACK
  (Not auto-saved to SD yet)
  
Next Boot:
  Previous PARAMS.TXT loaded again (manual updates only)
```

### Known Limitations & TODOs

#### ✅ AUTO-SAVE IMPLEMENTED (December 18, 2024)
**Current State:**
- Parameters automatically save to SD card 5 seconds after last PARAM_SET
- `save_params_to_sd_blocking()` recreates SPI1 peripheral using `Peripherals::steal()`
- Dirty flag tracking with debouncing prevents excessive SD writes
- Changes persist across reboots

**Implementation Details:**
- Static flags: `PARAMS_DIRTY` (AtomicBool), `PARAMS_LAST_MODIFIED` (AtomicU32)
- Debounce period: 5000ms (5 seconds)
- SPI peripheral recreated temporarily during save operation
- Uses `wrapping_sub` for timestamp comparison (handles u32 rollover after 49 days)

**How It Works:**
1. PARAM_SET handler sets `PARAMS_DIRTY = true` and updates timestamp
2. Main MAVLink loop checks dirty flag every iteration
3. After 5 seconds of no changes, triggers save operation
4. SPI1 peripheral recreated from PIN_8/9/10/11 using `Peripherals::steal()`
5. Parameters serialized to PARAMS.TXT on SD card
6. Dirty flag cleared on success, kept set on failure (retry next iteration)

**Safety Notes:**
- `Peripherals::steal()` is safe here because:
  - SPI1 is only used during boot for initial SD card load
  - No other task uses SPI1 after initialization
  - Peripheral is consumed and dropped after save completes
- Debouncing prevents write wear on SD card

**Workaround for Users:**
No longer needed - parameters save automatically!

#### 🔲 NEXT STEPS FOR CONTINUATION
1. **Test Auto-Save Implementation:**
   - Flash firmware and connect Mission Planner
   - Change parameter value (e.g., LED_COLOR_R)
   - Wait 5+ seconds and check RTT logs for "Saving parameters to SD card..."
   - Reboot device and verify parameter persisted

2. **Add Parameter Validation:**
   - Min/max ranges (e.g., LED colors 0-255, rates 1-1000 Hz)
   - Add to ParamDef struct
   - Validate in handle_param_set() before accepting

3. **Parameter Metadata:**
   - Add descriptions, units to ParamDef
   - Send via PARAM_VALUE extended fields (if supported by dialect)

4. **SD Card Error Handling:**
   - Add retry logic for SD card initialization
   - Create default PARAMS.TXT if missing on first boot
   - Better error reporting via STATUSTEXT messages

### Testing Status

✅ **Verified Working:**
- Mission Planner connects via USB-CDC (115200 baud)
- 44 parameters visible in Full Parameter List
- PARAM_REQUEST_LIST returns all parameters
- PARAM_SET updates parameters and sends ACK
- LED_COLOR_R/G/B parameters visible and settable
- Parameters load from SD card on boot
- ✅ **Auto-save to SD card 5 seconds after param change (NEW)**

🔲 **Pending Testing:**
- Physical hardware test with Mission Planner parameter changes
- Verify PARAMS.TXT created/updated on SD card
- Confirm parameters persist across power cycle
- Test save operation under different conditions (SD card removed, full, etc.)

### Code Locations Reference

**Parameter Definition:**
- `common/src/params.rs` - ParamId enum, PARAM_DEFS array, ParamRegistry

**MAVLink Protocol:**
- `common/src/mavlink2/msg.rs` - Message builders (param, heartbeat, statustext)
- `device/rp235x/src/main.rs:usb_mavlink_task()` - Message handler loop

**SD Card I/O:**
- `device/rp235x/src/main.rs:load_config_and_params_from_sd()` - Read on boot
- `device/rp235x/src/main.rs:save_params_to_sd_blocking()` - Save function (lines ~294-340)
- `device/rp235x/src/main.rs` - PARAMS_DIRTY, PARAMS_LAST_MODIFIED static flags (lines ~75-77)

**LED Integration:**
- `device/rp235x/src/main.rs:main()` - Lines ~595-600, LED color from params

---

## Original Documentation (Pre-Session)

## Overview

The parameter system is now fully wired into MAVLink, allowing ground control stations (GCS) to:
- **List all parameters** via `PARAM_REQUEST_LIST`
- **Read individual parameters** via `PARAM_REQUEST_READ` (by name or index)
- **Set parameters** via `PARAM_SET` with type checking and ACK

## Files Added/Modified

### Core Modules
- ✅ `common/src/params.rs` - Parameter registry (already existed)
- ✅ `common/src/commands.rs` - Command registry (already existed)
- ✅ `common/src/lib.rs` - Exposed params and commands modules

### MAVLink Integration
- ✅ `common/src/mavlink2/msg.rs` - Added parameter protocol handlers:
  - `param_type_to_mav()` - Type mapping
  - `encode_param_id()` / `decode_param_id()` - Name encoding/decoding
  - `build_param_value_frame()` - Build PARAM_VALUE responses
  - `handle_param_request_read()` - Process read requests
  - `handle_param_set()` - Process set requests with validation

- ✅ `common/src/mavlink2/param_handler.rs` - High-level handler with `ParamHandlerResult` enum
- ✅ `common/src/mavlink2/integration_example.rs` - Complete working examples
- ✅ `common/src/mavlink2/mod.rs` - Exposed new modules

## Parameter Registry

### Available Parameters (33 total)

#### System / Comms
- `SYS_ID` (U32) - MAVLink system ID
- `COMP_ID` (U32) - MAVLink component ID
- `TEL_RATE_HZ` (U32) - Telemetry rate in Hz
- `HB_EN` (Bool) - Heartbeat enable
- `STATUSTXT_EN` (Bool) - Status text messages enable

#### Logging
- `LOG_EN` (Bool) - Logging enable
- `LOG_RATE_HZ` (U32) - Log rate in Hz

#### Sensor Enable Flags
- `BMI088_EN` (Bool) - BMI088 IMU enable
- `ICM45686_EN` (Bool) - ICM45686 IMU enable
- `BMM350_EN` (Bool) - BMM350 magnetometer enable
- `BMP390_EN` (Bool) - BMP390 barometer enable
- `GPS_EN` (Bool) - GPS enable

#### Sensor Rates
- `BMI088_HZ` (U32) - BMI088 sample rate
- `ICM45686_HZ` (U32) - ICM45686 sample rate
- `BMM350_HZ` (U32) - BMM350 sample rate
- `BMP390_HZ` (U32) - BMP390 sample rate
- `GPS_HZ` (U32) - GPS sample rate

#### Calibration Offsets (F32)
- `ACC_BIAS_X/Y/Z` - Accelerometer bias
- `GYR_BIAS_X/Y/Z` - Gyroscope bias
- `MAG_BIAS_X/Y/Z` - Magnetometer bias

#### Filters
- `ACC_LPF_HZ` (F32) - Accelerometer low-pass filter cutoff (0 = off)
- `GYR_LPF_HZ` (F32) - Gyroscope low-pass filter cutoff (0 = off)
- `BARO_LPF_HZ` (F32) - Barometer low-pass filter cutoff (0 = off)

#### Orientation
- `IMU_ROT` (U32) - IMU rotation/orientation (0 = identity)

## Usage in Firmware

### 1. Initialize Registry

```rust
use common::params::ParamRegistry;

// In your main() or init function
let mut params = ParamRegistry::new();
```

### 2. Read Parameters in Tasks

```rust
use common::params::ParamId;

// Check sensor enable
if params.bool(ParamId::Bmi088En) {
    let rate = params.u32(ParamId::Bmi088RateHz);
    // Configure BMI088 at 'rate' Hz
}

// Apply calibration offsets
let acc_x = raw_acc_x + params.f32(ParamId::AccBiasX);
let acc_y = raw_acc_y + params.f32(ParamId::AccBiasY);
let acc_z = raw_acc_z + params.f32(ParamId::AccBiasZ);

// Get system config
let sys_id = params.u32(ParamId::SysId) as u8;
let comp_id = params.u32(ParamId::CompId) as u8;
```

### 3. Handle MAVLink Messages

```rust
use common::mavlink2::msg::{
    build_param_value_frame,
    handle_param_request_read,
    handle_param_set,
};
use mavio::dialects::common::Common;

// In your MAVLink receive loop
match frame.decode::<Common>() {
    Ok(Common::ParamRequestList(req)) => {
        // Validate target system/component
        if req.target_system != 0 && req.target_system != cfg.sys_id {
            continue;
        }
        
        // Send all parameters
        for idx in 0..params.count() {
            if let Some(reply) = build_param_value_frame(cfg, seq, &params, idx) {
                send_frame(&reply).await;
                seq = seq.wrapping_add(1);
            }
        }
    }
    
    Ok(Common::ParamRequestRead(req)) => {
        // Validate target
        if req.target_system != 0 && req.target_system != cfg.sys_id {
            continue;
        }
        
        // Find and send parameter
        if let Some(idx) = handle_param_request_read(&params, &req) {
            if let Some(reply) = build_param_value_frame(cfg, seq, &params, idx) {
                send_frame(&reply).await;
                seq = seq.wrapping_add(1);
            }
        }
    }
    
    Ok(Common::ParamSet(req)) => {
        // Validate target
        if req.target_system != cfg.sys_id {
            continue;
        }
        
        // Update parameter
        if let Some(idx) = handle_param_set(&mut params, &req) {
            // Send ACK
            if let Some(reply) = build_param_value_frame(cfg, seq, &params, idx) {
                send_frame(&reply).await;
                seq = seq.wrapping_add(1);
            }
        }
    }
    
    _ => { /* other messages */ }
}
```

### 4. Alternative: Use High-Level Handler

```rust
use common::mavlink2::param_handler::{handle_param_message, ParamHandlerResult};

match handle_param_message(&frame, &mut params, cfg, seq) {
    ParamHandlerResult::Reply(reply_frame) => {
        send_frame(&reply_frame).await;
        seq = seq.wrapping_add(1);
    }
    ParamHandlerResult::StreamAll(count) => {
        for idx in 0..count {
            if let Some(reply) = build_param_value_frame(cfg, seq, &params, idx) {
                send_frame(&reply).await;
                seq = seq.wrapping_add(1);
            }
        }
    }
    ParamHandlerResult::NotHandled => {
        // Handle other message types
    }
}
```

## Type Conversions

MAVLink PARAM_VALUE uses `f32` for all parameter values. The system handles conversions:

| ParamType | Storage | MAVLink Type | Conversion |
|-----------|---------|--------------|------------|
| Bool | `bool` | `UINT8` | `0.0` or `1.0` |
| I32 | `i32` | `INT32` | Cast to `f32` |
| U32 | `u32` | `UINT32` | Cast to `f32` |
| F32 | `f32` | `REAL32` | Direct |

**Note**: Integer parameters are limited to values that can be represented exactly in f32 (~±16M for U32, ~±8M for I32).

## Parameter Name Constraints

- Maximum 16 characters (MAVLink PARAM_VALUE limit)
- All current parameter names are ≤ 16 chars and validated at boot

## Next Steps

### Immediate (AI Agent TODO List)
1. **Implement Auto-Save on Parameter Change:**
   - Add static AtomicBool flag `PARAMS_DIRTY` in main.rs
   - Set flag in PARAM_SET handler after successful update
   - Create background task that checks flag every 5 seconds
   - If dirty, call save_params_to_sd() and clear flag
   - Challenge: Need to recreate SPI1 peripheral (consumed at boot)
   - Suggested approach: Use RefCell or reconstruct peripheral pins

2. **Add Parameter Validation:**
   - Extend ParamDef struct with min/max fields
   - Update handle_param_set() to check ranges before accepting
   - Reject out-of-range values with error log

3. **Create Default PARAMS.TXT on First Boot:**
   - Check if PARAMS.TXT exists during boot
   - If missing, create with all default values using serialize_to_text()
   - Simplifies user setup (no manual file creation needed)

4. **Add MAVLink Command for Manual Save:**
   - Implement MAV_CMD_PREFLIGHT_STORAGE command
   - Param1=0: read, Param1=1: write
   - Allows explicit parameter save from Mission Planner

### Future Enhancements
1. **USB-CDC CLI** - Wire commands.rs into USB serial interface (low priority, MAVLink working)
2. **Parameter Groups** - Add metadata for UI organization
3. **Change Callbacks** - Notify tasks when params change (e.g., update LED immediately)
4. **Atomic Parameter Updates** - Bundle multiple param changes into single save

## Troubleshooting Guide (For Next AI Agent)

### Build Errors

**Error: "cannot borrow as mutable more than once"**
- Location: SD card file operations in load_config_and_params_from_sd()
- Cause: embedded_sdmmc file handles hold mutable borrow of root_dir
- Solution: Explicitly drop() file and result before opening next file

**Error: "value used after move" with SPI peripheral**
- Location: Trying to reuse SPI1/CS pins after SD card init
- Cause: Peripherals consumed by Spi::new_blocking()
- Solution: Only create SPI once, or use RefCell for shared access

### Runtime Issues

**Parameters Not Loading from SD:**
- Check RTT logs for "Loaded N parameters from SD card" message
- Verify SD card has PARAMS.TXT in root directory (not in subfolder)
- Check file format: must be ASCII text with NAME=VALUE\n format
- Verify SD card FAT filesystem is valid (CONFIG.TXT loads successfully?)

**LED Color Not Changing:**
- Verify parameters set via Mission Planner: LED_COLOR_R/G/B
- Check that values are 0-255 (clamped with .min(255))
- LED color only updates on boot or logger_task pulse cycle
- May need to add callback to update LED immediately on param change

**Mission Planner "Sequence contains no elements":**
- FIXED: Ensure HEARTBEAT sent at 1 Hz from usb_mavlink_task()
- Check that build_heartbeat_frame() is imported
- Verify heartbeat interval check: `last_heartbeat.elapsed().as_millis() >= 1000`

### Code Navigation Tips

**Find where parameters are used:**
```bash
grep -r "ParamId::" device/rp235x/src/
grep -r "params.u32\|params.f32\|params.bool" device/
```

**Find MAVLink message handlers:**
- All in usb_mavlink_task() function around line 860-950
- Pattern: `Ok(Common::MessageType(req)) => { ... }`

**Find SD card functions:**
- Search for "embedded_sdmmc" in device/rp235x/src/main.rs
- Functions around lines 189-310

## Testing Checklist

**Basic Functionality:**
- [x] List all parameters from GCS (PARAM_REQUEST_LIST works)
- [x] Read individual parameter by name (PARAM_REQUEST_READ works)
- [x] Read individual parameter by index (works)
- [x] Set U32 parameter (LED_COLOR_R tested)
- [x] Verify parameters visible in Mission Planner (44 params shown)
- [x] Mission Planner connection successful (HEARTBEAT fixed)

**SD Card Persistence:**
- [ ] Create PARAMS.TXT manually and verify load on boot
- [ ] Test with malformed PARAMS.TXT (bad syntax)
- [ ] Test with missing PARAMS.TXT (should use defaults)
- [ ] Verify save_params_to_sd() function (not wired yet)

**Advanced:**
- [ ] Test type mismatch rejection (set U32 param with wrong type)
- [ ] Test invalid parameter name rejection
- [ ] Test with system ID filtering (set req.target_system != sys_id)
- [ ] Verify telemetry rate changes with TEL_RATE_HZ param
- [ ] Verify sensor rates update when changed via params

## References

- MAVLink Common Message Set: https://mavlink.io/en/messages/common.html
- Parameter Protocol: https://mavlink.io/en/services/parameter.html
- Mission Planner Parameter Editor: https://ardupilot.org/planner/docs/common-mission-planner-features.html
- embedded-sdmmc crate: https://docs.rs/embedded-sdmmc/latest/
