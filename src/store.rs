//! The firmware's one parameter store (`fw_core::params::Params` over the devices' tables), in
//! RAM; `settings` loads it from flash at boot and writes it there only on an explicit save (a
//! write takes effect here and is lost at reset unless saved). It sits in a critical-section mutex so the link task (the
//! parameter protocol, through [`Store`]), the action code and the device tasks can all reach it;
//! every access is short and none is held across an await.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::params::Params;
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::MarvDevice;
use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};

use crate::device::raise_apply;

/// The number of parameters.
pub const COUNT: usize = fw_settings::COUNT;

static PARAMS: Mutex<CriticalSectionRawMutex, RefCell<Option<Params<COUNT>>>> =
    Mutex::new(RefCell::new(None));

/// Builds the store from the table and installs the validators. Called once, before any task.
pub fn init() {
    let Ok(mut params) = Params::<COUNT>::new(&fw_settings::TABLE) else {
        defmt::panic!("[FAIL] parameter table rejected");
    };
    if !params.set_validator(
        MarvDevice::MARV_DEVICE_BARO as u8,
        fw_settings::baro::validate,
    ) || !params.set_validator(
        MarvDevice::MARV_DEVICE_MAG as u8,
        fw_settings::mag::validate,
    ) {
        defmt::panic!("[FAIL] validator not installed");
    }
    PARAMS.lock(|cell| *cell.borrow_mut() = Some(params));
    defmt::info!(
        "[INFO] params: {=usize} parameters, in RAM; flash only on save",
        COUNT
    );
}

/// Runs `f` on the store (a short critical section; `f` must not call `with` again).
pub fn with<R>(f: impl FnOnce(&mut Params<COUNT>) -> R) -> Option<R> {
    PARAMS.lock(|cell| cell.borrow_mut().as_mut().map(f))
}

/// The barometer's whole `Config` from the stored values.
pub fn baro_config() -> Result<bmp581::Config, &'static str> {
    with(|params| {
        fw_settings::baro::settings_to_config(|name| match params.get_named(name) {
            Some(ParamValue::I32(v)) => Some(v),
            _ => None,
        })
    })
    .unwrap_or(Err("store not ready"))
}

/// The magnetometer's whole `Config` from the stored values.
pub fn mag_config() -> Result<bmm350::Config, &'static str> {
    with(|params| {
        let rate = match params.get_named(fw_settings::mag::ODR_HZ) {
            Some(ParamValue::F32(v)) => Some(v),
            _ => None,
        };
        let avg = match params.get_named(fw_settings::mag::AVG) {
            Some(ParamValue::I32(v)) => Some(v),
            _ => None,
        };
        fw_settings::mag::settings_to_config(rate, avg)
    })
    .unwrap_or(Err("store not ready"))
}

/// The magnetometer's calibration offsets in microtesla (X, Y, Z), read for every sample.
pub fn mag_offsets() -> [f32; 3] {
    with(|params| {
        [
            fw_settings::mag::OFF_X,
            fw_settings::mag::OFF_Y,
            fw_settings::mag::OFF_Z,
        ]
        .map(|name| match params.get_named(name) {
            Some(ParamValue::F32(v)) => v,
            _ => 0.0,
        })
    })
    .unwrap_or([0.0; 3])
}

/// `ParamStore` for the node: the one store, behind the lock.
pub struct Store;

impl ParamStore for Store {
    fn count(&self) -> u16 {
        with(|p| p.count()).unwrap_or(0)
    }

    fn name(&self, i: u16) -> Option<ParamName> {
        with(|p| p.name(i)).flatten()
    }

    fn get(&self, i: u16) -> Option<ParamValue> {
        with(|p| p.get(i)).flatten()
    }

    fn find(&self, n: &ParamName) -> Option<u16> {
        with(|p| p.find(n)).flatten()
    }

    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        let (result, notice, configure, live, name) = with(|p| {
            (
                p.set(i, v),
                p.take_notice(),
                p.take_configure_mask(),
                p.take_live_mask(),
                p.name(i),
            )
        })
        .ok_or(ParamError::NotFound)?;
        let name = name.as_ref().map_or("?", ParamName::as_str);
        match result {
            Ok(echo) => {
                defmt::info!(
                    "[INFO] param {=str} = {}: accepted{=str}",
                    name,
                    echo,
                    if configure == 0 && live == 0 {
                        " (no change, nothing to apply)"
                    } else {
                        ""
                    }
                );
                raise_apply(configure);
            }
            Err(error) => {
                let reason = notice.map_or("refused", |n| n.kind.text());
                defmt::info!(
                    "[INFO] param {=str} = {}: refused, {} ({=str})",
                    name,
                    v,
                    error,
                    reason
                );
            }
        }
        if let Some(notice) = notice {
            let severity = if result.is_err() {
                MavSeverity::MAV_SEVERITY_WARNING
            } else {
                MavSeverity::MAV_SEVERITY_INFO
            };
            crate::text_fmt(
                severity,
                format_args!("{}: {}", notice.name.as_str(), notice.kind.text()),
            );
        }
        result
    }
}
