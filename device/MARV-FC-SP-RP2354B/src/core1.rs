#![allow(dead_code)]

pub const ROLE_SUMMARY: &str = "storage, companion link, telemetry, diagnostics, indicators";

pub struct Core1Plan {
    pub owns_storage_domain: bool,
    pub owns_companion_link: bool,
    pub owns_status_indicators: bool,
}

pub const PLAN: Core1Plan = Core1Plan {
    owns_storage_domain: true,
    owns_companion_link: true,
    owns_status_indicators: true,
};
