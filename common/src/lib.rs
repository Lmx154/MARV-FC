#![cfg_attr(not(test), no_std)]

// Minimal host-side defmt logger to satisfy unit test linking.
#[cfg(test)]
mod defmt_test_logger {
    use defmt::{global_logger, Logger};

    #[global_logger]
    struct HostLogger;

    unsafe impl Logger for HostLogger {
        fn acquire() {}
        unsafe fn flush() {}
        unsafe fn release() {}
        unsafe fn write(_bytes: &[u8]) {}
    }

    // Provide the symbols defmt expects so host tests can link.
    #[unsafe(no_mangle)]
    fn _defmt_timestamp(_fmt: defmt::Formatter<'_>) {}

    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_TRACE_START: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_TRACE_END: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_DEBUG_START: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_DEBUG_END: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_INFO_START: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_INFO_END: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_WARN_START: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_WARN_END: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_ERROR_START: u8 = 0;
    #[used]
    #[unsafe(no_mangle)]
    static __DEFMT_MARKER_ERROR_END: u8 = 0;
}

pub mod drivers;
pub mod filters;
pub mod estimators;
pub mod tasks;
pub mod types;
pub mod utils;
pub mod config;
pub mod lora;
pub mod mavlink2;
pub mod coms;
pub mod log_config;
