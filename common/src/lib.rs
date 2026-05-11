#![cfg_attr(not(test), no_std)]

// Minimal host-side defmt logger to satisfy unit test linking.
#[cfg(test)]
mod defmt_test_logger {
    use defmt::{Logger, global_logger};

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

pub mod comms;
pub mod control;
pub mod drivers;
pub mod interfaces;
pub mod localization;
pub mod messages;
pub mod policies;
pub mod prelude;
pub mod protocol;
pub mod services;
pub mod tasks;
pub mod utilities;

#[cfg(test)]
pub(crate) mod test_helpers;

pub mod utils;
