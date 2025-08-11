#![no_std]
#![no_main]

use panic_probe as _;

#[cfg(feature = "usb_logger")]
mod logger;
#[cfg(not(feature = "usb_logger"))]
mod defmt_noop {
    // A unit struct marked as the global defmt logger
    #[defmt::global_logger]
    struct Logger;

    // Minimal implementation: drop all bytes
    unsafe impl defmt::Logger for Logger {
        fn acquire() {} // no locking needed
        unsafe fn write(_: &[u8]) {} // discard
        unsafe fn flush() {} // nothing to flush
        unsafe fn release() {} // no lock to release
    }
}

#[cfg(feature = "board-matekh743-mini")]
mod bsp_matek743;
#[cfg(feature = "board-matekh743-mini")]
use bsp_matek743 as bsp;

#[cfg(feature = "rt-stm32h7xx")]
include!("rt_stm32h7xx.rs");
