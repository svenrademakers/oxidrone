#![no_std]
#![no_main]

use panic_probe as _;

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
    unsafe fn flush() {
        todo!()
    }
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}

#[cfg(feature = "board-matekh743-mini")]
mod bsp_matek743;
#[cfg(feature = "board-matekh743-mini")]
use bsp_matek743 as bsp;

#[cfg(feature = "rt-stm32h7xx")]
include!("rt_stm32h7xx.rs");
