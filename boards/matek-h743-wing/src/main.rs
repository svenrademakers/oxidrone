#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use embassy_stm32::interrupt;
use oxidrone_core::{app_spawner, high_prio_spawner};
use panic_probe as _;

#[entry]
fn main() -> ! {
    info!("entring matek-h743-wing startup");
    let irq: interrupt::Interrupt = interrupt::Interrupt::TIM1_UP;
    oxidrone_hal::scheduler::run(high_prio_spawner, app_spawner, irq)
}
