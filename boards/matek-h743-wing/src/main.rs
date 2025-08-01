#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::interrupt;
use oxidrone_core::{flight_control::flight_controller, oxidrone_app};
use panic_probe as _;

static HIGH_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[entry]
fn main() -> ! {
    let irq: interrupt::Interrupt = interrupt::Interrupt::TIM1_UP;
    let spawner = HIGH_PRIO_EXECUTOR.start(irq);
    spawner.must_spawn(flight_controller());

    let mut executor = Executor::new();
    // SAFETY: `executor` lives until `run()` (which never returns), so it's safe to promote
    // its mutable reference to `'static`. This is required because `run()` expects a `'static` reference.
    let static_executor =
        unsafe { core::mem::transmute::<&'_ mut Executor, &'static mut Executor>(&mut executor) };
    static_executor.run(oxidrone_app)
}
