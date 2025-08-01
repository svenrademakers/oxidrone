use cortex_m::interrupt::InterruptNumber;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};

static HIGH_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

pub fn run(
    high_prio_scheduler: impl FnOnce(SendSpawner),
    app_tasks: impl FnOnce(Spawner),
    irq: impl InterruptNumber,
) -> ! {
    let spawner = HIGH_PRIO_EXECUTOR.start(irq);
    high_prio_scheduler(spawner);

    let mut executor = Executor::new();
    // SAFETY: `executor` lives until `run()` (which never returns), so it's safe to promote
    // its mutable reference to `'static`. This is required because `run()` expects a `'static` reference.
    let static_executor =
        unsafe { core::mem::transmute::<&'_ mut Executor, &'static mut Executor>(&mut executor) };
    static_executor.run(app_tasks)
}
