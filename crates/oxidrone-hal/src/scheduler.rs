#[cfg(all(target_arch = "arm", target_vendor = "unknown"))]
pub mod cortex_m;
#[cfg(all(target_arch = "arm", target_vendor = "unknown"))]
pub use cortex_m::*;

#[cfg(not(all(target_arch = "arm", target_vendor = "unknown")))]
pub fn run(
    _high_prio_scheduler: impl FnOnce(embassy_executor::SendSpawner),
    _app_tasks: impl FnOnce(embassy_executor::Spawner),
) -> ! {
    todo!()
}
