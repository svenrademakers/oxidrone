#![no_std]
#![no_main]
#![feature(maybe_uninit_array_assume_init)]

mod platform;

use panic_probe as _;
use rtic::app;

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

#[app(device = stm32h7xx_hal::pac, peripherals = true)]
mod app {
    use crate::platform::PlatformMatek;
    use oxidrone_core::SerialLogger;
    use oxidrone_core::flight_controller::FlightController;
    use stm32h7xx_hal::stm32::TIM2;
    use stm32h7xx_hal::timer::Timer;

    #[shared]
    struct Shared {/* shared resources */}

    #[local]
    struct Local {
        tim2: Timer<TIM2>,
        flight_controller: FlightController<PlatformMatek>,
        logger: SerialLogger<PlatformMatek>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut platform = PlatformMatek::system_init(cx.device);
        let tim2 = platform.timer2().unwrap();
        let logger = SerialLogger::new(&mut platform);
        let flight_controller = FlightController::new(platform);
        (
            Shared {},
            Local {
                tim2,
                flight_controller,
                logger,
            },
        )
    }

    #[task(binds = TIM2, local = [flight_controller, tim2 ])]
    fn timer2_tick(ctx: timer2_tick::Context) {
        ctx.local.tim2.clear_irq();
        ctx.local.flight_controller.update();
    }

    #[task(binds = OTG_FS, local = [logger])]
    fn usb_event(ctx: usb_event::Context) {
        ctx.local.logger.poll();
    }
}
