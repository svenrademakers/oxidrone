use rtic::app;

#[app(device = stm32h7xx_hal::pac, peripherals = true)]
mod app {
    use crate::bsp::Platform;
    use oxidrone_core::SerialLogger;
    use oxidrone_core::flight_controller::FlightController;
    use stm32h7xx_hal::stm32::TIM2;
    use stm32h7xx_hal::timer::Timer;

    #[shared]
    struct Shared {/* shared resources */}

    #[local]
    struct Local {
        tim2: Timer<TIM2>,
        flight_controller: FlightController<Platform>,
        logger: SerialLogger<Platform>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut platform = Platform::system_init(cx.device);
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
